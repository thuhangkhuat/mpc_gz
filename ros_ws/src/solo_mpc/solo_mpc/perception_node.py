"""Detect the target in UAV camera images and project it onto its ground plane.

With a ground_robot configured the target is its AprilTag (tag centre on the
tag plane); otherwise it is the magenta scenario marker.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from . import apriltag, ground_robot
from .core.sensors import PoseHistory, pixel_to_ground
from .ros_utils import (
    pose_arrays,
    pose_message,
    qos_profile_sensor_data,
    run_node,
    seconds,
)


class MarkerDetector(Node):
    def __init__(self, args):
        vehicle = getattr(args, "vehicle", None)
        super().__init__(f"mpc_marker_detector_{vehicle}" if vehicle else "mpc_marker_detector")
        names = [vehicle] if vehicle else args.names
        self.args = args
        self.bridge = CvBridge()
        self.history = {
            n: PoseHistory(args.settings_data["frame_history_tolerance"])
            for n in names
        }
        self.intrinsics = {}
        self.robot = args.settings_data.get("ground_robot")
        if self.robot is not None:
            apriltag.dictionary(self.robot["tag"]["family"])  # fail at startup without aruco
        self.publishers_keep = {}
        self.debug = {}
        # One line per processed frame for FOV/recall analysis (scripts/analyze_run.py).
        run_dir = getattr(args, "run_dir", "")
        self.frames = {n: open(Path(run_dir, f"detections_{n}.jsonl"), "a", buffering=1)
                       for n in names} if run_dir else {}
        self.subscriptions_keep = []
        for n in names:
            self.publishers_keep[n] = self.create_publisher(
                PoseStamped, f"/{n}/target/observation", 10
            )
            # Annotated camera view, drawn only while something subscribes.
            self.debug[n] = self.create_publisher(Image, f"/{n}/camera/detections", 2)
            for typ, suffix, cb in [
                (PoseStamped, "mpc/pose", self.pose),
                (Image, "camera/image_raw", self.image),
                (CameraInfo, "camera/camera_info", self.info),
            ]:
                self.subscriptions_keep.append(
                    self.create_subscription(
                        typ,
                        f"/{n}/{suffix}",
                        lambda msg, name=n, fn=cb: fn(name, msg),
                        qos_profile_sensor_data,
                    )
                )

    def pose(self, n, msg):
        self.history[n].add(seconds(msg.header.stamp), *pose_arrays(msg))

    def info(self, n, msg):
        self.intrinsics[n] = np.asarray(msg.k).reshape(3, 3)

    def detect(self, image):
        """(pixel centre, plane height, outline corners) of the target, or None."""
        if self.robot is not None:
            found = apriltag.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                    self.robot["tag"]["family"], self.robot["tag"]["id"])
            if found is None:
                return None
            return found[0], ground_robot.tag_height(self.robot), found[1]
        mask = cv2.inRange(
            cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
            np.array([135, 100, 70]),
            np.array([175, 255, 255]),
        )
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        if count < 2:
            return None
        idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[idx, cv2.CC_STAT_AREA] < self.args.settings_data["marker_min_pixels"]:
            return None
        x, y, w, h = stats[idx, :4]
        outline = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], float)
        return centers[idx], self.args.settings_data["target_height"] + 0.02, outline

    def annotate(self, n, msg, image, found, observed):
        if self.debug[n].get_subscription_count() == 0:
            return
        view = image.copy()
        label = "tag" if self.robot is not None else "marker"
        if found is None:
            cv2.putText(view, f"no {label}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            pixel, _, outline = found
            cv2.polylines(view, [np.round(outline).astype(np.int32)], True, (0, 255, 0), 2)
            cv2.circle(view, tuple(int(v) for v in np.round(pixel)), 4, (0, 255, 0), -1)
            if self.robot is not None:
                # Corner 0 marks the tag's own top-left, so its heading is visible.
                cv2.circle(view, tuple(int(v) for v in np.round(outline[0])), 5, (255, 0, 255), -1)
                label = f"tag {self.robot['tag']['id']}"
            text = label if observed is None else f"{label} ({observed[0]:.2f}, {observed[1]:.2f}) m"
            cv2.putText(view, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(view, "bgr8")
        out.header = msg.header
        self.debug[n].publish(out)

    def image(self, n, msg):
        if n not in self.intrinsics:
            return
        image = found = observed = None
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            found = self.detect(image)
            if found is None:
                return
            pixel, plane, _ = found
            p, q = self.history[n].at(seconds(msg.header.stamp))
            # Sensor mount is 0.04 m below body origin, pointing down.
            from scipy.spatial.transform import Rotation

            camera = p + Rotation.from_quat(q).apply([0.0, 0.0, -0.04])
            observed = pixel_to_ground(pixel, self.intrinsics[n], camera, q, plane)
            self.publishers_keep[n].publish(pose_message(observed, msg.header.stamp))
        except (ValueError, cv2.error) as exc:
            self.get_logger().debug(str(exc))
        finally:
            if n in self.frames and image is not None:
                self.frames[n].write(json.dumps({
                    "stamp": seconds(msg.header.stamp), "detected": found is not None,
                    "pixel": None if found is None else [float(v) for v in found[0]],
                    "observed": None if observed is None else [float(v) for v in observed[:2]],
                }) + "\n")
            if image is not None:
                try:
                    self.annotate(n, msg, image, found, observed)
                except (ValueError, cv2.error) as exc:
                    self.get_logger().debug(f"detection view: {exc}")

def main():
    run_node(MarkerDetector, "Camera-only ground marker observations")
