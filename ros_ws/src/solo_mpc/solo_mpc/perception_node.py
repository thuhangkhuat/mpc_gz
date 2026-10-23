"""Detect a magenta ground marker from images, using calibrated camera rays."""

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

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
        self.publishers_keep = {}
        self.subscriptions_keep = []
        for n in names:
            self.publishers_keep[n] = self.create_publisher(
                PoseStamped, f"/{n}/target/observation", 10
            )
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

    def image(self, n, msg):
        if n not in self.intrinsics:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            mask = cv2.inRange(
                cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                np.array([135, 100, 70]),
                np.array([175, 255, 255]),
            )
            count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
            if count < 2:
                return
            idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            if (
                stats[idx, cv2.CC_STAT_AREA]
                < self.args.settings_data["marker_min_pixels"]
            ):
                return
            p, q = self.history[n].at(seconds(msg.header.stamp))
            # Sensor mount is 0.04 m below body origin, pointing down.
            from scipy.spatial.transform import Rotation

            camera = p + Rotation.from_quat(q).apply([0.0, 0.0, -0.04])
            observed = pixel_to_ground(
                centers[idx],
                self.intrinsics[n],
                camera,
                q,
                self.args.settings_data["target_height"] + 0.02,
            )
            self.publishers_keep[n].publish(pose_message(observed, msg.header.stamp))
        except (ValueError, cv2.error) as exc:
            self.get_logger().debug(str(exc))


def main():
    run_node(MarkerDetector, "Camera-only ground marker observations")
