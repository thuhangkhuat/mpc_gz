"""Calibrate disarmed local EKF frames once, then publish measured world state.

Ground truth is used only for initial simulation alignment and evaluation. Pose
comes from MAVROS after calibration. FCU wall-time stamps are explicitly mapped
to ROS simulation receipt time; this is an approximation recorded in docs.
"""

import json
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .ros_utils import pose_arrays, pose_message, qos_profile_sensor_data, run_node


class Frames(Node):
    def __init__(self, args):
        super().__init__("mpc_frames")
        self.args = args
        self.local = {}
        self.truth = {}
        self.fcu = {}
        self.transforms = {}
        self.samples = {}
        self.publishers_keep = {}
        self.subscriptions_keep = []
        self.static = StaticTransformBroadcaster(self)
        self.dynamic = TransformBroadcaster(self)
        mounts = []
        for n in args.names:
            for child, offset, rotation in [
                ("laser", [0.0, 0.0, 0.08], Rotation.identity()),
                (
                    "camera_optical_frame",
                    [0.0, 0.0, -0.04],
                    Rotation.from_matrix([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]),
                ),
            ]:
                tf = TransformStamped()
                tf.header.frame_id = f"{n}/base_link"
                tf.child_frame_id = f"{n}/{child}"
                (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                ) = offset
                q = rotation.as_quat()
                (
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ) = map(float, q)
                mounts.append(tf)
        self.static.sendTransform(mounts)
        for n in args.names:
            self.publishers_keep[n] = (
                self.create_publisher(PoseStamped, f"/{n}/mpc/pose", 10),
                self.create_publisher(TwistStamped, f"/{n}/mpc/velocity", 10),
            )
            for typ, suffix, cb in [
                (PoseStamped, "mavros/local_position/pose", self.local_pose),
                (PoseStamped, "ground_truth/pose", self.ground_pose),
                (TwistStamped, "mavros/local_position/velocity_local", self.velocity),
                (State, "mavros/state", self.state),
            ]:
                self.subscriptions_keep.append(
                    self.create_subscription(
                        typ,
                        f"/{n}/{suffix}",
                        lambda msg, name=n, fn=cb: fn(name, msg),
                        qos_profile_sensor_data,
                    )
                )

    def state(self, n, msg):
        self.fcu[n] = msg

    def ground_pose(self, n, msg):
        self.truth[n] = (msg, time.monotonic())

    def local_pose(self, n, msg):
        self.local[n] = (msg, time.monotonic())
        if n not in self.transforms:
            fcu = self.fcu.get(n)
            truth = self.truth.get(n)
            if (
                not fcu
                or fcu.armed
                or not fcu.connected
                or not truth
                or time.monotonic() - truth[1] > 1.0
            ):
                return
            p, q = pose_arrays(msg)
            wp, wq = pose_arrays(truth[0])
            if np.linalg.norm(q) < 0.9 or np.linalg.norm(wq) < 0.9:
                return
            # Local position axes are ENU by the pinned Gazebo XYZ->NED
            # transform and MAVROS conversion. Compass yaw error describes
            # attitude error, not a rotation of those position axes. Using it
            # as frame yaw would accumulate lateral position error downfield.
            yaw = float(self.args.settings_data.get("world_from_local_yaw", 0.0))
            rot = Rotation.from_euler("z", yaw)
            self.samples.setdefault(n, []).append((yaw, wp - rot.apply(p)))
            if len(self.samples[n]) < self.args.settings_data["calibration_samples"]:
                return
            samples = self.samples[n]
            yaw = float(
                np.arctan2(
                    np.mean(np.sin([s[0] for s in samples])),
                    np.mean(np.cos([s[0] for s in samples])),
                )
            )
            rot = Rotation.from_euler("z", yaw)
            offset = np.mean([s[1] for s in samples], axis=0)
            self.transforms[n] = (rot, offset)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "world"
            tf.child_frame_id = f"{n}/map"
            (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ) = map(float, offset)
            q = rot.as_quat()
            (
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ) = map(float, q)
            self.static.sendTransform(tf)
            self.get_logger().info(
                f"{n}: frozen world alignment yaw={yaw:.4f}, offset={offset.tolist()}"
            )
            self.save()
        rot, offset = self.transforms[n]
        p, q = pose_arrays(msg)
        if (
            not np.isfinite(p).all()
            or not np.isfinite(q).all()
            or np.linalg.norm(q) < 0.9
        ):
            return
        transformed = pose_message(
            rot.apply(p) + offset,
            self.get_clock().now().to_msg(),
            (rot * Rotation.from_quat(q)).as_quat(),
        )
        self.publishers_keep[n][0].publish(transformed)
        tf = TransformStamped()
        tf.header = transformed.header
        tf.child_frame_id = f"{n}/base_link"
        tf.transform.translation.x = transformed.pose.position.x
        tf.transform.translation.y = transformed.pose.position.y
        tf.transform.translation.z = transformed.pose.position.z
        tf.transform.rotation = transformed.pose.orientation
        self.dynamic.sendTransform(tf)

    def velocity(self, n, msg):
        if n not in self.transforms:
            return
        rot, _ = self.transforms[n]
        v = msg.twist.linear
        values = rot.apply([v.x, v.y, v.z])
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = map(float, values)
        self.publishers_keep[n][1].publish(out)

    def save(self):
        if self.args.run_dir:
            data = {
                n: {"yaw": float(r.as_euler("xyz")[2]), "translation": t.tolist()}
                for n, (r, t) in self.transforms.items()
            }
            path = Path(self.args.run_dir, "alignment.json")
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(data, indent=2) + "\n")
            temporary.replace(path)


def main():
    run_node(Frames, "Calibrate simulation world frames from disarmed measured poses")
