"""Independent flight publisher with command expiry, altitude and proximity guard.

This process never imports the optimizer. A blocked solver cannot block the
publish loop. It only controls vehicles already armed in GUIDED by mission CLI.
"""

import json
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .core.references import sample_trajectory
from .ros_utils import pose_arrays, qos_profile_sensor_data, run_node, seconds


class Setpoints(Node):
    def __init__(self, args):
        super().__init__("mpc_setpoints")
        self.args = args
        self.references = {}
        self.trajectories = {}
        self.poses = {}
        self.states = {}
        self.state_stamps = {}
        self.align = {}
        self.publishers_keep = {}
        self.subscriptions_keep = []
        self.last_reason = {}
        self.enabled = False
        self.create_service(SetBool, "/mpc/enable", self.enable)
        self.status = self.create_publisher(String, "/mpc/setpoint_status", 10)
        for n in args.names:
            self.publishers_keep[n] = self.create_publisher(
                PositionTarget, f"/{n}/mavros/setpoint_raw/local", 10
            )
            for typ, suffix, cb in [
                (PositionTarget, "mpc/reference", self.reference),
                (JointTrajectory, "mpc/trajectory", self.trajectory),
                (PoseStamped, "mpc/pose", self.pose),
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
        self.create_timer(1.0 / args.settings_data["publish_rate"], self.tick)

    def reference(self, n, msg):
        if msg.header.frame_id == "world":
            self.references[n] = (msg, time.monotonic())

    def trajectory(self, n, msg):
        if msg.header.frame_id == 'world' and msg.joint_names == ['x', 'y']:
            self.trajectories[n] = msg

    def pose(self, n, msg):
        self.poses[n] = (msg, time.monotonic())

    def state(self, n, msg):
        self.states[n] = (msg, time.monotonic())
        self.state_stamps[n] = self.get_clock().now().nanoseconds * 1e-9

    def enable(self, req, res):
        self.enabled = req.data
        if not self.enabled:
            self.references.clear()
            self.trajectories.clear()
        res.success = True
        res.message = "enabled" if self.enabled else "disabled"
        return res

    def tick(self):
        if not self.enabled:
            return
        if self.args.run_dir and not self.align:
            path = Path(self.args.run_dir, "alignment.json")
            try:
                self.align = json.loads(path.read_text())
                if len(self.align) != len(self.args.names):
                    self.align = {}
            except (FileNotFoundError, json.JSONDecodeError):
                pass
        now = self.get_clock().now()
        sim = now.nanoseconds * 1e-9
        wall = time.monotonic()
        settings = self.args.settings_data
        cfg = self.args.cfg
        statuses = {}
        for n in self.args.names:
            state = self.states.get(n)
            if (
                not state
                or not state[0].connected
                or not state[0].armed
                or state[0].mode != "GUIDED"
            ):
                continue
            if n not in self.poses or n not in self.align:
                continue
            pose, at = self.poses[n]
            p, q = pose_arrays(pose)
            reference = self.references.get(n)
            reason = "tracking"
            if (
                wall - at > settings["pose_timeout_wall"]
                or wall - state[1] > settings["fcu_timeout_wall"]
                or sim - self.state_stamps[n] > settings["fcu_timeout_sim"]
            ):
                reason = "stale_pose_or_fcu"
            elif (
                not reference
                or sim < seconds(reference[0].header.stamp)
                or sim - seconds(reference[0].header.stamp)
                > settings["command_timeout_sim"]
                or wall - reference[1] > settings["command_timeout_wall"]
            ):
                reason = "expired_command"
            values = None
            if reason == "tracking":
                source = reference[0]
                position = np.array(
                    [source.position.x, source.position.y, source.position.z]
                )
                velocity = np.array(
                    [source.velocity.x, source.velocity.y, source.velocity.z]
                )
                acceleration = np.array(
                    [
                        source.acceleration_or_force.x,
                        source.acceleration_or_force.y,
                        source.acceleration_or_force.z,
                    ]
                )
                trajectory = self.trajectories.get(n)
                if trajectory is not None and trajectory.header.stamp == source.header.stamp:
                    try:
                        xy, vxy, axy = sample_trajectory(
                            trajectory.points, sim - seconds(source.header.stamp))
                        position[:2], velocity[:2], acceleration[:2] = xy, vxy, axy
                    except ValueError:
                        reason = 'invalid_command'
                if not np.isfinite(np.r_[position, velocity, acceleration]).all():
                    reason = "invalid_command"
                elif (
                    np.linalg.norm(velocity[:2]) > cfg["VMAX"] * 1.05
                    or np.linalg.norm(acceleration[:2]) > cfg["UMAX"] * 1.05
                ):
                    reason = "command_limit"
                else:
                    # Halt approaching motion before entering the minimum distance.
                    for other, (op, ot) in self.poses.items():
                        if other == n or wall - ot > settings["pose_timeout_wall"]:
                            continue
                        delta = pose_arrays(op)[0][:2] - p[:2]
                        distance = np.linalg.norm(delta)
                        closing = float(
                            np.dot(velocity[:2], delta / max(distance, 1e-6))
                        )
                        if distance < cfg["UAV_SAFE_DISTANCE"] + 0.25 and closing > 0.0:
                            reason = "proximity_guard"
                            break
                    if reason == "tracking":
                        values = (position, velocity, acceleration)
            out = PositionTarget()
            out.header.stamp = now.to_msg()
            out.header.frame_id = f"{n}/map"
            out.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            rot = Rotation.from_euler("z", self.align[n]["yaw"])
            offset = np.asarray(self.align[n]["translation"])
            if values is None:
                out.type_mask = (
                    PositionTarget.IGNORE_PX
                    | PositionTarget.IGNORE_PY
                    | PositionTarget.IGNORE_PZ
                    | PositionTarget.IGNORE_AFX
                    | PositionTarget.IGNORE_AFY
                    | PositionTarget.IGNORE_AFZ
                    | PositionTarget.IGNORE_YAW
                    | PositionTarget.IGNORE_YAW_RATE
                )
                vz = (
                    0.0
                    if reason == "stale_pose_or_fcu"
                    else float(
                        np.clip(
                            settings["altitude_kp"]
                            * (settings["flight_altitude"] - p[2]),
                            -settings["vertical_speed_limit"],
                            settings["vertical_speed_limit"],
                        )
                    )
                )
                velocity = rot.inv().apply([0.0, 0.0, vz])
                out.velocity.x, out.velocity.y, out.velocity.z = map(float, velocity)
            else:
                out.type_mask = (
                    PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
                )
                position, velocity, acceleration = values
                # Altitude remains an explicit FCU position reference.
                velocity[2] = float(
                    np.clip(
                        settings["altitude_kp"] * (settings["flight_altitude"] - p[2]),
                        -settings["vertical_speed_limit"],
                        settings["vertical_speed_limit"],
                    )
                )
                position = rot.inv().apply(position - offset)
                velocity = rot.inv().apply(velocity)
                acceleration = rot.inv().apply(acceleration)
                out.position.x, out.position.y, out.position.z = map(float, position)
                out.velocity.x, out.velocity.y, out.velocity.z = map(float, velocity)
                (
                    out.acceleration_or_force.x,
                    out.acceleration_or_force.y,
                    out.acceleration_or_force.z,
                ) = map(float, acceleration)
            self.publishers_keep[n].publish(out)
            statuses[n] = reason
        self.status.publish(
            String(data=json.dumps({"stamp": sim, "vehicles": statuses}))
        )


def main():
    run_node(Setpoints, "Publish flight references independently of MPC computation")
