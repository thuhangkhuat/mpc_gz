"""Simulation-only target motion, evaluated at Gazebo clock time.

A pose on /target/goal or RViz /goal_pose replaces the route with a fixed
position until the simulation stops; an RViz /clicked_point spawns a pole.
With a ground_robot configured the robot is the target: Gazebo publishes its
pose (bridged), this node only serves mission requests, and a goal pose
teleports the robot instead.
"""

import threading

import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GazeboNode
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from . import ground_robot, scene_edit
from .core.target_motion import target_route
from .ros_utils import pose_message, run_node


class MovingTarget(Node):
    def __init__(self, args):
        super().__init__("mpc_target")
        self.args = args
        self.running = False
        self.started = 0.0
        self.elapsed = 0.0
        self.gz = GazeboNode()
        self.path, self.times = target_route(args.scenario_data, args.cfg)
        self.goal = None
        for topic in ("/target/goal", "/goal_pose"):
            self.create_subscription(PoseStamped, topic, self.set_goal, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.add_pole, 10)
        self.robot = args.settings_data.get("ground_robot")
        self.create_service(SetBool, "/target/run", self.set_running)
        self.create_service(Trigger, "/target/reset", self.reset)
        if self.robot is None:
            self.publisher = self.create_publisher(PoseStamped, "/target/ground_truth", 10)
            self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_running(self, req, res):
        if req.data and not self.running:
            self.started = self.now() - self.elapsed
        elif not req.data and self.running:
            self.elapsed = self.now() - self.started
        self.running = req.data
        res.success = True
        res.message = "running" if req.data else "paused"
        return res

    def reset(self, req, res):
        self.running = False
        self.elapsed = 0.0
        self.started = self.now()
        res.success = True
        return res

    def _in_bounds(self, msg, x, y, what):
        x0, y0, x1, y1 = self.args.cfg["WORLD_BOUNDS"]
        if msg.header.frame_id not in ("", "world"):
            self.get_logger().warning(f"{what} ignored: frame {msg.header.frame_id!r} is not world")
        elif not (np.isfinite([x, y]).all() and x0 <= x <= x1 and y0 <= y <= y1):
            self.get_logger().warning(f"{what} ignored: ({x:.2f}, {y:.2f}) is outside the map")
        else:
            return True
        return False

    def set_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        if not self._in_bounds(msg, x, y, "Target goal"):
            return
        if self.robot is None:
            # Survives /target/reset, which each mission start calls.
            self.goal = np.array([x, y])
            self.get_logger().info(f"Target fixed at ({x:.2f}, {y:.2f})")
            return
        q = msg.pose.orientation
        request = Pose()
        request.name = ground_robot.MODEL
        request.position.x, request.position.y = float(x), float(y)
        if q.x or q.y or q.z or q.w:
            request.orientation.x, request.orientation.y = q.x, q.y
            request.orientation.z, request.orientation.w = q.z, q.w
        else:
            request.orientation.w = 1.0
        ok, response = self.gz.request(
            f"/world/{self.args.world}/set_pose", request, Pose, Boolean, 1000)
        if ok and response.data:
            self.get_logger().info(f"Ground robot moved to ({x:.2f}, {y:.2f})")
        else:
            self.get_logger().error("Gazebo did not move the ground robot")

    def add_pole(self, msg):
        x, y = msg.point.x, msg.point.y
        if not self._in_bounds(msg, x, y, "Clicked obstacle"):
            return

        def spawn():
            try:
                name = scene_edit.spawn(self.gz, self.args.world, self.args.run_dir, "circle", x, y,
                                        [0.15], self.args.settings_data["obstacle_height"])
                self.get_logger().info(f"Spawned {name} (pole r=0.15) at ({x:.2f}, {y:.2f})")
            except (RuntimeError, ValueError) as exc:
                self.get_logger().error(str(exc))

        # A slow Gazebo reply must not stall the 20 Hz target publisher.
        threading.Thread(target=spawn, daemon=True).start()

    def tick(self):
        sampled_at = self.get_clock().now()
        elapsed = sampled_at.nanoseconds * 1e-9 - self.started if self.running else self.elapsed
        xy = self.goal if self.goal is not None else [
            np.interp(elapsed, self.times, self.path[:, 0]),
            np.interp(elapsed, self.times, self.path[:, 1]),
        ]
        point = np.array([*xy, self.args.settings_data["target_height"]])
        request = Pose()
        request.name = "tracking_target"
        request.position.x, request.position.y, request.position.z = map(float, point)
        request.orientation.w = 1.0
        ok, response = self.gz.request(
            f"/world/{self.args.world}/set_pose", request, Pose, Boolean, 200
        )
        if ok and response.data:
            self.publisher.publish(pose_message(point, sampled_at.to_msg()))
        if elapsed >= self.times[-1]:
            self.running = False
            self.elapsed = self.times[-1]


def main():
    run_node(MovingTarget, "Move the visible Gazebo target along the scenario spline")
