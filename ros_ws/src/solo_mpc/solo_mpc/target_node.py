"""Simulation-only target motion, evaluated at Gazebo clock time."""

import numpy as np
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GazeboNode
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

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
        self.publisher = self.create_publisher(PoseStamped, "/target/ground_truth", 10)
        self.create_service(SetBool, "/target/run", self.set_running)
        self.create_service(Trigger, "/target/reset", self.reset)
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

    def tick(self):
        sampled_at = self.get_clock().now()
        elapsed = sampled_at.nanoseconds * 1e-9 - self.started if self.running else self.elapsed
        point = np.array(
            [
                np.interp(elapsed, self.times, self.path[:, 0]),
                np.interp(elapsed, self.times, self.path[:, 1]),
                self.args.settings_data["target_height"],
            ]
        )
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
