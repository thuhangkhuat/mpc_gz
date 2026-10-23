"""Small ROS message adapters; the optimizer does not depend on these."""

import argparse
import signal

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data as qos_profile_sensor_data

from .settings import load_settings


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--settings", required=True)
    parser.add_argument("--fleet", required=True)
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--vehicle", help="Run the camera detector for one fleet vehicle")
    args, ros = parser.parse_known_args()
    from pathlib import Path

    import yaml

    args.names = [
        u["name"] for u in yaml.safe_load(Path(args.fleet).read_text())["uavs"]
    ]
    if args.vehicle is not None and args.vehicle not in args.names:
        parser.error("--vehicle must name a vehicle in --fleet")
    args.scenario_data, args.settings_data, args.cfg = load_settings(
        args.scenario, args.settings
    )
    return args, ros


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_arrays(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def pose_message(position, stamp, quaternion=(0.0, 0.0, 0.0, 1.0), frame="world"):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ) = map(float, quaternion)
    return msg



def run_node(node_class, description):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    args, ros = arguments(description)
    rclpy.init(args=ros)
    node = None
    try:
        node = node_class(args)
        node.get_logger().info(f'{node.get_name()}: initialized; waiting for inputs')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward SIGINT after the terminal already delivered it.
        # Finish bounded worker shutdown before accepting another interrupt.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
