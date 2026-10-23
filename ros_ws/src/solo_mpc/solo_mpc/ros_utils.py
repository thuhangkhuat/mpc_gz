"""Small ROS message adapters; the optimizer does not depend on these."""

import argparse

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
    parser.add_argument("--world", default="mpc_small50")
    parser.add_argument("--vehicle", help="Run the camera detector for one fleet vehicle")
    parser.add_argument('--startup-component', help='Component name in the launch startup inventory')
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

    args, ros = arguments(description)
    rclpy.init(args=ros)
    node = None
    progress = None
    if args.run_dir and args.startup_component:
        from solo_bringup.startup import ComponentProgress

        progress = ComponentProgress(args.run_dir, args.startup_component)
    try:
        if progress:
            progress.update('initializing', f'Constructing {node_class.__name__}')
        node = node_class(args)
        if progress:
            progress.update('initialized', 'ROS node initialized; waiting for mission inputs')
        node.get_logger().info(f'{node.get_name()}: initialized; waiting for mission inputs')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException as error:
        if progress:
            progress.update('failed', f'{type(error).__name__}: {error}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
