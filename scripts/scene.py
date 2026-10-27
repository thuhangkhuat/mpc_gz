"""Edit the running simulation: move the target, reposition a UAV, add/remove obstacles.

  target X Y                     Fix the target at (X, Y)
  drone X Y [--uav NAME]         Fly a UAV to (X, Y) by fixing the target there; waits for arrival
  obstacle add circle X Y R      Pole of radius R centred at (X, Y)
  obstacle add box X Y W D       Box W (along X) by D (along Y) centred at (X, Y)
  obstacle remove NAME | clear | list

Coordinates are world metres. Obstacles get the run's obstacle_height. Drone
repositioning flies with MPC while a mission runs; it never teleports the
vehicle, which would invalidate the ArduPilot estimator and frame alignment.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml

RUN = Path("/workspace/runs/latest")


def run_data():
    cfg = json.loads((RUN / "mpc-config.json").read_text())
    scenario = yaml.safe_load((RUN / "scenario.yaml").read_text())
    settings = yaml.safe_load((RUN / "controller.yaml").read_text())
    return cfg, scenario.get("world_name", "mpc_small50"), settings


class Ros:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("scene_editor")
        self.status = self.target = None
        self.node.create_subscription(String, "/mpc/status", self._status, 10)
        self.node.create_subscription(PoseStamped, "/target/ground_truth", self._target, 10)
        self.goal = self.node.create_publisher(PoseStamped, "/target/goal", 10)

    def _status(self, msg):
        self.status = json.loads(msg.data)

    def _target(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)

    def spin(self, seconds, until=lambda: False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def set_target(self, x, y):
        from geometry_msgs.msg import PoseStamped

        self.spin(5.0, lambda: self.goal.get_subscription_count() > 0)
        if not self.goal.get_subscription_count():
            raise SystemExit("Target node is not running; start the simulation first")
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        self.goal.publish(msg)
        moved = lambda: self.target and math.hypot(self.target[0] - x, self.target[1] - y) < 0.05
        self.spin(3.0, moved)
        if not moved():
            raise SystemExit("Target did not move; check ./scripts/dev logs")

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def check_bounds(cfg, x, y):
    x0, y0, x1, y1 = cfg["WORLD_BOUNDS"]
    if not (math.isfinite(x) and math.isfinite(y) and x0 <= x <= x1 and y0 <= y <= y1):
        raise SystemExit(f"({x}, {y}) is outside the map x[{x0}, {x1}] y[{y0}, {y1}]")


def drone(ros, cfg, args):
    ros.spin(3.0, lambda: ros.status is not None)
    vehicles = (ros.status or {}).get("vehicles", {})
    name = args.uav or next(iter(vehicles), None)
    if name not in vehicles or not vehicles[name].get("armed"):
        raise SystemExit("Drone repositioning needs a flying mission: run ./scripts/dev mission run")
    ros.set_target(args.x, args.y)
    radius = cfg["VIEWING_RADIUS"]
    print(f"Flying {name} to ({args.x}, {args.y}); waiting up to {args.timeout:g} s", flush=True)
    deadline = time.monotonic() + args.timeout
    next_report = 0.0
    while time.monotonic() < deadline:
        ros.spin(0.5)
        p = ros.status["vehicles"][name]["position"]
        distance = math.hypot(p[0] - args.x, p[1] - args.y)
        if distance < radius:
            print(f"{name} arrived at ({p[0]:.2f}, {p[1]:.2f})")
            return
        if time.monotonic() >= next_report:
            print(f"  {distance:.1f} m to go ({ros.status['vehicles'][name]['result']})", flush=True)
            next_report = time.monotonic() + 5.0
    raise SystemExit(f"{name} did not arrive within {args.timeout:g} s")


def obstacle(ros, cfg, world, settings, args):
    from gz.transport13 import Node as GazeboNode
    from solo_mpc import scene_edit

    gz = GazeboNode()
    if args.action == "list":
        for name, data in sorted(scene_edit.load(RUN).items()):
            print(f"{name}: {data['shape']} at ({data['x']}, {data['y']}) size {data['size']}")
        return
    if args.action == "clear":
        for name in sorted(scene_edit.load(RUN)):
            scene_edit.remove(gz, world, RUN, name)
            print(f"Removed {name}")
        return
    if args.action == "remove":
        scene_edit.remove(gz, world, RUN, args.values[0])
        print(f"Removed {args.values[0]}")
        return
    shape, numbers = args.values[0], args.values[1:]
    sizes = {"circle": 1, "box": 2}
    if shape not in sizes or len(numbers) != 2 + sizes[shape]:
        raise SystemExit("usage: obstacle add circle X Y R | obstacle add box X Y W D")
    try:
        x, y, *size = map(float, numbers)
    except ValueError:
        raise SystemExit("obstacle coordinates and sizes must be numbers")
    check_bounds(cfg, x, y)
    if not args.force:
        # The LiDAR sees only SENSING_RADIUS ahead; a pole dropped closer than
        # that may leave too little room to stop at Solo speed.
        ros.spin(2.0, lambda: ros.status is not None and ros.target is not None)
        keep = [(n, v["position"][:2], cfg["SENSING_RADIUS"] + cfg["ROBOT_RADIUS"])
                for n, v in ((ros.status or {}).get("vehicles") or {}).items()]
        if ros.target:
            keep.append(("target", ros.target, cfg["VIEWING_RADIUS"]))
        for what, point, distance in keep:
            if scene_edit.clearance(shape, x, y, size, point) < distance:
                raise SystemExit(f"Obstacle would be within {distance:.2f} m of {what}; "
                                 "move it or pass --force")
    name = scene_edit.spawn(gz, world, RUN, shape, x, y, size, settings["obstacle_height"])
    print(f"Spawned {name}: {shape} at ({x}, {y}) size {size}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    target = sub.add_parser("target", help="fix the target at X Y")
    target.add_argument("x", type=float)
    target.add_argument("y", type=float)
    move = sub.add_parser("drone", help="fly a UAV to X Y (mission must be running)")
    move.add_argument("x", type=float)
    move.add_argument("y", type=float)
    move.add_argument("--uav")
    move.add_argument("--timeout", type=float, default=120.0)
    obs = sub.add_parser("obstacle", help="add, remove, clear or list live obstacles")
    obs.add_argument("action", choices=["add", "remove", "clear", "list"])
    obs.add_argument("values", nargs="*")
    obs.add_argument("--force", action="store_true", help="skip UAV/target clearance checks")
    args = parser.parse_args(argv)
    if args.command == "obstacle" and args.action == "remove" and len(args.values) != 1:
        parser.error("obstacle remove NAME")
    cfg, world, settings = run_data()
    if args.command in ("target", "drone"):
        check_bounds(cfg, args.x, args.y)
    ros = Ros()
    try:
        if args.command == "target":
            ros.set_target(args.x, args.y)
            print(f"Target fixed at ({args.x}, {args.y})")
        elif args.command == "drone":
            drone(ros, cfg, args)
        else:
            obstacle(ros, cfg, world, settings, args)
    finally:
        ros.close()


if __name__ == "__main__":
    sys.exit(main())
