"""Drive the AprilTag ground robot from the keyboard (./scripts/dev drive).

  w / x   faster / slower forward (negative is reverse)
  a / d   turn left / right faster
  s       stop turning        space  stop
  q       quit (sends a stop first)

--auto drives by itself instead, using the `auto:` section of the robot config:
A* over the run's known obstacles (scenario plus live ones), pure pursuit along
the path, random goals or a patrol loop. Ctrl+C stops it.

Each key press changes the requested speed by the configured step; the sent
command ramps toward it at the configured acceleration and never exceeds the
limits in the robot config. Limits are read when this tool starts, so edit the
config and restart it; the simulation keeps running.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "ros_ws/src/solo_mpc"), str(ROOT / "ros_ws/src/solo_bringup")]


def ramp(current, target, limit, dt):
    """Move current toward target by at most limit * dt."""
    step = limit * dt
    return target if abs(target - current) <= step else current + math.copysign(step, target - current)


@dataclass
class Teleop:
    max_linear: float
    max_angular: float
    linear_step: float
    angular_step: float
    linear_accel: float
    angular_accel: float
    rate: float
    target_linear: float = 0.0
    target_angular: float = 0.0
    linear: float = 0.0
    angular: float = 0.0

    @classmethod
    def from_config(cls, robot):
        return cls(**{k: float(robot["teleop"][k]) for k in (
            "max_linear", "max_angular", "linear_step", "angular_step",
            "linear_accel", "angular_accel", "rate")})

    def key(self, char):
        """Apply one key; returns False to quit."""
        clip = lambda v, m: max(-m, min(m, v))
        if char == "w":
            self.target_linear = clip(self.target_linear + self.linear_step, self.max_linear)
        elif char == "x":
            self.target_linear = clip(self.target_linear - self.linear_step, self.max_linear)
        elif char == "a":
            self.target_angular = clip(self.target_angular + self.angular_step, self.max_angular)
        elif char == "d":
            self.target_angular = clip(self.target_angular - self.angular_step, self.max_angular)
        elif char == "s":
            self.target_angular = 0.0
        elif char == " ":
            self.target_linear = self.target_angular = 0.0
        elif char in ("q", "\x03"):
            self.target_linear = self.target_angular = 0.0
            return False
        return True

    def step(self, dt):
        self.linear = ramp(self.linear, self.target_linear, self.linear_accel, dt)
        self.angular = ramp(self.angular, self.target_angular, self.angular_accel, dt)
        return self.linear, self.angular


class Autopilot:
    """Goal selection, planning and stuck recovery around robot_autopilot."""

    STUCK_TIME = 6.0      # s without STUCK_PROGRESS metres of motion
    STUCK_PROGRESS = 0.15

    def __init__(self, robot, run_dir, log=print):
        import numpy as np
        from solo_mpc import robot_autopilot, scene_edit

        self.np, self.lib, self.scene_edit = np, robot_autopilot, scene_edit
        self.robot, self.run_dir, self.log = robot, Path(run_dir), log
        self.auto = robot["auto"]
        body = robot["body"]
        self.inflate = math.hypot(body["length"], body["width"]) / 2 + self.auto["clearance"]
        self.bounds = json.loads((self.run_dir / "mpc-config.json").read_text())["WORLD_BOUNDS"]
        self.rng = np.random.default_rng(self.auto.get("seed"))
        self.patrol = 0
        self.goal = self.path = None
        self.anchor = None
        self.retried = False

    def grid(self):
        scenario = yaml.safe_load((self.run_dir / "scenario.yaml").read_text())
        shapes = [("box", x + w / 2, y + h / 2, [w, h]) for x, y, w, h in scenario["obstacles"]["rects"]]
        shapes += [("circle", x, y, [r]) for x, y, r in scenario["obstacles"]["circles"]]
        shapes += [(d["shape"], d["x"], d["y"], d["size"]) for d in self.scene_edit.load(self.run_dir).values()]
        return self.lib.OccupancyMap(self.bounds, shapes, self.inflate)

    def next_goal(self, grid, position):
        if self.auto["mode"] == "patrol":
            points = self.auto["waypoints"]
            goal = self.np.asarray(points[self.patrol % len(points)], float)
            self.patrol += 1
            return goal
        return self.lib.random_goal(grid, self.rng, position, self.auto["min_leg"])

    def replan(self, position, now, new_goal):
        grid = self.grid()  # reload: live obstacles may have been added
        for _ in range(10):
            if new_goal or self.goal is None:
                self.goal = self.next_goal(grid, position)
            self.path = None if self.goal is None else self.lib.plan(grid, position, self.goal)
            if self.path:
                self.log(f"goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}), path {len(self.path)} points")
                self.anchor = (position, now)
                return True
            new_goal = True
        self.log("no reachable goal found; holding")
        return False

    def command(self, pose, now, teleop):
        """Requested (linear, angular) for pose (x, y, yaw) at wall time now."""
        position = self.np.asarray(pose[:2], float)
        if self.path is None and not self.replan(position, now, new_goal=True):
            return 0.0, 0.0
        if self.np.linalg.norm(position - self.anchor[0]) > self.STUCK_PROGRESS:
            self.anchor, self.retried = (position, now), False
        elif now - self.anchor[1] > self.STUCK_TIME:
            self.log("no progress; " + ("new goal" if self.retried else "replanning"))
            if not self.replan(position, now, new_goal=self.retried):
                return 0.0, 0.0
            self.retried = not self.retried
        speed = min(self.auto["speed"], teleop.max_linear)
        linear, angular, done = self.lib.pursuit(pose, self.path, speed, self.auto["lookahead"],
                                                 teleop.max_angular, self.auto["goal_tolerance"])
        if done:
            self.log("goal reached")
            self.path = None
        return linear, angular


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1).lower() if ready else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(ROOT / "config/ground_robot.yaml"),
                        help="robot config with the teleop limits")
    parser.add_argument("--auto", action="store_true", help="drive autonomously around the map")
    parser.add_argument("--run-dir", default=str(ROOT / "runs/latest"),
                        help="running simulation whose obstacles --auto avoids")
    args = parser.parse_args(argv)
    from solo_mpc import ground_robot

    robot = ground_robot.load(args.config)
    teleop = Teleop.from_config(robot)
    if args.auto:
        if robot.get("auto") is None:
            raise SystemExit(f"{args.config} has no auto: section")
        return drive_auto(robot, teleop, args.run_dir)
    if not sys.stdin.isatty():
        raise SystemExit("drive needs an interactive terminal (run ./scripts/dev drive in a terminal)")

    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("ground_robot_teleop")
    publisher = node.create_publisher(Twist, ground_robot.CMD_TOPIC, 10)

    def send(linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        publisher.publish(msg)

    print(__doc__.split("\n\n")[1])
    print(f"limits: {teleop.max_linear} m/s, {teleop.max_angular} rad/s  (from {args.config})")
    saved = termios.tcgetattr(sys.stdin)
    period = 1.0 / teleop.rate
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = time.monotonic()
        running = True
        while running and rclpy.ok():
            char = read_key(max(0.0, period - (time.monotonic() - last)))
            if char is not None:
                running = teleop.key(char)
            now = time.monotonic()
            if now - last >= period or not running:
                linear, angular = teleop.step(now - last)
                send(linear, angular)
                last = now
                print(f"\rcmd {linear:+.2f} m/s {angular:+.2f} rad/s   "
                      f"(requested {teleop.target_linear:+.2f}, {teleop.target_angular:+.2f})  ",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the robot driving on the last command.
        for _ in range(3):
            send(0.0, 0.0)
            time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        print("\nstopped")
        node.destroy_node()
        rclpy.shutdown()


def drive_auto(robot, teleop, run_dir):
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from solo_mpc import ground_robot

    rclpy.init()
    node = rclpy.create_node("ground_robot_autopilot")
    publisher = node.create_publisher(Twist, ground_robot.CMD_TOPIC, 10)
    latest = {}

    def pose(msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        latest["pose"] = (msg.pose.position.x, msg.pose.position.y, yaw)

    node.create_subscription(PoseStamped, "/target/ground_truth", pose, 10)

    def send(linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        publisher.publish(msg)

    pilot = Autopilot(robot, run_dir, log=lambda text: print(f"\n{text}", flush=True))
    print(f"autonomous {robot['auto']['mode']} driving at {min(robot['auto']['speed'], teleop.max_linear)} m/s; "
          "Ctrl+C stops")
    period = 1.0 / teleop.rate
    try:
        deadline = time.monotonic() + 10.0
        while "pose" not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "pose" not in latest:
            raise SystemExit("No /target/ground_truth: start a simulation with ground_robot in drone.yaml")
        last = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0.0, period - (time.monotonic() - last)))
            now = time.monotonic()
            if now - last < period:
                continue
            teleop.target_linear, teleop.target_angular = pilot.command(latest["pose"], now, teleop)
            linear, angular = teleop.step(now - last)
            send(linear, angular)
            last = now
            x, y, _ = latest["pose"]
            print(f"\rpose ({x:5.1f}, {y:5.1f})  cmd {linear:+.2f} m/s {angular:+.2f} rad/s   ",
                  end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            send(0.0, 0.0)
            time.sleep(0.05)
        print("\nstopped")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
