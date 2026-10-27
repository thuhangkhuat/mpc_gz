"""Drive the AprilTag ground robot from the keyboard (./scripts/dev drive).

  w / x   faster / slower forward (negative is reverse)
  a / d   turn left / right faster
  s       stop turning        space  stop
  q       quit (sends a stop first)

Each key press changes the requested speed by the configured step; the sent
command ramps toward it at the configured acceleration and never exceeds the
limits in the robot config. Limits are read when this tool starts, so edit the
config and restart it; the simulation keeps running.
"""

import argparse
from dataclasses import dataclass
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


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1).lower() if ready else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(ROOT / "config/ground_robot.yaml"),
                        help="robot config with the teleop limits")
    args = parser.parse_args(argv)
    from solo_mpc import ground_robot

    robot = ground_robot.load(args.config)
    teleop = Teleop.from_config(robot)
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


if __name__ == "__main__":
    main()
