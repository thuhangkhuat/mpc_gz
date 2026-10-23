"""ROS adapter around the independent measured-state TeamController."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .core.controller import VehicleState
from .core.cycles import CycleRunner, result_is_current, stopping_result_matches
from .core.parallel_controller import ParallelTeamController
from .core.sensors import PoseHistory, project_scan
from .core.tracking import ExactTargetTracker
from .ros_utils import (
    pose_arrays,
    pose_message,
    qos_profile_sensor_data,
    run_node,
    seconds,
)


class Controller(Node):
    def __init__(self, args):
        super().__init__("mpc_controller")
        self.args = args
        self.execution = args.settings_data.get("controller_execution", "parallel")
        self.poses = {}
        self.velocities = {}
        self.accelerations = {}
        self.scans = {}
        self.fcu = {}
        self.histories = {
            n: PoseHistory(args.settings_data["frame_history_tolerance"])
            for n in args.names
        }
        self.observed = {}
        self.last_observation = -np.inf
        self.subscriptions_keep = []
        self.outputs = {}
        self.trajectories = {}
        self.status_pub = self.create_publisher(String, "/mpc/status", 10)
        self.log = None
        if args.run_dir:
            self.log = open(Path(args.run_dir, "mpc.jsonl"), "a", buffering=1)
            package = Path(__file__).parent
            sources = {
                str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(package.rglob("*.py"))
            }
            with open(
                Path(args.run_dir, "controller_sources.jsonl"), "a"
            ) as source_log:
                source_log.write(
                    json.dumps(
                        {"wall": time.time(), "sha256": sources, "config": args.cfg}
                    )
                    + "\n"
                )
        for n in args.names:
            self.trajectories[n] = self.create_publisher(JointTrajectory, f"/{n}/mpc/trajectory", 10)
            self.outputs[n] = (
                self.create_publisher(PositionTarget, f"/{n}/mpc/reference", 10),
                self.create_publisher(RosPath, f"/{n}/mpc/prediction", 10),
            )
            subscriptions = [
                (PoseStamped, "mpc/pose", self.pose),
                (TwistStamped, "mpc/velocity", self.velocity),
                (LaserScan, "scan", self.scan),
                (State, "mavros/state", self.state),
            ]
            if args.settings_data["target_mode"] == "camera":
                subscriptions.append((PoseStamped, "target/observation", self.observation))
            for typ, suffix, cb in subscriptions:
                self.subscriptions_keep.append(
                    self.create_subscription(
                        typ,
                        f"/{n}/{suffix}",
                        lambda msg, name=n, fn=cb: fn(name, msg),
                        qos_profile_sensor_data,
                    )
                )
        if args.settings_data["target_mode"] == "ground_truth":
            self.subscriptions_keep.append(
                self.create_subscription(
                    PoseStamped,
                    "/target/ground_truth",
                    lambda msg: self.observation("ground_truth", msg),
                    10,
                )
            )
        cfg = dict(args.cfg)
        if args.run_dir:
            cfg['DIAGNOSTIC_DIR'] = str(Path(args.run_dir, 'optimizer-inputs'))
            cfg['DIAGNOSTIC_CAPTURE_CYCLES'] = args.settings_data.get('diagnostic_capture_cycles', 128)
        self.team = ParallelTeamController(
            args.names, cfg,
            timeout=args.settings_data.get("controller_worker_timeout_wall", 1.0),
            startup_timeout=args.settings_data.get("controller_worker_startup_timeout_wall", 30.0),
            sequential=self.execution == 'sequential',
        )
        if args.settings_data["target_mode"] == "ground_truth":
            self.team.track = ExactTargetTracker(
                cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
        self.cycles = CycleRunner()
        self.cycle_context = None
        self.last_cycle_stamp = -np.inf
        try:
            # Poll completed work promptly. A start-only 10 Hz timer plus a separate
            # completion tick inserted two idle periods into every fleet cycle.
            self.create_timer(min(0.02, args.settings_data["control_period"]), self.tick)
        except BaseException:
            self.cycles.close()
            if isinstance(self.team, ParallelTeamController):
                self.team.close()
            raise

    def pose(self, n, msg):
        if msg.header.frame_id != "world":
            return
        p, q = pose_arrays(msg)
        stamp = seconds(msg.header.stamp)
        if (
            not np.isfinite(p).all()
            or not np.isfinite(q).all()
            or np.linalg.norm(q) < 0.9
        ):
            return
        if n in self.poses and stamp <= self.poses[n][2]:
            return
        self.poses[n] = (p, q, stamp)
        self.histories[n].add(stamp, p, q)

    def velocity(self, n, msg):
        stamp = seconds(msg.header.stamp)
        v = msg.twist.linear
        v = np.array([v.x, v.y, v.z])
        if msg.header.frame_id != 'world' or not np.isfinite(v).all():
            return
        old = self.velocities.get(n)
        if old and stamp <= old[1]:
            return
        if old:
            estimate = (v - old[0]) / (stamp - old[1])
            prev = self.accelerations.get(n, np.zeros(3))
            self.accelerations[n] = (
                0.3 * np.clip(estimate, -self.args.cfg["UMAX"], self.args.cfg["UMAX"])
                + 0.7 * prev
            )
        self.velocities[n] = (v, stamp)

    def scan(self, n, msg):
        self.scans[n] = msg

    def state(self, n, msg):
        self.fcu[n] = msg

    def observation(self, n, msg):
        sources = ("ground_truth",) if self.args.settings_data["target_mode"] == "ground_truth" else self.args.names
        if n not in sources:
            return
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        position = pose_arrays(msg)[0]
        if (msg.header.frame_id != 'world' or not np.isfinite(position).all()
                or not 0 <= now - stamp <= self.args.settings_data['target_timeout_sim']):
            return
        if stamp > self.observed.get(n, -np.inf):
            self.observed[n] = stamp
        if self.team.track.observe(position, stamp):
            self.last_observation = stamp

    def tick(self):
        if self.cycles.future is not None:
            if not self.cycles.future.done():
                return
            start, now, stamp = self.cycle_context
            try:
                results = self.cycles.take()
                reason = ('partial_solver_failure' if any(
                    not out.get('nominal_ok', out['ok']) for out in results.values()) else 'ready')
            except Exception as exc:
                results, reason = {}, f'controller_error: {exc}'
                self.get_logger().error(reason)
            current = self.get_clock().now().nanoseconds * 1e-9
            if not result_is_current(stamp, current, self.cycles.started, self.args.settings_data):
                results, reason = {}, 'obsolete_cycle'
            elif not all(n in self.fcu and self.fcu[n].connected and self.fcu[n].armed
                         and self.fcu[n].mode == 'GUIDED' for n in self.args.names):
                results, reason = {}, 'flight_state_changed'
            for n, out in results.items():
                if out['ok'] and not out.get('nominal_ok', True):
                    p, q, at = self.poses[n]
                    v, vt = self.velocities[n]
                    measured = VehicleState(p, v, self.accelerations.get(n, np.zeros(3)), q, at)
                    if not stopping_result_matches(measured, out, stamp, self.args.cfg, velocity_stamp=vt):
                        results[n] = dict(out, ok=False, reason='stop_plan_measured_state_changed')
                        reason = 'partial_solver_failure'
            self._publish_cycle(start, now, stamp, reason, results)
            if reason != 'ready':
                return
        start = time.perf_counter()
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        if 0 <= stamp - self.last_cycle_stamp < self.args.settings_data['control_period'] - 1e-9:
            return
        self.last_cycle_stamp = stamp
        cfg = self.args.cfg
        settings = self.args.settings_data
        names = self.args.names
        reason = "ready"
        states = {}
        scans = {}
        results = {}
        if not all(
            n in self.poses and n in self.velocities and n in self.scans for n in names
        ):
            reason = "waiting_for_measured_state_and_scans"
        elif not all(
            n in self.fcu
            and self.fcu[n].connected
            and self.fcu[n].armed
            and self.fcu[n].mode == "GUIDED"
            for n in names
        ):
            reason = "waiting_for_armed_guided_fleet"
        elif (
            self.team.track.observed_at is None
            or not 0 <= stamp - self.team.track.observed_at <= settings[
                "target_timeout_sim" if settings["target_mode"] == "ground_truth"
                else "target_search_timeout_sim"
            ]
        ):
            reason = "target_unavailable"
        else:
            for n in names:
                p, q, at = self.poses[n]
                v, vt = self.velocities[n]
                scan = self.scans[n]
                if not all(0 <= stamp - value <= settings['state_timeout_sim'] for value in (at, vt)):
                    reason = "stale_state"
                    break
                if not 0 <= stamp - seconds(scan.header.stamp) <= settings['sensor_timeout_sim']:
                    reason = "stale_scan"
                    break
                states[n] = VehicleState(
                    p, v, self.accelerations.get(n, np.zeros(3)), q, at
                )
                try:
                    sp, sq = self.histories[n].at(seconds(scan.header.stamp))
                    from scipy.spatial.transform import Rotation

                    sp = sp + Rotation.from_quat(sq).apply([0.0, 0.0, 0.08])
                    scans[n] = project_scan(
                        scan.ranges,
                        scan.angle_min,
                        scan.angle_increment,
                        scan.range_min,
                        scan.range_max,
                        sp,
                        sq,
                        seconds(scan.header.stamp),
                        neighbors=[self.histories[k].at(seconds(scan.header.stamp))[0]
                                   for k in names if k != n],
                        neighbor_radius=cfg["ROBOT_RADIUS"] + 0.15,
                        height_tolerance=settings["scan_height_tolerance"],
                    )
                except ValueError:
                    reason = "scan_pose_unavailable"
                    break
                if len(scans[n].endpoints) < len(scan.ranges) * 0.7:
                    reason = "insufficient_valid_scan"
                    break
            if reason == "ready":
                seers = {
                    n
                    for n, at in self.observed.items()
                    if 0 <= stamp - at < settings["sensor_timeout_sim"]
                }
                if settings["target_mode"] == "ground_truth":
                    # Geometric tracking/formation gates need no camera visibility.
                    target = self.team.track.position(stamp)
                    seers = {
                        n
                        for n in names
                        if max(abs(self.poses[n][0][:2] - target))
                        <= cfg["VIEWING_RADIUS"]
                    }
                self.cycle_context = start, now, stamp
                self.cycles.submit(self.team.step, states, scans, stamp, seers, tracker=self.team.track)
                return
        self._publish_cycle(start, now, stamp, reason, results)

    def _publish_cycle(self, start, now, stamp, reason, results):
        settings = self.args.settings_data
        # A completed solve may already be obsolete. Publisher checks age too.
        for n, out in results.items():
            if not out["ok"]:
                continue
            msg = PositionTarget()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = "world"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            msg.position.x, msg.position.y = map(float, out["position"])
            msg.position.z = settings["flight_altitude"]
            msg.velocity.x, msg.velocity.y = map(float, out["velocity"])
            msg.acceleration_or_force.x, msg.acceleration_or_force.y = map(
                float, out["acceleration"]
            )
            self.outputs[n][0].publish(msg)
            trajectory = JointTrajectory()
            trajectory.header = msg.header
            trajectory.joint_names = ['x', 'y']
            controls = out.get('controls')
            if controls is not None:
                for k, row in enumerate(out['prediction']):
                    at = int(round(k * self.args.cfg['TIMESTEP'] * 1e9))
                    trajectory.points.append(JointTrajectoryPoint(
                        positions=list(map(float, row[:2])),
                        velocities=list(map(float, row[2:4])),
                        accelerations=list(map(float, controls[min(k, len(controls) - 1)])),
                        time_from_start=Duration(sec=at // 10**9, nanosec=at % 10**9)))
                self.trajectories[n].publish(trajectory)
            path = RosPath()
            path.header = msg.header
            path.poses = [
                pose_message([x[0], x[1], settings["flight_altitude"]], now.to_msg())
                for x in out["prediction"]
            ]
            self.outputs[n][1].publish(path)
        # Callbacks can receive a newer target while this cycle is solving.
        # Freshness describes the live input when reporting, not the solve's
        # earlier timestamp (which could produce a negative observation age).
        reported_at = self.get_clock().now().nanoseconds * 1e-9
        target_age = (None if self.team.track.observed_at is None
                      else reported_at - self.team.track.observed_at)
        record = {
            "stamp": stamp,
            "wall": time.time(),
            "compute_ms": 1000 * (time.perf_counter() - start),
            "execution": self.execution,
            "reason": reason,
            "leader": self.team.leader,
            "controller_modes": self.team.modes,
            "assignment": self.team.assignment,
            "slot_scales": self.team.slot_scales,
            "target_mode": settings["target_mode"],
            "target_input_fresh": target_age is not None and 0 <= target_age <= settings['target_timeout_sim'],
            "target_observation_age": target_age,
            "target_estimate": None
            if self.team.track.x is None
            else self.team.track.x.tolist(),
            "vehicles": {
                n: {
                    "position": p.tolist(),
                    "velocity": self.velocities.get(n, (np.zeros(3), 0))[0].tolist(),
                    "result": results.get(n, {}).get("reason", reason),
                    "nominal_ok": results.get(n, {}).get("nominal_ok", False),
                    "solver_status": results.get(n, {}).get("solver_status"),
                    "solver_ms": results.get(n, {}).get("solver_ms"),
                    **{
                        key: results.get(n, {}).get(key)
                        for key in (
                            "worker_pid", "cycle", "mapping_ms", "path_ms", "corridor_ms",
                            "solver_iterations", "planning_and_solve_ms",
                            "map_started_wall", "map_finished_wall",
                            "solve_started_wall", "solve_finished_wall", "corridor_faces",
                        )
                    },
                    "armed": bool(n in self.fcu and self.fcu[n].armed),
                    "mode": self.fcu[n].mode if n in self.fcu else None,
                }
                for n, (p, q, at) in self.poses.items()
            },
        }
        text = json.dumps(record)
        self.status_pub.publish(String(data=text))
        if self.log:
            self.log.write(text + "\n")

    def destroy_node(self):
        self.cycles.close()
        if isinstance(self.team, ParallelTeamController):
            self.team.close()
        if self.log:
            self.log.close()
        return super().destroy_node()


def main():
    run_node(
        Controller, "Multi-UAV MPC using measured ROS states and sensor observations"
    )
