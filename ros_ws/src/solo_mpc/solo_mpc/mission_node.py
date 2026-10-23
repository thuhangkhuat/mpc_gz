"""Persistent simulation mission ownership, supervision and landing.

Service callbacks only queue requests. Flight operations run outside callbacks,
so Flight.wait can spin the same executor for telemetry and stop requests without
threads or nested service callbacks blocking each other.
"""

import argparse
import json
import math
from pathlib import Path
import signal
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from solo_bringup.flight import Flight, FlightReleased, Ownership
from solo_bringup.startup import ComponentProgress
from solo_mpc_interfaces.srv import StartMission
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml

from .ros_utils import pose_arrays, qos_profile_sensor_data, seconds


class MissionStopped(Exception):
    """Interrupt preflight, takeoff or tracking, while leaving LAND uninterrupted."""


class MissionSupervisor(Flight):
    ACTIVE = {"starting", "tracking", "landing", "resetting"}

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir).resolve()
        manifest = json.loads((self.run_dir / "manifest.json").read_text())
        if manifest["mode"] != "simulation" or manifest["profile"] != "mpc":
            raise ValueError("Mission supervisor requires the MPC simulation profile")
        self.settings = yaml.safe_load((self.run_dir / "controller.yaml").read_text())
        self.scenario = yaml.safe_load((self.run_dir / "scenario.yaml").read_text())
        uavs = manifest["config"]["uavs"]
        self.names = [u["name"] for u in uavs]
        super().__init__(uavs, timeout=240.0)
        self.mission_state = "idle"
        self.mission_id = ""
        self.error = ""
        self.pending = None
        self.stop_requested = False
        self.cancellable = False
        self.shutdown_requested = False
        self.duration = 0.0
        self.mission_clients = {
            name: self.create_client(typ, name)
            for name, typ in [
                ("/mpc/enable", SetBool),
                ("/target/run", SetBool),
                ("/target/reset", Trigger),
            ]
        }
        self.snapshots = {}
        self.target = {}
        self.delivery = {}
        self.mission_subscriptions = [
            self.create_subscription(
                String,
                "/mpc/status",
                lambda msg: self.snapshots.update(json.loads(msg.data)),
                10,
            ),
            self.create_subscription(
                PoseStamped,
                "/target/ground_truth",
                lambda msg: self.target.update(pose=msg, wall=time.monotonic()),
                10,
            ),
            self.create_subscription(
                String,
                "/mpc/setpoint_status",
                lambda msg: self.delivery.update(json.loads(msg.data), wall=time.monotonic()),
                10,
            ),
        ]
        self.truth = {}
        self.truth_log = None
        for n in self.names:
            self.mission_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/{n}/ground_truth/pose",
                    lambda msg, name=n: self.truth_pose(name, msg),
                    qos_profile_sensor_data,
                )
            )

        self.request_services = [
            self.create_service(StartMission, "/mission/start", self.request_start),
            self.create_service(Trigger, "/mission/stop", self.request_stop),
            self.create_service(Trigger, "/mission/reset", self.request_reset),
            self.create_service(Trigger, "/mission/status", self.request_status),
        ]
        self.stop_timer = self.create_timer(
            0.1, self.check_stop, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def truth_pose(self, name, msg):
        self.truth[name] = msg
        if name == self.names[0] and self.truth_log and all(n in self.truth for n in self.names):
            self.truth_log.write(json.dumps({
                "stamp": seconds(msg.header.stamp),
                "vehicles": {
                    n: {"position": pose_arrays(self.truth[n])[0].tolist()}
                    for n in self.names
                },
                "target": pose_arrays(self.target["pose"])[0].tolist() if self.target else None,
            }) + "\n")

    def service_call(self, name, value=None):
        client = self.mission_clients[name]
        self.wait(client.service_is_ready, f"{name} service", 30.0)
        request = Trigger.Request() if value is None else SetBool.Request(data=value)
        future = client.call_async(request)
        self.wait(future.done, f"{name} reply", 30.0)
        result = future.result()
        if not result or not result.success:
            raise RuntimeError(f"{name}: rejected")

    def stop_flight(self):
        # Helper services and LAND each get independent wall-time budgets.
        # Never wait for publisher/target discovery or replies before LAND.
        errors = []
        pending = {name: None for name in ("/mpc/enable", "/target/run")}
        deadline = time.monotonic() + 2.0

        def poll():
            for name, future in list(pending.items()):
                try:
                    client = self.mission_clients[name]
                    if future is None and time.monotonic() < deadline and client.service_is_ready():
                        future = client.call_async(SetBool.Request(data=False))
                        pending[name] = future
                    if future is not None and future.done():
                        result = future.result()
                        if not result or not result.success:
                            errors.append(f'{name}: rejected')
                        del pending[name]
                    elif time.monotonic() >= deadline:
                        errors.append(f'Timeout: {name} cleanup')
                        if future is not None:
                            future.cancel()
                        del pending[name]
                except Exception as exc:
                    errors.append(f'{name}: {exc}')
                    del pending[name]

        try:
            self.land(self.names, poll=poll)
        except Exception as exc:
            errors.append(str(exc))
        while pending and rclpy.ok():
            poll()
            if pending:
                rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if pending:
            for future in pending.values():
                if future is not None:
                    future.cancel()
            errors.append('ROS stopped during helper cleanup')
        if errors:
            raise RuntimeError("; ".join(errors))

    def status(self):
        return {"state": self.mission_state, "mission_id": self.mission_id,
                "error": self.error, "run_dir": str(self.run_dir),
                "ownership": {n: owner.value for n, owner in self.ownership.items()},
                "controller": self.snapshots}

    def request_status(self, request, response):
        response.success = True
        response.message = json.dumps(self.status())
        return response

    def request_start(self, request, response):
        if Path(request.run_dir).resolve() != self.run_dir:
            response.message = "Request belongs to a different simulation run"
        elif not math.isfinite(request.seconds) or request.seconds < 0:
            response.message = "Duration must be finite and nonnegative"
        elif self.mission_state in self.ACTIVE:
            response.message = "Mission supervisor is busy"
        elif self.mission_state == "failed":
            response.message = "Previous mission failed; request stop to confirm landing before restarting"
        elif any(owner == Ownership.RELEASED for owner in self.ownership.values()) and not all(
            self.connected(n) and not self.states[n].armed for n in self.names
        ):
            response.message = "Pilot/FCU owns flight; confirm fleet disarmed before starting again"
        else:
            # A new start is the only operation allowed to clear takeover,
            # after the disarmed-fleet check above.
            for name, owner in self.ownership.items():
                if owner == Ownership.RELEASED:
                    self.ownership[name] = Ownership.UNOWNED
            self.mission_id = uuid.uuid4().hex
            self.duration = request.seconds
            self.error = ""
            self.stop_requested = False
            self.pending = "start"
            self.mission_state = "starting"
            response.success = True
            response.message = "Mission accepted"
            response.mission_id = self.mission_id
        return response

    def request_stop(self, request, response):
        self.stop_requested = True
        if self.mission_state not in self.ACTIVE:
            self.pending = "stop"
            self.mission_state = "landing"
        response.success = True
        response.message = "Stop requested; cleaning up companion-owned flight"
        return response

    def request_reset(self, request, response):
        if self.mission_state in self.ACTIVE or self.mission_state == "failed":
            response.message = "Stop the mission successfully before resetting the target"
        else:
            self.stop_requested = False
            self.pending = "reset"
            self.mission_state = "resetting"
            response.success = True
        return response

    def check_stop(self):
        if self.cancellable:
            if self.mission_state != "resetting":
                self.check_ownership()
            if self.stop_requested or self.shutdown_requested:
                raise MissionStopped

    def wait(self, condition, description, timeout=None):
        def checked():
            self.check_stop()
            return condition()
        return super().wait(checked, description, timeout)

    def process_pending(self):
        operation, self.pending = self.pending, None
        if operation is None:
            return
        self.error = ""
        result = "completed"
        try:
            if operation == "start":
                self.cancellable = True
                self.check_stop()
                self.truth_log = open(self.run_dir / "truth.jsonl", "a", buffering=1)
                self.prepare_flight()
                self.mission_state = "tracking"
                self.supervise()
            elif operation == "reset":
                self.cancellable = True
                self.check_stop()
                self.service_call("/target/reset")
                result = "idle"
            else:
                result = "stopped"
        except FlightReleased as exc:
            self.error = str(exc)
            result = "released"
        except MissionStopped:
            result = "stopped"
        except Exception as exc:
            self.error = str(exc)
            result = "failed"
        finally:
            self.cancellable = False
            if operation != "reset":
                self.mission_state = "landing"
                try:
                    self.stop_flight()
                except Exception as exc:
                    self.error = "; ".join(filter(None, [self.error, f"Mission cleanup failed: {exc}"]))
                    result = "failed"
                if result != "failed" and Ownership.RELEASED in self.ownership.values():
                    result = "released"
            if self.truth_log:
                self.truth_log.close()
                self.truth_log = None
            self.mission_state = result
            print(json.dumps(self.status()), flush=True)

    def prepare_flight(self):
        # Freeze target and flight publisher during takeoff so velocity references
        # cannot override ArduPilot's takeoff controller.
        self.service_call("/mpc/enable", False)
        self.service_call("/target/reset")

        def fleet_ready():
            try:
                health = json.loads((self.run_dir / "health.json").read_text())
                return (
                    health["ready"] and time.time() - health["updated_wall_unix"] < 10
                )
            except (FileNotFoundError, json.JSONDecodeError, KeyError):
                return False

        self.wait(fleet_ready, "fleet preflight readiness", 240.0)
        self.wait(
            lambda: (
                (self.run_dir / "alignment.json").exists()
                and len(json.loads((self.run_dir / "alignment.json").read_text())) == len(self.names)
            ),
            "common world alignment",
            90.0,
        )
        self.takeoff(self.names, self.settings["flight_altitude"])
        self.wait(
            lambda: (
                self.snapshots.get("reason") == "ready"
                and all(
                    v.get("result") in ("tracking", "search")
                    for v in self.snapshots.get("vehicles", {}).values()
                )
                and len(self.snapshots.get("vehicles", {})) == len(self.names)
            ),
            "MPC with valid sensors and target",
            90.0,
        )
        self.service_call("/mpc/enable", True)
        self.service_call("/target/run", True)

    def supervise(self):
        goal = np.asarray(self.scenario["target"]["waypoints"][-1])[:2]
        begin = time.monotonic()
        target_paused = False
        failure_since = None
        arrival_since = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # Takeover wins over completion and health failures from this tick.
            self.check_stop()
            if (
                self.target
                and np.linalg.norm(pose_arrays(self.target["pose"])[0][:2] - goal) < 0.05
            ):
                positions = [
                    np.asarray(v["position"])[:2]
                    for v in self.snapshots.get("vehicles", {}).values()
                ]
                distances = [np.linalg.norm(p - goal) for p in positions]
                tracking_radius = (
                    self.scenario["viewing_radius"]
                    if self.settings.get("target_mode", "camera") == "ground_truth"
                    else self.settings["flight_altitude"] * np.tan(self.settings["camera_hfov"] / 2)
                )
                if (
                    len(distances) == len(self.names)
                    and max(distances) < self.settings.get("arrival_fleet_radius", 6.0)
                    and min(distances) < tracking_radius
                ):
                    break
                arrival_since = (
                    self.snapshots.get("stamp", 0.0)
                    if arrival_since is None
                    else arrival_since
                )
                if self.snapshots.get("stamp", 0.0) - arrival_since > self.settings.get(
                    "arrival_timeout_sim", 30.0
                ):
                    raise RuntimeError(
                        "Target reached its goal but the fleet did not arrive"
                    )
            if self.duration > 0 and time.monotonic() - begin >= self.duration:
                break
            if (
                time.monotonic() - self.target.get("wall", begin)
                > self.settings["pose_timeout_wall"]
            ):
                raise RuntimeError("Target simulator stopped reporting")
            if (
                time.monotonic() - self.delivery.get("wall", begin)
                > self.settings["command_timeout_wall"]
            ):
                raise RuntimeError("Independent setpoint publisher stopped reporting")
            if (
                time.time() - self.snapshots.get("wall", 0.0)
                > self.settings["command_timeout_wall"] + 1.0
            ):
                raise RuntimeError("Controller status stopped updating")
            if self.snapshots.get("reason") == "ready":
                failure_since = None
            else:
                failure_since = (
                    self.snapshots.get("stamp", 0.0)
                    if failure_since is None
                    else failure_since
                )
                if self.snapshots.get("stamp", 0.0) - failure_since > self.settings.get(
                    "planner_failure_timeout_sim", 3.0
                ):
                    raise RuntimeError("Planner inputs or solves remained unavailable")
            age = self.snapshots.get("target_observation_age")
            if age is not None and self.settings.get("target_mode", "camera") == "ground_truth":
                if age > self.settings["target_timeout_sim"]:
                    raise RuntimeError("External target position stopped updating")
            elif age is not None:
                # Pause during camera occlusion while UAVs SEARCH from their
                # last observation. Exact external input has no reacquisition.
                lost = age > self.settings["target_timeout_sim"]
                if lost != target_paused:
                    self.service_call("/target/run", not lost)
                    target_paused = lost
                if age > self.settings["target_search_timeout_sim"]:
                    raise RuntimeError(
                        "Target could not be reacquired within SEARCH timeout"
                    )
            if not all(
                self.connected(n)
                and self.states[n].armed
                and self.states[n].mode == "GUIDED"
                for n in self.names
            ):
                raise RuntimeError("Mission lost armed/connection state")
            if any(
                self.poses[n].pose.position.z < self.settings["flight_altitude"] - 0.7
                for n in self.names
            ):
                raise RuntimeError("Vehicle left the fixed-altitude flight envelope")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--startup-component")
    options, ros = parser.parse_known_args(args)
    progress = (
        ComponentProgress(options.run_dir, options.startup_component)
        if options.startup_component else None
    )
    previous_signals = {}
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        if progress:
            progress.update("initializing", "Constructing mission supervisor")
        node = MissionSupervisor(options.run_dir)
        if progress:
            progress.update("initialized", "Mission request services ready")
        def shutdown(signum, frame):
            node.shutdown_requested = True

        previous_signals = {
            sig: signal.signal(sig, shutdown) for sig in (signal.SIGINT, signal.SIGTERM)
        }
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.process_pending()
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        if progress:
            progress.update("failed", str(exc))
        raise
    finally:
        if node is not None:
            node.cancellable = False
            if node.flight_owned:
                try:
                    node.stop_flight()
                except Exception as exc:
                    print(f"Mission cleanup failed: {exc}", flush=True)
            if node.truth_log:
                node.truth_log.close()
            node.destroy_node()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
