"""Run analysis finds stops away from the target and attributes them."""

import importlib.util
import json
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("analyze_run", ROOT / "scripts/analyze_run.py")
analyze_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyze_run)


def write_run(run):
    pkg = ROOT / "ros_ws/src/solo_mpc"
    from solo_mpc.settings import load_settings

    scenario, settings, cfg = load_settings(pkg / "scenarios/square50_1uav.yaml",
                                            pkg.parents[2] / "config/drone.yaml")
    (run / "scenario.yaml").write_text((pkg / "scenarios/square50_1uav.yaml").read_text())
    (run / "controller.yaml").write_text(yaml.safe_dump(settings))
    (run / "mpc-config.json").write_text(json.dumps(cfg))
    records, truth = [], []
    x, t = 1.0, 0.0
    for k in range(200):  # 20 s: cruise, a 3 s halt at x~8, cruise, then hold at the target
        t = k * 0.1
        halted = 6.0 <= t < 9.0 or x >= 12.0
        v = 0.0 if halted else 0.7
        x = min(12.0, x + v * 0.1)
        records.append({"stamp": t, "target_estimate": [12.0, 1.0, 0, 0], "vehicles": {"uav1": {
            "position": [x, 1.0, 2.0], "velocity": [v, 0.0, 0.0], "armed": True,
            "result": "stop_plan: mpc_failed" if 6.0 <= t < 9.0 else "search",
            "solver_ms": 40.0}}})
        truth.append({"stamp": t, "vehicles": {"uav1": {"position": [x, 1.0, 2.0],
                                                        "yaw": math.remainder(0.5 * t, 2 * math.pi)}}})
    (run / "mpc.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
    (run / "truth.jsonl").write_text("".join(json.dumps(r) + "\n" for r in truth))
    (run / "setpoints.jsonl").write_text("".join(json.dumps(r) + "\n" for r in [
        {"stamp": 0.0, "vehicle": "uav1", "reason": "tracking"},
        {"stamp": 6.0, "vehicle": "uav1", "reason": "command_limit"},
        {"stamp": 9.0, "vehicle": "uav1", "reason": "tracking"}]))
    return cfg


def test_reports_mid_route_stop_but_not_arrival(tmp_path):
    cfg = write_run(tmp_path)
    vehicle = analyze_run.analyze(tmp_path)["vehicles"]["uav1"]
    (stop,) = vehicle["stops_away_from_target"]
    assert stop["start_s"] == 6.0 and stop["duration_s"] == 3.0
    assert stop["controller_result"] == "stop_plan: mpc_failed"
    assert stop["setpoint_reason"] == "command_limit"
    assert vehicle["setpoint_time_s"]["command_limit"] == 3.0
    assert vehicle["speed_mps"]["max"] == 0.7 == cfg["VMAX"]
    # 0.5 rad/s for 19.9 s, across several wraps of +-pi.
    assert abs(vehicle["yaw"]["total_rotation_deg"] - math.degrees(0.5 * 19.9)) < 1
    assert vehicle["min_body_clearance_m"] > 0 and vehicle["body_contact_samples"] == 0
    assert vehicle["backing_episodes"] == 0 and vehicle["over_vmax_fraction"] == 0


def test_counts_backing_away_from_target(tmp_path):
    write_run(tmp_path)
    rows = [json.loads(l) for l in (tmp_path / "mpc.jsonl").read_text().splitlines()]
    for row in rows[20:25]:  # 0.5 s moving back toward the start at 0.4 m/s
        row["vehicles"]["uav1"]["velocity"] = [-0.4, 0.0, 0.0]
    for row in rows[40:43]:  # 0.3 s overshoot past VMAX, recorded with the command
        row["vehicles"]["uav1"]["velocity"] = [1.0, 0.0, 0.0]
        row["vehicles"]["uav1"]["command_velocity"] = [0.7, 0.0, 0.0]
    (tmp_path / "mpc.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    vehicle = analyze_run.analyze(tmp_path)["vehicles"]["uav1"]
    assert vehicle["backing_episodes"] == 1 and vehicle["backing_starts_s"] == [2.0]
    assert vehicle["backing_after"] == {"nominal solve": 1}
    assert vehicle["over_vmax_fraction"] == 3 / 200
    assert abs(vehicle["command_tracking_error_mps"] - 0.3) < 1e-9


def test_backing_after_path_flip_is_attributed(tmp_path):
    write_run(tmp_path)
    rows = [json.loads(l) for l in (tmp_path / "mpc.jsonl").read_text().splitlines()]
    for k, row in enumerate(rows):
        row["vehicles"]["uav1"]["path_heading"] = 0.0 if k < 18 else 3.0  # flips at 1.8 s
    for row in rows[20:25]:
        row["vehicles"]["uav1"]["velocity"] = [-0.4, 0.0, 0.0]
    (tmp_path / "mpc.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    vehicle = analyze_run.analyze(tmp_path)["vehicles"]["uav1"]
    assert vehicle["path_flips"] == 1
    assert vehicle["backing_after"] == {"path flip": 1}


def test_cpu_timeouts_report_cold_starts(tmp_path):
    write_run(tmp_path)
    rows = [json.loads(l) for l in (tmp_path / "mpc.jsonl").read_text().splitlines()]
    for k in (100, 101, 150):
        rows[k]["vehicles"]["uav1"].update(result="mpc_failed", solver_status="Maximum_CpuTime_Exceeded")
    for row in rows:
        row["vehicles"]["uav1"]["corridor_used_faces"] = 5
    (tmp_path / "mpc.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    vehicle = analyze_run.analyze(tmp_path)["vehicles"]["uav1"]
    assert vehicle["cpu_timeouts"]["count"] == 3
    assert vehicle["cpu_timeouts"]["right_after_failure"] == 1  # 101 follows 100
    assert vehicle["corridor_used_faces"]["max"] == 5
