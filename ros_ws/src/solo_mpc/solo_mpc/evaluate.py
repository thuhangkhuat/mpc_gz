"""Evaluate recorded measured-state flight, independent of controller predictions."""

import argparse
import json
from pathlib import Path

import numpy as np


def clearance(point, scenario):
    p = np.asarray(point)[:2]
    distances = []
    for x, y, w, h in scenario["obstacles"]["rects"]:
        distances.append(float(np.linalg.norm(p - np.clip(p, [x, y], [x + w, y + h]))))
    for x, y, r in scenario["obstacles"]["circles"]:
        distances.append(float(np.linalg.norm(p - [x, y]) - r))
    return min(distances) if distances else float("inf")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", default="/workspace/runs/latest")
    args = parser.parse_args()
    run = Path(args.run_dir)
    import yaml

    scenario = yaml.safe_load((run / "scenario.yaml").read_text())
    cfg = json.loads((run / "mpc-config.json").read_text())
    records = [
        json.loads(line) for line in (run / "mpc.jsonl").read_text().splitlines()
    ]
    airborne = [
        r
        for r in records
        if r["vehicles"]
        and all(v["position"][2] > 2.5 for v in r["vehicles"].values())
        and all(
            v.get("armed", True) and v.get("mode", "GUIDED") == "GUIDED"
            for v in r["vehicles"].values()
        )
    ]
    if not airborne:
        raise SystemExit("No complete airborne fleet records")
    computation = np.asarray([r["compute_ms"] for r in airborne])
    separation = []
    margins = []
    target_distances = []
    truth_path = run / "truth.jsonl"
    truth = (
        [json.loads(line) for line in truth_path.read_text().splitlines()]
        if truth_path.exists()
        else []
    )
    geometry = [
        r
        for r in truth
        if all(v["position"][2] > 2.5 for v in r["vehicles"].values())
        and airborne[0]["stamp"] <= r["stamp"] <= airborne[-1]["stamp"]
    ] or airborne
    for row in geometry:
        positions = [v["position"] for v in row["vehicles"].values()]
        if row.get("target") is not None:
            target = np.asarray(row["target"])[:2]
            target_distances.append(
                min(np.linalg.norm(np.asarray(p)[:2] - target) for p in positions)
            )
        for i, p in enumerate(positions):
            margins.append(clearance(p, scenario) - cfg["ROBOT_RADIUS"])
            separation.extend(
                np.linalg.norm(np.asarray(p) - q) for q in positions[i + 1 :]
            )
    output = {
        "samples": len(airborne),
        "target_mode": airborne[-1]["target_mode"],
        "geometry_source": "gazebo_ground_truth"
        if geometry is not airborne
        else "mavros_measurements",
        "minimum_obstacle_body_clearance_m": float(min(margins)),
        "minimum_inter_uav_distance_m": float(min(separation)) if separation else None,
        "compute_p50_p95_p99_max_ms": np.percentile(
            computation, [50, 95, 99, 100]
        ).tolist(),
        "over_100ms_fraction": float(np.mean(computation > 100.0)),
        "controller_ready_fraction": float(
            np.mean([r["reason"] == "ready" for r in airborne])
        ),
        "nearest_uav_target_distance_p50_p95_max_m": (
            np.percentile(target_distances, [50, 95, 100]).tolist()
            if target_distances
            else None
        ),
        "simulated_seconds_per_wall_second": (
            (airborne[-1]["stamp"] - airborne[0]["stamp"])
            / max(airborne[-1]["wall"] - airborne[0]["wall"], 1e-9)
        ),
        "final_uav_xy": {
            n: v["position"][:2] for n, v in geometry[-1]["vehicles"].items()
        },
    }
    target_truth = [r for r in truth if r.get("target") is not None]
    if target_truth:
        times = [r["stamp"] for r in target_truth]
        positions = np.asarray([r["target"] for r in target_truth])
        errors = [
            np.linalg.norm(
                np.asarray(r["target_estimate"])[:2]
                - [
                    np.interp(r["stamp"], times, positions[:, axis])
                    for axis in range(2)
                ]
            )
            for r in airborne
            if r["target_estimate"] is not None
        ]
        output["target_estimation_error_p50_p95_max_m"] = (
            np.percentile(errors, [50, 95, 100]).tolist() if errors else None
        )
    (run / "evaluation.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))
