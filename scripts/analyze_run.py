"""Summarize one MPC simulation run: stops, obstacle clearance, yaw, solver timing.

Reads <run>/mpc.jsonl, truth.jsonl, setpoints.jsonl, scenario.yaml, mpc-config.json
and live-obstacles.yaml; writes <run>/analysis.json and prints a readable report.
Runs inside the container (./scripts/dev analyze) or on the host:
  python3 scripts/analyze_run.py --run-dir runs/<run-id>
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "ros_ws/src/solo_mpc"), str(ROOT / "ros_ws/src/solo_bringup")]

STOP_SPEED = 0.1   # m/s: slower than this counts as stopped
MIN_STOP = 1.0     # s: shorter pauses are ignored
BACK_SPEED = 0.1   # m/s: moving away from the target faster than this counts as backing
MIN_BACK = 0.3     # s: shorter reversals are ignored


def lines(path):
    try:
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    except FileNotFoundError:
        return []


def obstacles(run):
    from solo_mpc.scene_edit import load

    scenario = yaml.safe_load((run / "scenario.yaml").read_text())
    shapes = [("box", x + w / 2, y + h / 2, [w, h]) for x, y, w, h in scenario["obstacles"]["rects"]]
    shapes += [("circle", x, y, [r]) for x, y, r in scenario["obstacles"]["circles"]]
    shapes += [(d["shape"], d["x"], d["y"], d["size"]) for d in load(run).values()]
    return shapes


def nearest(shapes, point):
    from solo_mpc.scene_edit import clearance

    return min((clearance(s, x, y, size, point) for s, x, y, size in shapes), default=math.inf)


def percentile(values, q):
    return float(np.percentile(values, q)) if values else None


def setpoint_time(transitions, name, end):
    rows = [t for t in transitions if t["vehicle"] == name]
    spent = Counter()
    for a, b in zip(rows, rows[1:] + [{"stamp": end}]):
        spent[a["reason"]] += max(0.0, b["stamp"] - a["stamp"])
    return rows, spent


def reason_at(rows, stamp):
    current = None
    for row in rows:
        if row["stamp"] > stamp:
            break
        current = row["reason"]
    return current


def overlap_report(rows, names, K, width, height, plane_z, period=0.2):
    """Shared ground area of the camera footprints on the target plane, sampled every period s."""
    from itertools import combinations

    from solo_mpc import fov

    samples, last = [], -math.inf
    for t in rows:
        if t["stamp"] - last >= period:
            samples.append(t)
            last = t["stamp"]
    pairs = {f"{a}-{b}": {"area": [], "iou": []} for a, b in combinations(names, 2)}
    singles, unions, sums, skipped = [], [], [], 0
    for t in samples:
        prints = {}
        for n in names:
            v = t["vehicles"][n]
            prints[n] = fov.footprint(v["position"], v.get("orientation") or fov.level(v["yaw"]),
                                      K, width, height, plane_z)
        if any(p is None for p in prints.values()):
            skipped += 1
            continue
        areas = {n: fov.area(p) for n, p in prints.items()}
        for a, b in combinations(names, 2):
            shared = fov.intersection(prints[a], prints[b])
            inter = fov.area(shared) if shared is not None else 0.0
            pairs[f"{a}-{b}"]["area"].append(inter)
            pairs[f"{a}-{b}"]["iou"].append(inter / (areas[a] + areas[b] - inter))
        singles.extend(areas.values())
        unions.append(fov.union_area(list(prints.values())))
        sums.append(sum(areas.values()))
    if not unions:
        return None
    return {
        "samples": len(unions), "skipped_tilted": skipped,
        "footprint_m2": float(np.mean(singles)),
        "pairs": {k: {"overlapping_fraction": float(np.mean(np.array(v["area"]) > 1e-6)),
                      "mean_m2": float(np.mean(v["area"])), "max_m2": float(np.max(v["area"])),
                      "mean_iou": float(np.mean(v["iou"]))} for k, v in pairs.items()},
        "union_m2": float(np.mean(unions)), "summed_m2": float(np.mean(sums)),
        "shared_fraction": float(1 - np.sum(unions) / np.sum(sums)),
    }


def fov_report(run, settings, cfg, truth, names):
    """Camera FOV coverage from ground truth, and detector recall/accuracy.

    The target is a disc (tag circumcircle, or the marker) on its plane; it is
    fully in view when the whole disc projects inside the image.
    """
    from solo_mpc import fov

    rows = [t for t in truth if t.get("target") is not None]
    if not rows:
        return None
    robot = settings.get("ground_robot")
    if robot is not None:
        from solo_mpc.ground_robot import tag_height
        radius, height, label = robot["tag"]["size"] * math.sqrt(2) / 2, tag_height(robot), "AprilTag"
    else:
        radius, height, label = cfg.get("TAR_RADIUS", 0.15) * math.sqrt(2), settings["target_height"], "marker"
    width, hfov = settings["camera_width"], settings["camera_hfov"]
    K = fov.intrinsics(width, settings["camera_height"], hfov)
    stamps = np.array([t["stamp"] for t in rows])
    attitude = "truth attitude" if all("orientation" in rows[0]["vehicles"][n] for n in names) \
        else "level approximation (log has yaw only)"
    out = {"target": label, "disc_radius_m": radius, "camera": [width, settings["camera_height"],
           round(math.degrees(hfov), 1)], "attitude": attitude, "vehicles": {}}
    fully_all = []
    detected_any = np.zeros(len(rows), bool)
    for n in names:
        views = []
        for t in rows:
            v = t["vehicles"][n]
            q = v.get("orientation") or fov.level(v["yaw"])
            views.append(fov.view([*t["target"][:2], height], radius, v["position"], q, K,
                                  width, settings["camera_height"]))
        centre = np.array([w["centre_in"] for w in views])
        fully = np.array([w["fully_in"] for w in views])
        offsets = [w["offset"] for w in views if w["offset"] is not None]
        fully_all.append(fully)
        entry = {"in_view_fraction": float(centre.mean()), "fully_in_view_fraction": float(fully.mean()),
                 "centre_offset": {"mean": float(np.mean(offsets)) if offsets else None,
                                   "p95": percentile(offsets, 95)},
                 "longest_out_of_view_s": fov.gaps(list(stamps), list(fully)), "detection": None}
        frames = [f for f in lines(run / f"detections_{n}.jsonl")
                  if stamps[0] <= f["stamp"] <= stamps[-1]]
        if frames:
            index = np.clip(np.searchsorted(stamps, [f["stamp"] for f in frames]), 0, len(stamps) - 1)
            hit = np.array([f["detected"] for f in frames])
            visible, present = fully[index], centre[index]
            # Accuracy of true detections; false ones are counted separately.
            errors = [math.dist(f["observed"], rows[k]["target"][:2])
                      for f, k in zip(frames, index) if f["observed"] is not None and centre[k]]
            entry["detection"] = {
                "frames": len(frames),
                "recall": float(hit[visible].mean()) if visible.any() else None,
                "false_detections": int((hit & ~present).sum()),
                "error_m": {"mean": float(np.mean(errors)) if errors else None, "p95": percentile(errors, 95)},
            }
            # A detection keeps the team "seeing" the target for the next 0.2 s.
            hits = np.array([f["stamp"] for f in frames if f["detected"]])
            if len(hits):
                marks = np.zeros(len(stamps) + 1)
                np.add.at(marks, np.searchsorted(stamps, hits, "left"), 1)
                np.add.at(marks, np.searchsorted(stamps, hits + 0.2, "right"), -1)
                detected_any |= np.cumsum(marks)[:-1] > 0
        out["vehicles"][n] = entry
    out["team"] = fov.team_coverage(stamps, fully_all)
    if len(names) > 1:
        out["overlap"] = overlap_report(rows, names, K, width, settings["camera_height"], height)
    if any(out["vehicles"][n]["detection"] for n in names):
        out["team"]["detected_by_any_fraction"] = float(detected_any.mean())
        out["team"]["longest_undetected_s"] = fov.gaps(list(stamps), list(detected_any))
    return out


def analyze(run):
    cfg = json.loads((run / "mpc-config.json").read_text())
    settings = yaml.safe_load((run / "controller.yaml").read_text())
    records = [r for r in lines(run / "mpc.jsonl") if r.get("vehicles")]
    truth = lines(run / "truth.jsonl")
    transitions = lines(run / "setpoints.jsonl")
    if not records:
        raise SystemExit(f"No controller records in {run}/mpc.jsonl; fly a mission first")
    shapes = obstacles(run)
    period = settings["control_period"]
    flying = [r for r in records if any(v.get("armed") for v in r["vehicles"].values())] or records
    end = flying[-1]["stamp"]
    report = {"run": str(run), "duration_s": end - flying[0]["stamp"], "vehicles": {}}
    for name in flying[0]["vehicles"]:
        rows = [(r["stamp"], r["vehicles"][name], r) for r in flying if name in r["vehicles"]]
        speed = np.array([np.linalg.norm(v["velocity"][:2]) for _, v, _ in rows])
        stamps = np.array([t for t, _, _ in rows])
        results = Counter(v["result"] for _, v, _ in rows)
        solver = [v["solver_ms"] for _, v, _ in rows if v.get("solver_ms") is not None]
        sp_rows, sp_time = setpoint_time(transitions, name, end)
        target = [r.get("target_estimate") for _, _, r in rows]
        statuses = Counter(v["solver_status"] for _, v, _ in rows if v.get("solver_status"))
        iterations = [v["solver_iterations"] for _, v, _ in rows if v.get("solver_iterations") is not None]
        faces = [v["corridor_used_faces"] for _, v, _ in rows if v.get("corridor_used_faces") is not None]
        timeouts = [k for k, (_, v, _) in enumerate(rows) if v.get("solver_status") == "Maximum_CpuTime_Exceeded"]
        ok = {"search", "tracking"}
        after_failure = sum(k > 0 and rows[k - 1][1]["result"] not in ok for k in timeouts)
        near = [nearest(shapes, rows[k][1]["position"][:2]) for k in timeouts]
        overspeed = sum(speed[k] > cfg["VMAX"] for k in timeouts)
        # Grid-path bearing turning > 90 deg within 1 s: the planner switched sides.
        headings = [(t, v["path_heading"]) for t, v, _ in rows if v.get("path_heading") is not None]
        flips = []
        for k, (t, h) in enumerate(headings):
            for t0, h0 in reversed(headings[max(0, k - 20):k]):
                if t - t0 > 1.0:
                    break
                if abs(math.remainder(h - h0, 2 * math.pi)) > math.pi / 2:
                    if not flips or t - flips[-1] > 1.0:
                        flips.append(t)
                    break
        command = [(np.linalg.norm(v["command_velocity"][:2]), s) for (_, v, _), s
                   in zip(rows, speed) if v.get("command_velocity") is not None]
        # Velocity component toward the target while still away from it.
        toward = np.array([np.nan if goal is None or math.dist(v["position"][:2], goal[:2])
                           <= cfg["VIEWING_RADIUS"] else float(np.dot(
                               v["velocity"][:2], np.subtract(goal[:2], v["position"][:2])
                               / math.dist(v["position"][:2], goal[:2])))
                           for (_, v, _), goal in zip(rows, target)])
        backing, backing_causes, begin = [], Counter(), None
        for i, value in enumerate(np.r_[toward, np.nan]):
            if value < -BACK_SPEED and begin is None:
                begin = i
            elif not value < -BACK_SPEED and begin is not None:
                if stamps[min(i, len(stamps) - 1)] - stamps[begin] >= MIN_BACK:
                    backing.append(round(float(stamps[begin] - flying[0]["stamp"]), 1))
                    # What the controller did in the second before the reversal began.
                    window = [rows[k][1]["result"] for k in range(len(rows))
                              if stamps[begin] - 1.0 <= stamps[k] <= stamps[begin]]
                    failed = [r for r in window if r not in ("search", "tracking")]
                    flipped = any(stamps[begin] - 1.0 <= f <= stamps[begin] + 0.2 for f in flips)
                    backing_causes[failed[0] if failed else
                                   "path flip" if flipped else "nominal solve"] += 1
                begin = None

        stops, start = [], None
        for i, s in enumerate(np.r_[speed, math.inf]):
            if s < STOP_SPEED and start is None:
                start = i
            elif s >= STOP_SPEED and start is not None:
                if stamps[min(i, len(stamps) - 1)] - stamps[start] >= MIN_STOP:
                    t0, v0, _ = rows[start]
                    p = v0["position"][:2]
                    goal = target[start]
                    window = [rows[k][1]["result"] for k in range(start, min(i, len(rows)))]
                    stops.append({
                        "start_s": round(t0 - flying[0]["stamp"], 2),
                        "duration_s": round(stamps[min(i, len(stamps) - 1)] - t0, 2),
                        "position": [round(c, 2) for c in p],
                        "nearest_obstacle_surface_m": round(nearest(shapes, p), 2),
                        "target_distance_m": None if goal is None else round(math.dist(p, goal[:2]), 2),
                        "controller_result": Counter(window).most_common(1)[0][0],
                        "setpoint_reason": reason_at(sp_rows, t0),
                    })
                start = None
        # A stop at the target is arrival, not a fault.
        stops = [s for s in stops if s["target_distance_m"] is None
                 or s["target_distance_m"] > cfg["VIEWING_RADIUS"]]

        body = [(t["stamp"], nearest(shapes, t["vehicles"][name]["position"][:2]) - cfg["ROBOT_RADIUS"])
                for t in truth if name in t["vehicles"]]
        yaws = np.unwrap([t["vehicles"][name]["yaw"] for t in truth
                          if "yaw" in t["vehicles"].get(name, {})])
        ytimes = [t["stamp"] for t in truth if "yaw" in t["vehicles"].get(name, {})]
        report["vehicles"][name] = {
            "speed_mps": {"mean_moving": float(speed[speed >= STOP_SPEED].mean()) if (speed >= STOP_SPEED).any() else 0.0,
                          "max": float(speed.max()), "vmax": cfg["VMAX"]},
            "stopped_fraction": float((speed < STOP_SPEED).mean()),
            "over_vmax_fraction": float((speed > 1.05 * cfg["VMAX"]).mean()),
            "command_tracking_error_mps": None if not command else float(np.mean([abs(a - b) for a, b in command])),
            "backing_episodes": len(backing),
            "backing_starts_s": backing[:50],
            "backing_after": dict(backing_causes.most_common()),
            "solver_status": dict(statuses.most_common()),
            "solver_iterations": {"p50": percentile(iterations, 50), "p95": percentile(iterations, 95),
                                  "max": max(iterations, default=None)},
            "path_flips": len(flips),
            "corridor_used_faces": {"p50": percentile(faces, 50), "p95": percentile(faces, 95),
                                    "max": max(faces, default=None)},
            "cpu_timeouts": {"count": len(timeouts), "right_after_failure": after_failure,
                             "measured_over_vmax": int(overspeed),
                             "median_obstacle_m": percentile(near, 50),
                             "within_1m_of_obstacle": sum(d < 1.0 for d in near)},
            "stops_away_from_target": stops,
            "min_body_clearance_m": min((c for _, c in body), default=None),
            "body_contact_samples": sum(c <= 0 for _, c in body),
            "controller_results": dict(results.most_common()),
            "setpoint_time_s": {k: round(v, 2) for k, v in sp_time.most_common()},
            "solver_ms": {"p50": percentile(solver, 50), "p95": percentile(solver, 95),
                          "max": max(solver, default=None),
                          "over_period": sum(s > 1000 * period for s in solver)},
            "yaw": None if len(yaws) < 2 else {
                "total_rotation_deg": float(np.degrees(np.abs(np.diff(yaws)).sum())),
                "net_change_deg": float(np.degrees(yaws[-1] - yaws[0])),
                "mean_rate_dps": float(np.degrees(np.abs(np.diff(yaws)).sum()) / max(ytimes[-1] - ytimes[0], 1e-9)),
            },
        }
    report["fov"] = fov_report(run, settings, cfg, truth, list(report["vehicles"]))
    return report


def show_fov(f):
    if f is None:
        print("\n== FOV: no target in truth.jsonl")
        return
    w, h, deg = f["camera"]
    print(f"\n== FOV ({f['target']} disc r={f['disc_radius_m']:.2f} m; camera {w}x{h}, {deg} deg; "
          f"{f['attitude']})")
    for name, v in f["vehicles"].items():
        c = v["centre_offset"]
        line = (f"{name}: tag in view {100 * v['in_view_fraction']:.0f}% "
                f"(fully {100 * v['fully_in_view_fraction']:.0f}%), longest out of view "
                f"{v['longest_out_of_view_s']:.1f} s")
        if c["mean"] is not None:
            line += f", offset from image centre mean {c['mean']:.2f} p95 {c['p95']:.2f} (1 = edge)"
        print(line)
        d = v["detection"]
        if d is None:
            print("  detection: no detections_<uav>.jsonl (older run or ground_truth mode)")
        else:
            recall = "n/a" if d["recall"] is None else f"{100 * d['recall']:.0f}%"
            error = "" if d["error_m"]["mean"] is None else \
                f", position error mean {d['error_m']['mean']:.2f} p95 {d['error_m']['p95']:.2f} m"
            print(f"  detection: recall {recall} of fully-visible frames, "
                  f"{d['false_detections']} false detections, {d['frames']} frames{error}")
    t = f["team"]
    if len(f["vehicles"]) > 1 or "detected_by_any_fraction" in t:
        line = (f"team: >= 1 UAV sees the tag {100 * t['any_in_view_fraction']:.0f}% of the time, "
                f"mean {t['mean_viewers']:.1f} viewers, longest unseen {t['longest_unseen_s']:.1f} s")
        if "detected_by_any_fraction" in t:
            line += (f"; detected by any {100 * t['detected_by_any_fraction']:.0f}%, "
                     f"longest without detection {t['longest_undetected_s']:.1f} s")
        print(line)
    if "overlap" in f:
        show_overlap(f["overlap"])


def show_overlap(o):
    if o is None:
        print("FOV overlap: no sample with every footprint on the ground plane")
        return
    print(f"FOV overlap on the target plane ({o['samples']} samples at 5 Hz; footprint "
          f"{o['footprint_m2']:.1f} m2 each" + (f", {o['skipped_tilted']} tilted samples skipped"
                                                 if o["skipped_tilted"] else "") + "):")
    for pair, v in o["pairs"].items():
        print(f"  {pair}: overlapping {100 * v['overlapping_fraction']:.0f}% of the time, "
              f"mean {v['mean_m2']:.2f} m2 (IoU {v['mean_iou']:.2f}), max {v['max_m2']:.2f} m2")
    print(f"  team: covers {o['union_m2']:.1f} m2 on average of {o['summed_m2']:.1f} m2 summed; "
          f"{100 * o['shared_fraction']:.0f}% of the summed footprint area duplicates another camera")


def show(report):
    print(f"Run {report['run']}  ({report['duration_s']:.0f} s of controller records)")
    for name, v in report["vehicles"].items():
        print(f"\n== {name}")
        s = v["speed_mps"]
        print(f"speed: mean while moving {s['mean_moving']:.2f} m/s, max {s['max']:.2f} (VMAX {s['vmax']}); "
              f"stopped {100 * v['stopped_fraction']:.0f}% of cycles")
        if v["min_body_clearance_m"] is not None:
            print(f"obstacle body clearance: min {v['min_body_clearance_m']:.2f} m; "
                  f"{v['body_contact_samples']} samples at or inside an obstacle")
        print(f"over 1.05 x VMAX {100 * v['over_vmax_fraction']:.0f}% of cycles; "
              + ("mean |commanded - measured speed| "
                 f"{v['command_tracking_error_mps']:.2f} m/s" if v["command_tracking_error_mps"] is not None
                 else "no command_velocity logged (older run)"))
        print(f"backing away from target (> {BACK_SPEED} m/s for {MIN_BACK} s): {v['backing_episodes']} times"
              + (f", first at {v['backing_starts_s'][:8]} s" if v["backing_starts_s"] else ""))
        if v["backing_after"]:
            print("  preceded within 1 s by:", ", ".join(f"{k} {n}" for k, n in v["backing_after"].items()))
        print("controller results:", ", ".join(f"{k} {n}" for k, n in v["controller_results"].items()))
        if v["solver_status"]:
            print("solver status:", ", ".join(f"{k} {n}" for k, n in v["solver_status"].items()))
        it = v["solver_iterations"]
        if it["p50"] is not None:
            print(f"solver iterations: p50 {it['p50']:.0f}, p95 {it['p95']:.0f}, max {it['max']}")
        print(f"grid path flips (> 90 deg within 1 s): {v['path_flips']}")
        f = v["corridor_used_faces"]
        if f["p50"] is not None:
            print(f"corridor faces used: p50 {f['p50']:.0f}, p95 {f['p95']:.0f}, max {f['max']}")
        c = v["cpu_timeouts"]
        if c["count"]:
            print(f"CPU timeouts: {c['count']}, {c['right_after_failure']} right after a failed cycle, "
                  f"{c['measured_over_vmax']} with measured speed above VMAX; "
                  f"median obstacle {c['median_obstacle_m']:.2f} m, {c['within_1m_of_obstacle']} within 1 m")
        if v["setpoint_time_s"]:
            print("setpoint time (s):", ", ".join(f"{k} {t}" for k, t in v["setpoint_time_s"].items()))
        else:
            print("setpoint time: no setpoints.jsonl (run recorded before setpoint logging)")
        sm = v["solver_ms"]
        if sm["p50"] is not None:
            print(f"solver ms: p50 {sm['p50']:.0f}, p95 {sm['p95']:.0f}, max {sm['max']:.0f}; "
                  f"{sm['over_period']} solves longer than the control period")
        if v["yaw"]:
            y = v["yaw"]
            print(f"yaw: turned {y['total_rotation_deg']:.0f} deg in total (net {y['net_change_deg']:.0f}), "
                  f"mean {y['mean_rate_dps']:.1f} deg/s")
        stops = v["stops_away_from_target"]
        print(f"stops away from target (> {MIN_STOP:g} s below {STOP_SPEED} m/s): {len(stops)}")
        for stop in stops[:15]:
            print(f"  t={stop['start_s']:7.1f}s for {stop['duration_s']:5.1f}s at {stop['position']}: "
                  f"obstacle {stop['nearest_obstacle_surface_m']} m, target {stop['target_distance_m']} m, "
                  f"controller {stop['controller_result']}, setpoint {stop['setpoint_reason']}")
        if len(stops) > 15:
            print(f"  ... {len(stops) - 15} more in analysis.json")
    if "fov" in report:
        show_fov(report["fov"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", default="/workspace/runs/latest")
    args = parser.parse_args(argv)
    run = Path(args.run_dir).resolve()
    report = analyze(run)
    (run / "analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    show(report)
    print(f"\nWrote {run / 'analysis.json'}")


if __name__ == "__main__":
    main()
