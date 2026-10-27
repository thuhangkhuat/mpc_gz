"""Camera FOV geometry and the FOV section of the run analysis."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from solo_mpc import fov
from solo_mpc.core.sensors import pixel_to_ground

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("analyze_run", ROOT / "scripts/analyze_run.py")
analyze_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyze_run)


@pytest.mark.parametrize("roll,pitch,yaw", [(0, 0, 90), (6, -4, 30), (-10, 8, -120)])
@pytest.mark.parametrize("pixel", [(320, 240), (10, 15), (630, 470)])
def test_projection_inverts_the_detector_ground_projection(roll, pitch, yaw, pixel):
    K = fov.intrinsics(640, 480, math.pi / 2)
    q = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_quat()
    body = np.array([5.0, 7.0, 2.08])
    camera = body + Rotation.from_quat(q).apply(fov.CAMERA_OFFSET)
    ground = pixel_to_ground(pixel, K, camera, q, 0.25)
    u, v, depth = fov.project(ground, body, q, K)
    assert (u, v) == pytest.approx(pixel, abs=1e-6) and depth > 0


def test_view_centre_edge_and_outside():
    K = fov.intrinsics(640, 480, math.pi / 2)
    level, body = fov.level(0.0), [0.0, 0.0, 2.0]
    below = fov.view([0.0, 0.0, 0.25], 0.2, body, level, K, 640, 480)
    assert below["fully_in"] and below["offset"] == pytest.approx(0.0, abs=0.01)
    # Image width spans +-(2.0 + 0.04 - 0.25) m at the tag plane along body Y.
    half = 2.0 - 0.04 - 0.25
    edge = fov.view([0.0, half - 0.1, 0.25], 0.2, body, level, K, 640, 480)
    assert edge["centre_in"] and not edge["fully_in"] and edge["offset"] > 0.9
    assert not fov.view([0.0, half + 0.5, 0.25], 0.2, body, level, K, 640, 480)["centre_in"]
    assert fov.project([0, 0, 3.0], body, level, K) is None  # above the camera


def test_gaps_and_team_coverage():
    stamps = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
    assert fov.gaps(stamps, [True, False, False, True, False, False]) == pytest.approx(1.0)
    team = fov.team_coverage(stamps, [[True, False, False, True, True, True],
                                      [False, False, True, True, False, True]])
    assert team["any_in_view_fraction"] == pytest.approx(5 / 6)
    assert team["mean_viewers"] == pytest.approx(7 / 6)
    assert team["longest_unseen_s"] == pytest.approx(0.5)


def test_analysis_reports_per_uav_fov_recall_and_team(tmp_path):
    from solo_mpc.settings import load_settings

    scenario_path = ROOT / "ros_ws/src/solo_mpc/scenarios/dense50_3uav.yaml"
    _, settings, cfg = load_settings(scenario_path, ROOT / "config/drone_3uav.yaml")
    (tmp_path / "scenario.yaml").write_text(scenario_path.read_text())
    (tmp_path / "controller.yaml").write_text(yaml.safe_dump(settings))
    (tmp_path / "mpc-config.json").write_text(json.dumps(cfg))
    level = list(fov.level(0.0))
    records, truth, frames = [], [], {"uav1": [], "uav2": []}
    for k in range(100):  # 10 s; uav1 above the tag, uav2 far away
        t, target = k * 0.1, [10.0 + 0.03 * k, 10.0, 0.0]
        vehicles = {"uav1": [target[0], target[1], 2.0], "uav2": [30.0, 30.0, 2.0]}
        records.append({"stamp": t, "target_estimate": target[:2] + [0, 0], "vehicles": {
            n: {"position": p, "velocity": [0.3, 0, 0], "armed": True, "result": "tracking"}
            for n, p in vehicles.items()}})
        truth.append({"stamp": t, "target": target, "vehicles": {
            n: {"position": p, "yaw": 0.0, "orientation": level} for n, p in vehicles.items()}})
        seen = k % 10 != 0  # uav1 misses one frame in ten
        frames["uav1"].append({"stamp": t, "detected": seen, "pixel": [320, 240] if seen else None,
                               "observed": [target[0] + 0.05, target[1]] if seen else None})
        frames["uav2"].append({"stamp": t, "detected": k == 50, "pixel": None,
                               "observed": [1.0, 1.0] if k == 50 else None})
    for name, data in (("mpc.jsonl", records), ("truth.jsonl", truth)):
        (tmp_path / name).write_text("".join(json.dumps(r) + "\n" for r in data))
    for n, rows in frames.items():
        (tmp_path / f"detections_{n}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    report = analyze_run.analyze(tmp_path)["fov"]
    assert report["attitude"] == "truth attitude"
    one, two = report["vehicles"]["uav1"], report["vehicles"]["uav2"]
    assert one["fully_in_view_fraction"] == 1.0 and one["centre_offset"]["mean"] < 0.05
    assert one["detection"]["recall"] == pytest.approx(0.9)
    assert one["detection"]["error_m"]["mean"] == pytest.approx(0.05)
    assert two["in_view_fraction"] == 0.0 and two["detection"]["false_detections"] == 1
    assert two["detection"]["error_m"]["mean"] is None  # the false detection is not an accuracy sample
    assert report["team"]["any_in_view_fraction"] == 1.0 and report["team"]["mean_viewers"] == 1.0
    assert report["team"]["detected_by_any_fraction"] > 0.9
    overlap = report["overlap"]["pairs"]["uav1-uav2"]
    assert overlap["overlapping_fraction"] == 0.0 and report["overlap"]["shared_fraction"] == pytest.approx(0.0)


def level_footprint(x, y, plane=0.25, altitude=2.0):
    K = fov.intrinsics(640, 480, math.pi / 2)
    return fov.footprint([x, y, altitude], fov.level(0.0), K, 640, 480, plane)


def test_level_footprint_matches_camera_geometry():
    depth = 2.0 - 0.04 - 0.25
    corners = level_footprint(0.0, 0.0)
    # yaw 0: image width runs along world Y (2 * depth), height along X (1.5 * depth).
    assert np.ptp(corners[:, 1]) == pytest.approx(2 * depth)
    assert np.ptp(corners[:, 0]) == pytest.approx(1.5 * depth)
    assert fov.area(corners) == pytest.approx(3 * depth ** 2)
    tilted = Rotation.from_euler("x", 50, degrees=True).as_quat()
    assert fov.footprint([0, 0, 2.0], tilted, fov.intrinsics(640, 480, math.pi / 2), 640, 480, 0.25) is None


@pytest.mark.parametrize("separation", [0.0, 1.0, 2.0, 3.0])
def test_pairwise_overlap_is_exact_for_offset_rectangles(separation):
    depth = 2.0 - 0.04 - 0.25
    a, b = level_footprint(0.0, 0.0), level_footprint(separation, 0.0)
    shared = fov.intersection(a, b)
    expected = max(0.0, 1.5 * depth - separation) * 2 * depth
    assert (fov.area(shared) if shared is not None else 0.0) == pytest.approx(expected, abs=1e-9)


def test_union_of_three_tilted_footprints_matches_sampling():
    K = fov.intrinsics(640, 480, math.pi / 2)
    prints = [fov.footprint(p, Rotation.from_euler("xyz", r, degrees=True).as_quat(), K, 640, 480, 0.25)
              for p, r in (([0, 0, 2], [5, 0, 10]), ([1.5, 0.5, 2], [0, -6, 40]), ([0.5, 2.0, 2], [3, 3, -20]))]
    rng = np.random.default_rng(0)
    points = rng.uniform(-4, 6, (200_000, 2))
    inside = lambda poly: np.all([(b[0] - a[0]) * (points[:, 1] - a[1]) - (b[1] - a[1]) * (points[:, 0] - a[0]) >= 0
                                  for a, b in zip(poly, np.roll(poly, -1, axis=0))], axis=0)
    sampled = np.mean(np.any([inside(p) for p in prints], axis=0)) * 100
    assert fov.union_area(prints) == pytest.approx(sampled, rel=0.02)
