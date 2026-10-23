"""Integration contracts that matter when replacing an offline simulator."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from solo_bringup.configuration import load_config
from solo_mpc.core.mapping import SensorGrid
from solo_mpc.core.sensors import ScanRays, pixel_to_ground, project_scan
from solo_mpc.core.tracking import TargetTracker
from solo_mpc.settings import load_settings
from solo_mpc.world import bridge_config, generate_mpc_world
from solo_mpc.scene_view import FOLLOW_OFFSET, OVERHEAD_TOPIC

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros_ws/src/solo_mpc"


def test_default_configuration_has_northbound_row_and_correct_vehicle_scale():
    scenario, settings, cfg = load_settings(
        PKG / "scenarios/small50_3uav.yaml", PKG / "config/controller.yaml"
    )
    assert scenario["xlim"] == [0, 50]
    assert scenario["ylim"] == [0, 30]
    assert (
        len(scenario["obstacles"]["rects"])
        == len(scenario["obstacles"]["circles"])
        == 12
    )
    starts = np.asarray(scenario["starts"], float)
    heading = np.diff(np.asarray(scenario["target"]["waypoints"][:2], float), axis=0)[0]
    assert np.allclose(starts[:, 1], starts[0, 1])
    assert np.allclose(starts[:, 2], settings["flight_altitude"])
    assert heading[0] == 0 and heading[1] > 0
    assert np.allclose((starts[:, :2] - starts[0, :2]) @ heading, 0)
    assert cfg["GRID_CELLS"] == [[-1, 0], [1, 0]]
    assert cfg["ROBOT_RADIUS"] == 0.36
    assert cfg["UAV_SAFE_DISTANCE"] >= 2 * cfg["ROBOT_RADIUS"]
    assert settings["target_mode"] == "camera"


def test_missing_scan_is_unknown_and_observed_no_return_is_free():
    scan = project_scan(
        [float("nan"), float("inf")],
        0,
        np.pi / 2,
        0.1,
        3.0,
        [1, 1, 3],
        [0, 0, 0, 1],
        1.0,
    )
    grid = SensorGrid((0, 0, 5, 5), resolution=0.1, inflate_radius=0.2)
    grid.integrate(scan)
    ix, iy = grid.world_to_grid(2, 1)
    assert grid.L[iy, ix] == 0
    ix, iy = grid.world_to_grid(1, 2)
    assert grid.L[iy, ix] < 0


def test_teammate_hit_never_becomes_a_static_obstacle():
    scan = project_scan(
        [1.0], 0, 1, 0.1, 3.0, [1, 1, 3], [0, 0, 0, 1], 1.0, neighbors=[[2, 1, 3]]
    )
    assert not scan.hits.any()


def test_observed_clear_rays_remove_old_occupied_cell():
    grid = SensorGrid((0, 0, 5, 5), resolution=0.1, inflate_radius=0.2)
    grid.integrate(
        ScanRays(np.array([1, 1, 3]), np.array([[2, 1, 3]]), np.array([True]), 1.0)
    )
    ix, iy = grid.world_to_grid(2, 1)
    assert grid.L[iy, ix] > 0
    for i in range(10):
        grid.integrate(
            ScanRays(
                np.array([1, 1, 3]), np.array([[3, 1, 3]]), np.array([False]), i + 2.0
            )
        )
    assert grid.L[iy, ix] < 0


def test_camera_geometry_center_and_right_pixel():
    K = np.array([[100, 0, 80], [0, 100, 80], [0, 0, 1.0]])
    center = pixel_to_ground([80, 80], K, [5, 5, 3], [0, 0, 0, 1], 0.0)
    right = pixel_to_ground([90, 80], K, [5, 5, 3], [0, 0, 0, 1], 0.0)
    assert np.allclose(center, [5, 5, 0])
    assert np.allclose(right, [5, 4.7, 0])


def test_target_coasts_without_fabricating_new_observations():
    tracker = TargetTracker()
    tracker.observe([0, 0], 0.0)
    tracker.observe([1, 0], 1.0)
    trajectory = tracker.trajectory(1.1, 10, 0.1, 2.0)
    assert trajectory[-1, 0] > trajectory[0, 0]
    assert tracker.observed_at == 1.0


def test_mpc_updates_do_not_integrate_measured_vehicle_state():
    pytest.importorskip("casadi")
    pytest.importorskip("_pydecomp")
    from solo_mpc.core.controller import TeamController, VehicleState

    _, _, cfg = load_settings(
        PKG / "scenarios/small50_3uav.yaml", PKG / "config/controller.yaml"
    )
    # This checks state ownership, not cold-start timing on a shared build host.
    cfg["IPOPT_OPTIONS"] = {**cfg["IPOPT_OPTIONS"], "ipopt.max_cpu_time": 1.0}
    team = TeamController(["uav1"], cfg)
    state = VehicleState(
        np.array([3.0, 3.0, 3.0]),
        np.zeros(3),
        np.zeros(3),
        np.array([0.0, 0.0, 0.0, 1.0]),
        0.0,
    )
    original = state.position.copy()
    team.track.observe([3.5, 3.5], 0.0)
    scan = project_scan(
        np.full(180, np.inf),
        -np.pi,
        2 * np.pi / 180,
        0.1,
        3.0,
        state.position,
        state.quaternion,
        0.0,
    )
    result = team.step({"uav1": state}, {"uav1": scan}, 0.0, {"uav1"})
    assert np.array_equal(state.position, original)
    assert result["uav1"]["ok"]


@pytest.mark.parametrize("record_overhead", [False, True])
def test_mpc_world_matches_scenario_and_has_unique_sensor_topics(tmp_path, record_overhead):
    source = Path("/opt/ardupilot_gazebo")
    if not source.exists():
        pytest.skip("Requires pinned Gazebo model source in image")
    fleet = load_config(ROOT / "config/fleet.yaml")
    path, resolved = generate_mpc_world(
        fleet,
        source,
        tmp_path / "world.sdf",
        PKG / "scenarios/small50_3uav.yaml",
        PKG / "config/controller.yaml",
        record_overhead=record_overhead,
    )
    world = ET.parse(path).getroot().find("world")
    assert (
        len(
            [m for m in world.findall("model") if m.get("name").startswith("obstacle_")]
        )
        == 24
    )
    topics = [
        s.findtext("topic")
        for s in world.iter("sensor")
        if s.get("type") in ("camera", "gpu_lidar")
    ]
    assert len(topics) == len(set(topics)) == 6 + int(record_overhead)
    assert resolved["uavs"][0]["spawn"][:2] == [5, 2]
    camera = world.find("model[@name='tracking_target']/link/sensor")
    if record_overhead:
        assert camera.findtext("topic") == OVERHEAD_TOPIC
        assert camera.findtext("camera/image/width") == "1920"
        assert camera.findtext("camera/image/height") == "1080"
        x, y, z, roll, pitch, yaw = map(float, camera.findtext("pose").split())
        assert (x, y, z) == FOLLOW_OFFSET
        # Gazebo cameras look along local +X; the optical axis must hit the target.
        forward = np.array([np.cos(yaw) * np.cos(pitch),
                            np.sin(yaw) * np.cos(pitch), -np.sin(pitch)])
        assert np.allclose(forward, -np.array(FOLLOW_OFFSET) / np.linalg.norm(FOLLOW_OFFSET))
    else:
        assert camera is None
    bridge = bridge_config([u["name"] for u in resolved["uavs"]], "mpc_small50",
                           tmp_path / "bridge.yaml", record_overhead=record_overhead)
    rows = yaml.safe_load(Path(bridge).read_text())
    assert len(rows) == 13 + int(record_overhead)
    assert any(row["gz_topic_name"] == OVERHEAD_TOPIC for row in rows) == record_overhead
    if record_overhead:
        assert next(row for row in rows if row['gz_topic_name'] == OVERHEAD_TOPIC)['lazy'] is True


def test_gazebo_cli_remains_available_after_ros_vendor_setup():
    if not Path("/opt/ardupilot_gazebo").exists():
        pytest.skip("Requires the simulation image entrypoint environment")
    result = subprocess.run(
        ["gz", "sim", "--versions"], capture_output=True, text=True, timeout=10
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert any(line.strip().startswith("8.") for line in result.stdout.splitlines())


@pytest.mark.parametrize("cells", [[[-1, 0]], [[-1, 0], [0, 0]], [[-1, 0], [-1, 0]]])
def test_three_uav_fleet_requires_valid_follower_slots(tmp_path, cells):
    scenario = yaml.safe_load((PKG / "scenarios/small50_3uav.yaml").read_text())
    scenario["params"]["GRID_CELLS"] = cells
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario))
    with pytest.raises(ValueError, match="GRID_CELLS"):
        load_settings(path, PKG / "config/controller.yaml")
