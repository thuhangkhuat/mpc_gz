"""Stopping certificates, measured-state fallback, and local JPS regressions."""

from pathlib import Path

import numpy as np
import pytest
import yaml

pytest.importorskip("casadi")
pytest.importorskip("_pydecomp")

from solo_mpc.core.controller import TeamController, VehicleState
from solo_mpc.core.dynamics import model
from solo_mpc.core.planner_grid import LocalGridPlanner, _dijkstra8, _jps8, _jps_cells
from solo_mpc.core.sensors import project_scan
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"


def config(**overrides):
    _, _, cfg = load_settings(PKG / "scenarios/small50_3uav.yaml",
                              PKG / "config/controller.yaml")
    cfg.update(overrides)
    cfg["IPOPT_OPTIONS"] = {**cfg["IPOPT_OPTIONS"], "ipopt.max_cpu_time": 1.0}
    return cfg


def vehicle(x, stamp):
    return VehicleState(np.r_[x[:2], 3.0], np.r_[x[2:4], 0.0],
                        np.r_[x[4:6], 0.0], np.array([0., 0., 0., 1.]), stamp)


def step(team, x, stamp):
    state = vehicle(x, stamp)
    scan = project_scan(np.full(360, np.inf), -np.pi, 2 * np.pi / 360,
                        .1, 3., state.position, state.quaternion, stamp)
    result = team.step({"solo": state}, {"solo": scan}, stamp, {"solo"})["solo"]
    np.testing.assert_array_equal(state.position, np.r_[x[:2], 3.0])
    return result


def started_team(**overrides):
    cfg = config(**overrides)
    team = TeamController(["solo"], cfg)
    team.track.observe([9., 4.], 0.)
    x = np.array([8., 4., .6, 0., 0., 0.])
    result = step(team, x, 0.)
    assert result["ok"] and result["nominal_ok"]
    return team, x, result


@pytest.mark.parametrize("terminal", [False, True])
def test_verified_stop_shares_first_action_and_stays_in_corridor(terminal):
    team, x, result = started_team(CONTINGENCY=not terminal, TERMINAL_STOP=terminal)
    plan = team.stop_plans["solo"]
    A, b = team.active_corridors["solo"]
    Ad, Bd = model(team.cfg["TIMESTEP"], team.cfg["D_FRAC"], team.cfg["ACCEL_TAU"], 2)
    np.testing.assert_allclose(plan.states[0], x, atol=1e-6)
    np.testing.assert_allclose(plan.controls[0], result["acceleration"], atol=1e-6)
    np.testing.assert_allclose(plan.states[1], result["prediction"][1], atol=1e-6)
    np.testing.assert_allclose(plan.states[1:], plan.states[:-1] @ Ad.T + plan.controls @ Bd.T,
                               atol=1e-6)
    np.testing.assert_allclose(plan.states[-1, 2:], 0., atol=1e-6)
    assert np.max(plan.states[1:, :2] @ A.T - b + team.cfg["CORRIDOR_MARGIN"]) < 1e-5
    assert np.linalg.norm(plan.controls, axis=1).max() <= team.cfg["UMAX"] + 1e-5
    assert np.linalg.norm(plan.states[:, 2:4], axis=1).max() <= team.cfg["VMAX"] + 1e-5


@pytest.mark.parametrize("failure", ["mpc", "planner", "corridor"])
@pytest.mark.parametrize("terminal", [False, True])
def test_failure_executes_remaining_stop_plan_until_rest(monkeypatch, failure, terminal):
    team, x, result = started_team(CONTINGENCY=not terminal, TERMINAL_STOP=terminal)
    plan = team.stop_plans["solo"]
    Ad, Bd = model(team.cfg["TIMESTEP"], team.cfg["D_FRAC"], team.cfg["ACCEL_TAU"], 2)
    x = Ad @ x + Bd @ result["acceleration"]
    if failure == "mpc":
        monkeypatch.setattr(team.solvers["solo"], "_solve", lambda **kw: (False, None, None))
    elif failure == "planner":
        monkeypatch.setattr(team, "_path", lambda *args: (False, []))
    else:
        def fail(*args):
            raise ValueError("decomposition unavailable")
        monkeypatch.setattr(team, "_corridor", fail)
    for k in range(1, len(plan.controls) + 3):
        result = step(team, x, k * team.cfg["TIMESTEP"])
        assert result["ok"] and not result["nominal_ok"]
        assert result["reason"].startswith("stop_plan: " + failure)
        index = min(k, len(plan.states) - 1)
        np.testing.assert_allclose(result["prediction"][0], plan.states[index], atol=1e-6)
        np.testing.assert_array_equal(team.predictions["solo"], result["prediction"])
        assert "solo" not in team.warm
        x = Ad @ x + Bd @ result["acceleration"]
    np.testing.assert_allclose(x[2:], 0., atol=1e-5)
    np.testing.assert_allclose(result["acceleration"], 0.)


@pytest.mark.parametrize("axis,delta", [(0, .3), (2, .6), (4, 1.)])
def test_divergent_measured_state_rejects_backup(monkeypatch, axis, delta):
    team, _, _ = started_team()
    x = team.stop_plans["solo"].states[1].copy()
    x[axis] += delta
    monkeypatch.setattr(team, "_path", lambda *args: (False, []))
    result = step(team, x, .1)
    assert not result["ok"]
    assert result["reason"] == "planner_failed"
    assert "solo" not in team.stop_plans


def test_backup_advances_with_simulation_time_and_rejects_clock_reset(monkeypatch):
    team, _, _ = started_team()
    plan = team.stop_plans["solo"]
    monkeypatch.setattr(team, "_path", lambda *args: (False, []))
    # Skip two controller cycles: never replay U[1] at time 0.3.
    result = step(team, plan.states[3], .3)
    np.testing.assert_allclose(result["acceleration"], plan.controls[3])
    np.testing.assert_allclose(result["position"], plan.states[4, :2])
    result = step(team, plan.states[0], -.1)
    assert not result["ok"] and "solo" not in team.stop_plans


def test_no_verified_plan_does_not_fabricate_fallback(monkeypatch):
    team = TeamController(["solo"], config(CONTINGENCY=False))
    team.track.observe([9., 4.], 0.)
    monkeypatch.setattr(team, "_path", lambda *args: (False, []))
    assert not step(team, np.array([8., 4., .6, 0., 0., 0.]), 0.)["ok"]


def test_corridor_switch_preserves_entire_backup_and_handles_decomposition_failure(monkeypatch):
    team, _, _ = started_team()
    plan = team.stop_plans["solo"]
    state = vehicle(plan.states[1], .1)
    previous = team.active_corridors["solo"]
    A, b = previous
    # This new box contains the UAV but excludes the rest of the stop branch.
    small = A @ state.position[:2] + team.cfg["CORRIDOR_MARGIN"] + .001
    assert team._contains(A, small, state.position[:2], team.cfg["CORRIDOR_MARGIN"])
    assert not team._contains(A, small, plan.states[1:, :2], team.cfg["CORRIDOR_MARGIN"])
    chosen = team._select_corridor("solo", state, .1, [(A, small)])
    np.testing.assert_array_equal(chosen[1], b)
    larger = b + .1
    chosen = team._select_corridor("solo", state, .1, [(A, larger)])
    np.testing.assert_array_equal(chosen[1], larger)
    monkeypatch.setattr(team.planners["solo"].gmap, "surface_points",
                        lambda *args: np.array([[9., 5.]]))
    from solo_mpc.core import controller
    def fail(*args):
        raise RuntimeError("pydecomp failed")
    monkeypatch.setattr(controller.pdc, "convex_decomposition_2D", fail)
    chosen = team._corridor("solo", state, [[8., 4.], [9., 4.]], .1)
    np.testing.assert_array_equal(chosen[1], b)
    team.active_corridors.clear()
    team.stop_plans.clear()
    with pytest.raises(ValueError, match="no finite polytope"):
        team._corridor("solo", state, [[8., 4.], [9., 4.]], .1)


def test_corridor_seed_starts_at_vehicle_keeps_turn_and_stays_in_lidar_range():
    team = TeamController([], config())
    # Nearest vertex is (0,0), but the UAV is on the preceding segment.
    path = [[-10., 0.], [0., 0.], [0., 10.]]
    pos = np.array([-1., 0., 3.])
    seed = team._corridor_seed(pos, path)
    np.testing.assert_array_equal(seed[0], pos[:2])
    assert seed[1, 0] < 0 and seed[1, 1] == 0
    box = team.cfg["CORRIDOR_BOX"]
    for dx in (-box, box):
        for dy in (-box, box):
            for endpoint in seed:
                assert np.linalg.norm(endpoint + [dx, dy] - pos[:2]) <= team.cfg["SENSING_RADIUS"]
    stationary = team._corridor_seed(pos, [pos[:2], pos[:2]])
    assert np.linalg.norm(stationary[1] - stationary[0]) > 0


def test_committed_path_keeps_next_corner_and_replans_after_empty_result():
    team = TeamController(["solo"], config())
    goal = np.array([10., 6.])
    team.paths["solo"] = [[8., 4.], [10., 4.], [10., 6.]]
    team.path_goals["solo"] = goal.copy()
    team.path_age["solo"] = 0
    ok, path = team._path("solo", np.array([9.8, 4., 3.]), goal)
    assert ok
    np.testing.assert_array_equal(path[1], [10., 4.])
    ok, path = team._path("solo", np.array([10., 4., 3.]), goal)
    assert ok
    np.testing.assert_array_equal(path[1], goal)
    team.paths["solo"] = []
    ok, path = team._path("solo", np.array([9.8, 4., 3.]), goal)
    assert ok and len(path) >= 2


def test_local_jps_matches_dijkstra_without_cutting_corners():
    rng = np.random.default_rng(42)
    for _ in range(30):
        blocked = rng.random((12, 15)) < .2
        blocked[0, 0] = blocked[-1, -1] = False
        distances, _ = _dijkstra8(blocked, np.ones(blocked.shape), 0, 0)
        parents, found = _jps8(blocked, 0, 0, 11, 14)
        assert found == np.isfinite(distances[-1, -1])
        if found:
            cells = _jps_cells(parents, blocked.shape[1], 11, 14)
            length = 0.
            for (y, x), (ny, nx) in zip(cells, cells[1:]):
                assert not blocked[ny, nx]
                if ny != y and nx != x:
                    assert not blocked[y, nx] and not blocked[ny, x]
                length += np.hypot(ny - y, nx - x)
            assert length == pytest.approx(distances[-1, -1])


def test_unreachable_jps_goal_uses_dijkstra_reachable_approach():
    planner = LocalGridPlanner((0., 0., 10., 10.), grid_resolution=1.,
                               inflate_radius=0., unknown_policy="optimistic", search="jps")
    planner.gmap.L[:] = -4.
    planner.gmap.L[:, 5] = 4.  # wall separates the goal from the vehicle
    planner.initialize([1.5, 4.5], [8.5, 4.5])
    ok, path, *_ = planner.plan(np.empty((0, 2)), 0., observe=False)
    assert ok and planner.n_jps_fallback == 1
    assert all(p[0] < 5. for p in path)


def test_sensor_margin_is_derived_after_overrides_and_can_be_pinned(tmp_path):
    cfg = config()
    assert cfg["LOCAL_SEARCH"] == "jps" and cfg["CONTINGENCY"]
    assert cfg["LIDAR_ANGULAR_RES"] == pytest.approx(np.pi / 180)
    expected = .36 + .03 + 1.2 * 25 * .1 * np.pi / 360 + .15 / np.sqrt(2)
    assert cfg["CORRIDOR_MARGIN"] == pytest.approx(expected)
    settings = yaml.safe_load((PKG / "config/controller.yaml").read_text())
    settings["params"]["CORRIDOR_MARGIN"] = .7
    settings["lidar_samples"] = 180
    path = tmp_path / "controller.yaml"
    path.write_text(yaml.safe_dump(settings))
    _, _, changed = load_settings(PKG / "scenarios/small50_3uav.yaml", path)
    assert changed["CORRIDOR_MARGIN"] == .7
    assert changed["LIDAR_ANGULAR_RES"] == pytest.approx(np.pi / 90)


@pytest.mark.parametrize("key,value", [("BACKUP_HORIZON", 0), ("CONTINGENCY", "true"),
                                       ("LOCAL_SEARCH", "invalid"), ("CORRIDOR_BOX", 2.2),
                                       ("CORRIDOR_MARGIN", .1), ("W_backup", -1)])
def test_invalid_contingency_settings_rejected(tmp_path, key, value):
    settings = yaml.safe_load((PKG / "config/controller.yaml").read_text())
    settings["params"][key] = value
    path = tmp_path / "controller.yaml"
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError, match=key):
        load_settings(PKG / "scenarios/small50_3uav.yaml", path)
