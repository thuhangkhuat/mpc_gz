"""Target speed schedules must survive spline sampling and configuration loading."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from solo_mpc.core.geometry import catmull_rom
from solo_mpc.core.target_motion import target_route
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'


@pytest.mark.parametrize('smooth', [False, True])
def test_per_leg_speed_preserves_waypoints_and_exact_arrival_times(smooth):
    scenario = {'tar_max_speed': 1, 'target': {
        'waypoints': [[0, 0], [10, 0], [10, 20]], 'speeds': [2, 5],
    }}
    cfg = {'TAR_SMOOTH_ENABLE': smooth, 'TAR_SPLINE_DS': 0.1}
    path, times = target_route(scenario, cfg)
    corner = np.flatnonzero(np.all(path == [10, 0], axis=1))[0]
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    assert times[corner] == pytest.approx(lengths[:corner].sum() / 2)
    assert times[-1] - times[corner] == pytest.approx(lengths[corner:].sum() / 5)
    assert np.all(np.diff(times) > 0)
    np.testing.assert_array_equal(path[[0, corner, -1]], scenario['target']['waypoints'])
    if not smooth:
        np.testing.assert_allclose(times, [0, 5, 9])


def test_route_without_speed_schedule_keeps_constant_speed():
    scenario = {'tar_max_speed': 0.6, 'target': {'waypoints': [[0, 0], [1, 1], [2, 1]]}}
    path, times = target_route(scenario, {})
    np.testing.assert_allclose(times[1:], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)) / 0.6)


@pytest.mark.parametrize('speeds', [[1], [1, 1, 1], [1, 0], [1, -1], [1, float('nan')]])
def test_invalid_speed_schedule_rejected_before_launch(tmp_path, speeds):
    scenario = yaml.safe_load((PKG / 'scenarios/small50_3uav.yaml').read_text())
    scenario['target'] = {'waypoints': [[0, 0], [1, 1], [2, 1]], 'speeds': speeds}
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(scenario))
    with pytest.raises(ValueError, match='target.speeds'):
        load_settings(path, PKG / 'config/controller.yaml')


def test_three_uav_route_preserves_waypoints_and_constant_speed():
    scenario, _, cfg = load_settings(PKG / 'scenarios/small50_3uav.yaml',
                                     PKG / 'config/controller.yaml')
    path, times = target_route(scenario, cfg)
    _, knots = catmull_rom(scenario['target']['waypoints'], cfg['TAR_SPLINE_DS'])
    np.testing.assert_allclose(path[knots], scenario['target']['waypoints'], atol=1e-12)
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    for a, b in zip(knots[:-1], knots[1:]):
        assert lengths[a:b].sum() / (times[b] - times[a]) == pytest.approx(scenario['tar_max_speed'])


def test_single_waypoint_is_a_fixed_target():
    path, times = target_route({'tar_max_speed': 0.6, 'target': {'waypoints': [[97, 34]]}}, {})
    for elapsed in (0.0, 5.0, 1e6):
        assert [np.interp(elapsed, times, path[:, i]) for i in (0, 1)] == [97, 34]
    with pytest.raises(ValueError, match='target.waypoints'):
        target_route({'tar_max_speed': 0.6, 'target': {'waypoints': [[97, 34]], 'speeds': [1]}}, {})


def test_soccer_scenario_crosses_field_to_fixed_target():
    scenario, settings, cfg = load_settings(PKG / 'scenarios/soccer_1uav.yaml',
                                            PKG / 'config/controller_solo.yaml')
    assert (scenario['xlim'], scenario['ylim']) == ([0, 105], [0, 68])
    assert settings['target_mode'] == 'ground_truth'
    start = np.asarray(scenario['starts'][0][:2])
    goal = np.asarray(scenario['target']['waypoints'][0])
    distance = np.linalg.norm(goal - start)
    assert settings['arrival_timeout_sim'] > 2 * distance / cfg['VMAX']
    obstacles = [(np.array([x + w / 2, y + h / 2]), np.hypot(w, h) / 2)
                 for x, y, w, h in scenario['obstacles']['rects']]
    obstacles += [(np.array([x, y]), r) for x, y, r in scenario['obstacles']['circles']]
    for i, (c, r) in enumerate(obstacles):
        assert min(np.linalg.norm(c - start), np.linalg.norm(c - goal)) - r >= 5
        assert all(np.linalg.norm(c - d) - r - s >= 2.5 for d, s in obstacles[i + 1:])
    # Obstacles block the straight crossing, so the planner has to avoid them.
    t = np.linspace(0, 1, 2000)[:, None]
    line = start + t * (goal - start)
    assert any(np.min(np.linalg.norm(line - c, axis=1)) < r + cfg['INFLATE_RADIUS']
               for c, r in obstacles)
