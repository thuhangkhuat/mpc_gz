"""Regressions for continuous flight rather than lockstep model integration."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from solo_mpc.core.references import sample_trajectory


def point(t, x, v, a):
    return SimpleNamespace(time_from_start=SimpleNamespace(sec=0, nanosec=int(t * 1e9)),
                           positions=[x, 0.], velocities=[v, 0.], accelerations=[a, 0.])


def test_delayed_plan_samples_elapsed_time_and_interval_control():
    points = [point(0, 0, 0, 2), point(.1, .01, .2, -2), point(.2, .03, .2, 0)]
    p, v, a = sample_trajectory(points, .15)
    np.testing.assert_allclose(p, [.02, 0.])
    np.testing.assert_allclose(v, [.2, 0.])
    np.testing.assert_allclose(a, [-2., 0.])
    for age in (-.1, .3, float('nan')):
        with pytest.raises(ValueError):
            sample_trajectory(points, age)
    points[1].time_from_start.nanosec = 0
    with pytest.raises(ValueError):
        sample_trajectory(points, .1)


@pytest.mark.parametrize('velocity,acceleration', [(1.25, .2), (1.19, 1.0)])
def test_measured_overspeed_has_bounded_feasible_recovery_and_stopping_branch(velocity, acceleration):
    pytest.importorskip('casadi')
    from solo_mpc.core.mpc_problem import MPCProblem, speed_recovery_limits
    from solo_mpc.settings import load_settings
    pkg = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'
    _, _, cfg = load_settings(pkg / 'scenarios/small50_3uav.yaml', pkg / 'config/controller.yaml')
    cfg['SPEED_RECOVERY'] = True
    cfg['IPOPT_OPTIONS'] = {**cfg['IPOPT_OPTIONS'], 'ipopt.max_cpu_time': 2.}
    x0 = np.array([10., 10., velocity, 0., acceleration, 0.])
    limits = speed_recovery_limits(x0, cfg, cfg['BACKUP_HORIZON'])
    assert limits[0] > cfg['VMAX']
    assert limits[-1] == cfg['VMAX']
    solver = MPCProblem(0, 4, cfg)
    A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
    ok, X, U = solver.solve(
        x0=x0, A_hard=A, b_hard=A @ x0[:2] + 3,
        A_cost=None, b_cost=None, use_corr_cost=False, others=[], nb_flags=[],
        tgt=[12., 10.], w_search=0., slot=[0., 0.], w_slot=0., leader=False,
        ref=[0., 0.], trk_goal=[12., 10.], s_ref=0.,
        X_init=np.tile(x0, (cfg['HORIZON_LENGTH'] + 1, 1)), U_init=np.zeros((cfg['HORIZON_LENGTH'], 2)))
    assert ok, solver.opti.stats()
    np.testing.assert_allclose(X[0], x0, atol=1e-7)
    for states, controls in ((X, U), solver.last_backup):
        assert np.all(np.linalg.norm(states[1:, 2:4], axis=1) <= limits[:len(states)-1] + 1e-5)
        assert np.max(np.linalg.norm(controls, axis=1)) <= cfg['UMAX'] + 1e-5
        assert np.all(states[1:, :2] @ A.T <= A @ x0[:2] + 3 - cfg['CORRIDOR_MARGIN'] + 1e-5)
    np.testing.assert_allclose(solver.last_backup[0][-1, 2:], 0., atol=1e-6)
