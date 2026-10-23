"""Callback responsiveness, snapshot isolation, expiry and exact solver capture."""

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from solo_mpc.core.cycles import CycleRunner, result_is_current


def test_one_outstanding_cycle_owns_a_copy_and_callbacks_can_continue():
    entered, release = threading.Event(), threading.Event()
    inputs = {'state': np.array([1., 2.]), 'observations': [1.]}
    def blocked(snapshot):
        entered.set()
        assert release.wait(2)
        return snapshot
    cycles = CycleRunner()
    try:
        cycles.submit(blocked, inputs)
        assert entered.wait(1)
        # The caller remains free to run sensor callbacks during optimization.
        inputs['state'][:] = 50
        inputs['observations'].append(2.)
        with pytest.raises(RuntimeError, match='already active'):
            cycles.submit(blocked, inputs)
        assert cycles.take() is None
        release.set()
        cycles.future.result(timeout=1)
        result = cycles.take()
        np.testing.assert_array_equal(result['state'], [1., 2.])
        assert result['observations'] == [1.]
    finally:
        release.set()
        cycles.close()
    assert not any(t.name.startswith('fleet-cycle') for t in threading.enumerate())
    with pytest.raises(RuntimeError, match='closed'):
        cycles.submit(blocked, inputs)


def test_obsolete_results_never_renew_command_validity():
    settings = dict(command_timeout_sim=.4, command_timeout_wall=3.)
    now = time.monotonic()
    assert result_is_current(1., 1.1, now, settings)
    assert not result_is_current(1., 1.5, now, settings)
    assert not result_is_current(1., .9, now, settings)
    assert not result_is_current(1., 1.1, now - 4, settings)


def test_capture_is_complete_before_solve_bounded_and_replays_backup_warm_start(tmp_path):
    pytest.importorskip('casadi')
    pytest.importorskip('_pydecomp')
    from solo_mpc.core.controller import TeamController, VehicleState
    from solo_mpc.core.diagnostics import SolverCapture, replay
    from solo_mpc.settings import load_settings
    pkg = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'
    _, _, cfg = load_settings(pkg / 'scenarios/small50_3uav.yaml', pkg / 'config/controller.yaml')
    cfg['IPOPT_OPTIONS'] = {k: v for k, v in cfg['IPOPT_OPTIONS'].items() if k != 'ipopt.max_cpu_time'}
    cfg.update(DIAGNOSTIC_DIR=str(tmp_path), DIAGNOSTIC_CAPTURE_CYCLES=2)
    team = TeamController(['u'], cfg)
    s = VehicleState(np.array([8., 4., 3.]), np.zeros(3), np.zeros(3), np.array([0.,0.,0.,1.]), 0.)
    team.track.observe([9., 4.], 0.)
    pred = team.neighbor_predictions({'u': s}, 0.)
    guidance = team.prepare_guidance({'u': s}, pred, 0., {'u'})['u']
    for k in range(3):
        result = team.solve_vehicle('u', {'u': s}, pred, guidance, k * .1)
        assert result['nominal_ok']
        files = list((tmp_path / 'u').glob('*.json'))
        assert len(files) <= 2
        path = max(files, key=lambda p: json.loads(p.read_text())['sequence'])
        record = json.loads(path.read_text())
        assert record['outcome'] == 'success'
        assert set(record['inputs']) == set(team.solver_inputs['u'])
        if k:
            assert record['backup_init'] is not None
        ok, X, U = replay(path)
        assert ok
        np.testing.assert_allclose(X, record['X'], atol=1e-7, rtol=1e-7)
        np.testing.assert_allclose(U, record['U'], atol=1e-7, rtol=1e-7)
    import os
    import subprocess
    import sys
    for flags in ([], ['--iteration-only']):
        result = subprocess.run([sys.executable, '-m', 'solo_mpc.replay_optimizer', str(path), *flags],
                                env={**os.environ, 'PYTHONPATH': os.pathsep.join(
                                    [str(pkg), os.environ.get('PYTHONPATH', '')])}, text=True,
                                capture_output=True, timeout=10)
        assert result.returncode == 0, result.stdout + result.stderr
        report = json.loads(result.stdout[result.stdout.index('{'):])
        assert report['matches_recorded_outcome']
        assert report['iteration_only'] == bool(flags)
    capture = SolverCapture(cfg, 'u', team.solvers['u'], .4, team.solver_inputs['u'])
    assert json.loads(capture.path.read_text())['outcome'] == 'pending'
    assert not list(tmp_path.rglob('*.tmp'))


def test_fallback_is_revalidated_after_new_measured_state_arrives():
    from types import SimpleNamespace
    from solo_mpc.core.cycles import stopping_result_matches
    cfg = dict(TIMESTEP=.1, BACKUP_POSITION_TOLERANCE=.15,
               BACKUP_VELOCITY_TOLERANCE=.3, BACKUP_ACCEL_TOLERANCE=.5)
    prediction = np.tile([8.,4.,0.,0.,0.,0.], (11, 1))
    measured = SimpleNamespace(position=np.array([8.,4.,3.]), velocity=np.zeros(3),
                               acceleration=np.zeros(3), stamp=.1)
    result = dict(prediction=prediction)
    assert stopping_result_matches(measured, result, 0., cfg)
    measured.position[0] += 1
    assert not stopping_result_matches(measured, result, 0., cfg)
