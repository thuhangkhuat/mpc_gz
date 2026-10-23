"""Real spawned workers: numerical parity, concurrency and bounded failures."""

import os
import json
from pathlib import Path
import signal
import time

import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("_pydecomp")
from solo_mpc.core.controller import TeamController, VehicleState
from solo_mpc.core.parallel_controller import ParallelTeamController
from solo_mpc.core.planner_grid import LocalGridPlanner, _dijkstra8, _jps8, warmup
from solo_mpc.core.sensors import project_scan
from solo_mpc.core.tracking import ExactTargetTracker
from solo_mpc.settings import load_settings

PKG = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"


def inputs(names, stamp):
    states = {
        n: VehicleState(np.array([8., 4. + 1.2 * i, 3.]), np.zeros(3),
                        np.zeros(3), np.array([0., 0., 0., 1.]), stamp)
        for i, n in enumerate(names)
    }
    scans = {}
    for n, state in states.items():
        ranges = np.full(180, np.inf)
        ranges[90:94] = 2.5  # real occupied cells as well as clearing rays
        scans[n] = project_scan(ranges, -np.pi, 2*np.pi/180, .1, 3.,
                                state.position, state.quaternion, stamp)
    return states, scans


@pytest.fixture
def cfg():
    _, _, config = load_settings(PKG / "scenarios/small50_3uav.yaml",
                                  PKG / "config/controller.yaml")
    # Numerical equivalence, not a deadline benchmark on a shared CI machine.
    config["IPOPT_OPTIONS"] = {**config["IPOPT_OPTIONS"], "ipopt.max_cpu_time": 1.0}
    return config


@pytest.mark.parametrize("search", ["jps", "dijkstra"])
def test_planner_warmup_covers_real_window_layouts(search):
    pytest.importorskip("numba")
    warmup()
    kernel = _jps8 if search == "jps" else _dijkstra8
    assert {signature[0].layout for signature in kernel.signatures} >= {"C", "A"}
    signatures = set(kernel.signatures)
    planner = LocalGridPlanner((0., 0., 20., 20.), grid_resolution=.5,
                               local_radius=3., unknown_policy="optimistic", search=search)
    planner.initialize([8., 10.], [10., 11.])
    ok, path, *_ = planner.plan(np.empty((0, 2)), .36, observe=False)
    assert ok and len(path) >= 2
    # Shape changes do not compile new kernels; array-layout changes do.
    assert set(kernel.signatures) == signatures


def test_fresh_cache_workers_finish_first_solve_with_default_deadline(cfg, tmp_path, monkeypatch):
    # Spawned interpreters start with no cached layout specializations, just as
    # a new deployment does. Previously every worker died on its first JPS job.
    monkeypatch.setenv("NUMBA_CACHE_DIR", str(tmp_path / "numba-cache"))
    names = ["alpha", "bravo", "charlie"]
    with ParallelTeamController(names, cfg) as parallel:
        parallel.track.observe([9., 4.], 0.)
        states, scans = inputs(names, 0.)
        results = parallel.step(states, scans, 0., {names[0]})
        assert all(r["ok"] for r in results.values()), {
            n: r["reason"] for n, r in results.items()}
        assert all(w.error is None and w.process.is_alive() for w in parallel.workers.values())


@pytest.mark.parametrize('target_mode', ['camera', 'ground_truth'])
def test_workers_match_serial_with_shared_neighbors_and_persistent_maps(cfg, tmp_path, target_mode):
    names = ["alpha", "bravo", "charlie"]
    serial = TeamController(names, cfg)
    captured = dict(cfg, DIAGNOSTIC_DIR=str(tmp_path), DIAGNOSTIC_CAPTURE_CYCLES=4)
    with ParallelTeamController(names, captured, timeout=3.0) as parallel:
        if target_mode == 'ground_truth':
            serial.track = ExactTargetTracker()
            parallel.track = ExactTargetTracker()
        pids = {w.pid for w in parallel.workers.values()}
        assert len(pids) == len(names) and os.getpid() not in pids
        # The coordinator owns no native solvers or local occupancy integrators.
        assert parallel.solvers == {}
        for k in range(4):
            stamp = k * .1
            states, scans = inputs(names, stamp)
            positions = {n: s.position.copy() for n, s in states.items()}
            for team in (serial, parallel):
                team.track.observe([9. + .02*k, 4.], stamp)
            expected = serial.step(states, scans, stamp, {names[0]})
            actual = parallel.step(states, scans, stamp, {names[0]})
            assert parallel.leader == serial.leader
            assert parallel.assignment == serial.assignment
            assert parallel.modes == serial.modes
            for n in names:
                assert expected[n]["ok"] and actual[n]["ok"]
                np.testing.assert_allclose(actual[n]["prediction"], expected[n]["prediction"],
                                           atol=1e-7, rtol=1e-7)
                np.testing.assert_allclose(actual[n]["acceleration"], expected[n]["acceleration"],
                                           atol=1e-7, rtol=1e-7)
                np.testing.assert_array_equal(states[n].position, positions[n])
                serial.planners[n].gmap._ensure()
                np.testing.assert_array_equal(parallel.planners[n].gmap.blocked,
                                               serial.planners[n].gmap.blocked)
                assert actual[n]["worker_pid"] in pids
                assert actual[n]["cycle"] == k + 1
                assert actual[n]["mapping_ms"] >= 0
                record = json.loads((tmp_path / n / f'solve-{(k+1) % 4:04d}.json').read_text())
                for key, value in serial.solver_inputs[n].items():
                    if value is None:
                        assert record['inputs'][key] is None
                    else:
                        np.testing.assert_allclose(record['inputs'][key], value, atol=1e-10, rtol=1e-12,
                                                   err_msg=key)
            # Each worker receives the same snapshot. Solves really overlap;
            # creating three processes alone would not establish concurrency.
            if k:
                assert max(r["solve_started_wall"] for r in actual.values()) < min(
                    r["solve_finished_wall"] for r in actual.values())
    assert all(not w.process.is_alive() for w in parallel.workers.values())
    parallel.close()  # idempotent shutdown


@pytest.mark.skipif(not hasattr(signal, "SIGSTOP"), reason="Requires POSIX process suspension")
def test_stalled_workers_have_one_deadline_and_never_use_stale_maps(cfg):
    names = ["alpha", "bravo", "charlie"]
    with ParallelTeamController(names, cfg, timeout=.2) as parallel:
        parallel.track.observe([9., 4.], 0.)
        for n in names[:2]:
            os.kill(parallel.workers[n].pid, signal.SIGSTOP)
        states, scans = inputs(names, 0.)
        started = time.perf_counter()
        result = parallel.step(states, scans, 0., {names[0]})
        assert time.perf_counter() - started < 1.0
        assert all(not r["ok"] for r in result.values())
        assert result["alpha"]["reason"] == "worker_timeout: map"
        assert result["bravo"]["reason"] == "worker_timeout: map"
        assert result["charlie"]["reason"] == "fleet_mapping_failed"
        # No queued late response may be reused in a later decision.
        assert all(p is None for p in parallel.predictions.values())
        assert all(not w.process.is_alive() for n, w in parallel.workers.items() if n != "charlie")
        assert all(not r["ok"] for r in parallel.step(states, scans, .1, {names[0]}).values())
    assert all(not w.process.is_alive() for w in parallel.workers.values())


def test_single_vehicle_worker_handles_empty_neighbor_set_and_disconnect(cfg):
    with ParallelTeamController(["solo"], cfg, timeout=3.) as parallel:
        parallel.track.observe([9., 4.], 0.)
        states, scans = inputs(["solo"], 0.)
        assert parallel.step(states, scans, 0., {"solo"})["solo"]["ok"]
        worker = parallel.workers["solo"]
        worker.process.terminate()
        worker.process.join(timeout=1.)
        result = parallel.step(states, scans, .1, {"solo"})["solo"]
        assert not result["ok"]
        assert "worker_disconnected" in result["reason"]


def test_startup_failure_joins_every_spawned_worker(cfg):
    import multiprocessing as mp
    before = {p.pid for p in mp.active_children()}
    invalid = dict(cfg, CORRIDOR_MAX_FACES=-1)
    with pytest.raises(RuntimeError, match='worker initialization failed'):
        ParallelTeamController(['a', 'b', 'c'], invalid, startup_timeout=15.)
    assert {p.pid for p in mp.active_children()} == before


def test_wrong_cycle_response_disables_worker_instead_of_reusing_it(cfg):
    from solo_mpc.core.parallel_controller import Worker
    import multiprocessing as mp
    context = mp.get_context('spawn')
    receiver, sender = context.Pipe()
    class Process:
        def is_alive(self):
            return False
        def join(self, timeout):
            pass
    team = ParallelTeamController.__new__(ParallelTeamController)
    team.workers = {'a': Worker(Process(), receiver)}
    sender.send({'phase': 'solve', 'cycle': 7, 'result': {'ok': True}})
    try:
        messages = team._collect({'a'}, 'solve', 8, .2)
        assert 'does not match the active cycle' in messages['a']['error']
        assert team.workers['a'].error
    finally:
        sender.close()
        receiver.close()
