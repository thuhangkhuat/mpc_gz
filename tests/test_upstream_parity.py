"""Behavior/input parity with actual reference methods, not a second port copy.

Synthetic controlled snapshots isolate semantics; these are NOT archive replays.
No upstream module is imported (and no files are written in the reference tree).
"""

import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('casadi')
pytest.importorskip('_pydecomp')
from solo_mpc.core.controller import TeamController, VehicleState
from solo_mpc.core.mapping import SensorGrid
from solo_mpc.core.tracking import TargetTracker
from upstream_oracle import REFERENCE, REFERENCE_RUN, configuration, oracle, robot

UAV_COUNT = 3

pytestmark = pytest.mark.skipif(REFERENCE is None or REFERENCE_RUN is None
                                or not (REFERENCE / 'robot_jps.py').exists()
                                or not (REFERENCE_RUN / 'config.json').exists(),
                                reason='Set MPC_REFERENCE and MPC_REFERENCE_RUN to the read-only 2D source/run')


@pytest.fixture(scope='module', autouse=True)
def reference_files_are_read_only():
    import hashlib
    def fingerprint():
        files = list(REFERENCE.glob('*.py')) + list((REFERENCE / '__pycache__').glob('*'))
        files += list(REFERENCE_RUN.glob('*'))
        return {str(p): (p.stat().st_mtime_ns, hashlib.sha256(p.read_bytes()).hexdigest())
                for p in files if p.is_file()}
    before = fingerprint()
    yield
    assert fingerprint() == before, 'Reference source, cache, or run changed during comparison'


def state(position, stamp=0, velocity=(.4, .1)):
    return VehicleState(np.r_[position, 30.], np.r_[velocity, 0.], np.zeros(3),
                        np.array([0., 0., 0., 1.]), stamp)


def predictions(states, cfg):
    t = np.arange(cfg['HORIZON_LENGTH'] + 1)[:, None] * cfg['TIMESTEP']
    return {n: np.column_stack([s.position[:2] + t * s.velocity[:2],
                                np.tile(s.velocity[:2], (len(t), 1)), np.zeros((len(t), 2))])
            for n, s in states.items()}


def trackers(cfg, Tracker):
    port = TargetTracker(cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
    upstream = Tracker(cfg['TIMESTEP'], cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
    for k in range(60):
        z = np.array([100., 100.]) + k * cfg['TIMESTEP'] * np.array([1., .2])
        port.observe(z, k * cfg['TIMESTEP'])
        upstream.update(z)
        np.testing.assert_allclose(port.x, upstream.x, atol=1e-12, rtol=1e-12)
        np.testing.assert_allclose(port.P, upstream.P, atol=1e-12, rtol=1e-12)
    return port, upstream


def test_modes_slots_leader_hysteresis_and_prediction_match_actual_upstream():
    cfg = configuration()
    Robot, Tracker, _ = oracle(cfg)
    track, reference_track = trackers(cfg, Tracker)
    target = track.observation.copy()
    names = [f'u{i}' for i in range(UAV_COUNT)]
    team = TeamController(names, cfg, local_names=[])
    team.track = track
    grid = SensorGrid(cfg['WORLD_BOUNDS'], resolution=cfg['GRID_RESOLUTION'],
                      inflate_radius=cfg['INFLATE_RADIUS'], treat_unknown_as_blocked=False)
    # Shared map contains occupied, inflated, free, and unknown cells.
    grid.L[:] = 0
    ix, iy = grid.world_to_grid(target[0] + 18, target[1] + 25)
    grid.L[iy:iy+2, ix:ix+3] = 4
    grid._ensure()
    team.planners = {n: SimpleNamespace(gmap=grid) for n in names}
    states = {n: state(target + [200 + 5*i, 150]) for i, n in enumerate(names)}
    states[names[0]] = state(target)
    pred = predictions(states, cfg)
    robots = [robot(Robot, reference_track, cfg, i, states[n], pred[n], target) for i, n in enumerate(names)]
    # Leader alone, entry at three cycles, exit at ten cycles, reacquisition,
    # sticky leader with a nearer seer, then leader loss and replacement.
    phases = [('far', 1), ('near', 3), ('far', 10), ('near', 3),
              ('sticky', 1), ('handoff', 1), ('lost', 1)]
    stamp = 6.
    for phase, cycles in phases:
        for _ in range(cycles):
            positions = [target.copy()]
            positions += [target + ([200 + 5*i, 150] if phase == 'far' else [-25 + 12*i, 28])
                          for i in range(1, UAV_COUNT)]
            if phase == 'sticky':
                positions[0] = target + [24, 0]
                positions[1] = target.copy()
            if phase == 'handoff':
                positions[0] = target + [30, 0]
                positions[1] = target.copy()
            if phase == 'lost':
                positions = [p + [100, 100] for p in positions]
            states = {n: state(p, stamp) for n, p in zip(names, positions)}
            pred = predictions(states, cfg)
            for i, r in enumerate(robots):
                r.state = np.r_[states[names[i]].position, states[names[i]].velocity]
                r.states_prediction = np.column_stack([pred[names[i]][:, :2], np.full(21, 30.),
                                                       pred[names[i]][:, 2:4], np.zeros(21)])
                r.time_stamp = stamp
                r.planner = SimpleNamespace(gmap=grid)
            seers = {n for n, s in states.items() if np.max(np.abs(s.position[:2] - target)) <= 25}
            # Freeze observation epoch for these mode-only probes.
            guidance = team.prepare_guidance(states, pred, track.stamp, seers)
            for i, r in enumerate(robots):
                r._update_mode_and_role(robots)
                r._slot = r._slot_target(robots) if r.mode == 'TRACK' and not r.is_leader_role else None
                n = names[i]
                assert guidance[n]['mode'] == r.mode
                assert guidance[n]['leader'] == r.is_leader_role
                assert team.leader == (None if r.leader_index is None else names[r.leader_index])
                assert team.c_in[n] == r.c_in and team.c_out[n] == r.c_out
                if r._slot is None:
                    assert guidance[n]['slot'] is None
                else:
                    np.testing.assert_allclose(guidance[n]['slot'], r._slot, atol=1e-12)
                    np.testing.assert_allclose(guidance[n]['moving_slot'], r._slot_trajectory(), atol=1e-12)
                    assert team.slot_scales[n] == pytest.approx(r.slot_scale, abs=1e-12)
                    assert team.assignment[n] == r._assign[i]
                np.testing.assert_allclose(guidance[n]['target'], r._target_trajectory(target), atol=1e-12)
                np.testing.assert_allclose(guidance[n]['goal'], r._compute_predicted_goal()[:2], atol=1e-12)
            stamp += cfg['TIMESTEP']


class RecordingSolver:
    def __init__(self, solver):
        self.solver, self.inputs = solver, None
        self.n_faces = solver.n_faces

    def solve(self, **inputs):
        self.inputs = copy.deepcopy(inputs)
        self.outcome = self.solver.solve(**inputs)
        return self.outcome

    def __getattr__(self, name):
        return getattr(self.solver, name)


@pytest.mark.parametrize('branch', ['search', 'distant_search', 'leader', 'path_slot', 'moving_slot'])
def test_assembled_solver_inputs_and_real_deterministic_outputs(branch, monkeypatch):
    cfg = configuration()
    assert 'ipopt.max_cpu_time' not in cfg['IPOPT_OPTIONS']
    Robot, Tracker, ReferenceMPC = oracle(cfg)
    track, rt = trackers(cfg, Tracker)
    target = track.observation
    names = [f'u{i}' for i in range(UAV_COUNT)]
    # A full fleet with identical measured positions, target measurements,
    # neighbor trajectories and occupied-grid/corridor inputs in each adapter.
    states = {n: state(target + [i * 8., 5.]) for i, n in enumerate(names)}
    if branch == 'distant_search':
        states = {n: state(target + [200 + i * 8., 150.]) for i, n in enumerate(names)}
    pred = predictions(states, cfg)
    team = TeamController(names, cfg, local_names=[names[0]])
    team.track = track
    r = robot(Robot, rt, cfg, 0, states[names[0]], pred[names[0]], target)
    robots = [r] + [robot(Robot, rt, cfg, i, states[n], pred[n], target) for i, n in enumerate(names[1:], 1)]
    r.mode = 'SEARCH' if branch.endswith('search') else 'TRACK'
    r.is_leader_role = branch == 'leader'
    r._track_goal = r._compute_predicted_goal()
    r.leader_state_pred = robots[1].states_prediction
    r.my_slot_cell, r.slot_scale = (0, 1), .7
    r._slot = target + [0., 18.] if 'slot' in branch else None
    start = states[names[0]].position[:2]
    path = np.array([start, start + [1., 2.], start + [40., 10.]]) if branch == 'path_slot' else np.array([start, start + [1., 1.]])
    r.traj_ref = path
    # Same nonempty map supplied to both corridor generators, independent of
    # physical ray integration differences (covered separately).
    points = np.array([start + [25., -10.], start + [30., 15.]])
    r.list_A, r.list_b = r.generateSafeCorridor(path, points)
    monkeypatch.setattr(team.planners[names[0]].gmap, 'surface_points', lambda *a: points)
    monkeypatch.setattr(team, '_path', lambda *a: (True, path))
    guide = dict(mode=r.mode, leader=r.is_leader_role, target=r._target_trajectory(target),
                 goal=r._track_goal[:2], slot=r._slot,
                 moving_slot=r._slot_trajectory() if r._slot is not None else None)
    r.mpc = RecordingSolver(ReferenceMPC(UAV_COUNT - 1, cfg['CORRIDOR_MAX_FACES'], cfg))
    r._fallback_control = lambda: np.zeros(3)
    # Two calls verify both nominal and backup warm starts, not just cold solves.
    for k in range(2):
        if k:
            # Independent static measured probe: discard corridor certificate to
            # avoid interpreting the deliberately unmoved state as a flight.
            team.stop_plans.clear()
        output = team.solve_vehicle(names[0], states, pred, guide, k * .1)
        r._control_parametric(robots, r.getNeighbors(robots), np.r_[target, 0.].reshape(1, 3))
        actual, expected = team.solver_inputs[names[0]], r.mpc.inputs
        assert actual.keys() == expected.keys()
        for key in actual:
            if actual[key] is None or expected[key] is None:
                assert actual[key] is expected[key], key
            else:
                a, b = actual[key], expected[key]
                # MPCProblem normalizes these transport layouts identically.
                if key in ('b_hard', 'b_cost'):
                    a, b = np.ravel(a), np.ravel(b)
                elif key == 'trk_goal':
                    a, b = np.asarray(a)[:2], np.asarray(b)[:2]
                elif key == 'others':
                    a, b = np.asarray(a)[..., :2], np.asarray(b)[..., :2]
                np.testing.assert_allclose(a, b, atol=1e-10, rtol=1e-12, err_msg=key)
        assert output['nominal_ok'] == r.mpc.outcome[0]
        if not output['nominal_ok']:
            assert output['solver_status'] == r.mpc.opti.stats()['return_status']
            break
        assert output['ok']
        assert output['reason'] == ('tracking' if guide['mode'] == 'TRACK' else 'search')
        np.testing.assert_allclose(output['prediction'], r._mpc_X, atol=1e-7, rtol=1e-7)
        np.testing.assert_allclose(output['acceleration'], r._mpc_U[0], atol=1e-7, rtol=1e-7)
        np.testing.assert_allclose(team.solvers[names[0]].last_backup[0], r.mpc.last_backup[0], atol=1e-7, rtol=1e-7)
        # Upstream commitState shifts these after all solves; do only the shifts.
        r._mpc_X = np.vstack([r._mpc_X[1:], r._mpc_X[-1:]])
        r._mpc_U = np.vstack([r._mpc_U[1:], r._mpc_U[-1:]])


def test_actual_local_planner_on_identical_occupied_grids():
    from upstream_oracle import definitions, disable_disk_caches
    from solo_mpc.core.planner_grid import LocalGridPlanner
    # Independent upstream planner definitions; relative imports resolved to
    # upstream definitions in memory rather than modifying sys.path or modules.
    jps = definitions('planner_jps.py', {})
    tree_source = (REFERENCE / 'planner_grid.py').read_text()
    import ast
    tree = ast.parse(tree_source)
    tree.body = [n for n in tree.body if not (isinstance(n, ast.ImportFrom) and n.module == 'planner_jps')]
    ns = {k: jps[k] for k in ('PersistentLogOddsGrid', '_dilate_disk', '_string_pull')}
    ns['__name__'] = 'upstream_grid_offline'
    exec(compile(disable_disk_caches(tree), str(REFERENCE / 'planner_grid.py'), 'exec'), ns)
    cfg = configuration()
    for i in range(UAV_COUNT):
        kwargs = dict(world_bounds=cfg['WORLD_BOUNDS'], grid_resolution=cfg['GRID_RESOLUTION'],
                      inflate_radius=cfg['INFLATE_RADIUS'], sensing_radius=cfg['SENSING_RADIUS'],
                      local_radius=cfg['LOCAL_PLAN_RADIUS'], unknown_policy=cfg['UNKNOWN_POLICY'],
                      unknown_cost=cfg['UNKNOWN_COST'], goal_clamp_margin=cfg['GOAL_CLAMP_MARGIN'],
                      start_snap_radius=cfg['START_SNAP_RADIUS'], search=cfg['LOCAL_SEARCH'])
        a, b = LocalGridPlanner(**kwargs), ns['LocalGridPlanner'](**kwargs)
        for planner in (a, b):
            gm = planner.gmap
            gm.L[:] = -4
            gm.L[50:65, 60:65] = 4
            planner.initialize([100 + i * 3, 100], [160, 140])
        ra = a.plan(np.empty((0, 2)), cfg['ROBOT_RADIUS'], observe=False)
        rb = b.plan(np.empty((0, 2)), cfg['ROBOT_RADIUS'], observe=False)
        assert ra[0] == rb[0] and ra[0]
        np.testing.assert_allclose(ra[1], rb[1], atol=1e-12)


def test_contingency_fallback_matches_upstream_for_all_fleet_members():
    from solo_mpc.core.controller import StopPlan
    from solo_mpc.core.dynamics import model
    cfg = configuration()
    Robot, Tracker, _ = oracle(cfg)
    names = [f'u{i}' for i in range(UAV_COUNT)]
    team = TeamController(names, cfg, local_names=[])
    A, B = model(cfg['TIMESTEP'], cfg['D_FRAC'], cfg['ACCEL_TAU'], 2)
    for i, n in enumerate(names):
        # A deterministic dynamically consistent, terminally stationary backup
        # is sufficient to compare fallback indexing independently of IPOPT.
        U = np.zeros((cfg['BACKUP_HORIZON'], 2))
        X = [np.array([100. + 10*i, 100., 0., 0., 0., 0.])]
        for u in U:
            X.append(A @ X[-1] + B @ u)
        X = np.asarray(X)
        team.stop_plans[n] = StopPlan(X, U, 0.)
        s = state(X[0, :2], velocity=(0., 0.))
        r = robot(Robot, Tracker(), cfg, i, s, np.tile(X[0], (21, 1)), [100., 100.])
        r._backup, r._bk_i, r.n_control = (X, U), 0, 3
        # Make nonzero controls distinguishable and solve the final two
        # controls for exactly zero terminal velocity/acceleration.
        U[:3, 0] = [.2, -.1, .05]
        for k in range(len(U) - 2):
            X[k+1] = A @ X[k] + B @ U[k]
        matrix = np.column_stack([(A @ B)[[2, 4], 0], B[[2, 4], 0]])
        U[-2:, 0] = np.linalg.solve(matrix, -(A @ A @ X[-3])[[2, 4]])
        for k in range(len(U)-2, len(U)):
            X[k+1] = A @ X[k] + B @ U[k]
        np.testing.assert_allclose(X[-1, 2:], 0., atol=1e-12)
        for k in range(1, len(U) + 3):
            x = X[min(k, len(X)-1)]
            s = VehicleState(np.r_[x[:2], 30.], np.r_[x[2:4], 0.], np.r_[x[4:], 0.],
                             np.array([0.,0.,0.,1.]), k*.1)
            actual = team._fallback(n, s, k*.1, 'mpc_failed')
            expected = r._backup_control()
            assert actual['ok'] and not actual['nominal_ok']
            np.testing.assert_allclose(actual['acceleration'], expected[:2], atol=1e-12)
            np.testing.assert_allclose(actual['prediction'][:, :2], r.states_prediction[:, :2], atol=1e-12)
        # A real FCU departure must invalidate the otherwise identical tail.
        s.position[0] += 10
        assert not team._fallback(n, s, k*.1, 'mpc_failed')['ok']


def test_dynamics_are_unchanged():
    # Solver behavior with recovery disabled is compared numerically above.
    # Physical overspeed recovery intentionally extends the upstream constraints.
    from solo_mpc.core import dynamics
    assert (REFERENCE / 'dynamics.py').read_bytes() == Path(dynamics.__file__).read_bytes()


def test_assignment_keeps_small_improvement_and_switches_for_large_improvement():
    cfg = configuration()
    cfg['SLOT_CONTRACT'] = False
    # Opposite follower slots make the assignment improvement measurable.
    cfg['GRID_CELLS'] = [[0, 1], [0, -1]]
    Robot, Tracker, _ = oracle(cfg)
    names = [f'u{i}' for i in range(UAV_COUNT)]
    lead = np.array([100., 100.])
    team = TeamController(names, cfg, local_names=[])
    team.modes = dict.fromkeys(names, 'TRACK')
    team.assignment = {n: i for i, n in enumerate(names[1:])}
    cells = np.asarray(cfg['GRID_CELLS'])
    offsets = cells * cfg['SLOT_SPACING_RATIO'] * cfg['VIEWING_RADIUS']
    r = Robot.__new__(Robot)
    shared = {'assign': {i+1: i for i in range(UAV_COUNT-1)}, 'key': (0, tuple(range(1,UAV_COUNT)), UAV_COUNT-1)}
    for amount, expected_pair in [(.5, (0, 1)), (10., (1, 0))]:
        positions = [lead, lead + [0., -amount], lead + [0., amount]]
        states = {n: state(p) for n, p in zip(names, positions)}
        pred = predictions(states, cfg)
        team.coordinate(states, pred, lead, {names[0]}, [])
        actual = (team.assignment[names[1]], team.assignment[names[2]])
        assert actual == expected_pair
        expected = r._assign_slots(list(range(1,UAV_COUNT)), {i: positions[i] for i in range(1,UAV_COUNT)},
                                   lead + offsets, UAV_COUNT-1, shared)
        assert team.assignment == {names[i]: slot for i, slot in expected.items()}
        shared['assign'] = expected
