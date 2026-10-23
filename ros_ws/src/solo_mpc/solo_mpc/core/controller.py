"""Measured-state team NMPC. No ROS imports and no simulated state integration.

The imported solver/dynamics/grid algorithm come from motion_planning_MPC.
Team leadership, assignment, mapping, reference selection and failures live here
so they can be modified independently of transport and vehicle operation.
"""

from dataclasses import dataclass
import time

import _pydecomp as pdc
import numpy as np
from .coordination import FleetGuidance, path_carrot

from .mapping import SensorGrid, warmup_scan_integration
from .mpc_problem import MPCProblem
from .planner_grid import LocalGridPlanner, warmup
from .tracking import TargetTracker

# The pinned pydecomp wheel exposes its computation in _pydecomp. Import the
# binding directly: its package initializer also loads plotting libraries,
# which are unnecessary for a headless ROS controller.


@dataclass
class VehicleState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    quaternion: np.ndarray
    stamp: float


@dataclass
class StopPlan:
    states: np.ndarray
    controls: np.ndarray
    stamp: float


class TeamController(FleetGuidance):
    def __init__(self, names, cfg, local_names=None):
        super().__init__(names, dict(cfg))
        self.local_names = self.names if local_names is None else list(local_names)
        if not set(self.local_names) <= set(self.names):
            raise ValueError("Local vehicles must belong to the fleet")
        self.track = TargetTracker(cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
        self.guidance = {}
        self.plan_failures = dict.fromkeys(names, 0)
        self.solver_inputs = {}
        self.planners, self.solvers, self.warm, self.paths = {}, {}, {}, {}
        self.path_goals, self.path_age = {}, {}
        self.stop_plans, self.active_corridors = {}, {}
        self.corridor_cost = {}
        self.last_scans = {n: -np.inf for n in names}
        for n in self.local_names:
            planner = LocalGridPlanner(
                cfg["WORLD_BOUNDS"],
                grid_resolution=cfg["GRID_RESOLUTION"],
                inflate_radius=cfg["INFLATE_RADIUS"],
                sensing_radius=cfg["SENSING_RADIUS"],
                local_radius=cfg["LOCAL_PLAN_RADIUS"],
                unknown_policy=cfg["UNKNOWN_POLICY"],
                unknown_cost=cfg["UNKNOWN_COST"],
                goal_clamp_margin=cfg["GOAL_CLAMP_MARGIN"],
                start_snap_radius=cfg.get("START_SNAP_RADIUS", 10.0),
                search=cfg["LOCAL_SEARCH"],
            )
            planner.gmap = SensorGrid(
                cfg["WORLD_BOUNDS"],
                resolution=cfg["GRID_RESOLUTION"],
                inflate_radius=cfg["INFLATE_RADIUS"],
                sensing_radius=cfg["SENSING_RADIUS"],
                treat_unknown_as_blocked=cfg["UNKNOWN_POLICY"] == "blocked",
            )
            self.planners[n] = planner
            self.solvers[n] = MPCProblem(len(names) - 1, cfg["CORRIDOR_MAX_FACES"], cfg)
        self.predictions = {n: None for n in names}
        self.prediction_stamps = {n: -np.inf for n in names}
        if self.local_names:
            warmup()
            warmup_scan_integration()

    def prepare_guidance(self, states, predictions, stamp, seers, tracker=None):
        track = self.track if tracker is None else tracker
        slots = self.coordinate(states, predictions, track.position(stamp), seers,
                                [p.gmap for p in self.planners.values()])
        target = track.trajectory(stamp, self.cfg['HORIZON_LENGTH'],
                                  self.cfg['TIMESTEP'], 2 * self.cfg['VMAX'])
        self.guidance = {
            n: dict(mode=self.modes[n], leader=n == self.leader, target=target,
                    goal=track.pursuit(states[n].position, stamp, self.cfg),
                    slot=slots.get(n), moving_slot=self.moving_slot(n, predictions) if n in slots else None)
            for n in self.names
        }
        return self.guidance

    def _path(self, name, position, goal):
        planner = self.planners[name]
        previous = self.paths.get(name)
        refresh = (
            previous is None or len(previous) < 2
            or self.path_age.get(name, 0) >= self.cfg["MAX_COMMIT_AGE"]
        )
        refresh |= (
            name not in self.path_goals
            or np.linalg.norm(goal - self.path_goals.get(name, goal))
            > self.cfg["TARGET_REPLAN_THRESHOLD"]
        )
        if not refresh:
            points = np.asarray(previous, float)
            nearest = self._path_segment(points, position[:2])
            if nearest + 2 < len(points) and np.linalg.norm(points[nearest + 1] - position[:2]) < 1e-6:
                nearest += 1
            trimmed = np.vstack(
                [position[:2], points[min(nearest + 1, len(points) - 1) :]]
            )
            delta = points[nearest + 1] - points[nearest]
            along = np.clip(np.dot(position[:2] - points[nearest], delta)
                            / max(np.dot(delta, delta), 1e-12), 0, 1)
            deviation = np.linalg.norm(position[:2] - points[nearest] - along * delta)
            refresh = deviation > self.cfg['PATH_DEVIATION_THRESHOLD'] or not all(
                planner.gmap.segment_clear(a, b)
                for a, b in zip(trimmed[:-1], trimmed[1:])
            )
            if not refresh:
                self.path_age[name] += 1
                return True, trimmed.tolist()
        planner.initialize(position, goal)
        ok, path, *_ = planner.plan(
            np.empty((0, 2)), self.cfg["ROBOT_RADIUS"], observe=False
        )
        if ok and len(path) >= 2:
            self.plan_failures[name] = 0
            self.path_goals[name] = np.asarray(goal).copy()
            self.path_age[name] = 0
            self.paths[name] = path
            return True, path
        self.plan_failures[name] += 1
        if previous is not None and len(previous) >= 2:
            points = np.asarray(previous, float)
            k = self._path_segment(points, position[:2])
            trimmed = np.vstack([position[:2], points[k + 1:]])
            if all(planner.gmap.segment_clear(a, b) for a, b in zip(trimmed[:-1], trimmed[1:])):
                self.path_age[name] = self.path_age.get(name, 0) + 1
                return True, trimmed.tolist()
        if self.plan_failures[name] < self.cfg['MAX_FAIL_BEFORE_STOP']:
            return True, [position[:2].copy(), position[:2].copy()]
        return False, []

    @staticmethod
    def _path_segment(points, position):
        """Project onto segments so trimming a committed path cannot skip a turn."""
        delta = points[1:] - points[:-1]
        along = np.clip(np.sum((position - points[:-1]) * delta, axis=1)
                        / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0, 1)
        return int(np.argmin(np.linalg.norm(points[:-1] + along[:, None] * delta
                                           - position, axis=1)))

    def _corridor_seed(self, position, path):
        points = np.asarray(path, float)[:, :2]
        pos = np.asarray(position[:2], float)
        k = self._path_segment(points, pos)
        nxt = points[k + 1]
        if np.linalg.norm(nxt - pos) < 1e-6 and k + 2 < len(points):
            nxt = points[k + 2]
        delta = nxt - pos
        length = np.linalg.norm(delta)
        if length < 1e-6:
            delta, length = np.array([self.cfg["GRID_RESOLUTION"], 0.0]), self.cfg["GRID_RESOLUTION"]
        delta = delta * min(1.0, self.cfg["CORRIDOR_SEED_MAX"] / length)
        return np.vstack([pos, pos + delta])

    @staticmethod
    def _contains(A, b, points, margin):
        return bool(np.all(np.asarray(points).reshape(-1, 2) @ A.T <= b - margin + 1e-6))

    def _stop_tail(self, name, state, stamp):
        """Use simulation time and measured-state agreement, never a solve counter.

        A model certificate cannot justify replay after the FCU deviates from it.
        The publisher's existing expiry/hold behavior handles a rejected tail.
        """
        plan = self.stop_plans.get(name)
        if plan is None:
            return None
        elapsed = (stamp - plan.stamp) / self.cfg["TIMESTEP"]
        if not np.isfinite(elapsed) or elapsed < -1e-6:
            self.stop_plans.pop(name, None)
            return None
        index = min(int(np.floor(elapsed + 1e-6)), len(plan.states) - 1)
        measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
        for values, expected, key in zip(
            measured, np.split(plan.states[index], 3),
            ("BACKUP_POSITION_TOLERANCE", "BACKUP_VELOCITY_TOLERANCE", "BACKUP_ACCEL_TOLERANCE"),
        ):
            if not np.isfinite(values).all() or np.linalg.norm(values - expected) > self.cfg[key]:
                self.stop_plans.pop(name, None)
                return None
        return plan, index

    def _select_corridor(self, name, state, stamp, candidates):
        previous = self.active_corridors.get(name)
        tail = self._stop_tail(name, state, stamp)
        if tail is not None:
            plan, index = tail
            for A, b in candidates:
                if self._contains(A, b, plan.states[index:, :2], self.cfg["CORRIDOR_MARGIN"]):
                    return A, b
            if previous is not None:
                return previous
        for A, b in candidates:
            if self._contains(A, b, state.position[:2], 0.0):
                return A, b
        if previous is not None and self._contains(*previous, state.position[:2], 0.0):
            return previous
        if not candidates:
            raise ValueError("Corridor decomposition returned no finite polytope")
        return min(candidates, key=lambda ab: np.max(ab[0] @ state.position[:2] - ab[1]))

    def _corridor(self, name, state, path, stamp):
        cfg = self.cfg
        position = state.position
        points = self.planners[name].gmap.surface_points(
            position, cfg["SENSING_RADIUS"] + 1.0
        )
        box = cfg["CORRIDOR_BOX"]
        if not len(points):
            A = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
            candidate = (A, A @ position[:2] + cfg['SENSING_RADIUS'] / np.sqrt(2))
            self.corridor_cost[name] = candidate
            return self._select_corridor(name, state, stamp, [candidate])
        segment = self._corridor_seed(position, path)
        try:
            As, bs = pdc.convex_decomposition_2D(points, segment, np.array([[box, box]]))
        except Exception:
            # Upstream retains the previous static corridor if decomposition
            # fails; an absent previous corridor still fails closed below.
            As, bs = [], []
        valid = [
            (np.asarray(A), np.asarray(b).ravel())
            for A, b in zip(As, bs)
            if len(b) and np.isfinite(A).all() and np.isfinite(b).all()
        ]
        self.corridor_cost[name] = valid[0] if valid else (None, None)
        return self._select_corridor(name, state, stamp, valid)

    def _fallback(self, name, state, stamp, reason, **metrics):
        self.warm.pop(name, None)
        tail = self._stop_tail(name, state, stamp)
        if tail is None:
            return {"ok": False, "nominal_ok": False, "reason": reason, **metrics}
        plan, index = tail
        H = self.cfg["HORIZON_LENGTH"]
        rows = plan.states[index:index + H + 1]
        rows = np.vstack([rows, np.repeat(rows[-1:], max(0, H + 1 - len(rows)), axis=0)])
        control = plan.controls[index] if index < len(plan.controls) else np.zeros(2)
        controls = plan.controls[index:index + H]
        controls = np.vstack([controls, np.zeros((H - len(controls), 2))])
        self.predictions[name] = rows.copy()
        self.prediction_stamps[name] = stamp
        return {"ok": True, "nominal_ok": False, "position": rows[1, :2].copy(),
                "velocity": rows[1, 2:4].copy(), "acceleration": control.copy(),
                "prediction": rows.copy(), "controls": controls,
                "reason": f"stop_plan: {reason}", **metrics}

    def integrate_scans(self, scans):
        """Update only maps owned by this instance."""
        for n in self.local_names:
            if scans[n].stamp > self.last_scans[n]:
                self.planners[n].gmap.integrate(scans[n])
                self.last_scans[n] = scans[n].stamp

    def neighbor_predictions(self, states, stamp):
        """Age-align cached plans to measured state, frozen before any solve."""
        H, dt = self.cfg['HORIZON_LENGTH'], self.cfg['TIMESTEP']
        predictions = {}
        for n, state in states.items():
            old = self.predictions[n]
            age = stamp - self.prediction_stamps[n]
            if old is None or age < 0 or age > 3 * dt:
                rows = np.tile(np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]], (H + 1, 1))
                rows[:, :2] += np.arange(H + 1)[:, None] * dt * state.velocity[:2]
            else:
                steps = min(H, max(0, int(np.floor(age / dt + 1e-6))))
                rows = np.vstack([old[steps:], np.repeat(old[-1:], steps, axis=0)])
                rows = rows.copy()
                rows[:, :2] += state.position[:2] - rows[0, :2]
                rows[0] = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
            predictions[n] = rows
        return predictions

    def step(self, states, scans, stamp, seers, tracker=None):
        self.integrate_scans(scans)
        predictions = self.neighbor_predictions(states, stamp)
        guidance = self.prepare_guidance(states, predictions, stamp, seers, tracker)
        return {n: self.solve_vehicle(n, states, predictions, guidance[n], stamp)
                for n in self.local_names}

    def solve_vehicle(self, n, states, predictions, guidance, stamp):
        """Local planning and solve against an immutable fleet-cycle snapshot."""
        cfg = self.cfg
        H = cfg["HORIZON_LENGTH"]
        state = states[n]
        goal = guidance['goal'] if guidance['slot'] is None else guidance['slot']
        started = time.perf_counter()
        ok, path = self._path(n, state.position, goal)
        metrics = {"path_ms": (time.perf_counter() - started) * 1000}
        if not ok or len(path) < 2:
            return self._fallback(n, state, stamp, "planner_failed", **metrics)
        started = time.perf_counter()
        try:
            A, b = self._corridor(n, state, path, stamp)
            faces = max(len(b), 0 if self.corridor_cost[n][1] is None else len(self.corridor_cost[n][1]))
            if faces > self.solvers[n].n_faces:
                self.solvers[n] = MPCProblem(len(self.names) - 1, max(cfg["CORRIDOR_MAX_FACES"], 16 * int(np.ceil(faces / 16))), cfg)
                self.warm.pop(n, None)
        except Exception as exc:
            return self._fallback(n, state, stamp, f"corridor_failed: {exc}", **metrics)
        metrics["corridor_ms"] = (time.perf_counter() - started) * 1000
        x0 = np.r_[state.position[:2], state.velocity[:2], state.acceleration[:2]]
        X, U = self.warm.get(n, (np.tile(x0, (H + 1, 1)), np.zeros((H, 2))))
        others = [k for k in self.names if k != n]
        slot = np.zeros(2)
        if guidance['slot'] is not None:
            carrot = path_carrot(state.position, path, cfg['VMAX'] * H * cfg['TIMESTEP'])
            slot = carrot if carrot is not None else guidance['moving_slot']
        guide = len(path) > 2
        inputs = dict(
            x0=x0, A_hard=A, b_hard=b,
            A_cost=self.corridor_cost[n][0], b_cost=self.corridor_cost[n][1],
            use_corr_cost=self.corridor_cost[n][0] is not None,
            others=[predictions[k][:, :2] for k in others],
            nb_flags=[float(np.linalg.norm(states[k].position - state.position)
                            < cfg['SENSING_NEIGHBOR']) for k in others],
            tgt=guidance['target'], w_search=cfg['W_search_track'] if guidance['mode'] == 'SEARCH' else 0.0,
            slot=slot, w_slot=cfg['W_sat_slot'] if guidance['slot'] is not None else 0.0,
            leader=guidance['leader'], ref=path[1] if guide else np.zeros(2),
            trk_goal=guidance['goal'], s_ref=float(guide), X_init=X, U_init=U,
            w_tra=0.0 if guidance['slot'] is not None else cfg['W_tra'],
        )
        self.solver_inputs[n] = inputs
        from .diagnostics import SolverCapture
        capture = SolverCapture(cfg, n, self.solvers[n], stamp, inputs)
        ok, X, U = self.solvers[n].solve(**inputs)
        capture.finish(ok, X, U, self.solvers[n])
        try:
            stats = self.solvers[n].opti.stats()
            solver_status = stats.get("return_status", "unknown")
            metrics["solver_iterations"] = stats.get("iter_count", 0)
        except RuntimeError:
            solver_status = "unavailable"
        solver_ms = self.solvers[n].solve_times[-1] * 1000
        if not ok or not np.isfinite(X).all() or not np.isfinite(U).all():
            return self._fallback(n, state, stamp, "mpc_failed", solver_status=solver_status,
                                  solver_ms=solver_ms, **metrics)
        self.active_corridors[n] = (A.copy(), b.copy())
        backup = self.solvers[n].last_backup if cfg["CONTINGENCY"] else None
        if backup is not None:
            self.stop_plans[n] = StopPlan(backup[0].copy(), backup[1].copy(), stamp)
        elif cfg["TERMINAL_STOP"]:
            self.stop_plans[n] = StopPlan(X.copy(), U.copy(), stamp)
        else:
            self.stop_plans.pop(n, None)
        self.predictions[n] = X.copy()
        self.prediction_stamps[n] = stamp
        self.warm[n] = (np.vstack([X[1:], X[-1:]]), np.vstack([U[1:], U[-1:]]))
        return {
            "ok": True,
            "nominal_ok": True,
            "position": X[1, :2],
            "velocity": X[1, 2:4],
            "acceleration": U[0],
            "prediction": X,
            "controls": U,
            "reason": "tracking" if guidance["mode"] == "TRACK" else "search",
            "controller_mode": guidance["mode"],
            "solver_status": solver_status,
            "solver_ms": solver_ms,
            **metrics,
        }
