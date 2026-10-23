"""One persistent spawned process per UAV, with coherent fleet-cycle snapshots.

Workers own maps, planners, corridors, solvers and warm starts. Only formation
assignment and prediction exchange remain in the coordinator. No ROS objects or
CasADi objects cross a process boundary. Spawn avoids inheriting DDS/native state.
"""

from dataclasses import dataclass
import math
import multiprocessing as mp
from multiprocessing.connection import wait
import os
import time
import threading
from types import SimpleNamespace

from .controller import TeamController


class MapSnapshot:
    """Read-only inflated map for the existing formation-slot checks."""

    def __init__(self, cfg):
        self.ox, self.oy = cfg["WORLD_BOUNDS"][:2]
        self.resolution = cfg["GRID_RESOLUTION"]
        self.blocked = None
        self.occupied = None
        self.unknown = None

    def world_to_grid(self, wx, wy):
        return int(math.floor((wx - self.ox) / self.resolution)), int(math.floor((wy - self.oy) / self.resolution))

    def is_blocked(self, ix, iy):
        if self.blocked is None:
            raise RuntimeError("Formation map has not been received")
        height, width = self.blocked.shape
        return not (0 <= ix < width and 0 <= iy < height) or bool(self.blocked[iy, ix])


def vehicle_worker(connection, name, names, cfg):
    """Process entry point; the fleet name list retains all neighbor constraints."""
    try:
        controller = TeamController(names, cfg, local_names=[name])
        grid = controller.planners[name].gmap
        connection.send({"phase": "ready", "pid": os.getpid(), "vehicle": name})
        while True:
            job = connection.recv()
            phase, cycle = job["phase"], job["cycle"]
            if phase == "stop":
                return
            started = time.perf_counter()
            try:
                if phase == "map":
                    controller.integrate_scans({name: job["scan"]})
                    # Inflation belongs to the worker as well as ray integration.
                    grid._ensure()
                    payload = {"blocked": grid.blocked, "occupied": grid.L >= grid.occ_logit,
                               "unknown": grid.unknown}
                elif phase == "solve":
                    payload = {"result": controller.solve_vehicle(
                        name, job["states"], job["predictions"], job["guidance"], job["stamp"],
                    ), "n_faces": controller.solvers[name].n_faces}
                else:
                    raise ValueError(f"Unknown worker phase: {phase}")
                connection.send({"phase": phase, "cycle": cycle, "pid": os.getpid(),
                                 "started_wall": started, "finished_wall": time.perf_counter(),
                                 **payload})
            except Exception as exc:
                connection.send({"phase": phase, "cycle": cycle, "pid": os.getpid(),
                                 "error": f"{type(exc).__name__}: {exc}"})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({"phase": "ready", "error": f"{type(exc).__name__}: {exc}"})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


@dataclass
class Worker:
    process: object
    connection: object
    pid: int | None = None
    error: str | None = None


class ParallelTeamController(TeamController):
    """Same measured-state interface as TeamController; local work is concurrent."""

    def __init__(self, names, cfg, timeout=1.0, startup_timeout=30.0, sequential=False):
        super().__init__(names, cfg, local_names=[])
        self.timeout = timeout
        self.sequential = sequential
        self.workers = {}
        self.vehicle_metrics = {}
        self.cycle = 0
        self.closed = False
        self._cycle_lock = threading.Lock()
        self.planners = {n: SimpleNamespace(gmap=MapSnapshot(cfg)) for n in names}
        context = mp.get_context("spawn")
        try:
            for n in names:
                parent, child = context.Pipe()
                process = context.Process(
                    target=vehicle_worker, args=(child, n, self.names, self.cfg),
                    name=f"mpc-{n}", daemon=True,
                )
                process.start()
                child.close()
                self.workers[n] = Worker(process, parent)
            ready = self._collect(set(names), "ready", None, startup_timeout)
            for n, message in ready.items():
                self.workers[n].pid = message.get("pid")
                if "error" in message:
                    raise RuntimeError(f"{n}: worker initialization failed: {message['error']}")
        except BaseException:
            self.close()
            raise

    def _disable(self, name, reason):
        worker = self.workers[name]
        worker.error = reason
        worker.connection.close()
        if worker.process.is_alive():
            worker.process.terminate()
        worker.process.join(timeout=0.05)
        if worker.process.is_alive():
            worker.process.kill()
            worker.process.join(timeout=0.1)

    def _collect(self, names, phase, cycle, timeout):
        """Collect whichever worker finishes first; one deadline for the phase."""
        pending = {self.workers[n].connection: n for n in names}
        messages = {}
        deadline = time.perf_counter() + timeout
        while pending:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            for connection in wait(list(pending), timeout=remaining):
                name = pending.pop(connection)
                try:
                    message = connection.recv()
                    if message.get("phase") != phase or (
                        cycle is not None and message.get("cycle") != cycle
                    ):
                        raise ValueError("Worker response does not match the active cycle")
                    messages[name] = message
                except (EOFError, OSError, ValueError) as exc:
                    reason = f"worker_disconnected: {exc}"
                    self._disable(name, reason)
                    messages[name] = {"error": reason}
        for name in pending.values():
            reason = f"worker_timeout: {phase}"
            self._disable(name, reason)
            messages[name] = {"error": reason}
        return messages

    def _dispatch(self, phase, jobs):
        if self.sequential and phase == 'solve':
            deadline = time.perf_counter() + self.timeout
            messages = {}
            for name, job in jobs.items():
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    messages[name] = {'error': 'fleet_solve_deadline'}
                else:
                    messages.update(self._dispatch_batch(phase, {name: job}, remaining))
            return messages
        return self._dispatch_batch(phase, jobs, self.timeout)

    def _dispatch_batch(self, phase, jobs, timeout):
        pending, messages = set(), {}
        for n, job in jobs.items():
            worker = self.workers[n]
            if worker.error:
                messages[n] = {"error": worker.error}
                continue
            try:
                worker.connection.send({"phase": phase, "cycle": self.cycle, **job})
                pending.add(n)
            except (BrokenPipeError, EOFError, OSError) as exc:
                reason = f"worker_disconnected: {exc}"
                self._disable(n, reason)
                messages[n] = {"error": reason}
        messages.update(self._collect(pending, phase, self.cycle, timeout))
        return messages

    def step(self, states, scans, stamp, seers, tracker=None):
        if not self._cycle_lock.acquire(blocking=False):
            raise RuntimeError('A fleet cycle is already in progress')
        try:
            return self._step(states, scans, stamp, seers, tracker)
        finally:
            self._cycle_lock.release()

    def _step(self, states, scans, stamp, seers, tracker=None):
        if self.closed:
            raise RuntimeError("Parallel controller is closed")
        self.cycle += 1
        self.vehicle_metrics = {}
        # Capture predictions before dispatch; no within-cycle solve feeds another.
        predictions = self.neighbor_predictions(states, stamp)
        mapped = self._dispatch("map", {n: {"scan": scans[n]} for n in self.names})
        if any("error" in m for m in mapped.values()):
            # Assignment requires all maps. Do not use a missing/stale map as free.
            return {n: {"ok": False, "reason": mapped[n].get("error", "fleet_mapping_failed")}
                    for n in self.names}
        for n, message in mapped.items():
            for key in ('blocked', 'occupied', 'unknown'):
                setattr(self.planners[n].gmap, key, message[key])
        guidance = self.prepare_guidance(states, predictions, stamp, seers, tracker)
        solved = self._dispatch("solve", {n: dict(states=states, predictions=predictions,
                                                 guidance=guidance[n], stamp=stamp) for n in self.names})
        results = {}
        for n, message in solved.items():
            if "error" in message:
                results[n] = {"ok": False, "reason": message["error"]}
                continue
            result = message["result"]
            self.vehicle_metrics[n] = {
                "worker_pid": message["pid"], "cycle": self.cycle,
                "mapping_ms": (mapped[n]["finished_wall"]-mapped[n]["started_wall"])*1000,
                "planning_and_solve_ms": (message["finished_wall"]-message["started_wall"])*1000,
                "map_started_wall": mapped[n]["started_wall"],
                "map_finished_wall": mapped[n]["finished_wall"],
                "solve_started_wall": message["started_wall"],
                "solve_finished_wall": message["finished_wall"],
                "corridor_faces": message["n_faces"],
            }
            result.update(self.vehicle_metrics[n])
            results[n] = result
            if result["ok"]:
                self.predictions[n] = result["prediction"].copy()
                self.prediction_stamps[n] = stamp
        return {n: results[n] for n in self.names}

    def close(self):
        """Wait for the bounded active cycle before closing its pipes."""
        with self._cycle_lock:
            self._close()

    def _close(self):
        """Bounded shutdown, including workers stalled inside a native solve."""
        if self.closed:
            return
        self.closed = True
        for worker in self.workers.values():
            if not worker.error:
                try:
                    worker.connection.send({"phase": "stop", "cycle": self.cycle})
                except (BrokenPipeError, EOFError, OSError):
                    pass
        deadline = time.perf_counter() + 1.0
        for worker in self.workers.values():
            worker.process.join(timeout=max(0, deadline-time.perf_counter()))
            if worker.process.is_alive():
                worker.process.terminate()
                worker.process.join(timeout=0.1)
                if worker.process.is_alive():
                    worker.process.kill()
                    worker.process.join(timeout=0.1)
            worker.connection.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
