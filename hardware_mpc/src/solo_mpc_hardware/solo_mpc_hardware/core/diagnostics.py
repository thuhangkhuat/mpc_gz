"""Bounded, atomic optimizer-input capture; no ROS or flight execution.

A pending record is persisted BEFORE IPOPT so worker termination still leaves
all optimizer inputs. The backup warm start is solver state, not a solve argument.
"""

import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    return value


class SolverCapture:
    _sequences = {}

    def __init__(self, cfg, name, solver, stamp, inputs):
        self.path = None
        directory = cfg.get('DIAGNOSTIC_DIR')
        if not directory:
            return
        limit = cfg.get('DIAGNOSTIC_CAPTURE_CYCLES', 128)
        if limit <= 0:
            return
        key = (str(directory), name)
        sequence = self._sequences.get(key, 0) + 1
        self._sequences[key] = sequence
        root = Path(directory, name)
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / f'solve-{sequence % limit:04d}.json'
        self.record = serializable(dict(
            schema=1, sequence=sequence, stamp=stamp, wall=time.time(), vehicle=name,
            cfg=cfg, n_others=solver.n_others, n_faces=solver.n_faces,
            inputs=inputs, backup_init=solver._bk_init, outcome='pending',
            sources={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in Path(__file__).parent.glob('*.py')},
        ))
        self._write()

    def _write(self):
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.record, allow_nan=False))
        os.replace(temporary, self.path)

    def finish(self, ok, X, U, solver):
        if self.path is not None:
            self.record.update(outcome='success' if ok else 'failure', X=serializable(X),
                               U=serializable(U), backup=serializable(solver.last_backup))
            try:
                self.record['stats'] = serializable(solver.opti.stats())
            except RuntimeError:
                pass
            self._write()


def replay(path, iteration_only=False):
    """Replay a complete capture using its solver options; never starts ROS."""
    from .mpc_problem import MPCProblem
    record = json.loads(Path(path).read_text())
    if record.get('schema') != 1:
        raise ValueError('Unsupported optimizer capture schema')
    if iteration_only:
        record['cfg']['IPOPT_OPTIONS'].pop('ipopt.max_cpu_time', None)
    solver = MPCProblem(record['n_others'], record['n_faces'], record['cfg'])
    solver._bk_init = (None if record['backup_init'] is None else
                       tuple(np.asarray(v) for v in record['backup_init']))
    inputs = record['inputs']
    for key in ('X_init', 'U_init'):
        inputs[key] = np.asarray(inputs[key], float)
    return solver.solve(**inputs)
