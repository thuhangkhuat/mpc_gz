"""Read upstream definitions without importing its simulator or writing bytecode."""

import ast
import json
import logging
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import _pydecomp as pdc
from scipy.optimize import linear_sum_assignment

REFERENCE = Path(os.environ['MPC_REFERENCE']) if os.environ.get('MPC_REFERENCE') else None
REFERENCE_RUN = Path(os.environ['MPC_REFERENCE_RUN']) if os.environ.get('MPC_REFERENCE_RUN') else None


def disable_disk_caches(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            for keyword in node.keywords:
                if keyword.arg == 'cache':
                    keyword.value = ast.Constant(False)
    return ast.fix_missing_locations(tree)


def definitions(file, namespace, names=None):
    tree = ast.parse((REFERENCE / file).read_text(), filename=str(REFERENCE / file))
    if names is not None:
        tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    else:
        tree.body = [n for n in tree.body if not (isinstance(n, ast.Import)
                                                and any(a.name == 'dynamics' for a in n.names))]
    exec(compile(disable_disk_caches(tree), str(REFERENCE / file), 'exec'), namespace)
    return namespace


def configuration():
    # config.json only saved isupper() names: recover mixed-case weights from
    # inspected source defaults + the saved scenario's explicit overrides.
    source = (REFERENCE / 'config.py').read_text()
    namespace = {}
    exec(source[:source.index('# 2. SCENARIOS')], namespace)
    cfg = {k: v for k, v in namespace.items() if k.isupper() or k.startswith('W_')}
    saved = json.loads((REFERENCE_RUN / 'config.json').read_text())
    cfg.update(saved['scenario_def']['params'])
    cfg.update(saved['params'])
    # ROS-only validation tolerances do not enter the numerical formulation.
    cfg.update(BACKUP_POSITION_TOLERANCE=.15, BACKUP_VELOCITY_TOLERANCE=.3, BACKUP_ACCEL_TOLERANCE=.5)
    return cfg


def oracle(cfg):
    dynamics = SimpleNamespace(**definitions('dynamics.py', {}))
    numerical = definitions('mpc_problem.py', {'dynamics': dynamics})['MPCProblem']
    tracker = definitions('kalman_target.py', {'np': np}, ['KalmanTargetTracker'])['KalmanTargetTracker']
    ns = dict(cfg, np=np, pdc=pdc, log=logging.getLogger('oracle'), _HAS_SCIPY=True,
              linear_sum_assignment=linear_sum_assignment, _MPC_CFG=lambda: cfg, MPCProblem=numerical,
              MODE_SEARCH='SEARCH', MODE_TRACK='TRACK')
    Robot = definitions('robot_jps.py', ns, ['Robot', '_point_to_segment_distance'])['Robot']
    return Robot, tracker, numerical


def robot(Robot, tracker, cfg, i, state, prediction, target):
    r = Robot.__new__(Robot)
    r.index, r.time_stamp = i, 0.
    r.state = np.r_[state.position, state.velocity]
    r.accel = state.acceleration.copy()
    r.states_prediction = np.column_stack([prediction[:, :2], np.full(len(prediction), state.position[2]),
                                           prediction[:, 2:4], np.zeros(len(prediction))])
    r.goal = np.r_[target, 0.]
    r.target_tracker = tracker
    r.mode, r.c_in, r.c_out = 'SEARCH', 0, 0
    r.is_leader_role, r.leader_index = False, None
    r.USE_LEAD_PURSUIT = cfg['USE_LEAD_PURSUIT']
    for key in ('VELOCITY_TRUSTED_THRESHOLD', 'LEAD_GAIN_MIN', 'LEAD_GAIN_MAX'):
        setattr(r, key, cfg[key])
    r._mpc_X = r._mpc_U = None
    r.corridors = []
    r._backup = None
    # Neither simulator dynamics nor a flight command is executed by the oracle.
    r.updateState = lambda *args: None
    return r
