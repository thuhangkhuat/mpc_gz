"""Replay one complete optimizer capture offline: python -m solo_mpc.replay_optimizer FILE."""

import argparse
import json
from pathlib import Path

import numpy as np

from .core.diagnostics import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--iteration-only', action='store_true',
                        help='Diagnostic: remove the recorded CPU cap; retain its iteration limit')
    parser.add_argument('--tolerance', type=float, default=1e-7)
    args = parser.parse_args()
    record = json.loads(args.capture.read_text())
    ok, X, U = replay(args.capture, iteration_only=args.iteration_only)
    report = dict(vehicle=record['vehicle'], stamp=record['stamp'], recorded=record['outcome'],
                  replay_ok=bool(ok), iteration_only=args.iteration_only)
    matches = None
    if record['outcome'] == 'success' and ok:
        report['max_state_error'] = float(np.max(np.abs(X - np.asarray(record['X']))))
        report['max_control_error'] = float(np.max(np.abs(U - np.asarray(record['U']))))
        matches = bool(np.allclose(X, record['X'], atol=args.tolerance, rtol=args.tolerance)
                       and np.allclose(U, record['U'], atol=args.tolerance, rtol=args.tolerance))
    elif record['outcome'] in ('success', 'failure'):
        matches = bool(ok) == (record['outcome'] == 'success')
    report['matches_recorded_outcome'] = matches
    print(json.dumps(report, indent=2))
    return 1 if matches is False else 0


if __name__ == '__main__':
    raise SystemExit(main())
