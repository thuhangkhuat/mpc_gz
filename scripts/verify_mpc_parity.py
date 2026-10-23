#!/usr/bin/env python3
"""Run the read-only upstream comparison; never invokes ROS or a simulator."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reference', type=Path, required=True)
parser.add_argument('--reference-run', type=Path, required=True)
args = parser.parse_args()
for path in (args.reference / 'robot_jps.py', args.reference_run / 'config.json'):
    if not path.is_file():
        parser.error(f'Required read-only input missing: {path}')
environment = {**os.environ, 'MPC_REFERENCE': str(args.reference.resolve()),
               'MPC_REFERENCE_RUN': str(args.reference_run.resolve()), 'PYTHONDONTWRITEBYTECODE': '1',
               'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
raise SystemExit(subprocess.call([sys.executable, '-m', 'pytest', '-q', 'tests/test_upstream_parity.py'],
                                cwd=Path(__file__).resolve().parents[1], env=environment))
