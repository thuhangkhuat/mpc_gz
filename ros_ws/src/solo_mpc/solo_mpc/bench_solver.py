"""Time the MPC optimizer on this machine by replaying recorded captures.

    ros2 run <package> bench_solver runs/<run-id>/optimizer-inputs

A simulation run with a run directory keeps the last `diagnostic_capture_cycles`
optimizer inputs per UAV (optimizer-inputs/<uav>/solve-NNNN.json). Each one is
solved again here with its recorded solver options, CPU cap included, so the
timings are what this computer would have taken for the same problems. One
solver per problem shape is built up front and warmed up; building it is not
timed. Never starts ROS. Mapping/planning (numba) time is not included.
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import platform
import time

import numpy as np

from .core.mpc_problem import MPCProblem

# Solve-time verdict thresholds as fractions of the control period (0.2 s ->
# 80 / 120 ms): mapping, planning and messaging need the rest of the period.
GOOD_P95 = 0.4
BORDERLINE_P95 = 0.6


def load_captures(paths):
    files = []
    for path in paths:
        files += sorted(path.rglob('solve-*.json')) if path.is_dir() else [path]
    records = []
    for file in files:
        record = json.loads(file.read_text())
        if record.get('schema') != 1 or record.get('outcome') == 'pending':
            continue                 # interrupted mid-solve: inputs exist, outcome unknown
        record['file'] = str(file)
        records.append(record)
    return sorted(records, key=lambda r: (r['vehicle'], r['sequence']))


def solver_key(record):
    return record['n_others'], record['n_faces'], json.dumps(record['cfg'], sort_keys=True)


def solve(solver, record):
    solver._bk_init = (None if record['backup_init'] is None else
                       tuple(np.asarray(v) for v in record['backup_init']))
    inputs = dict(record['inputs'])
    for key in ('X_init', 'U_init'):
        inputs[key] = np.asarray(inputs[key], float)
    start = time.perf_counter()
    ok, _, _ = solver.solve(**inputs)
    elapsed = time.perf_counter() - start
    try:
        stats = solver.opti.stats()
    except RuntimeError:
        stats = {}
    return bool(ok), elapsed, stats.get('return_status', 'unavailable'), stats.get('iter_count')


def read(path, scale=1.0):
    try:
        return float(Path(path).read_text().split()[0]) * scale
    except (OSError, ValueError, IndexError):
        return None


def machine():
    model = None
    for name in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
        try:
            model = Path(name).read_text().strip('\0\n ')
            break
        except OSError:
            pass
    if model is None:
        try:
            model = next(line.split(':', 1)[1].strip() for line in open('/proc/cpuinfo')
                         if line.startswith('model name'))
        except (OSError, StopIteration):
            model = platform.processor() or 'unknown'
    return dict(model=model, arch=platform.machine(), cpus=os.cpu_count(),
                cpu_mhz=read('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 1e-3),
                temp_c=read('/sys/class/thermal/thermal_zone0/temp', 1e-3))


def percentiles(values_ms):
    values = np.asarray(values_ms)
    return {f'p{q}': float(np.percentile(values, q)) for q in (50, 90, 95, 99)} | {
        'max': float(values.max()), 'mean': float(values.mean())}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('captures', type=Path, nargs='+', help='optimizer-inputs directories or capture files')
    parser.add_argument('--repeat', type=int, default=1, help='solve every capture this many times')
    parser.add_argument('--warmup', type=int, default=3, help='untimed solves per solver before timing')
    parser.add_argument('--output', type=Path, help='also write the full report as JSON')
    args = parser.parse_args(argv)

    records = load_captures(args.captures)
    if not records:
        parser.error('no completed captures found (solve-*.json with outcome success/failure)')
    info = machine()
    print(f"machine: {info['model']} ({info['arch']}, {info['cpus']} CPUs"
          + (f", {info['cpu_mhz']:.0f} MHz" if info['cpu_mhz'] else '')
          + (f", {info['temp_c']:.1f} C" if info['temp_c'] else '') + ')')

    solvers, build_s = {}, []
    for record in records:
        key = solver_key(record)
        if key not in solvers:
            start = time.perf_counter()
            solvers[key] = MPCProblem(record['n_others'], record['n_faces'], record['cfg'])
            build_s.append(time.perf_counter() - start)
            for _ in range(args.warmup):
                solve(solvers[key], record)
    print(f"captures: {len(records)} from {len({r['vehicle'] for r in records})} UAV(s), "
          f"{len(solvers)} solver shape(s) built in {sum(build_s):.1f} s")

    rows = []
    for _ in range(args.repeat):
        for record in records:
            ok, elapsed, status, iterations = solve(solvers[solver_key(record)], record)
            recorded = record.get('stats', {})
            rows.append(dict(file=record['file'], vehicle=record['vehicle'], ok=ok, status=status,
                             iterations=iterations, ms=1000 * elapsed,
                             recorded_outcome=record['outcome'],
                             recorded_ms=(None if recorded.get('t_wall_total') is None
                                          else 1000 * recorded['t_wall_total'])))

    cfg = records[0]['cfg']
    period_ms = 1000 * cfg['TIMESTEP']
    cpu_cap = cfg.get('IPOPT_OPTIONS', {}).get('ipopt.max_cpu_time')
    ms = [r['ms'] for r in rows]
    stats = percentiles(ms)
    statuses = Counter(r['status'] for r in rows)
    capped = statuses.get('Maximum_CpuTime_Exceeded', 0)
    failed = sum(not r['ok'] for r in rows)
    newly_failed = sum(not r['ok'] and r['recorded_outcome'] == 'success' for r in rows)
    ratios = [r['ms'] / r['recorded_ms'] for r in rows if r['recorded_ms']]

    print(f"control period {period_ms:.0f} ms, IPOPT CPU cap "
          + (f"{1000 * cpu_cap:.0f} ms" if cpu_cap else 'none'))
    print('solve ms: ' + '  '.join(f"{k} {v:.1f}" for k, v in stats.items()))
    print(f"over period: {sum(m > period_ms for m in ms)}/{len(ms)}   "
          f"CPU cap hit: {capped}   failed: {failed} (of which succeeded when recorded: {newly_failed})")
    print('status: ' + ', '.join(f"{k} {v}" for k, v in statuses.most_common()))
    if ratios:
        print(f"vs recording machine: median x{np.median(ratios):.2f} slower "
              f"(recorded p95 {np.percentile([r['recorded_ms'] for r in rows if r['recorded_ms']], 95):.1f} ms)")
    p95 = stats['p95'] / period_ms
    verdict = ('GOOD' if p95 < GOOD_P95 and not capped and not newly_failed else
               'BORDERLINE' if p95 < BORDERLINE_P95 and capped <= 0.01 * len(rows) else 'TOO SLOW')
    print(f"verdict: {verdict}  (p95 {stats['p95']:.0f} ms; good < {GOOD_P95 * period_ms:.0f} ms, "
          f"borderline < {BORDERLINE_P95 * period_ms:.0f} ms, no new failures or CPU-cap hits)")

    if args.output:
        args.output.write_text(json.dumps(dict(machine=info, period_ms=period_ms, cpu_cap_s=cpu_cap,
                                               solve_ms=stats, statuses=dict(statuses), capped=capped,
                                               failed=failed, newly_failed=newly_failed,
                                               verdict=verdict, solves=rows), indent=2) + '\n')
        print(f"report: {args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
