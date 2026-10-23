# Controller parity verification

[Development](DEVELOPMENT.md) · [Validation record](VALIDATION.md)

The supplied simulation uses three UAVs. Offline comparisons check controller
behavior against the external `motion_planning_MPC` implementation; they do not
establish successful flight or achieved control rates.

## Regression coverage

Run the normal suite in the simulation image:

```bash
./scripts/dev test
```

`tests/test_parallel_mpc.py` compares three persistent workers with serial
execution for camera and ground-truth target input. It checks assembled solver
inputs, predictions, map state, persistent process IDs, overlapping solves,
worker failures, deadlines, and shutdown. Single-worker checks cover empty
neighbor sets and disconnects.

`tests/test_upstream_parity.py` optionally compares three-UAV coordination,
leader and assignment hysteresis, moving slots, target estimates, planner paths,
solver inputs, nominal/backup outputs, warm starts, and stopping fallback with
actual upstream definitions. These controlled snapshots use the external
reference settings on both sides; they are not replays of a Gazebo mission.
The harness fingerprints reference files before and after the tests and disables
Numba disk caches in extracted definitions.

The external source and matching saved run are optional, read-only inputs.
Supply their paths explicitly in an environment containing the numerical test
dependencies:

```bash
python3 scripts/verify_mpc_parity.py \
  --reference /path/to/motion_planning_MPC \
  --reference-run /path/to/matching/saved-run
```

The saved run must contain `config.json` with `scenario_def.params` and `params`
from the compatible upstream revision recorded in
[UPSTREAM.json](../ros_ws/src/solo_mpc/UPSTREAM.json). The numerical probes expect
its original 20-step horizon and iteration-only IPOPT options. The wrapper sets
`MPC_REFERENCE` and `MPC_REFERENCE_RUN`; without both inputs, these optional
tests skip. The simulation itself needs no upstream checkout.

## Physical adaptations

The ROS controller uses measured FCU state, real scan projection, camera target
observations, and common-world frame alignment. Ground-truth target input is
available explicitly through `./scripts/dev mpc --ground-truth-target`; it
preserves measured positions while estimating velocity and confidence.

Workers exchange a frozen fleet prediction snapshot between cycles. Corridors
include LiDAR sampling, noise, and occupied-cell quantization margins. Retained
stopping plans advance with simulation time and require measured-state agreement;
failed nominal solves remain visible to mission supervision.

The three-UAV controller keeps its 0.2-second IPOPT CPU cap and one-second worker
phase deadline. The independent publisher samples trajectories at elapsed
simulation time and retains their source timestamps for expiry. Offline
comparisons use the reference solver options to isolate numerical differences.
Passing those comparisons does not establish runtime deadline performance.

## Optimizer capture and replay

`diagnostic_capture_cycles` retains a bounded ring of optimizer calls per UAV
(default 128; 0 disables capture) under
`runs/<run-id>/optimizer-inputs/<uav>/solve-NNNN.json`. A pending record is written
before entering IPOPT. Records contain resolved settings, solver inputs, nominal
and backup warm starts, source hashes, timestamps, outcomes, and solved arrays.
Copy records before later calls overwrite them.

Replay a captured call in the simulation environment:

```bash
PYTHONPATH=ros_ws/src/solo_mpc OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m solo_mpc.replay_optimizer \
  runs/RUN_ID/optimizer-inputs/uav1/solve-0001.json
```

Replay uses recorded budgets. `--iteration-only` removes the CPU cap for a
separate numerical diagnostic. Timing and dependency differences can affect
results. Optimizer capture does not reconstruct sensor, map, or camera history.

For supervised simulation checks and evidence collection, follow
[Getting started](GETTING_STARTED.md) and [Operations](OPERATIONS.md#logs-and-evaluation).
