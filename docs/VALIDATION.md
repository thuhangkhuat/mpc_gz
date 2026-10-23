# Validation record

[Project overview](../README.md) · [Collect a new run](OPERATIONS.md#logs-and-evaluation)

These are recorded results from **9–19 October 2026**, not guarantees for another
machine or code revision. Evidence under `runs/` and `artifacts/` is Git-ignored
and may be absent from a fresh clone. Test counts describe the suite at each
recorded checkpoint.

## Controller parity restoration — 20 October 2026

Offline regression: **276 passed, 4 skipped, 3 deselected**. Direct upstream
comparison: **20 passed**, covering fleet coordination, assembled solver
inputs, nominal/backup outputs, planning and fallback. Runtime CPU caps and mission
watchdogs remain unchanged. No Gazebo, SITL or mission was launched. Current
comparison coverage, physical adaptations, and replay commands are in
[the parity verification guide](MPC_PARITY.md).

## Scope and remaining work

Simulation has demonstrated a complete camera-driven route, short missions with
parallel compute, automatic landing after the CLI exits, and deployment without
a checkout. The complete route predates current controller/supervisor defaults.

Still required:

- Rerun the complete moving-target Gazebo route with current defaults and preserve
  source/configuration snapshots, image identity, solve failures, and landing results.
- Validate hardware GPU rendering, achieved sensor/control rates, and each new
  fleet size. Software-rendered flight workloads were about 0.17–0.19× real time.
- Establish native ARM64 runtime results and measured Pi solver tail latency,
  state age, and publisher jitter under full sensor/perception load.
- Integrate and validate Solo firmware/FCU links, LD19, GoPro recording/streaming,
  shared localization, clock synchronization, pilot override, and flight failsafes.

There is no physical Pi/Solo flight validation or neural detector in these
results. Planning is planar, scan coverage is 2D, and optimization constraints
are soft. Sampled clearances do not establish continuous collision avoidance.
Follow [Hardware migration](HARDWARE.md) before physical deployment.

## Recorded environment

Ubuntu WSL2, Docker Desktop 29.8.2, Compose 5.5.1, Linux amd64, 14 allocated CPUs,
and about 15.37 GiB RAM. WSLg display worked; the inspected server renderer was
CPU `llvmpipe`. Components were ROS 2 Jazzy, ArduCopter 4.5.3, MAVROS overlay
2.15.1, Gazebo Harmonic / gz-sim 8.15.0, ROS Python 3.12.3, and Waf Python
3.11.16. Source pins are in [versions.json](../locks/versions.json); APT patch
versions can change on rebuild.

## Upstream algorithm sync — 19 October

Synced the numerical solver and local planner to `motion_planning_MPC` commit
`d7e5a791b2eefd0c3a55a2a5e444eec75dc709ae`. Both files match the inspected source
apart from relative imports. An identical-input solve produced exactly equal
nominal and backup trajectories. Source hashes and ROS adaptations are recorded
in [UPSTREAM.json](../ros_ws/src/solo_mpc/UPSTREAM.json).

Local ROS Jazzy tests used the project `.venv`, single-threaded BLAS, and a local
build of `solo_mpc_interfaces` under `artifacts/mpc-sync-interface-install`.
The final suite excluding three ROS communication cases reported **228 passed,
4 skipped, 3 deselected**. The three cases failed in the full run and again with
an isolated ROS domain/UDP transport: the two detached-supervisor completion
cases and remote takeoff cancellation could not discover or receive replies
from the spawned mission service. They do not exercise the optimizer. The
remaining tests cover contingency/terminal stopping, forced solver/planner/
corridor failures, rejection of divergent measured state, elapsed-time replay,
JPS/Dijkstra correctness, corridor retention and bounds, measured-state ownership,
parallel parity/failures, ROS reference publication, and mission failure visibility.

A sequential three-UAV trace used the first **600 steps / 60 simulation seconds**
of the current scenario, analytic 360-ray scans, exact acceleration-lag dynamics,
and synthetic target observations. All **1,800 nominal solves succeeded**, with
no fallback use. Minimum sampled body clearance was **0.718 m** and minimum
center separation was **1.708 m**. Fleet compute median/p95 was **180/221 ms**
in sequential execution; this is not a parallel flight timing measurement.
This trace follows part of the route and does not include camera, FCU, or sensor
transport errors. The earlier 60 ms CPU allowance repeatedly exhausted its
budget with the new branch; current defaults allow 200 ms and 500 iterations.

Evidence: `artifacts/mpc-sync-tests-20261019.log`,
`artifacts/mpc-sync-focused-tests-20261019.log`,
`artifacts/mpc-sync-ros-service-recheck-20261019.log`,
`artifacts/mpc-sync-upstream-parity-20261019.json`, and
`artifacts/mpc-sync-final-route-20261019.log`. Docker's WSL integration was
unavailable during the initial sync; the later startup-fix check below includes
a short Gazebo/SITL mission. The complete route with these defaults remains
unvalidated.

## First planning job exceeding the worker deadline — 19 October

A fresh-cache Docker probe reproduced `worker_timeout: solve` for all three
workers on their first cycle. With a diagnostic ten-second deadline, that cycle
finished in 2.41 s: path planning spent about 1.95 s compiling a Numba specialization,
followed by about 0.44 s in the cold optimizer. The warmup had only compiled
contiguous grids, while real local windows are noncontiguous NumPy slices.
The one-second operational deadline killed each worker during compilation;
subsequent cycles retained its failure until controller restart.

Warmup now compiles both layouts before workers report ready. With another
empty cache and the unchanged one-second deadline, initialization took 9.16 s
and the first fleet cycle took 0.58 s; all three solves succeeded. Subsequent
cycles took about 0.14 s. The targeted container suite reported **43 passed**,
including fresh-cache first-solve, JPS/Dijkstra layout, and stalled-worker checks.

Evidence: `artifacts/mpc-cold-before.log`, `artifacts/mpc-cold-timing.log`,
`artifacts/mpc-cold-kernel-fix.log`, and `artifacts/mpc-worker-timeout-tests.log`.

Camera-mode Gazebo/SITL run `runs/20261019T043934Z-13303` then completed the
requested **30 wall seconds of tracking**, landed and disarmed all three UAVs,
and stopped the container cleanly. Across 64 solve cycles it recorded **zero
worker timeouts**; first-job planning took 0.25–0.96 ms and the cold native solves
328–338 ms. Eight individual UAV1 solves later reached IPOPT's CPU limit; they
were reported as ordinary `mpc_failed` results and subsequent solves recovered.
Minimum sampled ground-truth body clearance/separation was **0.847/2.438 m**.
Airborne fleet compute median/p95 was **102/225 ms**, with about **0.094×**
real-time simulation speed; this does not establish a 100 ms fleet guarantee.
The finalized overhead AVI decoded all **422 frames at 1920×1080 / 20 FPS**.

Flight evidence: the run's `evaluation.json`, `mpc.jsonl`, `truth.jsonl`, and
`overhead/overhead.avi`; plus `artifacts/mpc-worker-timeout-fixed-mission.log`,
`artifacts/mpc-worker-timeout-fixed-stop.log`, and
`artifacts/mpc-worker-timeout-fixed-summary.json`.

## Release packaging — 15 October

Native AMD64 builds separated compiler/input archives from flattened runtimes.
Image sizes from `docker image inspect .Size`, in decimal GB:

| Image | Previous | Release runtime |
|---|---:|---:|
| Simulation | 8.04 | 4.25 |
| Companion | 3.33 | 2.36 |

Tested local image IDs, which are not registry pull references:

- Simulation: `sha256:b69386064a5e9ab3031df48d9a211251db62dd783bf4f2fc9a57cb0475496733`
- Companion: `sha256:b84608000fa4c8ca5b36cb03872ffe11b0585d78c7ab4b20396ccaf49bdd62b5`

All **204 tests passed** in the final simulation runtime. Separate `release-smoke`
acceptance used extracted deployment files without a checkout. It checked imports,
MAVROS overlay, application/dataset hashes, shared libraries, absence of build
environments/tools, and writes as UID/GID 1000 with read-only root/configuration.

Simulation initialized every node and reached three-FCU telemetry/pose readiness
in **253 wall seconds**, without arming. Camera/LiDAR messages were sampled. The
live server log identified Mesa 25.2.8, `llvmpipe (LLVM 20.1.2, 256 bits)`, and
roughly 0.14–0.19× real-time speed. Companion acceptance checked ROS/MAVROS and
filesystem behavior without an FCU. A companion replay from preserved inputs
with build-command network access disabled also passed smoke acceptance.

Evidence: `artifacts/release-final-tests.log`,
`artifacts/simulation-release-final-smoke/`,
`artifacts/companion-release-final-smoke/`, and
`artifacts/companion-offline-smoke/`. Preserved local inputs are
`solo-simulation-inputs:release`, `solo-companion-inputs:release`, and
`solo-base:inputs`. This local check did not test ARM64, GPU acceleration, or
registry publication. The [release workflow](../.github/workflows/release.yml)
provides native platform acceptance; see [Reproducibility](REPRODUCIBILITY.md).

## Persistent supervision — 15 October

Run `runs/20261015T065414Z-9371` used the default three-UAV camera scenario,
parallel controller, 3 m altitude, and live sources. Local image:
`sha256:ccd3426614a2fb3d8666f55fc0719174553168e5bebd155b0b1217c22aaf515e`.
Initialization took 263 wall seconds at about 0.13–0.19× real time.

`mission start --seconds 10` returned during tracking. Independent status calls
then observed `tracking`, `landing`, and `completed` without error. The supervisor
enforced the limit and landed/disarmed all three UAVs after CLI exit. Truth logs
contained 938 samples over simulation time 21.15–68.0 s; maximum altitudes were
about 3.12 m. Normal stop succeeded.

The checkpoint suite passed 165 tests, including supervision after CLI exit,
duration limits, paused-clock takeoff cancellation, request serialization, and
injected failure policies. Injected failures used simulated flight I/O; the live
Gazebo run checked short tracking and landing, not full-route completion.
Evidence: `artifacts/mission-supervisor-validation.json` and the
`artifacts/mission-supervisor-*` status, flight, test, and stop logs.

## Startup performance

On 15 October, two fresh disarmed runs used the same image, default three-UAV
scenario, both video flags, and software-rendered WSL host (Core Ultra 5 225H,
14 CPUs available to Docker). The change deferred overhead rendering until
arming and removed empty waits between parameter/telemetry bootstrap phases.

| Measurement | Before | After |
|---|---:|---:|
| Host readiness wait | 361 s | 234.4 s |
| Sustained disarmed simulation speed | About 0.11× | About 0.18× |
| EKF initialization from launch inventory start | 36.2 s | 24.9 s |
| GPS detection from launch inventory start | 81.9 s | 53.8 s |
| EKF origin from launch inventory start | 170.5 s | 111.6 s |
| Last blocking stage | Fresh local poses | Fresh local poses |

Both runs needed about 41 simulation seconds for local-pose readiness. The
127-second (35%) reduction is one paired observation, not a startup guarantee.
Controller construction stayed around 3.6–3.7 seconds. Most savings came from
faster simulation during estimator initialization. FCUs initialize concurrently;
accepted telemetry requests do not mean an estimator can already supply pose.
Approximately, `wall time = simulation time / real-time factor + startup overhead`.

Overhead video adds a 1920 × 1080 camera at 20 Hz, about 41.5 million pixels per
simulation second, versus 0.61 million across the three downward cameras.
The lazy bridge has no overhead image subscriber while disarmed and activates
on arming. That rendering cost returns in flight; external subscribers can
activate it earlier. Camera recording itself reuses the perception images.

Baseline run: `runs/20261015T061044Z-20121`; improved run:
`runs/20261015T061726Z-20498`. Evidence: `artifacts/startup-baseline.log`,
`artifacts/startup-improved.log`, matching container logs and health/startup/manifest
snapshots, and `artifacts/startup-improved-overhead-topic.txt`.

The checkpoint passed 142 tests, including a Gazebo rendering test that activated
the feed with synthetic armed telemetry, moved the target, and decoded finalized
1080p/20 FPS video with increasing timestamps. That video test used a static
isolated scene; the startup comparison used three live disarmed FCUs. Neither
startup benchmark armed or flew. See [startup diagnosis](OPERATIONS.md#diagnose-startup)
and [renderer verification](WSL_GPU.md#verify-the-renderer) to measure a new run.

## Complete camera-driven route — 11 October

Run `runs/20261011T055529Z-29926` took three UAVs to about 3 m, tracked the
magenta marker through the obstacle field to the final waypoint near `(47, 5.5)`,
and landed/disarmed. LiDAR used Gazebo returns; the camera-mode controller did
not subscribe to target truth. Evaluation used independent Gazebo vehicle poses.

| Metric | Recorded result |
|---|---:|
| Airborne controller samples | 747 |
| Minimum sampled obstacle body clearance, including 0.36 m radius | 0.361 m |
| Minimum sampled inter-UAV center distance | 1.135 m |
| Nearest UAV-to-target distance: median / p95 / maximum | 1.080 / 1.553 / 1.829 m |
| Target estimate error: median / p95 / maximum | 0.094 / 0.311 / 0.922 m |
| Whole-fleet compute: median / p95 / maximum | 156 / 199 / 270 ms |
| Cycles with all three solves successful | 58.8% |
| Simulation seconds per wall second | 0.172 |

This run used the earlier **32-face** allocation. During disarmed bringup the
controller was restarted with corrected ENU alignment and compiled scan
integration; the initial manifest predates that restart. Current runs also save
controller startup hashes in `controller_sources.jsonl`. Compute exceeded the
100 ms target; successful route completion does not establish real-time readiness.

Evidence: the run's `evaluation.json`, `artifacts/mpc-camera-flight.log`, and
`artifacts/mpc-camera-evaluation.json`.

## Supporting integration checks

| Check | Result and scope |
|---|---|
| 16-face pure-controller route | 850 synthetic-scan/dynamics steps; 0.402 m minimum body clearance, 1.145 m separation; 55 failed solves of 2,550. Evidence: `artifacts/mpc-core-route-16.log` |
| 16-face stationary Gazebo endpoint | 22 wall seconds publishing, then LAND/disarm; 43 airborne cycles, 90.7% successful fleet cycles, 127/157 ms median/p95 compute, 0.030 m median target error. Evidence: `runs/mpc-endpoint-verification`, `artifacts/mpc-endpoint-flight.log` |
| Parallel workers, 14 October | 67 tests passed, covering solve overlap/parity, map persistence, measured-state ownership, worker failures/shutdown, and camera isolation. Headless run `runs/20261014T044239Z-15013` tracked briefly, landed, and disarmed |
| Initial bringup, 9–10 October | Three isolated FCU links, telemetry IDs 30/32, finite pose, prefixed frames, flight commands, and LAND/disarm; subsequent checks covered the MAVROS callback and exited-container waiter fixes |

These workloads are not direct performance comparisons. The stationary endpoint
revealed false heartbeat expiry under slow SITL. Historical follow-up used a
10 s wall limit; the [current configuration](CONFIGURATION.md#timing-and-expiry)
uses 60 wall seconds plus 2.5 simulation seconds. A full Gazebo route with the
final defaults remains outstanding.

Earlier validation used a temporary Dockerfile with host-downloaded, hash-checked
Python/ROS packages after container downloads stalled. That historical build is
recorded in `artifacts/mpc-build-final.log`; the canonical build command remains
`./scripts/dev build sim`. Temporary caches are not runtime dependencies.
