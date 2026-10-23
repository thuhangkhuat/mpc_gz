# Development

[Project overview](../README.md) · [Configuration](CONFIGURATION.md)

Use this guide when changing code. For running experiments, see
[Operations](OPERATIONS.md).

## Architecture

```text
scripts/dev → Docker Compose → ROS Python launch
  mpc.launch.py
    ├─ simulation.launch.py → Gazebo + per-UAV SITL/MAVROS + readiness
    ├─ ros_gz sensor, ground-truth, and clock bridges
    ├─ frame adapter + per-UAV camera detectors + simulated target
    ├─ controller → persistent mapping/planning/MPC worker per UAV
    ├─ independent setpoint publisher
    └─ persistent mission supervisor ← mission CLI requests

  hardware.launch.py → MAVROS + readiness
```

Compose owns containers, networks, and devices; ROS launch owns processes.
Readiness and mission control use typed ROS APIs. The GUI is an independent
client of the Gazebo server.

`solo_bringup` validates configuration, generates worlds and runtime files,
creates isolated runs, bootstraps MAVROS parameters, and monitors readiness.
Its `flight.py` provides internal simulation takeoff/LAND helpers.

`solo_mpc` separates numerical code from ROS adapters:

| Module | Responsibility |
|---|---|
| `core/mpc_problem.py`, `core/dynamics.py` | CasADi/IPOPT optimization and acceleration-lag dynamics |
| `core/planner_grid.py`, `core/planner_jps.py`, `core/geometry.py` | Local planning and geometry |
| `core/controller.py` | Measured-state coordination, paths, leader/slot assignment, corridors |
| `core/parallel_controller.py` | Persistent workers, map snapshots, deadlines, shutdown |
| `core/mapping.py`, `core/sensors.py`, `core/tracking.py` | Occupancy, scan projection, camera geometry, target filtering |
| `controller_node.py` | ROS inputs, references, predictions, diagnostics, logs |
| `setpoint_node.py` | Publishing, expiry, altitude feedback, proximity guard |
| `frame_node.py` | Disarmed alignment and common-world measured state |
| `perception_node.py`, `target_node.py` | Marker detection and simulated target motion |
| `world.py`, `settings.py` | World generation and resolved settings |
| `mission_node.py`, `mission.py`, `evaluate.py` | Supervision, request CLI, evaluation |

`TeamController.step(states, scans, stamp, seers)` has no ROS imports. It uses
measured state rather than integrating a replacement vehicle state. Team state
and configuration are instance-local. Solver/dynamics provenance and licensing
are in [UPSTREAM.json](../ros_ws/src/solo_mpc/UPSTREAM.json) and
[LICENSE-MPC](../ros_ws/src/solo_mpc/LICENSE-MPC); no external upstream checkout
is required at runtime.

The algorithm sync inspected on 19 October 2026 adds upstream contingency MPC,
optional terminal stopping, local JPS with Dijkstra fallback, LiDAR-aware corridor
margins, bounded forward corridor seeds, and corridor retention around the last
stop branch. The ROS controller stores a copy of each successful stop certificate;
failed solves cannot erase it. Replay checks measured-state agreement and follows
simulation time. Both serial and parallel execution freeze neighbor predictions
before solving, already providing upstream's two-phase fleet snapshot behavior.
Altitude control remains independent of the planar optimization.

### Parallel execution

`ParallelTeamController` spawns one process per UAV, each owning its map,
planner, corridor, solver, committed path, and warm start. Workers update maps
concurrently. The coordinator checks formation slots against all current maps,
then dispatches local planning/solves using identical measured states, assignments,
and frozen neighbor predictions. Results become the next cycle's predictions.
Fleet time includes the slowest job per phase, coordination, and IPC overhead.

Spawn avoids inheriting ROS/DDS state and native solver objects. A timed-out or
disconnected worker is stopped until controller restart; late replies are not
reused. Missing map data suppresses the fleet decision; an individual solve
failure suppresses that UAV's reference. Shutdown closes workers even if stuck
in native code. Independent publishing and supervision remain able to expire
commands and request LAND.

Worker initialization warms the grid kernels for both contiguous arrays and
noncontiguous local map windows. Numba specializes these layouts separately;
warming only a contiguous test grid leaves compilation on the first flight job,
which can exceed the one-second worker deadline and permanently disable it.
The deadline remains active after initialization. A fresh-cache regression
exercises the first solve with that default deadline.

In camera mode, detection runs in a separate ROS process per UAV. This is local
multiprocessing; distribution across Pis still needs transport, common frames,
and synchronized timestamps. Detection currently uses a magenta marker.
Worker PIDs, phase intervals, and mapping/planning/solve durations are recorded
in `mpc.jsonl` and `/mpc/status`. Select `controller_execution: sequential` for
an algorithm comparison with ordered dispatch to the same persistent UAV workers
and the same shared phase deadline. Sensor callbacks continue while a single
background fleet cycle runs. See [the parity verification guide](MPC_PARITY.md)
for offline upstream comparisons and bounded optimizer replay capture.

With `./scripts/dev mpc --ground-truth-target`, the external target process
publishes timestamped route positions directly to the controller. An
`ExactTargetTracker` preserves each position while using the same Kalman
velocity estimator, confidence gate, and filtered lead-pursuit goal as upstream;
sample intervals follow simulation timestamps. Camera mode uses `TargetTracker`.
The controller subscribes only to the selected source. World generation, sensor
bridging, and the startup inventory omit unused cameras and detectors. Recording
can independently enable camera streams without enabling detection.

## Edit and rebuild

In development, the checkout is mounted at `/workspace`; the entrypoint places
live project Python modules ahead of installed copies. Restart to apply edits
to existing Python modules, launch files, and configuration. New executables,
package metadata, interfaces, or dependencies require rebuilding:

```bash
./scripts/dev build sim
./scripts/dev test
git diff --check
```

For an experimental package build inside `./scripts/dev shell`:

```bash
cd /workspace/ros_ws
colcon build --symlink-install
source install/setup.bash
```

This overlay affects only that shell. Launched processes need explicit entrypoint
sourcing in the order ROS → `/opt/mavros_ws` → project overlay. Persistent dependency
changes belong in the Dockerfile and locks; manual container installs disappear
when replaced. Release mode uses installed code; see [Reproducibility](REPRODUCIBILITY.md).

## ROS interfaces

`uavN` means the configured UAV name, normally `uav1`–`uav3`. Paths are absolute.

| Topic / service | Type and role |
|---|---|
| `/uavN/mavros/local_position/pose`, `/uavN/mavros/local_position/velocity_local` | `PoseStamped`, `TwistStamped`: FCU local state |
| `/uavN/mpc/pose`, `/uavN/mpc/velocity` | `PoseStamped`, `TwistStamped`: common-world measured state |
| `/uavN/scan` | `sensor_msgs/LaserScan` |
| `/uavN/camera/image_raw`, `/uavN/camera/camera_info` | `Image`, `CameraInfo` |
| `/uavN/target/observation` | `PoseStamped`: world-frame camera observation |
| `/uavN/mpc/reference`, `/uavN/mpc/prediction` | `PositionTarget`, `nav_msgs/Path`: world ENU reference/prediction |
| `/uavN/mpc/trajectory` | `trajectory_msgs/JointTrajectory`: world ENU XY positions, velocities and interval acceleration commands; original cycle epoch |
| `/uavN/mavros/setpoint_raw/local` | `PositionTarget`: local ENU, converted to NED by MAVROS |
| `/mpc/status`, `/mpc/setpoint_status` | `std_msgs/String`: JSON diagnostics |
| `/mpc/enable`, `/target/run`, `/target/reset` | `SetBool`, `SetBool`, `Trigger`: publisher ownership and target control |
| `/mission/start` | `solo_mpc_interfaces/StartMission`: tracking duration and run directory |
| `/mission/stop`, `/mission/reset`, `/mission/status` | `Trigger`: cleanup, idle reset, JSON status |
| `/target/ground_truth`, `/uavN/ground_truth/pose` | `PoseStamped`: alignment, supervision, evaluation; target also drives control with `--ground-truth-target` |
| `/solo/readiness`, `/clock` | JSON `String` health, `rosgraph_msgs/Clock` simulation time |

## Frames and clocks

MAVROS uses per-UAV frames such as `uav1/map` and `uav1/base_link`; MPC uses
shared `world` ENU. Namespaces alone do not align FCU origins. While disarmed,
the frame adapter calibrates translation against Gazebo truth with configured
`world_from_local_yaw`, then freezes it in `alignment.json`. Subsequent state
comes from MAVROS. In this world, Gazebo XYZ-to-NED and MAVROS NED-to-ENU
conversions give matching axes. Hardware needs independent origin/yaw calibration.
Positions require rotation plus translation; velocity and acceleration require
rotation only. Verify `PositionTarget` frame, `type_mask`, and ENU/NED conversion
on the actual firmware.

MPC, perception, target, frame, and setpoint nodes use Gazebo `/clock`; MAVROS
uses wall/FCU time. The adapter stamps measured state at simulation receipt time
and projects sensors against nearby poses. Takeoff uses receipt time for vertical
speed and altitude hold, rejecting repeated samples. Slow simulation stretches
wall gaps without invalidating a valid simulation-time hold. Receipt stamping
still has latency error and does not synchronize pose with camera exposure.
Hardware needs a deliberate synchronization policy. Without `/clock`, nodes
using `use_sim_time=true` stop their ROS timers.

Simulation allows 60 wall seconds between FCU heartbeats; setpoint expiry also
checks 2.5 simulation seconds and three wall seconds for poses. MAVLink timesync
and SYSTEM_TIME transmission are disabled because clocks advance at different
rates. The FCU parameter plugin is disabled in simulation, where SITL loads
defaults directly. Hardware retains upstream timing and parameter-plugin defaults.
See [timing settings](CONFIGURATION.md#timing-and-expiry) before changing watchdogs.

## Control ownership and failure behavior

The mission supervisor lives independently of the CLI. Service callbacks queue
work; flight operations spin ROS outside callbacks. A steady timer handles stop
requests during preflight/tracking, and SIGINT/SIGTERM request cleanup before
exit. Publishing is disabled during takeoff and LAND. Ownership and pilot takeover
rules are documented in [Operations](OPERATIONS.md#stop-and-recover).

The independent publisher imports no optimizer. It controls connected, armed
GUIDED UAVs with alignment and measured pose. Expired/invalid references produce
zero horizontal velocity with altitude feedback when state is fresh. Stale pose
or FCU status removes altitude feedback; missing connection, armed/GUIDED state,
pose, or alignment can prevent output entirely.

Corridor/solver failure uses a retained stop plan only while measured state agrees
with it; otherwise it produces no fresh reference. Failed decomposition retains
a valid previous static corridor or fails closed. A proximity guard stops approaching teammates, but soft
constraints and sampled checks do not prove collision avoidance. Acceleration
references describe the lag model, not motor thrust; hardware requires response
and saturation identification.

## MAVROS parameter bootstrap

The pinned MAVROS plugin nodes use `use_global_arguments(false)`, ignoring global
command-line parameter files. Bootstrap uses `ListParameters`, `GetParameters`,
and `SetParameters`, writing only changed values. Readiness requires completion.
Undeclared keys warn and are skipped; rejected declared parameters fail
initialization. UAS frames are configured separately on `mavros_node`.

The pinned `setpoint_attitude.use_quaternion` callback captures a constructor-local
QoS object by reference; later writes can cause `invalid allocator` crashes.
Bootstrap leaves the correct default `false` untouched and rejects mode changes.
Attitude/quaternion control requires fixing that capture, rebuilding MAVROS, and
validating a new release.

## Verification

`./scripts/dev test` covers configuration/world generation, controller and sensor
logic, ROS construction/transforms, expiry, mission guards, and CLI ordering.
The [simulation workflow](../.github/workflows/simulation.yml) builds, tests, and
checks three-FCU heartbeat/pose readiness without arming. Flight-behavior changes
also need a supervised simulation and review of measured logs. Record evidence
and limitations in [Validation](VALIDATION.md); hardware follows the staged
[migration plan](HARDWARE.md).
