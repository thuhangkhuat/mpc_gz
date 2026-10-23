# Configuration

[Project overview](../README.md) · [Operations](OPERATIONS.md)

Stop the simulation before editing inputs, then launch a fresh run. Inputs are
snapshotted at launch; editing YAML does not change an active experiment.

## Select input files

| Input | Default file | Purpose |
|---|---|---|
| Fleet | [fleet.yaml](../config/fleet.yaml) | UAV names, SITL instances, MAVLink system IDs |
| Scenario | [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml) | Field, starts, obstacles, target route, algorithm parameters |
| Controller | [controller.yaml](../ros_ws/src/solo_mpc/config/controller.yaml) | Altitude, sensors, timing, frame calibration, parameter overrides |
| Hardware | [hardware.yaml](../config/hardware.yaml) | Physical FCU links and sensor placeholders |

Use container paths for CLI overrides. The checkout is mounted at `/workspace`:

```bash
./scripts/dev mpc \
  --fleet /workspace/config/fleet-custom.yaml \
  --scenario /workspace/config/scenario-custom.yaml \
  --settings /workspace/config/controller-custom.yaml
```

For persistent selection, edit `.env`:

| Variable | Default or meaning |
|---|---|
| `FLEET_CONFIG` | `/workspace/config/fleet.yaml` |
| `MPC_SCENARIO` | `/workspace/ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml` |
| `MPC_SETTINGS` | `/workspace/ros_ws/src/solo_mpc/config/controller.yaml` |
| `MPC_TARGET_MODE` | Empty uses controller settings; override with `camera` or `ground_truth`; `--ground-truth-target` selects `ground_truth` |
| `READINESS_TIMEOUT` | 600 wall seconds for initial simulation readiness |
| `SIM_IMAGE`, `COMPANION_IMAGE` | Development tags or registry digests |
| `BUILD_JOBS` | 4 compiler jobs by default |
| `ROS_DOMAIN_ID` | 42; simulation discovery is local to the container |
| `COMPOSE_PROJECT_NAME` | `solo`; also selects the Gazebo partition |
| `LIBGL_ALWAYS_SOFTWARE` | Empty by default; `1` requests software rendering |

The CLI supplies host UID/GID and a fresh run ID. See [WSL and GPU setup](WSL_GPU.md)
for Compose overrides and [Hardware migration](HARDWARE.md) for device variables.
Release deployments use a [separate environment file](REPRODUCIBILITY.md).

## Horizontal launch row, scattered obstacle field

The default [small50_3uav.yaml](../ros_ws/src/solo_mpc/scenarios/small50_3uav.yaml)
uses a 50 m east-west by 30 m north-south field with twelve rectangular and
twelve circular obstacles at irregular positions with varied sizes. The three
UAVs start at `[5, 2, 3]`,
`[2.5, 2, 3]`, and `[7.5, 2, 3]` in fleet order. They share Y = 2 m and altitude
3 m, forming a horizontal row with 2.5 m spacing. All face north (+Y, yaw 90°),
perpendicular to the row. The target begins at `[5, 3]`, ahead of the middle
UAV, initially heading north. Its smooth route sweeps across the field, with
east/west turns and changes between northward and southward travel, finishing
at `[47, 23]`.

Start a fresh simulation with the existing three-UAV fleet and controller:

```bash
./scripts/dev mpc
```

MPC launch takes the UAV positions from the scenario, so no fleet file
changes are needed. The row defines the initial formation; the controller can
adjust UAV positions while following the target and avoiding obstacles.

## Three-UAV fleet

The supplied simulation uses three fleet entries and three finite XYZ rows in
`scenario.starts`, in fleet order. Start altitudes must match the controller's
`flight_altitude`, and XY spacing must be at least `UAV_SAFE_DISTANCE`.
`params.GRID_CELLS` supplies the two unique, nonzero follower offsets.

Fleet schema version is `1`. Names match `[a-z][a-z0-9_]*`, system IDs are 1–255,
and SITL instances are 0–200. The fleet needs a valid `world_name`. Each `spawn`
contains `[x, y, z, roll, pitch, yaw]`, with finite values and angles in degrees.
MPC launch uses scenario XY starts, ground height 0.195 m, and orientation
`[0, 0, 90]`; `flight_altitude` controls flight height.
Count/slot mismatches fail before launch. Instance `i` uses MAVLink TCP
`5760 + 10*i` and JSON UDP `9002 + 10*i` internally, giving port pairs
5760/9002, 5770/9012, and 5780/9022 for the supplied fleet.

## Scenario and controller parameters

Coordinates are metres in a shared world ENU frame: east, north, up. `xlim` and
`ylim` bound the field. Rectangles are `[left, bottom, width, height]`; circles
are `[center_x, center_y, radius]`. Gazebo extrudes them into boxes/cylinders.
Target `waypoints` define a smooth route; `tar_max_speed` supplies constant target
speed unless `target.speeds` provides one positive speed per waypoint leg.
`TAR_SMOOTH_ENABLE: false` selects a polyline instead. An optional scenario
`world_name` selects the Gazebo world/service namespace (default `mpc_small50`).

Algorithm parameters resolve in this order:

```text
settings.py defaults → scenario.params → controller.params → derived values
```

Derived values include timestep from `control_period`, viewing radius from the
scenario, world bounds, inflation from vehicle radius plus margin, and standoff
distance from viewing radius and target speed. Inspect the final `mpc-config.json`
in the run directory rather than relying on an overridden YAML value.

`CORRIDOR_MARGIN` is derived after overrides unless explicitly supplied. It adds
vehicle radius, range-error allowance, half the ray spacing over the longer
horizon, and the grid-cell half diagonal. `LIDAR_ANGULAR_RES` follows `lidar_samples`, and the
default range allowance is three times `lidar_noise_stddev` (Gaussian noise is
unbounded). `CORRIDOR_SEED_MAX` is sensing range minus `sqrt(2)*CORRIDOR_BOX`;
configurations leaving less than one grid cell for the seed are rejected.

| Setting / parameter | Default | Meaning |
|---|---:|---|
| `flight_altitude` / `obstacle_height` | 3 / 5 m | Flight height / extruded obstacle height |
| `control_period` / `publish_rate` | 0.1 s / 20 Hz | MPC decisions / independent setpoint publishing, in simulation time |
| `HORIZON_LENGTH` | 10 steps | MPC prediction horizon |
| `CONTINGENCY` / `BACKUP_HORIZON` | true / 25 steps | Separate stop branch sharing the nominal first control, ending at rest in the hard corridor |
| `W_backup` / `TERMINAL_STOP` | 0.1 / false | Relative backup effort weight / optional nominal terminal stop for comparison |
| `LOCAL_SEARCH` | `jps` | Local JPS with Dijkstra fallback; select `dijkstra` to apply `UNKNOWN_COST` |
| `ROBOT_RADIUS` / `INFLATE_MARGIN` | 0.36 / 0.15 m | Vehicle footprint / obstacle margin |
| `UAV_SAFE_DISTANCE` | 1 m | Initial spacing and separation parameter |
| `VMAX` / `UMAX` | 1.2 m/s / 2 m/s² | Horizontal speed / acceleration limits |
| `ACCEL_TAU` | 0.2 s | Acceleration-lag time constant |
| `CORRIDOR_MAX_FACES` | 16 | Initial solver allocation; grows if needed |
| `CORRIDOR_BOX` / derived `CORRIDOR_MARGIN` | 1.5 / approximately 0.522 m | Decomposition box half size / hard obstacle margin |
| `MPC_SOLVER` | `ipopt` | Solver backend |

Keep obstacle height above flight altitude plus vehicle radius. `IPOPT_OPTIONS`
sets iterations, tolerances, and CPU limits; the default 0.2 s per-solve limit
does not guarantee a 0.1 s fleet cycle. Planning is planar and separation/visibility
constraints are soft, so these settings do not establish collision guarantees.
The 500-iteration limit follows upstream. The CPU allowance was raised from
60 ms for the additional contingency branch; worker deadlines still apply.

On planner, corridor, or solver failure, the controller follows the remaining
verified stop branch (or nominal tail with `TERMINAL_STOP`) and publishes its
prediction to teammates. Replay is indexed by elapsed simulation time. Measured
XY position, velocity and acceleration must agree with that branch within
`BACKUP_POSITION_TOLERANCE` (0.15 m), `BACKUP_VELOCITY_TOLERANCE` (0.3 m/s), and
`BACKUP_ACCEL_TOLERANCE` (0.5 m/s²). Otherwise no fresh reference is produced and
the independent publisher's expiry behavior applies. A completed branch holds
its terminal position with zero velocity and acceleration. New corridors must
contain the remaining branch with the margin, or the previous static corridor
is retained. Fallback results report `nominal_ok: false` and `stop_plan: ...`,
so persistent failures still trigger mission cleanup. Missing sensors and failed
workers continue to suppress references.

## Sensors and target input

Defaults are a 160 × 160 camera at 8 Hz, approximately 53.13° horizontal FOV,
and horizontal LiDAR with 360 samples at 10 Hz, with 0.01 m Gaussian range-noise
standard deviation. At 3 m the camera sees roughly
a 3 m square of ground. Rates follow simulation time. The mission model has no
gimbal; the optional recorders encode video separately.

`camera` mode uses marker detections for target tracking; the controller does
not subscribe to target truth. Ground truth still supports disarmed frame
alignment, route-completion checks, and evaluation. To use truth as controller
input instead, start a fresh run:

```bash
./scripts/dev mpc --ground-truth-target
./scripts/dev mission run --seconds 20
```

The target process evaluates the configured route and per-leg speed schedule
at Gazebo simulation timestamps, including mission pause, resume, and reset.
It publishes the same position used to move the pink marker on
`/target/ground_truth`, stamped at the route evaluation time. The controller
preserves those exact position measurements. Its constant-velocity Kalman
estimator supplies velocity, velocity confidence, and
lead pursuit. An external position does not imply perfect knowledge of velocity
or future motion. Future MPC references still use bounded constant-velocity
prediction; the controller does not read the future route.

This flag overrides `MPC_TARGET_MODE` and the controller's `target_mode` setting.
The environment variable and `target_mode:=ground_truth` ROS launch argument
remain supported. The resolved source is saved in the run's `controller.yaml`
and reported in `/mpc/status`, `mpc.jsonl`, and evaluation results.

In `ground_truth` mode, marker detector processes are omitted from launch and
startup readiness. UAV camera sensors and bridges are omitted unless
`--record-cameras` is requested. Both recording flags and `--gui` can be combined
with `--ground-truth-target`. LiDAR, measured UAV state, frame alignment,
formation distance gates, and independent setpoint expiry still apply.
`SEARCH` retains the upstream meaning: actively pursue the supplied target,
rather than hold position or wait for a detection. Target availability and
formation membership are separate. A leader must be inside the square viewing
radius; a distant follower pursues the target until it is close enough to join
the leader's formation. If no UAV satisfies the viewing gate, all UAVs can be
in `SEARCH` while the external target input remains fresh. Mission progress
prints `search (target pursuit)` and `external target: fresh` to distinguish
this case from a missing input. `/mpc/status` and `mpc.jsonl` also expose
`target_input_fresh` independently of the controller modes.
Completion uses the scenario's `viewing_radius` as the nearest-UAV distance
threshold. Camera occlusion does not pause the target in this mode; an external
sample older than `target_timeout_sim` suppresses controller output and fails
the mission with normal landing cleanup.

LiDAR always uses Gazebo returns. Missing rays stay unknown; observed no-return
rays clear space; strongly tilted rays are discarded. Default `UNKNOWN_POLICY`
is `optimistic`, so unobserved space is not guaranteed clear. A 2D scan cannot
represent arbitrary 3D obstacles, and camera projection assumes a known flat
target plane.

## Timing and expiry

`*_sim` means Gazebo time; `*_wall` means elapsed real time. These budgets differ
when simulation runs below real time. See [frames and clocks](DEVELOPMENT.md#frames-and-clocks).

| Setting | Default | Purpose |
|---|---:|---|
| `controller_execution` | `parallel` | Persistent worker per UAV; `sequential` orders worker solves against the same phase deadline |
| `controller_worker_timeout_wall` / `controller_worker_startup_timeout_wall` | 1 / 30 s | Shared phase / initialization deadlines |
| `state_timeout_sim` / `sensor_timeout_sim` | 0.5 / 0.5 s | Input freshness |
| `target_timeout_sim` / `target_search_timeout_sim` | 1.5 / 8 s | Target loss / reacquisition deadline |
| `planner_failure_timeout_sim` | 3 s | Persistent planning-failure deadline |
| `command_timeout_sim` / `command_timeout_wall` | 0.4 / 3 s | Independent reference expiry limits |
| `pose_timeout_wall` | 3 s | Measured pose freshness |
| `fcu_timeout_sim` / `fcu_timeout_wall` | 2.5 / 60 s | FCU heartbeat freshness |
| `arrival_fleet_radius` / `arrival_timeout_sim` | 6 m / 30 s | Fleet arrival after the last target waypoint |

Worker timeouts bound failures; they are not the 100 ms decision target. Initial
native solves may take longer than warm solves. Increasing timeouts does not
restore sensor/control rates.
Initialization compiles both contiguous and noncontiguous planner kernels before
workers report ready, using the separate startup timeout rather than a flight job.

Controller semantics, intentional production differences, solver-budget policy,
and bounded `diagnostic_capture_cycles` optimizer snapshots are documented in
[the parity verification guide](MPC_PARITY.md).
