# Simulation operations

[Project overview](../README.md) · [Getting started](GETTING_STARTED.md)

Run commands from the repository root on the host. Blocks marked “inside the
container” require `./scripts/dev shell` first. Mission commands apply to
simulation; see [Hardware migration](HARDWARE.md) for the companion profile.

## Mission commands

| Command | Behavior |
|---|---|
| `./scripts/dev mpc` | Start a fresh stack, wait for initialization, leave UAVs disarmed |
| `./scripts/dev mpc --gui` | Also attach the viewer after initialization |
| `./scripts/dev mpc --no-wait` | Return without waiting; check readiness before flying |
| `./scripts/dev wait 600` | Wait up to 600 wall seconds for fleet readiness |
| `./scripts/dev mission run` | Take off, track to route completion, LAND, wait for disarm |
| `./scripts/dev mission run --seconds 20` | Limit tracking to 20 wall seconds |
| `./scripts/dev mission start --seconds 20` | Return when tracking starts; supervisor continues |
| `./scripts/dev mission status` | Show mission state, errors, ownership, controller diagnostics |
| `./scripts/dev mission stop` | Disable publishing, pause target, LAND eligible UAVs, wait for disarm |
| `./scripts/dev mission reset` | Reset and pause the target while idle |
| `./scripts/dev stop` | Complete mission stop, then stop the container |
| `./scripts/dev stop --force` | Stop simulation processes immediately, without LAND |

`mpc` waits for launched processes, MPC/perception nodes, requested recorders,
and fleet telemetry readiness. A mission separately checks frame alignment and
controller inputs. During takeoff, target motion and setpoint publishing remain
disabled until all UAVs hold within 0.5 m of flight altitude, with vertical speed
at most 0.2 m/s, for two simulation seconds of fresh telemetry.

The persistent supervisor handles both `run` and `start`. It records independent
Gazebo poses in `truth.jsonl`, enforces duration limits after the CLI exits, and
lands after route completion or timeout. Prolonged target loss, controller or
publisher failure, stale inputs, connection loss, and flight-envelope violations
trigger cleanup. In camera mode, during occlusion the estimate coasts, then searches; the
simulated target pauses during prolonged loss. This assumes a cooperative target.
With `--ground-truth-target`, a stale external feed fails the mission without
camera reacquisition or occlusion pauses.
Starts and resets are rejected while a mission or landing is active.

## Stop and recover

Ctrl+C in `mission run` requests cleanup. Check the result and confirm disarm.
After a failure, run `mission stop` successfully before starting again. If normal
`dev stop` fails, the container stays running for inspection and retry. Use
`stop --force` for a crashed simulation when landing cannot complete.

Cleanup disables publishing, pauses the target, and requests LAND concurrently.
Helper discovery/replies share a two-wall-second budget; each FCU's LAND request
has a five-wall-second budget. One missing service cannot block LAND to another
eligible UAV. Landing/disarm has a separate fleet-wide 240-wall-second deadline,
including with a paused simulation clock. Errors block restart; a later stop
retries cleanup.

Ownership progresses through `unowned`, `acquiring`, `guided`, `owned`,
`land_requested`, and `landing`. A mode change away from GUIDED or disarming
during companion flight latches `released`. The supervisor stops publishing,
pauses the target, and lands other companion-owned UAVs. Released and unowned
aircraft receive no cleanup LAND, including an already-armed aircraft rejected
by preflight. Release survives later stop requests and a return to GUIDED; a
new explicit mission may clear it only when the whole fleet is connected and
disarmed. Status reports ownership; `mission run` distinguishes release from
confirmed landing.

Normal shutdown sends SIGINT to ROS launch so children can close video writers.
Abrupt termination can leave incomplete AVI files. Closing the GUI or stopping
log following does not stop the simulation. Each launch creates fresh simulator
state; MPC runs cannot resume.

## Readiness and topology

One `sim` container contains Gazebo, SITL, MAVROS, bridges, and MPC. ROS discovery
is local to that container and FCU ports are not exposed to the host. Inspect ROS
through `dev shell`.

| UAV | SITL instance | SYSID | JSON UDP | MAVLink TCP | MAVROS namespace |
|---|---:|---:|---:|---:|---|
| uav1 | 0 | 1 | 9002 | 5760 | /uav1/mavros |
| uav2 | 1 | 2 | 9012 | 5770 | /uav2/mavros |
| uav3 | 2 | 3 | 9022 | 5780 | /uav3/mavros |

Each UAV needs applied MAVROS plugin parameters, a connected FCU, acknowledged
position (MAVLink ID 32) and attitude (ID 30) requests at 10 Hz, and at least three
finite poses with distinct timestamps. The latest pose must be less than ten
wall seconds old; FCU state must be less than 60 wall seconds old in simulation
or 20 on hardware. Requests are serialized per FCU and reset on reconnect.

`health.json` and `/solo/readiness` report live health; `ready.json` retains the
most recent ready report. Docker health uses the live report. The initial monitor
deadline defaults to 600 seconds; expiry shuts down launch. The host waiter has
its own timeout and may leave the stack running for inspection. It rejects health
from earlier container starts and detects exits. After first readiness, monitoring
continues without arming or restarting FCUs. Telemetry readiness alone does not
establish prearm acceptance or localization quality.

Gazebo, SITL, MAVROS, readiness, and the sensor bridge are critical: their exit
shuts down launch. MPC adapter failures leave the flight stack available for
cleanup. FCUs do not automatically respawn during an experiment.

If mission startup repeatedly reports `worker_timeout: solve`, the timed-out
worker has been disabled for that controller's lifetime. It will not recover
while the CLI waits for valid MPC inputs. Stop the mission and restart the stack
after fixing the cause. The local planner now compiles both full-grid and sliced
window kernels during initialization; an older checkout can spend around two
seconds compiling its first JPS window and hit the one-second flight deadline.
Increasing the mission `--seconds` value does not fix startup: it limits tracking
after startup succeeds. In a shell chain joined with `&&`, a failed mission also
prevents the final `dev stop` command from running, so run cleanup explicitly.

## Inspect ROS and Gazebo

Inside the container:

```bash
ros2 topic echo /uav1/mavros/state --once
ros2 topic echo /uav1/mavros/local_position/pose --qos-reliability best_effort --once
ros2 topic hz /uav1/mavros/local_position/pose
ros2 topic hz /uav1/scan
ros2 topic hz /uav1/camera/image_raw
ros2 topic echo /solo/readiness --once
ros2 topic echo /mpc/setpoint_status --once
timeout 5s gz topic -e -t /world/mpc_small50/stats
gz sdf -k /workspace/runs/latest/world.sdf
```

Use your configured world name if changed. Exit code 124 from `timeout` is
expected. Sensor/telemetry rates follow simulation time and may be lower on wall
time. Multiple shells can observe the same MAVROS instances.

The GUI follows the pink target from 24 m above, offset 10 m along each horizontal
axis. Press Escape in the 3D view to navigate freely; reopen it to restore follow.

## Logs and evaluation

| Output in `runs/<run-id>/` | Contents |
|---|---|
| `manifest.json`, `source-sha256.json` | Mode, fleet, versions, commit, file hashes |
| `fleet.yaml`, `scenario.yaml`, `controller.yaml`, `mpc-config.json` | Input snapshots and resolved settings |
| `world.sdf`, `bridge.yaml`, `uavN-mavros.yaml` | Generated world and runtime configuration |
| `startup.json`, `health.json`, `ready.json` | Initialization and readiness reports |
| `alignment.json` | Frozen world-to-local calibration |
| `logs/`, `sitl/<instance>/` | Process logs, SITL EEPROM, flight logs |
| `controller_sources.jsonl` | Controller startup hashes and settings |
| `mpc.jsonl`, `truth.jsonl` | Controller measurements/diagnostics and independent Gazebo poses |
| `evaluation.json` | Geometry, tracking, compute, and simulation-speed metrics |
| `cameras/`, `overhead/` | Optional videos and frame timestamps |

The image ID is stored beside the run in `runs/<run-id>.image-id.json`.
`runs/latest` changes on the next launch; preserve the actual directory.
Run data and `artifacts/` are Git-ignored and must be shared separately.

Evaluate after landing, before stopping the container:

```bash
./scripts/dev evaluate
./scripts/dev evaluate --run-dir /workspace/runs/YOUR_RUN_ID
```

Replace `YOUR_RUN_ID` with the saved run's directory name. Evaluation prefers
independent Gazebo geometry, falling back to MAVROS and
labeling `geometry_source`. Its airborne filter requires every UAV above 2.5 m;
revisit it when changing flight altitude. Sampled clearances do not prove
continuous collision avoidance; inspect physics contacts and tracking as well.

## Record video

Enable recording when launching a fresh simulation:

```bash
./scripts/dev mpc --record-cameras --record-overhead
./scripts/dev mission run --seconds 20
./scripts/dev stop
```

Either recording flag works alone; `--gui` is optional. Add
`--ground-truth-target` to the launch command to record video while tracking
the external timestamped target input. Camera detectors remain disabled in that mode.

| Option | View and defaults | Host output |
|---|---|---|
| `--record-cameras` | Downward camera per UAV, 160 × 160 at 8 FPS | `runs/<run-id>/cameras/<uav-name>.avi` |
| `--record-overhead` | Elevated target-following view, 1920 × 1080 at 20 FPS | `runs/<run-id>/overhead/overhead.avi` |

MJPEG videos start when a connected, armed UAV rises more than 0.15 m above its
disarmed ground altitude. UAV cameras have individual gates; overhead starts
with the first UAV to lift off. Recording continues through landing until the
stack stops. Mission completion and GUI closure do not finalize videos.

Each received frame is written once at the configured simulation camera rate.
Dropped frames shorten playback. Adjacent `.timestamps.jsonl` files contain
zero-based frame indices and original simulation timestamps (`stamp_ns`). No
video is created without images. Recorder errors appear in logs without stopping
the flight stack.

Overhead recording is independent of GUI navigation and window size. The camera
sits 24 m above the target, offset -10 m on each horizontal axis. This is a local target view; distant SEARCH vehicles and some outer
formation slots can be outside its frame.
Absence from this video alone does not establish a missing Gazebo
model or a failure to take off. Check `truth.jsonl` or use the GUI to inspect
distant vehicles. A lazy bridge
activates its feed on arming, avoiding unused Full HD rendering during startup.
Rendering/encoding costs return during flight; an external image subscriber can
activate rendering earlier. Direct ROS launch accepts `record_cameras:=true` and
`record_overhead:=true`.

## Record telemetry

Inside the container, use a new bag directory for each recording:

```bash
ros2 bag record -s mcap -o /workspace/runs/latest/telemetry \
  /uav1/mavros/state /uav1/mavros/local_position/pose \
  /uav2/mavros/state /uav2/mavros/local_position/pose \
  /uav3/mavros/state /uav3/mavros/local_position/pose /solo/readiness
```

Adjust topics for your fleet. Stop the recorder with Ctrl+C before the container.
If poses are absent, check sensor QoS and recording overrides. Measure storage
and CPU costs before adding raw image topics.

## Diagnose startup

Check the last FCU status, simulation clock, and recent real-time factor in the
progress report. Connected FCUs with accepted telemetry requests can still be
waiting for their estimator to produce local position.

- A stopped clock suggests paused/stalled physics, bridge failure, or rendering
  trouble.
- A slow clock with progressing EKF/GPS messages means initialization is advancing
  slowly. Increasing timeouts does not improve sensor or control rates.
- A healthy clock with a prolonged pose wait needs FCU estimator and telemetry
  investigation.

Preserve a startup report on the host:

```bash
mkdir -p artifacts
./scripts/dev mpc --record-overhead --record-cameras | tee artifacts/startup.log
./scripts/dev doctor
./scripts/dev status
```

See [renderer verification](WSL_GPU.md#verify-the-renderer) and
[startup measurements](VALIDATION.md#startup-performance).

## Troubleshooting

| Symptom | Action |
|---|---|
| Docker socket permission denied | Check Docker access and WSL integration |
| Stack exits during initialization | Read process logs; a stopped container cannot become ready |
| TCP opens but MAVROS stays disconnected | Check SITL defaults, PANIC messages, and JSON sensor input |
| Connected without pose | Check telemetry acknowledgements, EKF, and simulation speed |
| `PreArm: Gyros inconsistent` | Check model overlap, IMU scope, physics stability, and simulator load |
| `setpoint_attitude` / `invalid allocator` crash | Use the [MAVROS parameter bootstrap](DEVELOPMENT.md#mavros-parameter-bootstrap) |
| Wrong MAVROS version | Ensure the entrypoint sources `/opt/mavros_ws/install/setup.bash` |
| EGL/OpenGL errors or slow sensors | Check the server renderer using [WSL and GPU setup](WSL_GPU.md) |
| Unhealthy after reaching readiness | Inspect live health for stale poses/state or disconnection |
| `No complete airborne fleet records` | Check takeoff and the evaluator's 2.5 m altitude filter |
| LAND cleanup fails | Inspect FCU state and service errors, then retry `mission stop` |
