# Getting started

[Project overview](../README.md) · Next: [Operations](OPERATIONS.md)

This guide runs a short three-UAV simulation using the default configuration.
Run all commands from the repository root on the host unless stated otherwise.

## 1. Prepare the host

Use Linux or Ubuntu under Windows 11 WSL2, with Docker, Docker Compose, and
Python 3 available. On WSL2, enable Docker Desktop integration for your Ubuntu
distribution and keep the checkout in the Linux filesystem, such as
`~/3dr-solo`.

Gazebo cameras and LiDAR need rendering even with the GUI closed. For display
and GPU configuration, follow [WSL and GPU setup](WSL_GPU.md).

Create the local environment file once. Keep your existing `.env` if configured:

```bash
cp .env.example .env
```

## 2. Build and check the environment

```bash
./scripts/dev build sim
./scripts/dev doctor
./scripts/dev test
```

The first build downloads dependencies and compiles ArduPilot, MAVROS, and the
Gazebo plugin. `doctor` checks Docker, configuration, architecture, and device
access. It can inspect the server renderer once a simulation is running.
`test` runs the automated suite inside the image.

## 3. Start the simulation

```bash
./scripts/dev mpc
```

This starts Gazebo, one SITL flight controller and MAVROS instance per UAV,
sensor bridges, and the MPC stack. MAVROS connects ROS to each flight controller
(FCU). **The vehicles remain disarmed until you start a mission.**

Wait for readiness. `RUNNING` means a process has started; `READY` means its
initialization has completed. The command also waits for FCU connections,
telemetry acknowledgements, and fresh local poses. It reports changes immediately
and refreshes progress every 15 seconds.

Startup can take several minutes with software rendering. Estimator initialization
follows simulation time: at 0.1× real-time factor, one simulation second takes
ten wall seconds. See [startup diagnosis](OPERATIONS.md#diagnose-startup) if
progress stalls.

Optionally replace the launch command with one of these:

```bash
./scripts/dev mpc --gui                              # Viewer after readiness
./scripts/dev mpc --record-cameras --record-overhead  # UAV and elevated videos
./scripts/dev mpc --ground-truth-target               # Exact target input without FPV detection
```

Choose options before launching; stop an existing run before starting another.
`--ground-truth-target` uses the scenario route evaluated at Gazebo simulation
timestamps. It skips camera detectors and UAV camera streams unless
`--record-cameras` is also enabled. LiDAR and measured flight telemetry remain
active. The target still pauses during takeoff and starts with the mission.
With `--gui`, use a second terminal for mission commands. You can also attach
the viewer later with `./scripts/dev gui`. Closing it leaves the server running.

## 4. Fly, evaluate, and stop

Once the stack is ready:

```bash
./scripts/dev mission run --seconds 20
./scripts/dev evaluate
./scripts/dev stop
```

The mission checks readiness and frame alignment, then arms and launches the
fleet concurrently. Tracking starts after every UAV holds near flight altitude
with fresh telemetry and usable controller inputs. `--seconds 20` limits tracking
to 20 **wall-clock seconds**, excluding takeoff and initialization. The supervisor
then requests LAND and waits for disarm. Omit `--seconds` to attempt the full route.

Evaluate after landing while the container is still running. Results are written
to `runs/latest/evaluation.json`. Normal `stop` completes cleanup before stopping
the container and finalizing videos. See [recordings](OPERATIONS.md#record-video)
for file locations.

## 5. Inspect or recover a run

Use another host terminal while the simulation runs:

```bash
./scripts/dev status          # Fleet readiness and container state
./scripts/dev mission status  # Mission state, ownership, diagnostics
./scripts/dev logs            # Process logs; Ctrl+C stops following
```

Ctrl+C in `mission run` attempts cleanup and landing. Confirm its result;
failed cleanup needs attention. Normal `stop` leaves the container running if
landing fails. Follow [recovery guidance](OPERATIONS.md#stop-and-recover) before
forcing shutdown.

Continue with [Operations](OPERATIONS.md) for routine work or
[Configuration](CONFIGURATION.md) to change the experiment. To run a tested image
without a checkout, use [Reproducible deployment](REPRODUCIBILITY.md).
