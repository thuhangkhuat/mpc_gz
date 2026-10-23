# 3DR Solo: multi-UAV target tracking

This project simulates a fleet of UAVs tracking a moving target through an
obstacle field using model predictive control (MPC). The simulation has
three Iris vehicles in a 50 × 30 m field of scattered obstacles, flying at 3 m.
They launch in a horizontal row, facing north perpendicular to the row, then
follow a target along a winding route across the field. Each vehicle uses
MAVROS flight telemetry, horizontal LiDAR, and a downward camera that detects a
magenta target marker. Planning and control operate in the horizontal plane.
Use `./scripts/dev mpc --ground-truth-target` to track exact timestamped target
positions from the simulator instead of camera detections.

Docker provides ROS 2 Jazzy, ArduCopter 4.5.3 software-in-the-loop (SITL),
MAVROS 2.15.1, and Gazebo Harmonic. The host needs Docker Compose and Python 3;
it does not need a ROS or Gazebo installation.

**Start with [Getting started](docs/GETTING_STARTED.md)** to build the image,
launch a simulation, fly a short mission, and inspect its results.

Simulation is the working environment. The Raspberry Pi / 3DR Solo profile
currently starts MAVROS and readiness monitoring only. Physical sensors and
hardware MPC flight require further integration and validation. The
[hardware migration guide](docs/HARDWARE.md) preserves that deployment path.
The Iris model has not been calibrated to a physical Solo.

For a self-contained MPC node workspace to share or integrate on a Raspberry Pi,
see [hardware_mpc](hardware_mpc/README.md). It includes the controller, numerical
dependencies and hardware integration instructions under the separate ROS package
name `solo_mpc_hardware`.

## Documentation

Read the first two guides in order, then use the references for your task.

| Guide | What you will find |
|---|---|
| [Getting started](docs/GETTING_STARTED.md) | Requirements and your first simulation mission |
| [Operations](docs/OPERATIONS.md) | Mission commands, recovery, recordings, logs, and evaluation |
| [Configuration](docs/CONFIGURATION.md) | Fleet size, scenarios, sensors, controller settings, and timing |
| [WSL and GPU setup](docs/WSL_GPU.md) | Display access, rendering checks, and GPU overrides |
| [Development](docs/DEVELOPMENT.md) | Architecture, ROS interfaces, frames, clocks, and verification |
| [Reproducible deployment](docs/REPRODUCIBILITY.md) | Tested images without a checkout and preserved build inputs |
| [Hardware migration](docs/HARDWARE.md) | Pi, Solo, LD19, GoPro, and staged hardware validation |
| [Validation record](docs/VALIDATION.md) | Recorded results, startup measurements, and remaining limitations |

## Repository layout

```text
config/                        Fleet definitions and hardware link template
docker/                        Image build and runtime entrypoint
locks/                         Source pins and dependency locks
hardware_mpc/                  Standalone MPC node workspace for hardware integration
ros_ws/src/solo_bringup/        Launch, readiness, and flight helpers
ros_ws/src/solo_mpc/            Planning, MPC, perception, missions, evaluation
ros_ws/src/solo_mpc_interfaces/ Mission service definition
scripts/                       Development and release command-line tools
tests/                         Automated checks
docs/                          User and contributor guides
runs/                          Generated run data (Git-ignored)
artifacts/                     Manifests and validation evidence (Git-ignored)
```

Each simulation creates `runs/<run-id>/`; `runs/latest` points to the newest run.
Keep the specific run directory, configuration snapshots, source hashes, and
image identity when sharing or comparing an experiment.
