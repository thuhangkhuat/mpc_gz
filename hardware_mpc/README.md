# Solo MPC node for hardware integration

A self-contained ROS 2 workspace containing the MPC trajectory generator and an
optional ground-target detector. The target platform is a Raspberry Pi 4 on a
3DR Solo (3DR's ArduCopter 4.0.1) with an LD19 LiDAR and a C270 camera. The Solo
is reached only through `solo_bridge` and the Solo's text relay
(`~/mpc_ws/3DR/solo_ros`); nothing here speaks MAVLink or runs MAVROS.

The ROS and Python package name is **`solo_mpc_hardware`**. This folder can be
committed in the simulation repository or copied into a separate GitHub
repository without any files from its parent directory. Gazebo, SITL and the
original `solo_mpc` and `solo_bringup` packages are not runtime dependencies.

The package produces trajectories. Adapters between the bridge's `/solo/*`
topics and the controller (state, `/solo/cmd_vel` commands, supervision) are
still to be written; the [hardware integration guide](docs/HARDWARE_INTEGRATION.md)
lists the bridge's fixed limits and the topic mapping.

## Contents

```text
src/solo_mpc_hardware/
  solo_mpc_hardware/controller_node.py  ROS input and trajectory output
  solo_mpc_hardware/core/               Solver, dynamics, mapping and workers
  solo_mpc_hardware/perception_node.py  Optional ground-marker detector
  config/                              Single-UAV configuration examples
  launch/controller.launch.py          Controller launch; real clock by default
requirements/                          Pinned numerical dependencies
docs/HARDWARE_INTEGRATION.md            Pi, Solo, LD19 and camera integration
PROVENANCE.json                        Origin and extraction changes
LICENSE                                MIT license retained from upstream
```

## Keep in sync with the simulation

The Python package is generated: never edit `src/solo_mpc_hardware/solo_mpc_hardware/`
by hand. Change `ros_ws/src/solo_mpc` in the simulation repository, then run
`python3 scripts/export_hardware.py` there. It copies every module reachable from
the controller and detector verbatim and records hashes in `PROVENANCE.json`;
the simulation test `tests/test_hardware_export.py` fails when this copy is stale.

## On the Pi: inside the solo container (recommended)

The MPC runs in the existing `solo` container of `~/mpc_ws/3DR/solo_ros`, next to
`solo_bridge`, the LD19 and camera nodes. `scripts/export_hardware.py` (in the
simulation repository) mirrors this package into `solo_ros/ros2_ws/src/` and the
lock into `solo_ros/docker/`; the solo_ros Dockerfile installs the lock and
builds the package with the others. Needs a 64-bit OS (`uname -m` = `aarch64`).

```bash
cd ~/mpc_ws/3DR/solo_ros && docker compose build && docker compose up -d
docker exec -it solo bash
ros2 launch solo_mpc_hardware controller.launch.py
```

[requirements/mpc-py310-aarch64.lock](requirements/mpc-py310-aarch64.lock)
(Python 3.10, ARM64, hash-checked) has the same direct pins as the simulation,
with wheels for everything except pycddlib (GMP) and pydecomp (Eigen, Boost.Geometry), which
compile on the Pi. `ros2_ws/src` is mounted, so re-exporting and relaunching a
node needs no rebuild; numba's compiled kernels persist in the `numba-cache` volume.

## Native install on the Pi (alternative)

Use **Ubuntu Server 24.04 ARM64, ROS 2 Jazzy and Python 3.12** as the baseline.
Install ROS 2 first. Run the following from this folder, not the simulation
repository root:

```bash
cd hardware_mpc  # Or the root of a separate repository containing these files.
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y build-essential cmake libgmp-dev python3-dev python3-venv \
  python3-colcon-common-extensions python3-rosdep python3-pytest
```

Initialize rosdep once on a new installation with `sudo rosdep init`, then:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --require-hashes -r requirements/mpc-py312.lock
```

CasADi 3.7.2 provides a Python 3.12 Linux ARM64 wheel. The pinned pydecomp 2.0.0
and pycddlib 2.1.8.post1 distributions need source builds on ARM64. The compiler,
Python and GMP headers above support those builds; check any additional build
errors on the actual Pi. Keep pycddlib on the pinned 2.x release.

In each terminal, from this folder, expose the numerical environment to the
system-Python ROS executables:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1
python3 -c 'import casadi, numba, _pydecomp; print("numerical imports OK")'
```

## Build and start

```bash
colcon build --symlink-install --packages-select solo_mpc_hardware
source install/setup.bash
ros2 launch solo_mpc_hardware controller.launch.py
```

The launch uses installed example configuration, starts only the controller by
default and sets `use_sim_time=false`. It does not start drivers, talk to the
bridge, arm or issue flight commands. It reports waiting states until measured inputs,
target observations and the connected/armed/GUIDED conditions are supplied.
Use SITL or a recorded-data harness to exercise those conditions before flight.

Select your own configuration paths explicitly:

```bash
ros2 launch solo_mpc_hardware controller.launch.py \
  fleet:=/absolute/path/fleet.yaml \
  scenario:=/absolute/path/scenario.yaml \
  settings:=/absolute/path/controller.yaml
```

After adapting/calibrating the optional detector, enable it with
`marker_detector:=true`. Otherwise publish `/uav1/target/observation` from your
own detector. To see every option:

```bash
ros2 launch solo_mpc_hardware controller.launch.py --show-args
```

The examples retain simulation-derived physical limits and mounting assumptions.
They are configuration scaffolds, not calibrated Solo flight settings. The
[integration guide](docs/HARDWARE_INTEGRATION.md) identifies the hardcoded sensor
geometry that needs replacing. The scenario's waypoint fields satisfy the
existing settings loader; the controller receives its actual target position
from ROS observations.

## Verify the extraction

After building and sourcing the same environment:

```bash
python3 -m pytest src/solo_mpc_hardware/test -q
```

The offline smoke test starts a native worker, verifies interrupt ownership,
consumes a synthetic scan and measured-state snapshot, solves one MPC cycle and
checks its outputs. It starts no ROS flight publisher and uses no physical
devices. This is not a Pi timing
benchmark or hardware flight validation.

Development checks passed on Linux amd64 with ROS 2 Jazzy: package build,
installed launch/configuration, the native worker smoke test, controller startup
and clean Ctrl+C shutdown. Native Pi runtime and physical flight are unvalidated.

## Share on GitHub

For the existing repository, from its root:

```bash
git add hardware_mpc README.md
git commit -m "Add dedicated hardware MPC node workspace"
git push
```

Then share the GitHub link to `hardware_mpc/README.md`. To maintain it in a
separate repository, copy **the contents of `hardware_mpc/`** into the new
repository root and commit them there. Include `src/`, `requirements/`, `docs/`,
`PROVENANCE.json`, `LICENSE` and `.gitignore`; omit generated `.venv/`, `build/`,
`install/` and `log/` directories. Do not copy a built x86 Python environment to
the Pi; build/install its ARM64 dependencies on the target platform.

## Source and maintenance

The Python package is exported verbatim from `ros_ws/src/solo_mpc` by the parent
project's `scripts/export_hardware.py`; `PROVENANCE.json` records the source
commit and the hash of every module. Only the ROS/Python package name differs.
Startup hooks, worker interrupt ownership and bounded Ctrl+C cleanup live in the
simulation sources themselves, so nothing is patched during export. Fix code in
the simulation and re-export; local edits here are overwritten.

Keep `LICENSE`, package `LICENSE-MPC` and `UPSTREAM.json` when redistributing.
Hardware diagrams, firmware sources and dependency references are linked in the
[integration guide](docs/HARDWARE_INTEGRATION.md).
