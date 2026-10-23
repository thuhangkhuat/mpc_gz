# WSL and GPU setup

[Project overview](../README.md) · [Getting started](GETTING_STARTED.md)

Gazebo cameras and GPU LiDAR require rendering even without a GUI. Display
access and server GPU acceleration are separate checks.

## Windows 11 with WSL2

Use Ubuntu WSL2 with Docker Desktop integration enabled. Keep the checkout under
`/home/<user>/...` rather than `/mnt/c/...` and run commands from Linux.

When `/mnt/wslg` and `/tmp/.X11-unix` exist, `scripts/dev` automatically includes
[compose.wslg.yaml](../compose.wslg.yaml) to map display sockets and environment
variables. The entrypoint creates a writable runtime directory; `xhost +` is
unnecessary. Attach the viewer with `./scripts/dev gui`, or launch with
`./scripts/dev mpc --gui`. Closing it leaves the server running.

## Verify the renderer

On the host, run `./scripts/dev doctor` after starting the simulation. It checks
device access and reads the live Gazebo server's dedicated Ogre log. A separate
GUI renderer does not establish server acceleration.

Inside `./scripts/dev shell`:

```bash
glxinfo -B
gz sim --versions
grep -E 'GL_RENDERER|GL_VENDOR' /workspace/runs/latest/gazebo-home/.gz/rendering/ogre2.log
timeout 5s gz topic -e -t /world/mpc_small50/stats
ros2 topic hz /uav1/camera/image_raw
ros2 topic hz /uav1/scan
```

`glxinfo` describes the display path; the server log describes headless rendering.
`llvmpipe` means software rendering. Exit code 124 from `timeout` is expected.
Compare changes in Gazebo `sim_time` and `real_time` over an interval: individual
statistics can be bursty. Low real-time factor reduces wall-time sensor rates.

For rendering failures, stop the old simulation and try software fallback:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ./scripts/dev mpc --gui
```

You can persist this in `.env`. Measured software-rendered missions ran around
0.17–0.19× real time; see [Validation](VALIDATION.md).

## Native Linux: Intel or AMD

If `/dev/dri` exists on the Docker daemon host, use actual device group IDs:

```bash
export COMPOSE_FILE=compose.yaml:compose.gpu.yaml
export RENDER_GID=$(stat -c %g /dev/dri/renderD128)
export VIDEO_GID=$(stat -c %g /dev/dri/card0)
./scripts/dev mpc
```

Adjust device names to the host. [compose.gpu.yaml](../compose.gpu.yaml) grants
DRM access. For GUI display on native Linux, include [compose.x11.yaml](../compose.x11.yaml)
(e.g., `COMPOSE_FILE=compose.yaml:compose.x11.yaml:compose.gpu.yaml`) and ensure local X11
access via `xhost +local:`.

## NVIDIA

With host drivers and NVIDIA container support configured:
- On WSL2: `export COMPOSE_FILE=compose.yaml:compose.wslg.yaml:compose.nvidia.yaml`
- On native Linux: `export COMPOSE_FILE=compose.yaml:compose.x11.yaml:compose.nvidia.yaml` (run `xhost +local:` on host for GUI access)

```bash
./scripts/dev mpc
```

[compose.nvidia.yaml](../compose.nvidia.yaml) requests GPU/graphics capabilities;
it does not install host drivers or the toolkit. Hardware acceleration for this
setup remains unvalidated. WSL device visibility does not establish access from
Docker Desktop's VM.

## Keep override selection consistent

Setting `COMPOSE_FILE` disables automatic WSLg selection. Include all required
overrides and keep the same selection for start, inspection, and stop. Use
`unset COMPOSE_FILE` to restore automatic selection. Device group IDs must come
from the Docker daemon host. Release deployments require the release base file;
see [Reproducibility](REPRODUCIBILITY.md).

Default simulation needs no host ROS discovery or published FCU ports. A
Pi-to-laptop experiment requires explicit LAN endpoints and routing changes;
see [Hardware migration](HARDWARE.md#2-test-the-pi-against-laptop-sitl).
