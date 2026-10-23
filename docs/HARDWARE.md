# Hardware migration

[Project overview](../README.md) · [Development](DEVELOPMENT.md)

The intended hardware is Raspberry Pi 4, 3DR Solo, LD19 LiDAR, and GoPro.
**This profile has not been validated on a physical Pi or Solo.** It starts
MAVROS and readiness only. It does not flash firmware, arm, start sensors, or
run hardware MPC. The simulation mission CLI rejects hardware runs, and
`companion-stop` stops software without sending LAND.

## Current support and integration gaps

| Area | Available | Still required |
|---|---|---|
| Companion image | ROS Jazzy, pinned MAVROS, readiness, rosbag, camera libraries, project packages | Native ARM64/runtime validation |
| FCU link | Configurable `fcu_url`, SYSID, serial mapping | Verified Solo firmware, wiring, routing, baud, command ownership |
| MPC | Controller source; numerical dependencies in the simulation image | ARM64 dependencies, hardware launch/adapters, solver profiling |
| Localization | MAVROS local pose and simulation frame adapter | Real estimator, common origin/yaw, calibration, clock synchronization |
| LD19 / GoPro | Disabled configuration placeholders | Drivers, stream decoding, calibrated ROS topics |
| Flight supervision | Simulation ownership and cleanup policy | Hardware policy, FCU failsafes, pilot override, flight validation |

The companion image excludes Gazebo, SITL, and the MPC numerical dependencies.
Project source in that image does not provide a complete hardware controller.
Enabling `ld19` or `gopro` currently fails launch explicitly until drivers are
integrated. Keep both disabled while establishing the FCU link.

## 1. Prepare and build the Pi environment

Evaluate Ubuntu Server 24.04 ARM64 as the baseline for ROS Jazzy; check selected
sensor driver/SDK requirements before choosing the OS. Configure host UARTs,
udev names, networking, clock synchronization, power, and cooling.

Prepare `.env` from [.env.example](../.env.example), then build on the Pi:

```bash
./scripts/dev build companion
```

Alternatively, build on a Buildx-capable machine:

```bash
docker buildx build --platform linux/arm64 --target companion \
  -f docker/Dockerfile -t solo-companion:gd0-arm64 --load .
```

Transfer via a registry or `docker save` / `docker load`, and set
`COMPANION_IMAGE` to the chosen tag or digest. Emulated build success does not
validate runtime performance. Preserve platform-specific manifests and image
identity using the [reproducibility workflow](REPRODUCIBILITY.md).

## 2. Test the Pi against laptop SITL

Validate networking, interfaces, and Pi compute budget against simulation before
connecting the Solo. This topology requires implementation: the default stack
keeps all FCU connections inside one container.

1. Choose LAN addresses, a TCP/UDP MAVLink endpoint, ports, and SYSIDs. For TCP,
   make SITL listen on a reachable interface and publish the port in Compose.
   The Pi connects to the laptop's reachable address, not `127.0.0.1`. Check
   firewall, WSL, and Docker routing.
2. Replace the corresponding laptop MAVROS control connection, or add a MAVLink
   router with explicit ownership. Prevent competing control publishers and
   unsupported sharing of one TCP connection.
3. For ROS traffic across the LAN, configure DDS discovery beyond `LOCALHOST`.
   Verify QoS, timestamps, packet loss, and connectivity.
4. Install the numerical dependencies and add hardware launch/state adapters.
   Replace Gazebo time and ground-truth alignment with hardware clock/frame
   handling before testing the Pi controller.

Passing this stage does not validate Solo dynamics or physical sensors.

## 3. Verify the Solo and connect MAVROS

Record the FCU board, bootloader, firmware, Solo/OpenSolo version, and existing
parameters before considering an upgrade. The simulation's ArduCopter 4.5.3 pin
is not a compatibility guarantee for a Solo. Consult the
[ArduPilot Solo documentation](https://ardupilot.org/dev/docs/solo.html) and
[OpenSolo releases](https://github.com/OpenSolo/OpenSolo/releases) for the actual
hardware/software combination.

ArduPilot runs on the FCU; companion software runs on the Pi. Preserve the Solo's
existing i.MX6/controller link. Verify the carrier pinout, voltage levels, UART
or routed MAVLink connection, and port ownership; do not assume a generic TELEM2
mapping.

Edit [hardware.yaml](../config/hardware.yaml) for the real SYSID and link.
`/dev/solo-fcu:921600` is a placeholder, not a measured Solo baud rate. Create a
stable host device name using a verified udev rule. For serial on native Linux:

```bash
export COMPOSE_FILE=compose.yaml:compose.hardware-serial.yaml
export FCU_DEVICE=/dev/solo-fcu
export DIALOUT_GID=$(stat -Lc %g "$FCU_DEVICE")
./scripts/dev companion
docker compose --profile hardware logs -f companion
```

Replace `FCU_DEVICE` with the verified host path. The override maps it to
`/dev/solo-fcu` in the container and adds the device group; a privileged container
is unnecessary. The hardware profile uses host networking. USB passthrough
under WSL is separate from this Pi baseline.

Inspect `runs/hardware/<run-id>/` and `runs/hardware/latest` for logs and health.
Readiness requires local pose as well as heartbeat; an FCU without GPS or another
fused localization source may never become ready. Raw images and scans do not
automatically supply EKF pose. Use `./scripts/dev doctor --service companion`
for environment diagnostics.

Stop companion software with:

```bash
./scripts/dev companion-stop
```

This sends no LAND command. Implement and validate link-loss, stale-state,
solver-stall, and shutdown behavior before physical flight.

## 4. Integrate LD19

Evaluate the manufacturer's [ldlidar_stl_ros2 driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2),
pin a tested commit for Jazzy ARM64, and verify USB–serial output independently.
Add the device mapping and hardware launch integration only after that check.

Publish `/uavN/scan` as `sensor_msgs/LaserScan` with calibrated `uavN/laser`
extrinsics relative to `base_link`. Use a stable udev name based on the adapter's
identity. Measure scan rate, packet loss, range, timestamps, and tilt effects;
match simulation range/rate/noise to the sensor and mounting.

A body-mounted LD19 scan plane tilts with the vehicle. It is neither a complete
3D obstacle map nor a full localization source. Preserve invalid/no-return ray
distinctions and tilt rejection in the replacement adapter.

## 5. Integrate GoPro image input and recording

The intended flow is SD-card recording alongside a perception stream:

```text
GoPro → SD recording
      → stream/capture → decode → ROS Image + CameraInfo → perception
```

Identify the HERO model and firmware, then check [Open GoPro](https://gopro.github.io/OpenGoPro/)
and its [streaming FAQ](https://gopro.github.io/OpenGoPro/docs/faq/) for supported
modes. Concurrent recording/streaming is unverified here. Verify network-over-USB,
Wi-Fi, or HDMI capture support; a USB cable does not establish UVC `/dev/video0`.

Start with low-resolution perception and, where supported, separate high-quality
SD recording. Measure latency, buffering, drops, and CPU load with LiDAR and MPC
active. Publish `/uavN/camera/image_raw` and `/uavN/camera/camera_info` with measured
intrinsics and calibrated optical-frame extrinsics. Replace the magenta-marker
and flat-plane assumptions as needed for the physical target.

Receipt time is not exposure time: synchronize timestamps and measure delay.
Gimbal motion or stabilization can change extrinsics; moving mounts require
corresponding transforms.

## 6. Validate in stages

1. Confirm native ARM64 boot/restart, stable power/cooling, and no automatic arming.
2. With propellers removed, test SYSID, modes, reconnect, command ownership,
   pilot override, and FCU failsafes alongside the existing Solo link.
3. Validate localization, shared ENU origin/yaw, sensor timestamps, calibration,
   and ENU/NED setpoint conversion on the actual firmware.
4. Under full Pi load, measure solver p95/p99, state age, publish jitter,
   deadline misses, and failure responses.
5. Identify position/velocity/acceleration response, saturation, and altitude control.
6. Progress through simple position flight, single-UAV MPC, then multiple UAVs
   with shared localization and validated separation behavior.

At every stage preserve firmware, parameters, image digest, source/configuration
snapshots, and measured outcomes. [Simulation validation](VALIDATION.md) establishes
software integration only; it does not qualify these hardware stages.
