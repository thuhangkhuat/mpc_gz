# Hardware integration reference

This is a small ROS 2 source workspace for your colleague to integrate with a
3DR Solo, ArduCopter 4.5.3, LD19 and a camera. It contains trajectory generation
and optional magenta-marker detection. It does not contain sensor drivers,
MAVROS bringup, a hardware localization adapter, a flight-command publisher,
or a hardware mission supervisor. Those are integration work.

The controller and numerical sources are unchanged. The extraction removes the
optional simulation startup-progress hook from `ros_utils.py`, reduces package
metadata/entry points, and supplies single-UAV example configuration. Terminal
interrupt handling leaves worker cleanup to the ROS coordinator. See
`PROVENANCE.json` for provenance. The original project is unchanged.

## 1. What the Solo already provides

3DR's architecture diagram shows a Pixhawk 2 flight controller, an i.MX6
SoloLink companion, a serial GPS connection and an I2C compass connection.
ArduPilot's EKF fuses navigation/inertial measurements to estimate position,
velocity and attitude. This is the intended initial source of MPC vehicle state:

    Solo GPS + compass + IMUs + barometers
                     -> ArduCopter EKF on Pixhawk
                     -> verified MAVLink route
                     -> MAVROS on the Pi
                     -> hardware frame/time adapter
                     -> /uav1/mpc/pose and /uav1/mpc/velocity

The Pi runs the high-level optimizer. The Pixhawk continues to run the flight
controller. Keep the Solo's existing i.MX6/controller link; the Pi does not have
to replace it. Pick and verify one MAVLink route, its baud/ports, flow control,
and command ownership. Do not assume a generic TELEM2 wiring or a specific UDP
port merely because a simulation uses one. The original accessory documentation
labels its serial/CAN connections unsupported for third-party use; use its
pinout and the matching schematic as references, not plug-and-play wiring advice.

The public Copter-4.5.3 source contains a `CubeSolo` target explicitly described
as for a stock Pixhawk 2 cube or Cube Black in a 3DR Solo. Confirm the actual
cube variant and matching firmware/defaults on the vehicle. The generic simulator
firmware image does not identify the physical board or install Solo firmware.

An indoor GPS-denied setup needs additional localization such as VIO, motion
capture, or suitably integrated LiDAR localization. The LD19 driver and raw
camera images alone do not produce the required vehicle state. Even outdoors,
measure EKF uncertainty against the controller's tight clearance/map settings;
the simulation settings are not a statement of consumer GPS accuracy.

## 2. Files included

The ament package is `src/solo_mpc_hardware`. It contains:

- `solo_mpc_hardware/controller_node.py`: ROS inputs and MPC trajectory outputs.
- `solo_mpc_hardware/ros_utils.py`, `settings.py`: arguments, messages and configuration.
- `solo_mpc_hardware/core/`: solver, dynamics, planners, mapping, fleet coordination,
  tracking, workers, timing, sensor geometry and optimizer diagnostics.
- `solo_mpc_hardware/perception_node.py`: optional magenta ground-marker detector.
- `setup.py`, `setup.cfg`, `package.xml`, `resource/solo_mpc_hardware`: ROS packaging.
- `LICENSE-MPC` and `UPSTREAM.json`: license and numerical-source provenance.

The dedicated ROS and Python package name is `solo_mpc_hardware`. Its topics
retain the configured UAV names. Use a single controller for those topics.
No external `motion_planning_MPC` directory is needed at runtime.

Gazebo world/target generation, simulation launch, frame calibration using
ground truth, recording, evaluation, flight publishing and simulation missions
are deliberately outside this export. This package does not implement them.

## 3. Required ROS input and output contract

`uav1` comes from `src/solo_mpc_hardware/config/fleet.example.yaml`. Rename it consistently if needed.

| Input | Type | Required meaning |
|---|---|---|
| `/uav1/mpc/pose` | `geometry_msgs/PoseStamped` | Body pose in a common ENU frame, with `header.frame_id` exactly `world`; normalized body-to-world orientation. |
| `/uav1/mpc/velocity` | `geometry_msgs/TwistStamped` | Linear velocity expressed in the same `world` frame. |
| `/uav1/scan` | `sensor_msgs/LaserScan` | Timestamped LD19 scan; calibrated sensor geometry must be implemented as described below. |
| `/uav1/mavros/state` | `mavros_msgs/State` | Real FCU connection, arming and mode state. Normal solves require connected, armed, `GUIDED`. |
| `/uav1/target/observation` | `geometry_msgs/PoseStamped` | Detected target position in `world`, stamped at observation time. Required in camera mode. |

The optional detector additionally consumes `/uav1/camera/image_raw` as
`sensor_msgs/Image` and `/uav1/camera/camera_info` as `sensor_msgs/CameraInfo`,
as well as the body pose topic above. Otherwise, supply target observations
from your own detector instead.

Outputs are `/uav1/mpc/trajectory` (`trajectory_msgs/JointTrajectory`, XY
positions/velocities/acceleration commands), `/uav1/mpc/reference`
(`mavros_msgs/PositionTarget`, still expressed in `world`),
`/uav1/mpc/prediction` (`nav_msgs/Path`), and `/mpc/status` (JSON in `std_msgs/String`).

The reference is an internal MPC output, not a command ready to relabel and send
to the autopilot. A separate flight bridge must sample the trajectory by its
source timestamp, enforce expiry, transform `world` to FCU local ENU, handle
altitude, and publish the correct MAVROS setpoint fields/masks. MAVROS performs
the MAVLink NED conversion; avoid converting twice. Verify the actual firmware's
supported position/velocity/acceleration setpoint semantics.

The node subscribes with ROS sensor-data QoS. Check compatibility and delivery
on the selected driver/MAVROS versions. For a fleet, this remains one coordinator
with workers for its configured UAVs; distributing one copy per Pi needs an
explicit coordination/transport design and common localization.

## 4. Hardware adaptations required before connecting control

### Localization and timing

Write a hardware state adapter using MAVROS local pose/velocity. Transform both
into `world`: position needs rotation and translation; velocity needs rotation.
Account for attitude/body conventions and EKF-origin resets. Set a deliberate
origin and ground-height reference, since altitude and camera ground intersection
use the same world coordinates. Enforce estimator health in the hardware
supervisor. Raw GPS fixes are not a replacement for these fused-state topics.

Do not reuse the original `frame_node.py` unchanged: it obtains initial frame
alignment from Gazebo ground truth. The original `setpoint_node.py` also expects
the resulting `alignment.json`; it is not a completed hardware bridge.

Use `use_sim_time:=false` on all hardware-side nodes. Synchronize measurement,
ROS and FCU time handling. LD19 scans, image exposure times and pose samples must
be comparable; do not stamp delayed camera frames as if just captured. Settings
with names ending `_sim` are used against the node's current ROS clock; with
simulation time disabled their durations apply to real ROS time.

### LD19

Use a ROS 2 LD19 driver compatible with Jazzy/ARM64, e.g. the manufacturer's
`ldlidar_stl_ros2`, and publish/remap its scan to `/uav1/scan`. Pin a verified
driver revision and measure the achieved scan rate, ranges, invalid values and
timestamp behavior. Match `lidar_samples`, effective sensing range, noise,
clearance margins and timeout settings to that output.

There is a code-level change to make in `controller_node.py`: scan projection
currently assumes a sensor 0.08 m above the body origin, aligned with the body.
It does not look up `LaserScan.header.frame_id` in TF. Publishing an accurate
TF alone will not change this calculation. Replace that offset/orientation with
the measured transform (or implement a timestamped TF lookup):

    p_world_laser = p_world_body + R_world_body * t_body_laser
    R_world_laser = R_world_body * R_body_laser

Pass that laser pose to `project_scan`. Its invalid/no-return handling and tilt
rejection should be preserved and checked with the physical sensor. One pose is
currently used for the whole scan; evaluate motion distortion for your rates.

### Camera

The existing detector finds a magenta marker on a known flat plane; it is not a
general object detector or a localization system. It assumes a camera 0.04 m
below the body and a fixed downward optical orientation in `pixel_to_ground()`.
Replace these assumptions with calibrated intrinsics/extrinsics. Provide
rectified images and matching intrinsics or implement distortion correction.
Account for gimbal orientation and stabilization if the camera moves relative
to the drone; body attitude alone cannot describe such a camera.

For a GoPro, identify the model/firmware first. The original Solo video path is
camera HDMI -> i.MX6 -> video stream. A camera's presence on the Solo does not
provide ROS Image messages on the Pi. Pi 4 HDMI connectors are video outputs,
so HDMI ingestion requires a capture device, or use a verified decoded stream.
Measure stream/exposure latency. A supported CSI/USB camera is another option.
Keep the input interface generic until the actual camera is selected.

### Flight bridge and supervision

Bring your own hardware-specific setpoint bridge and supervisor. They must
preserve command ownership, flight-mode/arming checks, stale-state/command
handling, altitude control, pilot override and validated FCU failure behavior.
The original simulation mission CLI explicitly rejects hardware use.

Start with one UAV. The supplied example still uses simulation-derived geometry,
3 m altitude, 1.2 m/s speed, 2 m/s^2 acceleration and 0.2 s model lag. Calibrate
physical limits, stopping behavior, sensing and controller weights instead of
treating these as validated Solo flight parameters. The optimizer remains planar.

## 5. Installation and runtime budget

Follow the [workspace setup instructions](../README.md) for native Pi installation,
launch, dependency checks and sharing.

Profile the Pi under sensor/camera load. The requested 0.1 s control period is
not a proven 10 Hz achieved rate: the inherited solver CPU cap is 0.2 s and
worker phase timeout is 1 s. Measure solve p95/p99, input age, expiry and publish
jitter before choosing the final rate, horizon and deadlines. Disabling capture
in the example avoids one source of SD-card I/O; normal compute logs still
require storage planning if you enable `--run-dir`.

## 6. Power and physical connection

The Solo accessory documentation limits the combined accessory 5 V output to
1.05 A. Raspberry Pi 4's product brief specifies a 5 V / 3 A supply, before
allowing for your actual peripherals. Use a suitably regulated Pi supply with
margin for LD19/camera and mounting/cooling needs; do not power this assembly
from that accessory 5 V rail. Battery-voltage power requires a suitable regulator,
not a direct connection to the Pi. Verify the physical board revision, pinout,
signal voltage levels and grounds before choosing a wired MAVLink connection.

## 7. Commissioning

Begin with independent sensor and telemetry checks, then Pi-to-SITL integration,
then measured hardware commissioning with an appropriate flight supervisor.
A successful package build does not establish ARM64 dependency compatibility,
real-time solver performance, sensor calibration or physical flight behavior.

## Sources inspected on 21 October 2026

- [3DR architecture and system diagram](https://github.com/3drobotics/solodevguide/blob/master/book/concept-architecture.md)
- [3DR accessory pinout, power limits and peripheral mapping](https://github.com/3drobotics/solodevguide/blob/master/book/hardware-accessorybay.md)
- [Actual Solo mainboard REV A schematic, hosted by OpenSolo](https://github.com/OpenSolo/OpenSolo/blob/master/documentation/Solo-Main-Board-REV-A-Schematic.pdf)
- [3DR accessory breakout schematic](https://github.com/3drobotics/Pixhawk_OS_Hardware/blob/master/Accessory_Breakout_X1/Schematic%20Print/Schematic%20Prints.PDF)
- [3DR gimbal/camera HDMI path](https://github.com/3drobotics/solodevguide/blob/master/book/hardware-gimbalbay.md)
- [ArduPilot EKF overview](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [Copter 4.5.3 CubeSolo hardware definition](https://github.com/ArduPilot/ardupilot/blob/Copter-4.5.3/libraries/AP_HAL_ChibiOS/hwdef/CubeSolo/hwdef.dat)
- [Raspberry Pi 4 product brief](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf)
- [LD19 ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- [CasADi 3.7.2 distributions](https://pypi.org/project/casadi/3.7.2/#files)
- [pydecomp 2.0.0 distributions](https://pypi.org/project/pydecomp/2.0.0/#files)
- [pycddlib 2.1.8.post1 distributions](https://pypi.org/project/pycddlib/2.1.8.post1/#files)

The public hardware documents describe stock designs and some historical
software. They identify the intended architecture; they cannot establish the
installed sensors, wiring, firmware or EKF settings on an individual Solo.
