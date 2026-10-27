# Hardware integration: 3DR Solo through solo_bridge

The vehicle is a 3DR Solo running 3DR's customised ArduCopter 4.0.1. Its only
supported command path is the existing text relay; nothing in this package
speaks MAVLink, and no MAVROS node runs anywhere:

    solo container on the Pi (~/mpc_ws/3DR/solo_ros)
        solo_mpc_hardware (this package)
          -> ROS 2 topics /solo/*
        solo_bridge
          -> serial text, /dev/ttyAMA0 921600
    solo_relay.py on the Solo's i.MX6      (fixed, cannot be modified)
        -> MAVLink via telem_forwarder
    Pixhawk, ArduCopter 4.0.1 (3DR firmware)

The MPC runs in the same container as the bridge, LiDAR and camera nodes; their
ROS 2 topics stay inside the Pi (and reach laptops for RViz through the
container's `ROS_PEERS` settings). None of that reaches the Solo.

## 1. What the relay provides (fixed)

| Direction | Bridge interface | Content and limits |
|---|---|---|
| Solo -> Pi | `/solo/global_position` `sensor_msgs/NavSatFix` | EKF position (GLOBAL_POSITION_INT), lat/lon/alt AMSL, 10 Hz |
| | `/solo/rel_alt` `std_msgs/Float64` | Height above home, m |
| | `/solo/velocity` `geometry_msgs/TwistStamped` | EKF velocity, already ENU (frame `enu`) |
| | `/solo/heading_deg` `std_msgs/Float64` | Compass heading, deg from north clockwise; absent when unavailable |
| | `/solo/mode`, `/solo/armed`, `/solo/link_ok` | Flight mode, arming, relay link (1 Hz) |
| Pi -> Solo | `/solo/cmd_vel` `geometry_msgs/Twist` | **Velocity only**, ENU. `angular.z` ignored (no yaw control) |
| | `/solo/set_mode`, `/solo/arm`, `/solo/takeoff` | Services; modes limited to STABILIZE, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, LAND, BRAKE |

Consequences the integration must respect:

- No local position, no roll/pitch, no position or acceleration setpoints.
- The bridge forwards velocity at 10 Hz only in GUIDED + armed, clamps it to
  2 m/s horizontal / 1 m/s vertical, and sends `V,0,0,0` (hover) when no
  `cmd_vel` arrived for 0.5 s. Stopping the command publisher is therefore a
  safe hold; Ctrl+C on the bridge holds, then switches to LOITER.
- Yaw cannot be commanded. In GUIDED velocity control the heading follows the
  autopilot's `WP_YAW_BEHAVIOR`; set it to 0 (keep heading) once with a GCS so
  the vehicle does not turn into its direction of travel. Run the MPC with
  `yaw_mode: none`.

## 2. Topic mapping to the controller

The controller is the simulation controller exported verbatim, so it keeps its
simulation topic contract. Thin adapter nodes (phase B, not yet written) map the
bridge onto it:

| Controller input | Built from | Adapter work |
|---|---|---|
| `/uav1/mpc/pose` (`PoseStamped`, frame `world`) | `/solo/global_position`, `/solo/rel_alt`, `/solo/heading_deg` | lat/lon -> local ENU metres around an origin latched at takeoff; z = `rel_alt`; yaw only, ENU yaw = 90 deg - heading (level-attitude approximation) |
| `/uav1/mpc/velocity` (`TwistStamped`, `world`) | `/solo/velocity` | Already ENU; restamp frame |
| `/uav1/mavros/state` | `/solo/mode`, `/solo/armed`, `/solo/link_ok` | connected = `link_ok`. Only the `mavros_msgs/State` *message definition* is used (installed as `ros-humble-mavros-msgs`); MAVROS itself is not installed or run |
| `/uav1/scan` | LD19 `/scan` (solo container) | Remap; set the measured mount in the controller (see 4) |
| `/uav1/target/observation` | `/camera/detections` + TF `camera_optical -> tag36h11:<id>` (apriltag_ros) | Tag position -> `world` through the vehicle pose |

| Controller output | Sent as | Adapter work |
|---|---|---|
| `/uav1/mpc/trajectory` (XY positions/velocities, source-stamped) | `/solo/cmd_vel` at >= 10 Hz | Sample the trajectory at the current time; `v = v_ref + Kp (p_ref - p)`, clamped to the configured `VMAX`; vertical term holds the flight altitude; drop expired trajectories so the bridge falls back to hover |

A supervisor checks link, GUIDED + armed, fix status and data age before the
command adapter publishes, and never sends mode or arming commands on its own
except an explicit operator action.

## 3. Localisation outdoors

Position is the Solo's EKF (GPS + compass + IMU), typically a metre or more of
absolute error and slow drift. Obstacle avoidance uses the LiDAR map built in
the same frame, so drift shifts the map with the vehicle rather than breaking
avoidance, but keep clearance margins at or above the measured EKF jitter and
fly slowly (0.5-0.7 m/s as in the simulation).

## 4. Sensors

**LD19.** `controller_node.py` projects scans assuming a sensor 0.08 m above the
body origin, aligned with the body; it does not read TF. Replace that with the
measured mount (the `z:=`/`yaw:=` values given to `ld19.launch.py`). Check scan
rate, invalid returns and timestamps against `lidar_samples`, sensing range and
timeouts.

**C270.** About 55 deg diagonal (~45 x 34 deg) at 640x480: the ground footprint
at 2 m is only about 1.7 x 1.2 m, so the tag leaves the image quickly. Use the
calibrated intrinsics from `c270_640x480.yaml`, set the real tag size, and
consider a higher altitude or a wider lens before field tests.

## 5. Pi runtime budget

Profile solve times on the Pi 4 under sensor and camera load (phase A3) before
choosing the control period, horizon and deadlines. The example configuration
in `config/` still carries simulation values and is replaced in phase B.

## 6. Power

The Solo accessory bay's 5 V output is limited to about 1 A; a Pi 4 needs
5 V / 3 A plus the LiDAR and camera. Power the Pi from its own regulator off the
flight battery, with a common ground to the serial link.

## 7. Commissioning order

1. Bridge, LiDAR and camera checks on the bench (existing `solo` container).
2. MPC solve-time benchmark on the Pi (A3).
3. Adapters against a simulated relay on the development PC.
4. Props off: full chain with real bridge, commands checked but vehicle disarmed.
5. Tethered/low-altitude GUIDED hover with zero command, then short slow runs.
