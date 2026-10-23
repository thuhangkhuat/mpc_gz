"""Existing fleet bringup plus scenario, sensors and independent ROS MPC nodes."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from solo_bringup.configuration import load_config
from solo_bringup.launching import critical, observed
from solo_mpc.settings import load_settings
from solo_mpc.scene_view import OVERHEAD_FPS
from solo_mpc.world import bridge_config


def start(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    if get("resume") == "true":
        raise ValueError(
            "Start a fresh MPC run; old alignment and sensor state cannot be resumed"
        )
    fleet = load_config(get("fleet"))
    scenario, settings, cfg = load_settings(get("scenario"), get("settings"))
    if get("target_mode"):
        settings["target_mode"] = get("target_mode")
    if settings["target_mode"] not in ("camera", "ground_truth"):
        raise ValueError("Invalid target_mode")
    # Resolve before world generation so sensors and ROS processes agree.
    context.launch_configurations["target_mode"] = settings["target_mode"]
    camera_enabled = settings["target_mode"] == "camera" or get("record_cameras") == "true"
    if len(fleet["uavs"]) != len(scenario["starts"]):
        raise ValueError("Fleet/scenario UAV count mismatch")
    for uav, point in zip(fleet["uavs"], scenario["starts"]):
        uav["spawn"] = [point[0], point[1], 0.195, 0, 0, 90]
    fleet["world_name"] = scenario.get("world_name", "mpc_small50")
    # Reuse all existing SITL, readiness, critical-process and parameter behavior.
    import importlib.util

    path = Path("/workspace/ros_ws/src/solo_bringup/launch/simulation.launch.py")
    if os.environ.get("SOLO_DEV") != "1" or not path.exists():
        from ament_index_python.packages import get_package_share_directory

        path = Path(
            get_package_share_directory("solo_bringup"), "launch/simulation.launch.py"
        )
    spec = importlib.util.spec_from_file_location("solo_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context.launch_configurations["profile"] = "mpc"
    actions = module.start(context)
    progress = context.solo_startup_progress
    progress.stage('Preparing MPC nodes and sensor bridges')
    run = Path(context.solo_run_dir)
    # Save the actual resolved experiment input alongside the existing manifests.
    import json
    import shutil

    shutil.copyfile(get("scenario"), run / "scenario.yaml")
    import yaml

    (run / "controller.yaml").write_text(yaml.safe_dump(settings, sort_keys=False))
    (run / "mpc-config.json").write_text(json.dumps(cfg, indent=2) + "\n")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="mpc_sensor_bridge",
        parameters=[
            {
                "config_file": bridge_config(
                    [u["name"] for u in fleet["uavs"]],
                    fleet["world_name"],
                    run / "bridge.yaml",
                    record_overhead=get("record_overhead") == "true",
                    camera_enabled=camera_enabled,
                )
            }
        ],
        output="both",
    )
    actions += [critical(bridge)] + observed(bridge, progress, 'sensor-bridge')
    args = [
        "--scenario",
        str(run / "scenario.yaml"),
        "--settings",
        str(run / "controller.yaml"),
        "--fleet",
        str(run / "fleet.yaml"),
        "--run-dir",
        str(run),
        "--world",
        fleet["world_name"],
    ]
    for executable in (
        "frames",
        "target",
        "controller",
        "setpoints",
    ):
        node = Node(
            package="solo_mpc",
            executable=executable,
            arguments=args + ['--startup-component', executable],
            parameters=[{"use_sim_time": True}],
            output="both",
        )
        # Keep SITL and MAVROS alive if a controller/perception process exits.
        # The independent publisher expires references; the supervisor can LAND.
        actions += observed(node, progress, executable, initializes=True)
    supervisor = Node(
        package="solo_mpc", executable="mission_supervisor",
        arguments=['--run-dir', str(run), '--startup-component', 'mission-supervisor'],
        parameters=[{"use_sim_time": True}], output="both",
    )
    actions += observed(supervisor, progress, 'mission-supervisor', initializes=True)
    # Each camera callback runs in its own process, independently of the other
    # vehicles and of all optimization workers. Reuse the installed executable.
    for uav in fleet["uavs"] if settings["target_mode"] == "camera" else []:
        name = f"camera-detector-{uav['name']}"
        node = Node(
            package="solo_mpc", executable="marker_detector",
            arguments=args + ["--vehicle", uav["name"], '--startup-component', name],
            parameters=[{"use_sim_time": True}], output="both",
        )
        actions += observed(node, progress, name, initializes=True)
    if get("record_cameras") == "true":
        # Run the live module so enabling recording needs no image rebuild.
        # Disk writes run independently of camera detection and flight control.
        recorder = ExecuteProcess(
            cmd=[
                "python3", "-m", "solo_mpc.camera_recorder",
                "--fleet", str(run / "fleet.yaml"),
                "--output-dir", str(run / "cameras"),
                "--fps", str(settings["camera_rate"]),
                '--run-dir', str(run), '--startup-component', 'camera-recorder',
                "--ros-args", "-p", "use_sim_time:=true",
            ],
            name="mpc-camera-recorder", output="both",
        )
        actions += observed(recorder, progress, 'camera-recorder', initializes=True)
    if get("record_overhead") == "true":
        recorder = ExecuteProcess(
            cmd=[
                "python3", "-m", "solo_mpc.camera_recorder", "--overhead",
                '--fleet', str(run / 'fleet.yaml'),
                "--output-dir", str(run / "overhead"),
                "--fps", str(OVERHEAD_FPS),
                '--run-dir', str(run), '--startup-component', 'overhead-recorder',
                "--ros-args", "-p", "use_sim_time:=true",
            ],
            name="mpc-overhead-recorder", output="both",
        )
        actions += observed(recorder, progress, 'overhead-recorder', initializes=True)
    progress.stage('Starting processes; waiting for fleet telemetry')
    return actions


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("solo_mpc"))
    live = Path("/workspace/ros_ws/src/solo_mpc")
    if os.environ.get("SOLO_DEV") == "1" and live.exists():
        share = live
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fleet",
                default_value=os.environ.get(
                    "FLEET_CONFIG", "/workspace/config/fleet.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=os.environ.get(
                    "MPC_SCENARIO", str(share / "scenarios/small50_3uav.yaml")
                ),
            ),
            DeclareLaunchArgument(
                "settings",
                default_value=os.environ.get(
                    "MPC_SETTINGS", str(share / "config/controller.yaml")
                ),
            ),
            DeclareLaunchArgument("profile", default_value="mpc"),
            DeclareLaunchArgument(
                "record_cameras",
                default_value=os.environ.get("MPC_RECORD_CAMERAS", "false"),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "record_overhead",
                default_value=os.environ.get("MPC_RECORD_OVERHEAD", "false"),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "target_mode", default_value=os.environ.get("MPC_TARGET_MODE", "")
            ),
            DeclareLaunchArgument("run_id", default_value=os.environ.get("RUN_ID", "")),
            DeclareLaunchArgument("resume", default_value="false"),
            DeclareLaunchArgument(
                "readiness_timeout",
                default_value=os.environ.get("READINESS_TIMEOUT", "600"),
            ),
            OpaqueFunction(function=start),
        ]
    )
