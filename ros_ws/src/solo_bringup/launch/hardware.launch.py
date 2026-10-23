"""MAVROS-only hardware foundation; no automatic ARM or sensor assumptions."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from solo_bringup.configuration import load_config
from solo_bringup.launching import critical, mavros
from solo_bringup.runtime import create_run


def start(context):
    config = load_config(LaunchConfiguration('config').perform(context), hardware=True)
    for uav in config['uavs']:
        if uav['fcu_url'].startswith('/dev/'):
            device = uav['fcu_url'].split(':')[0]
            if not Path(device).exists():
                raise RuntimeError(f'Missing FCU device {device}; verify hardware config/device mapping')
    if config.get('ld19', {}).get('enabled') or config.get('gopro', {}).get('enabled'):
        raise RuntimeError('Sensor drivers are extension points; see docs/HARDWARE.md before enabling')
    run = create_run(config, 'hardware')
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run / 'logs'))]
    for uav in config['uavs']:
        bridge = mavros(uav, run)
        actions += [critical(bridge), bridge]
    monitor = Node(
        package='solo_bringup', executable='readiness', name='companion_readiness',
        arguments=['--fleet', str(run / 'fleet.yaml'), '--run-dir', str(run),
                   '--hardware', '--timeout', '600'], output='both',
    )
    return actions + [critical(monitor), monitor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.environ.get('HARDWARE_CONFIG', '/workspace/config/hardware.yaml')),
        OpaqueFunction(function=start),
    ])
