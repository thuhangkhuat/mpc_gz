"""Launch trajectory generation and, optionally, calibrated marker detection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('solo_mpc_hardware')) / 'config'
    arguments = [
        '--fleet', LaunchConfiguration('fleet'),
        '--scenario', LaunchConfiguration('scenario'),
        '--settings', LaunchConfiguration('settings'),
    ]
    parameters = [{
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }]
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value=str(config / 'fleet.example.yaml')),
        DeclareLaunchArgument('scenario', default_value=str(config / 'scenario.example.yaml')),
        DeclareLaunchArgument('settings', default_value=str(config / 'controller.example.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'marker_detector', default_value='false', choices=['true', 'false'],
            description='Start the optional detector after calibrating its camera geometry.',
        ),
        Node(
            package='solo_mpc_hardware', executable='controller',
            arguments=arguments, parameters=parameters, output='screen',
        ),
        Node(
            package='solo_mpc_hardware', executable='marker_detector',
            arguments=arguments, parameters=parameters, output='screen',
            condition=IfCondition(LaunchConfiguration('marker_detector')),
        ),
    ])
