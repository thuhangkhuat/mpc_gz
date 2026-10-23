"""Common MAVROS process and fail-fast policies for both launch targets."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node

from solo_bringup.runtime import mavros_config


def observed(action, progress, name, initializes=False):
    """Register lifecycle callbacks before executing the process action."""
    progress.register(name, initializes)

    def started(event, context):
        progress.running(name, event.pid)

    def exited(event, context):
        progress.exited(name, event.returncode)

    return [
        RegisterEventHandler(OnProcessStart(target_action=action, on_start=started)),
        RegisterEventHandler(OnProcessExit(target_action=action, on_exit=exited)),
        action,
    ]


def critical(action):
    return RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[LogInfo(msg='Critical process exited; shutting down stack.'),
                 EmitEvent(event=Shutdown(reason='critical process exited'))],
    ))


def mavros(uav, run, *, simulation=False):
    share = Path(get_package_share_directory('mavros'))
    version = ET.parse(share / 'package.xml').getroot().findtext('version')
    if version != '2.15.1':
        raise RuntimeError(f'MAVROS runtime {version}; expected 2.15.1. Source the pinned overlay via solo-entrypoint.')
    return Node(
        package='mavros', executable='mavros_node', namespace=uav['name'],
        output='both', parameters=[
            str(share / 'launch/apm_pluginlists.yaml'),
            mavros_config(uav, run / f"{uav['name']}-mavros.yaml", simulation=simulation),
            {'fcu_url': uav['fcu_url'], 'gcs_url': '', 'tgt_system': uav['sysid'],
             'base_link_frame': f"{uav['name']}/base_link", 'map_frame': f"{uav['name']}/map",
             'odom_frame': f"{uav['name']}/odom",
             'tgt_component': 1, 'fcu_protocol': 'v2.0', 'use_sim_time': False},
        ],
    )
