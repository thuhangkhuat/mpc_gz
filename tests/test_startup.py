"""Startup status must reflect lifecycle events without controlling readiness."""

import json
import os
import sys

import pytest

from solo_bringup.startup import ComponentProgress, StartupProgress, startup_blockers


def test_startup_inventory_and_component_writes_are_independent(tmp_path):
    progress = StartupProgress(tmp_path)
    progress.stage('Generating world')
    progress.register('controller', initializes=True)
    assert json.loads(progress.path.read_text())['processes']['controller']['state'] == 'pending'
    progress.running('controller', os.getpid())
    component = ComponentProgress(tmp_path, 'controller')
    component.update('initializing', 'Constructing controller workers')
    component.update('initialized', 'Waiting for mission inputs')
    inventory = json.loads(progress.path.read_text())
    assert inventory['processes']['controller']['state'] == 'running'
    assert inventory['phase'] == 'Processes launched'
    assert json.loads(component.path.read_text())['state'] == 'initialized'
    progress.exited('controller', 7)
    assert json.loads(progress.path.read_text())['processes']['controller']['exit_code'] == 7
    assert not list(tmp_path.rglob('*.tmp'))


def test_launch_events_record_real_process_start_and_exit(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess
    from solo_bringup.launching import observed

    progress = StartupProgress(tmp_path)
    action = ExecuteProcess(cmd=[sys.executable, '-c', 'raise SystemExit(7)'])
    service = LaunchService()
    service.include_launch_description(LaunchDescription(observed(action, progress, 'test-process')))
    service.run()
    process = json.loads(progress.path.read_text())['processes']['test-process']
    assert process['pid'] > 0
    assert process['state'] == 'exited'
    assert process['exit_code'] == 7


def test_mission_startup_gate_requires_current_initialized_nodes(tmp_path):
    progress = StartupProgress(tmp_path)
    progress.register('controller', initializes=True)
    progress.register('camera-recorder', initializes=True)
    assert startup_blockers(tmp_path, json.loads(progress.path.read_text()))
    progress.running('controller', os.getpid())
    progress.running('camera-recorder', os.getpid())
    ComponentProgress(tmp_path, 'controller').update('initialized', 'ready')
    assert startup_blockers(tmp_path, json.loads(progress.path.read_text())) == ['camera-recorder: initializing']
    ComponentProgress(tmp_path, 'camera-recorder').update('initialized', 'waiting for takeoff')
    assert startup_blockers(tmp_path, json.loads(progress.path.read_text())) == []
    progress.running('controller', os.getpid() + 1)
    assert startup_blockers(tmp_path, json.loads(progress.path.read_text())) == ['controller: initializing']


@pytest.mark.parametrize('failure', ['exited', 'failed'])
def test_mission_startup_gate_rejects_failed_nodes(tmp_path, failure):
    progress = StartupProgress(tmp_path)
    progress.register('controller', initializes=True)
    progress.running('controller', os.getpid())
    if failure == 'exited':
        progress.exited('controller', 7)
    else:
        ComponentProgress(tmp_path, 'controller').update('failed', 'worker could not start')
    with pytest.raises(RuntimeError, match='controller.*(exited|failed)'):
        startup_blockers(tmp_path, json.loads(progress.path.read_text()))
