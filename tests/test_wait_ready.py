import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('wait_ready', Path(__file__).parents[1] / 'scripts/wait_ready.py')
wait_ready = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wait_ready)


def test_resume_does_not_accept_health_from_before_container_restart():
    data = {'ready': True, 'updated_wall_unix': 100}
    assert not wait_ready.health_is_ready(data, now=101, container_started=100.5)
    data['updated_wall_unix'] = 101
    assert wait_ready.health_is_ready(data, now=102, container_started=100.5)
    assert not wait_ready.health_is_ready(data, now=112, container_started=100.5)


def test_stopped_container_with_exit_zero_reports_logs_immediately(monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[1] == 'inspect':
            return SimpleNamespace(returncode=0, stdout=json.dumps({'Status': 'exited', 'ExitCode': 0}))
        return SimpleNamespace(stdout='RuntimeError: plugin parameters rejected: invalid allocator')

    monkeypatch.setattr(wait_ready.subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='invalid allocator'):
        wait_ready.inspect_container('sim-container')
    assert len(calls) == 2


def test_created_container_cannot_accept_previous_ready_health(monkeypatch):
    monkeypatch.setattr(wait_ready.subprocess, 'run', lambda *args, **kwargs:
                        SimpleNamespace(returncode=0, stdout=json.dumps({'Status': 'created'})))
    started = wait_ready.inspect_container('sim-container')
    assert not wait_ready.health_is_ready({'ready': True, 'updated_wall_unix': 100}, 101, started)


def health_payload():
    return {
        'updated_wall_unix': 100, 'ready': False, 'plugin_parameters_applied': False,
        'uavs': {
            'uav1': {
                'checks': {
                    'FCU': {'ready': True, 'detail': 'connected'},
                    'MAVROS parameters': {'ready': False, 'detail': '1/3 plugins configured'},
                    'telemetry': {'ready': False, 'detail': 'waiting for message 32 acknowledgement'},
                    'local pose': {'ready': False, 'detail': 'waiting for first local pose'},
                },
                'plugins': {
                    'sys': {'ready': True, 'phase': 'ready'},
                    'imu': {'ready': False, 'phase': 'applying parameters'},
                    'local_position': {'ready': False, 'phase': 'waiting for node'},
                },
            },
        },
    }


def test_progress_explains_ready_checks_and_specific_waits(tmp_path):
    output = '\n'.join(wait_ready.progress_lines(tmp_path, health_payload(), None, 101))
    assert 'READY uav1: FCU' in output
    assert 'MAVROS parameters (1/3 plugins configured)' in output
    assert 'waiting for message 32 acknowledgement' in output
    assert 'waiting for first local pose' in output
    assert 'MAVROS applying parameters: imu' in output
    assert 'MAVROS waiting for node: local_position' in output
    assert 'sys' not in output


def test_large_plugin_lists_prioritize_required_plugins_and_remain_readable(tmp_path):
    data = health_payload()
    names = ['imu', 'local_position', 'sys'] + [f'optional{i}' for i in range(10)]
    data['uavs']['uav1']['plugins'] = {
        name: {'ready': False, 'phase': 'reading parameters'} for name in names
    }
    output = '\n'.join(wait_ready.progress_lines(tmp_path, data, None, 101))
    assert 'MAVROS reading parameters: imu, local_position, sys' in output
    assert '+7 more; details: ./scripts/dev status' in output


@pytest.mark.parametrize('updated,container_started,reason', [
    (80, 0, 'no live health update'),
    (100, 100.5, 'this container start'),
    (102, 0, 'this container start'),
])
def test_progress_does_not_claim_ready_from_stale_or_old_health(tmp_path, updated, container_started, reason):
    data = health_payload()
    data['updated_wall_unix'] = updated
    output = '\n'.join(wait_ready.progress_lines(tmp_path, data, None, 101, container_started))
    assert reason in output
    assert 'READY uav1' not in output


def test_process_started_is_not_node_initialized_and_exit_overrides_ready(tmp_path):
    directory = tmp_path / 'startup-components'
    directory.mkdir()
    startup = {'started_wall_unix': 100, 'phase': 'Starting processes', 'processes': {
        'gazebo': {'state': 'running'},
        'controller': {'state': 'running', 'initializes': True, 'pid': 123},
        'frames': {'state': 'pending', 'initializes': True},
    }}
    output = '\n'.join(wait_ready.process_status(tmp_path, startup))
    assert 'RUNNING processes: gazebo' in output
    assert 'controller (loading imports / initializing)' in output
    assert 'frames (not started)' in output
    assert 'READY nodes' not in output
    path = directory / 'controller.json'
    path.write_text(json.dumps({'state': 'initialized', 'pid': 123, 'updated_wall_unix': 101}))
    assert 'READY nodes: controller' in '\n'.join(wait_ready.process_status(tmp_path, startup))
    startup['processes']['controller'].update(state='exited', exit_code=1)
    output = '\n'.join(wait_ready.process_status(tmp_path, startup))
    assert 'FAILED processes/nodes: controller (exited, code 1)' in output
    assert 'READY nodes' not in output


@pytest.mark.parametrize('pid,updated', [(122, 101), (123, 99)])
def test_previous_component_initialization_is_ignored(tmp_path, pid, updated):
    directory = tmp_path / 'startup-components'
    directory.mkdir()
    (directory / 'controller.json').write_text(json.dumps({
        'state': 'initialized', 'pid': pid, 'updated_wall_unix': updated,
    }))
    startup = {'started_wall_unix': 100, 'processes': {
        'controller': {'state': 'running', 'initializes': True, 'pid': 123},
    }}
    assert 'READY nodes' not in '\n'.join(wait_ready.process_status(tmp_path, startup))


def test_legacy_health_still_explains_pending_telemetry(tmp_path):
    data = {'ready': False, 'updated_wall_unix': 100, 'plugin_parameters_applied': True,
            'uavs': {'uav1': {'connected': True, 'telemetry': [32], 'pose_messages': 0}}}
    output = '\n'.join(wait_ready.progress_lines(tmp_path, data, None, 101))
    assert 'READY uav1: FCU, MAVROS parameters' in output
    assert 'accepted messages [32]; need [30, 32]' in output
    assert 'waiting for fresh local poses (0/3)' in output


def test_simulation_diagnostics_explain_slow_clock_and_fcu_progress(tmp_path):
    data = health_payload()
    data['uavs']['uav1']['last_fcu_status'] = 'EKF3 IMU0 tilt alignment complete'
    data['simulation'] = {'time_s': 7.4, 'real_time_factor': 0.11, 'clock_age_wall_s': 0.1}
    output = '\n'.join(wait_ready.progress_lines(tmp_path, data, None, 101)
                       + wait_ready.simulation_lines(data))
    assert 'FCU status: EKF3 IMU0 tilt alignment complete' in output
    assert '7.4s simulated; 0.11x real time' in output
    assert 'FCU initialization follows simulation time' in output
    data['simulation']['clock_age_wall_s'] = 6
    assert 'no clock update for 6s' in '\n'.join(wait_ready.simulation_lines(data))
    data['simulation'] = {'time_s': None}
    assert 'waiting for Gazebo /clock' in '\n'.join(wait_ready.simulation_lines(data))
    assert wait_ready.simulation_lines({}) == []


def test_wait_reports_transitions_immediately_but_only_repeats_every_15_seconds(tmp_path, monkeypatch, capsys):
    run = tmp_path / 'runs/test'
    run.mkdir(parents=True)
    data = health_payload()
    data['simulation'] = {'time_s': 0, 'real_time_factor': 0.1, 'clock_age_wall_s': 0.1}
    ticks = [0]
    monkeypatch.setattr(wait_ready, '__file__', str(tmp_path / 'scripts/wait_ready.py'))
    monkeypatch.setattr(wait_ready.time, 'monotonic', lambda: ticks[0])
    monkeypatch.setattr(wait_ready.time, 'time', lambda: 100 + ticks[0])

    def refresh():
        data['updated_wall_unix'] = 100 + ticks[0]
        (run / 'health.json').write_text(json.dumps(data))

    def sleep(_):
        ticks[0] += 1
        data['simulation']['time_s'] += 0.1
        if ticks[0] == 1:
            data['uavs']['uav1']['checks']['telemetry']['ready'] = True
        if ticks[0] == 17:
            data['ready'] = True
        refresh()

    refresh()
    monkeypatch.setattr(wait_ready.time, 'sleep', sleep)
    wait_ready.main(['--run-id', 'test', '--timeout', '20'])
    output = capsys.readouterr().out
    assert '[startup +0s]' in output
    assert '[startup +1s]' in output
    assert '[startup +16s]' in output
    assert '[startup +17s]' in output
    assert output.count('[startup +') == 4
    assert 'Simulation ready' in output


def test_mpc_wait_requires_node_initialization_even_with_ready_fcu_health(tmp_path, monkeypatch):
    run = tmp_path / 'runs/test'
    run.mkdir(parents=True)
    (run / 'startup-components').mkdir()
    (run / 'health.json').write_text(json.dumps({
        'ready': True, 'updated_wall_unix': 100, 'plugin_parameters_applied': True,
        'uavs': {},
    }))
    (run / 'startup.json').write_text(json.dumps({
        'started_wall_unix': 100, 'phase': 'Processes launched',
        'processes': {'controller': {'state': 'running', 'initializes': True, 'pid': 123}},
    }))
    ticks = [0]
    monkeypatch.setattr(wait_ready, '__file__', str(tmp_path / 'scripts/wait_ready.py'))
    monkeypatch.setattr(wait_ready.time, 'monotonic', lambda: ticks[0])
    monkeypatch.setattr(wait_ready.time, 'time', lambda: 100 + ticks[0])

    def sleep(_):
        ticks[0] += 1
        (run / 'startup-components/controller.json').write_text(json.dumps({
            'state': 'initialized', 'pid': 123, 'updated_wall_unix': 101,
        }))

    monkeypatch.setattr(wait_ready.time, 'sleep', sleep)
    wait_ready.main(['--run-id', 'test', '--all-processes', '--timeout', '3'])
    assert ticks[0] == 1
