"""Exercise mission progress without a running ROS installation or fleet."""

from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


@pytest.fixture
def cli(monkeypatch, tmp_path):
    now = [0.0]
    snapshots = []
    observations = []
    ready = [True]
    destroyed = []

    def call(action, request):
        future = Future()
        message = json.dumps(snapshots.pop(0)) if action == 'status' else 'accepted'
        future.set_result(SimpleNamespace(success=True, message=message, mission_id='test'))
        return future

    def spin(*args, **kwargs):
        now[0] += 1.0
        for observe in observations:
            observe()

    node = SimpleNamespace(
        create_client=lambda typ, name: SimpleNamespace(
            service_is_ready=lambda: ready[0],
            call_async=lambda request: call(name.rsplit('/', 1)[-1], request)),
        destroy_node=lambda: destroyed.append(True),
    )
    modules = {
        'rclpy': SimpleNamespace(init=lambda **kw: None, ok=lambda: True,
                                 spin_once=spin, shutdown=lambda: None),
        'rclpy.clock': SimpleNamespace(Clock=object, ClockType=object),
        'rclpy.node': SimpleNamespace(Node=lambda name: node),
        'rclpy.signals': SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
        'solo_mpc_interfaces.srv': SimpleNamespace(StartMission=SimpleNamespace(Request=SimpleNamespace)),
        'std_srvs.srv': SimpleNamespace(Trigger=SimpleNamespace(Request=SimpleNamespace)),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    source = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc/solo_mpc/mission.py'
    spec = importlib.util.spec_from_file_location('mission_cli_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    (tmp_path / 'manifest.json').write_text(json.dumps({'mode': 'simulation', 'profile': 'mpc'}))

    def snapshot(state, **kwargs):
        return dict(state=state, run_dir=str(tmp_path.resolve()), mission_id='test',
                    error='', **kwargs)

    return SimpleNamespace(main=lambda command='run': module.main([
        command, '--run-dir', str(tmp_path)]), snapshots=snapshots, snapshot=snapshot,
        now=now, ready=ready, observations=observations, destroyed=destroyed)


def test_run_reports_progress_before_completion_and_throttles_heartbeat(cli, capsys):
    controller = {'reason': 'ready', 'vehicles': {'uav1': {'result': 'tracking'}}}
    cli.snapshots.extend([
        cli.snapshot('starting'), *[cli.snapshot('tracking', controller=controller) for _ in range(5)],
        cli.snapshot('landing'), cli.snapshot('completed'),
    ])
    output = []
    cli.observations.append(lambda: output.append(capsys.readouterr()))
    cli.main()
    output.append(capsys.readouterr())
    progress = ''.join(item.err for item in output)
    assert 'Requesting mission start' in output[0].err
    assert 'Mission accepted: test' in progress
    assert 'Mission starting' in progress
    assert progress.count('Mission tracking') == 2
    assert 'controller: ready; uav1: tracking' in progress
    assert 'Mission landing' in progress
    assert 'Mission completed' in progress
    assert all('fleet landed' not in item.out for item in output[:-1])
    assert 'fleet landed' in output[-1].out
    assert cli.destroyed


def test_service_discovery_logs_while_waiting(cli, capsys):
    cli.ready[0] = False
    output = []

    def observe():
        output.append(capsys.readouterr().err)
        if cli.now[0] >= 7:
            cli.ready[0] = True

    cli.observations.append(observe)
    cli.snapshots.append(cli.snapshot('completed'))
    cli.main()
    assert any('Waiting for mission supervisor start service' in text for text in output)


def test_external_target_progress_explains_search_as_target_pursuit(cli, capsys):
    cli.snapshots.extend([
        cli.snapshot('tracking', controller={
            'reason': 'ready', 'target_mode': 'ground_truth', 'target_input_fresh': True,
            'vehicles': {'uav1': {'result': 'search'}, 'uav3': {'result': 'tracking'}},
        }),
        cli.snapshot('completed'),
    ])
    cli.main()
    progress = capsys.readouterr().err
    assert 'uav1: search (target pursuit)' in progress
    assert 'uav3: tracking' in progress
    assert 'external target: fresh' in progress


def test_status_stdout_remains_json(cli, capsys):
    snapshot = cli.snapshot('tracking')
    cli.snapshots.append(snapshot)
    cli.main('status')
    output = capsys.readouterr()
    assert json.loads(output.out) == snapshot
    assert output.err == ''


def test_failure_is_reported_with_cleanup(cli, capsys):
    snapshot = cli.snapshot('failed')
    snapshot['error'] = 'Target simulator stopped reporting'
    cli.snapshots.append(snapshot)
    with pytest.raises(SystemExit, match='Target simulator stopped reporting'):
        cli.main()
    assert 'Mission failed' in capsys.readouterr().err
    assert cli.destroyed
