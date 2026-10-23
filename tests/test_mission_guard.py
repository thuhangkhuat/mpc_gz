"""Mission commands must reject unsupported runs before initializing ROS."""
import json
import time

import pytest

pytest.importorskip("rclpy")
from solo_mpc.mission import main


@pytest.mark.parametrize("mode,profile", [("hardware", "mpc"), ("simulation", "control")])
@pytest.mark.parametrize("command", ["run", "start", "stop", "status", "reset"])
def test_mission_rejects_non_mission_runs(tmp_path, mode, profile, command):
    (tmp_path / "manifest.json").write_text(json.dumps({"mode": mode, "profile": profile}))
    with pytest.raises(SystemExit, match="MPC simulation profile"):
        main([command, "--run-dir", str(tmp_path)])


@pytest.mark.parametrize("seconds", ["-1", "nan", "inf"])
def test_mission_rejects_invalid_duration(seconds):
    with pytest.raises(SystemExit) as exc:
        main(["run", "--seconds", seconds])
    assert exc.value.code == 2


def test_startup_can_be_cancelled_before_run_directory_exists(tmp_path):
    cancel = tmp_path / 'mission.cancel'
    cancel.touch()
    main(['run', '--run-dir', str(tmp_path / 'not-created'),
          '--wait-for-startup', '--cancel-file', str(cancel)])


def test_cancellation_works_while_simulation_clock_is_paused(tmp_path, capsys):
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc', 'config': {'uavs': []},
    }))
    (tmp_path / 'controller.yaml').write_text('{}')
    (tmp_path / 'scenario.yaml').write_text('{}')
    cancel = tmp_path / 'mission.cancel'
    cancel.touch()
    started = time.monotonic()
    # Flight's /clock remains zero. Cancellation must use a steady timer,
    # including while waiting for a service that never becomes available.
    main(['run', '--run-dir', str(tmp_path), '--cancel-file', str(cancel)])
    assert time.monotonic() - started < 3
    assert 'Interrupted; stopping target' in capsys.readouterr().out


@pytest.mark.parametrize('mode,profile', [('hardware', 'mpc'), ('simulation', 'control')])
def test_supervisor_rejects_unsupported_runs_before_ros_initialization(tmp_path, mode, profile):
    from solo_mpc.mission_node import MissionSupervisor
    (tmp_path / 'manifest.json').write_text(json.dumps({'mode': mode, 'profile': profile}))
    with pytest.raises(ValueError, match='MPC simulation profile'):
        MissionSupervisor(tmp_path)


def test_cli_reports_takeover_without_claiming_fleet_landed(tmp_path, monkeypatch, capsys):
    from concurrent.futures import Future
    from types import SimpleNamespace
    from solo_mpc import mission

    (tmp_path / 'manifest.json').write_text(json.dumps({'mode': 'simulation', 'profile': 'mpc'}))
    def client(service, topic):
        def call(request):
            result = SimpleNamespace(success=True, mission_id='test', message=json.dumps({
                'run_dir': str(tmp_path), 'mission_id': 'test', 'state': 'released',
            }))
            future = Future()
            future.set_result(result)
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    monkeypatch.setattr(mission, 'Node', lambda name: SimpleNamespace(
        create_client=client, destroy_node=lambda: None))
    monkeypatch.setattr(mission.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(mission.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(mission.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(mission.rclpy, 'spin_once', lambda *args, **kwargs: None)
    main(['run', '--run-dir', str(tmp_path)])
    output = capsys.readouterr().out
    assert 'released to pilot/FCU' in output
    assert 'fleet landed' not in output
