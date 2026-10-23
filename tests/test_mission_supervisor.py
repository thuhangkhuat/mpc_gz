"""A detached mission keeps its supervisor, failure policy and landing owner."""

import json
from concurrent.futures import Future
import multiprocessing
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from solo_bringup.flight import Ownership
from solo_mpc.mission_node import MissionSupervisor
from solo_mpc_interfaces.srv import StartMission
from std_srvs.srv import Trigger


@pytest.fixture
def run_dir(tmp_path):
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc',
        'config': {'uavs': [{'name': 'test_uav'}]},
    }))
    (tmp_path / 'controller.yaml').write_text('''flight_altitude: 3.0
camera_hfov: 0.9
pose_timeout_wall: 3.0
command_timeout_wall: 3.0
target_timeout_sim: 1.5
target_search_timeout_sim: 8.0
planner_failure_timeout_sim: 3.0
arrival_timeout_sim: 3.0
''')
    (tmp_path / 'scenario.yaml').write_text('target:\n  waypoints: [[0, 0], [10, 0]]\n')
    return tmp_path


@pytest.fixture
def supervisor(run_dir):
    rclpy.init()
    node = MissionSupervisor(run_dir)
    node.service_call = lambda *args: None
    mock_cleanup_clients(node)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def mock_cleanup_clients(node):
    def call(name, request):
        future = Future()
        try:
            node.service_call(name, request.data)
            future.set_result(SimpleNamespace(success=True))
        except Exception as exc:
            future.set_exception(exc)
        return future
    node.mission_clients = {name: SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request, name=name: call(name, request),
    ) for name in node.mission_clients}


def own_flight(node):
    node.ownership.update(dict.fromkeys(node.names, Ownership.OWNED))


def mock_landing(node, callback):
    def land(names, *, poll):
        poll()
        callback(names)
        for name in names:
            node.observe_ownership(name)
            if node.ownership[name] != Ownership.RELEASED:
                node.ownership[name] = Ownership.UNOWNED
    node.land = land


def healthy(node, stamp=1.0, position=0.0):
    node.snapshots = {
        'stamp': stamp, 'wall': time.time(), 'reason': 'ready',
        'target_observation_age': 0.0,
        'vehicles': {'test_uav': {'position': [position, 0, 3], 'result': 'tracking'}},
    }
    pose = PoseStamped()
    pose.pose.position.z = 3.0
    node.poses['test_uav'] = pose
    node.states['test_uav'] = SimpleNamespace(connected=True, armed=True, mode='GUIDED')
    node.connected = lambda _: True
    target = PoseStamped()
    target.pose.position.x = position
    node.target = {'pose': target, 'wall': time.monotonic()}
    node.delivery = {'wall': time.monotonic()}


def start(node, seconds=0.0):
    return node.request_start(StartMission.Request(
        run_dir=str(node.run_dir), seconds=seconds), StartMission.Response())


@pytest.mark.parametrize('case,error', [
    ('target_loss', 'SEARCH timeout'),
    ('planner_failure', 'Planner inputs or solves remained unavailable'),
    ('arrival_timeout', 'fleet did not arrive'),
    ('target_stale', 'Target simulator stopped reporting'),
    ('publisher_stale', 'setpoint publisher stopped reporting'),
    ('controller_stale', 'Controller status stopped updating'),
    ('altitude', 'altitude flight envelope'),
    ('target_reacquired', None),
    ('arrival', None),
    ('duration', None),
])
def test_persistent_supervision_handles_completion_and_failures(supervisor, monkeypatch, case, error):
    node = supervisor
    events = []
    steps = 0

    def prepare():
        own_flight(node)
        healthy(node)

    def tick(*args, **kwargs):
        nonlocal steps
        steps += 1
        healthy(node, stamp=steps * 4.0)
        if case == 'target_loss':
            node.snapshots['target_observation_age'] = 2.0 if steps == 1 else 9.0
        elif case == 'planner_failure':
            node.snapshots['reason'] = 'unavailable'
        elif case == 'arrival_timeout':
            node.target['pose'].pose.position.x = 10
            node.snapshots['vehicles']['test_uav']['position'] = [20, 0, 3]
        elif case == 'target_stale':
            node.target['wall'] -= 10
        elif case == 'publisher_stale':
            node.delivery['wall'] -= 10
        elif case == 'controller_stale':
            node.snapshots['wall'] -= 10
        elif case == 'altitude':
            node.poses['test_uav'].pose.position.z = 0
        elif case == 'arrival':
            healthy(node, position=10)
        elif case == 'target_reacquired':
            if steps == 1:
                node.snapshots['target_observation_age'] = 2.0
            elif steps == 3:
                healthy(node, position=10)
        elif case == 'duration':
            time.sleep(0.02)
        assert steps < 5, 'supervision did not terminate'

    node.prepare_flight = prepare
    node.service_call = lambda name, value=None: events.append((name, value))
    mock_landing(node, lambda names: events.append(('LAND', tuple(names))))
    monkeypatch.setattr(rclpy, 'spin_once', tick)
    assert start(node, seconds=0.01 if case == 'duration' else 0).success
    node.process_pending()
    assert node.mission_state == ('failed' if error else 'completed')
    assert error is None or error in node.error
    assert events[-3:] == [('/mpc/enable', False), ('/target/run', False), ('LAND', ('test_uav',))]
    if case == 'target_loss':
        assert events[0] == ('/target/run', False)
    elif case == 'target_reacquired':
        assert events[:2] == [('/target/run', False), ('/target/run', True)]
    assert node.truth_log is None


@pytest.mark.parametrize('case,error', [
    ('arrival', None),
    ('external_stale', 'External target position stopped updating'),
    ('target_stale', 'Target simulator stopped reporting'),
])
def test_external_target_supervision_does_not_depend_on_camera(supervisor, monkeypatch, case, error):
    node = supervisor
    node.settings['target_mode'] = 'ground_truth'
    node.settings['camera_hfov'] = .01
    node.scenario['viewing_radius'] = 2.
    events = []

    def prepare():
        own_flight(node)
        healthy(node)

    def tick(*args, **kwargs):
        healthy(node)
        if case == 'arrival':
            node.target['pose'].pose.position.x = 10.
            node.snapshots['vehicles']['test_uav']['position'] = [11.5, 0, 3]
        elif case == 'external_stale':
            node.snapshots['target_observation_age'] = 2.
        else:
            node.target['wall'] -= 10.

    node.prepare_flight = prepare
    node.service_call = lambda name, value=None: events.append((name, value))
    mock_landing(node, lambda names: events.append(('LAND', tuple(names))))
    monkeypatch.setattr(rclpy, 'spin_once', tick)
    assert start(node).success
    node.process_pending()
    assert node.mission_state == ('failed' if error else 'completed')
    assert error is None or error in node.error
    # No camera-loss pause/resume requests; helper cleanup and LAND still run.
    assert events == [('/mpc/enable', False), ('/target/run', False), ('LAND', ('test_uav',))]


def test_requests_serialize_flight_ownership(supervisor):
    node = supervisor
    assert start(node, seconds=7).success
    mission_id = node.mission_id
    assert not start(node).success
    assert node.duration == 7
    assert node.mission_id == mission_id
    assert not node.request_reset(Trigger.Request(), Trigger.Response()).success
    assert node.request_stop(Trigger.Request(), Trigger.Response()).success
    node.process_pending()
    assert node.mission_state == 'stopped'
    assert not node.flight_owned


@pytest.mark.parametrize('seconds', [-1.0, float('nan'), float('inf')])
def test_server_rejects_invalid_duration(supervisor, seconds):
    assert not start(supervisor, seconds).success
    assert supervisor.mission_state == 'idle'


def test_server_rejects_wrong_run(supervisor):
    result = supervisor.request_start(StartMission.Request(run_dir='/another/run'), StartMission.Response())
    assert not result.success
    assert supervisor.mission_state == 'idle'


def test_cleanup_failure_is_reported_and_blocks_restart(supervisor):
    node = supervisor
    node.prepare_flight = lambda: own_flight(node)
    node.supervise = lambda: None
    def fail():
        raise RuntimeError('LAND rejected')
    node.stop_flight = fail
    assert start(node).success
    node.process_pending()
    assert node.mission_state == 'failed'
    assert 'LAND rejected' in node.error
    assert not start(node).success
    node.stop_flight = lambda: None
    node.request_stop(Trigger.Request(), Trigger.Response())
    node.process_pending()
    assert node.mission_state == 'stopped'
    assert start(node).success


def test_target_reset_after_stop_clears_the_previous_cancellation(supervisor):
    node = supervisor
    node.stop_flight = lambda: None
    node.request_stop(Trigger.Request(), Trigger.Response())
    node.process_pending()
    calls = []
    node.service_call = lambda name: calls.append(name)
    assert node.request_reset(Trigger.Request(), Trigger.Response()).success
    node.process_pending()
    assert calls == ['/target/reset']
    assert node.mission_state == 'idle'


def test_publisher_failure_still_pauses_target_and_attempts_land(supervisor):
    node = supervisor
    events = []
    def call(name, value):
        events.append((name, value))
        if name == '/mpc/enable':
            raise RuntimeError('publisher unavailable')
    node.service_call = call
    mock_landing(node, lambda _: events.append('LAND'))
    node.request_stop(Trigger.Request(), Trigger.Response())
    node.process_pending()
    assert events == [('/mpc/enable', False), ('/target/run', False), 'LAND']
    assert node.mission_state == 'failed'
    assert 'publisher unavailable' in node.error


def test_landing_cannot_be_interrupted_by_more_stop_requests(supervisor):
    node = supervisor
    def land():
        assert not start(node).success
        node.request_stop(Trigger.Request(), Trigger.Response())
        node.check_stop()
    node.stop_flight = land
    node.request_stop(Trigger.Request(), Trigger.Response())
    node.process_pending()
    assert node.mission_state == 'stopped'


def detached_supervisor(run_dir):
    """ROS server with simulated flight I/O, in a process separate from the CLI."""
    from rclpy.clock import Clock, ClockType
    rclpy.init()
    node = MissionSupervisor(run_dir)
    run = Path(run_dir)
    try:
        def prepare():
            own_flight(node)
            healthy(node)
            if (run / 'block-takeoff').exists():
                (run / 'flight-started').touch()
                node.wait(lambda: False, 'simulated takeoff', 30)
        def update():
            healthy(node)
            if (run / 'lose-target').exists():
                node.snapshots['target_observation_age'] = 9.0
        node.prepare_flight = prepare
        node.service_call = lambda *args: None
        node.stop_flight = lambda: (run / 'landed').touch()
        node.create_timer(0.05, update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        (run / 'server-ready').touch()
        while not (run / 'exit-server').exists():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.process_pending()
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('completion', ['target_loss', 'duration'])
def test_start_cli_exit_keeps_supervision_and_automatic_landing(run_dir, completion):
    from solo_mpc.mission import main
    server = multiprocessing.get_context('spawn').Process(
        target=detached_supervisor, args=(str(run_dir),))
    server.start()
    try:
        deadline = time.monotonic() + 15
        while not (run_dir / 'server-ready').exists():
            assert server.is_alive()
            assert time.monotonic() < deadline
            time.sleep(0.05)
        args = ['start', '--run-dir', str(run_dir)]
        if completion == 'duration':
            args += ['--seconds', '2']
        main(args)
        assert server.is_alive()
        assert not (run_dir / 'landed').exists()
        if completion == 'target_loss':
            (run_dir / 'lose-target').touch()
        deadline = time.monotonic() + 5
        while not (run_dir / 'landed').exists():
            assert server.is_alive()
            assert time.monotonic() < deadline
            time.sleep(0.05)
        assert (run_dir / 'truth.jsonl').exists()
    finally:
        (run_dir / 'exit-server').touch()
        server.join(timeout=5)
        if server.is_alive():
            server.terminate()
            server.join(timeout=5)
        assert server.exitcode == 0


def cancel_during_takeoff(run_dir):
    run = Path(run_dir)
    deadline = time.monotonic() + 15
    while not (run / 'flight-started').exists():
        if time.monotonic() >= deadline:
            raise RuntimeError('Mission never entered takeoff')
        time.sleep(0.05)
    (run / 'cancel').touch()


def test_cli_cancellation_interrupts_remote_takeoff_with_paused_sim_clock(run_dir):
    from solo_mpc.mission import main
    context = multiprocessing.get_context('spawn')
    (run_dir / 'block-takeoff').touch()
    server = context.Process(target=detached_supervisor, args=(str(run_dir),))
    cancel = context.Process(target=cancel_during_takeoff, args=(str(run_dir),))
    server.start()
    cancel.start()
    try:
        main(['run', '--run-dir', str(run_dir), '--cancel-file', str(run_dir / 'cancel')])
        assert (run_dir / 'landed').exists()
        assert server.is_alive()
    finally:
        (run_dir / 'exit-server').touch()
        for process in (server, cancel):
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=5)
            assert process.exitcode == 0


@pytest.mark.parametrize('mode', ['LOITER', 'ALT_HOLD', 'RTL', 'LAND'])
def test_supervisor_reports_release_and_stop_preserves_takeover(supervisor, monkeypatch, mode):
    node = supervisor
    events = []
    def prepare():
        own_flight(node)
        healthy(node)
    def tick(*args, **kwargs):
        healthy(node, position=10)  # Takeover must win over arrival in this tick.
        node.states['test_uav'].mode = mode
    node.prepare_flight = prepare
    node.service_call = lambda *args: events.append(args)
    monkeypatch.setattr(rclpy, 'spin_once', tick)
    assert start(node).success
    node.process_pending()
    assert node.mission_state == 'released'
    assert node.status()['ownership'] == {'test_uav': 'released'}
    assert not node.flight_owned
    assert events == [('/mpc/enable', False), ('/target/run', False)]
    assert not start(node).success
    node.request_stop(Trigger.Request(), Trigger.Response())
    node.process_pending()
    assert node.mission_state == 'released'
    assert node.states['test_uav'].mode == mode
    # Disarm and a new explicit start are required to clear released ownership.
    node.states['test_uav'].armed = False
    assert start(node).success
    assert node.ownership['test_uav'] == Ownership.UNOWNED


def test_target_reset_after_takeover_keeps_flight_released(supervisor):
    node = supervisor
    healthy(node)
    node.ownership['test_uav'] = Ownership.RELEASED
    node.mission_state = 'released'
    calls = []
    node.service_call = lambda name: calls.append(name)
    assert node.request_reset(Trigger.Request(), Trigger.Response()).success
    node.process_pending()
    assert calls == ['/target/reset']
    assert node.ownership['test_uav'] == Ownership.RELEASED
    assert not start(node).success


@pytest.mark.parametrize('unavailable', ['discovery', 'reply', 'rejected'])
def test_cleanup_land_does_not_wait_for_helper_services(supervisor, monkeypatch, unavailable):
    from solo_mpc import mission_node
    from solo_bringup import flight
    from mavros_msgs.srv import SetMode

    node = supervisor
    own_flight(node)
    healthy(node)
    now = [100.0]
    commands = []
    helper_futures = []
    def helper(request):
        future = Future()
        helper_futures.append(future)
        if unavailable == 'rejected':
            future.set_result(SimpleNamespace(success=False))
        return future
    node.mission_clients = {name: SimpleNamespace(
        service_is_ready=lambda: unavailable != 'discovery', call_async=helper,
    ) for name in node.mission_clients}
    def land(request):
        commands.append((request.custom_mode, now[0]))
        future = Future()
        future.set_result(SetMode.Response(mode_sent=True))
        return future
    node.command_clients[('test_uav', 'set_mode')] = SimpleNamespace(
        service_is_ready=lambda: True, call_async=land)
    def tick(*args, **kwargs):
        now[0] = round(now[0] + 0.1, 6)
        # Landing completes long before helper discovery/reply times out.
        if now[0] >= 100.3:
            node.states['test_uav'].armed = False
    monkeypatch.setattr(mission_node, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setattr(flight, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setattr(rclpy, 'spin_once', tick)
    with pytest.raises(RuntimeError, match='rejected' if unavailable == 'rejected' else 'cleanup'):
        node.stop_flight()
    assert commands == [('LAND', 100.0)]
    assert now[0] <= 102.0
    assert all(future.done() for future in helper_futures)
    assert not node.flight_owned
