"""Exercise concurrent takeoff and the fleet altitude barrier with a fake FCU."""
from concurrent.futures import Future
import json
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from solo_bringup import flight


class SimulatedFlight:
    wait = flight.Flight.wait
    connected = flight.Flight.connected
    check_response = flight.Flight.check_response
    takeoff = flight.Flight.takeoff
    wait_for_stable_altitude = flight.Flight.wait_for_stable_altitude
    state = flight.Flight.state
    observe_ownership = flight.Flight.observe_ownership
    check_ownership = flight.Flight.check_ownership
    land = flight.Flight.land

    def __init__(self, monkeypatch):
        self.now = 100.0
        self.sim_now = 100.0
        self.sim_rate = 1.0
        self.pose_interval_wall = 0.0
        self.timeout = 15.0
        self.names = ('uav1', 'uav2', 'uav3')
        self.ownership = dict.fromkeys(self.names, flight.Ownership.UNOWNED)
        self.states = {n: State(connected=True, mode='STABILIZE') for n in self.names}
        self.poses = {}
        self.state_at = {}
        self.pose_at = {}
        self.pose_sim_at = {}
        self.commands = []
        self.pending = []
        self.delays = {}
        self.rejections = set()
        self.launches = {}
        self.altitudes = lambda n, t: min(3.0, max(0.0, t - self.launches.get(n, t)))
        self.frozen = set()
        self.command_clients = {
            (n, suffix): SimpleNamespace(
                service_is_ready=lambda: True,
                call_async=lambda req, n=n, s=suffix: self.command(n, s, req),
            )
            for n in self.names for suffix in ('set_mode', 'cmd/arming', 'cmd/takeoff')
        }
        monkeypatch.setattr(flight, 'time', SimpleNamespace(monotonic=lambda: self.now))
        monkeypatch.setattr(flight.rclpy, 'ok', lambda: True)
        monkeypatch.setattr(flight.rclpy, 'spin_once', lambda *a, **kw: self.tick())

    def command(self, name, suffix, request):
        self.commands.append((name, suffix, self.now))
        future = Future()
        self.pending.append((self.now + self.delays.get((name, suffix), 0.1),
                             name, suffix, request, future))
        return future

    def tick(self):
        self.now = round(self.now + 0.1, 6)
        self.sim_now = round(self.sim_now + 0.1 * self.sim_rate, 6)
        for item in list(self.pending):
            deadline, name, suffix, request, future = item
            if self.now < deadline:
                continue
            self.pending.remove(item)
            accepted = (name, suffix) not in self.rejections
            if suffix == 'set_mode':
                response = SetMode.Response(mode_sent=accepted)
                if accepted:
                    self.states[name].mode = request.custom_mode
            elif suffix == 'cmd/arming':
                response = CommandBool.Response(success=accepted, result=0 if accepted else 4)
                if accepted:
                    self.states[name].armed = True
            else:
                response = CommandTOL.Response(success=accepted, result=0 if accepted else 4)
                if accepted:
                    self.launches[name] = self.now
            future.set_result(response)
        for name in self.names:
            self.state_at[name] = self.now
            if name in self.frozen:
                continue
            if self.now - self.pose_at.get(name, 0) < self.pose_interval_wall:
                continue
            pose = PoseStamped()
            pose.header.stamp.sec = int(self.now)
            pose.header.stamp.nanosec = round((self.now % 1) * 1e9)
            pose.pose.position.z = self.altitudes(name, self.now)
            self.poses[name] = pose
            self.pose_at[name] = self.now
            self.pose_sim_at[name] = self.sim_now


@pytest.fixture
def fleet(monkeypatch):
    return SimulatedFlight(monkeypatch)


def test_takeoff_launches_fleet_before_waiting_for_altitude(fleet):
    fleet.takeoff(fleet.names, 3.0)
    launches = list(fleet.launches.values())
    assert len(launches) == 3
    assert max(launches) - min(launches) < 0.2
    assert fleet.now >= max(launches) + 5.0


def test_slow_arm_reply_does_not_block_other_takeoffs(fleet):
    fleet.delays[('uav1', 'cmd/arming')] = 4.0
    fleet.takeoff(fleet.names, 3.0)
    assert fleet.launches['uav2'] < fleet.launches['uav1'] - 3.0
    assert fleet.launches['uav3'] < fleet.launches['uav1'] - 3.0
    assert fleet.now >= fleet.launches['uav1'] + 5.0


def test_preflight_checks_entire_fleet_before_arming(fleet):
    fleet.states['uav3'].armed = True
    with pytest.raises(RuntimeError, match='uav3: already armed'):
        fleet.takeoff(fleet.names, 3.0)
    assert not fleet.commands


@pytest.mark.parametrize('suffix', ['set_mode', 'cmd/arming', 'cmd/takeoff'])
def test_rejected_command_aborts_takeoff(fleet, suffix):
    fleet.rejections.add(('uav2', suffix))
    with pytest.raises(RuntimeError, match=f'uav2/{suffix}:'):
        fleet.takeoff(fleet.names, 3.0)


def test_missing_reply_times_out(fleet):
    fleet.delays[('uav2', 'cmd/arming')] = 100.0
    with pytest.raises(RuntimeError, match='Timeout: uav2/cmd/arming'):
        fleet.takeoff(fleet.names, 3.0)
    assert set(fleet.launches) == {'uav1', 'uav3'}


def hover(fleet):
    for state in fleet.states.values():
        state.armed, state.mode = True, 'GUIDED'
    fleet.altitudes = lambda n, t: 3.0


def test_altitude_crossing_while_climbing_is_not_stable(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: 3.0 + 0.3 * (t - 100.0) if n == 'uav3' else 3.0
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_earlier_uav_must_remain_stable_while_last_one_climbs(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: (4.0 if n == 'uav1' and t >= 103.0
                                    else 0.0 if n == 'uav3' and t < 104.0 else 3.0)
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_altitude_departure_resets_settling_interval(fleet):
    hover(fleet)
    fleet.altitudes = lambda n, t: 4.0 if n == 'uav2' and 101.5 <= t < 102.0 else 3.0
    fleet.wait_for_stable_altitude(fleet.names, 3.0)
    assert fleet.now >= 104.0


def test_frozen_pose_cannot_complete_settling_interval(fleet):
    hover(fleet)
    fleet.tick()
    fleet.frozen.add('uav3')
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_slow_simulation_settles_using_sim_time_despite_wall_pose_gaps(fleet):
    hover(fleet)
    fleet.sim_rate = 0.1
    fleet.pose_interval_wall = 1.2  # A fresh 10 Hz stream in a slow simulator.
    fleet.timeout = 60
    fleet.wait_for_stable_altitude(fleet.names, 3.0)
    assert 120 <= fleet.now < 140


def test_paused_simulation_cannot_complete_settling_interval(fleet):
    hover(fleet)
    fleet.sim_rate = 0
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


def test_repeated_pose_stamp_cannot_accumulate_simulation_hold(fleet):
    hover(fleet)
    tick = fleet.tick
    def repeat_stamps():
        tick()
        for pose in fleet.poses.values():
            pose.header.stamp.sec = 100
            pose.header.stamp.nanosec = 0
    fleet.tick = repeat_stamps
    with pytest.raises(RuntimeError, match='fleet stable'):
        fleet.wait_for_stable_altitude(fleet.names, 3.0)


@pytest.mark.parametrize('outcome', ['tracking', 'takeoff_failure', 'cancelled'])
def test_mission_moves_only_after_entire_fleet_settles(
        fleet, monkeypatch, tmp_path, outcome):
    import rclpy
    from solo_mpc.mission_node import MissionSupervisor
    from solo_mpc_interfaces.srv import StartMission

    events = []
    (tmp_path / 'manifest.json').write_text(json.dumps({
        'mode': 'simulation', 'profile': 'mpc',
        'config': {'uavs': [{'name': n} for n in fleet.names]},
    }))
    (tmp_path / 'controller.yaml').write_text('flight_altitude: 3.0\ntarget_mode: camera\n')
    (tmp_path / 'scenario.yaml').write_text('{}')
    (tmp_path / 'health.json').write_text(json.dumps({
        'ready': True, 'updated_wall_unix': time.time(),
    }))
    (tmp_path / 'alignment.json').write_text(json.dumps(dict.fromkeys(fleet.names)))
    rclpy.init()
    node = MissionSupervisor(tmp_path)
    try:
        def client(name):
            def call(request):
                events.append((name, getattr(request, 'data', None), fleet.now))
                future = Future()
                future.set_result(SimpleNamespace(success=True))
                return future
            return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

        node.mission_clients = {name: client(name) for name in node.mission_clients}
        node.wait = fleet.wait
        node.takeoff = fleet.takeoff
        node.ownership = fleet.ownership
        def land(names, *, poll):
            poll()
            events.append(('land', None, fleet.now))
        node.land = land
        node.snapshots = {'reason': 'ready', 'vehicles': {
            n: {'result': 'tracking'} for n in fleet.names}}
        node.supervise = lambda: events.append(('supervise', None, fleet.now))
        if outcome == 'cancelled':
            tick = fleet.tick
            def interrupt_takeoff():
                tick()
                if fleet.launches:
                    node.stop_requested = True
                node.check_stop()
            fleet.tick = interrupt_takeoff
        elif outcome == 'takeoff_failure':
            fleet.altitudes = lambda n, t: 0.0 if n == 'uav3' else 3.0
        response = node.request_start(
            StartMission.Request(run_dir=str(tmp_path)), StartMission.Response())
        assert response.success
        assert not events  # Service callback queues work; it cannot fly itself.
        node.process_pending()
        starts = [(name, t) for name, enabled, t in events if enabled is True]
        if outcome == 'tracking':
            assert [name for name, t in starts] == ['/mpc/enable', '/target/run']
            assert all(t >= max(fleet.launches.values()) + 5.0 for name, t in starts)
            assert any(name == 'supervise' for name, _, _ in events)
            assert node.mission_state == 'completed'
        else:
            assert not starts
            assert node.mission_state == ('stopped' if outcome == 'cancelled' else 'failed')
            if outcome == 'takeoff_failure':
                assert 'fleet stable' in node.error
        assert events[-1][0] == 'land'
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('mode', ['LOITER', 'ALT_HOLD', 'RTL', 'LAND', 'STABILIZE'])
def test_pilot_or_fcu_takeover_is_latched_and_excluded_from_land(fleet, mode):
    hover(fleet)
    fleet.tick()
    fleet.ownership.update(dict.fromkeys(fleet.names, flight.Ownership.OWNED))
    fleet.state(State(connected=True, armed=True, mode=mode), 'uav1')
    with pytest.raises(flight.FlightReleased, match='uav1'):
        fleet.check_ownership()
    # A subsequent GUIDED heartbeat cannot silently reacquire control.
    fleet.state(State(connected=True, armed=True, mode='GUIDED'), 'uav1')
    fleet.land(['uav1'])
    assert fleet.ownership['uav1'] == flight.Ownership.RELEASED
    assert not fleet.commands


@pytest.mark.parametrize('phase', ['arming', 'settling'])
def test_takeover_aborts_takeoff_without_further_commands(fleet, phase):
    if phase == 'arming':
        fleet.delays[('uav1', 'cmd/arming')] = 3.0
    tick = fleet.tick
    def takeover():
        tick()
        if (phase == 'arming' and fleet.now >= 100.5
                or phase == 'settling' and fleet.launches):
            fleet.state(State(connected=True, armed=phase == 'settling', mode='RTL'), 'uav1')
    fleet.tick = takeover
    with pytest.raises(flight.FlightReleased, match='uav1'):
        fleet.takeoff(fleet.names, 3.0)
    assert fleet.ownership['uav1'] == flight.Ownership.RELEASED
    if phase == 'arming':
        assert 'uav1' not in fleet.launches
    before = list(fleet.commands)
    fleet.land(['uav1'])
    assert fleet.commands == before


def test_landing_dispatch_and_deadlines_are_independent_of_other_fcus(fleet):
    hover(fleet)
    fleet.tick()
    fleet.ownership.update(dict.fromkeys(fleet.names, flight.Ownership.OWNED))
    fleet.delays[('uav1', 'set_mode')] = 100.0
    fleet.command_clients[('uav2', 'set_mode')].service_is_ready = lambda: False
    tick = fleet.tick
    def disarm_third():
        tick()
        if fleet.now >= 102:
            fleet.states['uav3'].armed = False
    fleet.tick = disarm_third
    started = fleet.now
    with pytest.raises(RuntimeError) as exc:
        fleet.land(fleet.names, landing_timeout=fleet.timeout)
    assert 'uav1/LAND reply' in str(exc.value)
    assert 'uav2/LAND service or connection' in str(exc.value)
    assert all(t == started for _, _, t in fleet.commands)
    assert {n for n, _, _ in fleet.commands} == {'uav1', 'uav3'}
    assert fleet.now - started == pytest.approx(fleet.timeout)
    assert fleet.ownership['uav3'] == flight.Ownership.UNOWNED


@pytest.mark.parametrize('mode', ['RTL', 'LAND'])
def test_takeover_while_waiting_for_land_service_never_dispatches(fleet, mode):
    hover(fleet)
    fleet.tick()
    fleet.ownership['uav1'] = flight.Ownership.OWNED
    fleet.command_clients[('uav1', 'set_mode')].service_is_ready = lambda: fleet.now >= 102
    tick = fleet.tick
    def takeover():
        tick()
        fleet.state(State(connected=True, armed=True, mode=mode), 'uav1')
    fleet.tick = takeover
    fleet.land(['uav1'])
    assert not fleet.commands
    assert fleet.now < 102
    assert fleet.ownership['uav1'] == flight.Ownership.RELEASED


def test_pilot_can_override_companion_landing_with_guided(fleet):
    hover(fleet)
    fleet.tick()
    fleet.ownership['uav1'] = flight.Ownership.OWNED
    tick = fleet.tick
    def takeover():
        tick()
        fleet.observe_ownership('uav1')
        if fleet.now >= 101:
            fleet.state(State(connected=True, armed=True, mode='GUIDED'), 'uav1')
    fleet.tick = takeover
    fleet.land(['uav1'])
    assert fleet.ownership['uav1'] == flight.Ownership.RELEASED
    assert len(fleet.commands) == 1


def test_failed_preflight_does_not_claim_or_land_already_armed_aircraft(fleet):
    fleet.states['uav3'].armed = True
    with pytest.raises(RuntimeError, match='already armed'):
        fleet.takeoff(fleet.names, 3.0)
    fleet.land(fleet.names)
    assert not fleet.commands
    assert set(fleet.ownership.values()) == {flight.Ownership.UNOWNED}


def test_cleanup_lands_remaining_owned_vehicles_after_one_takeover(fleet):
    hover(fleet)
    fleet.tick()
    fleet.ownership.update(dict.fromkeys(fleet.names, flight.Ownership.OWNED))
    fleet.state(State(connected=True, armed=True, mode='RTL'), 'uav1')
    tick = fleet.tick
    def disarm_owned():
        tick()
        for name in ('uav2', 'uav3'):
            fleet.states[name].armed = False
    fleet.tick = disarm_owned
    fleet.land(fleet.names)
    assert {n for n, _, _ in fleet.commands} == {'uav2', 'uav3'}
    assert fleet.states['uav1'].mode == 'RTL'
    assert fleet.ownership['uav1'] == flight.Ownership.RELEASED
    assert fleet.ownership['uav2'] == fleet.ownership['uav3'] == flight.Ownership.UNOWNED


def test_takeover_heartbeat_is_latched_between_supervision_ticks(fleet):
    hover(fleet)
    fleet.tick()
    fleet.ownership['uav1'] = flight.Ownership.OWNED
    fleet.state(State(connected=True, armed=True, mode='LOITER'), 'uav1')
    fleet.state(State(connected=True, armed=True, mode='GUIDED'), 'uav1')
    with pytest.raises(flight.FlightReleased):
        fleet.check_ownership()


def test_paused_simulation_does_not_extend_landing_deadline(fleet):
    hover(fleet)
    fleet.tick()
    fleet.sim_rate = 0
    fleet.ownership['uav1'] = flight.Ownership.OWNED
    started = fleet.now
    sim_started = fleet.sim_now
    with pytest.raises(RuntimeError, match='landed/disarmed'):
        fleet.land(['uav1'], landing_timeout=3.0)
    assert fleet.now - started == pytest.approx(3.0)
    assert fleet.sim_now == sim_started
