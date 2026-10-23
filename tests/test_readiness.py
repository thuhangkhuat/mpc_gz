"""Validate diagnostic checks against the existing live readiness predicate."""

import json
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from solo_bringup import readiness


class ReadyPlugins:
    def __init__(self, *args):
        pass

    def tick(self):
        return True

    def status(self):
        return {'/alpha/mavros/local_position': {'ready': True, 'phase': 'ready'}}


def test_clock_reports_speed_stall_and_reset_without_relaxing_readiness(tmp_path, monkeypatch):
    monkeypatch.setattr(readiness, 'PluginParameters', ReadyPlugins)
    wall = [100.0]
    monkeypatch.setattr(readiness.time, 'monotonic', lambda: wall[0])
    rclpy.init()
    node = readiness.Readiness({'uavs': [{'name': 'alpha'}]}, tmp_path, 600, 10, simulation=True)

    def clock(seconds):
        node.clock(SimpleNamespace(clock=SimpleNamespace(sec=seconds, nanosec=0)))

    try:
        node.tick()
        assert json.loads((tmp_path / 'health.json').read_text())['simulation']['time_s'] is None
        clock(1)
        node.tick()
        wall[0] = 110.0
        clock(2)
        node.status_text(SimpleNamespace(text='EKF3 IMU0 initialised'), 'alpha')
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert health['simulation']['real_time_factor'] == pytest.approx(0.1)
        assert health['uavs']['alpha']['last_fcu_status'] == 'EKF3 IMU0 initialised'
        assert not health['ready']
        assert health['first_ready_elapsed_wall_s'] is None
        wall[0] = 116.0
        node.tick()
        assert json.loads((tmp_path / 'health.json').read_text())['simulation']['clock_age_wall_s'] == 6
        clock(0)
        node.tick()
        assert json.loads((tmp_path / 'health.json').read_text())['simulation']['real_time_factor'] is None
        record = node.records['alpha']
        record.update(state=SimpleNamespace(connected=True, armed=False, mode='STABILIZE'),
                      state_at=116, intervals={30, 32}, pose_count=3, pose_at=116)
        node.tick()
        assert json.loads((tmp_path / 'health.json').read_text())['first_ready_elapsed_wall_s'] == 16
        wall[0] = 117.0
        node.tick()
        assert json.loads((tmp_path / 'health.json').read_text())['first_ready_elapsed_wall_s'] == 16
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_telemetry_is_serial_but_success_dispatches_next_request_immediately(tmp_path, monkeypatch):
    monkeypatch.setattr(readiness, 'PluginParameters', ReadyPlugins)
    wall = [100.0]
    monkeypatch.setattr(readiness.time, 'monotonic', lambda: wall[0])
    rclpy.init()
    node = readiness.Readiness({'uavs': [{'name': 'alpha'}]}, tmp_path, 600, 10)
    requests = []
    pending = SimpleNamespace(done=lambda: False, cancel=lambda: None)

    def call(request):
        requests.append(request.message_id)
        return pending

    try:
        record = node.records['alpha']
        record.update(state=SimpleNamespace(connected=True, armed=False, mode='STABILIZE'),
                      state_at=100, client=SimpleNamespace(service_is_ready=lambda: True, call_async=call))
        node.tick()
        node.tick()
        assert requests == [32]  # No overlapping command 511 acknowledgements.
        pending.done = lambda: True
        pending.result = lambda: SimpleNamespace(success=True)
        node.tick()
        assert requests == [32, 30]
        pending.result = lambda: SimpleNamespace(success=False)
        node.tick()
        node.tick()
        assert requests == [32, 30]  # Rejected requests retain their retry backoff.
        wall[0] += 1
        node.tick()
        assert requests == [32, 30, 30]
        node.state(SimpleNamespace(connected=False), 'alpha')
        assert not record['intervals']
        assert record['future'] is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_readiness_explains_telemetry_pose_and_fleet_parameter_waits(tmp_path, monkeypatch):
    class Plugins:
        configured = False

        def __init__(self, *args):
            pass

        def tick(self):
            return self.configured

        def status(self):
            return {
                '/alpha/mavros/local_position': {'ready': True, 'phase': 'ready'},
                '/bravo/mavros/local_position': {'ready': self.configured, 'phase': 'applying parameters'},
            }

    monkeypatch.setattr(readiness, 'PluginParameters', Plugins)
    monkeypatch.setattr(readiness.time, 'monotonic', lambda: 100)
    rclpy.init()
    node = readiness.Readiness({'uavs': [{'name': 'alpha'}, {'name': 'bravo'}]},
                               tmp_path, timeout=600, max_pose_age=10)
    try:
        for record in node.records.values():
            record.update(state=SimpleNamespace(connected=True, armed=False, mode='STABILIZE'),
                          state_at=100, client=SimpleNamespace(service_is_ready=lambda: False))
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        checks = health['uavs']['alpha']['checks']
        assert checks['FCU']['ready']
        assert checks['MAVROS parameters']['ready']
        assert checks['telemetry']['detail'] == 'waiting for MAVROS set_message_interval service'
        assert checks['local pose']['detail'].startswith('waiting for first local pose')
        assert not health['ready']
        for record in node.records.values():
            record.update(intervals={30, 32}, pose_count=3, pose_at=100)
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert all(check['ready'] for check in health['uavs']['alpha']['checks'].values())
        assert not health['uavs']['alpha']['ready']  # The other UAV's parameters still gate the fleet.
        node.plugin_parameters.configured = True
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert health['ready']
        assert (tmp_path / 'ready.json').exists()
        node.records['alpha']['pose_at'] = 80
        node.tick()
        health = json.loads((tmp_path / 'health.json').read_text())
        assert not health['ready']
        assert health['uavs']['alpha']['checks']['local pose']['detail'] == 'local pose stream stale'
    finally:
        node.destroy_node()
        rclpy.shutdown()
