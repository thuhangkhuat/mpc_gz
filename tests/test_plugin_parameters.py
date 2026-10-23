from types import SimpleNamespace

import pytest
import yaml

pytest.importorskip('rclpy')
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter
from solo_bringup.parameters import PluginParameters, changed_parameters


def value(name, data):
    return Parameter(name, value=data).to_parameter_msg()


def test_runtime_attitude_mode_change_requires_safe_upstream_callback():
    desired = value('use_quaternion', True)
    with pytest.raises(RuntimeError, match='use_quaternion at runtime is unsafe'):
        changed_parameters('/uav1/mavros/setpoint_attitude', [desired], [value('use_quaternion', False).value])


class FakeNode:
    def __init__(self):
        self.values = {
            '/uav1/mavros/sys': {'heartbeat_rate': value('heartbeat_rate', 1.0).value},
            '/uav1/mavros/imu': {'frame_id': value('frame_id', 'base_link').value},
            '/uav1/mavros/local_position': {'frame_id': value('frame_id', 'map').value},
            '/uav1/mavros/setpoint_attitude': {'use_quaternion': value('use_quaternion', False).value},
        }
        self.set_calls = []

    def get_node_names_and_namespaces(self):
        return [tuple(reversed(target.rsplit('/', 1))) for target in self.values]

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warning=lambda _: None)

    def create_client(self, service, path):
        target = path.rsplit('/', 1)[0]

        def call(request):
            if service is ListParameters:
                response = SimpleNamespace(result=SimpleNamespace(names=list(self.values[target])))
            elif service is GetParameters:
                response = SimpleNamespace(values=[self.values[target][name] for name in request.names])
            else:
                assert service is SetParameters
                # Re-entering this callback used to recreate a subscription with dangling QoS.
                assert not any(p.name == 'use_quaternion' for p in request.parameters)
                self.set_calls.append((target, [p.name for p in request.parameters]))
                for parameter in request.parameters:
                    self.values[target][parameter.name] = parameter.value
                response = SimpleNamespace(results=[SimpleNamespace(successful=True)
                                                    for _ in request.parameters])
            return SimpleNamespace(done=lambda: True, result=lambda: response, cancel=lambda: None)

        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)


def test_bootstrap_changes_frames_without_reentering_unchanged_callbacks(tmp_path):
    data = {
        '/**/sys': {'ros__parameters': {'heartbeat_rate': 1.0}},
        '/**/imu': {'ros__parameters': {'frame_id': 'uav1/base_link'}},
        '/**/local_position': {'ros__parameters': {'frame_id': 'uav1/map'}},
        '/**/setpoint_attitude': {'ros__parameters': {'use_quaternion': False}},
    }
    (tmp_path / 'uav1-mavros.yaml').write_text(yaml.safe_dump(data))
    node = FakeNode()
    for _ in range(2):
        bootstrap = PluginParameters(node, [{'name': 'uav1'}], tmp_path)
        for _ in range(12):
            if bootstrap.tick():
                break
        else:
            pytest.fail('Plugin bootstrap did not complete')
    assert node.set_calls == [('/uav1/mavros/imu', ['frame_id']),
                              ('/uav1/mavros/local_position', ['frame_id'])]
    assert node.values['/uav1/mavros/local_position']['frame_id'].string_value == 'uav1/map'


def test_plugin_progress_distinguishes_missing_nodes_services_and_parameter_phases(tmp_path):
    data = {f'/**/{name}': {'ros__parameters': values} for name, values in {
        'sys': {'heartbeat_rate': 1.0}, 'imu': {'frame_id': 'uav1/base_link'},
        'local_position': {'frame_id': 'uav1/map'},
    }.items()}
    (tmp_path / 'uav1-mavros.yaml').write_text(yaml.safe_dump(data))
    node = FakeNode()
    bootstrap = PluginParameters(node, [{'name': 'uav1'}], tmp_path)
    imu = '/uav1/mavros/imu'
    assert bootstrap.status()[imu]['phase'] == 'waiting for node'
    assert not bootstrap.tick()
    assert bootstrap.status()[imu]['phase'] == 'listing parameters'
    bootstrap.tick()
    assert bootstrap.status()[imu]['phase'] == 'reading parameters'
    bootstrap.tick()
    assert bootstrap.status()[imu]['phase'] == 'applying parameters'
    # The next phase is dispatched in the same tick as the previous response.
    assert bootstrap.items[imu]['future'] is not None
    bootstrap.items[imu]['future'] = None
    bootstrap.items[imu]['client'].service_is_ready = lambda: False
    assert bootstrap.status()[imu]['phase'] == 'waiting for parameter services'
    bootstrap.items[imu]['client'].service_is_ready = lambda: True
    for _ in range(6):
        bootstrap.tick()
    assert all(item['ready'] for item in bootstrap.status().values())
