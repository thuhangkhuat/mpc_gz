"""Apply plugin YAML through services: pinned MAVROS disables global arguments on plugins."""
from pathlib import Path
import time

from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter
import yaml


def flatten_parameters(values, prefix=''):
    for name, value in values.items():
        key = prefix + name
        if isinstance(value, dict):
            yield from flatten_parameters(value, key + '.')
        else:
            yield key, value


def changed_parameters(target, parameters, current_values):
    if len(parameters) != len(current_values):
        raise RuntimeError(f'{target}: incomplete parameter readback')
    changed = [parameter for parameter, current in zip(parameters, current_values)
               if parameter.value != current]
    # This commit captures constructor-local subscriber_qos by reference in this
    # callback. Re-entering it after construction can read invalid stack memory.
    if target.endswith('/setpoint_attitude') and any(p.name == 'use_quaternion' for p in changed):
        raise RuntimeError(f'{target}: changing use_quaternion at runtime is unsafe in pinned '
                           'MAVROS; patch its QoS callback before changing the attitude input mode')
    return changed


class PluginParameters:
    def __init__(self, node, uavs, run_dir):
        self.node = node
        self.items = {}
        self.required = set()
        self.active = set()
        for uav in uavs:
            prefix = f"/{uav['name']}/mavros/"
            self.required.update(prefix + plugin for plugin in ('local_position', 'imu', 'sys'))
            data = yaml.safe_load((Path(run_dir) / f"{uav['name']}-mavros.yaml").read_text())
            for selector, config in data.items():
                if not selector.startswith('/**/'):
                    continue
                target = prefix + selector.removeprefix('/**/')
                values = config['ros__parameters']
                self.items[target] = {
                    'client': node.create_client(SetParameters, target + '/set_parameters'),
                    'list_client': node.create_client(ListParameters, target + '/list_parameters'),
                    'get_client': node.create_client(GetParameters, target + '/get_parameters'),
                    'request': SetParameters.Request(parameters=[
                        Parameter(key, value=value).to_parameter_msg()
                        for key, value in flatten_parameters(values)
                    ]),
                    'future': None, 'done': False, 'at': 0, 'phase': 'list',
                }

    def tick(self):
        now = time.monotonic()
        active = {f"{namespace.rstrip('/')}/{name}" for name, namespace in
                  self.node.get_node_names_and_namespaces()}
        self.active = active
        for target, item in self.items.items():
            if item['done'] or target not in active:
                continue
            future = item['future']
            if future is not None:
                if future.done():
                    response = future.result()
                    item['future'] = None
                    if response is None:
                        raise RuntimeError(f'{target}: no plugin parameter response')
                    if item['phase'] == 'list':
                        declared = set(response.result.names)
                        parameters = item['request'].parameters
                        obsolete = [p.name for p in parameters if p.name not in declared]
                        if target.endswith(('/local_position', '/imu')) and 'frame_id' not in declared:
                            raise RuntimeError(f'{target}: required frame_id parameter missing')
                        if obsolete:
                            self.node.get_logger().warning(f'{target}: upstream YAML has undeclared keys; ignored: {obsolete}')
                        item['request'].parameters = [p for p in parameters if p.name in declared]
                        item['phase'] = 'get'
                    elif item['phase'] == 'get':
                        item['request'].parameters = changed_parameters(
                            target, item['request'].parameters, response.values)
                        if item['request'].parameters:
                            item['phase'] = 'set'
                        else:
                            item['done'] = True
                            self.node.get_logger().info(f'{target}: plugin parameters already correct')
                    else:
                        if not all(result.successful for result in response.results):
                            raise RuntimeError(f'{target}: plugin parameters rejected: {response}')
                        item['done'] = True
                        self.node.get_logger().info(f'{target}: plugin parameters applied')
                elif now - item['at'] > 20:
                    future.cancel()
                    item['future'] = None
            if not item['done'] and item['future'] is None and all(
                    item[key].service_is_ready() for key in ('client', 'list_client', 'get_client')):
                if item['phase'] == 'list':
                    item['future'] = item['list_client'].call_async(ListParameters.Request(depth=0))
                elif item['phase'] == 'get':
                    item['future'] = item['get_client'].call_async(GetParameters.Request(
                        names=[parameter.name for parameter in item['request'].parameters]))
                else:
                    item['future'] = item['client'].call_async(item['request'])
                item['at'] = now
        expected = (active & self.items.keys()) | self.required
        return all(self.items[target]['done'] for target in expected)

    def status(self):
        """Expose the same required/active plugin set used by the readiness gate."""
        expected = (self.active & self.items.keys()) | self.required
        report = {}
        for target in sorted(expected):
            item = self.items[target]
            if item['done']:
                phase = 'ready'
            elif target not in self.active:
                phase = 'waiting for node'
            elif item['future'] is not None or all(item[key].service_is_ready()
                                                  for key in ('client', 'list_client', 'get_client')):
                phase = {'list': 'listing parameters', 'get': 'reading parameters',
                         'set': 'applying parameters'}[item['phase']]
            else:
                phase = 'waiting for parameter services'
            report[target] = {'ready': item['done'], 'phase': phase}
        return report
