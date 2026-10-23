"""Configuration validation shared by world generation and mission launch."""
import math
import re
from pathlib import Path

import yaml


def load_config(path, hardware=False):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Expected schema_version: 1')
    uavs = data.get('uavs')
    if not isinstance(uavs, list) or not uavs:
        raise ValueError('uavs must be a nonempty list')
    names, systems, instances = set(), set(), set()
    for uav in uavs:
        name = uav.get('name', '')
        if not re.fullmatch(r'[a-z][a-z0-9_]*', name) or name in names:
            raise ValueError(f'Invalid/duplicate UAV name: {name}')
        names.add(name)
        sysid = uav.get('sysid')
        if type(sysid) is not int or not 1 <= sysid <= 255 or sysid in systems:
            raise ValueError(f'Invalid/duplicate SYSID: {sysid}')
        systems.add(sysid)
        if hardware:
            if not isinstance(uav.get('fcu_url'), str) or not uav['fcu_url'].strip():
                raise ValueError(f'{name}: fcu_url required')
            continue
        instance = uav.get('instance')
        if type(instance) is not int or not 0 <= instance <= 200 or instance in instances:
            raise ValueError(f'Invalid/duplicate SITL instance: {instance}')
        instances.add(instance)
        spawn = uav.get('spawn')
        if not isinstance(spawn, list) or len(spawn) != 6 or not all(
            type(x) in (float, int) and math.isfinite(x) for x in spawn
        ):
            raise ValueError(f'{name}: spawn must have six finite values (degrees)')
        uav['tcp_port'] = 5760 + 10 * instance
        uav['json_port'] = 9002 + 10 * instance
        uav['fcu_url'] = f"tcp://127.0.0.1:{uav['tcp_port']}"
    if not hardware and not re.fullmatch(r'[a-z][a-z0-9_]*', data.get('world_name', '')):
        raise ValueError('Invalid world_name')
    return data
