import json

import pytest
import yaml

from solo_bringup.configuration import load_config
from solo_bringup.runtime import create_run, mavros_config
from test_configuration_world import ROOT


def test_state_isolation_and_explicit_resume(tmp_path, monkeypatch):
    monkeypatch.setenv('RUNS_ROOT', str(tmp_path))
    config = load_config(ROOT / 'config/fleet.yaml')
    first = create_run(config, 'simulation', 'control', 'first')
    state = first / 'sitl/0/eeprom.bin'
    state.parent.mkdir(parents=True)
    state.write_bytes(b'persisted parameters')
    second = create_run(config, 'simulation', 'control', 'second')
    assert (tmp_path / 'latest').resolve() == second
    assert not (second / 'sitl/0/eeprom.bin').exists()
    with pytest.raises(ValueError):
        create_run(config, 'simulation', 'control', 'first')
    (first / 'health.json').write_text('{}')
    resumed = create_run(config, 'simulation', 'control', 'first', resume=True)
    assert resumed == first
    assert state.read_bytes() == b'persisted parameters'
    assert not (first / 'health.json').exists()
    assert json.loads((first / 'manifest.json').read_text())['mode'] == 'simulation'
    with pytest.raises(ValueError):
        create_run(config, 'simulation', 'reference', 'first', resume=True)


@pytest.mark.parametrize('run_id', ['latest', 'hardware', '../outside', '/absolute'])
def test_reject_reserved_or_escaping_run_ids(tmp_path, monkeypatch, run_id):
    monkeypatch.setenv('RUNS_ROOT', str(tmp_path))
    with pytest.raises(ValueError):
        create_run({}, 'simulation', run_id=run_id)


@pytest.mark.parametrize('simulation', [False, True])
def test_mavros_slow_simulation_overrides_preserve_hardware_defaults(tmp_path, monkeypatch, simulation):
    pytest.importorskip('ament_index_python')
    import ament_index_python.packages

    launch = tmp_path / 'launch'
    launch.mkdir()
    original = {
        '/**': {'ros__parameters': {'startup_px4_usb_quirk': False}},
        '/**/sys': {'ros__parameters': {'conn_timeout': 10.0}},
        '/**/time': {'ros__parameters': {'timesync_mode': 'MAVLINK', 'timesync_rate': 10.0,
                                      'system_time_rate': 1.0}},
    }
    (launch / 'apm_config.yaml').write_text(yaml.safe_dump(original))
    (launch / 'apm_pluginlists.yaml').write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'plugin_denylist': ['ftp', 'hil']}},
    }))
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lambda _: str(tmp_path))
    destination = tmp_path / 'mavros.yaml'
    mavros_config({'name': 'alpha'}, destination, simulation=simulation)
    result = yaml.safe_load(destination.read_text())
    if simulation:
        assert result['/**/sys']['ros__parameters']['conn_timeout'] == 60
        assert result['/**/time']['ros__parameters'] == {
            'timesync_mode': 'NONE', 'timesync_rate': 0, 'system_time_rate': 0,
        }
        assert result['/**']['ros__parameters']['plugin_denylist'] == ['ftp', 'hil', 'param']
    else:
        assert result == original
