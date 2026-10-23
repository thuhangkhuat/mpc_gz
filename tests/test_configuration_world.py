import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from solo_bringup.configuration import load_config
from solo_bringup.world import generate_world

ROOT = Path(__file__).resolve().parents[1]


def test_fleet_port_mapping():
    fleet = load_config(ROOT / 'config/fleet.yaml')
    assert [(u['tcp_port'], u['json_port'], u['sysid']) for u in fleet['uavs']] == [
        (5760, 9002, 1), (5770, 9012, 2), (5780, 9022, 3)]


@pytest.mark.parametrize('field,value', [('name', 'uav1'), ('sysid', 1), ('instance', 0),
                                        ('name', '../outside'), ('spawn', [float('nan')] * 6)])
def test_reject_conflicting_or_invalid_fleet(tmp_path, field, value):
    fleet = yaml.safe_load((ROOT / 'config/fleet.yaml').read_text())
    fleet['uavs'][1][field] = value
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump(fleet))
    with pytest.raises(ValueError):
        load_config(path)


@pytest.fixture
def upstream(tmp_path):
    # Small fixture covers nested IMU names and the gimbal removal/isolation contract.
    source = tmp_path / 'official'
    (source / 'worlds').mkdir(parents=True)
    (source / 'models/iris_with_gimbal').mkdir(parents=True)
    (source / 'models/gimbal_small_3d').mkdir(parents=True)
    (source / 'worlds/iris_runway.sdf').write_text('''<sdf version="1.9"><world name="iris_runway">
      <plugin name="gz::sim::systems::Sensors"/><plugin name="gz::sim::systems::Imu"/>
      <include><uri>model://iris_with_gimbal</uri></include></world></sdf>''')
    (source / 'models/iris_with_gimbal/model.sdf').write_text('''<sdf><model name="iris_with_gimbal">
      <include><uri>model://iris_with_standoffs</uri></include>
      <include><uri>model://gimbal_small_3d</uri><name>gimbal</name><pose>0 0 0 0 0 0</pose></include>
      <joint name="gimbal_joint"/>
      <plugin name="ArduPilotPlugin"><fdm_port_in>9002</fdm_port_in>
        <imuName>iris_with_standoffs::imu_link::imu_sensor</imuName>
        <control channel="0"/><control channel="8"><cmd_topic>/gimbal/cmd_roll</cmd_topic></control>
      </plugin><plugin name="gz::sim::systems::JointPositionController"><topic>/gimbal/cmd_roll</topic></plugin>
      </model></sdf>''')
    (source / 'models/gimbal_small_3d/model.sdf').write_text(
        '<sdf><model name="gimbal"><link name="pitch"><sensor><plugin><udp_port>5600</udp_port>'
        '</plugin><plugin name="CameraZoomPlugin"/></sensor></link></model></sdf>')
    return source


@pytest.mark.parametrize('profile', ['control', 'reference'])
def test_world_preserves_imu_and_matches_fleet(tmp_path, upstream, profile):
    fleet = load_config(ROOT / 'config/fleet.yaml')
    original = copy.deepcopy(fleet)
    output = generate_world(fleet, upstream, tmp_path / 'world.sdf', profile)
    assert fleet == original
    world = ET.parse(output).getroot().find('world')
    models = world.findall('model')
    assert [m.get('name') for m in models] == ['uav1', 'uav2', 'uav3']
    assert [m.findtext("plugin[@name='ArduPilotPlugin']/fdm_port_in") for m in models] == ['9002', '9012', '9022']
    for model in models:
        assert model.findtext("plugin[@name='ArduPilotPlugin']/imuName") == 'iris_with_standoffs::imu_link::imu_sensor'
        if profile == 'control':
            assert model.find('joint') is None
            assert model.find('model') is None
            assert len(model.find("plugin[@name='ArduPilotPlugin']").findall('control')) == 1
        else:
            assert model.find("model[@name='gimbal']") is not None
    if profile == 'reference':
        assert [m.findtext('.//udp_port') for m in models] == ['5600', '5610', '5620']
        assert [m.findtext(".//plugin[@name='CameraZoomPlugin']/topic") for m in models] == [
            '/uav1/camera/cmd_zoom', '/uav2/camera/cmd_zoom', '/uav3/camera/cmd_zoom']


def test_mission_world_rejects_mismatched_fleet(tmp_path, upstream):
    from solo_mpc.world import generate_mpc_world

    fleet = load_config(ROOT / "config/fleet.yaml")
    fleet["uavs"].pop()
    pkg = ROOT / "ros_ws/src/solo_mpc"
    with pytest.raises(ValueError, match="same number of UAVs"):
        generate_mpc_world(fleet, upstream, tmp_path / "world.sdf",
                           pkg / "scenarios/small50_3uav.yaml",
                           pkg / "config/controller.yaml")


@pytest.mark.parametrize('record_overhead', [False, True])
def test_three_uav_world_covers_field_and_configures_sensors(tmp_path, upstream, record_overhead):
    import numpy as np
    from solo_mpc.settings import load_settings
    from solo_mpc.world import bridge_config, generate_mpc_world

    pkg = ROOT / 'ros_ws/src/solo_mpc'
    scenario_path = pkg / 'scenarios/small50_3uav.yaml'
    settings_path = pkg / 'config/controller.yaml'
    scenario, settings, cfg = load_settings(scenario_path, settings_path)
    fleet = load_config(ROOT / 'config/fleet.yaml')
    fleet['world_name'] = scenario.get('world_name', 'mpc_small50')
    assert len(fleet['uavs']) == len(scenario['starts']) == 3
    assert len(cfg['GRID_CELLS']) == 2
    assert (cfg['VMAX'], cfg['UMAX'], cfg['SENSING_RADIUS']) == (1.2, 2, 3)
    assert cfg['TARGET_MAX_SPEED'] == 1.2
    assert np.allclose(np.asarray(scenario['starts'])[:, 2], settings['flight_altitude'])
    assert settings['obstacle_height'] > settings['flight_altitude'] + cfg['ROBOT_RADIUS']
    footprint = settings['flight_altitude'] * np.tan(settings['camera_hfov'] / 2)
    assert abs(footprint - scenario['viewing_radius']) < 0.1
    assert max(abs(np.asarray(scenario['starts'][0][:2]) - scenario['target']['waypoints'][0])) < footprint

    output, resolved = generate_mpc_world(fleet, upstream, tmp_path / 'world.sdf',
                                         scenario_path, settings_path, record_overhead=record_overhead)
    world = ET.parse(output).getroot().find('world')
    assert world.get('name') == 'mpc_small50'
    ground = world.find("model[@name='field_ground']")
    assert list(map(float, ground.findtext('pose').split()))[:2] == [25, 15]
    assert all(actual >= required for actual, required in zip(
        map(float, ground.findtext('.//plane/size').split()), [50, 30]))
    assert len([m for m in world.findall('model') if m.get('name').startswith('obstacle_')]) == 24
    assert world.findtext("model[@name='tracking_target']//box/size") == '0.3 0.3 0.04'
    topics, ports = [], []
    for uav, start in zip(resolved['uavs'], scenario['starts']):
        model = world.find(f"model[@name='{uav['name']}']")
        assert uav['spawn'][:3] == [*start[:2], 0.195]
        ports.append(int(model.findtext("plugin[@name='ArduPilotPlugin']/fdm_port_in")))
        assert model.findtext('.//ray/range/max') == '3.0'
        assert model.findtext('.//ray/range/min') == '0.15'
        topics.extend(s.findtext('topic') for s in model.iter('sensor'))
    assert ports == [9002, 9012, 9022]
    assert len(set(topics)) == len(topics) == 6
    if record_overhead:
        camera = world.find("model[@name='tracking_target']//sensor")
        offset = np.array(list(map(float, camera.findtext('pose').split()))[:3])
        assert offset[2] > settings['obstacle_height']
        assert float(camera.findtext('camera/clip/far')) > np.linalg.norm(offset)
    bridge = tmp_path / 'bridge.yaml'
    bridge_config([u['name'] for u in resolved['uavs']], 'mpc_small50', bridge)
    rows = yaml.safe_load(bridge.read_text())
    assert len(rows) == 13
    assert sum(r['ros_topic_name'] == '/clock' for r in rows) == 1
    assert rows[0]['gz_topic_name'] == '/world/mpc_small50/clock'


def test_rejects_mismatched_start_and_controller_altitudes(tmp_path):
    from solo_mpc.settings import load_settings

    pkg = ROOT / 'ros_ws/src/solo_mpc'
    scenario = yaml.safe_load((pkg / 'scenarios/small50_3uav.yaml').read_text())
    scenario['starts'][0][2] = 4
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(scenario))
    with pytest.raises(ValueError, match='starts altitude must match flight_altitude'):
        load_settings(path, pkg / 'config/controller.yaml')


def test_three_uav_lidar_stays_usable_at_commanded_acceleration():
    import numpy as np
    from scipy.spatial.transform import Rotation
    from solo_mpc.core.sensors import project_scan
    from solo_mpc.settings import load_settings

    pkg = ROOT / 'ros_ws/src/solo_mpc'
    _, settings, cfg = load_settings(pkg / 'scenarios/small50_3uav.yaml',
                                     pkg / 'config/controller.yaml')
    lean = np.arctan(cfg['UMAX'] / 9.81)
    position = np.array([5., 2., settings['flight_altitude'] + 0.08])
    scan = project_scan(np.full(settings['lidar_samples'], np.inf), -np.pi,
                        cfg['LIDAR_ANGULAR_RES'], cfg.get('LIDAR_RANGE_MIN', 0.15), cfg['SENSING_RADIUS'],
                        position, Rotation.from_euler('y', lean).as_quat(), 0.,
                        height_tolerance=settings['scan_height_tolerance'])
    assert len(scan.endpoints) == settings['lidar_samples']
    assert np.min(scan.endpoints[:, 2]) > 0
    assert np.max(scan.endpoints[:, 2]) < settings['obstacle_height']


@pytest.mark.parametrize('target_mode', ['', 'ground_truth', 'settings_ground_truth'])
@pytest.mark.parametrize('record_cameras', [False, True])
@pytest.mark.parametrize('record_overhead', [False, True])
def test_three_vehicle_launch_builds_shared_and_per_vehicle_processes_without_starting(
    tmp_path, upstream, monkeypatch, target_mode, record_cameras, record_overhead,
):
    import importlib.util
    import json
    from unittest.mock import MagicMock

    pytest.importorskip('launch')
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    import solo_bringup.launching
    import solo_mpc.world

    # Construct launch actions only. No LaunchService, Gazebo, SITL, or MAVROS runs.
    monkeypatch.setenv('SOLO_DEV', '0')
    monkeypatch.setenv('RUNS_ROOT', str(tmp_path / 'runs'))
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(ROOT / 'ros_ws/src' / name))
    monkeypatch.setattr(solo_bringup.launching, 'mavros',
                        lambda uav, run, **kw: ExecuteProcess(cmd=['test-mavros', uav['name']]))
    generate = solo_mpc.world.generate_mpc_world
    monkeypatch.setattr(solo_mpc.world, 'generate_mpc_world',
                        lambda fleet, source, *a, **kw: generate(fleet, upstream, *a, **kw))
    is_file = Path.is_file
    monkeypatch.setattr(Path, 'is_file', lambda p: (
        str(p).startswith('/opt/ardupilot/Tools/autotest/default_params/') or is_file(p)))
    # Port availability is independent of the action topology tested here.
    import socket
    monkeypatch.setattr(socket, 'socket', MagicMock())
    pkg = ROOT / 'ros_ws/src/solo_mpc'
    spec = importlib.util.spec_from_file_location('mpc_launch_test', pkg / 'launch/mpc.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    settings_path = pkg / 'config/controller.yaml'
    settings = yaml.safe_load(settings_path.read_text())
    settings['sitl_params'] = {'WPNAV_SPEED': 120, 'WPNAV_ACCEL': 200}
    if target_mode == 'settings_ground_truth':
        settings['target_mode'] = 'ground_truth'
    settings_path = tmp_path / 'controller.yaml'
    settings_path.write_text(yaml.safe_dump(settings, sort_keys=False))
    resolved_mode = 'ground_truth' if target_mode else 'camera'
    context = LaunchContext()
    context.launch_configurations.update({
        'fleet': str(ROOT / 'config/fleet.yaml'),
        'scenario': str(pkg / 'scenarios/small50_3uav.yaml'),
        'settings': str(settings_path),
        'resume': 'false', 'run_id': 'three-uav-offline', 'readiness_timeout': '600',
        'record_cameras': str(record_cameras).lower(),
        'record_overhead': str(record_overhead).lower(),
        'target_mode': target_mode if target_mode != 'settings_ground_truth' else '',
    })
    actions = module.start(context)
    run = context.solo_run_dir
    inventory = json.loads((run / 'startup.json').read_text())['processes']
    shared = {'gazebo-server', 'readiness-monitor', 'sensor-bridge', 'frames',
              'target', 'controller', 'setpoints', 'mission-supervisor'}
    if record_cameras:
        shared.add('camera-recorder')
    if record_overhead:
        shared.add('overhead-recorder')
    kinds = ['sitl', 'mavros'] + (['camera-detector'] if resolved_mode == 'camera' else [])
    individual = {f'{kind}-uav{i}' for kind in kinds
                  for i in range(1, 4)}
    assert set(inventory) == shared | individual
    assert all(item['state'] == 'pending' for item in inventory.values())
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 8 + 3 * len(kinds) + int(record_cameras) + int(record_overhead)
    from launch.utilities import perform_substitutions
    commands = [[perform_substitutions(context, part) for part in a.cmd]
                for a in processes if not isinstance(a, Node)]
    sitl_commands = [c for c in commands if c[0].endswith('/arducopter')]
    assert len(sitl_commands) == 3
    assert {c[c.index('-I') + 1] for c in sitl_commands} == {'0', '1', '2'}
    assert all(c[c.index('--defaults') + 1].endswith(','+str(run / 'sitl-overrides.parm'))
               for c in sitl_commands)
    assert len(list((run / 'sitl').iterdir())) == 3
    assert (run / 'sitl-overrides.parm').read_text() == 'WPNAV_SPEED 120\nWPNAV_ACCEL 200\n'
    assert yaml.safe_load((run / 'fleet.yaml').read_text())['world_name'] == 'mpc_small50'
    assert yaml.safe_load((run / 'controller.yaml').read_text())['target_mode'] == resolved_mode
    world = ET.parse(run / 'world.sdf').getroot().find('world')
    cameras_enabled = resolved_mode == 'camera' or record_cameras
    rows = yaml.safe_load((run / 'bridge.yaml').read_text())
    topics = {r['ros_topic_name'] for r in rows}
    for i in range(1, 4):
        model = world.find(f"model[@name='uav{i}']")
        assert len(model.findall(".//sensor[@type='camera']")) == int(cameras_enabled)
        assert model.find(".//sensor[@type='gpu_lidar']") is not None
        assert (f'/uav{i}/camera/image_raw' in topics) == cameras_enabled
        assert (f'/uav{i}/camera/camera_info' in topics) == cameras_enabled
        assert f'/uav{i}/scan' in topics
        assert f'/uav{i}/ground_truth/pose' in topics
        # Frame alignment and evaluation need live UAV poses even without FPV.
        publishers = model.findall("plugin[@name='gz::sim::systems::PosePublisher']")
        assert len(publishers) == 1
        publisher = publishers[0]
        assert publisher.findtext('topic') == f'/uav{i}/ground_truth/pose'
        assert publisher.findtext('publish_model_pose') == 'true'
        assert publisher.findtext('use_pose_vector_msg') == 'false'
        assert float(publisher.findtext('update_frequency')) > 0
    assert len(world.findall("model[@name='tracking_target']//sensor")) == int(record_overhead)
