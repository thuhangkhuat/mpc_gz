"""Generate a fleet world from the pinned official Iris models."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


def generate_world(config, source, destination, profile='control'):
    if profile not in ('reference', 'control'):
        raise ValueError('profile must be reference or control')
    source, destination = Path(source), Path(destination)
    tree = ET.parse(source / 'worlds/iris_runway.sdf')
    world = tree.getroot().find('world')
    template = ET.parse(source / 'models/iris_with_gimbal/model.sdf').getroot().find('model')
    if world is None or template is None:
        raise ValueError('Official Iris world/model missing')
    originals = [i for i in world.findall('include')
                 if i.findtext('uri') == 'model://iris_with_gimbal']
    if len(originals) != 1:
        raise ValueError('Expected exactly one Iris include in official world')
    world.remove(originals[0])
    world.set('name', config['world_name'])
    if profile == 'control':
        # No rendering sensors in this profile; keep IMU/NavSat systems.
        for plugin in list(world.findall('plugin')):
            if plugin.get('name') == 'gz::sim::systems::Sensors':
                world.remove(plugin)
    for uav in config['uavs']:
        model = copy.deepcopy(template)
        model.set('name', uav['name'])
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.set('degrees', 'true')
        pose.text = ' '.join(map(str, uav['spawn']))
        plugin = model.find("plugin[@name='ArduPilotPlugin']")
        if plugin is None or plugin.find('fdm_port_in') is None:
            raise ValueError('ArduPilot plugin/JSON port missing')
        plugin.find('fdm_port_in').text = str(uav['json_port'])
        if profile == 'control':
            for include in list(model.findall('include')):
                if include.findtext('name') == 'gimbal':
                    model.remove(include)
            for joint in list(model.findall('joint')):
                if joint.get('name') == 'gimbal_joint':
                    model.remove(joint)
            for control in list(plugin.findall('control')):
                if int(control.get('channel')) >= 8:
                    plugin.remove(control)
            for system in list(model.findall('plugin')):
                if system.get('name') == 'gz::sim::systems::JointPositionController':
                    model.remove(system)
        else:
            # Keep original physics/sensors, but isolate gimbal topics/video ports.
            for topic in model.iter('cmd_topic'):
                topic.text = f"/{uav['name']}{topic.text}"
            for system in model.findall('plugin'):
                topic = system.find('topic')
                if topic is not None and topic.text.startswith('/gimbal/'):
                    topic.text = f"/{uav['name']}{topic.text}"
            gimbal = next(i for i in model.findall('include') if i.findtext('name') == 'gimbal')
            nested = ET.parse(source / 'models/gimbal_small_3d/model.sdf').getroot().find('model')
            nested.set('name', 'gimbal')
            nested.insert(0, copy.deepcopy(gimbal.find('pose')))
            for port in nested.iter('udp_port'):
                port.text = str(5600 + 10 * uav['instance'])
            for camera_plugin in nested.iter('plugin'):
                if camera_plugin.get('name') == 'CameraZoomPlugin':
                    topic = camera_plugin.find('topic')
                    if topic is None:
                        topic = ET.SubElement(camera_plugin, 'topic')
                    topic.text = f"/{uav['name']}/camera/cmd_zoom"
            model.remove(gimbal)
            model.append(nested)
        world.append(model)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return destination
