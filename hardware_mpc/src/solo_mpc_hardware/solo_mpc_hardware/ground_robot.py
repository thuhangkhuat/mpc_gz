"""Keyboard-driven ground robot carrying an AprilTag, the UAV's tracking target.

The robot is a box moved by Gazebo's VelocityControl system from
/ground_robot/cmd_vel (geometry_msgs/Twist, bridged), so there is no wheel
slip to tune. A PosePublisher reports its exact pose for supervision and
evaluation; in camera mode the controller sees only AprilTag detections.
"""

import math
from pathlib import Path

import yaml

from .apriltag import CELLS, FAMILIES

MODEL = "ground_robot"
CMD_TOPIC = "/ground_robot/cmd_vel"
POSE_TOPIC = "/ground_robot/pose"
QUIET_CELLS = 1  # white margin printed around the black tag square


def _positive(data, keys, where):
    for key in keys:
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                or value <= 0:
            raise ValueError(f"ground_robot {where}.{key} must be a positive number")


def load(value, bounds=None):
    """Validated robot configuration from a YAML path or an already loaded mapping."""
    data = yaml.safe_load(Path(value).read_text()) if isinstance(value, (str, Path)) else value
    if not isinstance(data, dict):
        raise ValueError("ground_robot must be a mapping or a YAML file path")
    spawn = data.get("spawn")
    if not isinstance(spawn, list) or len(spawn) != 3 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in spawn):
        raise ValueError("ground_robot spawn must be [x, y, yaw_degrees]")
    if bounds is not None:
        x0, y0, x1, y1 = bounds
        if not (x0 <= spawn[0] <= x1 and y0 <= spawn[1] <= y1):
            raise ValueError("ground_robot spawn must lie inside the scenario xlim/ylim")
    body, tag, teleop = data.get("body", {}), data.get("tag", {}), data.get("teleop", {})
    _positive(body, ("length", "width", "height", "mass"), "body")
    if tag.get("family") not in FAMILIES:
        raise ValueError(f"ground_robot tag.family must be one of {sorted(FAMILIES)}")
    if isinstance(tag.get("id"), bool) or not isinstance(tag.get("id"), int) or not 0 <= tag["id"] <= 586:
        raise ValueError("ground_robot tag.id must be an integer in [0, 586] for tag36h11")
    _positive(tag, ("size",), "tag")
    if tag["size"] * (CELLS + 2 * QUIET_CELLS) / CELLS > min(body["length"], body["width"]):
        raise ValueError("ground_robot tag (with its white margin) must fit on the body top")
    _positive(teleop, ("max_linear", "max_angular", "linear_step", "angular_step",
                       "linear_accel", "angular_accel", "rate"), "teleop")
    auto = data.get("auto")
    if auto is not None:
        if auto.get("mode") not in ("random", "patrol"):
            raise ValueError("ground_robot auto.mode must be random or patrol")
        _positive(auto, ("speed", "lookahead", "clearance", "goal_tolerance", "min_leg"), "auto")
        points = auto.get("waypoints", [])
        if auto["mode"] == "patrol" and (not isinstance(points, list) or len(points) < 2 or not all(
                isinstance(p, list) and len(p) == 2 and all(isinstance(v, (int, float)) for v in p)
                for p in points)):
            raise ValueError("ground_robot auto.waypoints needs at least two [x, y] points for patrol")
        if bounds is not None and any(not (bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3])
                                      for x, y in (points or [])):
            raise ValueError("ground_robot auto.waypoints must lie inside the scenario xlim/ylim")
        seed = auto.get("seed")
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int)):
            raise ValueError("ground_robot auto.seed must be an integer or null")
    return data


def tag_height(robot):
    """Height of the tag plane above the ground."""
    return robot["body"]["height"] + 0.002


def write_tag_texture(robot, directory):
    import cv2

    from .apriltag import tag_image

    tag = robot["tag"]
    path = Path(directory) / f"apriltag_{tag['family']}_{tag['id']}.png"
    cv2.imwrite(str(path), tag_image(tag["family"], tag["id"], quiet_cells=QUIET_CELLS))
    return path


def add_model(world, robot, texture, element):
    """Append the robot model to an SDF <world> element."""
    body, tag = robot["body"], robot["tag"]
    x, y, yaw = robot["spawn"]
    L, W, H, mass = body["length"], body["width"], body["height"], body["mass"]
    model = element(world, "model", name=MODEL)
    element(model, "pose", f"{x} {y} 0 0 0 {math.radians(yaw)}")
    link = element(model, "link", name="body")
    element(link, "pose", f"0 0 {H / 2} 0 0 0")
    inertial = element(link, "inertial")
    element(inertial, "mass", mass)
    inertia = element(inertial, "inertia")
    for axis, value in (("ixx", W * W + H * H), ("iyy", L * L + H * H), ("izz", L * L + W * W)):
        element(inertia, axis, mass * value / 12)
    collision = element(link, "collision", name="collision")
    element(element(element(collision, "geometry"), "box"), "size", f"{L} {W} {H}")
    # Commanded velocity, not wheel contact, moves the box: no ground friction.
    friction = element(element(element(collision, "surface"), "friction"), "ode")
    element(friction, "mu", 0.0)
    element(friction, "mu2", 0.0)
    for name, size, pose, color in (
        ("chassis", f"{L} {W} {H}", "0 0 0 0 0 0", "0.2 0.3 0.55 1"),
        # Short bar at the front so the heading is visible from above.
        ("heading", f"{0.06} {W * 0.8} 0.01", f"{L / 2 - 0.03} 0 {H / 2 + 0.005} 0 0 0", "0.9 0.2 0.1 1"),
    ):
        visual = element(link, "visual", name=name)
        element(visual, "pose", pose)
        element(element(element(visual, "geometry"), "box"), "size", size)
        material = element(visual, "material")
        element(material, "ambient", color)
        element(material, "diffuse", color)
    edge = tag["size"] * (CELLS + 2 * QUIET_CELLS) / CELLS
    visual = element(link, "visual", name="apriltag")
    element(visual, "pose", f"0 0 {H / 2 + 0.001} 0 0 0")
    element(element(element(visual, "geometry"), "box"), "size", f"{edge} {edge} 0.002")
    material = element(visual, "material")
    for tag_name in ("ambient", "diffuse"):
        element(material, tag_name, "1 1 1 1")
    metal = element(element(material, "pbr"), "metal")
    element(metal, "albedo_map", str(Path(texture).resolve()))
    element(metal, "metalness", 0.0)
    element(metal, "roughness", 1.0)
    control = element(model, "plugin", filename="gz-sim-velocity-control-system",
                      name="gz::sim::systems::VelocityControl")
    element(control, "topic", CMD_TOPIC)
    publisher = element(model, "plugin", filename="gz-sim-pose-publisher-system",
                        name="gz::sim::systems::PosePublisher")
    for key, value in (("publish_model_pose", "true"), ("publish_link_pose", "false"),
                       ("publish_nested_model_pose", "false"), ("use_pose_vector_msg", "false"),
                       ("update_frequency", "20"), ("topic", POSE_TOPIC)):
        element(publisher, key, value)
    return model
