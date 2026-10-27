"""Ground robot config, keyboard teleop limits, and AprilTag detection."""

import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

from solo_mpc import ground_robot

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT / "config/ground_robot.yaml").read_text())
spec = importlib.util.spec_from_file_location("drive", ROOT / "scripts/drive.py")
drive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(drive)


def test_shipped_config_is_valid_and_starts_in_view_of_the_uav():
    import math

    from solo_mpc.settings import load_settings

    scenario, settings, _ = load_settings(ROOT / "ros_ws/src/solo_mpc/scenarios/square50_1uav.yaml",
                                          ROOT / "config/drone.yaml")
    robot = settings["ground_robot"]
    footprint = settings["flight_altitude"] * math.tan(settings["camera_hfov"] / 2)
    offset = np.abs(np.subtract(robot["spawn"][:2], scenario["starts"][0][:2]))
    assert np.all(offset < footprint)
    assert robot["teleop"]["max_linear"] < settings["params"]["VMAX"]


@pytest.mark.parametrize("path,value", [
    (("spawn",), [1, 2]), (("spawn",), [100, 0, 0]), (("body", "height"), 0),
    (("tag", "family"), "tag25h9"), (("tag", "id"), 600), (("tag", "size"), 0.5),
    (("teleop", "max_linear"), -1), (("teleop", "rate"), True),
])
def test_invalid_robot_config_is_rejected(path, value):
    data = copy.deepcopy(CONFIG)
    target = data
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ValueError, match="ground_robot"):
        ground_robot.load(data, bounds=(0, 0, 50, 50))


def test_teleop_steps_clip_and_ramp_to_configured_limits():
    teleop = drive.Teleop.from_config(CONFIG)
    limits = CONFIG["teleop"]
    for _ in range(100):
        teleop.key("w")
        teleop.key("a")
    assert teleop.target_linear == pytest.approx(limits["max_linear"])
    assert teleop.target_angular == pytest.approx(limits["max_angular"])
    linear, angular = teleop.step(0.1)
    assert linear == pytest.approx(limits["linear_accel"] * 0.1)
    assert angular == pytest.approx(limits["angular_accel"] * 0.1)
    for _ in range(100):
        teleop.step(0.1)
    assert (teleop.linear, teleop.angular) == pytest.approx((limits["max_linear"], limits["max_angular"]))
    teleop.key(" ")
    assert (teleop.target_linear, teleop.target_angular) == (0.0, 0.0)
    assert teleop.key("q") is False


@pytest.mark.parametrize("mirrored", [False, True])
def test_rendered_tag_is_detected_with_its_centre(mirrored):
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV without aruco")
    from solo_mpc import apriltag

    tag = apriltag.tag_image("tag36h11", CONFIG["tag"]["id"], cell_px=6)
    frame = np.full((480, 640), 128, np.uint8)
    top, left = 150, 300
    frame[top:top + tag.shape[0], left:left + tag.shape[1]] = tag[:, ::-1] if mirrored else tag
    centre, corners = apriltag.detect(frame, "tag36h11", CONFIG["tag"]["id"])
    expected = (left + (tag.shape[1] - 1) / 2, top + (tag.shape[0] - 1) / 2)
    assert centre == pytest.approx(expected, abs=1.5)
    assert apriltag.detect(frame, "tag36h11", CONFIG["tag"]["id"] + 1) is None
