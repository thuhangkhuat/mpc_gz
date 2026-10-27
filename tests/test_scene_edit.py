"""Live obstacle SDF, naming and clearance without a running Gazebo."""

import xml.etree.ElementTree as ET

import pytest

from solo_mpc import scene_edit


@pytest.mark.parametrize("shape,size,geometry,expected", [
    ("circle", [0.3], "cylinder", {"radius": "0.3", "length": "5.0"}),
    ("box", [1.0, 0.4], "box", {"size": "1.0 0.4 5.0"}),
])
def test_obstacle_sdf_is_a_static_extruded_solid(shape, size, geometry, expected):
    model = ET.fromstring(scene_edit.obstacle_sdf("live_obstacle_0", shape, 2.0, -3.0, size, 5.0)).find("model")
    assert model.get("name") == "live_obstacle_0"
    assert model.findtext("static") == "true"
    assert model.findtext("pose").split()[:3] == ["2.0", "-3.0", "2.5"]
    for kind in ("visual", "collision"):
        for tag, value in expected.items():
            assert model.findtext(f"link/{kind}/geometry/{geometry}/{tag}") == value


@pytest.mark.parametrize("shape,size", [("circle", [0]), ("circle", [1, 2]), ("box", [1]),
                                        ("box", [1, -1]), ("cone", [1]), ("circle", [float("nan")])])
def test_obstacle_sdf_rejects_invalid_sizes(shape, size):
    with pytest.raises(ValueError):
        scene_edit.obstacle_sdf("live_obstacle_0", shape, 0, 0, size, 5.0)


def test_registry_reuses_the_lowest_free_name(tmp_path):
    assert scene_edit.next_name(scene_edit.load(tmp_path)) == "live_obstacle_0"
    scene_edit.save(tmp_path, {"live_obstacle_0": {}, "live_obstacle_2": {}})
    assert scene_edit.next_name(scene_edit.load(tmp_path)) == "live_obstacle_1"


def test_clearance_to_circle_and_box_footprints():
    assert scene_edit.clearance("circle", 0, 0, [0.5], (3, 4)) == pytest.approx(4.5)
    assert scene_edit.clearance("box", 0, 0, [2, 4], (4, 0)) == pytest.approx(3)
    assert scene_edit.clearance("box", 0, 0, [2, 4], (4, 6)) == pytest.approx(5)
    assert scene_edit.clearance("box", 0, 0, [2, 4], (0.5, 1)) == 0


def test_only_live_obstacles_can_be_removed(tmp_path):
    with pytest.raises(ValueError, match="live obstacles"):
        scene_edit.remove(None, "w", tmp_path, "obstacle_circle_0")
