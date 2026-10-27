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


def test_scenario_entries_convert_centres_and_skip_saved():
    obstacles = {
        "live_obstacle_0": {"shape": "box", "x": 8.0, "y": -2.0, "size": [1.6, 0.4]},
        "live_obstacle_1": {"shape": "circle", "x": 5.0, "y": 5.0, "size": [0.3]},
        "live_obstacle_2": {"shape": "circle", "x": 1.0, "y": 1.0, "size": [0.3], "saved": True},
    }
    assert scene_edit.scenario_entries(obstacles) == ([[7.2, -2.2, 1.6, 0.4]], [[5.0, 5.0, 0.3]])


def test_append_keeps_comments_and_validates(tmp_path):
    from pathlib import Path

    import yaml

    from solo_mpc.settings import load_settings

    pkg = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"
    original = (pkg / "scenarios/square50_1uav.yaml").read_text()
    text = scene_edit.append_to_scenario(original, [[20.0, 40.0, 1.0, 1.0]], [[40.5, 20.5, 0.15]])
    before, after = yaml.safe_load(original), yaml.safe_load(text)
    assert after["obstacles"]["rects"] == before["obstacles"]["rects"] + [[20, 40, 1, 1]]
    assert after["obstacles"]["circles"] == before["obstacles"]["circles"] + [[40.5, 20.5, 0.15]]
    assert {k: v for k, v in after.items() if k != "obstacles"} == {
        k: v for k, v in before.items() if k != "obstacles"}
    kept = [line for line in original.splitlines() if line.lstrip().startswith("#")]
    assert all(line in text.splitlines() for line in kept)
    path = tmp_path / "scenario.yaml"
    path.write_text(text)
    load_settings(path, pkg / "config/controller_solo.yaml")


def test_append_fills_empty_lists():
    text = "obstacles:\n  rects: []  # none\n  circles: []\nparams: {}\n"
    out = yaml_load(scene_edit.append_to_scenario(text, [[0, 0, 1, 1]], [[2, 2, 0.5]]))
    assert out == {"obstacles": {"rects": [[0, 0, 1, 1]], "circles": [[2, 2, 0.5]]}, "params": {}}


def yaml_load(text):
    import yaml
    return yaml.safe_load(text)
