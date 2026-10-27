"""Live scene edits in a running Gazebo world: spawn and remove obstacles.

Spawned obstacles are extruded to the run's obstacle_height like scenario
obstacles, so the horizontal LiDAR observes them; the controller learns them
only through the LiDAR. They are recorded in <run>/live-obstacles.yaml and are
not part of scenario.yaml or the evaluation's obstacle list.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .world import solid

PREFIX = "live_obstacle_"
REGISTRY = "live-obstacles.yaml"
COLOR = "0.85 0.35 0.1 1"


def obstacle_sdf(name, shape, x, y, size, height):
    """SDF for a static box (size = [width, depth], centred) or cylinder (size = [radius])."""
    values = [x, y, *size, height]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError("obstacle position and size must be finite numbers")
    if shape == "box" and len(size) == 2 and min(size) > 0:
        geometry = [size[0], size[1], height]
    elif shape == "circle" and len(size) == 1 and size[0] > 0:
        geometry, shape = [size[0], height], "cylinder"
    else:
        raise ValueError("use circle RADIUS or box WIDTH DEPTH with positive sizes")
    root = ET.Element("sdf", version="1.9")
    solid(root, name, [x, y, height / 2], shape, geometry, COLOR)
    return ET.tostring(root, encoding="unicode")


def load(run_dir):
    path = Path(run_dir) / REGISTRY
    try:
        return yaml.safe_load(path.read_text()) or {}
    except FileNotFoundError:
        return {}


def save(run_dir, obstacles):
    Path(run_dir, REGISTRY).write_text(yaml.safe_dump(obstacles, sort_keys=True))


def next_name(obstacles):
    used = {int(n[len(PREFIX):]) for n in obstacles if n[len(PREFIX):].isdigit()}
    return f"{PREFIX}{next(i for i in range(len(used) + 1) if i not in used)}"


def spawn(gz, world, run_dir, shape, x, y, size, height):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory

    obstacles = load(run_dir)
    name = next_name(obstacles)
    request = EntityFactory()
    request.sdf = obstacle_sdf(name, shape, x, y, list(size), height)
    request.allow_renaming = False
    ok, reply = gz.request(f"/world/{world}/create", request, EntityFactory, Boolean, 3000)
    if not (ok and reply.data):
        raise RuntimeError(f"Gazebo did not create {name}")
    obstacles[name] = {"shape": shape, "x": float(x), "y": float(y),
                       "size": [float(v) for v in size]}
    save(run_dir, obstacles)
    return name


def remove(gz, world, run_dir, name):
    if not name.startswith(PREFIX):
        raise ValueError(f"only live obstacles ({PREFIX}N) can be removed")
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_pb2 import Entity

    request = Entity()
    request.name = name
    request.type = Entity.MODEL
    ok, reply = gz.request(f"/world/{world}/remove", request, Entity, Boolean, 3000)
    if not (ok and reply.data):
        raise RuntimeError(f"Gazebo did not remove {name}")
    obstacles = load(run_dir)
    obstacles.pop(name, None)
    save(run_dir, obstacles)


def clearance(shape, x, y, size, point):
    """Distance from a point to the obstacle footprint (negative inside a circle)."""
    px, py = point[0] - x, point[1] - y
    if shape == "circle":
        return math.hypot(px, py) - size[0]
    dx, dy = max(abs(px) - size[0] / 2, 0.0), max(abs(py) - size[1] / 2, 0.0)
    return math.hypot(dx, dy)


def scenario_entries(obstacles):
    """Scenario rows for unsaved live obstacles: rects [left, bottom, w, h], circles [x, y, r]."""
    rects, circles = [], []
    for name in sorted(obstacles):
        data = obstacles[name]
        if data.get("saved"):
            continue
        x, y, size = data["x"], data["y"], data["size"]
        if data["shape"] == "circle":
            circles.append([round(x, 2), round(y, 2), round(size[0], 2)])
        else:
            rects.append([round(x - size[0] / 2, 2), round(y - size[1] / 2, 2),
                          round(size[0], 2), round(size[1], 2)])
    return rects, circles


def append_to_scenario(text, rects, circles):
    """Insert rows after the last item of obstacles.rects/circles, keeping comments."""
    lines = text.splitlines(keepends=True)
    top = next((i for i, line in enumerate(lines) if line.rstrip() == "obstacles:"), None)
    if top is None:
        raise ValueError("scenario has no top-level obstacles: block")
    end = next((i for i in range(top + 1, len(lines))
                if lines[i].strip() and not lines[i].startswith((" ", "#"))), len(lines))
    # Later list first, so the earlier insertion index stays valid.
    keys = sorted(((k, rows) for k, rows in (("rects", rects), ("circles", circles)) if rows),
                  key=lambda item: -next((i for i in range(top, end)
                                          if lines[i].lstrip().startswith(f"{item[0]}:")), end))
    for key, rows in keys:
        head = next((i for i in range(top + 1, end) if lines[i].lstrip().startswith(f"{key}:")), None)
        if head is None:
            raise ValueError(f"scenario obstacles has no {key}: list")
        indent = len(lines[head]) - len(lines[head].lstrip())
        item = " " * (indent + 2)
        new = [f"{item}- [{', '.join(f'{v:g}' for v in row)}]  # live\n" for row in rows]
        if lines[head].split(":", 1)[1].split("#")[0].strip() == "[]":
            lines[head] = f"{' ' * indent}{key}:\n"
            last = head
        else:
            last = head
            for i in range(head + 1, end):
                if lines[i].startswith(item + "-"):
                    last = i
                elif lines[i].strip() and not lines[i].lstrip().startswith("#"):
                    break
        lines[last + 1:last + 1] = new
    return "".join(lines)
