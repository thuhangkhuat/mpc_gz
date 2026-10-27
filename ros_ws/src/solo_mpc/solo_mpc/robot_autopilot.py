"""Autonomous ground robot driving: A* on the known map, then pure pursuit.

The robot knows the scenario obstacles and live obstacles (it does not sense
anything), inflated by its body half-diagonal plus a clearance. No ROS here.
"""

import heapq
import math

import numpy as np

from .scene_edit import clearance


class OccupancyMap:
    def __init__(self, bounds, shapes, inflate, resolution=0.2):
        self.x0, self.y0, x1, y1 = bounds
        self.resolution = resolution
        self.inflate = inflate
        self.nx = int(math.ceil((x1 - self.x0) / resolution))
        self.ny = int(math.ceil((y1 - self.y0) / resolution))
        xs = self.x0 + (np.arange(self.nx) + 0.5) * resolution
        ys = self.y0 + (np.arange(self.ny) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)  # [iy, ix]
        blocked = ((gx - self.x0 < inflate) | (x1 - gx < inflate)
                   | (gy - self.y0 < inflate) | (y1 - gy < inflate))
        for shape, x, y, size in shapes:
            if shape == "circle":
                blocked |= np.hypot(gx - x, gy - y) - size[0] < inflate
            else:
                dx = np.maximum(np.abs(gx - x) - size[0] / 2, 0)
                dy = np.maximum(np.abs(gy - y) - size[1] / 2, 0)
                blocked |= np.hypot(dx, dy) < inflate
        self.blocked = blocked
        self.shapes = shapes

    def cell(self, p):
        return (min(max(int((p[1] - self.y0) / self.resolution), 0), self.ny - 1),
                min(max(int((p[0] - self.x0) / self.resolution), 0), self.nx - 1))

    def centre(self, cell):
        iy, ix = cell
        return np.array([self.x0 + (ix + 0.5) * self.resolution, self.y0 + (iy + 0.5) * self.resolution])

    def free(self, p):
        return not self.blocked[self.cell(p)]

    def segment_free(self, a, b):
        steps = max(1, int(np.linalg.norm(np.subtract(b, a)) / (self.resolution / 2)))
        return all(self.free(np.add(a, np.subtract(b, a) * k / steps)) for k in range(steps + 1))

    def nearest_free(self, p):
        """Closest free cell centre, so a robot pushed into the margin can leave it."""
        if self.free(p):
            return np.asarray(p, float)
        free = np.argwhere(~self.blocked)
        if not len(free):
            return None
        target = np.array(self.cell(p))
        return self.centre(tuple(free[np.argmin(np.sum((free - target) ** 2, axis=1))]))


def plan(grid, start, goal):
    """Shortest 8-connected path as world points (start first), shortcut by line of sight."""
    begin, end = grid.nearest_free(start), grid.nearest_free(goal)
    if begin is None or end is None:
        return None
    s, g = grid.cell(begin), grid.cell(end)
    octile = lambda a: math.hypot(a[0] - g[0], a[1] - g[1])
    cost, parent, frontier = {s: 0.0}, {s: None}, [(octile(s), s)]
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == g:
            break
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nxt = (current[0] + dy, current[1] + dx)
                if (dx == dy == 0 or not (0 <= nxt[0] < grid.ny and 0 <= nxt[1] < grid.nx)
                        or grid.blocked[nxt]):
                    continue
                # No diagonal corner cutting between two blocked cells.
                if dx and dy and (grid.blocked[current[0] + dy, current[1]]
                                  or grid.blocked[current[0], current[1] + dx]):
                    continue
                step = cost[current] + (math.sqrt(2) if dx and dy else 1.0)
                if step < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = step, current
                    heapq.heappush(frontier, (step + octile(nxt), nxt))
    if g not in parent:
        return None
    cells, node = [], g
    while node is not None:
        cells.append(node)
        node = parent[node]
    points = [np.asarray(start, float)] + [grid.centre(c) for c in reversed(cells[:-1])]
    points[-1] = np.asarray(end, float) if len(points) > 1 else points[-1]
    shortcut, i = [points[0]], 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not grid.segment_free(points[i], points[j]):
            j -= 1
        shortcut.append(points[j])
        i = j
    return shortcut


def random_goal(grid, rng, start, min_distance, attempts=500):
    free = np.argwhere(~grid.blocked)
    for _ in range(attempts):
        candidate = grid.centre(tuple(free[rng.integers(len(free))]))
        if np.linalg.norm(candidate - start) >= min_distance:
            return candidate
    return None


def pursuit(pose, path, speed, lookahead, max_angular, goal_tolerance):
    """(linear, angular, done) toward a lookahead point on path for pose (x, y, yaw).

    The lookahead point never passes the end of the segment being followed, so
    the robot reaches each corner and turns there instead of cutting inside it,
    where the obstacle that caused the corner usually is.
    """
    x, y, yaw = pose
    position = np.array([x, y])
    if np.linalg.norm(np.asarray(path[-1]) - position) < goal_tolerance:
        return 0.0, 0.0, True
    segments = []
    for i, (a, b) in enumerate(zip(path, path[1:])):
        a, b = np.asarray(a, float), np.asarray(b, float)
        length = max(np.linalg.norm(b - a), 1e-9)
        t = float(np.clip((position - a) @ (b - a) / length ** 2, 0.0, 1.0))
        segments.append((np.linalg.norm(a + t * (b - a) - position), i, t, a, b, length))
    closest = min(d for d, *_ in segments)
    # Among near-equal distances (at a corner) prefer the later segment.
    _, _, t, a, b, length = max((s for s in segments if s[0] <= closest + 0.05), key=lambda s: s[1])
    target = a + (b - a) * min(1.0, t + lookahead / length)
    dx, dy = target - position
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    alpha = math.atan2(local_y, local_x)
    if abs(alpha) > math.radians(60):
        # Turn in place toward the path before driving.
        return 0.0, math.copysign(0.8 * max_angular, alpha), False
    distance = max(math.hypot(dx, dy), 1e-6)
    linear = speed * max(0.3, math.cos(alpha)) * min(1.0, np.linalg.norm(np.asarray(path[-1]) - position))
    angular = float(np.clip(2 * linear * math.sin(alpha) / distance, -max_angular, max_angular))
    return float(linear), angular, False


def nearest_surface(shapes, point):
    return min((clearance(s, x, y, size, point) for s, x, y, size in shapes), default=math.inf)
