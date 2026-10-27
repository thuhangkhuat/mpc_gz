"""Camera field-of-view geometry: where a world point lands in a UAV image.

Inverse of core.sensors.pixel_to_ground for the generated downward camera
(mount 0.04 m below the body origin; optical right = body -Y, optical down =
body -X, optical forward = body -Z). Gazebo uses square pixels with the
principal point at the image centre.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

CAMERA_OFFSET = np.array([0.0, 0.0, -0.04])


def intrinsics(width, height, hfov):
    focal = width / (2 * math.tan(hfov / 2))
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1.0]])


def project(point, position, quaternion, K):
    """(u, v, depth) of a world point, or None when it is behind the camera."""
    rotation = Rotation.from_quat(quaternion)
    camera = np.asarray(position, float) + rotation.apply(CAMERA_OFFSET)
    d = rotation.inv().apply(np.asarray(point, float) - camera)
    ray = np.array([-d[1], -d[0], -d[2]])
    if ray[2] <= 1e-6:
        return None
    return K[0, 0] * ray[0] / ray[2] + K[0, 2], K[1, 1] * ray[1] / ray[2] + K[1, 2], ray[2]


def view(point, radius, position, quaternion, K, width, height):
    """FOV state of a disc of `radius` metres centred on point.

    Returns dict(centre_in, fully_in, offset): offset is 0 at the image centre
    and 1 at the nearest image edge (Chebyshev, normalised per axis).
    """
    projected = project(point, position, quaternion, K)
    if projected is None:
        return dict(centre_in=False, fully_in=False, offset=None)
    u, v, depth = projected
    r = K[0, 0] * radius / depth
    centre_in = 0 <= u < width and 0 <= v < height
    fully_in = r <= u < width - r and r <= v < height - r
    offset = max(abs(u - K[0, 2]) / K[0, 2], abs(v - K[1, 2]) / K[1, 2])
    return dict(centre_in=centre_in, fully_in=fully_in, offset=offset if centre_in else None)


def level(yaw):
    """Quaternion (x, y, z, w) for a level vehicle: old truth logs record yaw only."""
    return Rotation.from_euler("z", yaw).as_quat()


def gaps(stamps, flags):
    """Longest run of consecutive False flags, in seconds of stamps."""
    longest, start = 0.0, None
    for t, flag in zip(list(stamps) + [None], list(flags) + [True]):
        if not flag and start is None:
            start = t
        elif flag and start is not None:
            longest = max(longest, (t if t is not None else stamps[-1]) - start)
            start = None
    return longest


def team_coverage(stamps, per_vehicle):
    """Team metrics from per-vehicle fully-in-view flags sampled at stamps."""
    counts = np.sum(np.asarray(per_vehicle, bool), axis=0) if per_vehicle else np.zeros(len(stamps))
    seen = counts > 0
    return dict(any_in_view_fraction=float(np.mean(seen)) if len(seen) else None,
                mean_viewers=float(np.mean(counts)) if len(counts) else None,
                longest_unseen_s=gaps(list(stamps), list(seen)))


def footprint(position, quaternion, K, width, height, plane_z):
    """Counter-clockwise ground quadrilateral seen by the camera on z = plane_z, or None.

    None when an image corner ray misses the plane (vehicle tilted past the
    half field of view); a partial footprint would not be comparable.
    """
    from .core.sensors import pixel_to_ground

    rotation = Rotation.from_quat(quaternion)
    camera = np.asarray(position, float) + rotation.apply(CAMERA_OFFSET)
    try:
        corners = np.array([pixel_to_ground(p, K, camera, quaternion, plane_z)[:2]
                            for p in ((0, 0), (width, 0), (width, height), (0, height))])
    except ValueError:
        return None
    return corners if area(corners) > 0 else corners[::-1]


def area(polygon):
    """Signed shoelace area; positive for counter-clockwise vertices."""
    if polygon is None or len(polygon) < 3:
        return 0.0
    x, y = np.asarray(polygon, float).T
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def intersection(subject, clip):
    """Sutherland-Hodgman intersection of two counter-clockwise convex polygons."""
    output = [np.asarray(p, float) for p in subject]
    for a, b in zip(clip, np.roll(clip, -1, axis=0)):
        edge = b - a
        inside = lambda p: edge[0] * (p[1] - a[1]) - edge[1] * (p[0] - a[0]) >= 0
        points, output = output, []
        for p, q in zip(points, points[1:] + points[:1]):
            if inside(q):
                if not inside(p):
                    output.append(_cross(p, q, a, b))
                output.append(q)
            elif inside(p):
                output.append(_cross(p, q, a, b))
        if not output:
            return None
    return np.array(output)


def _cross(p, q, a, b):
    """Intersection of segment p-q with the line through a-b."""
    d, e = q - p, b - a
    t = (e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])) / (d[0] * e[1] - d[1] * e[0])
    return p + t * d


def union_area(polygons):
    """Area covered by any polygon, by inclusion-exclusion over up to four of them."""
    from itertools import combinations

    total = 0.0
    for k in range(1, len(polygons) + 1):
        for group in combinations(polygons, k):
            shared = group[0]
            for polygon in group[1:]:
                shared = None if shared is None else intersection(shared, polygon)
            total += (-1) ** (k + 1) * (area(shared) if shared is not None else 0.0)
    return total
