"""Sample a target route with one positive speed per waypoint leg."""

import numpy as np

from .geometry import catmull_rom


def target_route(scenario, cfg):
    target = scenario["target"]
    points = np.asarray(target["waypoints"], float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2
            or not np.isfinite(points).all()
            or np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) <= 0)):
        raise ValueError("target.waypoints must contain distinct consecutive finite XY points")
    base_speed = float(scenario["tar_max_speed"])
    if not np.isfinite(base_speed) or base_speed <= 0:
        raise ValueError("tar_max_speed must be finite and positive")
    speeds = np.asarray(target.get("speeds", [base_speed] * (len(points) - 1)), float)
    if (speeds.shape != (len(points) - 1,) or not np.isfinite(speeds).all()
            or np.any(speeds <= 0)):
        raise ValueError("target.speeds needs one finite positive speed per waypoint leg")
    ds = float(cfg.get("TAR_SPLINE_DS", 0.1))
    if not np.isfinite(ds) or ds <= 0:
        raise ValueError("TAR_SPLINE_DS must be finite and positive")
    if cfg.get("TAR_SMOOTH_ENABLE", True):
        path, knots = catmull_rom(points, ds)
    else:
        path, knots = points.copy(), np.arange(len(points))
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    durations = lengths / np.repeat(speeds, np.diff(knots))
    return path, np.r_[0.0, np.cumsum(durations)]
