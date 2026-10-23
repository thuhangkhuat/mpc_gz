"""Sample a solved trajectory at its original simulation epoch."""

import numpy as np


def sample_trajectory(points, elapsed):
    """Interpolate state, hold interval control; never extrapolate past a plan."""
    times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                      for p in points])
    if (len(times) < 2 or not np.isfinite(times).all() or times[0] != 0
            or np.any(np.diff(times) <= 0) or not 0 <= elapsed <= times[-1]):
        raise ValueError('Invalid trajectory times or sample outside plan')
    k = min(int(np.searchsorted(times, elapsed, side='right')) - 1, len(times) - 2)
    alpha = (elapsed - times[k]) / (times[k + 1] - times[k])
    rows = np.array([[*p.positions, *p.velocities, *p.accelerations] for p in points], float)
    if rows.shape != (len(times), 6) or not np.isfinite(rows).all():
        raise ValueError('Invalid planar trajectory')
    state = (1 - alpha) * rows[k, :4] + alpha * rows[k + 1, :4]
    return state[:2], state[2:4], rows[k, 4:]
