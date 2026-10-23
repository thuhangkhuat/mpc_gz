"""Geometry and scan validity shared by ROS adapters and offline tests."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class ScanRays:
    origin: np.ndarray
    endpoints: np.ndarray
    hits: np.ndarray
    stamp: float


def project_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    position,
    quaternion,
    stamp,
    neighbors=(),
    neighbor_radius=0.5,
    height_tolerance=1.0,
):
    """Only observed rays carve free space. NaN/zero/too-short rays remain unknown.

    Positive infinity is an observed no-return ray. Rays ending on teammates
    retain their free prefix but never permanently mark the teammate occupied.
    """
    ranges = np.asarray(ranges, float)
    valid = np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= range_min))
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    lengths = np.minimum(ranges[valid], range_max)
    local = np.column_stack(
        [
            lengths * np.cos(angles[valid]),
            lengths * np.sin(angles[valid]),
            np.zeros(valid.sum()),
        ]
    )
    origin = np.asarray(position, float)
    endpoints = Rotation.from_quat(quaternion).apply(local) + origin
    hits = np.isfinite(ranges[valid]) & (ranges[valid] < range_max - 1e-3)
    # Ground hits and strongly tilted rays do not establish traversable XY space.
    plane = (np.abs(endpoints[:, 2] - origin[2]) < height_tolerance) & (
        endpoints[:, 2] > 0.3
    )
    endpoints, hits = endpoints[plane], hits[plane]
    for p in neighbors:
        hits &= np.linalg.norm(endpoints - np.asarray(p), axis=1) > neighbor_radius
    return ScanRays(origin, endpoints, hits, stamp)


def pixel_to_ground(pixel, intrinsic, position, quaternion, ground_z=0.05):
    """Gazebo camera: +X forward, +Y left, +Z up; camera points body -Z.

    Optical right = body -Y; optical down = body -X; optical forward = body -Z.
    Matches sensor pose roll=0, pitch=pi/2, yaw=0 in the generated SDF.
    """
    ray = np.linalg.solve(np.asarray(intrinsic).reshape(3, 3), [*pixel, 1.0])
    body_ray = np.array([-ray[1], -ray[0], -ray[2]])
    direction = Rotation.from_quat(quaternion).apply(body_ray)
    if direction[2] >= -1e-6:
        raise ValueError("Camera ray does not intersect ground in front of the camera")
    scale = (ground_z - position[2]) / direction[2]
    if scale <= 0:
        raise ValueError("Ground plane is behind camera")
    return np.asarray(position) + scale * direction


class PoseHistory:
    def __init__(self, tolerance=0.3, capacity=100):
        from collections import deque

        self.samples = deque(maxlen=capacity)
        self.tolerance = tolerance

    def add(self, stamp, position, quaternion):
        self.samples.append((stamp, np.asarray(position), np.asarray(quaternion)))

    def at(self, stamp):
        if not self.samples:
            raise ValueError("No measured pose available")
        sample = min(self.samples, key=lambda x: abs(x[0] - stamp))
        if abs(sample[0] - stamp) > self.tolerance:
            raise ValueError("No measured pose close to sensor exposure")
        return sample[1], sample[2]
