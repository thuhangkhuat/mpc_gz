"""Timestamped target inputs with bounded coasting and optional camera filtering."""

import numpy as np


class TargetTracker:
    def __init__(self, process_noise_std=2.0, obs_noise_std=0.3, coast_time=1.5):
        self.x = None
        self.P = np.eye(4)
        self.stamp = None
        self.observed_at = None
        self.observation = None
        self.process_noise_std = process_noise_std
        self.obs_noise_std = obs_noise_std
        self.coast_time = coast_time

    def _predict(self, stamp):
        dt = max(0.0, stamp - self.stamp)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        B = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + B @ B.T * self.process_noise_std ** 2
        self.stamp = stamp

    def observe(self, position, stamp, variance=None):
        z = np.asarray(position, float)[:2]
        variance = self.obs_noise_std ** 2 if variance is None else variance
        if z.shape != (2,) or not np.isfinite(z).all() or not np.isfinite(stamp):
            return False
        if not np.isfinite(variance) or variance <= 0:
            return False
        if self.x is None:
            self.x = np.r_[z, 0.0, 0.0]
            self.P = np.diag([1.0, 1.0, 100.0, 100.0])
            self.stamp = stamp
        elif stamp > self.stamp:
            self._predict(stamp)
            H = np.eye(2, 4)
            K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + np.eye(2) * variance)
            self.x += K @ (z - H @ self.x)
            self.P = (np.eye(4) - K @ H) @ self.P
        else:
            return False
        self.observed_at = stamp
        self.observation = z.copy()
        return True

    def velocity(self, stamp):
        age = max(0.0, stamp - self.stamp)
        return self.x[2:].copy() * max(0.0, 1.0 - age / self.coast_time)

    def position(self, stamp):
        if self.x is None:
            raise ValueError('Target has never been observed')
        # At an observation epoch the upstream horizon starts at the measurement,
        # while lead pursuit starts at the filtered position.
        return self.observation + min(max(0.0, stamp - self.stamp), self.coast_time) * self.x[2:]

    def trajectory(self, stamp, horizon, dt, max_speed):
        position = self.position(stamp)
        velocity = self.velocity(stamp)
        speed = np.linalg.norm(velocity)
        if speed > max_speed:
            velocity *= max_speed / speed
        return position + np.arange(horizon + 1)[:, None] * dt * velocity

    def pursuit(self, position, stamp, cfg):
        current = self.position(stamp)
        uncertainty = np.sqrt(self.P[2, 2] + self.P[3, 3])
        if not cfg['USE_LEAD_PURSUIT'] or uncertainty > cfg['VELOCITY_TRUSTED_THRESHOLD']:
            return current
        velocity = self.velocity(stamp)
        gain = np.clip(1 - np.linalg.norm(velocity) / cfg['VMAX'], cfg['LEAD_GAIN_MIN'], cfg['LEAD_GAIN_MAX'])
        filtered = self.x[:2] + current - self.observation
        travel_time = np.linalg.norm(filtered - position[:2]) / max(cfg['VMAX'], 1e-3)
        return filtered + velocity * travel_time * gain


class ExactTargetTracker(TargetTracker):
    """Keep exact external positions with upstream Kalman velocity/lead pursuit."""

    def __init__(self, process_noise_std=2.0, obs_noise_std=0.3, coast_time=1.5):
        super().__init__(process_noise_std, obs_noise_std, coast_time)
        self.estimator = TargetTracker(process_noise_std, obs_noise_std, coast_time)

    def observe(self, position, stamp, variance=None):
        if not self.estimator.observe(position, stamp, variance):
            return False
        # Upstream also receives the true target position globally. Its SEARCH
        # horizon starts at that measurement, while lead pursuit uses the CV
        # filter's position, velocity and velocity-confidence gate.
        self.x = self.estimator.x.copy()
        self.x[:2] = self.estimator.observation
        self.P = self.estimator.P.copy()
        self.stamp = self.observed_at = stamp
        self.observation = self.estimator.observation.copy()
        return True

    def pursuit(self, position, stamp, cfg):
        return self.estimator.pursuit(position, stamp, cfg)
