"""Exercise the standalone package's spawned worker and native solver offline."""

import os
import signal
from pathlib import Path

import numpy as np

from solo_mpc_hardware.core.controller import VehicleState
from solo_mpc_hardware.core.parallel_controller import ParallelTeamController
from solo_mpc_hardware.core.sensors import ScanRays
from solo_mpc_hardware.settings import load_settings


def test_single_uav_worker_solves_and_stops_without_parent_packages():
    config = Path(__file__).resolve().parents[1] / 'config'
    scenario, settings, cfg = load_settings(
        config / 'scenario.example.yaml', config / 'controller.example.yaml',
    )
    stamp = 1.0
    position = np.asarray(scenario['starts'][0], dtype=float)
    state = VehicleState(
        position, np.zeros(3), np.zeros(3), np.array([0., 0., 0., 1.]), stamp,
    )
    angles = np.linspace(-np.pi, np.pi, settings['lidar_samples'], endpoint=False)
    origin = position + [0., 0., .08]
    endpoints = origin + cfg['SENSING_RADIUS'] * np.column_stack([
        np.cos(angles), np.sin(angles), np.zeros(len(angles)),
    ])
    scan = ScanRays(origin, endpoints, np.zeros(len(angles), dtype=bool), stamp)
    with ParallelTeamController(
        ['uav1'], cfg,
        timeout=settings['controller_worker_timeout_wall'],
        startup_timeout=settings['controller_worker_startup_timeout_wall'],
    ) as team:
        worker = team.workers['uav1'].process
        assert worker.pid != os.getpid()
        # Terminal interrupts belong to the coordinator; it explicitly stops
        # workers after joining the active cycle and closing their channels.
        os.kill(worker.pid, signal.SIGINT)
        assert team.track.observe([5., 3., .07], stamp)
        result = team.step({'uav1': state}, {'uav1': scan}, stamp, {'uav1'})['uav1']
        assert result.get('nominal_ok'), result
        prediction, controls = result['prediction'], result['controls']
        assert prediction.shape == (cfg['HORIZON_LENGTH'] + 1, 6)
        assert controls.shape == (cfg['HORIZON_LENGTH'], 2)
        assert np.isfinite(prediction).all() and np.isfinite(controls).all()
        np.testing.assert_allclose(prediction[0], np.r_[position[:2], np.zeros(4)], atol=1e-5)
        assert np.max(np.linalg.norm(prediction[:, 2:4], axis=1)) <= cfg['VMAX'] + 1e-3
        assert np.max(np.linalg.norm(controls, axis=1)) <= cfg['UMAX'] + 1e-3
    assert not worker.is_alive()
