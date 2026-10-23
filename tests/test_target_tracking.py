"""External positions stay exact; timestamps determine velocity and freshness."""

import numpy as np
import pytest

from solo_mpc.core.tracking import ExactTargetTracker, TargetTracker


def test_exact_samples_preserve_turns_with_upstream_velocity_estimation():
    track = ExactTargetTracker()
    estimator = TargetTracker()
    for position, stamp in [([1, 2], 3.), ([3, 2], 3.5), ([3, 1], 3.75)]:
        assert track.observe([*position, .02], stamp)
        assert estimator.observe(position, stamp)
        np.testing.assert_array_equal(track.position(stamp), position)
        np.testing.assert_array_equal(track.x[:2], position)
        np.testing.assert_allclose(track.x[2:], estimator.x[2:])
        np.testing.assert_allclose(track.P, estimator.P)
        assert track.observed_at == stamp
    trajectory = track.trajectory(3.75, 2, .1, 3)
    np.testing.assert_array_equal(trajectory[0], [3, 1])
    assert np.all(np.linalg.norm(np.diff(trajectory, axis=0), axis=1) <= .3 + 1e-12)
    # Predictions cannot renew the feed's freshness, and coasting remains bounded.
    track.position(100.)
    np.testing.assert_array_equal(track.velocity(100.), [0, 0])
    assert track.observed_at == 3.75


def test_external_input_keeps_the_upstream_velocity_confidence_gate():
    track = ExactTargetTracker()
    track.observe([100, 100], 0.)
    track.observe([100.5, 100], .1)
    cfg = {'USE_LEAD_PURSUIT': True, 'VELOCITY_TRUSTED_THRESHOLD': 1.5}
    assert np.sqrt(track.P[2, 2] + track.P[3, 3]) > cfg['VELOCITY_TRUSTED_THRESHOLD']
    np.testing.assert_array_equal(track.pursuit([400, 350], .1, cfg), [100.5, 100])


@pytest.mark.parametrize('position,stamp', [([9, 9], 2.), ([9, 9], 1.),
                                          ([float('nan'), 1], 3.), ([1], 3.),
                                          ([1, 2], float('nan'))])
def test_invalid_or_nonincreasing_exact_samples_do_not_replace_the_latest(position, stamp):
    track = ExactTargetTracker()
    assert track.observe([1, 2], 2.)
    assert not track.observe(position, stamp)
    np.testing.assert_array_equal(track.x, [1, 2, 0, 0])
    assert track.observed_at == 2.
