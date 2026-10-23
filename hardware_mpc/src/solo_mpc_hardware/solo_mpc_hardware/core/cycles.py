"""One outstanding fleet computation; callbacks own inputs, workers own planning."""

from concurrent.futures import ThreadPoolExecutor
import copy
import time


class CycleRunner:
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='fleet-cycle')
        self.future = None
        self.started = None
        self.closed = False

    def submit(self, function, *args, **kwargs):
        if self.closed or self.future is not None:
            raise RuntimeError('A fleet cycle is already active or closed')
        # Callbacks can replace/mutate live messages immediately after submission.
        args, kwargs = copy.deepcopy((args, kwargs))
        self.started = time.monotonic()
        self.future = self.executor.submit(function, *args, **kwargs)

    def take(self):
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        return future.result()

    def close(self):
        self.closed = True
        # ParallelTeamController enforces phase deadlines and kills stalled
        # native workers, so the coordinator is joined before its pipes close.
        self.executor.shutdown(wait=True, cancel_futures=True)


def result_is_current(source_stamp, now, started_wall, settings):
    age = now - source_stamp
    return (0 <= age <= settings['command_timeout_sim']
            and time.monotonic() - started_wall <= settings['command_timeout_wall'])


def stopping_result_matches(state, result, source_stamp, cfg, velocity_stamp=None):
    """Revalidate a fallback against measurements received while it computed."""
    import numpy as np
    rows = result['prediction']
    velocity_stamp = state.stamp if velocity_stamp is None else velocity_stamp
    indices = [min(int(max(0., stamp - source_stamp) / cfg['TIMESTEP'] + 1e-6), len(rows)-1)
               for stamp in (state.stamp, velocity_stamp, velocity_stamp)]
    measured = (state.position[:2], state.velocity[:2], state.acceleration[:2])
    expected = [rows[index, 2 * component:2 * component + 2]
                for component, index in enumerate(indices)]
    return all(np.isfinite(value).all() and np.linalg.norm(value - target) <= cfg[key]
               for value, target, key in zip(measured, expected,
                   ('BACKUP_POSITION_TOLERANCE', 'BACKUP_VELOCITY_TOLERANCE', 'BACKUP_ACCEL_TOLERANCE')))
