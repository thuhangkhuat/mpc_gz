"""Log-odds integration from explicit rays, without inferring missing rays."""

import numpy as np

from .planner_grid import IncrementalLogOddsGrid, njit


@njit(cache=False)
def ray_cells(origin, endpoints, hits, ox, oy, resolution, width, height):
    """Rasterize observed rays once per cell, protecting simultaneous hits."""
    free = np.zeros(width * height, dtype=np.bool_)
    occupied = np.zeros(width * height, dtype=np.bool_)
    step = resolution * 0.5
    for i in range(len(endpoints)):
        dx, dy = endpoints[i, 0] - origin[0], endpoints[i, 1] - origin[1]
        length = np.sqrt(dx * dx + dy * dy)
        if length < resolution:
            continue
        samples = max(1, int((length - resolution) / step))
        for j in range(samples):
            x = origin[0] + j * step / length * dx
            y = origin[1] + j * step / length * dy
            if x < ox or y < oy:
                continue
            ix, iy = int((x - ox) / resolution), int((y - oy) / resolution)
            if ix < width and iy < height:
                free[iy * width + ix] = True
        if hits[i] and endpoints[i, 0] >= ox and endpoints[i, 1] >= oy:
            ix = int((endpoints[i, 0] - ox) / resolution)
            iy = int((endpoints[i, 1] - oy) / resolution)
            if ix < width and iy < height:
                occupied[iy * width + ix] = True
    return free, occupied


def warmup_scan_integration():
    ray_cells(
        np.zeros(2), np.empty((0, 2)), np.empty(0, dtype=np.bool_), 0.0, 0.0, 0.15, 2, 2
    )


class SensorGrid(IncrementalLogOddsGrid):
    def integrate(self, scan):
        free, occupied = ray_cells(
            np.ascontiguousarray(scan.origin[:2], dtype=float),
            np.ascontiguousarray(scan.endpoints[:, :2], dtype=float),
            scan.hits,
            self.ox,
            self.oy,
            self.resolution,
            self.size_x,
            self.size_y,
        )
        flat = self.L.ravel()
        # Each cell receives one update per scan; grazing rays cannot clear hits.
        flat[free & ~occupied] += self.l_free
        flat[occupied] += self.l_occ
        np.clip(self.L, self.l_min, self.l_max, out=self.L)
        self._dirty = self._full_dirty = True

    def surface_points(self, center, radius):
        ys, xs = np.nonzero(self.L >= self.occ_logit)
        points = np.column_stack(
            [
                self.ox + (xs + 0.5) * self.resolution,
                self.oy + (ys + 0.5) * self.resolution,
            ]
        )
        return points[np.linalg.norm(points - center[:2], axis=1) <= radius]
