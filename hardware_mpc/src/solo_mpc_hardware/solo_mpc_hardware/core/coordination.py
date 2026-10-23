"""Upstream robot_jps mode/assignment/slot semantics over a fleet snapshot.

Camera mode supplies physical detections; external mode supplies geometric
visibility from a fresh global position. SEARCH is active target pursuit:
distant followers join formation after passing the leader-distance hysteresis.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment


class FleetGuidance:
    def __init__(self, names, cfg):
        self.names, self.cfg = list(names), cfg
        self.leader = None
        self.assignment = {}
        self.modes = {n: 'SEARCH' for n in names}
        self.c_in = dict.fromkeys(names, 0)
        self.c_out = dict.fromkeys(names, 0)
        self.slot_cells = {}
        self.slot_scales = dict.fromkeys(names, 1.0)

    def _free_ray_length(self, maps, origin, direction, length):
        cfg = self.cfg
        s = np.arange(.5 * cfg['GRID_RESOLUTION'], length + 1e-9,
                      .5 * cfg['GRID_RESOLUTION'])
        P = np.asarray(origin)[None, :2] + s[:, None] * direction[None]
        x0, y0, x1, y1 = cfg['WORLD_BOUNDS']
        m = cfg['GOAL_CLAMP_MARGIN']
        occ = ~((P[:, 0] >= x0 + m) & (P[:, 0] <= x1 - m)
                & (P[:, 1] >= y0 + m) & (P[:, 1] <= y1 - m))
        blk = np.zeros(len(s), bool)
        for gm in maps:
            gm.is_blocked(0, 0)  # materialize inflation
            ny, nx = gm.blocked.shape
            ix = np.floor((P[:, 0] - gm.ox) / gm.resolution).astype(int)
            iy = np.floor((P[:, 1] - gm.oy) / gm.resolution).astype(int)
            inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            occ[inb] |= (gm.L[iy[inb], ix[inb]] >= gm.occ_logit) if hasattr(gm, 'L') else gm.occupied[iy[inb], ix[inb]]
            blk[inb] |= (gm.blocked[iy[inb], ix[inb]].astype(bool)
                         & ~gm.unknown[iy[inb], ix[inb]].astype(bool))
        n = len(s) if not occ.any() else int(np.argmax(occ))
        free = np.flatnonzero(~blk[:n])
        if n == len(s) and len(free) and free[-1] == len(s) - 1:
            return length
        return float(s[free[-1]]) if len(free) else 0.0

    def coordinate(self, states, predictions, target, seers, maps):
        cfg = self.cfg
        L = cfg['VIEWING_RADIUS']
        distance = {n: float(np.max(np.abs(s.position[:2] - target))) for n, s in states.items()}
        relaxed = [n for n in self.names if n in seers and distance[n] <= L]
        strict = [n for n in relaxed if distance[n] <= L * (1 - cfg['VISIBILITY_MARGIN_RATIO'])]
        if self.leader not in relaxed:
            candidates = strict or relaxed
            self.leader = min(candidates, key=distance.get) if candidates else None
        for n in self.names:
            if self.leader is None:
                self.modes[n], self.c_in[n] = 'SEARCH', 0
            elif n == self.leader:
                self.modes[n] = 'TRACK'
            else:
                enter = L + cfg['FORMATION_GAP']
                leave = enter + cfg['TRACK_EXIT_HYSTERESIS']
                if n in self.slot_cells:
                    leave = max(leave, np.linalg.norm(self.slot_cells[n]) * cfg['SLOT_SPACING_RATIO'] * L
                                + cfg['TRACK_EXIT_HYSTERESIS'])
                d = np.linalg.norm(states[n].position[:2] - states[self.leader].position[:2])
                if self.modes[n] == 'SEARCH':
                    self.c_in[n] = self.c_in[n] + 1 if d <= enter else 0
                    if self.c_in[n] >= cfg['K_IN_THRESHOLD']:
                        self.modes[n], self.c_out[n] = 'TRACK', 0
                else:
                    self.c_out[n] = self.c_out[n] + 1 if d >= leave else 0
                    if self.c_out[n] >= cfg['K_OUT_THRESHOLD']:
                        self.modes[n], self.c_in[n] = 'SEARCH', 0
        if self.leader is None:
            return {}
        sats = [n for n in self.names if n != self.leader]
        active = [n for n in sats if self.modes[n] == 'TRACK']
        # Upstream retains the team assignment while every follower is SEARCH.
        if not active:
            return {}
        m = max(len(sats), min(4 if cfg['OPEN_ALL_SLOTS'] else len(sats), len(cfg['GRID_CELLS'])))
        cells = np.asarray((list(cfg['GRID_CELLS']) + list(cfg['EXTRA_CELLS']))[:m], float)
        lead = states[self.leader].position[:2]
        offsets = cells * cfg['SLOT_SPACING_RATIO'] * L
        costs = np.array([[np.linalg.norm(states[n].position[:2] - lead - o) for o in offsets] for n in sats])
        rows, cols = linear_sum_assignment(costs)
        assign = {sats[r]: int(c) for r, c in zip(rows, cols)}
        if set(self.assignment) == set(sats) and all(i < m for i in self.assignment.values()):
            old = sum(costs[sats.index(n), i] for n, i in self.assignment.items())
            new = sum(costs[sats.index(n), i] for n, i in assign.items())
            if new > old - cfg['SWITCH_MARGIN']:
                assign = self.assignment
        self.assignment = assign
        lp = predictions[self.leader]
        points = list(lp[::max(1, cfg['HORIZON_LENGTH'] // 4), :2])
        # Predictions have canonical [px,py,vx,vy,ax,ay] layout.
        for tau in np.arange(.5, cfg['SLOT_LOOKAHEAD_TIME'] - cfg['HORIZON_LENGTH'] * cfg['TIMESTEP'] + 1e-9, .5):
            points.append(lp[-1, :2] + lp[-1, 2:4] * tau)
        slots = {}
        for n in active:
            cell = cells[assign[n]]
            self.slot_cells[n] = cell
            full = np.linalg.norm(offsets[assign[n]])
            scale = 1.0
            if cfg['SLOT_CONTRACT']:
                free = min(self._free_ray_length(maps, p, cell / np.linalg.norm(cell), full) for p in points)
                scale = min(float(np.clip(free / full, cfg['SLOT_MIN_RATIO'], 1)),
                            self.slot_scales[n] + cfg['SLOT_EXPAND_SPEED'] * cfg['TIMESTEP'] / full)
            self.slot_scales[n] = scale
            slots[n] = lead + offsets[assign[n]] * scale
        return slots

    def moving_slot(self, name, predictions):
        return predictions[self.leader][:, :2] + self.slot_cells[name] * (
            self.cfg['SLOT_SPACING_RATIO'] * self.cfg['VIEWING_RADIUS'] * self.slot_scales[name])


def path_carrot(position, path, distance):
    points = np.vstack([position[:2], np.asarray(path)[1:, :2]])
    for a, b in zip(points[:-1], points[1:]):
        length = float(np.linalg.norm(b - a))
        if length >= distance:
            return a + (b - a) * (distance / length) if length else a.copy()
        distance -= length
    return None
