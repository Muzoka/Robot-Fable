"""The three competition maps, built cell-by-cell from the organisers'
spreadsheet (Robotics_Competition_Maps_v2_5x5_grid.xlsx, vendored in
docs/): a 5x5 grid of 1 ft (305 mm) cells, wall thickness 0.75 in
(negligible for simulation), wall height 7.5 in. Blue cells = corridor;
a missing outer border marks the entrance/exit mouth.

Grid coords: gx 0..4 west->east (columns B..F), gy 0..4 south->north
(rows 6..2). World mm: cell centre = ((gx+0.5)*M, (gy+0.5)*M).

Route scripts: list of (trigger, turn) maneuvers consumed in order.
  ('F', 'L'/'R'): at a blocked front, pivot that way.
  ('O', 'L'/'R'): at the next opening on that side, creep to centre, pivot.
  ('X', None):   pass straight through a 4-way crossing (active maneuver).
Between maneuvers the robot is straight-first. Note: with the confirmed
layouts, NO map needs an 'O' maneuver - every junction is a plain corner
or map 2's crossing.
"""
import numpy as np

M = 305.0          # corridor module (1 ft)
H = M / 2
RES = 5.0          # grid resolution mm
MOUTH = 80.0       # entry/exit stub past the maze wall line
APRON = 480.0      # open floor modelled beyond each mouth

DIRS = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
HEAD = {'N': 90.0, 'S': -90.0, 'E': 0.0, 'W': 180.0}


class Track:
    def __init__(self, name, rects, start_xy, start_heading_deg, script,
                 finish_rect):
        self.name = name
        self.rects = rects
        self.start_xy = np.array(start_xy, float)
        self.start_heading = np.radians(start_heading_deg)
        self.script = list(script)
        self.finish_rect = finish_rect
        xs = [r[0] for r in rects] + [r[2] for r in rects]
        ys = [r[1] for r in rects] + [r[3] for r in rects]
        pad = 3 * RES
        self.x0, self.y0 = min(xs) - pad, min(ys) - pad
        self.x1, self.y1 = max(xs) + pad, max(ys) + pad
        nx = int(np.ceil((self.x1 - self.x0) / RES))
        ny = int(np.ceil((self.y1 - self.y0) / RES))
        self.free = np.zeros((ny, nx), dtype=bool)
        for (a, b, c, d) in rects:
            i0 = int((a - self.x0) / RES); i1 = int(np.ceil((c - self.x0) / RES))
            j0 = int((b - self.y0) / RES); j1 = int(np.ceil((d - self.y0) / RES))
            self.free[j0:j1, i0:i1] = True

    def blocked(self, pts):
        i = ((pts[:, 0] - self.x0) / RES).astype(int)
        j = ((pts[:, 1] - self.y0) / RES).astype(int)
        ok = (i >= 0) & (i < self.free.shape[1]) & (j >= 0) & (j < self.free.shape[0])
        out = np.ones(len(pts), dtype=bool)
        out[ok] = ~self.free[j[ok], i[ok]]
        out[~ok] = False        # beyond the grid = open floor past the aprons
        return out

    def raycast(self, x, y, ang, max_mm):
        step = 2.5
        dx, dy = np.cos(ang) * step, np.sin(ang) * step
        px, py = x, y
        for k in range(1, int(max_mm / step) + 1):
            px += dx; py += dy
            i = int((px - self.x0) / RES); j = int((py - self.y0) / RES)
            if i < 0 or j < 0 or i >= self.free.shape[1] or j >= self.free.shape[0]:
                return None, None
            if not self.free[j, i]:
                bx, by = px - dx / 2, py - dy / 2
                ci = int((bx - self.x0) / RES); cj = int((by - self.y0) / RES)
                if ci != i:
                    normal = np.pi if dx > 0 else 0.0
                elif cj != j:
                    normal = -np.pi / 2 if dy > 0 else np.pi / 2
                else:
                    normal = ang + np.pi
                return k * step, normal
        return None, None

    def in_finish(self, x, y):
        a, b, c, d = self.finish_rect
        return a <= x <= c and b <= y <= d


def _cell_rect(gx, gy):
    return (gx * M, gy * M, (gx + 1) * M, (gy + 1) * M)


def _mouth_rects(gx, gy, edge):
    """Stub through the wall line + open-floor apron beyond it."""
    dx, dy = DIRS[edge]
    cx, cy = (gx + 0.5) * M, (gy + 0.5) * M
    if dx:      # east/west mouth
        x_edge = (gx + (1 if dx > 0 else 0)) * M
        stub = (min(x_edge, x_edge + dx * MOUTH), cy - H,
                max(x_edge, x_edge + dx * MOUTH), cy + H)
        a0, a1 = sorted((x_edge + dx * (MOUTH - 5), x_edge + dx * (MOUTH + APRON)))
        apron = (a0, cy - H - 400, a1, cy + H + 400)
    else:
        y_edge = (gy + (1 if dy > 0 else 0)) * M
        stub = (cx - H, min(y_edge, y_edge + dy * MOUTH),
                cx + H, max(y_edge, y_edge + dy * MOUTH))
        a0, a1 = sorted((y_edge + dy * (MOUTH - 5), y_edge + dy * (MOUTH + APRON)))
        apron = (cx - H - 400, a0, cx + H + 400, a1)
    return stub, apron


def _finish_band(gx, gy, edge):
    """Success = the axle fully past the mouth line (the robot has driven
    out through the exit gate) - the competition's finish condition. The
    band spans the whole apron width so an exit with residual yaw still
    counts, exactly as it would on real open floor."""
    dx, dy = DIRS[edge]
    cx, cy = (gx + 0.5) * M, (gy + 0.5) * M
    w = H + 380
    if dx:
        x_edge = (gx + (1 if dx > 0 else 0)) * M
        lo, hi = sorted((x_edge + dx * (MOUTH + 100), x_edge + dx * (MOUTH + APRON - 20)))
        return (lo, cy - w, hi, cy + w)
    y_edge = (gy + (1 if dy > 0 else 0)) * M
    lo, hi = sorted((y_edge + dy * (MOUTH + 100), y_edge + dy * (MOUTH + APRON - 20)))
    return (cx - w, lo, cx + w, hi)


def _build(name, cells, entrance, exit_, script):
    """cells: set of (gx, gy). entrance/exit_: ((gx, gy), edge)."""
    rects = [_cell_rect(*c) for c in cells]
    (egx, egy), eedge = entrance
    stub, _ = _mouth_rects(egx, egy, eedge)
    rects.append(stub)
    (xgx, xgy), xedge = exit_
    stub2, apron = _mouth_rects(xgx, xgy, xedge)
    rects += [stub2, apron]
    # start: on the mouth line of the entrance cell, heading inward
    # (eedge names the OUTER edge the mouth is on; drive away from it)
    dx, dy = DIRS[eedge]
    sx = (egx + 0.5) * M + dx * H
    sy = (egy + 0.5) * M + dy * H
    heading = HEAD[{'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[eedge]]
    return Track(name, rects, (sx, sy), heading, script,
                 _finish_band(xgx, xgy, xedge))


def map1():
    """3 sectors: U ring - up the west column, east along the top row,
    down the east column. Entrance/exit on the south wall."""
    cells = {(0, gy) for gy in range(5)} | {(4, gy) for gy in range(5)} \
        | {(gx, 4) for gx in range(1, 4)}
    return _build('map1-3pt', cells, ((0, 0), 'S'), ((4, 0), 'S'),
                  [('F', 'R'), ('F', 'R')])


def map2():
    """5 sectors: top row, east down-leg, lower loop, middle row with the
    4-way crossing at (3,2); exit is the crossing's east arm."""
    cells = ({(gx, 4) for gx in range(4)} | {(3, 3)}
             | {(gx, 2) for gx in range(1, 5)}
             | {(1, 1), (3, 1)} | {(gx, 0) for gx in range(1, 4)})
    return _build('map2-5pt', cells, ((0, 4), 'W'), ((4, 2), 'E'),
                  [('F', 'R'), ('X', None), ('F', 'R'), ('F', 'R'),
                   ('F', 'R'), ('X', None)])


def map3():
    """9 sectors: pure serpentine, eight plain corners, no branches."""
    cells = {(0, 4), (1, 4), (3, 4), (4, 4), (1, 3), (3, 3),
             (0, 2), (1, 2), (3, 2), (4, 2), (0, 1), (4, 1),
             (0, 0), (1, 0), (2, 0), (3, 0), (4, 0)}
    return _build('map3-9pt', cells, ((0, 4), 'W'), ((4, 4), 'E'),
                  [('F', 'R'), ('F', 'R'), ('F', 'L'), ('F', 'L'),
                   ('F', 'L'), ('F', 'L'), ('F', 'R'), ('F', 'R')])


MAPS = {'map1': map1, 'map2': map2, 'map3': map3}
