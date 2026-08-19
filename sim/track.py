"""The three competition maps as unions of corridor rectangles, rasterized
to a 5 mm occupancy grid for raycasting and collision.

Layouts are reconstructed from the team's photos (docs/images/) plus the
prior-art description of map 2; segment lengths are the 30.5 cm module
best guess and are easy to adjust once the team confirms them.

Route scripts: list of (trigger, turn) maneuvers consumed in order.
  ('F', 'L'/'R'): at a blocked front, pivot that way.
  ('O', 'L'/'R'): at the next opening on that side, creep to centre, pivot.
  ('X', None):   pass straight through a 4-way crossing. Consumed by the
                 both-sides-open -> both-sides-closed signature; while
                 pending, any front-block is a phantom (yawed view of the
                 crossing corner) and triggers realign instead of a turn.
Between maneuvers the robot is straight-first: it drives through any
junction whose front is open.
"""
import numpy as np

M = 305.0          # corridor module
H = M / 2          # half width, 152.5
RES = 5.0          # grid resolution mm
MOUTH = 80.0       # how far the entry/exit stubs extend past the maze


class Track:
    def __init__(self, name, rects, start_xy, start_heading_deg, script,
                 finish_rect):
        """rects: free-space rectangles (x0,y0,x1,y1). finish_rect: ground
        truth success zone (axle inside = course complete)."""
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
        """pts: (N,2) world mm -> bool array, True where inside a wall."""
        i = ((pts[:, 0] - self.x0) / RES).astype(int)
        j = ((pts[:, 1] - self.y0) / RES).astype(int)
        ok = (i >= 0) & (i < self.free.shape[1]) & (j >= 0) & (j < self.free.shape[0])
        out = np.ones(len(pts), dtype=bool)
        out[ok] = ~self.free[j[ok], i[ok]]
        # outside the grid bounding box = open floor beyond the mouths
        out[~ok] = False
        return out

    def raycast(self, x, y, ang, max_mm):
        """March a ray; returns (distance, wall_normal_angle) or (None, None).
        Normal angle is the axis-aligned outward normal of the hit cell face,
        used for the specular-reflection model."""
        step = 2.5
        dx, dy = np.cos(ang) * step, np.sin(ang) * step
        px, py = x, y
        n = int(max_mm / step)
        for k in range(1, n + 1):
            px += dx; py += dy
            i = int((px - self.x0) / RES); j = int((py - self.y0) / RES)
            if i < 0 or j < 0 or i >= self.free.shape[1] or j >= self.free.shape[0]:
                return None, None          # left the map: open floor
            if not self.free[j, i]:
                # decide which face was crossed by backtracking half a step
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


APRON = 480.0      # open floor modelled beyond each exit mouth


def _apron_x(cx, y_edge, direction):
    """Open-floor rectangle past a mouth at y_edge, corridor centred on cx.
    direction +1 = exit points up (+y), -1 = down."""
    if direction > 0:
        return (cx - H - 400, y_edge - 5, cx + H + 400, y_edge + APRON)
    return (cx - H - 400, y_edge - APRON, cx + H + 400, y_edge + 5)


def map1():
    """3-point map: U corridor around a block, two right turns."""
    rects = [
        (-M, -MOUTH, 0.0, 3 * M),          # left corridor
        (-M, 3 * M, 3 * M, 4 * M),         # top corridor
        (2 * M, -MOUTH, 3 * M, 4 * M),     # right corridor
        _apron_x(2.5 * M, -MOUTH, -1),     # open floor past the exit mouth
    ]
    script = [('F', 'R'), ('F', 'R')]
    return Track('map1-3pt', rects, (-H, 0.0), 90.0, script,
                 finish_rect=(2 * M, -MOUTH - 320, 3 * M, -MOUTH - 160))


def map2():
    """5-point map: ring with a middle corridor crossing (the 4-way) and a
    top exit branch. Straight through the crossing twice; 4 right turns."""
    rects = [
        (-H, -MOUTH, H, 2 * M + H),                 # entry, bottom-left, north
        (-H, 2 * M - H, 4 * M + H, 2 * M + H),      # top row, east
        (4 * M - H, -H, 4 * M + H, 2 * M + H),      # east leg, south
        (2 * M - H, -H, 4 * M + H, H),              # bottom row, west
        (2 * M - H, -H, 2 * M + H, 3 * M + MOUTH),  # middle vertical + exit
        _apron_x(2 * M, 3 * M + MOUTH, +1),
    ]
    script = [('F', 'R'), ('X', None), ('F', 'R'), ('F', 'R'), ('F', 'R'),
              ('X', None)]
    return Track('map2-5pt', rects, (0.0, 0.0), 90.0, script,
                 finish_rect=(2 * M - H, 3 * M + MOUTH + 160,
                              2 * M + H, 3 * M + MOUTH + 320))


def map3():
    """9-point map: entry, middle corridor, full ring around a block, back
    along the middle, up the second left branch to the top exit."""
    rects = [
        (-H, -MOUTH, H, 2 * M + H),                 # entry, north
        (-H, 2 * M - H, 3 * M + H, 2 * M + H),      # middle corridor, east
        (3 * M - H, -H, 3 * M + H, 4 * M + H),      # ring west leg
        (3 * M - H, -H, 5 * M + H, H),              # ring south leg
        (5 * M - H, -H, 5 * M + H, 4 * M + H),      # ring east leg
        (3 * M - H, 4 * M - H, 5 * M + H, 4 * M + H),  # ring north leg
        (M - H, 2 * M - H, M + H, 4 * M + MOUTH),   # top-left branch + exit
        _apron_x(M, 4 * M + MOUTH, +1),
    ]
    script = [('F', 'R'), ('F', 'R'), ('F', 'L'), ('F', 'L'), ('F', 'L'),
              ('F', 'L'), ('O', 'R'), ('O', 'R')]
    return Track('map3-9pt', rects, (0.0, 0.0), 90.0, script,
                 finish_rect=(M - H, 4 * M + MOUTH + 160,
                              M + H, 4 * M + MOUTH + 320))


MAPS = {'map1': map1, 'map2': map2, 'map3': map3}
