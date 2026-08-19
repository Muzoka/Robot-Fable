"""HC-SR04 model with the verified failure modes: beam cone (min distance
inside ~±12° of boresight), specular dropout as incidence leaves
perpendicular, close-range garbage (<45 mm reads random-far or nothing),
gaussian noise + per-unit bias, random dropout, max-range timeout.

Firing is strictly sequential (F, L, F, R — one ping per 20 ms control
tick), so cross-talk never exists in the model, same as the firmware."""
import numpy as np
import sim.config as C

INVALID = -1.0

CONE_DEG = [0.0, -5.0, 5.0, -11.0, 11.0]


class Sonar:
    def __init__(self, track, x_mm, y_mm, ang_deg, rng, noise_mm=3.0,
                 dropout=0.02):
        self.track = track
        self.off = np.array([x_mm, y_mm])
        self.ang = np.radians(ang_deg)
        self.rng = rng
        self.noise = noise_mm
        self.bias = rng.normal(0.0, 4.0)
        self.dropout = dropout

    def ping(self, robot):
        pos = robot.body_to_world(self.off[None, :])[0]
        base = robot.th + self.ang
        best = None
        best_inc = None
        for cd in CONE_DEG:
            d, normal = self.track.raycast(pos[0], pos[1],
                                           base + np.radians(cd), C.US_MAX_MM)
            if d is None:
                continue
            inc = abs(((base + np.radians(cd)) - (normal + np.pi) + np.pi)
                      % (2 * np.pi) - np.pi)
            if best is None or d < best:
                best, best_inc = d, inc
        if best is None:
            return INVALID                       # nothing in range: timeout
        # specular reflection: return probability vs incidence
        inc_deg = np.degrees(best_inc)
        if inc_deg < 15:
            p = 0.98
        elif inc_deg < 40:
            p = 0.9 - (inc_deg - 15) / 25 * 0.7
        else:
            p = 0.03
        if self.rng.random() > p or self.rng.random() < self.dropout:
            return INVALID
        if best < C.US_MIN_RELIABLE_MM:
            # ring-down zone: half the time a wild multipath spike, else lost
            if self.rng.random() < 0.5:
                return self.rng.uniform(170.0, 2000.0)
            return INVALID
        return max(20.0, best + self.bias + self.rng.normal(0.0, self.noise))


class SonarSuite:
    """The three sensors + the firmware-identical scheduling and filtering."""

    ORDER = ('F', 'L', 'F', 'R')

    def __init__(self, track, rng, noise_mm=3.0, dropout=0.02):
        self.s = {
            'F': Sonar(track, C.US_F_X_MM, C.US_F_Y_MM, 0.0, rng, noise_mm, dropout),
            'L': Sonar(track, C.US_L_X_MM, C.US_L_Y_MM, 90.0, rng, noise_mm, dropout),
            'R': Sonar(track, C.US_R_X_MM, C.US_R_Y_MM, -90.0, rng, noise_mm, dropout),
        }
        self.hist = {k: [] for k in 'FLR'}       # last 3 valid readings
        self.med = {k: None for k in 'FLR'}      # median-of-3
        self.last_raw = {k: None for k in 'FLR'}  # latest raw (None=no echo)
        self.misses = {k: 0 for k in 'FLR'}
        self.far = {k: False for k in 'FLR'}     # believed-far flag
        self.k = 0

    def tick(self, robot):
        """One control tick: ping the scheduled sensor, update filters.
        Returns the name of the sensor that was refreshed."""
        name = self.ORDER[self.k % 4]
        self.k += 1
        raw = self.s[name].ping(robot)
        self.last_raw[name] = None if raw == INVALID else raw
        if raw == INVALID:
            self.misses[name] += 1
            if self.misses[name] >= C.FAR_MISSES:
                self.far[name] = True            # believed open/far
        else:
            self.misses[name] = 0
            self.far[name] = False
            h = self.hist[name]
            if not h:
                h.extend([raw, raw, raw])        # prime with first real value
            else:
                h.append(raw)
                if len(h) > 3:
                    h.pop(0)
            self.med[name] = sorted(h)[1]
        return name

    def dist(self, name):
        """Filtered distance; far/invalid reported as a huge value."""
        if self.far[name] or self.med[name] is None:
            return C.US_MAX_MM
        return self.med[name]

    def wall(self, name):
        """Is there a wall on this side (valid, near reading)?"""
        if self.far[name] or self.med[name] is None:
            return False
        limit = C.SIDE_OPEN_MM if name in 'LR' else C.FRONT_OPEN_MM
        return self.med[name] < limit
