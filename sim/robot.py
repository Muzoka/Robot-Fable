"""Differential-drive physics with the measured robot's quirks: motor
deadband + hysteresis, first-order lag, per-motor gain mismatch (the
measured rightward drift), battery sag, traction-limited acceleration,
active-brake vs coast stops, and wall-contact sliding."""
import numpy as np
import sim.config as C


class Motor:
    def __init__(self, gain, deadband_start, deadband_sustain, rng):
        self.gain = gain
        self.db_start = deadband_start
        self.db_sustain = deadband_sustain
        self.v = 0.0
        self.rng = rng

    def step(self, pwm, brake, battery, dt):
        db = self.db_sustain if abs(self.v) > 5.0 else self.db_start
        if brake or abs(pwm) < db:
            tau = C.BRAKE_TAU_S if brake else 0.25   # coast: draggy gearbox
            target = 0.0
        else:
            tau = C.MOTOR_TAU_S
            mag = (abs(pwm) - C.PWM_FLOOR) / (C.PWM_MAX - C.PWM_FLOOR)
            mag = max(0.0, min(1.0, mag))
            target = np.sign(pwm) * C.V_MAX_MMS * battery * self.gain * mag
        dv = (target - self.v) / tau * dt
        lim = C.ACCEL_LIMIT_MMS2 * dt
        self.v += max(-lim, min(lim, dv))
        return self.v


class Robot:
    """Pose = axle midpoint. Heading in radians, 0 = +x."""

    def __init__(self, track, rng, gain_l=1.0, gain_r=1.0, battery=1.0,
                 db_start=C.PWM_FLOOR, db_sustain=C.PWM_FLOOR - 15):
        self.track = track
        self.rng = rng
        self.x, self.y = track.start_xy
        self.th = track.start_heading
        self.battery = battery
        self.ml = Motor(gain_l, db_start, db_sustain, rng)
        self.mr = Motor(gain_r, db_start, db_sustain, rng)
        self.contacts = 0
        self.in_contact = False
        self.last_contact_t = -1.0
        self.t = 0.0
        # perimeter sample points in body frame (x fwd, y left)
        xs = np.linspace(-C.AXLE_TO_REAR_MM, C.AXLE_TO_FRONT_MM, 8)
        w = C.ROBOT_WIDTH_MM / 2
        pts = ([(x, w) for x in xs] + [(x, -w) for x in xs] +
               [(C.AXLE_TO_FRONT_MM, y) for y in np.linspace(-w, w, 6)] +
               [(-C.AXLE_TO_REAR_MM, y) for y in np.linspace(-w, w, 6)])
        self.perim = np.array(pts)

    def body_to_world(self, pts, x=None, y=None, th=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        th = self.th if th is None else th
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s], [s, c]])
        return pts @ R.T + np.array([x, y])

    def step(self, pwm_l, pwm_r, brake, dt):
        vl = self.ml.step(pwm_l, brake, self.battery, dt)
        vr = self.mr.step(pwm_r, brake, self.battery, dt)
        v = (vl + vr) / 2.0
        w = (vr - vl) / C.WHEEL_TRACK_MM
        nx = self.x + v * np.cos(self.th) * dt
        ny = self.y + v * np.sin(self.th) * dt
        nth = self.th + w * dt
        hit = self.track.blocked(self.body_to_world(self.perim, nx, ny, nth))
        if hit.any():
            # wall contact: slide (keep only the motion component that
            # doesn't push in), damp, count an episode
            if not self.in_contact and self.t - self.last_contact_t > 0.3:
                self.contacts += 1
            self.in_contact = True
            self.last_contact_t = self.t
            # try damped-rotation-only, then axis-projected translations
            half = self.th + 0.5 * (nth - self.th)
            for cx, cy, cth in ((self.x, self.y, half),
                                (nx, self.y, self.th),
                                (self.x, ny, self.th)):
                h = self.track.blocked(self.body_to_world(self.perim, cx, cy, cth))
                if not h.any():
                    self.x, self.y, self.th = cx, cy, cth
                    break
            self.ml.v *= 0.5
            self.mr.v *= 0.5
        else:
            self.in_contact = False
            self.x, self.y, self.th = nx, ny, nth
        self.t += dt
