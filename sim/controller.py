"""The competition controller — written to port line-for-line to Arduino.

State machine:
  CRUISE     wall-centering PD at cruise speed; straight-first at junctions
  APPROACH   front wall seen: slow, range down to the stop line
  DECIDE     stopped + settled: consume the next script maneuver
  TURN       timed pivot, then sensor verify + micro-correction
  REACQUIRE  gentle PD re-entry after a turn
  CREEP      side opening matched a script 'O' maneuver: roll axle to centre
  BACKUP     stuck/too-close recovery
  FINISH     script done, all sensors open: clear the mouth and stop

No floats are required by the logic itself except the PD math — everything
transfers to the Uno directly."""
import sim.config as C


class Controller:
    def __init__(self, script, auto=False):
        # auto=True: map-agnostic mode. No script. The universal rule set
        # for these tracks (and any chain of them): follow the corridor;
        # at a blocked front turn to the open side (wiggle-verify if both
        # look open); at a both-sides-open crossing drive straight through;
        # only sustained all-open space is the exit. Works because every
        # turn on every official map is forced and every crossing is a
        # straight-through.
        self.auto = auto
        self.auto_wig = False
        self.script = [] if auto else list(script)
        self.state = 'CRUISE'
        self.timer = 0
        self.pwm_l = 0
        self.pwm_r = 0
        self.brake = False
        self.err_prev = 0.0
        self.err_hist = []
        self.deriv = 0.0
        self.front_blocked_n = 0
        self.open_n = {'L': 0, 'R': 0}
        self.side_was_wall = {'L': False, 'R': False}
        self.turn_dir = None
        self.trims_done = 0
        self.all_open_n = 0
        self.stuck_ms = 0
        self.last_meds = None
        self.backups = 0
        self.done = False
        self.anomaly = []
        self.last_valid_f = None
        self.f_prev_refresh = None
        self.chaos = 0
        self.dec_phase = 0
        self.x_open_n = 0
        self.x_close_n = 0
        self.x_armed = False
        self.fin_armed = False
        self.fin_wall_n = 0
        self.appr_abort_n = 0
        self.empty_blocks = 0
        self.x_fclear_ms = 0
        self.forced_dir = None

    # ---------------- helpers ----------------

    def _slew(self, tl, tr):
        d = C.PWM_SLEW
        self.pwm_l += max(-d, min(d, tl - self.pwm_l))
        self.pwm_r += max(-d, min(d, tr - self.pwm_r))

    def _steer(self, sonar, base, gain=1.0, refreshed=None, kd_only=False,
               both_only=False):
        """Wall-centering PD. Positive error = robot right of centre.
        The derivative only updates when a side sensor actually refreshed
        (every 80 ms) — differentiating stale data invents spikes.
        kd_only: derivative-only mode — nulling the error RATE squares the
        heading to the corridor regardless of lateral position (the
        no-gyro wall-parallel alignment used after every pivot)."""
        L, R = sonar.dist('L'), sonar.dist('R')
        # track a side only while median AND the latest raw sample agree a
        # wall is near - the median lags ~2 refreshes at openings and the
        # PD would chase the stale value sideways into the gap
        rl, rr = sonar.last_raw['L'], sonar.last_raw['R']
        lw = sonar.wall('L') and L < C.TRACK_L_MM \
            and (rl is None or rl < C.TRACK_L_MM + 60)
        rw = sonar.wall('R') and R < C.TRACK_R_MM \
            and (rr is None or rr < C.TRACK_R_MM + 60)
        if both_only and not (lw and rw):
            lw = rw = False                # junction ahead: hold, don't chase
        if lw and rw:
            err = ((L - C.TARGET_L_MM) - (R - C.TARGET_R_MM)) / 2.0
        elif lw:
            err = L - C.TARGET_L_MM
        elif rw:
            err = C.TARGET_R_MM - R
        else:
            err = 0.0
        # derivative from the RAW error (clamping first would freeze the
        # rate signal exactly when it matters most), over a 4-refresh window
        # (median-filter steps make adjacent-sample derivatives read zero)
        if refreshed in ('L', 'R'):
            self.err_hist.append(err)
            if len(self.err_hist) > 5:
                self.err_hist.pop(0)
            if len(self.err_hist) >= 2:
                span = (len(self.err_hist) - 1) * 0.040
                d = (err - self.err_hist[0]) / span
                self.deriv = max(-C.DERIV_CLAMP_MMS,
                                 min(C.DERIV_CLAMP_MMS, d))
            self.err_prev = err
        err = max(-C.ERR_CLAMP_MM, min(C.ERR_CLAMP_MM, err))
        if abs(err) < C.ERR_DEADBAND_MM:
            err = 0.0
        if kd_only:
            steer = C.KD_SQUARE * self.deriv
        else:
            steer = gain * (C.KP_WALL * err + C.KD_WALL * self.deriv)
        steer = max(-C.STEER_LIMIT, min(C.STEER_LIMIT, steer))
        # err > 0: too far right -> steer left -> right wheel faster
        tl = base - steer
        tr = base + steer + C.TRIM_R_PWM
        clamp = lambda v: max(-C.PWM_MAX, min(C.PWM_MAX, v))
        return clamp(tl), clamp(tr)

    def _edges(self, sonar, refreshed):
        """Track debounced wall->open edges on the refreshed side sensor,
        from RAW samples: a valid near echo resets (wall really there); a
        valid LONG echo (seeing through the gap) or a timeout counts as
        open evidence. A specular mirage of a near wall yields long pure-
        timeout streaks only rarely at OPEN_DEBOUNCE=5."""
        if refreshed not in 'LR':
            return
        s = refreshed
        raw = sonar.last_raw[s]
        if raw is not None and raw < C.SIDE_OPEN_MM:
            self.side_was_wall[s] = True
            self.open_n[s] = 0
        elif self.side_was_wall[s]:
            self.open_n[s] += 1

    def _next(self):
        return self.script[0] if self.script else None

    # ---------------- main tick (50 Hz) ----------------

    def tick(self, sonar, refreshed):
        self.timer += C.LOOP_MS
        self.run_ms = getattr(self, 'run_ms', 0) + C.LOOP_MS
        F = sonar.dist('F')
        self._edges(sonar, refreshed)
        if refreshed == 'F':
            if not sonar.far['F'] and sonar.med['F'] is not None:
                self.last_valid_f = F
            # chaos score: wild refresh-to-refresh front jumps mean the nose
            # is in the sensor blind zone or multipath - always bad news
            if self.f_prev_refresh is not None:
                far_now = sonar.far['F']
                if abs(F - self.f_prev_refresh) > C.CHAOS_JUMP_MM:
                    self.chaos = min(self.chaos + 1, C.CHAOS_TRIP + 2)
                elif not far_now and self.chaos > 0:
                    self.chaos -= 1        # only a calm VALID pair is proof
                elif far_now:
                    self.f_far_n = getattr(self, 'f_far_n', 0) + 1
                    if self.f_far_n >= 8 and self.chaos > 0:
                        self.chaos -= 1    # slow leak on long genuine-far
                        self.f_far_n = 0
                if not far_now:
                    self.f_far_n = 0
            self.f_prev_refresh = F

        # -------- global guards (not during pivots: mid-turn readings are
        # garbage by design, and not in BACKUP/FINISH) --------
        if self.state not in ('TURN', 'BACKUP', 'FINISH') and not self.done:
            if sonar.med['F'] is not None and not sonar.far['F'] \
                    and F < C.FRONT_TOOCLOSE_MM:
                self._enter('BACKUP')
            elif self.chaos >= C.CHAOS_TRIP \
                    and self.state not in ('DECIDE', 'CROSS') \
                    and self.pwm_l > 0 and self.pwm_r > 0:
                self.chaos = 0
                self._enter('BACKUP')
            # stuck: readings frozen while commanded forward in CRUISE.
            # Compared per sensor *refresh*; threshold above sensor noise.
            nearing = (sonar.med['F'] is not None and not sonar.far['F']
                       and F < C.STUCK_FRONT_MM)
            if self.state == 'CRUISE' and nearing:
                meds = (sonar.dist('F'), sonar.dist('L'), sonar.dist('R'))
                moving = self.pwm_l > C.PWM_FLOOR and self.pwm_r > C.PWM_FLOOR
                idx = 'FLR'.index(refreshed)
                if self.last_meds is not None and \
                        abs(meds[idx] - self.last_meds[idx]) > 6.0:
                    self.stuck_ms = 0
                elif moving and not self.brake:
                    self.stuck_ms += C.LOOP_MS
                self.last_meds = meds
                if self.stuck_ms > C.STUCK_MS:
                    self.stuck_ms = 0
                    self._enter('BACKUP')
            else:
                self.stuck_ms = 0
                self.last_meds = None
            # progress watchdog: a wedged robot with NOISY sensors never
            # trips the frozen-readings detector. Coarser net: in CRUISE,
            # commanded forward, front valid & inside 700 mm, yet the front
            # distance stays inside a +-40 mm band for 6 s -> not moving.
            prog_near = (sonar.med['F'] is not None and not sonar.far['F']
                         and F < 700.0)
            if self.state == 'CRUISE' and prog_near \
                    and self.pwm_l > 0 and self.pwm_r > 0 and not self.brake:
                f_ref = getattr(self, 'f_ref', None)
                if f_ref is None or abs(F - f_ref) > 40.0:
                    self.f_ref = F
                    self.prog_ms = 0
                else:
                    self.prog_ms = getattr(self, 'prog_ms', 0) + C.LOOP_MS
                if getattr(self, 'prog_ms', 0) > 6000:
                    self.f_ref = None
                    self.prog_ms = 0
                    self.anomaly.append('no-progress')
                    self._enter('BACKUP')
            else:
                self.f_ref = None
                self.prog_ms = 0

        getattr(self, '_st_' + self.state.lower())(sonar, refreshed, F)
        return self.pwm_l, self.pwm_r, self.brake

    def _enter(self, state):
        self.state = state
        self.timer = 0
        if state in ('DECIDE', 'BACKUP'):
            self.brake = True
            self.pwm_l = self.pwm_r = 0
        if state == 'BACKUP':
            self.last_backup_ms = getattr(self, 'run_ms', 0)
        if state == 'DECIDE':
            self.dec_phase = 0
        if state == 'CROSS':
            self.x_fclear_ms = 0
        if state == 'APPROACH':
            self.appr_far_ms = 0
        if state in ('TURN', 'BACKUP'):
            self.appr_abort_n = 0
        if state == 'TURN':
            self.trims_done = 0
            self.turn_retries = 0
            self.turn_rev_ms = 0
            self.back_trims = 0
            self.back_ms = 0

    # -------- states --------

    def _st_cruise(self, sonar, refreshed, F):
        self.brake = False
        # front wall? (debounced per front *refresh*, not per tick)
        if refreshed == 'F':
            if sonar.med['F'] is not None and not sonar.far['F'] \
                    and F < C.FRONT_BLOCKED_MM:
                self.front_blocked_n += 1
            else:
                self.front_blocked_n = 0
        if self.front_blocked_n >= C.FRONT_DEBOUNCE:
            self.turn_dir = None           # decision not chosen yet
            self._enter('APPROACH')
            return
        nxt = self._next()
        # crossing pending (scripted X, or always in auto mode)? on the
        # both-sides-open signature, take it as an active straight-through.
        # In auto the front must ALSO be open: at a real crossing straight
        # ahead is clear, while at a corner a side mirage plus a nearing
        # front wall would otherwise fake this signature and drive the
        # robot into the corner.
        if self.auto or (nxt and nxt[0] == 'X'):
            f_openish = sonar.far['F'] or sonar.med['F'] is None \
                or F > C.FRONT_FAR_MM
            if (f_openish or not self.auto) \
                    and not sonar.wall('L') and not sonar.wall('R'):
                self.x_open_n += 1
                if self.x_open_n >= 6:
                    self.x_open_n = 0
                    self._enter('CROSS')
                    return
            else:
                self.x_open_n = 0
        # side opening matching the next scripted 'O' maneuver?
        if nxt and nxt[0] == 'O':
            side = nxt[1]
            if self.open_n[side] >= C.OPEN_DEBOUNCE:
                self.script.pop(0)
                self.turn_dir = side
                self.open_n = {'L': 0, 'R': 0}
                self.side_was_wall = {'L': False, 'R': False}
                self._enter('CREEP')
                return
        # finish? Script done AND the robot has demonstrably settled into
        # the final corridor (both walls, sustained) BEFORE everything
        # opens up - junction confusion can never arm the finish watch.
        if not self.script:
            lw, rw = sonar.wall('L'), sonar.wall('R')
            if not self.fin_armed:
                if refreshed in 'LR':
                    if lw and rw:
                        self.fin_wall_n += 1
                        if self.fin_wall_n >= C.FINISH_ARM_N:
                            self.fin_armed = True
                    else:
                        self.fin_wall_n = max(0, self.fin_wall_n - 1)
            else:
                f_open = (sonar.far['F'] or sonar.med['F'] is None
                          or F > C.FRONT_FAR_MM)
                if self.auto:
                    # strict: a yawed robot inside the maze reads 230-450mm
                    # on both sides ("not wall" but not open floor), and a
                    # wedged robot at a junction sees open arms with a
                    # static valid front. The real exit reads FAR. Demand it.
                    sides_far = (sonar.far['L'] or sonar.dist('L') > 500) \
                        and (sonar.far['R'] or sonar.dist('R') > 500)
                    f_far = sonar.far['F'] or sonar.med['F'] is None \
                        or F > 700.0
                    open_now = sides_far and f_far
                else:
                    open_now = not lw and not rw and f_open
                if open_now:
                    self.all_open_n += 1
                else:
                    self.all_open_n = 0
                if self.all_open_n >= C.FINISH_OPEN_TICKS:
                    self._enter('FINISH')
                    return
        both = sonar.wall('L') and sonar.wall('R')
        base = C.PWM_CRUISE if both else C.PWM_SLOW + 20
        # auto mode steers both-walls-only everywhere: near any opening it
        # holds straight on trim instead of chasing the gap (the scripted
        # X behavior, generalized - openings are never turned into)
        x_pend = self.auto or (nxt is not None and nxt[0] == 'X')
        self._slew(*self._steer(sonar, base, refreshed=refreshed,
                                both_only=x_pend))

    def _st_approach(self, sonar, refreshed, F):
        """Committed approach: once armed, creep to the stop line; a mid-
        approach dropout does NOT abort (corner echoes flicker) — only a
        sustained far with no close history, or a VALID far reading."""
        self.brake = False
        if self.timer > C.APPROACH_TIMEOUT_MS:
            self._enter('DECIDE')
            return
        if sonar.far['F']:
            if self.last_valid_f is not None and self.last_valid_f < 120.0:
                self._enter('DECIDE')      # inside the blind zone
                return
            self.appr_far_ms += C.LOOP_MS
            if self.appr_far_ms > C.APPROACH_FAR_MS:
                self.front_blocked_n = 0   # genuine dropout, thing was far
                self.appr_abort_n += 1
                self._enter('BACKUP' if self.appr_abort_n >= 3 else 'CRUISE')
                return
        else:
            self.appr_far_ms = 0
            # arriving yawed: a side about to touch ends the approach NOW
            for sd in 'LR':
                r = sonar.last_raw[sd]
                if r is not None and r < 55.0:
                    self._enter('DECIDE')
                    return
            if F > C.APPROACH_ABORT_MM:
                self.front_blocked_n = 0   # truly receded (weave echo)
                self.appr_abort_n += 1
                self._enter('BACKUP' if self.appr_abort_n >= 3 else 'CRUISE')
                return
            if F <= C.FRONT_STOP_MM:
                self._enter('DECIDE')
                return
        self._slew(*self._steer(sonar, C.PWM_SLOW, gain=0.5,
                                refreshed=refreshed))

    def _st_decide(self, sonar, refreshed, F):
        if self.dec_phase == 0:            # settle at a dead stop
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            if self.timer < C.SETTLE_MS:
                return
            if self.turn_dir is not None:
                # O-turn: verify the opening with a wiggle check before
                # committing (a specular mirage of a smooth wall reads
                # open only until the sensor sweeps past perpendicular)
                self.dec_phase = 3
                self.wiggle_hit = False
                self.timer = 0
                return
            # phantom wall (weave-angle echo)? Only a VALID far-ish reading
            # proves that; far/invalid after a close approach means we are
            # in the blind zone and must back out before pivoting.
            if not sonar.far['F'] and sonar.med['F'] is not None \
                    and F > C.FRONT_STOP_MM + 80:
                self.front_blocked_n = 0
                self._enter('CRUISE')
                return
            if sonar.far['F'] or F < 60.0 or \
                    (self.last_valid_f is not None
                     and self.last_valid_f < 70.0):
                self.dec_phase = 1         # blind-zone escape reverse
                self.timer = 0
                return
            self.dec_phase = 2
        if self.dec_phase == 1:
            if self.timer < C.DECIDE_REVERSE_MS:
                self.brake = False
                self.pwm_l = self.pwm_r = -C.PWM_BACK
                return
            if self.timer < C.DECIDE_REVERSE_MS + C.POST_TURN_SETTLE_MS:
                self.brake = True
                self.pwm_l = self.pwm_r = 0
                return
            self.dec_phase = 2
        if self.dec_phase == 3:
            # wiggle: sweep toward the opening, back past centre, return.
            t = self.timer
            raw = sonar.last_raw[self.turn_dir]
            if raw is not None and raw < C.SIDE_OPEN_MM:
                self.wiggle_hit = True     # a wall answered: mirage
            if t < 480:
                self.brake = False
                seg = 1 if (t < 120 or t >= 360) else -1
                d = 1 if self.turn_dir == 'R' else -1
                self.pwm_l = C.PWM_TURN * seg * d
                self.pwm_r = -C.PWM_TURN * seg * d
                return
            if t < 480 + C.POST_TURN_SETTLE_MS:
                self.brake = True
                self.pwm_l = self.pwm_r = 0
                return
            if self.wiggle_hit:
                if self.auto_wig:
                    # auto corner with both sides reading open: the chosen
                    # side answered with a wall - it was a mirage. Turn the
                    # other way instead.
                    self.auto_wig = False
                    self.turn_dir = 'L' if self.turn_dir == 'R' else 'R'
                    self.dec_phase = 2
                else:
                    # not a real opening: restore the script entry and
                    # demand a fresh wall->open edge before it fires again
                    self.script.insert(0, ('O', self.turn_dir))
                    self.side_was_wall[self.turn_dir] = False
                    self.open_n[self.turn_dir] = 0
                    self.turn_dir = None
                    self.front_blocked_n = 0
                    self._enter('CRUISE')
                    return
            else:
                self.auto_wig = False
            self.dec_phase = 2
        if self.turn_dir is None and self.auto:
            # map-agnostic corner: turn to the side the sensors say is open.
            # Every turn on the official maps is forced (exactly one side
            # open), so this is a read, not a guess.
            l_open = sonar.far['L'] or sonar.med['L'] is None \
                or sonar.dist('L') > C.SIDE_OPEN_MM
            r_open = sonar.far['R'] or sonar.med['R'] is None \
                or sonar.dist('R') > C.SIDE_OPEN_MM
            if l_open != r_open:
                self.empty_blocks = 0
                self.forced_dir = None
                self.turn_dir = 'L' if l_open else 'R'
            elif l_open and r_open:
                # both look open: T-junction (not on these maps) or a
                # specular mirage. A VALID medium echo proves an opening
                # (the sensor sees the next corridor's far wall); silence/
                # far-latch proves nothing - a wall can mirage invisible
                # for seconds. Prefer proof; wiggle only if tied.
                self.empty_blocks = 0
                self.forced_dir = None
                l_val = (not sonar.far['L']) and sonar.med['L'] is not None
                r_val = (not sonar.far['R']) and sonar.med['R'] is not None
                if l_val != r_val:
                    self.turn_dir = 'L' if l_val else 'R'
                    self._enter('TURN')
                    return
                self.turn_dir = 'L' if sonar.dist('L') > sonar.dist('R') \
                    else 'R'
                self.dec_phase = 3
                self.wiggle_hit = False
                self.auto_wig = True
                self.timer = 0
                return
            else:
                # neither side open: dead end (never on these maps) or a
                # phantom front block - realign first, force a turn after 3.
                # Remember the forced direction: if we get blocked again
                # right away, keep turning the SAME way (a committed 180
                # escapes a pocket; flip-flopping on noisy width readings
                # never does).
                self.anomaly.append('auto-no-side-open')
                self.empty_blocks += 1
                if self.empty_blocks < 3:
                    self.front_blocked_n = 0
                    self._enter('BACKUP')
                    return
                self.empty_blocks = 0
                if self.forced_dir is None:
                    self.forced_dir = 'L' \
                        if sonar.dist('L') > sonar.dist('R') else 'R'
                self.turn_dir = self.forced_dir
            self._enter('TURN')
            return
        if self.turn_dir is None:          # front-blocked path (not CREEP)
            nxt = self._next()
            if nxt and nxt[0] == 'X':
                # a front-block while a crossing is pending is a phantom
                # (yawed view of the crossing corner): the straight path is
                # open by the map - drive it
                self.front_blocked_n = 0
                self._enter('CROSS')
                return
            if nxt and nxt[0] == 'F':
                self.script.pop(0)
                self.turn_dir = nxt[1]
            elif nxt is None:
                # script done: the job is straight out of the mouth, so a
                # blocked front is usually a phantom/graze: realign. But a
                # robot that keeps meeting a REAL block is pointed wrong -
                # after 3 realigns it may turn toward the open side.
                self.anomaly.append('unexpected-front-block')
                self.empty_blocks += 1
                if self.empty_blocks < 3:
                    self.front_blocked_n = 0
                    self._enter('BACKUP')
                    return
                self.empty_blocks = 0
                self.turn_dir = 'L' if sonar.dist('L') > sonar.dist('R') \
                    else 'R'
            else:
                # script pending ('O' not yet seen) but front blocked: the
                # opening was probably missed - turn toward the open side
                self.anomaly.append('unexpected-front-block')
                self.turn_dir = 'L' if sonar.dist('L') > sonar.dist('R') else 'R'
        self._enter('TURN')

    def _st_turn(self, sonar, refreshed, F):
        self.brake = False
        dur = C.TURN90_MS_L if self.turn_dir == 'L' else C.TURN90_MS_R
        if self.turn_retries:
            dur = int(dur * 0.6)           # finishing an interrupted pivot
        # over-rotation unwind (auto): pivot BACK until the outer wall
        # appears, then settle and re-verify
        if self.back_ms > 0:
            outer = 'L' if self.turn_dir == 'R' else 'R'
            raw = sonar.last_raw[outer]
            self.back_ms -= C.LOOP_MS
            if (raw is not None and raw < 300.0) or self.back_ms <= 0:
                self.back_ms = 0
                self.timer = dur           # -> settle, then verify again
                return
            p = C.PWM_TURN
            if self.turn_dir == 'L':       # unwind = opposite of the turn
                self.pwm_l, self.pwm_r = p, -p
            else:
                self.pwm_l, self.pwm_r = -p, p
            return
        if self.timer <= dur:
            p = C.PWM_TURN
            if self.turn_dir == 'L':
                self.pwm_l, self.pwm_r = -p, p
            else:
                self.pwm_l, self.pwm_r = p, -p
            return
        # settle, then verify with the front sensor
        if self.timer <= dur + C.POST_TURN_SETTLE_MS:
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            return
        # reverse-retry window (a pivot against a wall loses most of its
        # angle: back off ~straight and finish the same turn)
        if self.timer <= dur + C.POST_TURN_SETTLE_MS + self.turn_rev_ms:
            self.brake = False
            self.pwm_l = self.pwm_r = -C.PWM_BACK
            return
        if self.turn_rev_ms:
            self.turn_rev_ms = 0
            self.timer = 0                 # re-run the (shortened) pivot
            return
        ok = sonar.far['F'] or sonar.med['F'] is None or \
            sonar.dist('F') > C.FRONT_OPEN_MM
        # auto: front-open alone cannot tell a 90 from a 180 (both face
        # corridors). After a true corner turn the OLD front wall must sit
        # on the outboard side. If it doesn't, we over-rotated: unwind
        # until it appears (once per turn).
        if self.auto and ok and self.back_trims < 1:
            outer = 'L' if self.turn_dir == 'R' else 'R'
            outer_wall = (sonar.med[outer] is not None
                          and not sonar.far[outer]
                          and sonar.dist(outer) < 350.0)
            if not outer_wall:
                self.back_trims += 1
                self.back_ms = int(dur * 0.75)
                return
        if not ok and self.trims_done < C.TURN_MAX_TRIMS:
            self.trims_done += 1
            self.timer = dur - C.TURN_TRIM_PULSE_MS   # one more short pulse
            return
        if not ok and self.turn_retries < 2:
            self.turn_retries += 1
            self.trims_done = 0
            self.turn_rev_ms = 450         # back out, then finish the turn
            self.timer = dur + C.POST_TURN_SETTLE_MS
            return
        self.err_prev = 0.0
        self.err_hist = []
        self.deriv = 0.0
        self.front_blocked_n = 0
        self.turn_dir = None
        self.side_was_wall = {'L': False, 'R': False}
        self.open_n = {'L': 0, 'R': 0}
        self._enter('SQUARE')

    def _st_square(self, sonar, refreshed, F):
        """Post-maneuver: brief brake-settle to kill any diagonal momentum,
        then D-only steering squares the heading to the corridor before
        position control re-engages."""
        if self.timer < 140:
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            return
        self.brake = False
        if self.timer >= 140 + C.SQUARE_MS:
            self._enter('REACQUIRE')
            return
        self._slew(*self._steer(sonar, C.PWM_SLOW, refreshed=refreshed,
                                kd_only=True))

    def _st_reacquire(self, sonar, refreshed, F):
        self.brake = False
        if self.timer >= C.REACQUIRE_MS:
            self._enter('CRUISE')
            return
        self._slew(*self._steer(sonar, C.PWM_SLOW, gain=0.9,
                                refreshed=refreshed))

    def _st_cross(self, sonar, refreshed, F):
        """Scripted straight-through crossing: brief settle, then hold a
        straight line at slow speed. Front flicker is ignored (the map says
        straight is open); only the valid too-close and chaos guards watch.
        Ends when both walls are back (or on the hard cap)."""
        if self.timer < C.POST_TURN_SETTLE_MS:
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            return
        self.brake = False
        # auto mode: a "crossing" whose walls never come back, with truly
        # FAR readings all around, is the exit mouth - hand over to FINISH
        # (which bounces back to CRUISE if a wall reappears). Extra guards
        # against a robot wedged at a junction corner (arms read open while
        # it isn't moving): the front must be genuinely far, and a valid
        # front must have CHANGED since crossing entry (proof of motion).
        # echo-free-front clock: a true exit sees NOTHING ahead. A wedged
        # robot's front flickers between valid mid-range and far - any
        # valid raw under 900 mm resets the clock.
        raw_f = sonar.last_raw['F']
        if refreshed == 'F' and raw_f is not None and raw_f < 900.0:
            self.x_fclear_ms = 0
        else:
            self.x_fclear_ms += C.LOOP_MS
        if self.auto and self.timer > 2000 \
                and (sonar.far['L'] or sonar.dist('L') > 500) \
                and (sonar.far['R'] or sonar.dist('R') > 500) \
                and (sonar.far['F'] or sonar.med['F'] is None or F > 700.0):
            # motion proof: if a valid front was seen during this crossing,
            # only a VALID front that has moved >60 mm since then counts -
            # a far-flag now proves nothing (mirages read far too)
            # ... and never within 5 s of a BACKUP: a real exit is reached
            # cruising, not thrashing in a pocket with mirage-far sensors
            calm = getattr(self, 'run_ms', 0) \
                - getattr(self, 'last_backup_ms', -99999) > 5000
            if self.x_fclear_ms > 1000 and calm:
                self._enter('FINISH')
                return
        done = self.timer >= C.CROSS_MAX_MS
        if self.timer > C.CROSS_MIN_MS and sonar.wall('L') and sonar.wall('R'):
            self.x_close_n += 1
            if self.x_close_n >= 4:
                done = True
        else:
            self.x_close_n = 0
        if done:
            if self._next() and self._next()[0] == 'X':
                self.script.pop(0)
            self.x_close_n = 0
            self.side_was_wall = {'L': False, 'R': False}
            self.open_n = {'L': 0, 'R': 0}
            self.front_blocked_n = 0
            self._enter('SQUARE')
            return
        self._slew(*self._steer(sonar, C.PWM_SLOW, gain=1.0,
                                refreshed=refreshed, both_only=True))

    def _st_creep(self, sonar, refreshed, F):
        self.brake = False
        if self.timer >= C.CREEP_MS:
            self._enter('DECIDE')
            return
        # keep only single-wall centering off the side that is still there
        self._slew(*self._steer(sonar, C.PWM_SLOW, gain=0.5,
                                refreshed=refreshed))

    def _st_backup(self, sonar, refreshed, F):
        if self.timer < C.SETTLE_MS:
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            self.esc_dir = 'L' if sonar.dist('L') > sonar.dist('R') else 'R'
            return
        # every third consecutive backup escalates: reverse twice as far,
        # then swing toward the more open side to break out of pockets
        escal = self.backups >= 2 and self.backups % 3 == 2
        dur = C.BACKUP_MS * (2 if escal else 1)
        if self.timer < C.SETTLE_MS + dur:
            self.brake = False
            bias = 6 if (self.backups % 2) else -6
            self.pwm_l = -(C.PWM_BACK + bias)
            self.pwm_r = -(C.PWM_BACK - bias)
            return
        if escal and self.timer < C.SETTLE_MS + dur + 350:
            p = C.PWM_TURN
            if self.esc_dir == 'L':
                self.pwm_l, self.pwm_r = -p, p
            else:
                self.pwm_l, self.pwm_r = p, -p
            return
        self.backups += 1
        self.err_prev = 0.0
        self.err_hist = []
        self.deriv = 0.0
        self.front_blocked_n = 0
        self._enter('SQUARE')

    def _st_finish(self, sonar, refreshed, F):
        self.brake = False
        # a wall came back on either side: crossing or wall-hug, not exit
        if sonar.wall('L') or sonar.wall('R'):
            self.all_open_n = 0
            self._enter('CRUISE')
            return
        # auto: a valid close front during the exit drive means we are NOT
        # driving out an open mouth - bail back to navigation
        if self.auto and not sonar.far['F'] and sonar.med['F'] is not None \
                and F < 300.0:
            self.all_open_n = 0
            self._enter('CRUISE')
            return
        if self.timer >= C.FINISH_DRIVE_MS:
            self.brake = True
            self.pwm_l = self.pwm_r = 0
            self.done = True
            return
        self._slew(C.PWM_SLOW, C.PWM_SLOW + C.TRIM_R_PWM)
