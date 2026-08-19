# Master Plan — Winning the Track Competition

Scoring priority (from the team): **completion of all maps > fewest wall
touches > time**. Every decision below follows from that ordering.

## Strategy in one paragraph

The three maps are **known in advance**, small, and built from identical
30.5 cm corridors with 90° corners. The winning approach is a **hybrid
controller**: reactive wall-centering (PID on left/right ultrasonic
distances, with per-sensor calibration offsets) for staying safely in the
middle of every corridor, plus a **scripted route** per map (the known
sequence of turns) that tells the robot which way to turn at each decision
point. Reactive control gives zero wall touches; the script removes all
guesswork at corners and the intersection. Every maneuver is
**stop → settle → read → act → verify**, which directly implements the
team's own ideas 1 and 3. Speed is deliberately moderate: time is only the
*second* tie-breaker, and a crash or a missed turn costs infinitely more
than five slow seconds.

## Phases

### Phase 0 — Understand, audit, plan  ✅ (this document)
- Read every attachment (ideas, measurements, wiring, maps, parts). Done.
- Evaluate the team's ideas → `IDEAS_REVIEW.md`.
- Audit the wiring → `HARDWARE_AUDIT.md`. **Contains required rewires —
  read it first, one connection (D12 → L298N 5V terminal) must be removed
  before the next power-on.**
- Research prior art and component facts, adversarially verified →
  `RESEARCH.md`.
- Derive track geometry and route hypotheses → `TRACK_NOTES.md`.
- Open questions for the team (see bottom of this file).

### Phase 1 — Python simulator, control-law tuning
- Digitize the three maps into 2D wall-segment models (30.5 cm corridor
  modules; exact segment lengths refined once the team confirms them).
- Simulate the differential drive (410 g, 15.7 cm track width, TT-motor
  speed model with left/right mismatch and battery sag) and the HC-SR04s
  (beam cone, specular dropout at oblique incidence, 30–60 ms update,
  noise, lost-echo timeouts).
- Port the exact control logic that will run on the Arduino; tune gains and
  thresholds by Monte-Carlo over start-pose error, sensor noise, motor
  mismatch and battery level until **100 % completion with zero wall
  contact** across all perturbations, on all three maps.
- Output: `sim/` package, animated traces, and a tuned constants block that
  is copy-pasted into the firmware.

### Phase 2 — Arduino firmware + Wokwi verification
- `firmware/competition/` — the race sketch:
  - Staggered HC-SR04 firing (no cross-talk), median-of-3 filtering,
    timeout ⇒ "opening" classification.
  - Wall-centering PD controller using the calibrated offsets (centered ≠
    equal readings on this robot: right reads ~0.85 cm more than left).
  - State machine: CRUISE → APPROACH (slow when front wall nears) → TURN
    (stop, settle, pivot, verify, correct) → REACQUIRE → CRUISE; special
    INTERSECTION state (hold heading straight while both walls vanish);
    FINISH (drive out past the final opening and stop).
  - Scripted per-map turn sequence; map selected with jumper wires on the
    two free pins (D2/D3) so **no re-upload is needed between maps**.
  - Stuck/crash-avoidance recovery: if the front distance collapses or
    readings freeze, stop, back off straight, re-center, resume script.
  - Motor layer: PWM trim for straight driving, deadband floor, soft-start
    ramps, active-brake stops (no coasting into walls).
- `firmware/calibration/` — serial-menu test sketch the team runs on the
  real robot (Phase 3 protocol): sensor statistics at known distances, PWM
  deadband finder, straight-line trim finder, 90° pivot timing at race
  voltage, brake-distance check.
- `wokwi/` — Wokwi project (diagram + scripted sensor scenarios) proving
  the state machine logic end-to-end before it ever touches the robot.

### Phase 3 — On-robot calibration (team executes, ~30 min)
A written, numbered protocol; the team pastes back the serial output:
1. Sensor truth check: robot centered at start line → report L/F/R stats.
2. PWM deadband per motor; straight-line trim over 2 m.
3. 90° pivot time left and right at race PWM, at full charge.
4. Brake/coast distance from cruise speed.
5. Repeat key items at a half-discharged battery to measure sag slope.

### Phase 4 — Final firmware + race-day pack
- Fold calibration numbers in, re-run the simulator with measured values,
  freeze the code, tag it.
- Race-day checklist: charge policy (always race in the same battery
  window), start-line placement ritual, pre-run sensor self-test (the
  robot refuses to start and blinks the LED if a sensor disagrees with the
  known start-box geometry), per-map jumper setting, what to do between
  runs.

## What I need from the team (answer whenever ready)

1. **Routes**: on each map photo, draw the exact required path
   (start pad → finish pad, every corridor in order). Especially: the
   5-point map's intersection behavior ("go straight through" — confirm)
   and the full 9-point route. Also: are the striped (zebra) pads
   start/finish only, or checkpoints too?
2. **Rules**: is there a time limit per map? Is a wall *touch* penalized or
   only a hard crash? Are restarts allowed and how are they scored? May we
   re-upload code / change jumpers between maps (three separate runs)?
3. **Wiring confirmations** (see `HARDWARE_AUDIT.md`): a close-up photo of
   the L298N end of the wiring, and confirm what D4 connects to and
   whether the L298N's small ENA/ENB jumper caps are removed.
4. **Two more measurements**: (a) distance from the wheel axle to the
   robot's front-most point and to its rear-most point (decides pivot
   clearance); (b) wall height, and the length of one corridor segment on
   any map (wall-to-wall inner length of, e.g., the 3-point map's top
   corridor) so the simulator's maps are to scale.
5. **A photo of the assembled robot** (top view + front view). The PDF has
   the stock part photos but not the built robot.
6. Which battery races: the 2× 18650 pack (recommended, see audit) — and
   do you have a charger so runs always start in the same voltage window?
7. **`github.com/Muzoka/Robotic-comp`** (same GitHub account as this repo)
   is a prior firmware for what looks like this exact competition — is it
   your team's earlier build? It assumed an MPU-6050 gyro and two wheel
   encoders: do you still have those parts, and are additions allowed? (We
   plan for the current no-gyro/no-encoder hardware regardless; a gyro
   would only make turns even more repeatable.)
8. Are the side sensors mounted level with the wheel axle, ahead of it, or
   behind it (roughly how many cm)? Prior measurement showed axle-line
   mounting is worth ~20 percentage points of completion; if ours sit far
   forward, remounting is a cheap win (the sensor holders allow it).

## Deliverable flow

Each phase lands as commits on this branch with a summary; say **continue**
to advance to the next phase. Phase 1 (simulator) and Phase 2 (firmware)
don't block on the team's answers — defaults are assumed and marked, and
answers refine constants, not architecture.
