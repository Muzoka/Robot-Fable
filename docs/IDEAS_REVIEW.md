# Review of the Team's Ideas

The attachment opens with three ideas. Short version: **all three are
directionally right**, and each one maps onto a standard, proven technique.
They are adopted — with concrete engineering shape — as follows.

## Idea 1 — "Don't act on a reading until it's real; slow down / stop so
reaction latency doesn't smear the maneuver"

**Verdict: correct, adopted.** This is the classic sense–decide–act latency
problem: with HC-SR04s a full 3-sensor refresh takes ~30–60 ms, and a fast
robot travels 1–3 cm in that window, so acting on stale data at speed turns
into oscillation and overshoot at corners.

Adopted as:
- **Debounced state transitions** — a state change (e.g. "front wall ahead",
  "side opening") requires N consecutive agreeing samples after a
  median-of-3 filter, never a single reading.
- **Speed staging** — full cruise only in a clean corridor; automatic slow
  zone as the front distance drops; **full active-brake stop, ~150 ms
  settle, fresh read** before any turn is committed. Stopping to read
  costs ≈1–2 s per map and buys near-zero risk of a misjudged turn — the
  right trade under completion-first scoring.
- One nuance: we do *not* stop-and-go continuously in straight corridors
  (that would triple run time and add jerk); continuous centering at
  moderate speed with the filter running is reliable there. Stops are for
  *decision points* — which is what the idea was really about.

## Idea 2 — "Use the measurements + real Arduino IDE test feedback to shape
the code around the actual robot"

**Verdict: correct, adopted as the backbone of the whole plan.** The
measurements already taken (wall-to-wall, centered sensor readings per eye,
weight, width, length, sensor heights) directly parameterize the simulator
and the firmware constants — see `HARDWARE_AUDIT.md` for what each number
becomes. The "feedback from the Arduino IDE" part becomes **Phase 3**: a
dedicated calibration sketch with a serial menu; the team runs numbered
tests on the real robot and pastes the log back, and those numbers (motor
deadband, straight trim, pivot timing, sensor noise) replace the assumed
defaults. This closes the loop between the physical robot and the code
without needing any parts we don't have (no encoders, no IMU).

## Idea 3 — "At a turn, stop, re-read, correct yourself; keep a centering
margin between two walls, but be careful at corners/intersections where
the two-walls assumption breaks"

**Verdict: correct, and the caveat is the most valuable part.** Adopted as:
- **Turn procedure**: stop → settle → read → pivot ~90° → stop → read again
  → micro-correct heading (equalize side readings toward their calibrated
  offsets) → re-acquire corridor → resume. Exactly the proposed
  "reset the margin of error at each turn".
- **Centering PID runs only when its assumptions hold**: both side sensors
  must report a valid wall within ~14 cm. The moment either side opens up
  (corner mouth, intersection), centering **freezes** and the robot holds
  its last heading with locked motor trims — this is the guard the idea
  asks for, so the controller never "corrects" toward a gap.
- At the 5-point map's intersection: both walls vanish briefly → the
  scripted route says "straight", so the robot enters hold-heading mode at
  reduced speed until both walls return, then re-centers gently.
- The requested "tell us the centered readings" test exists — the team
  already measured it (L≈8.65 / R≈9.50 cm), and the start-line self-test in
  the firmware re-checks it before every run.

## What the ideas were missing (added by us)

1. **Known-map scripting** — the turn sequence of each map is known, so the
   robot should never *decide* where to go, only *when it is safe* to do
   it. Reactive-only robots lose runs to a single misclassified opening.
2. **Cross-talk-free sensor scheduling** — three HC-SR04s fired together
   hear each other's echoes; they must be fired staggered round-robin.
3. **Stuck recovery** — completion-first scoring means the robot must never
   deadlock: if front distance collapses unexpectedly or readings freeze,
   back off straight, re-center, and resume the script one step back.
4. **Battery discipline** — timed elements (pivot duration) drift with
   voltage; sensor-verified turns plus a fixed charge window at race time
   remove that drift.
