# Calibration tests — reference manual

How to run every test in `firmware/calibration/calibration.ino`, what
each one measures, what "good" looks like on THIS robot, and when to
re-run which test. Written to be usable without Claude in the loop.

## Setup (every session)

1. Battery slide switch **ON** for the whole session (the robot must
   survive USB unplugs). Use the **3800 mAh pair** — never mix a 3800
   with a 5000 in the holder.
2. Upload `firmware/calibration/calibration.ino` (Arduino IDE, board
   "Arduino Uno").
3. Serial Monitor: **115200 baud**, line ending **Newline**.
4. The menu prints the saved values every time it appears:
   `saved: TRIM_R=…  TURN_MS_L=…  TURN_MS_R=…`. These live in the
   robot's EEPROM — they survive resets, re-uploads, and power-off.

**Batch typing trick:** the input line can take many keys at once.
Typing `zzzzzzzzzz` + Enter applies ten `z` presses one at a time,
printing the value after each. Use it for big adjustments.

**The unplug rhythm** (tests 3, 4, 6, 8 — anything that drives):

1. Press the action key → robot prints `UNPLUG USB NOW`, LED
   double-blinks for 8 seconds.
2. Unplug, put the robot down where the test needs it, stand clear.
3. It runs the maneuver on battery, then stops with a slow LED blink.
4. Replug, reopen the Serial Monitor. The board resets to the menu —
   normal; nothing is lost (see EEPROM note above).

## The tests

### 0 — Enable-wire check · robot on a box, wheels free · cable in

Verifies the Arduino actually controls motor SPEED (not just
direction). Direction pins go active with speed ZERO for 2 s — **both
wheels must stay still** — then speed 140 — **both wheels spin at
medium speed**.

*If a wheel moves during the ZERO phase:* the ENA/ENB jumper caps are
on the L298N (pull them off), or an enable wire is on the wrong pin.
D5 → ENA signal pin (leftmost of the freed pair), D6 → ENB signal pin;
the +5V partner pins stay EMPTY.
*If nothing spins at 140:* an enable wire is disconnected.

### 1 — Motor direction · box, wheels free · cable in

LEFT wheel drives "forward" for 1 s, then RIGHT. For each, judge: would
that spin push the robot forward? If one is backwards, don't rewire —
flip `MOTOR_L_INVERT` / `MOTOR_R_INVERT` in both sketches.

**This robot:** both correct with the current wiring (verified
2026-08-20).

### 2 — Deadband · on the floor next to the laptop · cable in

Finds the PWM range where the motors buzz but the robot doesn't move.
Two sub-tests from the test-2 menu:

- **`r` breakaway ramp:** PWM 90 → 200 in 0.5 s pulses. Write down the
  PWM where the robot FIRST moves, and which side moved first. Nudge
  the robot back between pulses.
- **`k` kick test:** each step fires 0.09 s at full power (a short
  lunge — intended), then holds a lower PWM for 0.8 s, stepping DOWN
  190 → 110. Write down the lowest hold PWM where it KEPT rolling
  after the kick. Give the USB cable slack.

**This robot (2026-08-20, on floor):** breakaway L ≈ 125–140,
R ≈ 115–125 (run-to-run spread ±15 is normal — battery and friction);
kick-sustain floor ≈ 110–130. The race firmware's speeds are built on
these numbers (floor 130, cruise 180, turns 200) — if a future
measurement moves by more than ~20, the firmware constants should be
revisited.

### 3 — Straight trim · ~2 m clear floor · UNPLUG runs

Balances the two motors so the robot drives straight with no walls.
The value is `TRIM_R` — extra PWM added to the RIGHT motor (negative =
right motor slowed).

- `g` = drive 2 s at cruise (unplug rhythm).
- After the run: `d` if it drifted RIGHT, `a` if it drifted LEFT
  (one press ≈ 1 PWM count ≈ subtle; a few presses per clear drift).
- `s` = back to menu. Every press saves instantly.

Rules of thumb:
- **Align the start:** launch the robot against a straight reference
  (tape line, floorboard). A 2° crooked launch looks exactly like
  drift.
- **Compare only back-to-back runs** — drift changes on its own as the
  battery drains.
- **Sanity check if trim seems dead:** batch-type `a`×20 (TRIM ≈ −40)
  → must arc clearly RIGHT; then `d`×40 (TRIM ≈ +40) → must arc
  clearly LEFT. If the extremes do nothing, the right enable wire is
  loose — re-run test 0. Set it back near the balanced value after.
- **Good enough is good enough:** the race firmware steers off the
  walls and self-calibrates centering at the start line; trim alone
  only carries the ~30 cm of the open crossing. Within ±4 counts of
  true is fine.

**This robot:** ≈ 0 on a fresh-ish pack, drifted to −13 late in a long
session (see "left channel" below).

### 4 — Pivot timing · open floor, tape mark for the nose · UNPLUG runs

**The most important test.** Calibrates the duration of the timed 90°
pivots (`TURN_MS_L` / `TURN_MS_R`).

- `e` = FOUR right pivots in a row (unplug rhythm). Four perfect 90s
  end with the nose exactly back on the tape mark. The error you see,
  divided by 4, is the per-turn error.
- `d` / `c` = right turn +10 / −10 ms. Each 10 ms ≈ 3–4°.
  Turned too little → `d`; too far → `c`.
- `q` and `a` / `z` = the same for LEFT.
- `E` / `Q` = single pivots for a quick look.
- Target: ×4 run ends within ~10° of the mark, each side.

**This robot:** fresh-pack values land around **250–270 ms right,
270–290 ms left**; on a tired pack they grew to ~300/320 (+15–20%).
That growth is battery sag and is why these numbers are re-measured on
race morning with the freshly charged pack — the tuned values feed the
race firmware automatically (see "How the values reach the race
firmware" below).

### 5 — Sensor statistics · robot stationary, centred in a corridor · cable in

100 samples per sensor; prints mean / spread (sd) / min / max /
misses. Place the robot centred and square between two walls
(~30 cm apart — real track if available), then also measure with a
ruler: each wheel's outer edge to its wall.

**Pass:** sd under ~3 mm on the sides, misses 0–2. A big sd or many
misses = something in a beam (wire!) or a loose sensor.
**This robot's baseline:** LEFT 87.9 mm (sd 1.4), RIGHT 90.3 mm
(sd 0.9), FRONT sd 3.9, misses 0/0/0, at a true 77 mm per side — the
side sensors read ~11–13 mm above true. The race firmware re-derives
its centering targets at every start line, so this test is a health
check, not a required calibration.

### 6 — Brake distance · ~2.5 m clear floor · UNPLUG run

One run, two halves: drives 1.5 s then hard-brakes (LED ON at the brake
moment) — mark the stop; pauses; drives again and coasts to a stop —
mark that too. Report both distances. Optional — the race firmware's
margins already assume a few cm of coast.

### 7 — Live stream · hand-held · cable in

Streams `F= L= R=` continuously. Sweep the robot around: point each
sensor at far things, near things, your hand at ~10 cm. You're looking
for anomalies only: a sensor stuck on one value, constant misses, or a
short reading with nothing there (= a wire drooping into the beam).
Any key stops it.

### 8 — Cruise run · 2 m, two tape lines 1 m apart · UNPLUG run

Drives 3 s at race cruise; time the marked metre with a stopwatch.
Health check for overall speed (expect roughly 2.5–3.5 s/m on this
drivetrain at cruise 180); also a good way to compare battery packs.

### 9 — Show/reset saved values

Shows the EEPROM values. **The reset option wipes your tuning back to
factory defaults (680 ms turns!) — never use it casually.** If it is
used by accident, re-run tests 3 and 4.

### w — Wiggle test (left-channel hunter) · box, wheels free · cable in

Both motors run at cruise for 20 s. While they run, wiggle each
LEFT-channel connection ONE AT A TIME and watch/listen to the left
wheel: the OUT1/OUT2 screw-terminal wires, the D4 / D7 / D5 jumpers at
both ends, and press gently on the left motor's two solder tabs. Any
stutter, speed change, or stop names the faulty connection — re-seat
or re-tighten it, then hot-glue/tape it.

**Why this exists:** during the 2026-08-20 session the left channel
(a) raised its breakaway 125→140 between runs, (b) once ignored a
full-power kick entirely and recovered on handling, and (c) drifted
the trim from 0 to −13. That pattern is an intermittent
high-resistance joint. Until one is found and fixed, treat left-motor
weirdness as wiring, not calibration.

## When to re-run what

| Situation | Run |
|---|---|
| **Race morning** (fresh pack, venue floor — ~10 min) | 3 (until straight), 4 (×4 both sides), then upload the race sketch and check its boot line says `cal: EEPROM` with your numbers |
| Swapped battery pack | 3 and 4 (the numbers belong to the pack) |
| Any wiring touched / robot dropped | 0, 1, 7, w |
| Robot curves in corridors | 3 |
| Turns over/under-rotate consistently | 4 |
| Robot refuses to start (blink code 6) | 7 — look for a wire in a side sensor's beam |
| A wheel dead or weak | w, then 0 |
| New floor surface for practice | 2 (`r` only) — breakaway shifts with grip |

## How the values reach the race firmware

`TRIM_R`, `TURN_MS_L`, `TURN_MS_R` are saved to EEPROM the moment you
adjust them. The race sketch (`firmware/competition/competition.ino`)
**reads the same EEPROM slots at boot** — so tuning in the calibration
sketch and then uploading the race sketch transfers the numbers with no
code editing. The race sketch's Serial Monitor boot line confirms it:
`cal: EEPROM trimR=… turnL=… turnR=…`. If it prints `DEFAULTS`, the
stored values were implausible (e.g. after a test-9 reset) — redo
tests 3 and 4. Uploading sketches never erases EEPROM.

Everything else the robot needs (speeds, sensor targets) is either a
compiled constant built from the test-2/5 measurements above or
self-calibrated at the start line.
