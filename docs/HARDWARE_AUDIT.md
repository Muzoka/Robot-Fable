# Hardware & Wiring Audit

Source: the team's wiring list and measurements (attachment pages 3–4).
Verdict summary: the build is fundamentally sound — right parts, right
topology, common ground done correctly — but there are **two wiring
problems to fix** and a few confirmations needed. Firmware pinout will be
`#define`-based, so any wiring change is a one-line code change.

## ⚠ Fix 1 (do before next power-on): remove D12 → L298N "5V slot"

The list has `D12 ——— 5V slot in L298N`. The L298N board's 5V terminal is
**not a logic input**. With the onboard-regulator jumper installed (the
usual state), that terminal is the **output** of the board's own 78M05 5 V
regulator. Driving it from an Arduino GPIO pits the pin against the
regulator: best case it corrupts the board's logic supply, worst case it
burns pin D12 or the regulator.

**Action: pull that wire out entirely and leave the L298N 5V terminal
empty.** Correct power config for this robot:

- 18650 pack + → L298N `12V` terminal (7.4–8.4 V) ✓ (already done)
- Onboard 5 V-regulator jumper: **installed** (board powers its own logic)
- L298N `5V` terminal: **nothing connected**
- 18650 pack + → Arduino `VIN` ✓ (already done; Uno's regulator handles 7–12 V)
- Pack − → L298N GND + Arduino GND + sensor GND rail ✓ (already done — good)

## Fix 2 resolved: the L298N pin list is almost certainly fine as built

The team's wiring list has misaligned lines:

```
D4  ——— L298N            ← no label
IN1 D5 ——— L298N         ← label on the wrong line
ENA D6 ——— L298N ENB     ← two labels, one pin
```

Read literally, ENA would sit on D4 (which has no PWM — that would make
speed control on the right motor impossible). But the prior firmware for
this same competition under the same GitHub account
(`Muzoka/Robotic-comp`, see `docs/RESEARCH.md` §0) uses **exactly the same
pins for everything else** and resolves the ambiguity naturally — the list
is the same map with the labels slid one line:

**Team-confirmed (2026-08-19):** D4→IN1, D5→ENA, D6→ENB, and the motor
sides are LEFT on OUT1/OUT2, RIGHT on OUT3/OUT4. Final map:

| Arduino pin | L298N | Role |
|---|---|---|
| D4 | IN1 | **left** motor direction |
| D5 (PWM) | ENA | **left** motor speed |
| D6 (PWM) | ENB | **right** motor speed |
| D7 | IN2 | left motor direction |
| D8 | IN3 | right motor direction |
| D11 | IN4 | right motor direction |
| D12 | — | **disconnected** (was the 5V-terminal wire — Fix 1) |

Nothing needs rewiring except removing the D12 wire. Note the sides are
swapped relative to the Robotic-comp pin map (there ENA drove the right
motor) — the firmware `#define`s carry the correct mapping.

Also team-confirmed: the 18650 battery socket has a slide power switch,
and the drift test (6 cm right / 150 cm) was run on the 2×3800 mAh cells
with the Arduino OFF — i.e., at full raw pack voltage with no PWM. The
mismatch ratio at our actual cruise PWM may differ (it is not constant
across PWM), which is exactly why the Phase-3 trim calibration runs at the
cruise PWM the robot will race at.

Also **remove the two small jumper caps on ENA and ENB** if still present —
they tie the enables to 5 V (permanent full speed) and would fight the
Arduino.

Sensors stay exactly as wired:

| Arduino pin | Signal |
|---|---|
| D9 / D10 | RIGHT trig / echo |
| A0 / A1 | FRONT trig / echo |
| A2 / A3 | LEFT trig / echo |
| 5V | all three HC-SR04 VCC (3 × ~15 mA — fine) |

Free pins after the fix: **D2, D3** (map-select jumpers to GND, read with
internal pull-ups — pick map 1/2/3 without re-uploading), **D12** (spare —
optionally a GO button to GND, though the firmware's hand-over-the-front-
sensor arming needs no button at all), **D13** (onboard LED = status/
self-test blink codes).

## Power & motor numbers the design uses

- 2S li-ion: 8.4 V full → ~7.4 V nominal → 6.0 V empty. The L298N is an
  old Darlington driver and drops ~1.8–2.6 V total, so the TT motors see
  roughly **5–6.5 V** — exactly their happy range (3–6 V rating). This is
  why the sag matters: fixed-PWM speeds drift ~10–15 % across a discharge,
  which is why turns will be **sensor-verified, not purely timed**, and why
  race runs should always start in the same charge window.
- **Do not race on the 9 V PP3 (Energizer) battery.** A PP3 alkaline has
  high internal resistance; two TT motors pulling a surge amp will sag it
  below the Uno's brownout and reset the board mid-run. Keep it as a bench
  spare for logic-only tests at most. The 18650 pack is the race supply.
- TT motors (1:48): ~150–250 mA no-load, up to ~1–1.5 A stalled at these
  voltages; the L298N handles two of them, but sustained stalls (robot
  pressing a wall) heat it — another reason the firmware backs off instead
  of pushing.
- 410 g total on smooth painted wood: expect a few cm of coasting from
  cruise speed with a passive stop — the firmware uses **active braking**
  (both IN pins low/high with enable on) before every read-and-decide, so
  decisions happen from a true standstill (the team's Idea 1).

## Sensor geometry (from the team's measurements)

Wall-to-wall 30.5 cm; robot 15.7 cm wide → 7.4 cm nominal clearance/side.

Centered-robot readings (team-measured):
- LEFT sensor: fore eye 8.6 cm, back eye 8.7 cm → ≈ **8.65 cm when centered**
- RIGHT sensor: fore eye 9.6 cm, back eye 9.4 cm → ≈ **9.50 cm when centered**

Two important consequences:
1. **Centered ≠ equal readings.** The centering target is
   `right − left ≈ 0.85 cm`, not zero. The firmware bakes this in as a
   calibration constant (`CENTER_OFFSET_CM`), which is exactly the team's
   Idea 3 "margin" made concrete.
2. The fore/back eye differences (0.2 cm right, 0.1 cm left) mean each
   sensor is mounted with ~1–3° of yaw skew — small, and absorbed by the
   same offsets; no remounting needed.

Sensor heights (sides 8.8 cm, front 5.4 cm) are fine against these walls;
just confirm nothing on the chassis (wires, breadboard edge, caster mount)
sits in front of any transducer — a jumper wire drooping into the beam is
the classic source of phantom 3 cm readings.

## Second measurement round (2026-08-19, photos + chat)

- Chassis plate width 9.8 cm; tire width 2.6 cm each → 9.8 + 2×2.6 +
  mounting gaps ≈ the 15.7 cm overall width ✓ consistent.
- **Wheel diameter 6.2 cm** (drives all speed/turn math).
- **Layout from photos: caster at the FRONT (under the low-mounted front
  sensor), drive wheels at the REAR half.** Pivots therefore swing the nose
  through the larger arc — modeled in the simulator; axle-to-front distance
  still to be measured (photo estimate ~9–10 cm).
- Side sensors sit on acrylic holders roughly over the wheel axle — the
  favorable position. Exact fore/aft offset to be confirmed.
- Drift quantified: 6 cm right over 150 cm (2.29°). That is a ~0.7 %
  wheel-speed mismatch (a ~19 m-radius arc) — small; a few PWM counts of
  trim plus the centering PID cover it.
- Wiring loom risk (photos): long jumper loops arcing above the robot,
  secured by two zip ties. Action before race day: shorten/bundle flat to
  the chassis, keep all wires out of the three sensors' view cones, and
  hot-glue/tape every DuPont connector shell. No rewiring — just securing.
- Open questions from the photos: the under-chassis 2×18650 holder appears
  empty and a loose DC barrel plug is visible — confirm which battery
  actually powers the robot in runs and where the barrel plug goes; the
  yellow-taped bundle on the top deck looks like it may be the cell pack.

## Status updates (2026-08-20)

- ✅ **D12 → L298N 5V wire removed by the team** — Fix 1 is DONE.
- ✅ ENA→~D5 (left speed) and ENB→~D6 (right speed) re-confirmed by the
  team; matches the firmware pinout exactly.
- ✅ **ENA/ENB jumper caps found and removed** during the calibration
  session (they had been hidden under the wire loom; symptoms: full-power
  lunges regardless of commanded PWM, wheel spinning at power-on).
  D5/D6 wires re-seated onto the freed ENA/ENB signal pins; the +5V
  partner pins left empty.
- ✅ **Calibration test 0 (enable-wire check) PASSES**: both wheels still
  at speed-zero with direction pins active, medium spin at PWM 140.
  This is the robot's first verified real speed control — all earlier
  driving (including the original drift test) was effectively full-power
  only, so pre-session drive observations are superseded by the
  calibration measurements.
- ✅ Motor directions verified correct (test 1): no inversion needed.
- ✅ **Test 2 (deadband, on floor, measured 2026-08-20):**
  - Breakaway ramp: RIGHT wheel first moved at **PWM 115**, LEFT at
    **PWM 125**. Right in the expected band for TT+L298N at this weight —
    hardware healthy. (The original firmware draft's CRUISE 110 would not
    have moved the robot at all; all PWM constants shift upward.)
  - Kick-and-hold (150→60 range): no hold PWM clearly sustained rolling;
    150 "kinda pushed", right side stronger than left. Sustain floor is
    evidently ≥ ~150 → re-testing with holds 190→110.
  - Note the RIGHT motor is the stronger one at low PWM, opposite to the
    old full-power right-drift observation (which predates real speed
    control — caps-on data is superseded). Trim (test 3) measures the
    real mismatch at the new cruise PWM 180.
  - Calibration sketch constants raised: PWM_CRUISE 110→**180**,
    PWM_TURN 120→**200** (pending confirmation by tests 3/4/8).
  - **Second run (after a left-channel dropout):** during the first
    190→110 kick test the LEFT wheel did not move at all — even at the
    255 kick — while RIGHT sustained down to ~110. After the team checked
    the left wiring it recovered fully. ⚠ Cause not identified: if no
    specific loose wire was found, treat the left motor circuit as having
    an intermittent connection until proven otherwise (wiggle-test before
    race day, hot-glue every left-channel connector: OUT1/OUT2 screws,
    D4/D7/D5 jumpers, motor solder tabs).
  - **Consolidated deadband numbers (floor, this battery charge):**
    breakaway L ≈ 125–140, R ≈ 115–125 (run-to-run spread ~10–15 PWM —
    battery drain + friction variance, expected); kick-sustain: left kept
    rolling until the ~110 hold, right until ~120. Working floor for the
    race firmware: sustain ≈ 130 worst-case, static breakaway ≈ 140
    worst-case → PWM_FLOOR ~150, CRUISE 180, SLOW ~160, TURN 200.
- ✅ **Test 5 (sensor stats, 100 samples/sensor, ruler-verified):** robot
  centred with both wheels 77 mm true from their walls: FRONT mean 359.7
  sd 3.9 (one 326 outlier — median filter's job), LEFT mean 87.9 sd 1.4,
  RIGHT mean 90.3 sd 0.9, **0 misses on all three**. Mount offsets:
  left reads true+10.9 mm, right reads true+13.3 mm; centred differential
  right−left = **+2.4 mm** (supersedes the earlier hand-measured ~8.5 mm).
  Firmware targets updated: TARGET_L 85.0, TARGET_R 87.5, SIDE_SUM 172.5
  (offsets applied to the real track's 74 mm centred clearance).
- ✅ **Test 3 (straight trim @ cruise 180): TRIM_R = 0** — one `a` press
  from the old value of 1, then drives straight. The motors are
  essentially matched at cruise PWM; the old full-power drift does not
  appear at 180.
- ⏳ **Test 4 (pivot timing @ PWM_TURN 200), in progress:** right pivot
  tuned to **TURN_MS_R = 250 ms** (~90° by eye, tuned down from 680).
  LEFT still at the 680 default — must come down to ~250 before any left
  pivot is run. ×4-accuracy verification pending for both directions;
  fresh- vs tired-battery repeat pending. Final TURN90 constants for the
  race firmware wait on that.

## Team-reported field observations

- **The robot drifts slightly to the RIGHT when driven straight open-loop**
  (equal power, just powered up and told to go forward). Meaning: the left
  motor is effectively a bit faster than the right at the same PWM — the
  expected ±10 % TT-motor tolerance showing up. Handled three ways, in
  layers: (1) a `TRIM_RIGHT_PWM` constant (a few extra PWM counts to the
  right motor) calibrated in Phase 3 at the exact cruise PWM — this removes
  the bias; (2) the wall-centering PID absorbs whatever bias remains while
  walls are visible; (3) in wall-less stretches (the intersection) the
  robot drives with the calibrated trim applied, which is why measuring it
  accurately matters — trim quality is what carries the robot straight
  through the crossing. Note the trim is *not* a constant ratio across PWM
  values or battery voltages, so it gets measured at cruise PWM and race
  charge, not once at full throttle.

## Confirmations requested from the team

1. Close-up photo of the L298N with wires attached (settles D4/ENA/ENB).
2. Confirm the ENA/ENB jumper caps are off, the 5 V-regulator jumper is on.
3. Confirm which OUT pair drives which physical side and the forward
   direction of each motor (the calibration sketch will verify this too).
4. Axle-to-front and axle-to-rear distances (pivot sweep clearance check).
