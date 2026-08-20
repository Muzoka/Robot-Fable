# Phase 3 — On-robot calibration protocol

**Time needed: ~30–40 minutes. Do it on the real track floor (or the most
similar floor you have), with the 18650 pack charged into the window you
will race in.** Upload `firmware/calibration/calibration.ino` with the
Arduino IDE, open the Serial Monitor at **115200 baud** (line ending
"Newline"), and run the numbered tests in order. Copy the ENTIRE serial
output (plus your notes where a test asks you to observe something) and
paste it back to Claude — the numbers become the final constants in the
competition firmware.

## Short USB cable? Built in.

The sketch is designed so you NEVER walk with the laptop:

- Keep the **battery slide switch ON** the whole session — the robot
  keeps running when USB is unplugged.
- Motion tests (3, 4, 6, 8) print `UNPLUG USB NOW` and count down 8 s
  (LED double-blinks each second). Unplug, put the robot down, and it
  runs the maneuver on battery, then stops with a slow LED blink.
- Observe/measure, replug USB, reopen the Serial Monitor. The Arduino
  **resets when the monitor opens — that's normal**; every adjustment
  you made (trim, turn times) was already saved to EEPROM and the menu
  prints the saved values each time.
- Stationary tests (1, 2, 5, 7, 9) run with the cable connected; the
  deadband test uses short pulses so the robot only creeps — keep it
  next to the laptop and nudge it back between pulses.

## Battery policy (decided 2026-08-20)

The team has 2×3800 mAh and 2×5000 mAh "Lucky Sky" 18650 cells.

- **Race pair = the two 3800 mAh cells** — the pair the drift test already
  ran on. The 5000 pair is the emergency backup (note it isn't internally
  matched: one prints 3.6 V, one 3.7 V).
- **Never mix a 3800 with a 5000 in the holder** — series cells must be a
  matched pair or one over-drains and the pack collapses mid-run.
- **Calibrate and race on the same pair at the same charge level.** Turn
  timing follows battery voltage; the numbers belong to "these two cells,
  freshly charged". Charge fully before the calibration session and again
  before the competition.
- Optional tiebreaker: run test 8 once per fully-charged pair and time
  the marked metre; the faster, more consistent pair sags less — use it.

## Collecting the results for Claude

The Serial Monitor does NOT keep text across an unplug/reopen — but your
adjustments live in the robot's EEPROM, and unplugged-run results are
physical observations. So: keep a notes file open; AFTER each test (before
the next unplug) copy the serial text into it and add your measurement in
plain words ("drifted RIGHT ~5 cm", "4 pivots ended ~25° short, CCW").
Send the whole file to Claude at the end, or in parts. Screen photos are
fine whenever copying is awkward.

## Before you start (one-time hardware checklist)

1. ✅ D12 → L298N "5V" wire — **already removed by the team** (confirmed).
2. Remove the small jumper caps on L298N ENA/ENB if still fitted; keep
   the 12V-regulator jumper ON; nothing connected to the 5V terminal.
3. Secure the wiring: bundle the jumper loops flat against the chassis,
   keep every wire out of the three sensors' view cones, hot-glue or
   tape each DuPont connector shell.
4. Fresh-charged 18650s, slide switch ON, USB connected for serial.

## If a wheel spins on its own at power-up

A 1–2 s twitch at plug-in is normal (pins float until the program starts).
Continuous spinning means the motor pins aren't being driven: usually the
calibration sketch isn't uploaded yet, or the ENA/ENB jumper caps are
still on the L298N (they force motors enabled - pull them off), or a
direction wire (D4/D7 left, D8/D11 right) slipped out. Nothing is damaged
by a free-spinning wheel. After uploading the sketch both wheels must sit
silent at the menu; test 1 then verifies each motor properly.

## The tests

| # | Test | Robot placement | You report |
|---|---|---|---|
| 1 | Motor direction | on a box, wheels free | did each wheel spin forward? |
| 2 | Deadband | on the floor, next to the laptop | PWM at which each wheel first moves |
| 3 | Straight trim | 2 m clear floor | final TRIM_R after a/d tuning |
| 4 | Pivot timing | open floor, mark heading | final L/R ms; ×4-pivot error in degrees |
| 5 | Sensor stats | centred in a corridor, square | full printout + ruler distance of each wheel to its wall |
| 6 | Brake distance | 2 m clear floor | brake vs coast stop distance (cm) |
| 7 | Live stream | hand-held sweep | note anything weird (a sensor reading a wire, etc.) |
| 8 | Cruise speed | 2 m, a marked metre | seconds to cross the metre |
| 9 | Saved values | anywhere | (shows/reset the EEPROM-saved numbers) |

Tips for test 4 (the most important one): use the `e` (right) / `q`
(left) keys — four pivots in a row should bring the robot back exactly
to its starting heading; whatever angle it is off by, divide by 4 and
tell me the number and direction. Adjust with `d`/`c` (right ±10 ms) and
`a`/`z` (left ±10 ms) — saved instantly — and rerun until the ×4 error
is under ~10° total, for LEFT and RIGHT separately.

Repeat tests 3 and 4 once more after the battery has done ~10 minutes of
driving — I want both the fresh-charge and the settled-charge numbers.

## What happens with the results

I fold them into `firmware/competition/competition.ino` (PWM_FLOOR, TRIM_R_PWM,
TURN90_MS_L/R, possibly PWM_CRUISE and the sensor targets), re-run the
simulator with the measured values, and deliver the frozen race build
plus the race-day checklist.
