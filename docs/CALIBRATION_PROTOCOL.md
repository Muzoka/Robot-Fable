# Phase 3 — On-robot calibration protocol

**Time needed: ~30–40 minutes. Do it on the real track floor (or the most
similar floor you have), with the 18650 pack charged into the window you
will race in.** Upload `firmware/calibration/calibration.ino`, open the
Serial Monitor at **115200 baud**, and run the numbered tests in order.
Copy the ENTIRE serial output (plus your notes where a test asks you to
observe something) and paste it back to Claude — the numbers become the
final constants in the competition firmware.

## Before you start (one-time hardware checklist)

1. ⚠ **Remove the D12 → L298N "5V" wire** (docs/HARDWARE_AUDIT.md Fix 1).
2. Remove the small jumper caps on L298N ENA/ENB if still fitted; keep
   the 12V-regulator jumper ON; nothing connected to the 5V terminal.
3. Secure the wiring: bundle the jumper loops flat against the chassis,
   keep every wire out of the three sensors' view cones, hot-glue or
   tape each DuPont connector shell.
4. Fresh-charged 18650s, slide switch ON, USB connected for serial.

## The tests

| # | Test | Robot placement | You report |
|---|---|---|---|
| 1 | Motor direction | on a box, wheels free | did each wheel spin forward? |
| 2 | Deadband | on the floor | PWM at which each wheel first moves |
| 3 | Straight trim | 2 m clear floor | final TRIM_R after a/d tuning |
| 4 | Pivot timing | open floor, mark heading | final L/R ms; ×4-pivot error in degrees |
| 5 | Sensor stats | centred in a corridor, square | full printout + ruler distance of each wheel to its wall |
| 6 | Brake distance | 2 m clear floor | brake vs coast stop distance (cm) |
| 7 | Live stream | hand-held sweep | note anything weird (a sensor reading a wire, etc.) |
| 8 | Cruise speed | 2 m, a marked metre | seconds to cross the metre |

Tips for test 4 (the most important one): use the `x` / `y` keys — four
pivots in a row should bring the robot back exactly to its starting
heading; whatever angle it is off by, divide by 4 and tell me the number
and direction. Tune until the ×4 error is under ~10° total, for LEFT and
RIGHT separately.

Repeat tests 3 and 4 once more after the battery has done ~10 minutes of
driving — I want both the fresh-charge and the settled-charge numbers.

## What happens with the results

I fold them into `firmware/competition/competition.ino` (PWM_FLOOR, TRIM_R_PWM,
TURN90_MS_L/R, possibly PWM_CRUISE and the sensor targets), re-run the
simulator with the measured values, and deliver the frozen race build
plus the race-day checklist.
