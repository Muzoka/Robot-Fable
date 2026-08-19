# Firmware

Two sketches, both self-contained (no libraries), both compile-verified
for Arduino Uno with `arduino-cli` (arduino:avr 1.8.8).

## `competition/` — the race sketch

A faithful port of the simulator controller tuned in `sim/` (same states,
same constants, same hardening mechanisms — see `sim/README.md` for the
list and why each exists). Highlights:

- **Map select without re-uploading**: jumper wire D2→GND = map 2,
  D3→GND = map 3, no jumper = map 1. Set it before power-on.
- **No start button**: power on → self-test (LED solid = pass; blink
  codes: 2/3/4 = front/left/right sensor dead, 5 = side sensors
  unreliable, 6 = robot badly placed at the start line) → hold a hand
  ~5–8 cm in front of the FRONT sensor for half a second, take it away →
  2 s LED countdown → the run starts. Hands are clear before any motion,
  and the robot refuses to launch on implausible sensor readings.
- `DEBUG 1` prints state transitions at 115200 baud; set `DEBUG 0` for
  race day.
- Constants marked `CAL` get their final values from the Phase-3
  calibration (`docs/CALIBRATION_PROTOCOL.md`).
- 240 s run timeout, then a full stop, whatever happens.

## `calibration/` — the Phase-3 measurement suite

Serial-menu driven (115200 baud). Eight numbered tests: motor direction,
PWM deadband, straight-line trim, 90° pivot timing (with a
four-pivots-in-a-row mode so the error is measured ×4), sensor
statistics at the start line, brake-vs-coast distance, a live sensor
stream, and a timed cruise run. The team runs them per
`docs/CALIBRATION_PROTOCOL.md` and pastes the output back; the results
become the final `competition/` constants.

## Building

```
arduino-cli compile --fqbn arduino:avr:uno firmware/competition
arduino-cli compile --fqbn arduino:avr:uno firmware/calibration
```

Or open either folder's `.ino` in the Arduino IDE and upload as usual.
