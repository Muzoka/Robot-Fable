# Wokwi logic verification

Runs the UNMODIFIED competition firmware against scripted sensor
scenarios — proving the state machine before it touches the robot.
Motors are intentionally unconnected: behavior is asserted through the
DEBUG serial telemetry (state transitions), which is how the harness in
`scenario-corner.yaml` checks the corner sequence.

## Interactive (browser, zero install)

1. Go to https://wokwi.com → new Arduino Uno project.
2. Replace `diagram.json` with this folder's file, and `sketch.ino` with
   `firmware/competition/competition.ino`.
3. Start the simulation, open the Serial Monitor.
4. Click each HC-SR04 and use its distance slider:
   - both sides at ~9 cm = centred in the corridor,
   - front 6 cm for a second then away = the hand-arming ritual,
   - front 18 → 8 cm = a corner approach; watch APPROACH → DECIDE → TURN.

## Headless (CI-style)

```
arduino-cli compile --fqbn arduino:avr:uno ../firmware/competition \
    --output-dir build
WOKWI_CLI_TOKEN=... wokwi-cli . --scenario scenario-corner.yaml --timeout 90000
```

Token: free at wokwi.com/ci. Note the scenario feature is alpha; if
`control:` is rejected by a newer CLI, rename it to `name:`.

Limits: Wokwi has no L298N/DC-motor part (verified in docs/RESEARCH.md
§4), and the robot never "moves", so sensors don't change on their own —
this tests decision logic and timing, not driving dynamics (that's what
`sim/` is for).
