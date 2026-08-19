# Robot-Fable — Maze Track Competition Robot

Arduino Uno 2WD robot that must autonomously traverse three walled tracks
(the 3-point, 5-point and 9-point maps). Competition scoring priority:

1. **Complete all maps** (most important — completing them wins)
2. **Fewest wall touches** (first tie-breaker)
3. **Time** (second tie-breaker)

Design consequence: **reliability beats speed everywhere**. We drive at a
moderate, controlled pace, stop-settle-verify at every decision point, and
only optimize time where it costs zero risk.

## Repository layout

| Path | Contents |
|------|----------|
| `docs/PLAN.md` | Master plan — phases, deliverables, what we need from the team |
| `docs/HARDWARE_AUDIT.md` | Wiring analysis, required rewires, power configuration |
| `docs/IDEAS_REVIEW.md` | Engineering evaluation of the team's proposed ideas |
| `docs/TRACK_NOTES.md` | Track geometry, derived numbers, route hypotheses per map |
| `docs/RESEARCH.md` | Verified component facts + prior art the design is built on |
| `docs/images/` | The three track photos |
| `firmware/` | Arduino sketches (competition code + calibration suite) — Phase 2 |
| `sim/` | Python track simulator used to tune the control logic — Phase 1 |
| `wokwi/` | Wokwi project for logic-level verification — Phase 2 |

## The robot

- Arduino Uno, L298N motor driver, 2× yellow TT gear motors (no encoders), rear caster
- 3× HC-SR04 ultrasonic sensors: LEFT, FRONT, RIGHT
- 2× 18650 li-ion in series (7.4–8.4 V) powering L298N 12V input and Arduino VIN
- 410 g, 15.7 cm wide (wheel-to-wheel), 14 cm long
- Corridor width 30.5 cm → ~7.4 cm nominal clearance per side
