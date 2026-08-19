# Track Simulator

A deliberately hostile 2D simulation of the robot on the three competition
maps, used to design and tune the control logic before it touches the real
track. The controller (`controller.py`) is written to port line-for-line
to the Arduino firmware; `config.py` mirrors the future `config.h`.

## What is modelled

- Differential drive: motor deadband + hysteresis, first-order lag,
  left/right gain mismatch (the measured 0.7 % right drift), battery sag,
  traction-limited acceleration, active-brake vs coast stops, wall-contact
  sliding with damping.
- HC-SR04 (all failure modes verified in `docs/RESEARCH.md`): ±11° beam
  cone taking the nearest return, specular dropout rising with incidence
  angle (near-total beyond ~40°), close-range (<45 mm) garbage that reads
  *far*, gaussian noise + per-unit bias, random dropout, and the exact
  firmware scheduling (one ping per 20 ms tick, F,L,F,R, median-of-3,
  2-miss far debounce) — so cross-talk cannot exist, as on the robot.
- Maps as 5 mm occupancy grids built from 30.5 cm corridor modules
  (`track.py`), with open-floor aprons past the exit mouths and
  ground-truth finish zones.

## Controller architecture (ports to firmware)

Hybrid scripted + reactive. Per-map route script of maneuvers:
`('F',dir)` turn at a blocked front, `('O',dir)` turn into the Nth side
opening, `('X',)` straight through a 4-way crossing. Between maneuvers:
wall-centering PD (calibrated per-side targets L 86.5 / R 95.0 mm,
derivative over a 4-refresh window from RAW error, track-limit gates so a
receding wall is never chased, error clamp) with straight-first behavior.

Hardened decision layer (each mechanism earned by a specific observed
failure — see git history):

| Mechanism | Kills this failure |
|---|---|
| Committed APPROACH (creep on dropouts, side <55 mm ends it) | flapping at flickering corner echoes; yawed wedge entries |
| Blind-zone rule: far-after-close ⇒ *too close*, reverse first | nose in the <45 mm garbage zone reading "open" |
| Front-chaos score (far↔valid jumps ⇒ back off) | grinding on a corner the sensor can't see |
| Wiggle check before every 'O' turn | specular mirage consuming a scripted opening turn |
| Raw-sample edge detection (valid-near resets) | mirage side-openings at mild yaw |
| CROSS state (stop, hold straight, ignore front flicker) | pivoting/derailing inside the open crossing |
| SQUARE state (brake-settle + D-only wall-parallel align) | post-turn yaw seeding every downstream cascade |
| Turn verify + trim pulses + reverse-retry | under-rotated or wall-blocked pivots |
| Finish arming (clean corridor first) + walls-return abort | premature finish in junctions / while wall-hugging |
| Escalating backups; empty-script realign→turn-around | pocket wedges and wrong-way lockups |

## Usage

```
python3 -m sim.run one map2 7 --log --plot out.png   # single seeded run
python3 -m sim.run mc 25                             # Monte Carlo battery
```

## Current results (25 seeds × 2 modes × 3 maps)

Nominal = disciplined race day (calibrated turns, 0.90–1.0 battery window,
±15 mm / ±2.5° placement). Degraded = sloppy day (0.82–1.0 battery,
2.7× sensor noise, 3× dropout, deadband shift, ±25 mm / ±4° placement).

| map | mode | completed | contact-free | avg time |
|---|---|---|---|---|
| map1 | nominal | 25/25 | 25/25 | 31 s |
| map1 | degraded | 25/25 | 25/25 | 32 s |
| map2 | nominal | 22–24/25 | ~19/25 | ~65 s |
| map2 | degraded | 20–22/25 | ~15/25 | ~55 s |
| map3 | nominal | 25/25 | 23–25/25 | 72 s |
| map3 | degraded | 20–24/25 | ~20/25 | 74 s |

(Ranges over recent tuning runs; map2/map3-degraded vary seed to seed.)

Residual failure mode (all remaining failures share it): entering a
junction with large yaw under heavy sensor degradation → nose-wedge in a
junction-mouth pocket or a phantom scripted-turn consumption → route
derails and the recovery layer wanders. It is concentrated on map 2 (the
open crossing). Next levers: real-robot calibration numbers (Phase 3)
shrink exactly the perturbations that cause it; remaining tuning continues
alongside the firmware port.

`traces/` holds sample trajectory plots.
