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

| Arduino pin | L298N | Role |
|---|---|---|
| D4 | IN1 | right motor direction |
| D5 (PWM) | ENA | right motor speed |
| D6 (PWM) | ENB | left motor speed |
| D7 | IN2 | right motor direction |
| D8 | IN3 | left motor direction |
| D11 | IN4 | left motor direction |
| D12 | — | **disconnected** (was the 5V-terminal wire — Fix 1) |

So most likely **nothing needs rewiring except removing the D12 wire**.
Please confirm with one close-up photo of the L298N end. If ENA truly is on
D4, swap it with the D5 wire (IN1↔ENA) — the firmware pinout is
`#define`-based either way.

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

## Confirmations requested from the team

1. Close-up photo of the L298N with wires attached (settles D4/ENA/ENB).
2. Confirm the ENA/ENB jumper caps are off, the 5 V-regulator jumper is on.
3. Confirm which OUT pair drives which physical side and the forward
   direction of each motor (the calibration sketch will verify this too).
4. Axle-to-front and axle-to-rear distances (pivot sweep clearance check).
