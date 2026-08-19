# Research & Prior Art — verified facts the design is built on

Method: six parallel research passes (sensors, power, wall-following prior
art, competition tactics, motor control, Wokwi), then an adversarial
fact-check of every load-bearing claim against primary sources (ST L298
datasheet, HC-SR04 datasheet, Adafruit TT-motor data, Wokwi docs, NewPing
source). Verdicts below are post-verification.

## 0. The big find: `github.com/Muzoka/Robotic-comp`

A repository under the **same GitHub account as this one** contains a
complete, heavily documented firmware + simulator for what is clearly this
same competition: 305 mm corridor grid "from the organisers' spreadsheet",
three maps, one open 4-way crossing on map 2, black sector lines, striped
start gate. Its `config.h` documents measured tuning sweeps (completion %
per parameter value per map). Key transferable, *measured* lessons:

- **Straight-first through crossings** ("the single highest-value line in
  the file"): go straight whenever the front is open; only consult the
  wall-following hand rule when blocked. It removed the two pivots at the
  map-2 open crossing (a pivot with no walls to re-square against was the
  source of every livelock), made left-hand vs right-hand produce identical
  routes on all three maps, and raised the safe cruise speed. This is
  exactly the team's "go forward at the intersection" requirement.
- **Geometry-derived thresholds, never hand-tuned**: side-open threshold =
  centered-reading + 0.45 × corridor (a hand-set 42 cm threshold once made
  the robot drive past every turn); front-blocked ≈ 20 cm and front-open ≈
  26 cm with hysteresis; openings only count on a wall→open *edge* debounced
  over 9 consecutive 20 ms loops.
- **Junction motion sequence**: latch which sides are open at detection,
  creep forward so the *axle* reaches the junction centre (~15 cm), stop
  dead (260 ms settle), pivot at moderate PWM (110), drive into the new
  cell, then re-enable decisions. Livelock guard: >3 decisions within
  25 cm of travel → 35 cm decision lockout. Run timeout 240 s.
- **Side sonars mounted on the wheel-axle line** (not ahead of it) were
  measured to be worth ~20 percentage points of completion on the hardest
  map — an opening is *seen* exactly when the axle draws level with it.
  → We must check where our side sensors sit relative to the axle.
- **A floor line sensor was removed on purpose** — its striped-gate detector
  false-fired mid-maze and ended runs; finish-by-sonars (all three see open
  space) fired correctly on every run. Vindicates our no-floor-sensor
  finish plan.
- Its cruise-speed sweep (with deliberately degraded hardware: 2.5× wheel
  slip, 4× sonar noise, 15 % battery sag) picked PWM 200 not because faster
  failed, but to keep **steering headroom** (steer is added to cruise then
  clamped at 255).
- Robot-size cliff: completion collapsed at body diagonal ≈ 280 mm in the
  305 mm corridor. **Our robot (157 × 140 mm, diagonal ≈ 210 mm) is
  comfortably safe.**

⚠ **Caveat**: Robotic-comp assumes an MPU-6050 gyro and two wheel encoders.
Our parts list has **neither**, so its gyro-heading control, encoder
odometry, encoder-based stuck detection and turn profiles do *not*
transfer; the sonar handling, thresholds, state machine and junction
choreography do. (Question for the team: is Robotic-comp your earlier
build? Do you still have the gyro/encoders? A $2 MPU-6050 is the single
best legal-if-allowed hardware addition for no-encoder turn accuracy.)

Copies of the three key files are vendored in `reference/robotic-comp/`.

## 1. Power & motors (fact-checked against ST / Adafruit datasheets)

- **L298N 5V terminal**: with the regulator jumper installed it is the
  *output* of the onboard 78M05. The current D12 wire to it reads harmless
  as an input, but one `pinMode(12, OUTPUT)` + LOW would fight a 500 mA
  regulator against a 40 mA-absolute-max AVR pin, and with the Arduino off
  it back-feeds 5 V through the pin's protection diode. **Disconnect it.**
  (Confirmed against ST 78M05 + ATmega328P datasheets.)
- **L298N voltage drop** (ST datasheet Table 4): 1.8–3.2 V at 1 A, typical
  ~2.55 V. From the 2S pack the motors realistically see **~5.9 V at full
  charge → ~4.9 V at nominal → ~4.4 V near cutoff** — inside the TT motor's
  3–6 V rating, but a ~15 % speed drift over a discharge. Consequences:
  timed maneuvers must be calibrated at race-day charge level, and turns
  are sensor-verified, never trusted open-loop.
- **Low-voltage floor**: below ~7.0 V pack under load, both the L298N's
  78M05 and the Uno's VIN regulator leave their spec windows. Race policy:
  never run below ~7.2 V resting; always race in the same charge window
  (li-ion plateau 7.4–7.6 V, or top-up before every run — pick one and
  stick to it; never calibrate on the plateau then run at 8.4 V).
- **9 V PP3 alkaline: confirmed non-viable for the drive** (1.7 Ω internal
  impedance; manufacturer publishes no data above 50 mA draw; two TT
  motors surging ~2.4 A would sag it ~4 V). Bench logic tests only.
- **TT motors** (Adafruit data): 200 RPM ± 10 % @ 6 V, no-load 150 mA,
  stall 1.5 A @ 6 V each. ±10 % tolerance ⇒ up to ~20 % left/right
  mismatch, and the mismatch is *not* a constant ratio across PWM — trim
  must be calibrated at the exact cruise PWM used.
- **PWM deadband**: loaded TT + L298N typically won't start below PWM
  ~60–100 at the Uno's default 490/980 Hz; sustain threshold is lower than
  breakaway (hysteresis). **Keep the default PWM frequency** — deadband
  grows sharply at higher frequencies.
- **Stops**: coasting (enable low) overshoots ~2–10 cm at our speeds;
  active brake (enable high, IN1=IN2) stops in ~1–2 cm. Every
  read-and-decide happens after an active-brake stop. Full sign-magnitude
  drive isn't wireable here (the IN pins sit on non-PWM pins), so:
  EN-PWM drive + active-brake stops.
- **Ramps**: step PWM up/down over ~100–300 ms starting just under the
  breakaway threshold; sudden steps break traction on smooth painted wood
  (slip is an acceleration-transient problem at 410 g, not steady-state).
- **Pivot turns only** (both wheels counter-rotating): a one-wheel swing
  turn translates the robot ~7 cm sideways — nearly the entire 7.4 cm
  clearance. Pivot at moderate PWM (~110–150) for repeatability; expect
  90° in roughly 200–600 ms (calibrated per direction, per charge).
- Recommended if obtainable (cheap, not required): ≥220 µF + 100 nF across
  the sensor 5 V rail and bulk capacitance at the L298N supply; two
  resistors for a VIN divider → battery-voltage-scaled PWM (read against
  the internal 1.1 V bandgap so the reference doesn't sag with VIN).

## 2. HC-SR04 realities (fact-checked; several claims were sharpened)

- **Reliable floor is ~4–5 cm, not the datasheet's 2 cm.** At 5 cm modules
  read ~7 cm; at 2 cm the output is random (17–380 cm spikes). Our 7–10 cm
  operating band is fine but sits only ~3–5 cm above the garbage zone, and
  the failure is **non-graceful**: a robot drifting too close to a wall can
  suddenly read *far* — which naive logic reads as an opening and steers
  *into* the wall. Countermeasures: an implausible one-cycle jump on a side
  sensor is treated as "possibly touching wall", never "opening"; the
  L+R sum (constant ≈ corridor − sensor span in a corridor) is used as a
  validity check.
- **Specular walls**: reliable echoes only within ~10–15° of perpendicular;
  flaky from ~15–40°; near-certain dropout beyond ~40° (Brown CS148
  measured a smooth wooden wall "disappearing" at ~40°). Angled returns
  that do come back are *foreshortened* (nearest point in the cone). So:
  never range mid-pivot; verify a turn only once roughly square again.
  Black paint is acoustically irrelevant.
- **Cross-talk**: never fire two sensors at once (guaranteed interference
  between hard parallel walls 30.5 cm apart). Round-robin **F, L, F, R**
  inside a fixed 20 ms/50 Hz loop, one ping per tick, echo timeout capped
  (~13 ms ≈ 2 m) so a lost echo can't blow the loop budget. Effective
  update ~25 Hz front / 12.5 Hz per side — the reason cruise speed stays
  20–25 cm/s (at 30 cm/s the robot moves ~5 cm per full scan, most of the
  side clearance).
- **No-echo is ambiguous** (specular dropout, too-close failure, hung clone
  sensor, or genuinely open) — treat it as *invalid data*, believe "open"
  only after ≥2 consecutive far/invalid reads on a wall→open edge, and run
  an echo-pin watchdog (some clones latch ECHO high forever until the pin
  is forced low) that unlatches a stuck sensor.
- **Filtering**: median-of-3 per sensor, history primed with the first real
  reading (a zero-primed history once created a phantom 0 mm wall that
  self-started Robotic-comp's robot). First reading after power-up is
  garbage — discard.
- Datasheet suggests a 60 ms measurement cycle; community-validated floor
  is 29–33 ms between pings; with a capped max distance, ~20 ms staggering
  is workable and is validated in the calibration phase.

## 3. Control architecture (consensus of all sources)

- Centering: `error = (right − left)/2 − offset`, PD control
  (Kp scale ≈ 0.4 PWM/mm, Kd with a clamped derivative so a dropout can't
  slam the wheels), steer clamp ~80 PWM, small error deadband (~10–15 mm)
  to stop hunting. Single-wall fallback against the geometric target
  (corridor/2 − sensor offset) when the other side is open.
- Known small track ⇒ **hybrid scripted + reactive** (validated across
  micromouse, line-maze, and Science-Olympiad communities): encode each
  map's turn sequence as *junction events* ("at side-opening #2: straight"),
  wall-centre continuously between events, keep an always-on front
  emergency stop. Pure timed replay (no encoders) is the least reliable
  option; pure hand-rule following false-turns at the open intersection.
- Turns: stop (active brake) → settle ~250 ms → timed pivot (per-direction
  ms, calibrated at race charge) → post-turn squaring: the corridor PD
  absorbs residual error over the next ~30–50 cm; additionally the L−R
  trend while driving ("corridor-parallel correction") estimates heading
  angle without a gyro and trims it out.
- Stuck detection without encoders: all three sonar readings changing
  < ~2 cm for ~0.5–1 s while commanded to move → reverse 400–600 ms with
  alternating steer bias, then re-decide; plus front < ~5 cm → immediate
  short reverse; plus livelock guard and whole-run timeout.
- Start ritual (micromouse practice): back-square the chassis against the
  start wall; centre laterally with a cut 7.4 cm spacer block; hand-over-
  the-nose arming (hold ≥300 ms then withdraw) so no start button is
  needed; firmware runs a sensor self-test at the line (L+R ≈ known sum,
  no sensor stuck at 0/timeout) and refuses to launch on implausible
  readings, blinking the D13 LED.
- Venue-day: one warm-up traverse (burns li-ion surface charge, warms
  gearboxes, verifies turn timing on the real floor); hot-glue/tape every
  DuPont connector (vibration-loosened jumpers are a leading first-run
  killer); same person places the robot every run.

## 4. Wokwi (verified against official docs)

- `wokwi-hc-sr04` is an official part; distance attribute 2–400 cm,
  settable interactively (slider) and headlessly via scenario
  `set-control`. Three of them + Uno in one diagram works.
- **No official L298N or DC-motor part.** Community custom chip
  `drf5n/Wokwi-Chip-L298N@1.0.5` exists (pin that exact version); for logic
  tests it's cleaner to assert on serial telemetry and pin states.
- `wokwi-cli` runs headless scenario-scripted simulations (alpha; needs a
  free `WOKWI_CLI_TOKEN`, internet required; `expect-pin` samples
  instantaneous levels so PWM duty is asserted via serial prints instead).
- Robotic-comp already contains a self-contained Wokwi sketch — reusable
  scaffolding for our logic tests.

## 5. What this means in numbers for OUR robot (pre-calibration defaults)

| Constant | Default | Source |
|---|---|---|
| Loop | 50 Hz / 20 ms tick, ping order F,L,F,R | Robotic-comp, NewPing floor |
| Centered targets | L 86.5 mm, R 95.0 mm (sum 181.5 ± 15 valid) | team measurement |
| Side-open threshold | ~220 mm (= centered + 0.45 × 305) | geometry rule |
| Front-blocked / front-open | ~200 mm / ~260 mm + hysteresis | geometry rule (refine with axle position) |
| Open debounce | edge + 2 far reads + ~9 loops | Robotic-comp |
| Kp / steer clamp / deadband | 0.42 PWM/mm / 80 / 12 mm | Robotic-comp scale, retune in sim |
| Cruise / slow / turn / reverse PWM | ~140 / 95 / 120 / 95 (≈20–25 cm/s cruise) | tactics research + sim retune |
| PWM floor | ~55–70, measured on-robot | deadband research |
| 90° pivot | ~250–450 ms per direction, measured on-robot | prior-art spread |
| Run timeout | 240 s (confirm rules) | Robotic-comp |
