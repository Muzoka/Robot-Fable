# Race-day run book

The race sketch is `firmware/competition/competition.ino`. It reads
TRIM_R and the two 90°-turn times **from EEPROM — the same slots the
calibration sketch saves to**. That is the core of the plan below: you
re-measure those three numbers on the venue floor with a fresh battery
in ~10 minutes, and the race sketch picks them up automatically.

## The night before

1. **Charge both 18650 pairs** (3800 race pair AND 5000 backup pair).
2. **The left-channel fix must be done**: run the calibration sketch's
   `w` wiggle test until the intermittent left-motor connection is found;
   re-seat/re-tighten it and **hot-glue or tape every left-channel
   connector**: OUT1/OUT2 screw terminals, D4/D7/D5 jumper ends, and the
   left motor's solder tabs. Then run test 3 twice back-to-back — the two
   trims must agree within ±2. If they don't, the fault is still there.
3. Secure the whole loom: bundle wires flat, nothing dangling in front
   of any sensor, tape every DuPont shell.
4. Pack: laptop + Arduino IDE, USB cable, spare jumper wires, small
   screwdriver, tape, the charged backup pair.

## At the venue, before your first run (~10 min)

1. Fresh 3800 pair in, slide switch ON.
2. **Upload `calibration.ino`.** On the venue floor:
   - Test **3**: `g` run, adjust `a`/`d`, repeat until straight
     (venue floors differ from home floors — this matters).
   - Test **4**: `e` (×4 right pivots) against a tape mark, adjust
     `d`/`c` until the net error over four pivots is under ~10°; same
     for `q` (left) with `a`/`z`.
   - Do NOT use test 9's reset — it wipes your tuning.
3. **Upload `competition.ino`.** Open the Serial Monitor once: the boot
   line must say `cal: EEPROM` followed by your just-tuned
   `trimR/turnL/turnR` values. If it says `DEFAULTS`, the EEPROM values
   were implausible — redo step 2.
4. Keep DEBUG 1; it only prints at boot and state changes and costs
   nothing at race time (nothing needs to be connected to serial).

## Per run

1. **Mode select** (set BEFORE power-on; jumper wire from pin to GND):
   - **No jumper = AUTO — the default. Use this.** The robot navigates
     ANY of the three maps, and any chain of maps joined together,
     without being told which one: it follows the corridor, turns to the
     side its sensors prove open, drives straight through crossings, and
     recognises the exit by sustained truly-open space. Simulator score:
     149/150 across all maps (beats the scripted mode's 146/150).
   - Scripted fallbacks (only if AUTO misbehaves on the day):
     Map 2: D2 → GND · Map 3: D3 → GND · Map 1: BOTH D2 and D3 → GND
2. Place the robot **centred in the start cell, nose square** to the
   corridor. Placement matters more than ever: the self-test now
   **measures the corridor at the start line and calibrates its
   centering targets from your placement** (so it adapts to the venue's
   real track width — no more blink-6 on a slightly different corridor).
   It still refuses (code 6) if a reading is implausible — usually a
   wire drooping into a sensor's beam or the robot not between walls.
3. Power on. LED goes **solid** when the self-test passes.
4. **Hold a hand ~5–8 cm in front of the FRONT sensor for half a
   second, then pull it away.** The LED fast-blinks a 2 s countdown and
   the run starts. Step well clear.
5. Between runs: power off, reposition, power on again.

### LED error codes (self-test failure, robot refuses to start)

| Blinks | Meaning |
|---|---|
| 2 | front sensor gives no echo (wire/loom in front of it?) |
| 3 | left sensor dead |
| 4 | right sensor dead |
| 5 | side sensors not answering consistently |
| 6 | side readings implausible — wire in a sensor's beam, or robot not between two walls |

## If something looks off between runs

- Robot curving in straight corridors → re-run calibration test 3
  (2 min) and re-upload the race sketch (values reload from EEPROM).
- Turns consistently over/under-rotating → calibration test 4, same loop.
- After many runs on one charge the pack sags; if turns visibly
  lengthen, swap to the charged backup pair **and redo the 10-minute
  venue calibration** — the numbers belong to the pair.
- A wheel not moving at all → the left-channel connection (see night-
  before list); wiggle-test it on the spot.

## What a correct run looks like (so you can spot trouble early)

In AUTO the robot needs no map knowledge, but the routes it should
produce are:

- **Map 1**: two right turns around the U, exit.
- **Map 2**: right turn, straight THROUGH the 4-way crossing, three
  rights, straight through the crossing again, exit.
- **Map 3**: serpentine — R R L L L L R R, exit.
- **Chained maps**: the same behaviors back to back; a small gap between
  joined maps reads as a crossing and is driven straight through.

A short pause before every turn (stop → think → pivot → re-square) is
normal and intended. Occasional single back-up-and-retry moves are the
recovery system working, not a fault.
