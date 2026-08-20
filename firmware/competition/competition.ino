/*
 * Robot-Fable competition firmware.
 *
 * A line-for-line port of the simulator controller tuned in sim/
 * (see sim/controller.py + sim/README.md for why each mechanism exists).
 * No external libraries. Arduino Uno.
 *
 * WIRING (team-confirmed, docs/HARDWARE_AUDIT.md):
 *   D4  L298N IN1   left motor direction
 *   D5  L298N ENA   left motor speed (PWM)
 *   D6  L298N ENB   right motor speed (PWM)
 *   D7  L298N IN2   left motor direction
 *   D8  L298N IN3   right motor direction
 *   D11 L298N IN4   right motor direction
 *   D9/D10   RIGHT HC-SR04 trig/echo
 *   A0/A1    FRONT HC-SR04 trig/echo
 *   A2/A3    LEFT  HC-SR04 trig/echo
 *   D2, D3   map-select jumpers to GND (see MAP SELECT below)
 *   D13      onboard LED (status)
 *   D12      free (the old wire to the L298N 5V terminal MUST be removed)
 *
 * MAP SELECT (internal pull-ups; jumper wire from pin to GND):
 *   no jumper          -> map 1 (3 sectors)
 *   D2 to GND          -> map 2 (5 sectors)
 *   D3 to GND          -> map 3 (9 sectors)
 *
 * START RITUAL (no button needed):
 *   power on -> self-test (LED solid on pass; error blink codes on fail)
 *   hold a hand ~5-8 cm in front of the FRONT sensor for 0.5 s  -> armed
 *   remove the hand                                             -> 2 s
 *   countdown (LED fast blink), then the run starts.
 */

#include <EEPROM.h>

// ================= DEBUG =================
#define DEBUG 1              // 1: serial telemetry. 0 for race day.
#if DEBUG
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#else
#define DBG(x)
#define DBGLN(x)
#endif

// ================= PINS =================
#define PIN_IN1 4            // left dir
#define PIN_ENA 5            // left speed (PWM)
#define PIN_ENB 6            // right speed (PWM)
#define PIN_IN2 7            // left dir
#define PIN_IN3 8            // right dir
#define PIN_IN4 11           // right dir
#define PIN_TRIG_R 9
#define PIN_ECHO_R 10
#define PIN_TRIG_F A0
#define PIN_ECHO_F A1
#define PIN_TRIG_L A2
#define PIN_ECHO_L A3
#define PIN_MAP2 2
#define PIN_MAP3 3
#define PIN_LED 13

// Flip to 1 if a wheel spins backwards (calibration test 1 verifies)
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 0

// ================= CONSTANTS (mirror sim/config.py; CAL = calibrate) ====
#define CORRIDOR_MM     305.0f
// Calibration test 5 (2026-08-20): centred at 77 mm true per side the
// sensors read L 87.9 / R 90.3 (sd 1.4/0.9, 0 misses) -> mount offsets
// L +10.9 / R +13.3 mm; targets below are those offsets applied to the
// real track's 74 mm-per-side centred clearance.
#define TARGET_L_MM     85.0f    // centred reading, left  (calibrated)
#define TARGET_R_MM     87.5f    // centred reading, right (calibrated)
#define SIDE_SUM_MM     172.5f
#define SIDE_OPEN_MM    220.0f
#define TRACK_L_MM      (TARGET_L_MM + 70.0f)
#define TRACK_R_MM      (TARGET_R_MM + 70.0f)
#define ERR_CLAMP_MM    40.0f
#define FRONT_BLOCKED_MM 200.0f
#define FRONT_OPEN_MM   260.0f
#define FRONT_STOP_MM   88.0f
#define FRONT_TOOCLOSE_MM 32.0f
#define FRONT_FAR_MM    400.0f
#define STUCK_FRONT_MM  400.0f
#define APPROACH_ABORT_MM 350.0f
#define CHAOS_JUMP_MM   150.0f
#define US_MAX_MM       2000.0f

#define OPEN_DEBOUNCE   5
#define FAR_MISSES      2
#define FRONT_DEBOUNCE  3
#define CHAOS_TRIP      6
#define FINISH_OPEN_TICKS 35
#define FINISH_ARM_N    10

// Calibration test 2 (2026-08-20, on floor): static breakaway L 125-140 /
// R 115-125; kick-sustain floor ~110-130. All speeds sit above the worst
// wheel with margin; a kick-start pulse (KICK_MS below) guarantees
// breakaway from standstill.
#define PWM_FLOOR       150      // CAL: never command the dead zone
#define PWM_CRUISE      180      // CAL: verified driving in tests 3/8
#define PWM_SLOW        160
#define PWM_TURN        200      // CAL: verified pivoting in test 4
#define PWM_BACK        170
#define PWM_SLEW        10
#define KICK_MS         70       // full-power pulse when starting straight

#define KP_WALL         0.22f
#define KD_WALL         0.20f
#define KD_SQUARE       0.45f
#define DERIV_CLAMP_MMS 600.0f
#define STEER_LIMIT     80
#define ERR_DEADBAND_MM 10.0f

#define LOOP_MS         20
// TURN90 times and straight trim live in EEPROM, written by the
// calibration sketch (tests 3/4) - same struct, same address. Re-running
// those two tests on race morning (fresh pack, venue floor) updates the
// race build without re-uploading. Defaults below = fresh-charge
// estimates, used only if EEPROM is empty or implausible.
#define TURN90_L_DEFAULT 270     // CAL test 4 (tired pack read 320)
#define TURN90_R_DEFAULT 250     // CAL test 4 (tired pack read 290-300)
#define TRIM_R_DEFAULT   0       // CAL test 3 (fresh 0; tired drifted -13)
#define TURN_TRIM_PULSE_MS 45    // ~10-15 deg nudge at PWM_TURN 200
#define WIG_SEG_MS      60       // wiggle-check segment (was 120 @ 680ms turns)
#define TURN_MAX_TRIMS  2
#define TURN_REV_MS     450
#define SETTLE_MS       250
#define POST_TURN_SETTLE_MS 150
#define CREEP_MS        450
#define REACQUIRE_MS    1000
#define SQUARE_MS       900
#define SQUARE_BRAKE_MS 140
#define APPROACH_TIMEOUT_MS 3000
#define APPROACH_FAR_MS 700
#define CROSS_MAX_MS    4000
#define CROSS_MIN_MS    800
#define BACKUP_MS       450
#define STUCK_MS        1100
#define DECIDE_REVERSE_MS 340
#define FINISH_DRIVE_MS 4200
#define RUN_TIMEOUT_MS  240000UL
#define ECHO_TIMEOUT_US 13000UL  // ~2.2 m; keeps a ping inside the tick

// ================= ROUTE SCRIPTS =================
// trig: 'F' = turn at blocked front; 'O' = turn into Nth side opening
// (unused on the confirmed maps, kept for map changes); 'X' = straight
// through a 4-way crossing.
struct Step { char trig; char dir; };
const Step SCRIPT1[] = {{'F','R'},{'F','R'}};
const Step SCRIPT2[] = {{'F','R'},{'X',0},{'F','R'},{'F','R'},{'F','R'},{'X',0}};
const Step SCRIPT3[] = {{'F','R'},{'F','R'},{'F','L'},{'F','L'},
                        {'F','L'},{'F','L'},{'F','R'},{'F','R'}};

Step script[10];
uint8_t scriptLen = 0, scriptPos = 0;

// state machine states (declared early: the Arduino builder
// inserts auto-prototypes before the first function)
enum State { ST_SELFTEST, ST_ARM, ST_COUNTDOWN,
             ST_CRUISE, ST_APPROACH, ST_DECIDE, ST_TURN, ST_SQUARE,
             ST_REACQUIRE, ST_CREEP, ST_CROSS, ST_BACKUP, ST_FINISH,
             ST_DONE };
const char* const ST_NAME[] = {"SELFTEST","ARM","COUNTDOWN","CRUISE",
  "APPROACH","DECIDE","TURN","SQUARE","REACQUIRE","CREEP","CROSS",
  "BACKUP","FINISH","DONE"};

// ============ CALIBRATED TUNABLES (EEPROM, shared w/ calibration.ino) ===
#define EE_MAGIC 0x5246          // 'RF' - must match calibration.ino
struct Persist { uint16_t magic; int16_t trim, turnL, turnR; };

int16_t trimR   = TRIM_R_DEFAULT;
int16_t turn90L = TURN90_L_DEFAULT;
int16_t turn90R = TURN90_R_DEFAULT;

void loadCal() {
  Persist p; EEPROM.get(0, p);
  if (p.magic == EE_MAGIC &&
      p.turnL >= 150 && p.turnL <= 500 &&
      p.turnR >= 150 && p.turnR <= 500 &&
      p.trim  >= -40 && p.trim  <= 40) {
    trimR = p.trim; turn90L = p.turnL; turn90R = p.turnR;
    DBGLN(F("cal: EEPROM"));
  } else {
    DBGLN(F("cal: DEFAULTS (EEPROM empty/implausible)"));
  }
  DBG(F("trimR="));   DBG(trimR);
  DBG(F(" turnL=")); DBG(turn90L);
  DBG(F(" turnR=")); DBGLN(turn90R);
}

// ================= SONAR =================
enum SonarId { SF = 0, SL = 1, SR = 2 };
const uint8_t TRIG[3] = {PIN_TRIG_F, PIN_TRIG_L, PIN_TRIG_R};
const uint8_t ECHO[3] = {PIN_ECHO_F, PIN_ECHO_L, PIN_ECHO_R};
const uint8_t PING_ORDER[4] = {SF, SL, SF, SR};   // front twice per cycle

float med[3];          // median-of-3 (mm)
bool medValid[3];
float hist3[3][3];
uint8_t histN[3];
uint8_t misses[3];
bool farFlag[3];
float lastRaw[3];      // last raw sample; <0 = no echo
uint8_t pingK = 0;

float pingOnce(uint8_t id) {   // returns mm, or -1 on no echo
  uint8_t t = TRIG[id], e = ECHO[id];
  if (digitalRead(e) == HIGH) {          // clone echo-latch watchdog
    pinMode(e, OUTPUT); digitalWrite(e, LOW);
    delayMicroseconds(500);
    pinMode(e, INPUT);
    return -1.0f;
  }
  digitalWrite(t, LOW); delayMicroseconds(3);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  unsigned long us = pulseIn(e, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0f;
  float mm = us * 0.1715f;               // us -> mm (343 m/s round trip)
  return (mm > US_MAX_MM) ? -1.0f : mm;  // beyond range = no echo (as sim)
}

uint8_t sonarTick() {                    // ping the scheduled sensor
  uint8_t id = PING_ORDER[pingK & 3];
  pingK++;
  float raw = pingOnce(id);
  lastRaw[id] = raw;
  if (raw < 0) {
    if (++misses[id] >= FAR_MISSES) farFlag[id] = true;
  } else {
    misses[id] = 0;
    farFlag[id] = false;
    if (histN[id] == 0) { hist3[id][0]=hist3[id][1]=hist3[id][2]=raw; histN[id]=3; }
    else { hist3[id][0]=hist3[id][1]; hist3[id][1]=hist3[id][2]; hist3[id][2]=raw; }
    float a=hist3[id][0], b=hist3[id][1], c=hist3[id][2];
    med[id] = max(min(a,b), min(max(a,b),c));
    medValid[id] = true;
  }
  return id;
}

float dist(uint8_t id) {
  if (farFlag[id] || !medValid[id]) return US_MAX_MM;
  return med[id];
}
bool wallAt(uint8_t id) {
  if (farFlag[id] || !medValid[id]) return false;
  return med[id] < (id == SF ? FRONT_OPEN_MM : SIDE_OPEN_MM);
}

// ================= MOTORS =================
int curL = 0, curR = 0;      // live PWM commands (signed)
bool curBrake = false;

void writeMotor(bool leftSide, int pwm, bool brake) {
  curBrake = brake;
  uint8_t en = leftSide ? PIN_ENA : PIN_ENB;
  uint8_t a  = leftSide ? PIN_IN1 : PIN_IN3;
  uint8_t b  = leftSide ? PIN_IN2 : PIN_IN4;
  bool inv   = leftSide ? MOTOR_L_INVERT : MOTOR_R_INVERT;
  if (brake || pwm == 0) {               // active brake: enable + both low
    digitalWrite(a, LOW); digitalWrite(b, LOW);
    analogWrite(en, brake ? 255 : 0);
    return;
  }
  bool fwd = (pwm > 0) != inv;
  digitalWrite(a, fwd ? HIGH : LOW);
  digitalWrite(b, fwd ? LOW : HIGH);
  int mag = abs(pwm);
  if (mag < PWM_FLOOR) mag = PWM_FLOOR;  // never command the dead zone
  if (mag > 255) mag = 255;
  analogWrite(en, mag);
}

// Kick-start (calibration test 2k): from standstill the loaded robot
// needs PWM >= ~140 to break static friction, but keeps rolling far
// lower once moving. A short full-power pulse on every stop->straight
// transition guarantees breakaway. Pivots are excluded: turn times were
// calibrated WITHOUT a kick and must keep identical dynamics.
unsigned long kickUntil = 0;
bool wasStopped = true;

void writePair(int l, int r, bool brake) {
  if (brake || (l == 0 && r == 0)) {
    wasStopped = true; kickUntil = 0;
    writeMotor(true, l, brake); writeMotor(false, r, brake);
    return;
  }
  bool straight = (l > 0 && r > 0) || (l < 0 && r < 0);
  if (wasStopped && straight) kickUntil = millis() + KICK_MS;
  wasStopped = false;
  if (straight && (long)(kickUntil - millis()) > 0) {
    writeMotor(true,  l > 0 ? 255 : -255, false);
    writeMotor(false, r > 0 ? 255 : -255, false);
  } else {
    writeMotor(true, l, false); writeMotor(false, r, false);
  }
}

void applyMotorsDirect(int tl, int tr, bool brake) {  // no slew: pivots
  curL = brake ? 0 : tl;
  curR = brake ? 0 : tr;
  writePair(curL, curR, brake);
}

void applyMotors(int tl, int tr, bool brake) {   // slew-limited
  int dl = tl - curL; if (dl > PWM_SLEW) dl = PWM_SLEW; if (dl < -PWM_SLEW) dl = -PWM_SLEW;
  int dr = tr - curR; if (dr > PWM_SLEW) dr = PWM_SLEW; if (dr < -PWM_SLEW) dr = -PWM_SLEW;
  curL += dl; curR += dr;
  if (brake) { curL = 0; curR = 0; }
  writePair(curL, curR, brake);
}

// ================= CONTROLLER STATE =================

State state = ST_SELFTEST;
unsigned long stateT0 = 0;         // millis at state entry
unsigned long runT0 = 0;
long timerMs() { return (long)(millis() - stateT0); }

float errPrev = 0, deriv = 0;
float errHist[5]; uint8_t errHistN = 0;
uint8_t frontBlockedN = 0;
uint8_t openN[3];                  // indexed by SL/SR
bool sideWasWall[3];
char turnDir = 0;
uint8_t trimsDone = 0, turnRetries = 0;
int turnRevMs = 0;
uint8_t allOpenN = 0;
int stuckMs = 0;
float lastMeds[3]; bool lastMedsValid = false;
uint8_t backups = 0;
float lastValidF = -1, fPrevRefresh = -1;
int8_t chaos = 0; uint8_t fFarN = 0;
uint8_t decPhase = 0; bool wiggleHit = false;
uint8_t xOpenN = 0, xCloseN = 0;
bool finArmed = false; uint8_t finWallN = 0;
uint8_t apprAbortN = 0, emptyBlocks = 0;
int apprFarMs = 0;
char escDir = 'L';

void enterState(State s) {
  state = s; stateT0 = millis();
  if (s == ST_DECIDE || s == ST_BACKUP) applyMotorsDirect(0, 0, true);
  if (s == ST_DECIDE) decPhase = 0;
  if (s == ST_APPROACH) apprFarMs = 0;
  if (s == ST_TURN) { trimsDone = 0; turnRetries = 0; turnRevMs = 0; }
  if (s == ST_TURN || s == ST_BACKUP) apprAbortN = 0;
  DBG(F("-> ")); DBGLN(ST_NAME[s]);
}

const Step* nextStep() {
  return (scriptPos < scriptLen) ? &script[scriptPos] : NULL;
}

// ---------------- steering (port of sim _steer) ----------------
void steer(int base, float gain, uint8_t refreshed, bool kdOnly,
           bool bothOnly, int* tl, int* tr) {
  float L = dist(SL), R = dist(SR);
  bool lw = wallAt(SL) && L < TRACK_L_MM &&
            (lastRaw[SL] < 0 || lastRaw[SL] < TRACK_L_MM + 60);
  bool rw = wallAt(SR) && R < TRACK_R_MM &&
            (lastRaw[SR] < 0 || lastRaw[SR] < TRACK_R_MM + 60);
  if (bothOnly && !(lw && rw)) { lw = false; rw = false; }
  float err;
  if (lw && rw)      err = ((L - TARGET_L_MM) - (R - TARGET_R_MM)) / 2.0f;
  else if (lw)       err = L - TARGET_L_MM;
  else if (rw)       err = TARGET_R_MM - R;
  else               err = 0.0f;
  // windowed derivative from RAW error (median steps null adjacent diffs)
  if (refreshed == SL || refreshed == SR) {
    if (errHistN < 5) errHist[errHistN++] = err;
    else { for (uint8_t i = 0; i < 4; i++) errHist[i] = errHist[i+1];
           errHist[4] = err; }
    if (errHistN >= 2) {
      float span = (errHistN - 1) * 0.040f;
      float d = (err - errHist[0]) / span;
      if (d >  DERIV_CLAMP_MMS) d =  DERIV_CLAMP_MMS;
      if (d < -DERIV_CLAMP_MMS) d = -DERIV_CLAMP_MMS;
      deriv = d;
    }
    errPrev = err;
  }
  if (err >  ERR_CLAMP_MM) err =  ERR_CLAMP_MM;
  if (err < -ERR_CLAMP_MM) err = -ERR_CLAMP_MM;
  if (err > -ERR_DEADBAND_MM && err < ERR_DEADBAND_MM) err = 0.0f;
  float s = kdOnly ? (KD_SQUARE * deriv)
                   : gain * (KP_WALL * err + KD_WALL * deriv);
  if (s >  STEER_LIMIT) s =  STEER_LIMIT;
  if (s < -STEER_LIMIT) s = -STEER_LIMIT;
  // err > 0: robot right of centre -> steer left -> right wheel faster
  *tl = base - (int)s;
  *tr = base + (int)s + trimR;
}

void resetSteering() { errPrev = 0; errHistN = 0; deriv = 0; }

void resetEdges() {
  sideWasWall[SL] = sideWasWall[SR] = false;
  openN[SL] = openN[SR] = 0;
}

// ---------------- edges (raw-sample based) ----------------
void trackEdges(uint8_t refreshed) {
  if (refreshed != SL && refreshed != SR) return;
  float raw = lastRaw[refreshed];
  if (raw >= 0 && raw < SIDE_OPEN_MM) {
    sideWasWall[refreshed] = true;
    openN[refreshed] = 0;
  } else if (sideWasWall[refreshed]) {
    openN[refreshed]++;
  }
}

// ================= STATE HANDLERS =================
void stCruise(uint8_t rf, float F);
void stApproach(uint8_t rf, float F);
void stDecide(uint8_t rf, float F);
void stTurn(uint8_t rf, float F);
void stSquare(uint8_t rf, float F);
void stReacquire(uint8_t rf, float F);
void stCreep(uint8_t rf, float F);
void stCross(uint8_t rf, float F);
void stBackup(uint8_t rf, float F);
void stFinish(uint8_t rf, float F);

void controllerTick(uint8_t rf) {
  float F = dist(SF);
  trackEdges(rf);
  if (rf == SF) {
    if (!farFlag[SF] && medValid[SF]) lastValidF = F;
    if (fPrevRefresh >= 0) {                    // chaos score
      bool farNow = farFlag[SF];
      float jump = F - fPrevRefresh; if (jump < 0) jump = -jump;
      if (jump > CHAOS_JUMP_MM) { if (chaos < CHAOS_TRIP+2) chaos++; }
      else if (!farNow && chaos > 0) chaos--;
      else if (farNow) { if (++fFarN >= 8 && chaos > 0) { chaos--; fFarN = 0; } }
      if (!farNow) fFarN = 0;
    }
    fPrevRefresh = F;
  }
  // global guards (never during pivots/backup/finish)
  if (state >= ST_CRUISE && state != ST_TURN && state != ST_BACKUP &&
      state != ST_FINISH && state != ST_DONE) {
    if (medValid[SF] && !farFlag[SF] && F < FRONT_TOOCLOSE_MM) {
      enterState(ST_BACKUP);
    } else if (chaos >= CHAOS_TRIP && state != ST_DECIDE &&
               state != ST_CROSS && curL > 0 && curR > 0) {
      chaos = 0;
      enterState(ST_BACKUP);
    }
    bool nearing = medValid[SF] && !farFlag[SF] && F < STUCK_FRONT_MM;
    if (state == ST_CRUISE && nearing) {        // frozen-readings stuck
      float meds[3] = {dist(SF), dist(SL), dist(SR)};
      bool moving = curL > PWM_FLOOR && curR > PWM_FLOOR;
      if (lastMedsValid) {
        float d = meds[rf] - lastMeds[rf]; if (d < 0) d = -d;
        if (d > 6.0f) stuckMs = 0;
        else if (moving && !curBrake) stuckMs += LOOP_MS;
      }
      for (uint8_t i = 0; i < 3; i++) lastMeds[i] = meds[i];
      lastMedsValid = true;
      if (stuckMs > STUCK_MS) { stuckMs = 0; enterState(ST_BACKUP); }
    } else { stuckMs = 0; lastMedsValid = false; }
  }
  switch (state) {
    case ST_CRUISE:    stCruise(rf, F); break;
    case ST_APPROACH:  stApproach(rf, F); break;
    case ST_DECIDE:    stDecide(rf, F); break;
    case ST_TURN:      stTurn(rf, F); break;
    case ST_SQUARE:    stSquare(rf, F); break;
    case ST_REACQUIRE: stReacquire(rf, F); break;
    case ST_CREEP:     stCreep(rf, F); break;
    case ST_CROSS:     stCross(rf, F); break;
    case ST_BACKUP:    stBackup(rf, F); break;
    case ST_FINISH:    stFinish(rf, F); break;
    default: break;
  }
}

void stCruise(uint8_t rf, float F) {
  if (rf == SF) {
    if (medValid[SF] && !farFlag[SF] && F < FRONT_BLOCKED_MM) frontBlockedN++;
    else frontBlockedN = 0;
  }
  if (frontBlockedN >= FRONT_DEBOUNCE) {
    turnDir = 0;
    enterState(ST_APPROACH);
    return;
  }
  const Step* nx = nextStep();
  // crossing pending: both-sides-open signature -> active straight-through
  if (nx && nx->trig == 'X') {
    if (!wallAt(SL) && !wallAt(SR)) {
      if (++xOpenN >= 6) { xOpenN = 0; enterState(ST_CROSS); return; }
    } else xOpenN = 0;
  }
  // side opening matching a scripted 'O' maneuver
  if (nx && nx->trig == 'O') {
    uint8_t side = (nx->dir == 'L') ? SL : SR;
    if (openN[side] >= OPEN_DEBOUNCE) {
      scriptPos++;
      turnDir = nx->dir;
      resetEdges();
      enterState(ST_CREEP);
      return;
    }
  }
  // finish: script done AND settled into the final corridor first
  if (!nx) {
    bool lw = wallAt(SL), rw = wallAt(SR);
    if (!finArmed) {
      if (rf == SL || rf == SR) {
        if (lw && rw) { if (++finWallN >= FINISH_ARM_N) finArmed = true; }
        else if (finWallN > 0) finWallN--;
      }
    } else {
      bool fOpen = farFlag[SF] || !medValid[SF] || F > FRONT_FAR_MM;
      if (!lw && !rw && fOpen) allOpenN++; else allOpenN = 0;
      if (allOpenN >= FINISH_OPEN_TICKS) { enterState(ST_FINISH); return; }
    }
  }
  bool both = wallAt(SL) && wallAt(SR);
  int base = both ? PWM_CRUISE : PWM_CRUISE - 10;   // single wall: gentler
  bool xPend = nx && nx->trig == 'X';
  int tl, tr; steer(base, 1.0f, rf, false, xPend, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stApproach(uint8_t rf, float F) {
  if (timerMs() > APPROACH_TIMEOUT_MS) { enterState(ST_DECIDE); return; }
  if (farFlag[SF]) {
    if (lastValidF >= 0 && lastValidF < 120.0f) { enterState(ST_DECIDE); return; }
    apprFarMs += LOOP_MS;
    if (apprFarMs > APPROACH_FAR_MS) {
      frontBlockedN = 0;
      apprAbortN++;
      enterState(apprAbortN >= 3 ? ST_BACKUP : ST_CRUISE);
      return;
    }
  } else {
    apprFarMs = 0;
    // arriving yawed: a side about to touch ends the approach NOW
    if ((lastRaw[SL] >= 0 && lastRaw[SL] < 55.0f) ||
        (lastRaw[SR] >= 0 && lastRaw[SR] < 55.0f)) {
      enterState(ST_DECIDE); return;
    }
    if (F > APPROACH_ABORT_MM) {
      frontBlockedN = 0;
      apprAbortN++;
      enterState(apprAbortN >= 3 ? ST_BACKUP : ST_CRUISE);
      return;
    }
    if (F <= FRONT_STOP_MM) { enterState(ST_DECIDE); return; }
  }
  int tl, tr; steer(PWM_SLOW, 0.5f, rf, false, false, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stDecide(uint8_t rf, float F) {
  long t = timerMs();
    if (decPhase == 0) {                     // settle at a dead stop
    applyMotors(0, 0, true);
    if (t < SETTLE_MS) return;
    if (turnDir != 0) {                    // O-turn path: wiggle-verify
      decPhase = 3; wiggleHit = false; stateT0 = millis(); return;
    }
    // phantom check: only a VALID far-ish reading proves a phantom
    if (!farFlag[SF] && medValid[SF] && F > FRONT_STOP_MM + 80.0f) {
      frontBlockedN = 0; enterState(ST_CRUISE); return;
    }
    if (farFlag[SF] || F < 60.0f ||
        (lastValidF >= 0 && lastValidF < 70.0f)) {
      decPhase = 1; stateT0 = millis(); return;   // blind-zone escape
    }
    decPhase = 2;
  }
  if (decPhase == 1) {
    if (t < DECIDE_REVERSE_MS) { applyMotorsDirect(-PWM_BACK, -PWM_BACK, false); return;
    }
    if (t < DECIDE_REVERSE_MS + POST_TURN_SETTLE_MS) { applyMotors(0, 0, true); return;
    }
    decPhase = 2;
  }
  if (decPhase == 3) {                     // wiggle: mirage-proof O-turns
    uint8_t side = (turnDir == 'L') ? SL : SR;
    float raw = lastRaw[side];
    if (raw >= 0 && raw < SIDE_OPEN_MM) wiggleHit = true;
    if (t < 4 * WIG_SEG_MS) {
      int seg = (t < WIG_SEG_MS || t >= 3 * WIG_SEG_MS) ? 1 : -1;
      int d = (turnDir == 'R') ? 1 : -1;
      applyMotorsDirect(PWM_TURN * seg * d, -PWM_TURN * seg * d, false);
      return;
    }
    if (t < 4 * WIG_SEG_MS + POST_TURN_SETTLE_MS) { applyMotors(0, 0, true); return;
    }
    if (wiggleHit) {                       // a wall answered: not an opening
      if (scriptPos) scriptPos--;          // restore the script entry
      uint8_t side2 = (turnDir == 'L') ? SL : SR;
      sideWasWall[side2] = false; openN[side2] = 0;
      turnDir = 0; frontBlockedN = 0;
      enterState(ST_CRUISE); return;
    }
    decPhase = 2;
  }
  if (turnDir == 0) {                      // front-blocked path
    const Step* nx = nextStep();
    if (nx && nx->trig == 'X') {
      // phantom corner at a pending crossing: straight is open by the map
      frontBlockedN = 0; enterState(ST_CROSS); return;
    }
    if (nx && nx->trig == 'F') {
      scriptPos++;
      turnDir = nx->dir;
    } else if (nx == NULL) {
      // script done: job is straight out the mouth. Realign on phantoms;
      // a repeatedly-confirmed real block means we're pointed wrong.
      DBGLN(F("!front-block, script done"));
      if (++emptyBlocks < 3) { frontBlockedN = 0; enterState(ST_BACKUP); return; }
      emptyBlocks = 0;
      turnDir = (dist(SL) > dist(SR)) ? 'L' : 'R';
    } else {
      DBGLN(F("!front-block, O pending"));
      turnDir = (dist(SL) > dist(SR)) ? 'L' : 'R';
    }
  }
  enterState(ST_TURN);
}

void stTurn(uint8_t rf, float F) {
  (void)rf;
  long t = timerMs();
  long dur = (turnDir == 'L') ? turn90L : turn90R;
  if (turnRetries) dur = dur * 6 / 10;     // finishing an interrupted pivot
  if (t <= dur) {
    int d = (turnDir == 'L') ? -1 : 1;
    applyMotorsDirect(PWM_TURN * d, -PWM_TURN * d, false);
    return;
  }
  if (t <= dur + POST_TURN_SETTLE_MS) { applyMotors(0, 0, true); return;
  }
  if (t <= dur + POST_TURN_SETTLE_MS + turnRevMs) { applyMotorsDirect(-PWM_BACK, -PWM_BACK, false); return;
  }
  if (turnRevMs) { turnRevMs = 0; stateT0 = millis(); return; }  // re-pivot
  bool ok = farFlag[SF] || !medValid[SF] || dist(SF) > FRONT_OPEN_MM;
  if (!ok && trimsDone < TURN_MAX_TRIMS) { // corrective micro-pulse
    trimsDone++;
    stateT0 = millis() - (dur - TURN_TRIM_PULSE_MS);
    return;
  }
  if (!ok && turnRetries < 2) {            // back out, finish the turn
    turnRetries++; trimsDone = 0; turnRevMs = TURN_REV_MS;
    stateT0 = millis() - (dur + POST_TURN_SETTLE_MS);
    return;
  }
  resetSteering();
  frontBlockedN = 0;
  turnDir = 0;
  resetEdges();
  enterState(ST_SQUARE);
}

void stSquare(uint8_t rf, float F) {
  (void)F;
  long t = timerMs();
  if (t < SQUARE_BRAKE_MS) { applyMotors(0, 0, true); return; }
  if (t >= SQUARE_BRAKE_MS + SQUARE_MS) { enterState(ST_REACQUIRE); return; }
  int tl, tr; steer(PWM_SLOW, 1.0f, rf, true, false, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stReacquire(uint8_t rf, float F) {
  (void)F;
  if (timerMs() >= REACQUIRE_MS) { enterState(ST_CRUISE); return; }
  int tl, tr; steer(PWM_SLOW, 0.9f, rf, false, false, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stCreep(uint8_t rf, float F) {
  (void)F;
  if (timerMs() >= CREEP_MS) { enterState(ST_DECIDE); return; }
  int tl, tr; steer(PWM_SLOW, 0.5f, rf, false, false, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stCross(uint8_t rf, float F) {
  (void)F;
  long t = timerMs();
  if (t < POST_TURN_SETTLE_MS) { applyMotors(0, 0, true); return; }
  bool done = t >= CROSS_MAX_MS;
  if (t > CROSS_MIN_MS && wallAt(SL) && wallAt(SR)) {
    if (++xCloseN >= 4) done = true;
  } else xCloseN = 0;
  if (done) {
    const Step* nx = nextStep();
    if (nx && nx->trig == 'X') scriptPos++;
    xCloseN = 0;
    resetEdges();
    frontBlockedN = 0;
    enterState(ST_SQUARE);
    return;
  }
  int tl, tr; steer(PWM_SLOW, 1.0f, rf, false, true, &tl, &tr);
  applyMotors(tl, tr, false);
}

void stBackup(uint8_t rf, float F) {
  (void)rf; (void)F;
  long t = timerMs();
  if (t < SETTLE_MS) { applyMotors(0, 0, true);
    escDir = (dist(SL) > dist(SR)) ? 'L' : 'R';
    return;
  }
  // every third consecutive backup escalates: longer reverse + a swing
  bool escal = backups >= 2 && (backups % 3) == 2;
  long dur = BACKUP_MS * (escal ? 2 : 1);
  if (t < SETTLE_MS + dur) {
    int bias = (backups & 1) ? 6 : -6;
    applyMotorsDirect(-(PWM_BACK + bias), -(PWM_BACK - bias), false);
    return;
  }
  if (escal && t < SETTLE_MS + dur + 350) {
    int d = (escDir == 'L') ? -1 : 1;
    applyMotorsDirect(PWM_TURN * d, -PWM_TURN * d, false);
    return;
  }
  backups++;
  resetSteering();
  frontBlockedN = 0;
  enterState(ST_SQUARE);
}

void stFinish(uint8_t rf, float F) {
  (void)rf; (void)F;
  // a wall back on either side: that was a crossing/hug, not the exit
  if (wallAt(SL) || wallAt(SR)) { allOpenN = 0; enterState(ST_CRUISE); return; }
  if (timerMs() >= FINISH_DRIVE_MS) {
    applyMotors(0, 0, true);
    enterState(ST_DONE);
    DBGLN(F("=== RUN COMPLETE ==="));
    return;
  }
  applyMotors(PWM_SLOW, PWM_SLOW + trimR, false);
}

// ================= START SEQUENCE =================
void ledBlinkForever(uint8_t code) {       // error code: N blinks, pause
  applyMotors(0, 0, true);
  for (;;) {
    for (uint8_t i = 0; i < code; i++) {
      digitalWrite(PIN_LED, HIGH); delay(180);
      digitalWrite(PIN_LED, LOW);  delay(180);
    }
    delay(900);
  }
}

void selfTestAndArm() {
  // 1) each sensor must produce a valid reading within 2 s
  for (uint8_t id = 0; id < 3; id++) {
    bool ok = false;
    for (uint8_t k = 0; k < 40; k++) {
      if (pingOnce(id) >= 0) { ok = true; break; }
      delay(50);
    }
    if (!ok) ledBlinkForever(2 + id);      // 2=F, 3=L, 4=R dead
  }
  // 2) at the start line the side sum must be plausible
  float sl = 0, sr = 0; uint8_t nl = 0, nr = 0;
  for (uint8_t k = 0; k < 10; k++) {
    float a = pingOnce(SL); if (a >= 0) { sl += a; nl++; }
    delay(35);
    float b = pingOnce(SR); if (b >= 0) { sr += b; nr++; }
    delay(35);
  }
  if (nl < 5 || nr < 5) ledBlinkForever(5);
  float sum = sl / nl + sr / nr;
  if (sum < SIDE_SUM_MM - 45 || sum > SIDE_SUM_MM + 45) {
    DBG(F("side sum ")); DBGLN(sum);
    ledBlinkForever(6);                    // badly placed / sensor skewed
  }
  digitalWrite(PIN_LED, HIGH);             // self-test passed: LED solid
  DBGLN(F("self-test OK; show hand to front sensor to arm"));
  // 3) hand over the nose >= 0.5 s arms; removing it starts the countdown
  unsigned long handT = 0;
  for (;;) {
    float f = pingOnce(SF);
    bool hand = f >= 0 && f < 100.0f;
    if (hand) { if (handT == 0) handT = millis(); }
    else if (handT && millis() - handT > 500) break;   // armed + removed
    else handT = 0;
    delay(60);
  }
  DBGLN(F("armed; countdown"));
  for (uint8_t i = 0; i < 10; i++) {       // 2 s fast blink
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(200);
  }
  digitalWrite(PIN_LED, LOW);
}

// ================= SETUP / LOOP =================
void loadScript() {
  pinMode(PIN_MAP2, INPUT_PULLUP);
  pinMode(PIN_MAP3, INPUT_PULLUP);
  delay(5);
  const Step* src; uint8_t n;
  if (digitalRead(PIN_MAP2) == LOW)      { src = SCRIPT2; n = 6; DBGLN(F("MAP 2")); }
  else if (digitalRead(PIN_MAP3) == LOW) { src = SCRIPT3; n = 8; DBGLN(F("MAP 3")); }
  else                                   { src = SCRIPT1; n = 2; DBGLN(F("MAP 1")); }
  for (uint8_t i = 0; i < n; i++) script[i] = src[i];
  scriptLen = n; scriptPos = 0;
}

void setup() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
    lastRaw[i] = -1; medValid[i] = false; histN[i] = 0;
    misses[i] = 0; farFlag[i] = false;
  }
  applyMotors(0, 0, true);
#if DEBUG
  Serial.begin(115200);
#endif
  DBGLN(F("Robot-Fable competition firmware"));
  loadCal();
  loadScript();
  selfTestAndArm();
  runT0 = millis();
  enterState(ST_CRUISE);
}

unsigned long lastTick = 0;

void loop() {
  if (state == ST_DONE) { applyMotors(0, 0, true); return; }
  if (millis() - runT0 > RUN_TIMEOUT_MS) {
    applyMotors(0, 0, true); state = ST_DONE; return;
  }
  if (millis() - lastTick < LOOP_MS) return;
  lastTick += LOOP_MS;
  if (millis() - lastTick > 100) lastTick = millis();  // resync after stalls
  uint8_t rf = sonarTick();
  controllerTick(rf);
}
