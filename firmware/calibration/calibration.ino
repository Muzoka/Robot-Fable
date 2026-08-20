/*
 * Robot-Fable calibration suite (Phase 3) - SHORT-USB-CABLE EDITION.
 *
 * Upload with the Arduino IDE, open Serial Monitor at 115200 baud
 * (line ending: Newline). Battery slide switch ON the whole time - the
 * robot must stay powered when USB is unplugged.
 *
 * HOW THE MOTION TESTS WORK WITH A SHORT CABLE
 *   Press the test key -> the sketch prints "UNPLUG USB NOW" and counts
 *   down 8 s (LED double-blinks each second) -> unplug -> the robot runs
 *   the maneuver on battery -> it stops and the LED blinks slowly ->
 *   observe/measure -> replug USB and reopen the Serial Monitor.
 *   Reopening the monitor RESETS the Arduino - that is normal. All your
 *   adjustments (trim, turn times) are saved to EEPROM the moment you
 *   make them, so nothing is lost across resets; the menu shows the
 *   saved values every time.
 *
 * MENU
 *  1  motor direction check   (stationary: robot on a box, wheels free)
 *  2  deadband finder         (near-stationary: short pulses, robot on
 *                              the floor next to the laptop; nudge it
 *                              back between pulses if needed)
 *  3  straight-line trim      (UNPLUG test: drives 2 s; watch drift;
 *                              then a/d adjust and rerun)
 *  4  90-degree pivot timing  (UNPLUG test: e = 4 RIGHT pivots in a row,
 *                              q = 4 LEFT; should net exactly 360; the
 *                              error you see divided by 4 is per-turn)
 *  5  sensor statistics       (stationary: robot centred in a corridor)
 *  6  brake vs coast distance (UNPLUG test: two runs back to back)
 *  7  live sensor stream      (stationary)
 *  8  cruise speed run        (UNPLUG test: 3 s at race cruise; time a
 *                              marked metre with a stopwatch)
 *  9  show / reset saved values (r inside resets to defaults)
 *
 * Report per test: paste the serial text + what you measured with your
 * eyes/ruler/stopwatch. Repeat 3 and 4 after ~10 min of driving so we
 * get fresh-charge AND settled-charge numbers.
 */
#include <EEPROM.h>

#define PIN_IN1 4
#define PIN_ENA 5            // left speed
#define PIN_ENB 6            // right speed
#define PIN_IN2 7
#define PIN_IN3 8
#define PIN_IN4 11
#define PIN_TRIG_R 9
#define PIN_ECHO_R 10
#define PIN_TRIG_F A0
#define PIN_ECHO_F A1
#define PIN_TRIG_L A2
#define PIN_ECHO_L A3
#define PIN_LED 13

#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 0

#define UNPLUG_SECONDS 8

int PWM_CRUISE = 110;
int PWM_TURN = 120;

// persisted (EEPROM) tunables
int TRIM_R = 1;
int TURN_MS_L = 680;
int TURN_MS_R = 680;

#define EE_MAGIC 0x5246      // 'RF'
struct Persist { uint16_t magic; int16_t trim, turnL, turnR; };

void saveVals() {
  Persist p = {EE_MAGIC, (int16_t)TRIM_R, (int16_t)TURN_MS_L, (int16_t)TURN_MS_R};
  EEPROM.put(0, p);
}
void loadVals() {
  Persist p; EEPROM.get(0, p);
  if (p.magic == EE_MAGIC) { TRIM_R = p.trim; TURN_MS_L = p.turnL; TURN_MS_R = p.turnR; }
}

const uint8_t TRIG[3] = {PIN_TRIG_F, PIN_TRIG_L, PIN_TRIG_R};
const uint8_t ECHOP[3] = {PIN_ECHO_F, PIN_ECHO_L, PIN_ECHO_R};
const char* const SNAME[3] = {"FRONT", "LEFT", "RIGHT"};

float pingOnce(uint8_t id) {
  uint8_t t = TRIG[id], e = ECHOP[id];
  if (digitalRead(e) == HIGH) {
    pinMode(e, OUTPUT); digitalWrite(e, LOW);
    delayMicroseconds(500); pinMode(e, INPUT);
    return -1.0f;
  }
  digitalWrite(t, LOW); delayMicroseconds(3);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  unsigned long us = pulseIn(e, HIGH, 20000UL);
  if (us == 0) return -1.0f;
  return us * 0.1715f;
}

void motor(bool leftSide, int pwm, bool brake) {
  uint8_t en = leftSide ? PIN_ENA : PIN_ENB;
  uint8_t a  = leftSide ? PIN_IN1 : PIN_IN3;
  uint8_t b  = leftSide ? PIN_IN2 : PIN_IN4;
  bool inv   = leftSide ? MOTOR_L_INVERT : MOTOR_R_INVERT;
  if (brake || pwm == 0) {
    digitalWrite(a, LOW); digitalWrite(b, LOW);
    analogWrite(en, brake ? 255 : 0);
    return;
  }
  bool fwd = (pwm > 0) != inv;
  digitalWrite(a, fwd ? HIGH : LOW);
  digitalWrite(b, fwd ? LOW : HIGH);
  analogWrite(en, min(abs(pwm), 255));
}

void stopAll() { motor(true, 0, true); motor(false, 0, true); }

void unplugCountdown() {
  Serial.println(F(">>> UNPLUG USB NOW (battery must be ON)."));
  Serial.print(F(">>> starting in ")); Serial.print(UNPLUG_SECONDS);
  Serial.println(F(" s; LED double-blinks each second"));
  Serial.flush();
  for (uint8_t s = 0; s < UNPLUG_SECONDS; s++) {
    digitalWrite(PIN_LED, HIGH); delay(120);
    digitalWrite(PIN_LED, LOW);  delay(120);
    digitalWrite(PIN_LED, HIGH); delay(120);
    digitalWrite(PIN_LED, LOW);  delay(640);
  }
}

void doneBlink() {                 // maneuver over: slow blink ~6 s
  stopAll();
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(PIN_LED, HIGH); delay(350);
    digitalWrite(PIN_LED, LOW);  delay(350);
  }
  Serial.println(F("(done - replug USB and reopen the Serial Monitor)"));
}

int readKey() {                 // wait for one printable char, then drain
  for (;;) {                    // the newline the Serial Monitor appends
    if (Serial.available()) {
      int c = Serial.read();
      if (c > ' ') {
        delay(3);
        while (Serial.available() && Serial.peek() <= ' ') Serial.read();
        return c;
      }
    }
  }
}

bool abortKey() {               // true only on a PRINTABLE char; drains
  while (Serial.available()) {  // stray newlines instead of aborting
    if (Serial.peek() > ' ') { Serial.read(); return true; }
    Serial.read();
  }
  return false;
}

void menu() {
  Serial.println(F("\n==== ROBOT-FABLE CALIBRATION ===="));
  Serial.println(F("0 enable-wire check (box)"));
  Serial.println(F("1 motor direction (box)     2 deadband (pulses)"));
  Serial.println(F("3 straight trim (UNPLUG)    4 pivot timing (UNPLUG)"));
  Serial.println(F("5 sensor stats              6 brake distance (UNPLUG)"));
  Serial.println(F("7 live stream               8 cruise run (UNPLUG)"));
  Serial.println(F("9 show/reset saved values"));
  Serial.print(F("saved: TRIM_R=")); Serial.print(TRIM_R);
  Serial.print(F("  TURN_MS_L=")); Serial.print(TURN_MS_L);
  Serial.print(F("  TURN_MS_R=")); Serial.println(TURN_MS_R);
  Serial.println(F("choose> "));
}

void t0_enable() {
  Serial.println(F("\n-- 0: ENABLE-WIRE CHECK. Robot on a box, wheels FREE."));
  Serial.println(F("Each motor gets direction ON but speed=ZERO for 2 s."));
  Serial.println(F("A correct enable wire means the wheel stays STILL."));
  Serial.println(F("If a wheel SPINS during its zero phase, its ENA/ENB"));
  Serial.println(F("wire is NOT reaching the Arduino pin (floating = full on)."));
  delay(2500);
  Serial.println(F("LEFT: speed ZERO now (must stay still)..."));
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);
  delay(2000);
  Serial.println(F("LEFT: speed 140 now (should spin medium)..."));
  analogWrite(PIN_ENA, 140);
  delay(1200);
  stopAll();
  delay(1500);
  Serial.println(F("RIGHT: speed ZERO now (must stay still)..."));
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENB, 0);
  delay(2000);
  Serial.println(F("RIGHT: speed 140 now (should spin medium)..."));
  analogWrite(PIN_ENB, 140);
  delay(1200);
  stopAll();
  Serial.println(F("Report per wheel: still during ZERO? spinning at 140?"));
}

void t1_direction() {
  Serial.println(F("\n-- 1: MOTOR DIRECTION. Robot on a box, wheels FREE."));
  Serial.println(F("LEFT motor 'forward' 1 s in 3..."));
  delay(3000);
  motor(true, 120, false); delay(1000); stopAll();
  Serial.println(F("-> would that spin have moved the robot FORWARD? note it"));
  delay(1500);
  Serial.println(F("RIGHT motor 'forward' 1 s in 3..."));
  delay(3000);
  motor(false, 120, false); delay(1000); stopAll();
  Serial.println(F("-> forward? note it. If a wheel ran backwards we set"));
  Serial.println(F("   MOTOR_x_INVERT in the code - just tell me which one."));
}

void t2_deadband() {
  Serial.println(F("\n-- 2: DEADBAND, pulsed. Robot ON THE FLOOR next to"));
  Serial.println(F("the laptop; each PWM step pulses 0.4 s then stops, so"));
  Serial.println(F("the robot only creeps. Nudge it back anytime."));
  Serial.println(F("WRITE DOWN the PWM where each wheel FIRST moves."));
  Serial.println(F("any key = abort"));
  for (int p = 40; p <= 110; p += 5) {
    Serial.print(F("PWM = ")); Serial.println(p);
    digitalWrite(PIN_LED, HIGH);
    motor(true, p, false); motor(false, p, false);
    delay(400);
    stopAll();
    digitalWrite(PIN_LED, LOW);
    delay(700);
    if (abortKey()) { Serial.println(F("(aborted)")); break; }
  }
  stopAll();
  Serial.println(F("done. Report: LEFT first moved at ___, RIGHT at ___"));
}

void t3_trim() {
  Serial.println(F("\n-- 3: STRAIGHT TRIM. Needs ~2 m of clear floor."));
  Serial.print(F("current TRIM_R = ")); Serial.println(TRIM_R);
  Serial.println(F("g = drive (unplug countdown, then 2 s at cruise)"));
  Serial.println(F("after the run: replug, reopen monitor, come back to"));
  Serial.println(F("this test, then a = fix drift-to-LEFT (-1),"));
  Serial.println(F("d = fix drift-to-RIGHT (+1). s = back to menu."));
  for (;;) {
    int c = readKey();
    if (c == 'g') {
      unplugCountdown();
      motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
      delay(2000);
      doneBlink();
      Serial.println(F("which way did it drift, and by how many cm over the run?"));
    } else if (c == 'a') { TRIM_R--; saveVals(); Serial.print(F("TRIM_R=")); Serial.println(TRIM_R); }
    else if (c == 'd') { TRIM_R++; saveVals(); Serial.print(F("TRIM_R=")); Serial.println(TRIM_R); }
    else if (c == 's') break;
  }
  Serial.print(F("saved TRIM_R = ")); Serial.println(TRIM_R);
}

void pivot(bool leftTurn, int ms) {
  int dl = leftTurn ? -PWM_TURN : PWM_TURN;
  motor(true, dl, false); motor(false, -dl, false);
  delay(ms);
  stopAll();
}

void t4_pivot() {
  Serial.println(F("\n-- 4: PIVOT TIMING. Open floor; put a tape mark or"));
  Serial.println(F("note exactly where the nose points before each run."));
  Serial.print(F("current TURN_MS_L=")); Serial.print(TURN_MS_L);
  Serial.print(F(" TURN_MS_R=")); Serial.println(TURN_MS_R);
  Serial.println(F("e = FOUR right pivots (unplug run; should net 360)"));
  Serial.println(F("q = FOUR left pivots  (unplug run)"));
  Serial.println(F("single pivots: E = one right, Q = one left"));
  Serial.println(F("adjust (saved instantly): d/c = R +10/-10, a/z = L +10/-10"));
  Serial.println(F("s = back to menu"));
  for (;;) {
    int c = readKey();
    if (c == 'e' || c == 'q') {
      unplugCountdown();
      for (uint8_t i = 0; i < 4; i++) { pivot(c == 'q', c == 'q' ? TURN_MS_L : TURN_MS_R); delay(800); }
      doneBlink();
      Serial.println(F("how many degrees off the mark, and which way? (divide by 4)"));
    } else if (c == 'E' || c == 'Q') {
      unplugCountdown();
      pivot(c == 'Q', c == 'Q' ? TURN_MS_L : TURN_MS_R);
      doneBlink();
    }
    else if (c == 'a') { TURN_MS_L += 10; saveVals(); Serial.println(TURN_MS_L); }
    else if (c == 'z') { TURN_MS_L -= 10; saveVals(); Serial.println(TURN_MS_L); }
    else if (c == 'd') { TURN_MS_R += 10; saveVals(); Serial.println(TURN_MS_R); }
    else if (c == 'c') { TURN_MS_R -= 10; saveVals(); Serial.println(TURN_MS_R); }
    else if (c == 's') break;
  }
  Serial.print(F("saved TURN_MS_L=")); Serial.print(TURN_MS_L);
  Serial.print(F(" TURN_MS_R=")); Serial.println(TURN_MS_R);
}

void t5_stats() {
  Serial.println(F("\n-- 5: SENSOR STATS. Robot CENTRED in a corridor,"));
  Serial.println(F("nose square to the walls. 100 samples per sensor..."));
  for (uint8_t id = 0; id < 3; id++) {
    float mn = 1e9, mx = -1, sum = 0, sum2 = 0; int n = 0, miss = 0;
    for (int k = 0; k < 100; k++) {
      float v = pingOnce(id);
      if (v < 0) miss++;
      else { n++; sum += v; sum2 += v * v; if (v < mn) mn = v; if (v > mx) mx = v; }
      delay(35);
    }
    Serial.print(SNAME[id]);
    if (n == 0) { Serial.println(F(": NO ECHOES")); continue; }
    float mean = sum / n;
    float var = sum2 / n - mean * mean;
    Serial.print(F(": mean ")); Serial.print(mean, 1);
    Serial.print(F(" mm  sd ")); Serial.print(sqrt(var > 0 ? var : 0), 1);
    Serial.print(F("  min ")); Serial.print(mn, 0);
    Serial.print(F("  max ")); Serial.print(mx, 0);
    Serial.print(F("  misses ")); Serial.println(miss);
  }
  Serial.println(F("Also measure with a ruler: each wheel's distance to its"));
  Serial.println(F("wall, so we can tie readings to true position."));
}

void t6_brake() {
  Serial.println(F("\n-- 6: BRAKE vs COAST. ~2.5 m clear floor."));
  Serial.println(F("One unplug run does BOTH: run A = cruise 1.5 s then"));
  Serial.println(F("HARD BRAKE (LED goes on at the brake moment - mark"));
  Serial.println(F("where it stops). 4 s pause. Run B = same but COAST."));
  Serial.println(F("g = go, s = menu"));
  int c = readKey();
  if (c != 'g') return;
  unplugCountdown();
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(1500);
  digitalWrite(PIN_LED, HIGH);
  stopAll();                                   // active brake
  delay(4000);
  digitalWrite(PIN_LED, LOW);
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(1500);
  digitalWrite(PIN_LED, HIGH);
  motor(true, 0, false); motor(false, 0, false);   // coast
  delay(3000);
  doneBlink();
  Serial.println(F("report both stopping distances (cm past the LED moment)"));
}

void t7_stream() {
  Serial.println(F("\n-- 7: LIVE STREAM. Move the robot by hand; any key stops."));
  for (;;) {
    if (abortKey()) return;
    for (uint8_t id = 0; id < 3; id++) {
      float v = pingOnce(id);
      Serial.print(SNAME[id][0]); Serial.print('=');
      if (v < 0) Serial.print(F("----")); else Serial.print(v, 0);
      Serial.print(' ');
      delay(30);
    }
    Serial.println();
  }
}

void t8_cruise() {
  Serial.println(F("\n-- 8: CRUISE RUN. Mark a metre on the floor."));
  Serial.println(F("3 s at race cruise; stopwatch the marked metre."));
  unplugCountdown();
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(3000);
  doneBlink();
  Serial.println(F("report: seconds per metre"));
}

void t9_values() {
  Serial.print(F("TRIM_R=")); Serial.print(TRIM_R);
  Serial.print(F(" TURN_MS_L=")); Serial.print(TURN_MS_L);
  Serial.print(F(" TURN_MS_R=")); Serial.println(TURN_MS_R);
  Serial.println(F("r = reset to defaults, anything else = back"));
  if (readKey() == 'r') {
    TRIM_R = 1; TURN_MS_L = 680; TURN_MS_R = 680;
    saveVals();
    Serial.println(F("reset."));
  }
}

void setup() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
    pinMode(ECHOP[i], INPUT);
  }
  stopAll();
  loadVals();
  Serial.begin(115200);
  Serial.println(F("Robot-Fable calibration (short-cable edition)"));
}

void loop() {
  menu();
  switch (readKey()) {
    case '0': t0_enable(); break;
    case '1': t1_direction(); break;
    case '2': t2_deadband(); break;
    case '3': t3_trim(); break;
    case '4': t4_pivot(); break;
    case '5': t5_stats(); break;
    case '6': t6_brake(); break;
    case '7': t7_stream(); break;
    case '8': t8_cruise(); break;
    case '9': t9_values(); break;
    default: Serial.println(F("?"));
  }
  stopAll();
}
