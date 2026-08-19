/*
 * Robot-Fable calibration suite (Phase 3).
 *
 * Upload, open Serial Monitor at 115200 baud (newline line ending),
 * put the robot where each test says, and send the menu key.
 * Paste the FULL serial output back to Claude - the results become the
 * final constants in firmware/competition/competition.ino.
 *
 * Wiring identical to the competition sketch. Robot on the REAL track
 * floor for motion tests, battery in the race-day charge window.
 *
 * MENU
 *  1  motor direction check  (robot on a box, wheels free)
 *  2  deadband finder        (robot on the floor; watch the wheels)
 *  3  straight-line trim     (2 m of clear floor; a/d adjust, s save)
 *  4  90-degree pivot timing (open floor; q/e run L/R, a/d adjust ms,
 *                             x runs 4 turns in a row = full 360)
 *  5  sensor statistics      (robot centred in the corridor)
 *  6  brake vs coast distance(2 m of clear floor)
 *  7  live sensor stream     (any position; move the robot around)
 *  8  cruise speed check     (2 m; measures nothing - time it yourself
 *                             over a marked metre and report seconds)
 */

#define PIN_IN1 4
#define PIN_ENA 5
#define PIN_ENB 6
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

int PWM_CRUISE = 110;
int PWM_TURN = 120;
int TRIM_R = 1;
int TURN_MS_L = 680;
int TURN_MS_R = 680;

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

int readKey() {                 // wait for one non-space serial char
  for (;;) {
    if (Serial.available()) {
      int c = Serial.read();
      if (c > ' ') return c;
    }
  }
}
bool keyPressed(int* c) {
  if (Serial.available()) { *c = Serial.read(); return *c > ' '; }
  return false;
}

void menu() {
  Serial.println(F("\n==== CALIBRATION MENU ===="));
  Serial.println(F("1 motor direction   2 deadband      3 straight trim"));
  Serial.println(F("4 pivot timing      5 sensor stats  6 brake distance"));
  Serial.println(F("7 live stream       8 cruise run"));
  Serial.print(F("current: TRIM_R=")); Serial.print(TRIM_R);
  Serial.print(F(" TURN_MS_L=")); Serial.print(TURN_MS_L);
  Serial.print(F(" TURN_MS_R=")); Serial.println(TURN_MS_R);
  Serial.println(F("choose> "));
}

void t1_direction() {
  Serial.println(F("\n-- 1: MOTOR DIRECTION. Wheels off the ground!"));
  Serial.println(F("LEFT motor forward 1 s...")); delay(800);
  motor(true, 120, false); delay(1000); stopAll();
  Serial.println(F("Did the LEFT wheel spin so the robot would move FORWARD? note y/n"));
  delay(1200);
  Serial.println(F("RIGHT motor forward 1 s...")); delay(800);
  motor(false, 120, false); delay(1000); stopAll();
  Serial.println(F("Did the RIGHT wheel spin forward? note y/n"));
  Serial.println(F("If a wheel ran backwards, set MOTOR_x_INVERT 1 in both sketches."));
}

void t2_deadband() {
  Serial.println(F("\n-- 2: DEADBAND. Robot on the floor, watch the wheels."));
  Serial.println(F("PWM ramps 40->110 on BOTH motors, 800 ms per step."));
  Serial.println(F("WRITE DOWN the PWM at which each wheel FIRST moves."));
  for (int p = 40; p <= 110; p += 5) {
    Serial.print(F("PWM = ")); Serial.println(p);
    motor(true, p, false); motor(false, p, false);
    delay(800);
  }
  stopAll();
  Serial.println(F("done. Report: left starts at ___, right starts at ___"));
}

void t3_trim() {
  Serial.println(F("\n-- 3: STRAIGHT TRIM. 2 m clear floor."));
  Serial.println(F("g = drive 2 s at cruise. a = more LEFT drift fix (-1),"));
  Serial.println(F("d = more RIGHT drift fix (+1), s = done."));
  for (;;) {
    int c = readKey();
    if (c == 'g') {
      Serial.print(F("run with TRIM_R=")); Serial.println(TRIM_R);
      motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
      delay(2000); stopAll();
      Serial.println(F("did it drift? a/d to adjust, g to rerun, s to finish"));
    } else if (c == 'a') { TRIM_R--; Serial.println(TRIM_R); }
    else if (c == 'd') { TRIM_R++; Serial.println(TRIM_R); }
    else if (c == 's') break;
  }
  Serial.print(F("FINAL TRIM_R_PWM = ")); Serial.println(TRIM_R);
}

void pivot(bool leftTurn, int ms) {
  int dl = leftTurn ? -PWM_TURN : PWM_TURN;
  motor(true, dl, false); motor(false, -dl, false);
  delay(ms);
  stopAll();
}

void t4_pivot() {
  Serial.println(F("\n-- 4: PIVOT TIMING. Open floor, mark the heading."));
  Serial.println(F("q = one LEFT pivot, e = one RIGHT pivot"));
  Serial.println(F("a/z = L time +10/-10   d/c = R time +10/-10"));
  Serial.println(F("x = FOUR right pivots (should be exactly 360)"));
  Serial.println(F("y = FOUR left pivots. s = done."));
  for (;;) {
    int c = readKey();
    if (c == 'q') { Serial.print(F("L ")); Serial.println(TURN_MS_L); pivot(true, TURN_MS_L); }
    else if (c == 'e') { Serial.print(F("R ")); Serial.println(TURN_MS_R); pivot(false, TURN_MS_R); }
    else if (c == 'a') { TURN_MS_L += 10; Serial.println(TURN_MS_L); }
    else if (c == 'z') { TURN_MS_L -= 10; Serial.println(TURN_MS_L); }
    else if (c == 'd') { TURN_MS_R += 10; Serial.println(TURN_MS_R); }
    else if (c == 'c') { TURN_MS_R -= 10; Serial.println(TURN_MS_R); }
    else if (c == 'x' || c == 'y') {
      for (uint8_t i = 0; i < 4; i++) {
        pivot(c == 'y', c == 'y' ? TURN_MS_L : TURN_MS_R);
        delay(500);
      }
      Serial.println(F("off from the start mark by how many degrees? divide by 4."));
    }
    else if (c == 's') break;
  }
  Serial.print(F("FINAL TURN90_MS_L = ")); Serial.println(TURN_MS_L);
  Serial.print(F("FINAL TURN90_MS_R = ")); Serial.println(TURN_MS_R);
}

void t5_stats() {
  Serial.println(F("\n-- 5: SENSOR STATS. Robot CENTRED in the corridor,"));
  Serial.println(F("nose square. 100 samples per sensor..."));
  for (uint8_t id = 0; id < 3; id++) {
    float mn = 1e9, mx = -1, sum = 0, sum2 = 0; int n = 0, miss = 0;
    for (int k = 0; k < 100; k++) {
      float v = pingOnce(id);
      if (v < 0) { miss++; }
      else { n++; sum += v; sum2 += v * v; if (v < mn) mn = v; if (v > mx) mx = v; }
      delay(35);
    }
    Serial.print(SNAME[id]);
    if (n == 0) { Serial.println(F(": NO ECHOES")); continue; }
    float mean = sum / n;
    float var = sum2 / n - mean * mean;
    Serial.print(F(": mean ")); Serial.print(mean, 1);
    Serial.print(F("mm sd ")); Serial.print(sqrt(var > 0 ? var : 0), 1);
    Serial.print(F(" min ")); Serial.print(mn, 0);
    Serial.print(F(" max ")); Serial.print(mx, 0);
    Serial.print(F(" misses ")); Serial.println(miss);
  }
  Serial.println(F("Also report: distance of each wheel to its wall (ruler)."));
}

void t6_brake() {
  Serial.println(F("\n-- 6: BRAKE DISTANCE. 2 m clear floor."));
  Serial.println(F("Run A: cruise 1.5 s then ACTIVE BRAKE. Mark stop point."));
  delay(1500);
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(1500);
  stopAll();
  delay(2500);
  Serial.println(F("Run B: cruise 1.5 s then COAST. Mark stop point."));
  delay(1500);
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(1500);
  motor(true, 0, false); motor(false, 0, false);   // coast
  delay(2500);
  stopAll();
  Serial.println(F("Report both distances from the brake point (cm)."));
}

void t7_stream() {
  Serial.println(F("\n-- 7: LIVE STREAM. Any key stops."));
  while (!Serial.available()) {
    for (uint8_t id = 0; id < 3; id++) {
      float v = pingOnce(id);
      Serial.print(SNAME[id][0]);
      Serial.print('=');
      if (v < 0) Serial.print(F("----"));
      else Serial.print(v, 0);
      Serial.print(' ');
      delay(30);
    }
    Serial.println();
  }
  Serial.read();
}

void t8_cruise() {
  Serial.println(F("\n-- 8: CRUISE RUN: 3 s at race cruise PWM."));
  Serial.println(F("Time it over a marked metre; report the seconds."));
  delay(1500);
  motor(true, PWM_CRUISE, false); motor(false, PWM_CRUISE + TRIM_R, false);
  delay(3000);
  stopAll();
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
  Serial.begin(115200);
  Serial.println(F("Robot-Fable calibration suite"));
}

void loop() {
  menu();
  switch (readKey()) {
    case '1': t1_direction(); break;
    case '2': t2_deadband(); break;
    case '3': t3_trim(); break;
    case '4': t4_pivot(); break;
    case '5': t5_stats(); break;
    case '6': t6_brake(); break;
    case '7': t7_stream(); break;
    case '8': t8_cruise(); break;
    default: Serial.println(F("?"));
  }
  stopAll();
}
