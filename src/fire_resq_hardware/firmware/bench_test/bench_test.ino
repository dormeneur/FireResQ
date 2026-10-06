// FireResQ ESP32 BENCH TEST - prove the wiring before any ROS integration.
// NOT the Phase 12 firmware: no ROS, no encoders, no odometry. Wiring: HOW_TO_MAKE_THE_COMPLETE_ROBOT.md, step 3.
//
// Type a command in the Serial Monitor (115200 baud, "Newline"):
//   f / b      forward / backward          l / r   spin left / right on the spot
//   s          stop                         1..9    speed (default 5)
//   m1 / m0    "magnet" ON / OFF (an LED now; the MOSFET gate later - same pin)
//   ?          help + current state
// SAFETY: every motion stops by itself after MOVE_MS. Nothing moves unless you keep sending commands.

// ---- pins (change here only) -------------------------------------------------------------------
// Motor driver inputs. L298N: IN1 IN2 ENA (left), IN3 IN4 ENB (right).
// TB6612FNG: AIN1 AIN2 PWMA (left), BIN1 BIN2 PWMB (right), and STBY -> 3.3 V.
const int L_IN1 = 26, L_IN2 = 27, L_EN = 25;
const int R_IN1 = 32, R_IN2 = 33, R_EN = 14;
const int MAGNET_PIN = 4;     // LED (+ 220 ohm) now; logic-level MOSFET module gate later
const int STATUS_LED = 2;     // the on-board LED on most ESP32 dev boards; mirrors the magnet
// Reserved for the encoders (Phase 12; input-only pins, need external pull-ups): 34, 35 (left A/B), 36, 39 (right A/B).

// If a wheel turns the wrong way, flip its sign here instead of rewiring.
const int L_DIR = +1, R_DIR = +1;

const unsigned long MOVE_MS = 1000;   // watchdog: a motion command lasts at most this long

int speedLevel = 5;                   // 1..9
unsigned long moveUntil = 0;
bool magnetOn = false;

void motor(int in1, int in2, int en, int dirSign, int pwm) {   // pwm -255..255
  pwm *= dirSign;
  digitalWrite(in1, pwm > 0 ? HIGH : LOW);
  digitalWrite(in2, pwm < 0 ? HIGH : LOW);
  analogWrite(en, abs(pwm));
}

void drive(int left, int right) {
  motor(L_IN1, L_IN2, L_EN, L_DIR, left);
  motor(R_IN1, R_IN2, R_EN, R_DIR, right);
}

void stopAll() { drive(0, 0); moveUntil = 0; }

void setMagnet(bool on) {
  magnetOn = on;
  digitalWrite(MAGNET_PIN, on ? HIGH : LOW);
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
  Serial.println(on ? "MAGNET ON" : "MAGNET OFF");
}

void help() {
  Serial.println("f b l r s | 1..9 speed | m1 m0 magnet | ?");
  Serial.printf("speed %d, magnet %s\n", speedLevel, magnetOn ? "ON" : "OFF");
}

void setup() {
  Serial.begin(115200);
  int outs[] = {L_IN1, L_IN2, L_EN, R_IN1, R_IN2, R_EN, MAGNET_PIN, STATUS_LED};
  for (int p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  stopAll();
  setMagnet(false);
  Serial.println("FireResQ bench test ready.");
  help();
}

void loop() {
  if (moveUntil && millis() > moveUntil) { stopAll(); Serial.println("auto-stop"); }
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim();
  if (c.length() == 0) return;
  int pwm = map(speedLevel, 1, 9, 90, 255);   // below ~90 most geared motors just hum
  if      (c == "f") { drive( pwm,  pwm); moveUntil = millis() + MOVE_MS; }
  else if (c == "b") { drive(-pwm, -pwm); moveUntil = millis() + MOVE_MS; }   // bench test only: the robot never reverses autonomously
  else if (c == "l") { drive(-pwm,  pwm); moveUntil = millis() + MOVE_MS; }
  else if (c == "r") { drive( pwm, -pwm); moveUntil = millis() + MOVE_MS; }
  else if (c == "s") { stopAll(); }
  else if (c == "m1") { setMagnet(true); }
  else if (c == "m0") { setMagnet(false); }
  else if (c.length() == 1 && c[0] >= '1' && c[0] <= '9') { speedLevel = c[0] - '0'; }
  else if (c == "?") { help(); return; }
  else { Serial.println("unknown command"); help(); return; }
  Serial.printf("ok: %s\n", c.c_str());
}
