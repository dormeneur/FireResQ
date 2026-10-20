// WANDER BOT firmware - Arduino Uno + Adafruit Motor Shield v1 (AFMotor).
// Flashed by wander_bot.py when it starts; the robot's normal firmware is flashed back when it exits.
//
//   Pi -> Uno   M a b c d   signed power (-255..255) for shield terminals M1..M4
//               U deg       sweep the servo (SERVO 1 header = D10) slowly to deg, then ping -> "U deg cm"
//               G deg       sweep the servo to deg, no answer
//               P           ping where the servo is now -> "D cm"
//               ?           identify
//   Uno -> Pi   WANDER2     at boot and for '?'      cm: 400 = nothing heard
// Watchdog: no M line for 400 ms releases every motor.
//
// Sonar: TRIG = A3, ECHO = A4.
#include <AFMotor.h>
#include <Servo.h>

const uint8_t TRIG_PIN = A3, ECHO_PIN = A4;
const uint8_t SERVO_PIN = 10;                 // shield SERVO 1
const unsigned long WATCHDOG_MS = 400;
const unsigned SERVO_STEP_MS = 20;            // ~50 degrees/second

AF_DCMotor motor[4] = {AF_DCMotor(1), AF_DCMotor(2), AF_DCMotor(3), AF_DCMotor(4)};
Servo servo;
int applied[4] = {0, 0, 0, 0};
int servoDeg = 90, servoTarget = 90, replyDeg = -1;
unsigned long lastStepMs = 0, settleUntil = 0;
unsigned long lastCmd = 0;
bool driving = false;
char line[32];
uint8_t len = 0;

void apply(uint8_t i, int p) {
  applied[i] = p;
  motor[i].setSpeed(abs(p));
  motor[i].run(p > 0 ? FORWARD : (p < 0 ? BACKWARD : RELEASE));
}
void releaseAll() { for (uint8_t i = 0; i < 4; i++) apply(i, 0); }

int pingOnce() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 24000UL);      // ~4 m
  if (us == 0) return 400;
  int cm = us / 58;
  return cm > 400 ? 400 : cm;
}
int ping() {                                                // median of 3 rejects single-ping noise
  int a = pingOnce(); delay(8); int b = pingOnce(); delay(8); int c = pingOnce();
  if (a > b) { int t = a; a = b; b = t; }                   // a <= b
  return c < a ? a : (c > b ? b : c);
}

bool parseM(const char *s, int *out) {
  char *end;
  for (uint8_t i = 0; i < 4; i++) {
    long v = strtol(s, &end, 10);
    if (end == s || v < -255 || v > 255) return false;
    out[i] = (int)v; s = end;
  }
  while (*s == ' ' || *s == '\r') s++;
  return *s == '\0';
}

void handle() {
  if (line[0] == '?') {
    Serial.println(F("WANDER2"));
  } else if (line[0] == 'M' && line[1] == ' ') {
    int p[4];
    if (parseM(line + 2, p)) {
      for (uint8_t i = 0; i < 4; i++) if (p[i] != applied[i]) apply(i, p[i]);
      lastCmd = millis(); driving = true;
    }
  } else if ((line[0] == 'U' || line[0] == 'G') && line[1] == ' ') {
    servoTarget = constrain(atoi(line + 2), 0, 180);        // the servo moves in loop(), so motors keep being served
    replyDeg = line[0] == 'U' ? servoTarget : -1;           // U answers when it has arrived, G is silent
    settleUntil = 0;
  } else if (line[0] == 'P') {
    Serial.print(F("D ")); Serial.println(ping());
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  releaseAll();
  servo.attach(SERVO_PIN);
  servo.write(servoDeg);
  Serial.println(F("WANDER2"));
}

void stepServo() {
  unsigned long now = millis();
  if (servoDeg != servoTarget) {
    if (now - lastStepMs >= SERVO_STEP_MS) {                // 1 degree per SERVO_STEP_MS
      lastStepMs = now;
      servoDeg += servoTarget > servoDeg ? 1 : -1;
      servo.write(servoDeg);
    }
  } else if (replyDeg >= 0) {
    if (settleUntil == 0) settleUntil = now + 120;          // let it settle before pinging
    else if (now >= settleUntil) {
      Serial.print(F("U ")); Serial.print(replyDeg); Serial.print(' '); Serial.println(ping());
      replyDeg = -1; settleUntil = 0;
    }
  }
}

void loop() {
  stepServo();
  if (driving && millis() - lastCmd > WATCHDOG_MS) { releaseAll(); driving = false; }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { line[len] = '\0'; if (len) handle(); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
    else len = 0;
  }
}
