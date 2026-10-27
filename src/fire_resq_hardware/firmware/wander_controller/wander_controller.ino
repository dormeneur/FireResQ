// FireResQ WANDER CONTROLLER - Arduino Uno + L293D motor shield (Adafruit Motor Shield v1, "AFMotor"),
// plus the HC-SR04 ultrasonic ranger and the MG90 servo that pans it.
// Same dumb-I/O idea as base_controller (the Pi decides everything, the Uno applies it), with one addition: it can point
// the servo and measure a distance. It announces itself as FIRERESQ_WANDER, NOT FIRERESQ_BASE, so the ROS base_node
// (which only talks to FIRERESQ_BASE 2) leaves this board alone. To go back to ROS: reflash base_controller.
//
// Serial, 115200 baud, one ASCII line per message:
//   Pi  -> Uno   M <m1> <m2> <m3> <m4>   signed power per shield terminal M1..M4, -255..255 (0 = released)
//                U <deg>                 turn the sonar SLOWLY to <deg> (0..180, 90 = straight ahead), then measure
//                ?                       identify
//   Uno -> Pi    FIRERESQ_WANDER 1 wd=<ms>   at boot and in answer to '?'
//                U <deg> <cm>            the answer to U, sent once the servo has ARRIVED: median of 3 pings;
//                                        400 = no echo (nothing within ~4 m)
//                S <m1> <m2> <m3> <m4>   the power actually applied, every STATUS_MS
//
// SAFETY: a WATCHDOG, as in base_controller. If no valid M line arrives for WATCHDOG_MS every motor is released.
// The servo moves one degree every SERVO_STEP_MS and never faster, without blocking the loop. A measurement blocks
// for at most ~75 ms, so the Pi's 10 Hz M/U cycle stays inside the 400 ms watchdog.
//
// Wiring (found on this robot with firmware/sonar_test, 2026-10-17): HC-SR04 TRIG = A3, ECHO = A4. The pan servo is on
// the shield's servo header; which header is SERVO1 is not certain, so BOTH D9 and D10 are driven with the same angle.
//
// Build: arduino-cli compile -b arduino:avr:uno src/fire_resq_hardware/firmware/wander_controller

#include <AFMotor.h>
#include <Servo.h>

const uint8_t MOTORS = 4;
AF_DCMotor motor[MOTORS] = {AF_DCMotor(1), AF_DCMotor(2), AF_DCMotor(3), AF_DCMotor(4)};

const uint8_t TRIG_PIN = A3;
const uint8_t ECHO_PIN = A4;
const uint8_t SERVO_PIN_A = 9;
const uint8_t SERVO_PIN_B = 10;
const unsigned long ECHO_TIMEOUT_US = 25000UL;   // ~4.3 m
const unsigned long WATCHDOG_MS = 400;
const unsigned long STATUS_MS = 100;
const int PROTOCOL = 1;
const int NO_ECHO_CM = 400;
const unsigned long SERVO_STEP_MS = 50;          // 1 degree per step = 20 degrees per second: slow on purpose

Servo panA, panB;
bool servoAttached = false;                      // attached on the first U command, so the servo never moves unasked
int panAngle = 90;                               // where the servo is (one degree at a time)
int panTarget = 90;
bool measurePending = false;                     // a U command is waiting for the servo to arrive
unsigned long lastStepMs = 0;

int applied[MOTORS] = {0, 0, 0, 0};
unsigned long lastCommandMs = 0, lastStatusMs = 0;
bool driving = false;

char line[32];
uint8_t lineLen = 0;
bool lineOverflow = false;

void apply(uint8_t i, int power) {
  applied[i] = power;
  motor[i].setSpeed(abs(power));
  motor[i].run(power > 0 ? FORWARD : (power < 0 ? BACKWARD : RELEASE));
}

void releaseAll() {
  for (uint8_t i = 0; i < MOTORS; i++) apply(i, 0);
}

void identify() {
  Serial.print(F("FIRERESQ_WANDER "));
  Serial.print(PROTOCOL);
  Serial.print(F(" wd="));
  Serial.println(WATCHDOG_MS);
}

int pingCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us < 100) return NO_ECHO_CM;                // nothing, or shorter than any real echo (under 2 cm)
  return (int)(us / 58);
}

int medianPingCm() {
  int a = pingCm(), b = pingCm(), c = pingCm();
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;                            // median of three
}

void sendMeasurement() {
  int cm = medianPingCm();
  Serial.print(F("U "));
  Serial.print(panAngle);
  Serial.print(' ');
  Serial.println(cm);
  measurePending = false;
}

void requestMeasurement(int deg) {
  panTarget = constrain(deg, 0, 180);
  if (!servoAttached) {                          // the one unavoidable fast move: from wherever it rests to 90
    panA.attach(SERVO_PIN_A);
    panB.attach(SERVO_PIN_B);
    panA.write(panAngle);
    panB.write(panAngle);
    servoAttached = true;
    lastStepMs = millis();
  }
  measurePending = true;
}

// One degree towards the target every SERVO_STEP_MS; measures when it gets there.
void stepServo(unsigned long now) {
  if (!measurePending) return;
  if (panAngle == panTarget) {
    sendMeasurement();
  } else if (now - lastStepMs >= SERVO_STEP_MS) {
    lastStepMs = now;
    panAngle += (panTarget > panAngle) ? 1 : -1;
    panA.write(panAngle);
    panB.write(panAngle);
  }
}

// "<m1> <m2> <m3> <m4>": exactly four numbers, each within -255..255, nothing after them. Anything else is rejected
// whole, so a corrupted line can never become a motion command.
bool parsePowers(const char *s, int *out) {
  char *end;
  for (uint8_t i = 0; i < MOTORS; i++) {
    long v = strtol(s, &end, 10);
    if (end == s || v < -255 || v > 255) return false;
    out[i] = (int)v;
    s = end;
  }
  while (*s == ' ' || *s == '\r') s++;
  return *s == '\0';
}

void handleLine() {
  if (line[0] == '?') {
    identify();
  } else if (line[0] == 'M' && line[1] == ' ') {
    int power[MOTORS];
    if (parsePowers(line + 2, power)) {
      for (uint8_t i = 0; i < MOTORS; i++) {
        if (power[i] != applied[i]) apply(i, power[i]);
      }
      lastCommandMs = millis();
      driving = true;
    }
  } else if (line[0] == 'U' && line[1] == ' ') {
    char *end;
    long deg = strtol(line + 2, &end, 10);
    if (end != line + 2 && deg >= 0 && deg <= 180) requestMeasurement((int)deg);
  }
}

void setup() {
  Serial.begin(115200);
  releaseAll();
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  identify();
}

void loop() {
  unsigned long now = millis();

  if (driving && now - lastCommandMs > WATCHDOG_MS) {
    releaseAll();
    driving = false;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLen] = '\0';
      if (!lineOverflow && lineLen > 0) handleLine();
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;                // too long to be ours: drop the whole line
    }
  }

  stepServo(now);

  if (now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    Serial.print('S');
    for (uint8_t i = 0; i < MOTORS; i++) {
      Serial.print(' ');
      Serial.print(applied[i]);
    }
    Serial.println();
  }
}
