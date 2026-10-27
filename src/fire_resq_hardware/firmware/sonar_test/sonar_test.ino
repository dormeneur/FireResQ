// FireResQ SONAR TEST - a bring-up sketch only, NOT the robot's firmware (that stays firmware/base_controller).
//
// Checks the ultrasonic ranger and its pan servo before either is wired into the real protocol. The pan servo sweeps
// slowly end to end. The shield's two servo headers are on D9 and D10, but which header is "SERVO1" is NOT certain
// (the Adafruit library's own example calls D9 servo1; Adafruit's guide, as remembered, says SERVO1 is D10), and an
// earlier version that drove only D9 never moved a servo that was probably on D10. So BOTH pins are driven with the
// same angle: whichever header the servo is plugged into moves. The sensor is meant to be TRIG = A5,
// ECHO = A0 - but this shield's A0-A5 header has NO PIN in the A0 column (bare holes; pins start at A1), so which
// pins the two wires are really on is the first thing to find out. This sketch does that instead of assuming:
//
//   1. "pullup" probe: every analog pin is sampled 20 times with its pull-up on. A sensor that is powered holds its
//      ECHO line driven LOW when idle, so the pin it is on is low on (nearly) every sample, while an unconnected or
//      input-only pin is high, and a loose wire picking up noise is low only occasionally. That identifies the ECHO
//      wire (and proves the sensor has power) without sending a single pulse. Shorting a pin to a Gnd-row pin with a
//      jumper also reads 20 of 20, which is how to find out which header position is really which pin.
//   2. Pings with every pin that is NOT driven low as TRIG, listening on the ECHO candidates, and reports any pair
//      that answers - confirmed by a second ping, so noise on a floating pin is not reported as a distance.
//
// A pin that reads low under pull-up is never driven as an output, so the sensor's own output is never fought.
//
// Run it with ./sonar_test.sh on the Pi (same folder): it flashes this, shows the output, and puts the real robot
// firmware back when you press Ctrl+C. The Uno's own LED (pin 13) blinks once per cycle with no wiring at all, so
// "is the new program actually running" can be answered by eye. It drives nothing else - no AFMotor, no protocol.
//
// Build:  arduino-cli compile -b arduino:avr:uno src/fire_resq_hardware/firmware/sonar_test

#include <Servo.h>

const uint8_t PINS[] = {A0, A1, A2, A3, A4, A5};
const char *const NAMES[] = {"A0", "A1", "A2", "A3", "A4", "A5"};
const uint8_t N = sizeof(PINS) / sizeof(PINS[0]);
const uint8_t SERVO_PIN_A = 9;                            // the shield's two servo headers; both are driven (see above)
const uint8_t SERVO_PIN_B = 10;

const int SWEEP_MIN_DEG = 10;                             // stay a little off the servo's mechanical end stops
const int SWEEP_MAX_DEG = 170;
const int SWEEP_STEP_DEG = 2;
const unsigned long SETTLE_MS = 60;                       // let the servo get there before measuring
const unsigned long PAUSE_AT_END_MS = 300;
const unsigned long ECHO_TIMEOUT_US = 12000UL;            // ~2 m of travel: plenty for a hand held 5-30 cm away
const unsigned long QUIET_PRINT_MS = 1000;                // with nothing heard, print only this often
const uint8_t SAMPLES = 20;                               // pull-up probe: samples per pin per cycle
const uint8_t STEADY_LOW = 18;                            // this many low samples = driven low, not noise

Servo sweeperA, sweeperB;
int angle = SWEEP_MIN_DEG;
int step = SWEEP_STEP_DEG;
unsigned long lastPrintMs = 0;
bool heartbeat = false;

// Ping with trigPin, listen on echoPin. Returns centimetres, or -1 if nothing came back. trigPin goes back to
// high-impedance straight afterwards, so a candidate that turns out to be wired to something else is never left driven.
float ping(uint8_t trigPin, uint8_t echoPin) {
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  pinMode(trigPin, INPUT);
  if (duration < 100) return -1.0;                        // nothing, or shorter than any real echo (2 cm)
  return duration / 58.0;                                 // the usual speed-of-sound/round-trip approximation
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);
  sweeperA.attach(SERVO_PIN_A);
  sweeperB.attach(SERVO_PIN_B);
  sweeperA.write(angle);
  sweeperB.write(angle);
  delay(500);
  Serial.println(F("FireResQ sonar test. Looking for the sensor's wires among A0..A5."));
  Serial.println(F("pullup: 0 = something is driving that pin LOW (a powered sensor's idle ECHO), 1 = nothing is."));
  Serial.println(F("FOUND trig=Ax echo=Ay <cm> = the sensor answered. Wave a hand 5-30 cm in front of it."));
}

void loop() {
  heartbeat = !heartbeat;
  digitalWrite(LED_BUILTIN, heartbeat);

  sweeperA.write(angle);
  sweeperB.write(angle);
  delay(SETTLE_MS);

  // 1. which pins look driven low? Each pin is sampled SAMPLES times with its pull-up on. A powered sensor holds its
  // idle ECHO line low all the time, so its pin reads low on (nearly) every sample. A floating or loosely connected
  // wire picking up noise reads low only now and then - that must NOT be mistaken for an echo.
  uint8_t lows[N];
  bool low[N];
  bool anyLow = false, flicker = false;
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT_PULLUP);
  delayMicroseconds(200);
  for (uint8_t i = 0; i < N; i++) lows[i] = 0;
  for (uint8_t k = 0; k < SAMPLES; k++) {
    for (uint8_t i = 0; i < N; i++) if (digitalRead(PINS[i]) == LOW) lows[i]++;
    delayMicroseconds(100);
  }
  for (uint8_t i = 0; i < N; i++) {
    low[i] = (lows[i] >= STEADY_LOW);
    if (low[i]) anyLow = true;
    else if (lows[i] > 0) flicker = true;
  }
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);

  // 2. ping: TRIG = any pin not driven low; ECHO = the pins that are (or, if none, every pin).
  bool found = false;
  char line[120];
  line[0] = '\0';
  for (uint8_t t = 0; t < N; t++) {
    if (low[t]) continue;
    for (uint8_t e = 0; e < N; e++) {
      if (e == t || (anyLow && !low[e])) continue;
      float cm = ping(PINS[t], PINS[e]);
      if (cm < 0) continue;
      delay(3);
      float again = ping(PINS[t], PINS[e]);                // a real pair answers twice, about the same
      if (again < 0 || fabs(again - cm) > 0.15 * cm) continue;
      found = true;
      Serial.print(F("angle="));
      Serial.print(angle);
      Serial.print(F("  FOUND trig="));
      Serial.print(NAMES[t]);
      Serial.print(F(" echo="));
      Serial.print(NAMES[e]);
      Serial.print(F("  "));
      Serial.print(cm, 1);
      Serial.println(F(" cm"));
    }
  }

  unsigned long now = millis();
  if (!found && now - lastPrintMs >= QUIET_PRINT_MS) {
    lastPrintMs = now;
    Serial.print(F("angle="));
    Serial.print(angle);
    Serial.print(F("  low samples out of "));
    Serial.print(SAMPLES);
    Serial.print(F(":"));
    for (uint8_t i = 0; i < N; i++) {
      Serial.print(' ');
      Serial.print(NAMES[i]);
      Serial.print('=');
      Serial.print(lows[i]);
    }
    if (anyLow) Serial.println(F("   -> steadily low: probably ECHO of a powered sensor"));
    else if (flicker) Serial.println(F("   -> only flickering: noise or a loose wire, NOT a steady echo"));
    else Serial.println(F("   -> all high: no powered sensor output on A0-A5"));
  }

  angle += step;
  if (angle >= SWEEP_MAX_DEG || angle <= SWEEP_MIN_DEG) {
    angle = constrain(angle, SWEEP_MIN_DEG, SWEEP_MAX_DEG);
    step = -step;
    delay(PAUSE_AT_END_MS);
  }
}
