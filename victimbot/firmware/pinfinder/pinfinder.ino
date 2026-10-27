// VICTIMBOT PIN FINDER - finds which Uno pins the HC-SR04's TRIG and ECHO wires are on. A test sketch only.
//
// The sensor is plugged into the L293D shield somewhere; the free pins there are A0-A5 (the analog header), D2, D13
// and D9 (the SERVO 2 header). D10 (SERVO 1) holds the MG90S, which this sketch parks at its calibrated centre, 128.
//
//   1. LISTEN. Every candidate is read 40 times with its pull-up on. A powered HC-SR04 drives its ECHO line LOW when
//      idle, so that pin reads low (nearly) every time; a free or input-only pin (TRIG is an input on the sensor)
//      floats up to high. No pin that reads low is ever driven as an output, so the sensor's output is never fought.
//   2. PING (earlier versions; superseded by 3). Every other candidate in turn sends a 10 us trigger pulse; every low pin from step 1 is watched for the
//      echo (goes high, then low again, 0.1-25 ms later). A pair that answers 3 times out of 3 is reported.
//
// Output, every ~2 s, on 115200 baud:
//   LISTEN A0:40 A1:0 ...      (how many of 40 reads were low)
//   TIMED echo=A4 none:0 A0:0 A1:0 A2:0 A3:20 ...   (echoes that came 0.2-1.5 ms after a pulse from that pin)
// The TRIG pin is the one near 20; 'none' (no pulse sent) is the baseline for spontaneous echoes.
// The motors are never touched (no AFMotor). Hold a hand or a book ~20 cm in front of the sensor.

#include <Servo.h>

const uint8_t PINS[] = {A0, A1, A2, A3, A4, A5, 2, 13, 9};
const char *const NAMES[] = {"A0", "A1", "A2", "A3", "A4", "A5", "D2", "D13", "D9"};
const uint8_t N = sizeof(PINS) / sizeof(PINS[0]);
const uint8_t SERVO_PIN = 10;           // shield SERVO 1
const int SERVO_CENTER_DEG = 128;       // calibrated straight ahead
const uint8_t LISTEN_READS = 40;
const uint8_t LOW_NEEDED = 36;          // of LISTEN_READS: that pin is driven low = an ECHO candidate
const uint8_t TRIES = 3;

Servo servo;

// One ping from trig, watched on echo: the echo pulse width in us, or 0 if no clean pulse came back.
unsigned long pingPair(uint8_t trig, uint8_t echo) {
  pinMode(echo, INPUT);
  if (digitalRead(echo) != LOW) return 0;              // must be idle-low before the trigger
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW); delayMicroseconds(4);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 30000UL);     // waits for the rise, then times the high part
  pinMode(trig, INPUT_PULLUP);                         // back to listening
  pinMode(echo, INPUT_PULLUP);
  return (us >= 100 && us <= 25000UL) ? us : 0;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT_PULLUP);
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER_DEG);
  Serial.println(F("PINFINDER1"));
}

// Two pins wired together: pull one LOW (as an output) and the other, an input with its pull-up, reads LOW too. Only
// pins that float high under pull-up are ever driven, so nothing that drives a line itself is fought.
void bridges() {
  Serial.print(F("BRIDGED"));
  bool any = false;
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    delay(2);
    if (digitalRead(PINS[i]) == LOW) continue;         // something drives it (the ECHO): never drive it
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], LOW);
    delay(2);
    for (uint8_t j = i + 1; j < N; j++) {
      pinMode(PINS[j], INPUT_PULLUP);
      delayMicroseconds(200);
      if (digitalRead(PINS[j]) == LOW) {
        pinMode(PINS[i], INPUT_PULLUP);                // is it really i holding j low? release i and look again
        delayMicroseconds(500);
        bool follows = digitalRead(PINS[j]) == HIGH;
        pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], LOW);
        if (follows) { Serial.print(' '); Serial.print(NAMES[i]); Serial.print('='); Serial.print(NAMES[j]); any = true; }
      }
    }
    pinMode(PINS[i], INPUT_PULLUP);
  }
  Serial.println(any ? "" : " none");
}

// Step 3, for a pair that answered: a REAL trigger raises ECHO ~0.45 ms after the pulse (the sensor's 8-cycle burst),
// a spontaneous echo (some clones free-run while TRIG floats high) at a random time. Count, out of 8 pings, the echoes
// that rose 0.2-3 ms after this pin's pulse (each ping printed as rise_us/echo_cm, '-' = no echo), and, as a baseline, out of 8 waits without any pulse, the echoes that
// rose in the same window.
uint8_t timedHits(int8_t trig, uint8_t echo) {
  uint8_t hits = 0;
  Serial.print('[');
  for (uint8_t k = 0; k < 8; k++) {
    delay(40);
    pinMode(echo, INPUT);
    if (digitalRead(echo) != LOW) { Serial.print(F("H ")); continue; }
    if (trig >= 0) {
      pinMode(trig, OUTPUT);
      digitalWrite(trig, LOW); delayMicroseconds(4);
      digitalWrite(trig, HIGH); delayMicroseconds(10);
      digitalWrite(trig, LOW);
    }
    unsigned long t0 = micros();
    while (digitalRead(echo) == LOW && micros() - t0 < 30000UL) {}
    unsigned long rise = micros() - t0;
    if (trig >= 0) pinMode(trig, INPUT_PULLUP);
    if (rise >= 30000UL) { Serial.print(F("- ")); continue; }
    unsigned long t1 = micros();
    while (digitalRead(echo) == HIGH && micros() - t1 < 40000UL) {}
    Serial.print(rise); Serial.print('/'); Serial.print((micros() - t1) / 58); Serial.print(F("cm "));
    if (rise >= 200 && rise <= 3000) hits++;
  }
  Serial.print(']');
  pinMode(echo, INPUT_PULLUP);
  return hits;
}

// Two pins that both "trigger" the sensor: ping from one while the other is held firmly LOW. If the held pin is the
// real TRIG wire the sensor stays silent (a pulse that only leaked across from the neighbouring wire cannot lift a
// line that is driven); if the pinging pin is the real one it still answers.
void heldTest(uint8_t ping, uint8_t held, uint8_t echo) {
  uint8_t ok = 0;
  for (uint8_t k = 0; k < 8; k++) {
    pinMode(held, OUTPUT); digitalWrite(held, LOW);
    delay(40);
    pinMode(echo, INPUT);
    pinMode(ping, OUTPUT);
    digitalWrite(ping, LOW); delayMicroseconds(4);
    digitalWrite(ping, HIGH); delayMicroseconds(10);
    digitalWrite(ping, LOW);
    ok += pulseIn(echo, HIGH, 30000UL) > 100;
  }
  pinMode(ping, INPUT_PULLUP); pinMode(held, INPUT_PULLUP); pinMode(echo, INPUT_PULLUP);
  Serial.print(ok);
}

void loop() {
  bridges();
  Serial.print(F("HELD ping A2, A3 held low: ")); heldTest(A2, A3, A4);
  Serial.print(F("/8   ping A3, A2 held low: ")); heldTest(A3, A2, A4); Serial.println(F("/8"));
  uint8_t lows[N];
  Serial.print(F("LISTEN"));
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    lows[i] = 0;
  }
  delay(5);
  for (uint8_t r = 0; r < LISTEN_READS; r++) {
    for (uint8_t i = 0; i < N; i++) lows[i] += digitalRead(PINS[i]) == LOW;
    delay(2);
  }
  for (uint8_t i = 0; i < N; i++) {
    Serial.print(' '); Serial.print(NAMES[i]); Serial.print(':'); Serial.print(lows[i]);
  }
  Serial.println();

  for (uint8_t e = 0; e < N; e++) {
    if (lows[e] < LOW_NEEDED) continue;                // not driven low: cannot be a powered ECHO
    Serial.print(F("TIMED echo=")); Serial.print(NAMES[e]);
    Serial.print(F("\n none:")); Serial.print(timedHits(-1, PINS[e]));
    for (uint8_t t = 0; t < N; t++) {
      if (t == e || lows[t] > LISTEN_READS - LOW_NEEDED) continue;   // never drive a pin something holds low
      Serial.print(F("\n ")); Serial.print(NAMES[t]); Serial.print(':'); Serial.print(timedHits(PINS[t], PINS[e]));
    }
    Serial.println();
  }
  delay(500);
}
