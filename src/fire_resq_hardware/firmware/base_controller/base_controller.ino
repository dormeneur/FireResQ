// FireResQ BASE CONTROLLER - Arduino Uno + L293D motor shield (Adafruit Motor Shield v1, "AFMotor").
// The low-level half of the hardware boundary (Hardware.md section 8): it applies a power to each of the shield's
// four motor terminals and nothing else. No autonomy, no kinematics, no calibration, and no idea which terminal
// is which wheel: all of that lives on the Pi (base_node.py + config/robot.yaml), so re-wiring or tuning the
// robot never needs a re-flash.
//
// Serial, 115200 baud, one ASCII line per message:
//   Pi  -> Uno   M <m1> <m2> <m3> <m4>   signed power per shield terminal M1..M4, -255..255 (0 = released)
//                ?                       identify
//   Uno -> Pi    FIRERESQ_BASE <protocol> wd=<ms>     at boot and in answer to '?'
//                S <m1> <m2> <m3> <m4>   the power actually applied, every STATUS_MS
//
// SAFETY: a WATCHDOG. If no valid M line arrives for WATCHDOG_MS every motor is released, so a crashed driver, a
// dead Pi or a dropped Wi-Fi link stops the robot instead of leaving it driving. The Pi must therefore repeat its
// command continuously. The S line reports what is APPLIED, so the Pi sees a watchdog stop as zeros.
//
// Build: arduino-cli lib install "Adafruit Motor Shield library"
//        arduino-cli compile -b arduino:avr:uno src/fire_resq_hardware/firmware/base_controller
//
// Pins: the shield itself uses D3 D5 D6 D11 (PWM) and D4 D7 D8 D12 (shift register); D9/D10 are its servo
// headers. Free for later (e.g. the electromagnet's MOSFET): D2, D13, A0-A5.

#include <AFMotor.h>

const uint8_t MOTORS = 4;
AF_DCMotor motor[MOTORS] = {AF_DCMotor(1), AF_DCMotor(2), AF_DCMotor(3), AF_DCMotor(4)};

const unsigned long WATCHDOG_MS = 400;
const unsigned long STATUS_MS = 100;
const int PROTOCOL = 2;

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
  Serial.print(F("FIRERESQ_BASE "));
  Serial.print(PROTOCOL);
  Serial.print(F(" wd="));
  Serial.println(WATCHDOG_MS);
}

// "<m1> <m2> <m3> <m4>": exactly four numbers, each within -255..255, nothing after them. Anything else is
// rejected whole, so a corrupted line can never become a motion command.
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
  }
}

void setup() {
  Serial.begin(115200);
  releaseAll();
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
