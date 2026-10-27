#!/usr/bin/env python3
"""Run ON THE PI: reads the ultrasonic sensor wired DIRECTLY to the Pi's GPIO and prints the distance, so you can
check the wiring before any of it goes into the robot's software. Ctrl+C to stop. Nothing else is touched: this
does not use the Uno, the servo or the robot's Docker container.

    TRIG -> GPIO23 (physical pin 16)
    ECHO -> GPIO24 (physical pin 18)   THROUGH A VOLTAGE DIVIDER unless the module is a 3.3 V one (see below)

An ordinary HC-SR04 answers with a 5 V ECHO pulse. The Pi's GPIO pins are 3.3 V only and are NOT 5 V tolerant:
connecting a 5 V echo straight to one can destroy that pin, or the whole Pi. A divider (1 kOhm from ECHO to the GPIO
pin, 2 kOhm from the GPIO pin to GND) brings 5 V down to 3.3 V. A 3.3 V-capable module (HC-SR04P, or marked
3-5.5 V) powered from 3.3 V needs no divider.

    python3 pi_ultrasonic_test.py [trig_gpio echo_gpio]
"""
import sys
import time
import warnings

from gpiozero import DistanceSensor

TRIG = int(sys.argv[1]) if len(sys.argv) > 2 else 23
ECHO = int(sys.argv[2]) if len(sys.argv) > 2 else 24
MAX_M = 4.0

warnings.simplefilter('ignore')                      # gpiozero warns on every missed echo; we report that ourselves
sensor = DistanceSensor(echo=ECHO, trigger=TRIG, max_distance=MAX_M, queue_len=3)
print(f'Ultrasonic on the Pi: TRIG=GPIO{TRIG}, ECHO=GPIO{ECHO}. Wave a hand 5-30 cm in front of it. Ctrl+C to stop.')
try:
    while True:
        cm = sensor.distance * 100.0
        if cm >= MAX_M * 100.0 * 0.99:
            print('nothing heard within 4 m (or no echo at all: check power, TRIG/ECHO and the divider)')
        else:
            print(f'{cm:6.1f} cm')
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
finally:
    sensor.close()
