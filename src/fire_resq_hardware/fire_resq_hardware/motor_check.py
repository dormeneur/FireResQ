"""WIRING CHECK, run once on the Pi with the wheels OFF THE GROUND and the motor batteries in:

    ros2 run fire_resq_hardware motor_check            (stop base_node first: one program per serial port)

Spins shield terminals M1..M4 one at a time and asks which wheel moved and which way, then prints the three lines
to put in config/robot.yaml. No ROS: it must work when nothing else does.
"""
from __future__ import annotations

import argparse
import time

import serial

from .protocol import BAUD, MOTORS, PROTOCOL, STOP, format_drive, parse_banner

WHEELS = {'fl': 'front-left', 'fr': 'front-right', 'rl': 'rear-left', 'rr': 'rear-right'}


def ask(prompt: str, options) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in options:
            return answer
        print(f"  one of: {', '.join(options)}")


def wait_for_banner(ser: serial.Serial, timeout_s: float = 5.0) -> None:
    deadline, buf = time.monotonic() + timeout_s, b''
    while time.monotonic() < deadline:
        buf += ser.read(64)
        if any(parse_banner(line.decode('ascii', 'replace')) == PROTOCOL for line in buf.split(b'\n')):
            return
    raise SystemExit('The board did not identify as the FireResQ base firmware. Flash firmware/base_controller first.')


def spin(ser: serial.Serial, terminal: int, power: int, seconds: float) -> None:
    powers = [0] * MOTORS
    powers[terminal - 1] = power
    end = time.monotonic() + seconds
    while time.monotonic() < end:                                # repeated: the firmware watchdog needs feeding
        ser.write(format_drive(powers))
        time.sleep(0.1)
    ser.write(format_drive(STOP))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--power', type=int, default=200, help='0..255')
    ap.add_argument('--seconds', type=float, default=1.5)
    args = ap.parse_args()

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    wait_for_banner(ser)
    print('Wheels off the ground? Each terminal spins for '
          f'{args.seconds:g} s at power {args.power}.\n')
    found = {}
    try:
        for terminal in range(1, MOTORS + 1):
            while True:
                input(f'M{terminal}: press Enter to spin it ')
                spin(ser, terminal, args.power, args.seconds)
                wheel = ask('  which wheel turned? [fl / fr / rl / rr / none / again] ', list(WHEELS) + ['none', 'again'])
                if wheel != 'again':
                    break
            if wheel == 'none':
                print(f'  M{terminal}: nothing turned - check its wiring and the motor battery.')
                continue
            forward = ask('  did it turn the way that drives the robot FORWARD? [y / n] ', ['y', 'n'])
            found[terminal] = (wheel, 1 if forward == 'y' else -1)
    finally:
        ser.write(format_drive(STOP))
        ser.close()

    if len(found) != MOTORS or len({w for w, _ in found.values()}) != MOTORS:
        raise SystemExit(f'\nIncomplete or inconsistent ({found}): four terminals must map to four different wheels.')
    print('\nPut this in src/fire_resq_hardware/config/robot.yaml (base_node):')
    print(f"    left_motors: {[t for t, (w, _) in found.items() if w.endswith('l')]}")
    print(f"    right_motors: {[t for t, (w, _) in found.items() if w.endswith('r')]}")
    print(f'    motor_signs: {[found[t][1] for t in range(1, MOTORS + 1)]}')
    for t, (w, s) in found.items():
        print(f"    # M{t} = {WHEELS[w]}{'' if s == 1 else ' (reversed)'}")


if __name__ == '__main__':
    main()
