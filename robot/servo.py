#!/usr/bin/env python3
"""Point the sonar servo at an angle and print the distance there. Run ON THE PI, from the folder with wander_bot.py.

    python3 servo.py 120     # servo to 120 deg (higher = more to the robot's LEFT), then one reading
    python3 servo.py 90 -n 5 # five readings

Flashes the wander firmware first if the Uno is not running it. The servo holds its angle after this exits, but the
Uno resets (and the servo returns to 90) the next time any program opens the port.
"""
import argparse

import wander_bot as wb

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('deg', type=int, help='servo angle, 0-180 (higher = left)')
ap.add_argument('-n', type=int, default=1, help='number of distance readings')
args = ap.parse_args()

robot = wb.Robot(dry=True)
robot.open()
cm = robot.look(max(0, min(180, args.deg)))
print(f'servo {args.deg} deg: ' + ('no answer' if cm is None else ('no echo' if cm >= 400 else f'{cm} cm')))
for _ in range(args.n - 1):
    cm = robot.ping()
    print('  ' + ('no answer' if cm is None else ('no echo' if cm >= 400 else f'{cm} cm')))
robot.ser.close()      # not robot.close(): that would flash the old firmware back
