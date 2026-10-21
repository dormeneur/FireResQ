#!/usr/bin/env python3
"""MOTION TEST - run ON THE PI from ~/rescue, motor switch ON, robot on the floor with ~50 cm free all round.

    python3 motion_test.py           # one move at a time: press Enter before each, watch the robot
    python3 motion_test.py --auto    # all moves, 2 s apart
    python3 motion_test.py --time 0.6 --power 200   # shorter / gentler moves

For every move it says what the robot SHOULD do, then what the camera measured it actually did: how far it turned
(+ = left, - = right; the scene in front of the camera slides sideways when the robot turns). A forward or backward
move should turn it ~0 deg. Compare with your eyes: if a "turn left" goes right, or "forward" goes backward, a
motor's wiring or FORWARD_SIGN is wrong; if a turn barely turns, that kind of turn does not work on this floor.
"""
import argparse
import math
import time

import rescue_bot as rb
import wander_bot as wb

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--auto', action='store_true', help='do not wait for Enter between moves')
ap.add_argument('--time', type=float, default=1.0, help='seconds per move (default 1.0)')
ap.add_argument('--power', type=int, default=255, help='motor power 0-255 (default 255)')
args = ap.parse_args()
P = args.power

# (name, what you should see, left side power, right side power) - + = that side drives forward
MOVES = [
    ('FORWARD', 'drives straight forward', P, P),
    ('BACKWARD', 'drives straight backward', -P, -P),
    ('PIVOT LEFT (right side forward)', 'turns LEFT, swinging around its stopped left wheels', 0, P),
    ('PIVOT RIGHT (left side forward)', 'turns RIGHT, swinging around its stopped right wheels', P, 0),
    ('PIVOT LEFT (left side backward)', 'turns LEFT, backing up on its left side', -P, 0),
    ('PIVOT RIGHT (right side backward)', 'turns RIGHT, backing up on its right side', 0, -P),
    ('SPIN LEFT (left back, right forward)', 'spins LEFT on the spot (may not work on tile)', -P, P),
    ('SPIN RIGHT (left forward, right back)', 'spins RIGHT on the spot (may not work on tile)', P, -P),
]

rb.stop_other_copies()
cam = rb.Camera()
cam.start()                                                         # also serves the live view on :8090
robot = rb.Uno(False)
robot.open()
robot.look(rb.SONAR_CENTER_DEG)
time.sleep(2)
print(f'\nMotion test: {args.time:.1f} s per move at power {P}. Camera at {cam.fps:.0f} fps.\n')
results = []
try:
    for name, expect, left, right in MOVES:
        if args.auto:
            time.sleep(2)
        else:
            input(f'Press Enter for {name} ...')
        y0 = cam.yaw
        print(f'  {name}: it should {expect}')
        wb.hold(robot, rb.STATUS, wb.drive_powers(left, right), args.time, name)
        wb.hold(robot, rb.STATUS, wb.STOP, 0.6, 'stopped')               # let the camera catch up
        turned = math.degrees(cam.yaw - y0)
        way = 'LEFT' if turned > 2 else 'RIGHT' if turned < -2 else 'no turn'
        print(f'  -> camera measured: turned {turned:+.0f} deg ({way})\n')
        results.append((name, turned))
finally:
    robot.close()
    rb.STOP_EVENT.set()

print('Summary (+ = left):')
for name, turned in results:
    print(f'  {name:40s} {turned:+6.0f} deg')
