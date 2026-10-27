#!/usr/bin/env python3
"""VICTIMBOT - a fire-rescue robot. Run ON THE PI, from the folder that holds dashboard.html and victimbot_fw.hex.

THE TASK. Victims (small SILVER METAL CYLINDERS standing on the floor) and FIRE (anything RED: the red box, the red
bag) are somewhere around the robot. Its start spot is the SAFE AREA. It runs until Ctrl+C:
    1. it WANDERS - mostly turning on the spot to look around, with short hops in between,
    2. when it sees victims it rescues the MOST DANGEROUS one first: the one closest to a fire (the same rule as the
       earlier rescue bot; with no fire known, the nearest one),
    3. it drives up to that victim until the magnet on its nose picks it up, checks it really did,
    4. carries it back to the safe area, STOPS FOR 5 SECONDS (take the victim off the magnet then),
    5. and goes back to wandering and rescuing, one victim at a time, for as long as it finds any.

    python3 -u victimbot.py                  # THE MISSION (5 s countdown first)
    python3 -u victimbot.py --dry            # the same, but no motor command is ever sent (robot can sit on a desk)
    python3 -u victimbot.py --view           # camera + detection on the dashboard, nothing moves
    python3 -u victimbot.py --plan           # look once, decide, explain - never moves
    python3 -u victimbot.py --moves          # wheels OFF THE GROUND: forward, back, left, right, 1 s each
    python3 -u victimbot.py --sonar          # servo + sonar only: distances at 5 angles, nothing drives
    python3 -u victimbot.py --calibrate-horizon 0.50   # ONE victim 0.50 m straight ahead (lens to the cylinder)
    python3 -u victimbot.py --selftest       # detector, decision, routes, motor table and turning - no hardware

DASHBOARD: http://pi.local:8090 - the same page as the earlier rescue bot: live camera, map, actions, thinking.

MOTORS: exactly what the car-control page (~/car-control) sends with the settings in ~/fireresq/motor_config.md,
with the robot's FRONT and BACK swapped and so LEFT and RIGHT swapped too (the camera end is the front):
    forward    = the page's S (backward): M -255 -255 -255 -255
    backward   = the page's W (forward):  M +255 +255 +255 +255
    turn LEFT  = the page's D (turn right): M 0 0 -255 -255
    turn RIGHT = the page's A (turn left):  M -255 -255 0 0
(The page multiplies each wheel's turn effort by its side, +1 left / -1 right, and THEN applies "reverse": reading the
table without that gave turns with one wheel the wrong way round, which did not turn the robot at all.)
SONAR (HC-SR04 on the MG90S servo, SERVO 1 = D10, straight ahead = 128 deg): TRIG = A3, ECHO = A4, found by
firmware/pinfinder on this robot.

HOW IT SEES. The camera (OV5647 ribbon camera, mounted turned 90 deg clockwise: every frame is turned back first) sits
CAM_HEIGHT_M above a flat floor. Where an object meets the floor N pixels below the horizon row it is
CAM_HEIGHT_M * F_PX / N metres away; its column gives the bearing.
    VICTIM = a narrow upright shape on the floor, darker than the floor beside it (the cylinder's body and its shadow:
             measured 20-55 grey levels under the floor), with no colour (silver: saturation like the floor's), with
             floor on both sides of it and real-world size of a small cylinder. Shoes, gaps under furniture and the
             thin seams between floor tiles all fail one of those.
    FIRE   = every saturated RED blob standing on the floor (the box and the bag: hue 0-3 / 179, saturation ~220).
MOVING (no encoders): turns are pulses with a look in between, and the camera measures how far the robot really
turned (the picture's shift between frames); straight legs hold that heading; distance is time x speed, and the speed
is re-learned from how fast a victim's range shrinks while homing on it. Back at the safe area the fire, if it was
seen from there at the start, re-anchors the position. Map frame = the start pose: x forward, y left, angles CCW.
"""
import argparse
import json
import math
import os
import queue
import random
import signal
import socket
import statistics
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

STOP_EVENT = threading.Event()           # Ctrl+C / SIGTERM / SIGHUP: every loop below must notice it
HERE = Path(__file__).resolve().parent

# ---- camera -----------------------------------------------------------------------------------------------------
CAM_SIZE = (960, 720)                    # what the sensor delivers (it is mounted on its side)
ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE  # undo the mounting: everything below works on the upright picture
W, H = 720, 960                          # the upright picture
HFOV_DEG = 41.4                          # horizontal FOV of the UPRIGHT picture (the sensor's short side)
F_PX = (W / 2) / math.tan(math.radians(HFOV_DEG / 2))
CAM_HEIGHT_M = 0.10                      # lens above the floor. TODO(measure); --calibrate-horizon absorbs most of it
HORIZON_Y = 156                          # the horizon row: the camera looks ~19 deg DOWN. ESTIMATED 2026-10-26 from the
#                                          red shoebox (~35 cm long, 293 px wide -> ~1.1 m away; its base at row 242).
#                                          --calibrate-horizon measures it properly
CAL_FILE = HERE / 'calibration.json'
try:
    HORIZON_Y = json.loads(CAL_FILE.read_text())['horizon_y']
except (OSError, ValueError, KeyError):
    pass
CAMERA_FPS = 10
WEB_PORT = 8090
DASHBOARD_HTML = HERE / 'dashboard.html'
VO_MIN_CONF = 0.1                        # frame-to-frame match quality below this is ignored (motion blur)
VO_MAX_BAD = 8                           # ...for at most this many frames in a row
VO_ROWS = (0, int(H * 0.8))              # band used to measure turning: everything but the bottom 20 %, where the
#                                          MAGNET sits in the picture - it turns with the camera, so it reads as "no turn"

# ---- what to look for (HSV, OpenCV: H 0-179), measured from this camera on 2026-10-26 ------------------------------
# THE VICTIMS: toy DC motors standing upright, EACH MARKED WITH A V-SHAPED YELLOW-GREEN STICKY NOTE (the maintainer
# added them on 2026-10-26: the bare motors' colour changed with the light and the tile seams looked like them).
FLAG_HSV = ((30, 90, 90), (62, 255, 255))    # yellow-green; the cardboard boxes are more orange (H ~20-28)
FLAG_MIN_AREA = 10                       # px in the half-size picture
FLAG_BASE_FRAC = 0.45                    # the motor under the note: its floor contact is this x the note's height lower
MAGNET_ROW = int(H * 0.86)               # below this row the picture shows the magnet itself
FLOOR_MIN_V, FLOOR_MAX_S = 85, 100       # what counts as floor (under a fire)
# THE HORIZON FROM THE VICTIMS. Every victim is the same motor, so for each one standing on the floor
#     (floor contact row - horizon row) = (CAM_HEIGHT_M / VICTIM_HEIGHT_M) x (its height in pixels).
# Every look re-measures the horizon from the victims in view: tilting the camera (its mount moves) changes nothing.
VICTIM_HEIGHT_M = 0.05                   # the sticky note's height (Blob.h of a victim is the note's). TODO(measure)
HORIZON_MIN_PX = 10                      # victims shorter than this in the picture are too small to measure it
RED_HSV = (((0, 170, 60), (4, 255, 255)), ((174, 170, 60), (179, 255, 255)))   # ONLY the Nike box's pure, vivid red
#                                          (H 0-1, S 219-236): the reddish-brown door and wood are more orange, duller
FIRE_MIN_AREA = 600                      # px: a red speck is not a fire
FIRE_MIN_WIDTH_PX = 25
MAX_RANGE_M = 3.0                        # ignore anything further away than this
BOX_COLOUR = {'fire': (0, 0, 255), 'victim': (255, 255, 0)}

# ---- motors (see the module docstring) ----------------------------------------------------------------------------
LEFT_SIDE = (0, 3)                       # M1 rear-left, M4 front-left    (indices into M1..M4)
RIGHT_SIDE = (1, 2)                      # M2 rear-right, M3 front-right
FORWARD_SIGN = -1                        # a NEGATIVE power drives every wheel forward
MAX_POWER = 255
CRUISE = 255                             # forward power for legs and homing: the config's 100 %
CRUISE_MPS = 0.20                        # TODO(calibrate): first guess; re-learned while homing on a victim
DRIVE_DEAD_S = 0.5                       # the motors do nothing for about this long after a command starts (measured
#                                          on turns: ~0.6 s)
SPIN_POWER = 255                         # the controller's turns are at 100 %
# EXACTLY what the car-control page (~/car-control/index.html) sends with the motor_config.md settings, swapped:
#   the page's D (turn right): RL 100 reversed, RR 100 -> wheel values x side (+1 left, -1 right), then "reverse"
#     -> RL -255, RR -255 -> M3 -255, M4 -255.               This robot's turn LEFT.
#   the page's A (turn left):  FL 100, FR 100 reversed -> FL -255, FR -255 -> M2 -255, M1 -255. This robot's turn RIGHT.
SPINS = {True: (0, 0, -SPIN_POWER, -SPIN_POWER),
         False: (-SPIN_POWER, -SPIN_POWER, 0, 0)}
STOP = (0, 0, 0, 0)
# Measured 2026-10-26 on the floor: a turn does NOTHING for ~0.6 s, then the robot whips round at ~100 deg/s (a 1 s
# hold turned it ~40 deg). A "has it started moving?" trigger does not work: the motors kicking in shake the camera
# before the robot turns. So a pulse is simply the dead time plus the part that turns, measured from two stills.
SPIN_DEAD_S = 0.6
SPIN_MAX_S = 0.95                        # longest single pulse (~35 deg): the turn must stay measurable between stills
SPIN_MIN_S = 0.65
SPIN_RATE_DPS = 100.0                    # deg/s once moving: first guess, re-learned every pulse
STILL_MIN_CONF = 0.15                    # a before/after still match better than this measures the turn directly
ORB_MIN_MATCHES = 12                     # ...else this many corners agreeing on one shift do
MOVING_CONF = 0.25                       # frame-to-frame match below this = heavy blur (the camera reports it)
TURN_TOL_DEG = 8.0                       # each pulse turns ~10-25 deg: a finer aim only oscillates
SPIN_AIM = 0.8                           # aim each pulse at this fraction of what is left (then measure, add more)
STALL_DEG = 1.0                          # a pulse that turned less than this did not move the robot
MAX_STALLS = 4
MAX_SPINS = 24
K_HEAD = 150                             # steering power per radian of heading error on straight legs
SETTLE_S = 0.35                          # stop this long before trusting a frame
CYCLE_S = 0.06                           # control loop period (the Uno's watchdog is 400 ms)

# ---- serial / Uno -------------------------------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'
BAUD = 115200
BANNER = 'VICTIMBOT1'
FW_HEX = HERE / 'victimbot_fw.hex'

# ---- sonar --------------------------------------------------------------------------------------------------------
SONAR_CENTER_DEG = 128                   # the servo's calibrated straight ahead (SERVO_CENTER_DEG in the firmware)
SONAR_LEFT_SIGN = 1                      # higher servo angle = the robot's LEFT (as on the earlier bot). --sonar checks
SWEEP_DEG = (-40, -20, 0, 20, 40)        # relative to centre, + = left
OBSTACLE_CM = 20                         # something this close ahead (and closer than the target) = stop
SONAR_MIN_CM = 4                         # nearer than this is the victim on the magnet or a bad echo, not an obstacle
CLEAR_CM = 45                            # a direction needs this much room to be worth a wander hop
# The sonar is ONLY for obstacles (stopping legs, choosing where to hop). Victims and fire: the camera alone.

# ---- mission ------------------------------------------------------------------------------------------------------
LOOK_FRAMES = 5                          # photos per look: a victim must be in most of them
MERGE_M = 0.10                           # sightings closer than this (across photos) are the same victim
SAME_VICTIM_M = 0.18                     # a sighting this close to a remembered victim IS that victim (one-to-one)
SAME_FIRE_M = 0.40
SAFE_RADIUS_M = 0.30                     # a victim inside this of the start is in the safe area: already saved
FIRE_CLEAR_M = 0.20                      # routes keep this far from a fire's base edge...
VICTIM_CLEAR_M = 0.15                    # ...and from victims that are not the target
OBSTACLE_CLEAR_M = 0.25                  # ...and from an obstacle the sonar found
MAX_REROUTES = 3
FIRE_STOP_M = 0.15                       # a fire straight ahead, this close and nearer than the target: stop
HOP_M = 0.20                             # homing: drive at most this far between still pictures
CENTER_TOL_DEG = 8                       # ...and only once the victim is within this of the picture's centre
MAX_HOPS = 40
CLOSE_LOST_M = 0.40                      # a victim last seen this close and then out of view is right in front
TRACK_GATE_DEG = 12                      # victims look alike: a tracked one must stay this close to where it was
ACQUIRE_GATE_DEG = 25                    # ...and this close to the map's prediction when homing starts
CLOSE_FRAC = 0.88                        # the victim's floor contact this low in the picture = right in front (~14 cm
#                                          from the lens). Lower down, the magnet itself (the dark arc across the bottom
#                                          of the picture) hides it
MAGNET_REACH_M = 0.08                    # how far ahead of the lens (along the floor) the magnet's face is.
#                                          TODO(measure): raise if the robot stops short of the victim, lower if it
#                                          shoves the victim away
TOUCH_PUSH_M = 0.02                      # drive this much past "just touching", so the magnet really meets it
VERIFY_BACK_M = 0.10                     # after the touch: back off this far and look - a victim still on the floor
VERIFY_NEAR_M = 0.45                     # right in front (nearer than this) was NOT picked up
MAX_TRIES = 2                            # failed rescues of one victim before its spot is ignored
SAFE_STOP_S = 5                          # at the safe area: stop this long (take the victim off the magnet)
HOME_TOL_M = 0.12
MAX_FIX_M = 0.6                          # a fire-based correction bigger than this is a mis-sighting: ignored
# wandering: MORE TURNS THAN DRIVING. A few turns on the spot (each one followed by a look), then one short hop.
WANDER_TURN_DEG = 40
WANDER_TURNS = (2, 4)                    # turns between hops (random in this range)
WANDER_HOP_M = 0.25
WANDER_RADIUS_M = 2.0                    # further than this from the safe area: hops lean back towards it
COUNTDOWN_S = 5


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- the dashboard's data (the same JSON the earlier rescue bot's dashboard.html reads) ----------------------------
class Dash:
    """say() = what the robot is DOING (Actions panel), think() = WHY (Thinking panel). Both print to the terminal."""

    def __init__(self):
        self.lock = threading.Lock()
        self.log, self.thoughts = [], []
        self.objs = []                   # [{name, kind, x, y, rank, reached}] (+ ax, ay, bx, by: a fire's base)
        self.hazards = []                # [[x, y, clearance, name]]
        self.path = []
        self.trail = [[0.0, 0.0]]
        self.pose = [0.0, 0.0, 0.0]
        self.target = None
        self.state = 'starting'

    def _add(self, feed, msg):
        print(msg, flush=True)
        with self.lock:
            feed.append([time.strftime('%H:%M:%S'), msg])
            del feed[:-300]

    def say(self, msg):
        self._add(self.log, msg)

    def think(self, msg):
        self._add(self.thoughts, msg)

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def moved(self, x, y, th):
        with self.lock:
            self.pose = [x, y, th]
            if math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.02:
                self.trail.append([x, y])
                del self.trail[:-2000]

    def json(self):
        with self.lock:
            return json.dumps({k: getattr(self, k) for k in
                               ('log', 'thoughts', 'objs', 'hazards', 'path', 'trail', 'pose', 'target', 'state')})


DASH = Dash()


# ---- perception -------------------------------------------------------------------------------------------------
def floor_range(row):
    """Floor range (m) of the point where an object meets the floor at picture row `row`; None at/above the horizon or
    beyond MAX_RANGE_M."""
    drop = row - HORIZON_Y
    r = CAM_HEIGHT_M * F_PX / drop if drop >= 4 else None
    return r if r is not None and r <= MAX_RANGE_M else None


def bearing_of(col):
    return math.atan2(W / 2 - col, F_PX)                            # rad, + = left of the camera axis


class Blob:
    """A fire or victim in the upright picture. (x, w) span its BASE; `bottom` is the row where it meets the floor.
    `clipped`: it runs off the bottom of the picture (right in front of the robot), so its range is not trusted."""

    def __init__(self, kind, x, y, w, bottom, clipped=False, h=None):
        self.kind, self.x, self.y, self.w, self.bottom, self.clipped = kind, x, y, w, bottom, clipped
        self.h = bottom - y if h is None else h                     # a victim: its can's height, not the cap
        self.bearing = bearing_of(x + w / 2)
        self.b_left, self.b_right = bearing_of(x), bearing_of(x + w)  # the base's two ends (a fire is wide)

    @property
    def range(self):
        return floor_range(self.bottom)                             # with the CURRENT horizon (it is re-measured)

    def label(self):
        r = '?' if self.range is None else f'{self.range:.2f}m'
        return f'{self.kind} {r} {math.degrees(self.bearing):+.0f}'


def is_floor(hsv, y0, y1, x0, x1, min_v=None):
    """Is this patch floor: without colour, and at least min_v bright (default FLOOR_MIN_V)? False off the picture."""
    y0, y1, x0, x1 = max(0, y0), min(H, y1), max(0, x0), min(W, x1)
    if y1 - y0 < 1 or x1 - x0 < 2:
        return False
    p = hsv[y0:y1, x0:x1]
    return np.median(p[..., 2]) >= (FLOOR_MIN_V if min_v is None else min_v) and np.median(p[..., 1]) <= FLOOR_MAX_S


def red_mask(hsv):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RED_HSV:
        m |= cv2.inRange(hsv, lo, hi)
    return m


def find_fires(hsv, red):
    """Every red blob big enough to be a fire. Its base: the blob's lowest row (a red box meets the floor directly; the
    pink glare on the glossy floor below it is not red); its two ends: the blob's full width (a white logo near the
    bottom of the box breaks up its lowest rows). The
    size test is in PIXELS, not metres, so a wrong horizon cannot hide a fire (the sonar re-measures the horizon FROM
    a fire); a fire above the horizon is still found, just without a range."""
    out = []
    m = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(c) < FIRE_MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w < FIRE_MIN_WIDTH_PX:
            continue
        out.append(Blob('fire', x, y, w, y + h, clipped=y + h >= H - 2))
    return out


def find_victims(bgr, red, why=None):
    """VICTIMS ARE MARKED: each toy motor carries a V-shaped yellow-green sticky note (2026-10-26), a colour nothing else
    in the room has. A victim = a note; its BEARING is the point of the V (the bottom of the note, right over the
    motor); its floor contact is under the motor, FLAG_BASE_FRAC x the note's height below the note.
    why: collects (x, y, w, h, reason) for candidates turned down."""
    small = cv2.resize(bgr, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    m = cv2.inRange(cv2.cvtColor(small, cv2.COLOR_BGR2HSV), *FLAG_HSV)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # the V's two arms in one piece
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < FLAG_MIN_AREA:
            if why is not None and area >= 3:
                why.append((2 * x, 2 * y, 2 * w, 2 * h, 'too small'))
            continue
        if y + h >= MAGNET_ROW // 2:                                # the magnet's area: carried, not standing
            if why is not None:
                why.append((2 * x, 2 * y, 2 * w, 2 * h, 'on the magnet'))
            continue
        rows = labels[y + h - max(1, h // 4):y + h, x:x + w] == i    # the point of the V: its lowest quarter
        cols = np.nonzero(rows.any(axis=0))[0]
        tip = x + (cols.min() + cols.max()) / 2 if cols.size else x + w / 2
        X, Y, Wf, Hf = 2 * x, 2 * y, 2 * w, 2 * h
        contact = min(H - 1, int(round(Y + Hf * (1 + FLAG_BASE_FRAC))))
        b = Blob('victim', int(2 * tip) - 6, Y, 12, contact, clipped=contact >= MAGNET_ROW, h=Hf)
        b.bearing = bearing_of(2 * tip)
        b.score = float(area)
        b.flag = (X, Y, Wf, Hf)
        out.append(b)

    def reflection(b):
        """The glossy floor mirrors a note just below it (measured 2026-10-28: an h~20 px blob ~8 px under every
        note): a smaller blob straight under a bigger one, closer than the bigger one's height, is that reflection."""
        X, Y, Wf, _ = b.flag
        return any(o.score > b.score and o.flag[0] < X + Wf and X < o.flag[0] + o.flag[2]
                   and 0 <= Y - (o.flag[1] + o.flag[3]) < o.flag[3] for o in out)
    return [b for b in out if not reflection(b)]


def detect(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    red = red_mask(hsv)
    fires = find_fires(hsv, red)
    victims = find_victims(bgr, red)        # red pixels are already left out: a victim on the fire's glare still counts
    return fires + victims


def rotation(prev, cur):
    """(rad, confidence): how far the camera turned between two grey bands (+ = left: the scene slides right when the
    camera turns left). The bands are at half resolution."""
    (dx, _), conf = cv2.phaseCorrelate(prev, cur)
    return math.atan2(2 * dx, F_PX), conf


def turn_between(before, after):
    """(rad, ok): how far the robot turned between two STILL turn-measuring bands (+ = left). ORB corners matched
    between the two pictures give the shift as long as they still overlap (up to ~35 deg). Phase correlation is the
    fallback, only for small shifts: past ~30 % of the width it aliases - a +23 deg turn read as -19 deg, confidently."""
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))       # the room is dim: lift the contrast first
    a, b = (clahe.apply(np.uint8(np.clip(g, 0, 255))) for g in (before, after))
    orb = cv2.ORB_create(800, fastThreshold=8, edgeThreshold=15)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is not None and db is not None:
        shifts = [kb[m.trainIdx].pt[0] - ka[m.queryIdx].pt[0]
                  for m in cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
                  if abs(kb[m.trainIdx].pt[1] - ka[m.queryIdx].pt[1]) < 12]   # a turn moves things sideways only
        if len(shifts) >= ORB_MIN_MATCHES:
            dx = statistics.median(shifts)
            if sum(abs(x - dx) < 6 for x in shifts) >= ORB_MIN_MATCHES:      # most must agree on one shift
                return math.atan2(2 * dx, F_PX), True
    (dx, _), conf = cv2.phaseCorrelate(before, after)
    if conf >= STILL_MIN_CONF and abs(dx) < 0.3 * before.shape[1]:
        return math.atan2(2 * dx, F_PX), True
    return 0.0, False


class DualStackServer(ThreadingHTTPServer):
    """IPv6 AND IPv4: a PC that resolves pi.local to the Pi's IPv6 address is otherwise refused."""
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


class Camera:
    """Grabs, turns upright and detects continuously; integrates the heading change frame to frame (`yaw`); serves the
    dashboard: / (page), /stream (MJPEG), /state (JSON). snap() hands out the first frame captured after a given time,
    so nothing acts on a picture taken before the robot stopped."""

    def __init__(self):
        self.cond = threading.Condition()
        self.t, self.blobs, self.jpeg, self.band = 0.0, [], None, None
        self.blur_run = 0                # frames in a row too blurred to match: the robot is whipping round
        self.fast = False                # set while turning: skip detection so the turn is tracked at a higher rate
        self.yaw = 0.0                   # rad, + = left, summed over consecutive frames since start
        self.fps = 0.0
        self._gray = None
        self._bad = 0                    # blurred frames in a row since the last one that matched

    def start(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        # queue=False: every capture is a frame exposed after the request (a queued one can be up to a second stale)
        cam.configure(cam.create_video_configuration(main={'size': CAM_SIZE, 'format': 'RGB888'},  # BGR in memory
                                                     queue=False, buffer_count=2))
        cam.start()
        threading.Thread(target=self._grab, args=(cam,), daemon=True).start()
        start_web()

    def _grab(self, cam):
        n, t_fps = 0, time.time()
        while not STOP_EVENT.is_set():
            t = time.time()                                         # stamp BEFORE the capture: conservative
            try:
                frame = cv2.rotate(cam.capture_array(wait=3.0), ROTATE)
            except Exception as e:                                  # a frontend timeout (power dip): carry on
                print(f'camera: {e}', flush=True)
                continue
            gray = np.float32(cv2.resize(cv2.cvtColor(frame[VO_ROWS[0]:VO_ROWS[1]], cv2.COLOR_BGR2GRAY), None,
                                         fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
            if self._gray is None:
                self._gray = gray
            else:
                d, conf = rotation(self._gray, gray)
                self.blur_run = self.blur_run + 1 if conf < MOVING_CONF else 0   # heavy blur = a real, fast turn
                if conf >= VO_MIN_CONF:
                    self.yaw += d                                   # a float store: atomic for the readers
                    self._gray, self._bad = gray, 0
                else:                                               # blurred mid-turn: keep the last sharp frame as
                    self._bad += 1                                  # reference, so the turn counts once sharp again;
                    if self._bad >= VO_MAX_BAD:                     # but a scene that never matches again (turned too
                        self._gray, self._bad = gray, 0             # far in the blur) has to be let go
            blobs = [] if self.fast else detect(frame)              # mid-turn: no detecting, twice the frame rate
            draw_blobs(frame, blobs)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with self.cond:
                self.t, self.blobs, self.band = t, blobs, gray
                if ok:
                    self.jpeg = jpg.tobytes()
                self.cond.notify_all()
            n += 1
            if time.time() - t_fps > 5:
                self.fps, n, t_fps = n / (time.time() - t_fps), 0, time.time()
            time.sleep(max(0.0, 1.0 / CAMERA_FPS - (time.time() - t)))
        cam.stop()

    def snap_band(self, after, timeout=3.0):
        """The turn-measuring band (grey, half size) of the first frame captured at or after `after`, or None."""
        with self.cond:
            self.cond.wait_for(lambda: self.t >= after, timeout)
            return self.band if self.t >= after else None

    def snap(self, after, timeout=3.0):
        """Blobs of the first frame captured at or after `after`; [] if none arrives in time."""
        with self.cond:
            self.cond.wait_for(lambda: self.t >= after, timeout)
            return list(self.blobs) if self.t >= after else []


def draw_blobs(frame, blobs):
    for b in blobs:
        c = BOX_COLOUR[b.kind]
        if hasattr(b, 'flag'):                                      # the note too, and the point it marks
            fx, fy, fw, fh = b.flag
            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 1)
        cv2.rectangle(frame, (b.x - 3, b.y - 3), (b.x + b.w + 3, b.bottom + 3), c, 2)
        cv2.putText(frame, b.label(), (max(0, b.x - 10), max(14, b.y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2,
                    cv2.LINE_AA)
    cv2.line(frame, (0, int(HORIZON_Y)), (W, int(HORIZON_Y)), (90, 90, 90), 1)


CAMERA = None


def start_web():
    camera = CAMERA

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/state':
                self._send(DASH.json().encode(), 'application/json')
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while not STOP_EVENT.is_set():
                        with camera.cond:
                            camera.cond.wait(timeout=2.0)
                            jpg = camera.jpeg
                        if jpg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                             % len(jpg) + jpg + b'\r\n')
                except OSError:
                    pass
            else:
                self._send(DASHBOARD_HTML.read_bytes(), 'text/html; charset=utf-8')   # re-read: edit it live

    server = DualStackServer(('::', WEB_PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:                                                            # the Pi's LAN address (no packet is sent)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as u:
            u.connect(('10.255.255.255', 1))
            ip = u.getsockname()[0]
    except OSError:
        ip = 'the-pi-address'
    print(f'DASHBOARD: open http://pi.local:{WEB_PORT}  or  http://{ip}:{WEB_PORT}  in a browser on your PC',
          flush=True)


# ---- the Uno ------------------------------------------------------------------------------------------------------
def drive_powers(left, right):
    """Left/right side power (+ forward, - backward) -> signed (M1..M4) powers for the Uno."""
    out = [0] * 4
    for side, motors in ((left, LEFT_SIDE), (right, RIGHT_SIDE)):
        for m in motors:
            out[m] = FORWARD_SIGN * max(-MAX_POWER, min(MAX_POWER, round(side)))
    return tuple(out)


class Uno:
    """The Uno over serial: M/U/G/P lines out, U/D answers in (a reader thread keeps the port drained). Flashes
    victimbot_fw.hex if the Uno does not answer as VICTIMBOT1. dry: no motor command is ever sent."""

    def __init__(self, dry=False):
        self.dry = dry
        self.ser = None
        self.answers = queue.Queue()

    def open(self):
        import serial
        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        time.sleep(2.0)                                             # opening the port resets the Uno
        if not self._banner():
            print('Uploading the victimbot firmware to the Uno...', flush=True)
            self.ser.close()
            r = subprocess.run(['avrdude', '-p', 'atmega328p', '-c', 'arduino', '-P', PORT, '-b', '115200',
                                '-U', f'flash:w:{FW_HEX}:i'], capture_output=True, text=True)
            if r.returncode != 0:
                sys.exit(f'Flashing {FW_HEX.name} failed:\n' + r.stderr[-600:])
            self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
            time.sleep(2.0)
            if not self._banner():
                sys.exit(f'Uploaded, but the Uno does not answer as {BANNER}.')
        threading.Thread(target=self._reader, daemon=True).start()

    def _banner(self):
        self.ser.reset_input_buffer()
        self.ser.write(b'?\n')
        end = time.time() + 1.5
        while time.time() < end:
            if self.ser.readline().decode('ascii', 'replace').startswith(BANNER):
                return True
        return False

    def _reader(self):
        import serial
        while self.ser and self.ser.is_open:
            try:
                parts = self.ser.readline().decode('ascii', 'replace').split()
            except (serial.SerialException, OSError, TypeError):
                return
            if len(parts) == 3 and parts[0] == 'U' and parts[1].isdigit() and parts[2].isdigit():
                self.answers.put(('U', int(parts[1]), int(parts[2])))
            elif len(parts) == 2 and parts[0] == 'D' and parts[1].isdigit():
                self.answers.put(('D', 0, int(parts[1])))

    def drive(self, powers):
        """Must be repeated every cycle: the Uno releases the motors after 400 ms of silence."""
        if not self.dry and self.ser:
            self.ser.write(('M %d %d %d %d\n' % tuple(powers)).encode())

    def _ask(self, line, kind, deg, timeout):
        while not self.answers.empty():
            self.answers.get_nowait()
        self.ser.write(line.encode())
        end = time.time() + timeout
        while time.time() < end and not STOP_EVENT.is_set():
            try:
                got_kind, got_deg, cm = self.answers.get(timeout=0.05)
            except queue.Empty:
                continue
            if got_kind == kind and got_deg == deg:
                return cm
        return None

    def ping(self):
        """Distance where the sonar points, cm (400 = no echo), or None if the Uno did not answer."""
        return self._ask('P\n', 'D', 0, 0.4)

    def look(self, deg):
        """Point the sonar at deg (servo angle) and return the distance there (cm), or None."""
        return self._ask(f'U {deg}\n', 'U', deg, 5.0)

    def aim(self, deg):
        if self.ser:
            self.ser.write(f'G {deg}\n'.encode())

    def close(self):
        if self.ser and self.ser.is_open:
            for _ in range(3):
                self.ser.write(b'M 0 0 0 0\n')
                time.sleep(0.05)
            self.ser.close()


def hold(robot, powers, seconds, state):
    """Keep sending one motor command for a while (the Uno's watchdog needs it repeated)."""
    DASH.state = state
    end = time.time() + seconds
    while time.time() < end and not STOP_EVENT.is_set():
        robot.drive(powers)
        time.sleep(CYCLE_S)
    if STOP_EVENT.is_set():
        robot.drive(STOP)


def servo_deg(rel_deg):
    """Servo angle for a direction relative to straight ahead (+ = left)."""
    return int(round(SONAR_CENTER_DEG + SONAR_LEFT_SIGN * rel_deg))


# ---- the scene: map, memory and decision (pure: --selftest exercises these) ---------------------------------------
def to_map(pose, r, bearing):
    x, y, th = pose
    return x + r * math.cos(th + bearing), y + r * math.sin(th + bearing)


def closest_on_segment(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return a[0] + t * dx, a[1] + t * dy


def seg_dist(a, b, p):
    q = closest_on_segment(a, b, p)
    return math.hypot(q[0] - p[0], q[1] - p[1])


def build_scene(frames, pose, need=None):
    """Several photos' blobs, all taken from `pose` -> (fires, victims) in the map frame.
    fires = [{'a', 'b' (its base's two ends), 'x', 'y'}]; victims = [{'x', 'y', 'r', 'bearing', 'n'}]. Something must
    be in most photos to count (a one-off speck is noise). Clipped blobs (running off the picture) are not mapped."""
    majority = len(frames) // 2 + 1
    need = majority if need is None else need                       # photos a victim must be in
    fire_obs, victim_obs = [], []
    for i, blobs in enumerate(frames):
        for b in blobs:
            if b.range is None or b.clipped:
                continue
            if b.kind == 'fire':
                fire_obs.append((i, to_map(pose, b.range, b.b_left), to_map(pose, b.range, b.b_right)))
            else:
                victim_obs.append((i, b, to_map(pose, b.range, b.bearing)))
    fires = []
    for g in _group(fire_obs, lambda o: ((o[1][0] + o[2][0]) / 2, (o[1][1] + o[2][1]) / 2), SAME_FIRE_M):
        if len(g) >= majority:
            a = (statistics.median(o[1][0] for o in g), statistics.median(o[1][1] for o in g))
            b = (statistics.median(o[2][0] for o in g), statistics.median(o[2][1] for o in g))
            fires.append({'a': a, 'b': b, 'x': (a[0] + b[0]) / 2, 'y': (a[1] + b[1]) / 2})
    victims = []
    for g in _group(victim_obs, lambda o: o[2], MERGE_M):
        if len(g) >= need:
            victims.append({'x': statistics.median(o[2][0] for o in g), 'y': statistics.median(o[2][1] for o in g),
                            'r': statistics.median(o[1].range for o in g),
                            'bearing': statistics.median(o[1].bearing for o in g), 'n': len(g),
                            'score': statistics.median(getattr(o[1], 'score', 0.0) for o in g)})
    return fires, victims


def _group(obs, where, gate):
    """Group observations (frame index first) by position: one per photo per group, within `gate` of the group."""
    groups = []
    for o in obs:
        p = where(o)
        g = next((g for g in groups if all(m[0] != o[0] for m in g)
                  and math.hypot(where(g[0])[0] - p[0], where(g[0])[1] - p[1]) < gate), None)
        if g:
            g.append(o)
        else:
            groups.append([o])
    return groups


def victim_name(i):
    """A, B, ... Z, AA, AB, ... - victims keep coming."""
    return victim_name(i // 26 - 1) + chr(ord('A') + i % 26) if i >= 26 else chr(ord('A') + i)


class World:
    """What the robot believes: victims (name, x, y, status) and fires, in the map frame. Victims look identical, so a
    sighting is matched to a remembered one by position. A remembered victim that should have been in view but was
    not, twice running, is forgotten (it was moved, or was never real)."""

    def __init__(self):
        self.victims = []                # [{'name', 'x', 'y', 'status': 'seen'|'saved'|'ignored', 'misses', 'tries'}]
        self.fires = []                  # [{'a', 'b', 'x', 'y'}]
        self.home_fires = []             # fires seen from the start pose (they re-anchor the position at home)
        self.named = 0

    def active(self):
        return [v for v in self.victims if v['status'] == 'seen']

    def update(self, fires, victims, pose, at_home=False):
        for f in fires:
            old = next((o for o in self.fires if math.hypot(o['x'] - f['x'], o['y'] - f['y']) < SAME_FIRE_M), None)
            if old:
                old.update(f)
            else:
                self.fires.append(f)
            if at_home and not any(math.hypot(o['x'] - f['x'], o['y'] - f['y']) < SAME_FIRE_M for o in self.home_fires):
                self.home_fires.append(dict(f))
        seen = set()
        victims = [s for s in victims if math.hypot(s['x'], s['y']) >= SAFE_RADIUS_M]   # in the safe area: saved
        # one-to-one, closest pairs first: victims can stand close together, so one remembered victim must never
        # swallow two sightings
        pairs = sorted((math.hypot(v['x'] - s['x'], v['y'] - s['y']), i, j)
                       for i, s in enumerate(victims) for j, v in enumerate(self.victims))
        match, used = {}, set()
        for d, i, j in pairs:
            if d < SAME_VICTIM_M and i not in match and j not in used:
                match[i] = j
                used.add(j)
        for i, s in enumerate(victims):
            if i in match:
                old = self.victims[match[i]]
                if old['status'] == 'seen':
                    old['x'], old['y'] = s['x'], s['y']
                    old['misses'] = 0
                seen.add(id(old))
            else:
                v = {'name': victim_name(self.named), 'x': s['x'], 'y': s['y'], 'status': 'seen', 'misses': 0,
                     'tries': 0}
                self.named += 1
                self.victims.append(v)
                seen.add(id(v))
        x, y, th = pose
        for v in self.active():                                     # should have been in view, but was not
            r = math.hypot(v['x'] - x, v['y'] - y)
            b = wrap(math.atan2(v['y'] - y, v['x'] - x) - th)
            if id(v) not in seen and 0.35 < r < 1.2 and abs(b) < math.radians(HFOV_DEG / 2 - 4):
                v['misses'] += 1
                if v['misses'] >= 2:
                    self.victims.remove(v)
                    DASH.think(f'Victim {v["name"]} is no longer where I saw it - forgetting it.')

    def danger(self, v):
        """Distance (m) from victim v to the nearest fire's base edge; None if no fire is known."""
        if not self.fires:
            return None
        return min(seg_dist(f['a'], f['b'], (v['x'], v['y'])) for f in self.fires)

    def ranked(self, pose):
        """The rescue order: MOST DANGEROUS FIRST = closest to a fire (no fire known: nearest to the robot). A tie
        within 5 cm goes to the nearer victim."""
        near = lambda v: math.hypot(v['x'] - pose[0], v['y'] - pose[1])
        if not self.fires:
            return sorted(self.active(), key=near)
        return sorted(self.active(), key=lambda v: (round(self.danger(v) / 0.05), near(v)))


def fire_hazards(fires):
    """The fires as hazards for the route planner: each base's two ends and its middle."""
    return [(p[0], p[1], FIRE_CLEAR_M, 'the fire') for f in fires for p in (f['a'], (f['x'], f['y']), f['b'])]


def plan(start, goal, hazards):
    """Waypoints from start to goal keeping each hazard's clearance [(x, y, clear)]: straight if the line is clear,
    else a detour point beside the hazard that blocks it, on the side the line already passes. A goal that is itself
    near a hazard (a victim beside the fire) shrinks that hazard's clearance to what the goal allows."""
    path, at = [], tuple(start)
    for _ in range(len(hazards)):
        blocking = []
        for hx, hy, clear in hazards:
            c = min(clear, math.hypot(goal[0] - hx, goal[1] - hy) - 0.05)
            q = closest_on_segment(at, goal, (hx, hy))
            if math.hypot(q[0] - hx, q[1] - hy) < c:
                blocking.append((math.hypot(at[0] - hx, at[1] - hy), (hx, hy), q, c))
        if not blocking:
            break
        _, f, q, c = min(blocking)
        dx, dy = q[0] - f[0], q[1] - f[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:                                                # hazard dead on the line: pass it on the left
            dx, dy = -(goal[1] - at[1]), goal[0] - at[0]
            n = math.hypot(dx, dy)
        at = (f[0] + dx / n * c * 1.3, f[1] + dy / n * c * 1.3)
        path.append(at)
    return path + [tuple(goal)]


def reach_distance():
    """How far ahead a victim is when its floor contact reaches CLOSE_FRAC of the picture."""
    return floor_range(H * CLOSE_FRAC) or 0.14


def touch_creep(rng):
    """The blind creep that brings the magnet onto a victim `rng` metres ahead of the lens."""
    return max(0.03, min(CLOSE_LOST_M, rng - MAGNET_REACH_M + TOUCH_PUSH_M))


def publish(world, ranked=()):
    objs = [{'name': 'FIRE', 'kind': 'fire', 'x': f['x'], 'y': f['y'], 'rank': None, 'reached': False,
             'ax': f['a'][0], 'ay': f['a'][1], 'bx': f['b'][0], 'by': f['b'][1]} for f in world.fires]
    order = [v['name'] for v in ranked]
    for v in world.victims:
        if v['status'] == 'ignored':
            continue
        objs.append({'name': v['name'], 'kind': 'victim', 'x': v['x'], 'y': v['y'],
                     'rank': order.index(v['name']) + 1 if v['name'] in order else None,
                     'reached': v['status'] == 'saved'})
    DASH.set(objs=objs, hazards=[list(h) for h in fire_hazards(world.fires)])


# ---- acting -------------------------------------------------------------------------------------------------------
class Nav:
    """The robot's pose in the map (the start pose's frame). Heading = the camera's integrated yaw minus an offset;
    position = dead reckoning (time x speed). `obstacle` = where the sonar last stopped a leg; `dead` = the motors will
    not turn the robot. dry: nothing moves, so turns and legs are pretended (the pose changes as if they happened)."""

    def __init__(self, robot, cam, dry=False):
        self.robot, self.cam, self.dry = robot, cam, dry
        self.x = self.y = 0.0
        self.yaw0 = cam.yaw
        self.obstacle = None
        self.dead = False
        self.mps = CRUISE_MPS
        self.spin_rate = {True: SPIN_RATE_DPS, False: SPIN_RATE_DPS}   # deg/s left / right, learned every pulse

    @property
    def th(self):
        return wrap(self.cam.yaw - self.yaw0)

    @property
    def pose(self):
        return self.x, self.y, self.th

    def set_heading(self, th):
        self.yaw0 = self.cam.yaw - th

    def moved(self):
        DASH.moved(self.x, self.y, self.th)

    def pause(self, s=SETTLE_S, state='stopped'):
        hold(self.robot, STOP, s, state)

    def look(self, frames=1):
        """Stop, settle, and return the blobs of `frames` fresh frames (a list of lists)."""
        self.pause()
        return [self.cam.snap(time.time()) for _ in range(frames)]

    def _advance(self, dist):
        self.x += dist * math.cos(self.th)
        self.y += dist * math.sin(self.th)

    def turn_to(self, target, state='turning'):
        """Turn until the camera-measured heading is within TURN_TOL_DEG of `target` (rad): pulse, STOP, let the camera
        measure what the robot really turned, learn the rate, pulse again for what is left. False if it will not turn
        (self.dead is then set)."""
        tol = math.radians(TURN_TOL_DEG)
        err = wrap(target - self.th)
        if abs(err) <= tol:
            return True
        if self.dry:
            DASH.state = f'{state} (dry run: pretending)'
            STOP_EVENT.wait(min(1.5, abs(math.degrees(err)) / SPIN_RATE_DPS))
            self.set_heading(target)
            self.moved()
            return True
        stalls = 0
        for _ in range(MAX_SPINS):
            err = wrap(target - self.th)
            if abs(err) <= tol or STOP_EVENT.is_set():
                break
            left = err > 0
            t = max(SPIN_MIN_S, min(SPIN_MAX_S, SPIN_DEAD_S + SPIN_AIM * abs(math.degrees(err)) / self.spin_rate[left]))
            th0 = self.th
            still = getattr(self.cam, 'snap_band', None)
            g0 = still(time.time()) if still else None
            self.cam.fast = True                                    # no detecting mid-turn
            hold(self.robot, SPINS[left], t, f'{state}: {math.degrees(err):+.0f} deg to go')
            self.cam.fast = False
            self.pause(0.4, state)                                  # stopped: let the camera catch up, then judge
            g1 = still(time.time()) if still else None
            if g0 is not None and g1 is not None:
                # two SHARP pictures, before and after: the turn between them, without the blur of the whip-round
                d, ok = turn_between(g0, g1)
                if ok:
                    self.set_heading(wrap(th0 + d))
            self.moved()
            turned = abs(math.degrees(wrap(self.th - th0)))
            if turned < STALL_DEG:
                stalls += 1
                if stalls >= MAX_STALLS:
                    DASH.say(f'The turns are not turning the robot ({MAX_STALLS} pulses, the camera saw no turn) - '
                             f'stopping. Motor switch off or the motor batteries flat? (Measured 2026-10-26 on weak '
                             f'batteries: the wheels lurch for half a second, then stall.)')
                    self.dead = True
                    return False
                self.spin_rate[left] = max(5.0, self.spin_rate[left] * 0.7)   # it skidded: next pulse longer
                continue
            stalls = 0
            self.spin_rate[left] = max(5.0, 0.5 * self.spin_rate[left] + 0.5 * turned / max(0.1, t - SPIN_DEAD_S))
        return abs(wrap(target - self.th)) <= tol * 1.5

    def leg(self, dist, heading, backward=False, label='', quiet=False, checks=True, obstacle_cm=OBSTACLE_CM):
        """Drive `dist` m FACING `heading`, forward or in reverse, holding that heading with the camera. Forward legs
        stop for an obstacle (sonar) or a fire close ahead (unless checks=False: the final touch). True if the whole
        distance was covered."""
        self.obstacle = None
        if not self.turn_to(heading, f'lining up for {label}') and self.dead:
            return False
        if not quiet:
            DASH.say(f'Driving {dist:.2f} m {"in reverse " if backward else ""}towards {label}')
        self.robot.aim(SONAR_CENTER_DEG)
        sign = -1 if backward else 1
        t0 = last = time.time()
        end = t0 + (0 if self.dry else DRIVE_DEAD_S) + dist / self.mps   # the motors start ~0.5 s late
        while time.time() < end and not STOP_EVENT.is_set():
            err = wrap(heading - self.th)
            steer = max(-60, min(60, K_HEAD * err))
            # + err (need to turn left): right side faster. In reverse the faster side swaps to turn the same way
            self.robot.drive(drive_powers(CRUISE - steer, CRUISE + steer) if not backward else
                             drive_powers(-(CRUISE + steer), -(CRUISE - steer)))
            DASH.state = f'driving {"back " if backward else ""}to {label}'
            if not backward and checks and not self.dry:
                fire_ahead = [b for b in self.cam.blobs if b.kind == 'fire' and b.range is not None
                              and b.range < FIRE_STOP_M and abs(b.bearing) < math.radians(20)]
                if fire_ahead:
                    self.pause(0.2)
                    self.moved()
                    DASH.say(f'FIRE {fire_ahead[0].range:.2f} m straight ahead - stopped the leg')
                    return False
                cm = self.robot.ping()
                if cm is not None and SONAR_MIN_CM <= cm < obstacle_cm:
                    r = cm / 100 + 0.05                             # the sonar sits a little behind the nose
                    now = time.time()
                    self._advance(self.mps * (now - last))
                    last = now
                    self.obstacle = (self.x + r * math.cos(self.th), self.y + r * math.sin(self.th))
                    self.pause(0.2)
                    self.moved()
                    DASH.say(f'OBSTACLE {cm} cm ahead - stopped')
                    return False
            else:
                time.sleep(CYCLE_S)
            now = time.time()
            self._advance(sign * self.mps * (now - last))
            last = now
            self.moved()
        self.pause(0.25)
        return True

    def go_to(self, p, label):
        d = math.hypot(p[0] - self.x, p[1] - self.y)
        if d < 0.03:
            return True
        return self.leg(d, math.atan2(p[1] - self.y, p[0] - self.x), label=label)

    def learn_speed(self, r0, r1, seconds):
        """A victim's range went r0 -> r1 during a straight hop of `seconds`: that is the real speed."""
        moved = r0 - r1
        if self.dry or seconds < 0.2 or moved < 0.03 or r0 > 1.2:
            return
        self.mps = max(0.05, min(0.6, 0.5 * self.mps + 0.5 * moved / seconds))


def sonar_sweep(robot, angles=SWEEP_DEG):
    """{relative angle (+ = left): cm or None}, servo centred again afterwards. The wheels do not move."""
    out = {rel: robot.look(servo_deg(rel)) for rel in angles}
    robot.aim(SONAR_CENTER_DEG)
    return out


def describe_sweep(seen):
    return ', '.join(f'{"left" if rel > 0 else "right" if rel < 0 else "ahead"} {abs(rel)}: '
                     + ('-' if cm is None else 'no echo' if cm >= 400 else f'{cm} cm') for rel, cm in seen.items())


def horizon_from_victims(frames):
    """The horizon row the victims in these photos imply (see VICTIM_HEIGHT_M), or None with too few to tell."""
    k = CAM_HEIGHT_M / VICTIM_HEIGHT_M
    est = [b.bottom - k * b.h for blobs in frames for b in blobs
           if b.kind == 'victim' and not b.clipped and b.h >= HORIZON_MIN_PX]
    return statistics.median(est) if len(est) >= max(2, len(frames) // 2) else None


def update_horizon(frames):
    global HORIZON_Y
    new = horizon_from_victims(frames)
    if new is None or not -H < new < H:
        return
    if abs(new - HORIZON_Y) > 15:
        DASH.think(f'The victims\' size in the picture puts the horizon at row {new:.0f} (I had {HORIZON_Y:.0f}): the '
                   f'camera has tilted, so my distances are re-scaled.')
    HORIZON_Y = 0.5 * HORIZON_Y + 0.5 * new if abs(new - HORIZON_Y) <= 15 else new


def look_at_scene(nav, frames=LOOK_FRAMES):
    photos = nav.look(frames)
    update_horizon(photos)
    return build_scene(photos, nav.pose)


def relocalise(nav, fire):
    """Back at (about) the start: face a fire seen from the start, measure its base over several photos. Its bearing
    fixes the heading and its range + bearing fix the position. A correction too big to be drift is ignored."""
    nav.turn_to(math.atan2(fire['y'] - nav.y, fire['x'] - nav.x), 'turning to the fire to re-localise')
    obs = []
    for blobs in nav.look(5):
        near = [b for b in blobs if b.kind == 'fire' and b.range is not None and not b.clipped]
        if near:
            b = min(near, key=lambda b: abs(b.bearing))
            obs.append((b.range, b.bearing))
    if len(obs) < 3:
        DASH.think('I cannot see the fire clearly from here, so I trust my dead reckoning.')
        return False
    r = statistics.median(o[0] for o in obs)
    b = statistics.median(o[1] for o in obs)
    x, y, th = nav.x, nav.y, nav.th
    for _ in range(2):                                              # heading from bearing, position from range
        th = wrap(math.atan2(fire['y'] - y, fire['x'] - x) - b)
        x, y = fire['x'] - r * math.cos(th + b), fire['y'] - r * math.sin(th + b)
    shift = math.hypot(x - nav.x, y - nav.y)
    if shift > MAX_FIX_M:
        DASH.think(f'The fire says I am {shift:.2f} m from where I think - too much to be drift, ignoring it.')
        return False
    DASH.think(f'Fire seen {r:.2f} m away: I am {shift * 100:.0f} cm and {abs(math.degrees(wrap(th - nav.th))):.0f} '
               f'deg off my estimate - corrected.')
    nav.x, nav.y = x, y
    nav.set_heading(th)
    nav.moved()
    return True


def home_on(nav, name, bearing, rng):
    """CENTRE, THEN GO, on still pictures: find the victim, and if it is more than CENTER_TOL_DEG off the picture's
    centre turn towards it and look again; only when it is centred drive a hop straight at it, then repeat. When its
    floor contact nears the bottom of the picture (or it drops out of view while close) it is right in front: creep
    TOUCH_CREEP_M blind so the magnet touches it. The first lock goes to the blob that best fits the map's
    (bearing, rng); after that the victim is tracked by bearing. True when touched; False if lost or blocked."""
    nav.robot.aim(SONAR_CENTER_DEG)
    nav.obstacle = None
    track, gate, misses, locked = bearing, math.radians(ACQUIRE_GATE_DEG), 0, False
    last_range = rng
    for _ in range(MAX_HOPS):
        if STOP_EVENT.is_set():
            return False
        blobs = [b for frame in nav.look(2) for b in frame]
        mine = [b for b in blobs if b.kind == 'victim' and abs(b.bearing - track) < gate
                and (b.range is not None or b.clipped)]
        if not mine:
            if locked and last_range <= CLOSE_LOST_M:
                creep = touch_creep(last_range)
                DASH.say(f'Victim {name} is too close for the camera now ({last_range * 100:.0f} cm when last seen) '
                         f'- creeping {creep * 100:.0f} cm so the magnet reaches it')
                nav.leg(creep, nav.th, label=f'victim {name}', quiet=True, checks=False)
                nav.pause(0.5, f'at victim {name}')
                return True
            misses += 1
            if misses >= 3:
                DASH.say(f'Lost sight of victim {name}')
                return False
            gate = math.radians(ACQUIRE_GATE_DEG)
            continue
        misses = 0
        b = (min(mine, key=lambda b: abs(b.bearing - track)) if locked else
             min(mine, key=lambda b: (math.degrees(b.bearing - bearing) / 3) ** 2
                 + (((b.range or reach_distance()) - rng) / 0.15) ** 2))
        locked = True
        DASH.state = f'homing on victim {name}: {b.label()}'
        if b.clipped or b.bottom >= H * CLOSE_FRAC:
            if abs(b.bearing) > math.radians(CENTER_TOL_DEG * 2):
                nav.turn_to(wrap(nav.th + b.bearing), f'centring victim {name}')
                track, gate = 0.0, math.radians(TRACK_GATE_DEG)
                continue
            creep = touch_creep(reach_distance() if b.clipped or b.range is None else b.range)
            DASH.say(f'Victim {name} is right in front - creeping {creep * 100:.0f} cm so the magnet picks it up')
            nav.leg(creep, nav.th, label=f'victim {name}', quiet=True, checks=False)
            nav.pause(0.5, f'touching victim {name}')
            return True
        last_range = b.range
        if abs(b.bearing) > math.radians(CENTER_TOL_DEG):           # not centred: turn, then look again
            nav.turn_to(wrap(nav.th + b.bearing), f'centring victim {name}')
            track, gate = 0.0, math.radians(TRACK_GATE_DEG)
            continue
        hop = max(0.05, min(HOP_M, 0.6 * (b.range - reach_distance())))
        limit = min(OBSTACLE_CM, max(SONAR_MIN_CM + 1, int((b.range - 0.10) * 100)))   # only things NEARER than it
        t0 = time.time()
        if not nav.leg(hop, nav.th, label=f'victim {name}', quiet=True, obstacle_cm=limit) and nav.obstacle:
            return False
        seconds = time.time() - t0
        after = [x for frame in nav.look(1) for x in frame if x.kind == 'victim' and x.range is not None
                 and not x.clipped and abs(x.bearing) < math.radians(TRACK_GATE_DEG)]
        if after:
            nav.learn_speed(b.range, min(after, key=lambda x: abs(x.bearing)).range, seconds - 0.25)
        track, gate = 0.0, math.radians(TRACK_GATE_DEG)              # it should still be dead ahead
    DASH.say(f'Victim {name}: too many hops - giving up')
    return False


def picked_up(nav, name):
    """Did the magnet take it? Back off a little and look: a victim still standing right in front was NOT picked up."""
    nav.leg(VERIFY_BACK_M, nav.th, backward=True, label=f'checking victim {name}', quiet=True)
    blobs = [b for frame in nav.look(3) for b in frame]
    left = [b for b in blobs if b.kind == 'victim' and not b.clipped and b.range is not None
            and b.range < VERIFY_NEAR_M and abs(b.bearing) < math.radians(TRACK_GATE_DEG)]
    if len(left) >= 2:
        DASH.think(f'Victim {name} is still standing on the floor {min(b.range for b in left) * 100:.0f} cm in front '
                   f'of me - the magnet did not catch it.')
        return False
    DASH.think(f'Victim {name} is no longer on the floor in front of me - it is on the magnet.')
    return True


def add_obstacle(hazards, p):
    hazards.append((p[0], p[1], OBSTACLE_CLEAR_M, 'an obstacle'))
    DASH.set(hazards=[list(h) for h in hazards])
    DASH.say(f'Obstacle marked on the map at ({p[0]:+.2f}, {p[1]:+.2f}) - REROUTING')


def drive_route(nav, goal, hazards, label, last_leg=True):
    """Drive the planned route towards goal (all legs, or all but the last). Sonar obstacles join the hazards and the
    route is re-planned. True when done."""
    for _ in range(MAX_REROUTES + 1):
        path = plan((nav.x, nav.y), goal, [h[:3] for h in hazards])
        if len(path) > 1:
            DASH.think(f'The straight line to {label} passes too close to {", ".join(sorted({h[3] for h in hazards}))}, '
                       f'so I go round: ' + ' -> '.join(f'({x:+.2f},{y:+.2f})' for x, y in path) + '.')
        DASH.set(path=[[nav.x, nav.y]] + [list(p) for p in path])
        ok = True
        for p in (path if last_leg else path[:-1]):
            if not nav.go_to(p, label):
                ok = False
                break
        if ok:
            return True
        if nav.obstacle is None or nav.dead:
            return False
        add_obstacle(hazards, nav.obstacle)
    DASH.say(f'Too many obstacles on the way to {label} - giving up this route')
    return False


def go_home(nav, world, carrying):
    """Carry the victim back to the safe area (the start point), re-anchor on a fire seen from there, fix what is
    left, then stop SAFE_STOP_S seconds."""
    DASH.say(f'Carrying victim {carrying} back to the safe area')
    hazards = fire_hazards(world.fires) + [(v['x'], v['y'], VICTIM_CLEAR_M, f'victim {v["name"]}')
                                           for v in world.active()]
    drive_route(nav, (0.0, 0.0), hazards, 'the safe area')
    for _ in range(2):
        if not world.home_fires or nav.dead or not relocalise(nav, world.home_fires[0]):
            break
        err = math.hypot(nav.x, nav.y)
        if err <= HOME_TOL_M:
            DASH.think(f'Back in the safe area (within {err * 100:.0f} cm of the start).')
            break
        DASH.think(f'I am {err * 100:.0f} cm from the start point - correcting.')
        drive_route(nav, (0.0, 0.0), hazards, 'the safe area')
    DASH.set(path=[])
    DASH.say(f'Victim {carrying} SAVED - in the safe area. Stopping {SAFE_STOP_S} s: take it off the magnet now.')
    for n in range(SAFE_STOP_S, 0, -1):
        hold(nav.robot, STOP, 1.0, f'SAFE AREA: victim {carrying} saved - stopped, {n} s')


def wander_step(nav, world, ws):
    """One step of wandering: MOSTLY TURNS. A turn of WANDER_TURN_DEG on the spot (a look follows, in the mission
    loop); every few turns, one short hop in the clearest direction the sonar finds, leaning back towards the safe
    area when far from it, and never at a known fire."""
    if ws['turns'] < ws['goal']:
        if random.random() < 0.15:
            ws['dir'] = -ws['dir']                                  # now and then look the other way round
        DASH.say(f'Wandering: turning {WANDER_TURN_DEG} deg {"left" if ws["dir"] > 0 else "right"} to look around')
        nav.turn_to(wrap(nav.th + ws['dir'] * math.radians(WANDER_TURN_DEG)), 'wandering: turning to look')
        ws['turns'] += 1
        return
    ws['turns'], ws['goal'] = 0, random.randint(*WANDER_TURNS)
    seen = sonar_sweep(nav.robot, (-40, 0, 40))
    DASH.say('Sonar look-around: ' + describe_sweep(seen))
    far = math.hypot(nav.x, nav.y) > WANDER_RADIUS_M
    home = math.atan2(-nav.y, -nav.x)
    options = []
    for rel, cm in seen.items():
        if cm is None or cm < CLEAR_CM:
            continue
        heading = wrap(nav.th + math.radians(rel))
        end = (nav.x + WANDER_HOP_M * math.cos(heading), nav.y + WANDER_HOP_M * math.sin(heading))
        if any(seg_dist(f['a'], f['b'], end) < FIRE_CLEAR_M + 0.1 for f in world.fires):
            continue
        score = -abs(wrap(heading - home)) if far else cm + random.uniform(0, 60)
        options.append((score, rel, heading, cm))
    if not options:
        DASH.think('No direction has room for a hop - I keep turning instead.')
        return
    _, rel, heading, cm = max(options)
    DASH.think(('I am far from the safe area, so I hop back towards it' if far else
                f'Most room {"ahead" if rel == 0 else ("to the left" if rel > 0 else "to the right")} '
                f'({"no echo" if cm >= 400 else f"{cm} cm"}) - a short hop that way') + ', then I look around again.')
    nav.leg(WANDER_HOP_M, heading, label='a new place to look from')


def mission(nav, world, drive=True):
    """Until Ctrl+C: look; if victims are known, rescue the most dangerous, carry it home, stop 5 s; else wander."""
    seen = sonar_sweep(nav.robot)
    DASH.say('Sonar look-around: ' + describe_sweep(seen))
    if not any(cm is not None and cm < 400 for cm in seen.values()):
        DASH.think('The sonar heard nothing at any angle - is it powered (it may run off the motor battery)? I rely on '
                   'the camera until it answers.')
    ws = {'turns': 0, 'goal': random.randint(*WANDER_TURNS), 'dir': random.choice((-1, 1))}
    saved, first = 0, True
    while not STOP_EVENT.is_set():
        fires, sightings = look_at_scene(nav)
        world.update(fires, sightings, nav.pose, at_home=math.hypot(nav.x, nav.y) < HOME_TOL_M)
        order = world.ranked(nav.pose)
        publish(world, order)
        if not order:
            if not drive:
                DASH.think('No victims in view.')
                return
            if first or ws['turns'] == 0:
                DASH.think(f'No victim to rescue in sight ({saved} saved so far) - wandering to find one.')
            first = False
            wander_step(nav, world, ws)
            if nav.dead:
                DASH.say('STOPPED: the motors cannot turn the robot (motor switch? batteries?)')
                return
            continue
        first = False
        target = order[0]
        name = target['name']
        explain(world, order, nav.pose)
        if not drive:
            DASH.set(state='plan only - nothing moves (run without --plan to do it)')
            return
        DASH.set(target=name)
        DASH.say(f'Going to rescue victim {name}')
        if rescue(nav, world, target):
            saved += 1
            target['status'] = 'saved'
            go_home(nav, world, name)
            DASH.think(f'{saved} victim{"s" * (saved != 1)} saved. Looking for the next one.')
        else:
            target['tries'] += 1
            if target['tries'] >= MAX_TRIES:
                target['status'] = 'ignored'
                DASH.think(f'Victim {name} could not be picked up in {MAX_TRIES} tries - I leave that spot alone.')
            else:
                DASH.think(f'Victim {name} was not picked up - I will try again.')
        DASH.set(target=None, path=[])
        publish(world, world.ranked(nav.pose))
        if nav.dead:
            DASH.say('STOPPED: the motors cannot turn the robot (motor switch? batteries?)')
            return


def explain(world, order, pose):
    near = lambda v: math.hypot(v['x'] - pose[0], v['y'] - pose[1])
    if world.fires:
        DASH.think(f'I know of {len(world.fires)} fire{"s" * (len(world.fires) != 1)} and {len(order)} victim'
                   f'{"s" * (len(order) != 1)} to rescue:')
        for v in order:
            DASH.think(f'  victim {v["name"]}: {world.danger(v):.2f} m from the fire, {near(v):.2f} m from me')
        t = order[0]
        msg = f'Victim {t["name"]} is closest to the fire, so it is in the most danger: I rescue it first'
        if len(order) > 1:
            gap = world.danger(order[1]) - world.danger(t)
            msg += (f' ({gap * 100:.0f} cm nearer the fire than the next one)' if gap >= 0.05 else
                    ' (about as close to the fire as the next one, and nearer to me)')
            msg += '. Next: ' + ', then '.join(v['name'] for v in order[1:])
        DASH.think(msg + '.')
    else:
        DASH.think(f'I see {len(order)} victim{"s" * (len(order) != 1)} but no fire, so I cannot judge danger: I take '
                   f'the nearest, {order[0]["name"]} ({near(order[0]):.2f} m away).')


def rescue(nav, world, target):
    """Drive to the target, home in until the magnet touches it, check it was picked up. True if carrying it."""
    name = target['name']
    goal = (target['x'], target['y'])
    hazards = fire_hazards(world.fires) + [(v['x'], v['y'], VICTIM_CLEAR_M, f'victim {v["name"]}')
                                           for v in world.active() if v is not target]
    if not drive_route(nav, goal, hazards, f'victim {name}', last_leg=False):
        return False
    nav.turn_to(math.atan2(goal[1] - nav.y, goal[0] - nav.x), f'facing victim {name}')
    if not home_on(nav, name, wrap(math.atan2(goal[1] - nav.y, goal[0] - nav.x) - nav.th),
                   math.hypot(goal[0] - nav.x, goal[1] - nav.y)):
        return False
    if not picked_up(nav, name):
        return False
    DASH.say(f'Victim {name} PICKED UP by the magnet')
    return True


# ---- calibration and tests ----------------------------------------------------------------------------------------
def calibrate_horizon(cam, dist):
    """One victim at a tape-measured `dist` metres straight ahead (lens to the cylinder): where it meets the floor fixes
    the horizon row (the camera's tilt and most of its height error). Writes calibration.json, which every run loads."""
    print(f'Put ONE victim {dist:.2f} m straight in front of the camera (lens to the cylinder), nothing else on the '
          f'floor near it. Measuring in 3 s...', flush=True)
    if STOP_EVENT.wait(3):
        return
    rows = []
    t_end = time.time() + 6
    while time.time() < t_end and not STOP_EVENT.is_set():
        found = [b for b in cam.snap(time.time()) if b.kind == 'victim' and not b.clipped]
        if found:
            rows.append(min(found, key=lambda b: abs(b.bearing)).bottom)
    if len(rows) < 10:
        print(f'Saw the victim in only {len(rows)} frames - is it in view and lit? Nothing saved.')
        return
    row = statistics.median(rows)
    horizon = row - CAM_HEIGHT_M * F_PX / dist
    print(f'victim meets the floor at row {row:.1f} (spread {max(rows) - min(rows)} px over {len(rows)} frames) -> '
          f'horizon row {horizon:.1f} (was {HORIZON_Y})')
    CAL_FILE.write_text(json.dumps({'horizon_y': round(horizon, 1), 'measured_at_m': dist,
                                    'date': time.strftime('%Y-%m-%d %H:%M')}, indent=1) + '\n')
    print(f'saved to {CAL_FILE.name}: every run now uses it')


def test_moves(robot):
    print('WHEELS OFF THE GROUND. Forward, backward, left, right: 1 s each, starting in 3 s.', flush=True)
    if STOP_EVENT.wait(3):
        return
    for name, powers in (('FORWARD', drive_powers(CRUISE, CRUISE)), ('BACKWARD', drive_powers(-CRUISE, -CRUISE)),
                         ('TURN LEFT', SPINS[True]), ('TURN RIGHT', SPINS[False])):
        print(f'{name}: M %d %d %d %d' % powers, flush=True)
        hold(robot, powers, 1.0, name)
        hold(robot, STOP, 1.0, 'stopped')


def synthetic_scene(victims, box=None, seed=0):
    """A picture like this room (grey-white floor below a dark cabinet), with silver cylinders standing on the floor at
    (centre column, floor-contact row) and a red box at (x0, x1, base row) - for the self-test."""
    rng = np.random.default_rng(seed)
    hsv = np.zeros((H, W, 3), np.uint8)
    hsv[:] = (20, 60, 8)                                            # the black recess under the cabinet
    floor_top = int(HORIZON_Y) + 80
    hsv[floor_top:] = (82, 45, 150)                                 # the floor
    if box:
        x0, x1, base = box
        hsv[base - 120:base, x0:x1] = (1, 230, 110)                 # the red box, standing on the floor
    for cx, contact in victims:
        r = floor_range(contact)
        note = max(8, int(round(VICTIM_HEIGHT_M * F_PX / r)))      # the sticky note's height in the picture
        motor = int(round(FLAG_BASE_FRAC * note))
        half = max(3, int(0.016 / 2 * F_PX / r))
        hsv[contact - motor:contact, cx - half:cx + half + 1] = (104, 60, 120)              # the grey motor
        for k in range(note):                                       # the V: two arms meeting over the motor
            spread = 1 + int(k * 0.6)                               # narrow at the bottom: the point of the V
            row = contact - motor - k - 1
            hsv[row, cx - spread:cx - spread + 4] = hsv[row, cx + spread - 3:cx + spread + 1] = (45, 200, 220)
    hsv = np.clip(hsv.astype(int) + rng.integers(-3, 4, hsv.shape), 0, 255).astype(np.uint8)
    hsv[..., 0] = np.clip(hsv[..., 0], 0, 179)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def selftest():
    # the detector: a red box and two cylinders on the floor; a tile seam is not a victim
    img = synthetic_scene([(200, 300), (520, 330)], box=(100, 390, 250))      # box ~1 m, victims 0.55-0.66 m
    cv2.line(img, (0, 820), (W, 760), (120, 125, 120), 1)          # a thin tile seam
    blobs = detect(img)
    fires = [b for b in blobs if b.kind == 'fire']
    vics = sorted((b for b in blobs if b.kind == 'victim'), key=lambda b: b.x)
    assert len(fires) == 1 and len(vics) == 2, [b.label() for b in blobs]
    assert abs(vics[0].bottom - 300) <= 8 and abs(vics[1].bottom - 330) <= 8, [b.bottom for b in vics]
    assert vics[1].bearing < 0 < vics[0].bearing                          # right of centre = negative bearing
    assert detect(synthetic_scene([])) == []                              # an empty floor has nothing on it
    # the horizon comes back out of the victims' size, and a tilted camera does not hide them
    assert abs(horizon_from_victims([blobs] * LOOK_FRAMES) - HORIZON_Y) < 15, horizon_from_victims([blobs] * 5)
    # blue (coloured) and wide dark things are not victims
    hsv = cv2.cvtColor(synthetic_scene([]), cv2.COLOR_BGR2HSV)
    hsv[600:640, 300:330] = (105, 209, 45)                                 # a blue SHOE: same hue, but dark and vivid
    hsv[650:700, 100:300] = (20, 60, 35)                                   # a wide dark shoe
    assert not [b for b in detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)) if b.kind == 'victim']
    # the decision: closest to the fire first, even when another victim is nearer the robot
    fires_m, victims_m = build_scene([blobs] * LOOK_FRAMES, (0.0, 0.0, 0.0))
    assert len(fires_m) == 1 and len(victims_m) == 2, (fires_m, victims_m)
    world = World()
    world.update(fires_m, victims_m, (0.0, 0.0, 0.0), at_home=True)
    order = world.ranked((0.0, 0.0, 0.0))
    assert [world.danger(v) for v in order] == sorted(world.danger(v) for v in order)
    w2 = World()
    w2.fires = [{'a': (1.0, 0.3), 'b': (1.0, 0.1), 'x': 1.0, 'y': 0.2}]
    w2.victims = [{'name': 'A', 'x': 0.5, 'y': -0.3, 'status': 'seen', 'misses': 0, 'tries': 0},
                  {'name': 'B', 'x': 1.1, 'y': 0.4, 'status': 'seen', 'misses': 0, 'tries': 0}]
    assert w2.ranked((0, 0, 0))[0]['name'] == 'B'                         # nearest-first would take A
    w2.fires = []
    assert w2.ranked((0, 0, 0))[0]['name'] == 'A'                         # no fire: the nearest
    # a sighting in the safe area is an already-saved victim; a remembered one keeps its name
    w3 = World()
    w3.update([], [{'x': 0.1, 'y': 0.05}, {'x': 0.8, 'y': 0.0}], (0, 0, 0))
    w3.update([], [{'x': 0.85, 'y': 0.05}], (0, 0, 0))
    assert [v['name'] for v in w3.victims] == ['A'], w3.victims
    w3.update([], [{'x': 0.62, 'y': 0.03}, {'x': 0.53, 'y': -0.11}, {'x': 0.85, 'y': 0.0}], (0, 0, 0))
    assert len(w3.victims) == 3, w3.victims                               # close together, still three victims
    # a one-off speck is noise
    speck = Blob('victim', 300, 600, 6, 640)
    _, v2 = build_scene([blobs + [speck]] + [blobs] * (LOOK_FRAMES - 1), (0.0, 0.0, 0.0))
    assert len(v2) == 2, v2
    # routes: a fire on the straight line forces a detour that keeps its distance
    path = plan((0, 0), (2, 0), [(1, 0.1, 0.4)])
    assert len(path) == 2 and path[-1] == (2, 0) and path[0][1] < 0
    for a, b in zip([(0, 0)] + path, path):
        assert seg_dist(a, b, (1, 0.1)) > 0.36, (a, b)
    # depth: lower in the picture = nearer; at/above the horizon = no range
    assert floor_range(HORIZON_Y + 40) > floor_range(HORIZON_Y + 80) and floor_range(HORIZON_Y) is None
    assert 0.08 < reach_distance() < 0.3 and 0.03 <= touch_creep(reach_distance()) < 0.15
    # turning LEFT slides the scene RIGHT: rotation() must call that +
    band = np.float32(cv2.GaussianBlur(np.random.default_rng(1).random((120, 360)), (0, 0), 3))
    assert math.degrees(rotation(band, np.roll(band, 20, axis=1))[0]) > 1
    assert math.degrees(rotation(band, np.roll(band, -20, axis=1))[0]) < -1
    tex = np.float32(cv2.resize(np.random.default_rng(2).integers(0, 255, (60, 105)).astype(np.uint8), (840, 480),
                                interpolation=cv2.INTER_NEAREST))         # a wide scene; the camera sees 360 px of it
    for px in (40, 200, -200):                                            # ~5, ~23 and ~-23 deg: a turn LEFT moves
        d, ok = turn_between(tex[:, 240:600], tex[:, 240 - px:600 - px])  # the scene RIGHT
        assert ok and abs(d - math.atan2(2 * px, F_PX)) < math.radians(1.5), (px, ok, math.degrees(d))
    # THE MOTOR TABLE: what the car-control page sends with motor_config.md, front/back and left/right swapped
    assert drive_powers(255, 255) == (-255, -255, -255, -255)              # forward = the page's S (backward)
    assert drive_powers(-255, -255) == (255, 255, 255, 255)                # backward = the page's W
    assert SPINS[True] == (0, 0, -255, -255)                               # left = the page's D
    assert SPINS[False] == (-255, -255, 0, 0)                              # right = the page's A
    # steering on a straight leg: more power to the right side turns left
    assert all(abs(a) <= MAX_POWER for a in drive_powers(CRUISE + 60, CRUISE - 60))
    # turning: pulse, measure, add more - converges even when skidding loses half of every pulse
    for efficiency in (1.0, 0.5):
        robot = FakeRobot(efficiency)
        nav = Nav(robot, robot.cam)
        for target in (math.radians(40), math.radians(-12), math.radians(80)):
            assert nav.turn_to(target), (efficiency, math.degrees(target), math.degrees(nav.th))
            assert abs(math.degrees(wrap(nav.th - target))) <= TURN_TOL_DEG * 1.5, (efficiency, math.degrees(nav.th))
    json.loads(DASH.json())
    print('selftest: OK')


class FakeRobot:
    """For the self-test: turns at 30 deg/s x `efficiency` (the rest lost to skidding) after a 50 ms break-away, and a
    camera that reports that heading."""

    def __init__(self, efficiency):
        self.efficiency, self.cmd, self.t = efficiency, STOP, time.time()
        robot = self

        class Cam:
            @property
            def yaw(self):
                robot._integrate()
                return robot._yaw
        self.cam, self._yaw, self._since = Cam(), 0.0, time.time()

    def _integrate(self):
        now = time.time()
        rate = {SPINS[True]: 1, SPINS[False]: -1}.get(tuple(self.cmd), 0) * math.radians(30.0)
        if now - self._since > 0.05:
            self._yaw += rate * self.efficiency * (now - max(self.t, self._since + 0.05))
        self.t = now

    def drive(self, powers):
        self._integrate()
        if tuple(powers) != tuple(self.cmd):
            self._since = time.time()
        self.cmd = tuple(powers)

    def aim(self, deg):
        pass

    def ping(self):
        return None


def stop_other_copies():
    """Only one program can hold the camera and the Uno: stop any earlier victimbot.py first."""
    me, others = os.getpid(), []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if int(pid) != me and Path(argv[0].decode(errors='replace')).name.startswith('python') \
                and any(a.endswith(b'victimbot.py') for a in argv[1:]):
            others.append(int(pid))
    if not others:
        return
    print(f'Stopping the copy of victimbot.py that is already running ({", ".join(map(str, others))})...', flush=True)
    for sig in (signal.SIGINT, signal.SIGKILL):
        for p in others:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        for _ in range(50):
            if not any(os.path.exists(f'/proc/{p}') for p in others):
                return
            time.sleep(0.1)


def main():
    global CAMERA
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag in ('--dry', '--view', '--plan', '--moves', '--sonar', '--selftest'):
        ap.add_argument(flag, action='store_true')
    ap.add_argument('--calibrate-horizon', type=float, metavar='METRES')
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP_EVENT.set())
    stop_other_copies()
    robot = None
    try:
        if not (args.moves or args.sonar):
            CAMERA = Camera()
            try:
                CAMERA.start()
            except RuntimeError as e:
                print(f'\nThe camera is busy or missing ({e}). Is another program using it?')
                return
            if args.calibrate_horizon:
                calibrate_horizon(CAMERA, args.calibrate_horizon)
                return
            if args.view:
                DASH.set(state='view only - nothing moves')
                print('Viewing only. Ctrl+C to stop.')
                STOP_EVENT.wait()
                return
        robot = Uno(dry=args.dry or args.plan)
        robot.open()
        robot.look(SONAR_CENTER_DEG)                                # centred before anything else happens
        if args.sonar:
            print('Sonar sweep (+ = left of straight ahead). Hold a hand ~20 cm away on one side to check left/right.')
            while not STOP_EVENT.is_set():
                print(describe_sweep(sonar_sweep(robot)), flush=True)
            return
        if args.moves:
            test_moves(robot)
            return
        if args.dry:
            DASH.say('DRY RUN: no motor command is sent; turns and drives are only pretended on the map.')
        elif not args.plan:
            for n in range(COUNTDOWN_S, 0, -1):
                DASH.set(state=f'starting in {n} s - Ctrl+C to cancel')
                print(f'Moving in {n} s (Ctrl+C to cancel)...', flush=True)
                if STOP_EVENT.wait(1):
                    return
        STOP_EVENT.wait(1.5)                                        # let the camera settle
        mission(Nav(robot, CAMERA, dry=args.dry), World(), drive=not args.plan)
        DASH.set(state=DASH.state if args.plan else 'finished - Ctrl+C to exit (the dashboard stays up until then)')
        STOP_EVENT.wait()
    except KeyboardInterrupt:
        pass
    finally:
        STOP_EVENT.set()
        if robot:
            robot.close()
        print('Stopped.')


if __name__ == '__main__':
    main()
