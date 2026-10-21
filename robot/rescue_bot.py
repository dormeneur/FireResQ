#!/usr/bin/env python3
"""FIRERESQ RESCUE BOT - run ON THE PI, from the same folder as wander_bot.py, wander_fw.hex and dashboard.html.

THE TASK. In front of the robot stand a FIRE (the tan cardboard box) and VICTIMS (wheels lying flat on the floor).
Everything is in view from the start spot (the SAFE AREA), so the robot never scans by turning. It runs until Ctrl+C:
    1. it watches the arena from the safe area (the sonar servo, centred first, sweeps once at the start),
    2. when victims are in view it picks the one CLOSEST TO THE FIRE,
    3. drives to it (on a route that keeps clear of the fire) until it reaches it - "RESCUED",
    4. reverses back to the safe area - "SAVED",
    5. waits for you to take that victim out of the arena, then watches again.
Add victims whenever you like: every look decides afresh, nothing about the layout is in the code.

    python3 -u rescue_bot.py --view          # camera + detection only, motors untouched
    python3 -u rescue_bot.py --plan          # look and decide, print/show the plan - never moves
    python3 -u rescue_bot.py                 # THE MISSION. Start it with the MOTOR SWITCH OFF: it says when
    python3 -u rescue_bot.py --calibrate-horizon 1.0   # ONE wheel 1.00 m ahead (lens to its near edge), no motors
    python3 -u rescue_bot.py --sonar-cal     # book 30 cm straight ahead: checks SONAR_CENTER_DEG (no motors)
    python3 -u rescue_bot.py --selftest      # the maths and the detector on a synthetic picture, no hardware

DASHBOARD: http://pi.local:8090 - live camera, the map (start, fire, victims, route, the robot's trail), what the
robot is doing, and why.

HOW IT SEES (measured in this room, 2026-10-21). In this light colour alone cannot separate things: the box, the
wheels' yellow, a chair and the curtain all read as dull tan/olive. What does stand out is the bright floor
(V 160-180). So:
    VICTIM = a wheel lying flat: a thin pale-yellow rim directly on top of a near-black tyre band, which ends on
             bright floor. (Fallback: a small dark puck sitting on the floor.) Its real width must be wheel-sized.
    FIRE   = the largest tan blob whose base sits on the floor. Only its BASE is used - a chair behind it merges
             into the blob higher up.
DEPTH. The camera is CAM_HEIGHT_M above a flat floor and looks level, so the horizon is a fixed image row. Where an
object meets the floor N pixels below the horizon, it is   range = CAM_HEIGHT_M * F_PX / N   away; the bearing is
the pixel column. "Closest to the fire" is the victim's distance to the box's BASE EDGE (a segment on the floor:
the box is ~40 cm wide, so a wheel by its corner is close to the fire even if far from its centre), measured inside
each photo (the robot's own errors cancel) and taken as the median over several photos.

MOVING (no encoders): turns are SKID spins (one side forward, the other backward), heading comes from the camera
(image shift between consecutive frames), straight legs hold that heading and are timed, the way home is driven IN REVERSE along the same route, and the fire, seen from
the start, re-anchors position and heading. Map frame = the start pose: x forward, y left, angles CCW (+ = left).
"""
import argparse
import json
import math
import signal
import statistics
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

import wander_bot as wb
from wander_bot import STOP_EVENT, hold

# ---- camera (OV5647 ribbon camera, mounted rotated 90 deg clockwise) ----------------------------------------------
CAM_SIZE = (640, 480)                    # what the sensor delivers
ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE  # undo the mounting: everything below works on the upright image
W, H = 480, 640                          # the upright image
HFOV_DEG = 41.4                          # horizontal FOV of the UPRIGHT image (the sensor's short side, 640x480 mode)
F_PX = (W / 2) / math.tan(math.radians(HFOV_DEG / 2))
CAM_HEIGHT_M = 0.10                      # lens above the floor
CAMERA_FPS = 12
WEB_PORT = 8090
HERE = Path(__file__).resolve().parent
DASHBOARD_HTML = HERE / 'dashboard.html'
VO_ROWS = (100, 330)                     # band used to track rotation: mostly above the horizon (distant things)
VO_MIN_CONF = 0.05                       # frame-to-frame match quality below this is ignored (motion blur)
HORIZON_Y = 318                          # image row of the horizon (camera tilt); --calibrate-horizon measures it
CAL_FILE = HERE / 'calibration.json'
try:
    HORIZON_Y = json.loads(CAL_FILE.read_text())['horizon_y']
except (OSError, ValueError, KeyError):
    pass

# ---- what to look for (HSV, OpenCV: H 0-179), sampled from this camera in this room on 2026-10-21 ------------------
FLOOR_V = 100                            # the floor is bright (V 160-180); anything darker is an object on it
DARK_V = 50                              # a black tyre (V 7-30)
PUCK_V = 30                              # a puck must be THIS dark: the box's shadow (V 35-50) must not join it
RIM_HSV = ((18, 35, 95), (38, 255, 255))  # the pale-yellow rim of a flat wheel: H 25-33, S 43-130, V 120-155
TAN_HSV = ((15, 70, 52), (26, 255, 130))   # the cardboard box: H 19-21, S 66-200, V 55-80; an orange cushion
#                                           (H 11-12) and the dimmer curtain and chair (V 45-51) must stay out.
#                                           Glare off its plastic wrap is NOT tan (S ~40)
# The fire can also be RED (the red bag used on 2026-10-22: H 0-3/179, S 184-235, V 62-112; the orange cushion, H ~12,
# V ~45, stays out).
RED_HSV = (((0, 150, 55), (4, 255, 255)), ((170, 150, 55), (179, 255, 255)))
FIRE_MASKS = {'tan': (TAN_HSV,), 'red': RED_HSV}
# Wheels can also STAND UP: a round, bright-yellow hub (H 26-30, S 138-209, V 123-140) over a black tyre.
HUB_HSV = ((20, 100, 90), (36, 255, 255))
HUB_WIDTH_M = (0.02, 0.07)               # a hub is ~4.5 cm across
WHEEL_RADIUS_M = 0.0325                  # a standing wheel's hub centre is this high above the floor
FIRE_MIN_AREA = 1500                     # px: the box is big; a tan speck is not the fire
FIRE_MIN_WIDTH_M = 0.20                  # ...and its base is ~40 cm wide in real life
MAX_RANGE_M = 4.0                        # ignore anything further away than this
WHEEL_WIDTH_M = (0.03, 0.12)             # a wheel found by its rim must be this wide in real life (part of a rim
#                                          may be unlit)
PUCK_WIDTH_M = (0.045, 0.12)             # a dark puck must be wheel-sized (6.5 cm): a shadow under a cloth is ~3 cm
BOX_COLOUR = {'fire': (0, 0, 255), 'victim': (0, 255, 255)}

# ---- motion -----------------------------------------------------------------------------------------------------
# The TT motors are 3-6 V parts and the L293D drops ~2 V, so the PWM is capped to keep the motors at <= ~7.5 V on
# average whatever the pack is: 4xAA (6 V) gets full power, 8xAA in series (12 V) is capped to 75%.
MOTOR_SUPPLY_V = 6.0                     # pack on EXT_PWR: 4xAA or 8xAA in PARALLEL = 6.0, 8xAA in series = 12.0
POWER_SCALE = min(1.0, 7.5 / (MOTOR_SUPPLY_V - 2.0))
wb.MAX_POWER = round(255 * POWER_SCALE)  # nothing is ever commanded above this
# M2 (back-left) and M4 (front-right) are stronger than M1 and M3 (the maintainer, 2026-10-22). The strong pair is
# scaled down to STRONG_RATIO so all four match; heading hold on straight legs mops up whatever this misses.
STRONG_MOTORS = (2, 4)                   # shield terminals M1..M4
STRONG_RATIO = 0.85                      # TODO(calibrate): see the 'steering bias' line each straight leg prints
wb.TRIM = tuple(STRONG_RATIO if m + 1 in STRONG_MOTORS else 1.0 for m in range(4))
# SKID (differential) turns, on the maintainer's instruction: one side forward, the other backward, so the robot
# spins about its centre (the chassis is too big to swing round a pivot). Both entries of a turn are the same spin.
SPIN_POWER = 255
SPINS = {True: (wb.drive_powers(-SPIN_POWER, SPIN_POWER),) * 2,      # LEFT: left side back, right side forward
         False: (wb.drive_powers(SPIN_POWER, -SPIN_POWER),) * 2}     # RIGHT: left side forward, right side back
SPIN_MAX_S = 1.0                         # longest single spin pulse
SPIN_MIN_S = 0.3                         # shortest: a skid spin from standstill needs this long to break loose
UNSTICK_S = 0.15                         # after 2 pulses that did not turn it: a nudge forward and back first
#                                          (rolling wheels slip round far more easily than tyres stuck to the tile)
SPIN_RATE_DPS = 26.0                     # measured 2026-10-22: a 1 s spin at full power turned 26 deg either way.
#                                          Only the starting guess - every pulse re-measures it
TURN_TOL_DEG = 3.0
SPIN_AIM = 0.8                           # aim each pulse at this fraction of what is left (then measure, add more)
STALL_DEG = 1.0                          # an arc that turned less than this did not move the robot
MAX_STALLS = 8
MAX_SPINS = 24                           # give up a turn after this many spins (it is oscillating or stuck)
CRUISE = round(210 * POWER_SCALE)        # forward power for legs and homing
CRUISE_MPS = 0.25                        # TODO(calibrate): forward speed at CRUISE. Reverse return cancels its error
K_HEAD = 160                             # steering power per radian of heading error on straight legs
SETTLE_S = 0.35                          # stop this long before trusting a frame
CYCLE_S = 0.06                           # control loop period (the Uno's watchdog is 400 ms)

# ---- sonar (servo-mounted HC-SR04) ------------------------------------------------------------------------------
SONAR_CENTER_DEG = 128                   # servo angle that points STRAIGHT AHEAD (found by hand; the firmware's power-on
#                                          angle, SERVO_CENTER_DEG in wander_fw.ino, is the same number)
SWEEP_DEG = (-40, -20, 0, 20, 40)        # the look-around at the start: right to left, relative to centre
OBSTACLE_CM = 22                         # something this close ahead (and closer than the target) = stop

# ---- mission ----------------------------------------------------------------------------------------------------
LOOK_FRAMES = 8                          # photos per look: the fire must be in most, a victim in a third
MERGE_M = 0.20                           # sightings of a victim closer than this (across photos) are the same victim
FIRE_CLEAR_M = 0.20                      # routes keep this far from the box's base edge...
VICTIM_CLEAR_M = 0.18                    # ...and from the victims that are not the current target
OBSTACLE_CLEAR_M = 0.25                  # ...and from an obstacle the sonar found
MAX_REROUTES = 3
FIRE_STOP_M = 0.15                       # the fire straight ahead, this close and nearer than the target: stop
HOP_M = 0.20                             # homing: drive at most this far between still pictures
CENTER_TOL_DEG = 4                       # ...and only once the victim is within this of the picture's centre
MAX_HOPS = 40
CLOSE_LOST_M = 0.45                      # a victim last seen this close and then out of view is right in front
TRACK_GATE_DEG = 12                      # victims look alike: a tracked blob must stay this close to where it was
ACQUIRE_GATE_DEG = 25                    # ...and this close to the map's prediction when homing starts
CLOSE_FRAC = 0.95                        # the tyre's floor contact this low in the image = right in front
TOUCH_CREEP_M = 0.17                     # then creep this far, blind, to touch: at CLOSE_FRAC the wheel's near edge is
#                                          ~0.22 m from the lens. TODO(calibrate): raise if it stops short, lower if it
#                                          pushes the wheel
HOME_TOL_M = 0.12                        # back at the start when re-localised within this
MAX_FIX_M = 0.6                          # a fire-based correction bigger than this is a mis-sighting: ignored
GONE_LOOKS = 2                           # the saved victim must be missing from this many looks in a row...
REMOVE_TIMEOUT_S = 120                   # ...or, after this long, the robot carries on and ignores that spot
SAME_VICTIM_M = 0.25                     # a victim within this of an ignored spot IS the ignored one
MAX_TRIES = 2                            # attempts on one victim before ignoring its spot
WATCH_S = 2.0                            # nothing to rescue: look again from the safe area this often
MOTOR_ON_COUNTDOWN_S = 15                # after the first decision: time to switch the motor battery on


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- the dashboard's data ---------------------------------------------------------------------------------------
class Dash:
    """Everything the web dashboard shows. The mission writes it, the web server reads it (as JSON, under the lock).
    say() = what the robot is DOING (bottom left), think() = WHY (bottom right). Both also print to the terminal."""

    def __init__(self):
        self.lock = threading.Lock()
        self.log, self.thoughts = [], []
        self.objs = []                   # [{name, kind, x, y, rank, reached}] (+ ax, ay, bx, by: the fire's base)
        self.hazards = []                # [[x, y, clearance, name]]
        self.path = []                   # the route being driven, [[x, y], ...]
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

    def json(self):
        with self.lock:
            return json.dumps({k: getattr(self, k) for k in
                               ('log', 'thoughts', 'objs', 'hazards', 'path', 'trail', 'pose', 'target', 'state')})


DASH = Dash()


class Status:
    """wander_bot's hold() writes status.state; route it to the dashboard."""

    @property
    def state(self):
        return DASH.state

    @state.setter
    def state(self, s):
        DASH.state = s


STATUS = Status()


# ---- perception -------------------------------------------------------------------------------------------------
def floor_range(row, horizon=None):
    """Floor range (m) of the point where an object meets the floor at image row `row`; None at/above the horizon or
    beyond MAX_RANGE_M (the arena is ~3 m deep; anything further is the room, not the arena)."""
    drop = row - (HORIZON_Y if horizon is None else horizon)
    r = CAM_HEIGHT_M * F_PX / drop if drop >= 4 else None
    return r if r is not None and r <= MAX_RANGE_M else None


def bearing_of(col):
    return math.atan2(W / 2 - col, F_PX)                            # rad, + = left of the camera axis


class Blob:
    """A fire or victim in the upright image. (x, w) span its BASE; `bottom` is the row where it meets the floor."""

    def __init__(self, kind, x, y, w, bottom):
        self.kind, self.x, self.y, self.w, self.bottom = kind, x, y, w, bottom
        self.h = bottom - y
        self.bearing = bearing_of(x + w / 2)
        self.range = floor_range(bottom)
        self.b_left, self.b_right = bearing_of(x), bearing_of(x + w)  # the base's two ends (the fire is wide)

    def label(self):
        r = '?' if self.range is None else f'{self.range:.2f}m'
        return f'{self.kind} {r} {math.degrees(self.bearing):+.0f}'


def floor_below(hsv, x, w, bottom):
    """Is the strip just under columns x..x+w at `bottom` unsaturated floor? (= the object stands there). Not
    'bright': the glossy floor reflects a dark object right under it (V 60-100 there, 160+ further off)."""
    y0, y1 = min(H - 1, bottom + 3), min(H, bottom + 12)
    strip = hsv[y0:y1, x + w // 4: max(x + w - w // 4, x + w // 4 + 1)]
    return strip.size > 0 and np.median(strip[..., 2]) > 70 and np.median(strip[..., 1]) < 90


def wheels_from_rims(hsv):
    """Flat wheels: a thin pale-yellow rim directly on top of a near-black tyre band that ends on bright floor."""
    out = []
    rim = cv2.inRange(hsv, *RIM_HSV)
    for c in cv2.findContours(rim, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        if w < 8 or h > 10 or w < 2.5 * h:
            continue
        cols = slice(x + w // 6, x + w - w // 6)
        yy, dark = y + h, 0
        while yy < H and yy - (y + h) <= 60 and np.median(hsv[yy, cols, 2]) < FLOOR_V:   # walk down the tyre
            dark += np.median(hsv[yy, cols, 2]) < DARK_V
            yy += 1
        r = floor_range(yy) if yy < H and yy - (y + h) <= 60 else None
        if 3 <= dark <= 50 and r:
            mid = (y + h + yy) // 2                                 # a row through the middle of the tyre
            xl, xr = x + w // 2, x + w // 2
            while xl > 0 and hsv[mid, xl - 1, 2] < DARK_V:
                xl -= 1
            while xr < W - 1 and hsv[mid, xr + 1, 2] < DARK_V:
                xr += 1
            if xr - xl > w and (xr - xl) * r / F_PX <= WHEEL_WIDTH_M[1]:
                x, w = xl, xr - xl + 1
            if WHEEL_WIDTH_M[0] <= w * r / F_PX <= WHEEL_WIDTH_M[1]:
                out.append(Blob('victim', x, y, w, yy))
    return out


def wheels_from_hubs(hsv):
    """Wheels standing up: a round bright-yellow hub over a black tyre. The range comes from the hub's CENTRE, which is
    always one wheel radius above the floor (walking down to the floor contact gets fooled by the floor's reflection
    right under the wheel); `bottom` is then set to the row where that range meets the floor."""
    out = []
    m = cv2.morphologyEx(cv2.inRange(hsv, *HUB_HSV), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        # roundish: a far hub whose lower half is shaded comes out up to 3x wider than tall (a flat wheel's rim is
        # thinner still, and is caught by wheels_from_rims instead)
        if cv2.contourArea(c) < 25 or not 0.5 <= w / h <= 3.0:
            continue
        cols = slice(x + w // 4, max(x + w - w // 4, x + w // 4 + 1))
        on_tyre = any(np.median(hsv[yy, cols, 2]) < DARK_V for yy in range(y + h, min(H, y + h + max(4, h // 2))))
        drop = y + h / 2 - HORIZON_Y
        if not on_tyre or drop < 4:                                 # a hub sits on a black tyre (a row just under)
            continue
        r = (CAM_HEIGHT_M - WHEEL_RADIUS_M) * F_PX / drop
        if r <= MAX_RANGE_M and HUB_WIDTH_M[0] <= w * r / F_PX <= HUB_WIDTH_M[1]:
            out.append(Blob('victim', x, y, w, min(H - 1, int(round(HORIZON_Y + CAM_HEIGHT_M * F_PX / r)))))
    return out


def wheels_from_pucks(hsv):
    """Fallback for a wheel whose rim is not lit: a small dark wide puck whose base meets the floor."""
    out = []
    m = cv2.morphologyEx(cv2.inRange(hsv, (0, 0, 0), (180, 255, PUCK_V)), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        r = floor_range(y + h)
        if cv2.contourArea(c) < 40 or r is None or not floor_below(hsv, x, w, y + h):
            continue
        if not (PUCK_WIDTH_M[0] <= w * r / F_PX <= PUCK_WIDTH_M[1] and 0.01 <= h * r / F_PX <= 0.06 and 1.2 <= w / h <= 6):
            continue
        # a wheel shows a hint of its yellowish hub just above the tyre; a shadow under a cushion has red/orange
        # fabric there (measured: wheels 19-69% yellowish, cushion shadows 0-3% yellowish and 74-89% red/orange)
        band = hsv[max(0, y - 6):y, x:x + w].reshape(-1, 3)
        yellowish = ((band[:, 0] >= 18) & (band[:, 0] <= 40) & (band[:, 1] >= 30) & (band[:, 2] >= 40)).mean()
        warm = (((band[:, 0] <= 12) | (band[:, 0] >= 165)) & (band[:, 1] >= 60)).mean()
        if yellowish >= 0.12 and warm < 0.5:
            out.append(Blob('victim', x, y, w, y + h))
    return out


def find_fire(hsv):
    """The largest tan or red blob standing on the floor. Glare off the box's plastic wrap is not tan, so the blob can end
    well above the box's real base: from the blob's bottom, walk down through whatever is not floor (glare, the box's
    shadow) to the first bright floor row - that is the base. Its width (the blob's widest row in its lower half) must
    be box-sized; its two ends are the base's ends."""
    best = None
    for ranges in FIRE_MASKS.values():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, lo, hi)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        best = _best_fire(hsv, cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)), best)
    return [best[1]] if best else []


def _best_fire(hsv, m, best):
    """The biggest box-sized blob of mask m standing on the floor, if bigger than `best` (area, Blob)."""
    for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if area < FIRE_MIN_AREA:
            continue
        blob = np.zeros(m.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        lower = blob[y + h // 2:y + h, x:x + w] > 0
        row = int(np.argmax(lower.sum(axis=1)))                     # the widest row in the lower half
        cols = np.flatnonzero(lower[row])
        xl, xr = x + int(cols.min()), x + int(cols.max())
        bottom, mid = y + h, slice(xl + (xr - xl) // 4, max(xr - (xr - xl) // 4, xl + (xr - xl) // 4 + 1))
        while bottom < H - 1 and bottom - (y + h) < 60 and np.median(hsv[bottom, mid, 2]) < FLOOR_V:
            bottom += 1                                             # down through glare and shadow to the floor
        r = floor_range(bottom)
        if r is None or (xr - xl) * r / F_PX < FIRE_MIN_WIDTH_M:     # in the air, or a tan scrap
            continue
        if best is None or area > best[0]:
            best = (area, Blob('fire', xl, y, xr - xl + 1, bottom))
    return best


def detect(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    wheels = wheels_from_hubs(hsv) + wheels_from_rims(hsv)
    for p in wheels_from_pucks(hsv):                                # fallback pucks not already found otherwise
        if all(p.x + p.w < q.x or q.x + q.w < p.x for q in wheels):
            wheels.append(p)
    return find_fire(hsv) + wheels


def rotation(prev, cur):
    """(rad, confidence): how far the camera turned between two grey bands (+ = left: the scene slides right when
    the camera turns left)."""
    (dx, _), conf = cv2.phaseCorrelate(prev, cur)
    return math.atan2(dx, F_PX), conf


class Camera:
    """Grabs, rotates and detects continuously; integrates the heading change frame to frame (`yaw`); serves the
    dashboard: / (page), /stream (MJPEG), /state (JSON). snap() hands out the first frame captured after a given time,
    so nothing acts on a picture taken before the robot stopped."""

    def __init__(self):
        self.cond = threading.Condition()
        self.t, self.blobs, self.jpeg = 0.0, [], None
        self.yaw = 0.0                   # rad, + = left, summed over consecutive frames since start
        self.fps = 0.0
        self._gray = None

    def start(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        # queue=False: every capture is a frame exposed after the request (a queued one is up to a second stale)
        cam.configure(cam.create_video_configuration(main={'size': CAM_SIZE, 'format': 'RGB888'},  # BGR in memory
                                                     queue=False, buffer_count=2))
        cam.start()
        threading.Thread(target=self._grab, args=(cam,), daemon=True).start()
        server = ThreadingHTTPServer(('0.0.0.0', WEB_PORT), self._handler())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f'DASHBOARD: open http://pi.local:{WEB_PORT} in a browser on your PC', flush=True)

    def _grab(self, cam):
        n, t_fps = 0, time.time()
        while not STOP_EVENT.is_set():
            t = time.time()                                         # stamp BEFORE the capture: conservative
            try:
                frame = cv2.rotate(cam.capture_array(wait=3.0), ROTATE)
            except Exception as e:                                  # a frontend timeout (power dip): carry on
                print(f'camera: {e}', flush=True)
                continue
            gray = np.float32(cv2.cvtColor(frame[VO_ROWS[0]:VO_ROWS[1]], cv2.COLOR_BGR2GRAY))
            if self._gray is not None:
                d, conf = rotation(self._gray, gray)
                if conf >= VO_MIN_CONF:
                    self.yaw += d                                   # a float store: atomic for the readers
            self._gray = gray
            blobs = detect(frame)
            for b in blobs:
                cv2.rectangle(frame, (b.x, b.y), (b.x + b.w, b.bottom), BOX_COLOUR[b.kind], 2)
                cv2.putText(frame, b.label(), (b.x, max(12, b.y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            BOX_COLOUR[b.kind], 1, cv2.LINE_AA)
            cv2.line(frame, (0, int(HORIZON_Y)), (W, int(HORIZON_Y)), (90, 90, 90), 1)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            with self.cond:
                self.t, self.blobs = t, blobs
                if ok:
                    self.jpeg = jpg.tobytes()
                self.cond.notify_all()
            n += 1
            if time.time() - t_fps > 5:
                self.fps, n, t_fps = n / (time.time() - t_fps), 0, time.time()
            time.sleep(max(0.0, 1.0 / CAMERA_FPS - (time.time() - t)))
        cam.stop()

    def snap(self, after, timeout=3.0):
        """Blobs of the first frame captured at or after `after`; [] if none arrives in time."""
        with self.cond:
            self.cond.wait_for(lambda: self.t >= after, timeout)
            return list(self.blobs) if self.t >= after else []

    def _handler(self):
        camera = self

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
        return Handler


# ---- the scene: map and decision (pure: --selftest exercises these) ---------------------------------------------
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


def build_scene(frames, pose):
    """Several photos' blobs, all taken from `pose` -> (fire, victims) in the map frame.
    fire = {'a', 'b' (its base's two ends), 'x', 'y' (the base's middle), 'r', 'bearing'} or None.
    victims = [{'x', 'y', 'r', 'bearing', 'd' (to the fire's base edge, same-photo median), 'n'}], each seen in most
    photos (a one-off speck is noise). Victims are told apart by bearing: two blobs in one photo are two victims."""
    majority = len(frames) // 2 + 1                                 # the fire: most photos
    enough = max(2, len(frames) // 3)                               # a victim: a third (a far hub flickers)
    fire_obs, victim_obs = [], []                                   # victim_obs: (frame, Blob, map point)
    for i, blobs in enumerate(frames):
        for b in blobs:
            if b.kind == 'fire' and b.range is not None:
                fire_obs.append((i, b))
            elif b.kind == 'victim' and b.range is not None:
                victim_obs.append((i, b, to_map(pose, b.range, b.bearing)))
    fire = None
    if len(fire_obs) >= majority:
        ends = [(to_map(pose, b.range, b.b_left), to_map(pose, b.range, b.b_right)) for _, b in fire_obs]
        a = (statistics.median(e[0][0] for e in ends), statistics.median(e[0][1] for e in ends))
        bb = (statistics.median(e[1][0] for e in ends), statistics.median(e[1][1] for e in ends))
        fire = {'a': a, 'b': bb, 'x': (a[0] + bb[0]) / 2, 'y': (a[1] + bb[1]) / 2,
                'r': statistics.median(b.range for _, b in fire_obs),
                'bearing': statistics.median(b.bearing for _, b in fire_obs)}
    fire_by_frame = {i: b for i, b in fire_obs}
    groups = []                                                     # [[(frame, blob, point), ...]]
    for o in sorted(victim_obs, key=lambda o: o[1].bearing):
        g = next((g for g in groups if all(m[0] != o[0] for m in g)
                  and abs(g[-1][1].bearing - o[1].bearing) < math.radians(3)
                  and math.hypot(g[-1][2][0] - o[2][0], g[-1][2][1] - o[2][1]) < MERGE_M), None)
        if g:
            g.append(o)
        else:
            groups.append([o])
    victims = []
    for g in groups:
        if len(g) < enough:
            continue
        v = {'x': statistics.median(m[2][0] for m in g), 'y': statistics.median(m[2][1] for m in g),
             'r': statistics.median(m[1].range for m in g), 'bearing': statistics.median(m[1].bearing for m in g),
             'n': len(g), 'd': None}
        same = []
        for i, b, p in g:                                           # distance to the fire's base IN THE SAME PHOTO
            f = fire_by_frame.get(i)
            if f is not None:
                same.append(seg_dist(to_map(pose, f.range, f.b_left), to_map(pose, f.range, f.b_right), p))
        if same:
            v['d'] = statistics.median(same)
        elif fire:
            v['d'] = seg_dist(fire['a'], fire['b'], (v['x'], v['y']))
        victims.append(v)
    return fire, victims


def choose(victims, fire, pose):
    """The victim closest to the fire (no fire seen: the one closest to the robot)."""
    if not victims:
        return None
    if fire is None:
        return min(victims, key=lambda v: math.hypot(v['x'] - pose[0], v['y'] - pose[1]))
    return min(victims, key=lambda v: v['d'])


def fire_hazards(fire):
    """The box as hazards for the route planner: its base's two ends and its middle."""
    if fire is None:
        return []
    return [(p[0], p[1], FIRE_CLEAR_M, 'the fire') for p in (fire['a'], (fire['x'], fire['y']), fire['b'])]


def plan(start, goal, hazards):
    """Waypoints from start to goal keeping each hazard's clearance [(x, y, clear)]: straight if the line is clear,
    else a detour point beside the hazard that blocks it, on the side the line already passes. A goal that is itself
    near a hazard (the victim beside the fire) shrinks that hazard's clearance to what the goal allows.
    ponytail: one detour per hazard, greedy; a grid A* if the arena gets crowded."""
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
    """How far ahead a wheel's near edge is when its floor contact reaches CLOSE_FRAC of the image."""
    return floor_range(H * CLOSE_FRAC)


# ---- acting -----------------------------------------------------------------------------------------------------
class Uno(wb.Robot):
    def close(self):
        """Stop and let go of the port, but leave the wander firmware on the Uno: this script uses it too, so there is
        no point re-flashing the old base firmware on every exit (wander_bot.py still does)."""
        if self.ser and self.ser.is_open:
            for _ in range(3):
                self.ser.write(b'M 0 0 0 0\n')
                time.sleep(0.05)
            self.ser.close()


class Nav:
    """The robot's pose in the map (the start pose's frame). Heading = the camera's integrated yaw minus an offset that
    the fire re-anchors; position = dead reckoning (timed legs), re-anchored on the fire at the start.
    `obstacle` = where the sonar last stopped a leg; `dead` = the motors will not turn the robot."""

    def __init__(self, robot, cam):
        self.robot, self.cam = robot, cam
        self.x = self.y = 0.0
        self.yaw0 = cam.yaw
        self.obstacle = None
        self.dead = False
        self.spin_rate = {True: SPIN_RATE_DPS, False: SPIN_RATE_DPS}   # deg/s left / right, learned from every
        #                                                               spin (measured 23.6 left vs 7.8 right once)

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
        hold(self.robot, STATUS, wb.STOP, s, state)

    def look(self, frames=1):
        """Stop, settle, and return the blobs of `frames` fresh frames (a list of lists)."""
        self.pause()
        return [self.cam.snap(time.time()) for _ in range(frames)]

    def _advance(self, dist):
        self.x += dist * math.cos(self.th)
        self.y += dist * math.sin(self.th)

    def turn_to(self, target, state='turning'):
        """SKID-spin until the camera-measured heading is within TURN_TOL_DEG of `target` (rad). Pulse, STOP, let the
        camera measure what the robot actually turned (a skid on the tile turns it less than commanded), learn the
        rate, pulse again for what is left. Each pulse aims a little short so the next one finishes the turn.
        False if it will not turn (self.dead is then set)."""
        tol = math.radians(TURN_TOL_DEG)
        err = wrap(target - self.th)
        if abs(err) <= tol:
            return True
        DASH.say(f'Turning {math.degrees(err):+.0f} deg (skid spin)')
        stalls = 0
        for _ in range(MAX_SPINS):
            err = wrap(target - self.th)
            if abs(err) <= tol or STOP_EVENT.is_set():
                break
            left = err > 0
            t = max(SPIN_MIN_S, min(SPIN_MAX_S, SPIN_AIM * abs(math.degrees(err)) / self.spin_rate[left]))
            if stalls >= 2:                                         # stuck to the floor: roll a little first
                hold(self.robot, STATUS, wb.drive_powers(CRUISE, CRUISE), UNSTICK_S, f'{state}: unsticking')
                hold(self.robot, STATUS, wb.drive_powers(-CRUISE, -CRUISE), UNSTICK_S, f'{state}: unsticking')
            th0 = self.th
            hold(self.robot, STATUS, SPINS[left][0], t, f'{state}: {math.degrees(err):+.0f} deg to go')
            self.pause(0.3, state)                                  # stopped: let the camera catch up, then judge
            self.moved()
            turned = abs(math.degrees(wrap(self.th - th0)))
            DASH.say(f'  spin pulse {t:.2f} s {"left" if left else "right"}: turned {turned:.1f} deg')
            if turned < STALL_DEG:
                stalls += 1
                if stalls >= MAX_STALLS:
                    DASH.say('The spins are not turning the robot - stopping. If the wheels did not even move: motor '
                             'switch / battery / a loose motor wire. If they spun in place: the tyres slip.')
                    self.dead = True
                    return False
                self.spin_rate[left] = max(5.0, self.spin_rate[left] * 0.7)   # it skidded: next pulse longer
                continue
            stalls = 0
            self.spin_rate[left] = 0.5 * self.spin_rate[left] + 0.5 * turned / t
        return abs(wrap(target - self.th)) <= tol * 1.5

    def leg(self, dist, heading, backward=False, label='', quiet=False, checks=True, obstacle_cm=OBSTACLE_CM):
        """Drive `dist` m with the robot FACING `heading`, forward or in reverse, holding that heading with the camera.
        Forward legs stop for an obstacle or the fire close ahead (unless checks=False: the final touch, next to the
        box). obstacle_cm: the sonar distance that counts as an obstacle (homing passes "nearer than the victim").
        True if the whole distance was covered."""
        self.obstacle = None
        if not self.turn_to(heading, f'lining up for {label}') and self.dead:     # a few degrees off is fine: the
            return False                                                          # leg holds the heading anyway
        if not quiet:
            DASH.say(f'Driving {dist:.2f} m {"in reverse " if backward else ""}towards {label}')
        self.robot.aim(SONAR_CENTER_DEG)
        sign = -1 if backward else 1
        t0 = last = time.time()
        end = t0 + dist / CRUISE_MPS
        steer_sum, n = 0.0, 0
        while time.time() < end and not STOP_EVENT.is_set():
            err = wrap(heading - self.th)
            steer = max(-80, min(80, K_HEAD * err))
            # + err (need to turn left): right side faster. In reverse the faster side swaps to turn the same way
            self.robot.drive(wb.drive_powers(CRUISE - steer, CRUISE + steer) if not backward else
                             wb.drive_powers(-(CRUISE + steer), -(CRUISE - steer)))
            steer_sum, n = steer_sum + steer, n + 1
            STATUS.state = f'driving {"back " if backward else ""}to {label}'
            if not backward and checks:
                fire_ahead = [b for b in self.cam.blobs if b.kind == 'fire' and b.range is not None
                              and b.range < FIRE_STOP_M and abs(b.bearing) < math.radians(20)]
                if fire_ahead:
                    self.pause(0.2)
                    self.moved()
                    DASH.say(f'FIRE {fire_ahead[0].range:.2f} m straight ahead - stopped the leg')
                    return False
                cm = self.robot.ping()
                if cm is not None and cm < obstacle_cm:
                    r = cm / 100 + 0.08                             # the sonar sits ~8 cm behind the robot's nose
                    now = time.time()
                    self._advance(CRUISE_MPS * (now - last))
                    self.obstacle = (self.x + r * math.cos(self.th), self.y + r * math.sin(self.th))
                    self.pause(0.2)
                    self.moved()
                    DASH.say(f'OBSTACLE {cm} cm ahead - stopped')
                    return False
            else:
                time.sleep(CYCLE_S)
            now = time.time()
            self._advance(sign * CRUISE_MPS * (now - last))
            last = now
            self.moved()
        self.pause(0.25)
        if n and abs(steer_sum / n) > 15:
            print(f'  steering bias {steer_sum / n:+.0f} (+ = right side needed more power): '
                  f'tune STRONG_MOTORS / STRONG_RATIO', flush=True)
        return True

    def go_to(self, p, label, reverse_ok=False):
        """Straight leg to point p. With reverse_ok, a point BEHIND the robot is reached by backing up (no turning
        round): the robot faces away from it and reverses."""
        d = math.hypot(p[0] - self.x, p[1] - self.y)
        if d < 0.03:
            return True
        bearing = math.atan2(p[1] - self.y, p[0] - self.x)
        if reverse_ok and abs(wrap(bearing - self.th)) > math.pi / 2:
            return self.leg(d, wrap(bearing + math.pi), backward=True, label=label)
        return self.leg(d, bearing, label=label)


def sonar_sweep(robot):
    """Centre the servo, look right-to-left with the sonar, centre again. The wheels do not move."""
    readings, echoes = [], 0
    for rel in SWEEP_DEG:
        cm = robot.look(SONAR_CENTER_DEG + rel)
        echoes += cm is not None and cm < 400
        readings.append(f'{"right" if rel < 0 else "left" if rel > 0 else "ahead"} {abs(rel)}: '
                        + ('-' if cm is None else 'no echo' if cm >= 400 else f'{cm} cm'))
    robot.look(SONAR_CENTER_DEG)
    DASH.say('Sonar look-around: ' + ', '.join(readings) + ' - centred again')
    if not echoes:
        DASH.think('The sonar heard nothing at any angle. It seems to be powered from the MOTOR battery, so with the '
                   'motor switch off it is deaf - I rely on the camera until the motors are on.')


def look_at_scene(nav, frames=LOOK_FRAMES):
    return build_scene(nav.look(frames), nav.pose)


def relocalise(nav, fire):
    """Back at (about) the start: face the fire, measure its base over several photos. Its BEARING fixes the heading
    and its RANGE + bearing fix the position. A correction too big to be drift is ignored."""
    nav.turn_to(math.atan2(fire['y'] - nav.y, fire['x'] - nav.x), 'turning to the fire to re-localise')
    obs = []
    for blobs in nav.look(5):
        near = [b for b in blobs if b.kind == 'fire' and b.range is not None]
        if near:
            obs.append((near[0].range, near[0].bearing))
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
    """CENTRE, THEN GO, on still pictures (in this light a moving camera blurs the thin yellow rim away): find the
    victim, and if it is more than CENTER_TOL_DEG off the picture's centre, spin towards it and look again (a skid that
    turned too little or too much is seen on the next picture and corrected); only when it is centred drive a hop
    straight at it, then repeat. When its floor contact nears the bottom of the picture it is right in front: creep
    TOUCH_CREEP_M blind to touch it. The first lock goes to the blob that best fits the map's (bearing, rng); after
    that the victim is tracked by bearing. True when touched; False if lost or blocked by something nearer than it."""
    nav.robot.aim(SONAR_CENTER_DEG)
    nav.obstacle = None
    track, gate, misses, locked = bearing, math.radians(ACQUIRE_GATE_DEG), 0, False
    last_range = rng
    for _ in range(MAX_HOPS):
        if STOP_EVENT.is_set():
            return False
        blobs = [b for frame in nav.look(2) for b in frame]          # two still photos
        mine = [b for b in blobs if b.kind == 'victim' and b.range is not None and abs(b.bearing - track) < gate]
        if not mine:
            if locked and last_range <= CLOSE_LOST_M:
                # up close the wheel drops out of the bottom of the picture: it was centred and this near, so it is
                # right in front - creep the rest of the way
                creep = max(0.0, min(CLOSE_LOST_M, last_range - (reach_distance() - TOUCH_CREEP_M)))
                DASH.say(f'Victim {name} is too close for the camera now ({last_range * 100:.0f} cm when last seen) '
                         f'- creeping {creep * 100:.0f} cm to reach it')
                nav.leg(creep, nav.th, label=f'victim {name}', quiet=True, checks=False)
                nav.pause(0.5, f'at victim {name}')
                return True
            misses += 1
            if misses >= 3:
                DASH.say(f'Lost sight of victim {name}')
                return False
            gate = math.radians(ACQUIRE_GATE_DEG)                    # a turn overshot? search wider round the track
            continue
        misses = 0
        b = (min(mine, key=lambda b: abs(b.bearing - track)) if locked else
             min(mine, key=lambda b: (math.degrees(b.bearing - bearing) / 3) ** 2 + ((b.range - rng) / 0.15) ** 2))
        locked = True
        last_range = b.range
        STATUS.state = f'homing on victim {name}: {b.label()}'
        if b.bottom >= H * CLOSE_FRAC:
            DASH.say(f'Victim {name} is right in front - creeping {TOUCH_CREEP_M * 100:.0f} cm to touch it')
            nav.leg(TOUCH_CREEP_M, nav.th, label=f'victim {name}', quiet=True, checks=False)
            nav.pause(0.5, f'touching victim {name}')
            return True
        if abs(b.bearing) > math.radians(CENTER_TOL_DEG):           # not centred: turn, then look again
            nav.turn_to(wrap(nav.th + b.bearing), f'centring victim {name}')
            track, gate = 0.0, math.radians(TRACK_GATE_DEG)
            continue
        hop = max(0.05, min(HOP_M, 0.6 * (b.range - reach_distance())))
        # only something clearly NEARER than the victim is an obstacle: a victim beside the box has the box behind it
        limit = min(OBSTACLE_CM, max(5, int((b.range - 0.10) * 100)))
        if not nav.leg(hop, nav.th, label=f'victim {name}', quiet=True, obstacle_cm=limit) and nav.obstacle:
            return False
        track, gate = 0.0, math.radians(TRACK_GATE_DEG)              # it should still be dead ahead
    DASH.say(f'Victim {name}: too many hops - giving up')
    return False


def add_obstacle(hazards, p):
    hazards.append((p[0], p[1], OBSTACLE_CLEAR_M, 'an obstacle'))
    DASH.set(hazards=[list(h) for h in hazards])
    DASH.say(f'Obstacle marked on the map at ({p[0]:+.2f}, {p[1]:+.2f}) - REROUTING')


def explain_route(start, goal, path, hazards, label):
    near = [(seg_dist(start, goal, h[:2]), h) for h in hazards]
    if len(path) > 1:
        d, h = min(near, key=lambda n: n[0] - n[1][2])
        DASH.think(f'A straight line to {label} would pass {d:.2f} m from {h[3]} (I keep {h[2]:.2f} m), so I go '
                   f'round it: ' + ' -> '.join(f'({x:+.2f},{y:+.2f})' for x, y in path) + '.')
    elif near:
        d, h = min(near, key=lambda n: n[0])
        if d < h[2]:
            DASH.think(f'{label[0].upper() + label[1:]} is right next to {h[3]}, so I go straight in, slowly, '
                       f'watching it with the camera.')
        else:
            DASH.think(f'The straight line to {label} stays {d:.2f} m from {h[3]} - safe, going direct.')


def go_out(nav, goal, hazards, label):
    """Drive the planned route towards goal, all legs but the last (the caller homes in visually for that one).
    Sonar obstacles join the hazards and the route is re-planned. Returns the waypoints actually passed, or None."""
    passed = []
    for _ in range(MAX_REROUTES + 1):
        path = plan((nav.x, nav.y), goal, [h[:3] for h in hazards])
        explain_route((nav.x, nav.y), goal, path, hazards, label)
        DASH.set(path=[[nav.x, nav.y]] + [list(p) for p in path])
        ok = True
        for p in path[:-1]:
            if not nav.go_to(p, label):
                ok = False
                break
            passed.append((nav.x, nav.y))
        if ok:
            return passed
        if nav.obstacle is None:
            return None
        passed.append((nav.x, nav.y))
        add_obstacle(hazards, nav.obstacle)
    DASH.say(f'Too many obstacles on the way to {label} - giving up this route')
    return None


def come_back(nav, passed, fire):
    """Reverse back through the waypoints passed on the way out to the start, without turning round; then re-anchor on
    the fire, fix what is left, and face the arena again."""
    DASH.set(path=[[nav.x, nav.y]] + [list(p) for p in reversed(passed)] + [[0.0, 0.0]])
    for p in list(reversed(passed)) + [(0.0, 0.0)]:
        if not nav.go_to(p, 'the start point', reverse_ok=True) and nav.dead:
            return
    for _ in range(2):
        if fire is None or not relocalise(nav, fire):
            break
        err = math.hypot(nav.x, nav.y)
        if err <= HOME_TOL_M:
            DASH.think(f'Back in the home region (within {err * 100:.0f} cm of the start).')
            break
        DASH.think(f'I am {err * 100:.0f} cm from the start point - correcting.')
        nav.go_to((0.0, 0.0), 'the start point', reverse_ok=True)
    nav.turn_to(0.0, 'facing the arena again')
    DASH.set(path=[])


def wait_until_removed(nav, spot, name):
    """Stay put until the saved victim is gone from its spot (GONE_LOOKS looks in a row): True. False after
    REMOVE_TIMEOUT_S (the caller then ignores that spot)."""
    DASH.say(f'Please take victim {name} out of the arena. I will wait here.')
    gone, t_end = 0, time.time() + REMOVE_TIMEOUT_S
    while not STOP_EVENT.is_set() and time.time() < t_end:
        _, victims = look_at_scene(nav, 3)
        still = any(math.hypot(v['x'] - spot[0], v['y'] - spot[1]) < SAME_VICTIM_M for v in victims)
        gone = 0 if still else gone + 1
        STATUS.state = f'waiting for victim {name} to be picked up ({int(t_end - time.time())} s)'
        if gone >= GONE_LOOKS:
            DASH.say(f'Victim {name} has been taken out of the arena.')
            return True
        STOP_EVENT.wait(1.5)
    DASH.say(f'Victim {name} was not taken out in {REMOVE_TIMEOUT_S} s - I will ignore that spot.')
    return False


def publish_scene(fire, victims, names, order, saved):
    objs = []
    if fire:
        objs.append({'name': 'FIRE', 'kind': 'fire', 'x': fire['x'], 'y': fire['y'], 'rank': None, 'reached': False,
                     'ax': fire['a'][0], 'ay': fire['a'][1], 'bx': fire['b'][0], 'by': fire['b'][1]})
    for v in victims:
        objs.append({'name': names[id(v)], 'kind': 'victim', 'x': v['x'], 'y': v['y'],
                     'rank': order.index(v) + 1, 'reached': False})
    for i, (sx, sy, sname) in enumerate(saved):
        objs.append({'name': sname, 'kind': 'victim', 'x': sx, 'y': sy, 'rank': None, 'reached': True})
    DASH.set(objs=objs, hazards=[list(h) for h in fire_hazards(fire)])


def victim_name(i):
    """A, B, ... Z, AA, AB, ... - victims keep coming."""
    return victim_name(i // 26 - 1) + chr(ord('A') + i % 26) if i >= 26 else chr(ord('A') + i)


def mission(nav, drive=True):
    """Runs until Ctrl+C: watch the arena from the safe area (the start), rescue the victim closest to the fire, come
    back, wait for it to be taken out, look again. Victims may be added at any time."""
    DASH.say('Staying where I am. Centring the sonar, then looking left and right.')
    sonar_sweep(nav.robot)
    saved_n, named, known = 0, 0, []                               # known: [(x, y, name)] from the last look
    history = []                                                    # [(x, y, name)]: saved, for the map
    ignored = []                                                    # [(x, y)]: spots never removed / never reached
    tries = []                                                      # [[x, y, n]]: failed attempts per spot
    motors_ok, watching = False, False
    while not STOP_EVENT.is_set():
        fire, victims = look_at_scene(nav)
        victims = [v for v in victims if all(math.hypot(v['x'] - s[0], v['y'] - s[1]) >= SAME_VICTIM_M
                                             for s in ignored)]
        victims.sort(key=lambda v: -v['bearing'])                   # new names go left to right
        names = {}
        for v in victims:                                           # a victim keeps its name from the last look
            old = next((k for k in known if math.hypot(k[0] - v['x'], k[1] - v['y']) < SAME_VICTIM_M), None)
            names[id(v)] = old[2] if old else victim_name(named)
            named += old is None
        known = [(v['x'], v['y'], names[id(v)]) for v in victims]
        order = sorted(victims, key=lambda v: v['d'] if fire else v['r'])
        publish_scene(fire, victims, names, order, history)
        if not victims:
            if not drive:
                DASH.think('No victims in view.')
                return
            if not watching:
                DASH.say(f'Watching the arena from the safe area ({saved_n} saved so far). Add a victim any time.')
                watching = True
            DASH.set(target=None, path=[], state='watching from the safe area - add a victim')
            STOP_EVENT.wait(WATCH_S)
            continue
        watching = False
        DASH.say('Looking at the arena')
        if fire:
            DASH.think(f'Fire {fire["r"]:.2f} m ahead, {abs(math.degrees(fire["bearing"])):.0f} deg '
                       f'{"left" if fire["bearing"] > 0 else "right"}. I see {len(victims)} '
                       f'victim{"s" * (len(victims) != 1)}:')
            for v in order:
                DASH.think(f'  victim {names[id(v)]}: {v["d"]:.2f} m from the fire, {v["r"]:.2f} m from me')
        else:
            DASH.think('I cannot see the fire. Without it I cannot judge danger, so I will take the nearest victim.')
        target = choose(victims, fire, nav.pose)
        name = names[id(target)]
        if fire and len(order) > 1:
            gap = order[1]['d'] - target['d']
            DASH.think(f'Victim {name} is closest to the fire, so it is in the most danger: I save it first'
                       + (f' ({gap * 100:.0f} cm nearer the fire than the next one).' if gap < 0.10 else '.')
                       + ' Next would be ' + ', then '.join(names[id(v)] for v in order[1:]) + '.')
        else:
            DASH.think(f'Victim {name} is the one to save now.')
        if not drive:
            DASH.set(state='plan only - nothing moves (run without --plan to do it)')
            return
        if not motors_ok:
            for n in range(MOTOR_ON_COUNTDOWN_S, 0, -1):
                DASH.set(state=f'READY: switch the MOTOR battery ON now - moving in {n} s')
                print(f'Switch the motor battery ON now - moving in {n} s', flush=True)
                if STOP_EVENT.wait(1):
                    return
            motors_ok = True
        DASH.set(target=name)
        DASH.say(f'Going to save victim {name}')
        others = [(o['x'], o['y'], VICTIM_CLEAR_M, f'victim {names[id(o)]}') for o in victims if o is not target]
        hazards = [h for h in fire_hazards(fire)] + others
        passed = go_out(nav, (target['x'], target['y']), hazards, f'victim {name}')
        reached = False
        if passed is not None:
            nav.turn_to(math.atan2(target['y'] - nav.y, target['x'] - nav.x), f'facing victim {name}')
            reached = home_on(nav, name, wrap(math.atan2(target['y'] - nav.y, target['x'] - nav.x) - nav.th),
                              math.hypot(target['x'] - nav.x, target['y'] - nav.y))
        if reached:
            DASH.say(f'Victim {name} RESCUED - I reached it. Going back to the safe area.')
            back = reach_distance() - TOUCH_CREEP_M                 # we are just short of it, facing it
            nav.x, nav.y = target['x'] - back * math.cos(nav.th), target['y'] - back * math.sin(nav.th)
            nav.moved()
        else:
            DASH.say(f'Could not reach victim {name} - going back to the safe area.')
        if nav.dead:
            DASH.say('STOPPED: the motors cannot turn the robot (is the motor switch on? batteries?)')
            return
        come_back(nav, passed or [], fire)
        DASH.set(target=None)
        if reached:
            saved_n += 1
            history.append((target['x'], target['y'], name))
            DASH.say(f'Victim {name} SAVED - I am back in the safe area ({saved_n} saved so far).')
            if not wait_until_removed(nav, (target['x'], target['y']), name):
                ignored.append((target['x'], target['y']))
        else:
            t = next((t for t in tries if math.hypot(t[0] - target['x'], t[1] - target['y']) < SAME_VICTIM_M), None)
            if t is None:
                t = [target['x'], target['y'], 0]
                tries.append(t)
            t[2] += 1
            if t[2] >= MAX_TRIES:
                ignored.append((t[0], t[1]))
                DASH.think(f'Victim {name} could not be reached in {MAX_TRIES} tries - I leave that spot alone.')
            else:
                DASH.think(f'Victim {name} could not be reached - I will try again.')


# ---- calibration and tests ----------------------------------------------------------------------------------------
def calibrate_horizon(cam, dist):
    """One wheel at a tape-measured `dist` metres straight ahead (along the floor, from the lens to the wheel's NEAR
    edge): where it meets the floor fixes the horizon row, i.e. the camera's tilt - the error that matters most for
    depth. Writes calibration.json, which every later run loads."""
    print(f'Put ONE wheel {dist:.2f} m straight in front of the camera (from the lens to the near edge of the wheel), '
          f'nothing else in view on the floor. Measuring in 3 s...', flush=True)
    if STOP_EVENT.wait(3):
        return
    rows = []
    t_end = time.time() + 6
    while time.time() < t_end and not STOP_EVENT.is_set():
        wheels = [b for b in cam.snap(time.time()) if b.kind == 'victim']
        if wheels:
            rows.append(min(wheels, key=lambda b: abs(b.bearing)).bottom)
    if len(rows) < 10:
        print(f'Saw the wheel in only {len(rows)} frames - is it in view and lit? Nothing saved.')
        return
    row = statistics.median(rows)
    horizon = row - CAM_HEIGHT_M * F_PX / dist
    print(f'wheel meets the floor at row {row:.1f} (spread {max(rows) - min(rows)} px over {len(rows)} frames) -> '
          f'horizon row {horizon:.1f} (was {HORIZON_Y})')
    CAL_FILE.write_text(json.dumps({'horizon_y': round(horizon, 1), 'measured_at_m': dist,
                                    'date': time.strftime('%Y-%m-%d %H:%M')}, indent=1) + '\n')
    print(f'saved to {CAL_FILE.name}: every run now uses it')


def sonar_cal(robot):
    """Sweep the servo; with one flat object straight ahead, the angles that see it nearest are centred on 'ahead'."""
    print('Put a flat object (a book) about 30 cm STRAIGHT in front of the robot. Sweeping in 3 s...')
    if STOP_EVENT.wait(3):
        return
    seen = []
    for deg in range(100, 157, 3):
        cm = robot.look(deg)
        print(f'  servo {deg:3d}: {cm} cm')
        if cm is not None and cm < 400:
            seen.append((deg, cm))
    robot.look(SONAR_CENTER_DEG)
    if not seen:
        print('No echoes at any angle: is the sonar powered (VCC/GND) and is the book within ~1 m?')
        return
    best = min(cm for _, cm in seen)
    near = [deg for deg, cm in seen if cm <= best + 3]
    print(f'Nearest {best} cm at servo {near[0]}..{near[-1]}: set SONAR_CENTER_DEG = {round(sum(near) / len(near))}')


def synthetic_scene(wheels, box=(100, 260, 375), seed=0):
    """A picture like this room (bright floor, dark background above the horizon), with the box standing on the floor
    and flat wheels (a pale-yellow rim over a black tyre) at the given (centre column, floor-contact row) - for the
    self-test. Colours are the ones measured on 2026-10-21."""
    rng = np.random.default_rng(seed)
    hsv = np.zeros((H, W, 3), np.uint8)
    hsv[:] = (20, 80, 40)                                           # dim tan/olive background (curtain, chair)
    hsv[int(HORIZON_Y) + 10:] = (98, 52, 168)                       # the floor
    x0, x1, base = box
    hsv[base - 180:base, x0:x1] = (18, 166, 61)                     # the box, standing on the floor
    for cx, contact in wheels:
        r = floor_range(contact)
        half = int(0.065 / 2 * F_PX / r)
        tyre = max(4, int(0.026 * F_PX / r))
        hsv[contact - tyre:contact, cx - half:cx + half] = (50, 89, 20)        # black tyre
        hsv[contact - tyre - 3:contact - tyre, cx - half:cx + half] = (28, 90, 140)   # pale-yellow rim
    hsv = np.clip(hsv.astype(int) + rng.integers(-3, 4, hsv.shape), 0, 255).astype(np.uint8)
    hsv[..., 0] = np.clip(hsv[..., 0], 0, 179)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def selftest():
    # the detector on a picture of this room: a box and two flat wheels on the floor
    img = synthetic_scene([(222, 377), (411, 383)])
    blobs = detect(img)
    fires = [b for b in blobs if b.kind == 'fire']
    wheels = sorted((b for b in blobs if b.kind == 'victim'), key=lambda b: b.x)
    assert len(fires) == 1 and len(wheels) == 2, [b.label() for b in blobs]
    assert abs(fires[0].bottom - 375) <= 2 and abs(wheels[0].bottom - 377) <= 2 and abs(wheels[1].bottom - 383) <= 2
    assert wheels[1].bearing < 0 < wheels[0].bearing                       # right of centre = negative bearing
    # the decision: the wheel at the box's front is closest to the fire, though the other is nearer the robot
    fire, victims = build_scene([blobs] * LOOK_FRAMES, (0.0, 0.0, 0.0))
    assert fire is not None and len(victims) == 2, (fire, victims)
    first = choose(victims, fire, (0, 0, 0))
    assert first['bearing'] == max(v['bearing'] for v in victims), victims  # the left wheel, at the box
    assert first['d'] < 0.1 and max(v['d'] for v in victims) > 0.25, [v['d'] for v in victims]
    assert min(victims, key=lambda v: v['r']) is not first                  # nearest-first would get it wrong
    # a speck seen in one photo out of six is noise; two blobs in one photo are two victims
    speck = Blob('victim', 300, 380, 20, 395)
    fire2, v2 = build_scene([blobs + [speck]] + [blobs] * (LOOK_FRAMES - 1), (0.0, 0.0, 0.0))
    assert len(v2) == 2, v2
    # distance to the fire is to its base EDGE: a wheel by the box's corner is close to the fire
    a, b = (1.0, 0.2), (1.0, -0.2)                                       # a 40 cm box, 1 m ahead
    assert seg_dist(a, b, (0.95, -0.25)) < 0.08 and math.hypot(0.95 - 1.0, -0.25) > 0.2
    # nothing on the floor = nothing found; a dark background above the horizon is not a wheel
    assert detect(synthetic_scene([], box=(0, 1, HORIZON_Y + 20))) == []
    # routes: a fire on the straight line forces a detour that keeps its distance
    path = plan((0, 0), (2, 0), [(1, 0.1, 0.4)])
    assert len(path) == 2 and path[-1] == (2, 0) and path[0][1] < 0
    for a, b in zip([(0, 0)] + path, path):
        assert seg_dist(a, b, (1, 0.1)) > 0.36, (a, b)
    assert plan((0, 0), (2, 0), [(2.2, 0, 0.4)]) == [(2, 0)]               # a victim beside the fire: reachable
    # depth: lower in the image = nearer; at/above the horizon = no range
    assert floor_range(HORIZON_Y + 30) > floor_range(HORIZON_Y + 60) and floor_range(HORIZON_Y) is None
    assert 0.15 < reach_distance() < 0.3
    # turning LEFT slides the scene RIGHT: rotation() must call that +
    band = np.float32(cv2.GaussianBlur(np.random.default_rng(1).random((230, 480)), (0, 0), 3))
    assert math.degrees(rotation(band, np.roll(band, 40, axis=1))[0]) > 3
    assert math.degrees(rotation(band, np.roll(band, -40, axis=1))[0]) < -3
    # skid spins: the two sides run opposite ways, and a LEFT spin has the right side going forward
    for left, (spin, _) in SPINS.items():
        sides = [spin[m] * wb.FORWARD_SIGN[m] for m in range(4)]           # + = that motor drives forward
        lft, rgt = (sides[0], sides[1]), (sides[3], sides[2])               # M1+M2 left, M4+M3 right
        assert all(v > 0 for v in (rgt if left else lft)) and all(v < 0 for v in (lft if left else rgt)), (left, spin)
    # the strong motors (M2, M4) get less power than the others for the same command
    fwd = wb.drive_powers(200, 200)
    assert abs(fwd[1]) < abs(fwd[0]) and abs(fwd[3]) < abs(fwd[2]), fwd
    # turning: pulse, measure, add more - converges even when skidding loses half of every pulse
    for efficiency in (1.0, 0.5):
        robot, cam = FakeRobot(efficiency), None
        cam = robot.cam
        nav = Nav(robot, cam)
        for target in (math.radians(30), math.radians(-12), math.radians(5)):
            assert nav.turn_to(target), (efficiency, math.degrees(target), math.degrees(nav.th))
            assert abs(math.degrees(wrap(nav.th - target))) <= TURN_TOL_DEG * 1.5, (efficiency, math.degrees(nav.th))
    json.loads(DASH.json())
    print('selftest: OK')


class FakeRobot:
    """For the self-test: a robot whose skid spins turn it at 26 deg/s x `efficiency` (the rest lost to skidding),
    after a 50 ms break-away, and a camera that reports that heading."""

    def __init__(self, efficiency):
        self.efficiency, self.cmd, self.t = efficiency, wb.STOP, time.time()
        robot = self

        class Cam:
            @property
            def yaw(self):
                robot._integrate()
                return robot._yaw
        self.cam, self._yaw, self._since = Cam(), 0.0, time.time()

    def _integrate(self):
        now = time.time()
        rate = {SPINS[True][0]: 1, SPINS[False][0]: -1}.get(tuple(self.cmd), 0) * math.radians(SPIN_RATE_DPS)
        if now - self._since > 0.05:                                # past the break-away
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
    """Only one copy can hold the camera and the Uno: stop any earlier rescue_bot.py (a forgotten --plan or --view,
    say) before starting."""
    import os
    me, others = os.getpid(), []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        # only a PYTHON process running this script - not the shell or `timeout` that launched one (their command
        # lines mention rescue_bot.py too)
        if int(pid) != me and Path(argv[0].decode(errors='replace')).name.startswith('python') \
                and any(a.endswith(b'rescue_bot.py') for a in argv[1:]):
            others.append(int(pid))
    if not others:
        return
    print(f'Stopping the copy of rescue_bot.py that is already running ({", ".join(map(str, others))})...', flush=True)
    for p in others:
        try:
            os.kill(p, signal.SIGINT)
        except ProcessLookupError:
            pass
    for _ in range(50):                                             # up to 5 s to let go of the camera
        if not any(os.path.exists(f'/proc/{p}') for p in others):
            return
        time.sleep(0.1)
    for p in others:
        try:
            os.kill(p, signal.SIGKILL)
        except ProcessLookupError:
            pass
    time.sleep(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag in ('--view', '--plan', '--sonar-cal', '--selftest'):
        ap.add_argument(flag, action='store_true')
    ap.add_argument('--calibrate-horizon', type=float, metavar='METRES')
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP_EVENT.set())
    stop_other_copies()
    cam = Camera()
    robot = None
    try:
        if not args.sonar_cal:
            try:
                cam.start()
            except RuntimeError:
                print('\nThe camera is busy: another program has it. Usually the drone project - run\n'
                      '    sudo systemctl stop targetfire\nand start this again.')
                return
        if args.calibrate_horizon:
            calibrate_horizon(cam, args.calibrate_horizon)
            return
        if args.view:
            DASH.set(state='view only - nothing moves')
            print('Viewing only. Ctrl+C to stop.')
            STOP_EVENT.wait()
            return
        robot = Uno(False)
        robot.open()
        robot.look(SONAR_CENTER_DEG)                                # centred before anything else happens
        if args.sonar_cal:
            sonar_cal(robot)
            return
        STOP_EVENT.wait(2)                                          # let the camera settle
        mission(Nav(robot, cam), drive=not args.plan)
        DASH.set(state=DASH.state if args.plan else 'finished - Ctrl+C to exit (the dashboard stays up until then)')
        STOP_EVENT.wait()                                           # keep the dashboard up to read the result
    except KeyboardInterrupt:
        pass
    finally:
        STOP_EVENT.set()
        if robot:
            robot.close()
        print('Stopped.')


if __name__ == '__main__':
    main()
