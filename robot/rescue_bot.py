#!/usr/bin/env python3
"""FIRERESQ RESCUE BOT - run ON THE PI, from the same folder as wander_bot.py, wander_fw.hex and dashboard.html.

THE TASK. A fire (the red shoe box) and victims (yellow-hubbed wheels) stand on the floor. From its start spot the
robot surveys, maps them, ranks the victims by how close each is to the fire, then for each in that order: drives
right up to it on a route that keeps clear of the fire, and comes back to the start. It ends at the start, facing the
arena. Nothing is hardcoded: every run maps the arena afresh, so the wheels can be anywhere it can see.

    python3 -u rescue_bot.py --view          # camera + detection only, motors untouched
    python3 -u rescue_bot.py --survey        # survey, map and rank - turns in place, does not drive anywhere
    python3 -u rescue_bot.py                 # THE MISSION (5 s countdown)
    python3 -u rescue_bot.py --sonar-cal     # flat object 30 cm straight ahead: checks SONAR_CENTER_DEG (no motors)
    python3 -u rescue_bot.py --selftest      # the maths, no hardware
    python3 -u rescue_bot.py --robustness    # Monte-Carlo: how often the ranking is right, given the camera's noise
    python3 -u rescue_bot.py --calibrate-horizon 1.0   # ONE wheel at a measured 1.00 m straight ahead (no motors):
                                             # measures the camera's tilt, the biggest depth error, into calibration.json

DASHBOARD: http://pi.local:8090 - live camera, the map, what the robot is doing, and why.

DEPTH - how a flat camera picture gives distances on the floor
    The camera is CAM_HEIGHT_M above a flat floor and looks level, so the horizon is a fixed image row (HORIZON_Y).
    A point on the floor N pixels below the horizon lies on a ray that dips atan(N / F_PX) below level, and that ray
    meets the floor at   range = (CAM_HEIGHT_M - z) * F_PX / N,   where z is the point's own height above the floor
    (0 for the box's base, the wheel radius for a hub's centre). The bearing comes from the pixel column. So every
    sighting becomes an (x, y) on the floor. Pixel height (object size) is NOT used: a hub is ~15 px tall at 2 m,
    so one pixel is a 7% range error, where the floor geometry is much steadier.
    The error grows with range squared: one pixel row is ~2 cm at 1 m and ~9 cm at 2 m for a hub. Hence three
    defences for the ORDER (which is all that matters - "who is closest to the fire"):
      1. Same-photo distances. A victim's distance to the fire is measured between the two of them IN THE SAME
         PHOTO wherever both are in view, so the robot's own position and heading errors cancel exactly.
      2. Many photos. Every view takes several frames and every object is seen from 2-3 views; medians throughout.
      3. Close calls are re-measured from nearer. If two victims' fire distances differ by less than their
         measurement uncertainty, the robot drives halfway to the fire (a quarter of the error) and looks again.
    --robustness quantifies all of it against simulated pixel noise.

MOVING WITHOUT ENCODERS (measured on this robot, 2026-10-20). Skid or pivot turns do not reliably turn it on
tile, so turns are ARCS: all wheels roll forward (or all backward), one side slower; a forward arc then a backward arc
turns it roughly on the spot. HEADING comes from the camera: the camera thread adds up the sideways shift between
every pair of consecutive frames (phase correlation) - never a big jump, so never lost. Straight legs hold that heading
(which also cancels the weak motor's pull) and are timed (CRUISE_MPS). The way back is driven IN REVERSE along the
same route, so the robot never turns round and a wrong CRUISE_MPS cancels out; then the fire's position, seen from the
start, re-anchors heading and position.
Map frame = the start pose: x forward, y left, angles CCW (+ = left).
"""
import argparse
import collections
import json
import math
import random
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
DASHBOARD_HTML = Path(__file__).resolve().parent / 'dashboard.html'
VO_ROWS = (100, 330)                     # band used to track rotation: mostly ABOVE the horizon, where distant walls
#                                          barely shift as the robot moves forward (floor texture would)
VO_MIN_CONF = 0.05                       # frame-to-frame match quality below this is ignored (motion blur, a blank wall)

# ---- what to look for: HSV (OpenCV: H 0-179), sampled from this camera in the room's light on 2026-10-20 ----------
COLOURS = {
    'fire':   [((0, 150, 60), (8, 255, 255)), ((170, 150, 60), (179, 255, 255))],     # red shoe box
    'victim': [((20, 150, 70), (33, 255, 255))],     # yellow wheel hubs (cardboard is duller: S 140-170, V 50-77)
}
ROUND = {'victim': (0.5, 2.0)}           # allowed width/height of a blob: a hub is round, a stripe of cardboard is not
MIN_AREA = {'fire': 300, 'victim': 60}   # px of the upright image; smaller blobs are noise
Z_M = {'fire': 0.0, 'victim': 0.0325}    # height of the point whose floor range is measured: box base, hub centre
HORIZON_Y = 318                          # image row of the horizon (camera tilt). Measured: the box's base agrees with
#                                          its size here. --calibrate-horizon measures it properly (saved below)
CAL_FILE = Path(__file__).resolve().parent / 'calibration.json'
try:
    HORIZON_Y = json.loads(CAL_FILE.read_text())['horizon_y']
except (OSError, ValueError, KeyError):
    pass
PX_NOISE = 1.5                           # assumed std of a blob's reference row (px): drives the uncertainty estimates
BOX_COLOUR = {'fire': (0, 0, 255), 'victim': (0, 255, 255)}

# ---- motion -----------------------------------------------------------------------------------------------------
# The TT motors are 3-6 V parts and the L293D drops ~2 V, so the PWM is capped to keep the motors at <= ~7.5 V on
# average whatever the pack is: 4xAA (6 V) gets full power, 8xAA in series (12 V) is capped to 75%.
MOTOR_SUPPLY_V = 6.0                     # pack on EXT_PWR: 4xAA or 8xAA in PARALLEL = 6.0, 8xAA in series = 12.0
POWER_SCALE = min(1.0, 7.5 / (MOTOR_SUPPLY_V - 2.0))
wb.MAX_POWER = round(255 * POWER_SCALE)  # nothing is ever commanded above this
# One motor turns slower than the other three. The weak motor gets the full command and the other three are scaled to
# WEAK_RATIO of it, so all four match at the top. Heading hold on straight legs mops up whatever this misses.
WEAK_MOTOR = 3                           # TODO(calibrate): shield terminal M1..M4 of the slow motor (robot.yaml: M3)
WEAK_RATIO = 0.85                        # TODO(calibrate): see the 'steering bias' line each straight leg prints
wb.TRIM = tuple(1.0 if m + 1 == WEAK_MOTOR else WEAK_RATIO for m in range(4))
# ARC turns: every wheel rolls the way the robot faces, one side slower - nothing is dragged sideways. To turn right
# the left side runs faster than the right, forwards or backwards. The slow side stays above stall (~110 after trim).
ARC_OUTER = 235
ARC_INNER = 135
ARCS = {True: (wb.drive_powers(ARC_INNER, ARC_OUTER), wb.drive_powers(-ARC_OUTER, -ARC_INNER)),    # LEFT: fwd, back
        False: (wb.drive_powers(ARC_OUTER, ARC_INNER), wb.drive_powers(-ARC_INNER, -ARC_OUTER))}   # RIGHT: fwd, back
ARC_MAX_S = 1.0                          # longest single arc before switching direction (keeps the robot in place)
TURN_TOL_DEG = 3.0
TURN_LEAD_DEG = 4.0                      # stop an arc this early: the camera reports ~1 frame late and the robot coasts
STALL_DEG = 1.0                          # an arc that turned less than this did not move the robot
MAX_STALLS = 6
MAX_ARCS = 24                            # give up a turn after this many arcs (it is oscillating or stuck)
CRUISE = round(210 * POWER_SCALE)        # forward power for legs and homing
CRUISE_MPS = 0.25                        # TODO(calibrate): forward speed at CRUISE. Reverse return cancels its error
ARC_MPS = CRUISE_MPS * (ARC_OUTER + ARC_INNER) / 2 / 210   # how fast an arc carries the robot along
K_HEAD = 160                             # steering power per radian of heading error on straight legs
SETTLE_S = 0.35                          # stop this long before trusting a frame
CYCLE_S = 0.06                           # control loop period (the Uno's watchdog is 400 ms)

# ---- sonar (servo-mounted HC-SR04) ------------------------------------------------------------------------------
SONAR_CENTER_DEG = 128                   # servo angle that points STRAIGHT AHEAD, found by hand (higher = left)
OBSTACLE_CM = 22                         # something this close ahead (and closer than the target) = stop

# ---- mission ----------------------------------------------------------------------------------------------------
SURVEY_VIEWS_DEG = (60, 30, 0, -30, -60, -90)   # turn to +60 once, then scan RIGHT; 30 deg steps overlap (FOV 41)
FRAMES_PER_VIEW = 3
MERGE_M = 0.30                           # sightings of one kind closer than this are the same object
FIRE_MERGE_M = 0.60                      # red blobs this close are one fire (the box and its shadowed side, say)
MIN_SIGHTINGS = 2                        # an object seen fewer times than this is noise
CLOSE_CALL_SIGMAS = 2.0                  # fire distances closer than this many std apart get re-measured from nearer
VANTAGE_FRACTION = 0.5                   # re-measure close calls from this far along the line start -> fire: halfway
#                                          quarters the depth error, and stays clear of the fire even when its own
#                                          range is badly over-estimated (a 3 px tilt error = +25% at 2.5 m)
FIRE_CLEAR_M = 0.40                      # routes keep this far from the fire...
OBSTACLE_CLEAR_M = 0.25                  # ...from an obstacle the sonar found...
VICTIM_CLEAR_M = 0.20                    # ...and from the victims that are not the current target
MAX_REROUTES = 3
FIRE_STOP_M = 0.25                       # fire straight ahead and this close while homing: stop
K_STEER = 150                            # motor power per radian of bearing error while homing
TRACK_GATE_DEG = 12                      # victims look alike: a tracked blob must stay this close to where it was
ACQUIRE_GATE_DEG = 25                    # ...and this close to the map's prediction when homing starts
LOST_S = 2.0
CLOSE_FRAC = 0.95                        # victim's hub this low in the image = about to pass under the camera
CREEP_M = 0.03                           # then creep this much further, blind: the hub centre is 0.15 m from the lens
#                                          when it leaves the image, the wheel face ~0.11 m: this leaves a ~5 cm gap
HOME_TOL_M = 0.12                        # back at the start when re-localised within this
MAX_FIX_M = 0.6                          # a fire-based correction bigger than this is a mis-sighting: ignored


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- the dashboard's data ---------------------------------------------------------------------------------------
class Dash:
    """Everything the web dashboard shows. The mission writes it, the web server reads it (as JSON, under the lock).
    say() = what the robot is DOING (bottom left), think() = WHY (bottom right). Both also print to the terminal."""

    def __init__(self):
        self.lock = threading.Lock()
        self.log, self.thoughts = [], []
        self.objs = []                   # [{name, kind, x, y, rank, reached}]
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
def floor_range(kind, ref_row, horizon=HORIZON_Y):
    """Floor range (m) of an object whose reference point (box base / hub centre) is at image row ref_row; None if
    that row is at or above the horizon (not on the floor, or too far to tell)."""
    drop = ref_row - horizon
    return None if drop < 4 else (CAM_HEIGHT_M - Z_M[kind]) * F_PX / drop


def range_sigma(kind, r):
    """1-sigma range error from PX_NOISE pixels of row error: dr = r^2 / ((h - z) f) per pixel."""
    return r * r * PX_NOISE / ((CAM_HEIGHT_M - Z_M[kind]) * F_PX)


class Blob:
    """One coloured blob in the upright image, with its bearing and (if it can be measured) its floor range."""

    def __init__(self, kind, x, y, w, h):
        self.kind, self.x, self.y, self.w, self.h = kind, x, y, w, h
        self.bottom = y + h
        self.bearing = math.atan2(W / 2 - (x + w / 2), F_PX)       # rad, + = left of the camera axis
        ref = y + h / 2 if kind == 'victim' else self.bottom
        clipped = kind == 'fire' and self.bottom >= H - 2           # the box's base is out of the picture
        self.range = None if clipped else floor_range(kind, ref)

    def label(self):
        r = '?' if self.range is None else f'{self.range:.2f}m'
        return f'{self.kind} {r} {math.degrees(self.bearing):+.0f}'


KERNEL = np.ones((5, 5), np.uint8)


def detect(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blobs = []
    for kind, ranges in COLOURS.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            x, y, w, h = cv2.boundingRect(c)
            lo, hi = ROUND.get(kind, (0, 99))
            if cv2.contourArea(c) >= MIN_AREA[kind] and lo <= w / h <= hi:
                blobs.append(Blob(kind, x, y, w, h))
    return blobs


def rotation(prev, cur):
    """(rad, confidence): how far the camera turned between two grey bands (+ = left: the scene slides right when
    the camera turns left). Confidence ~0.8 for an unmoved scene, lower the more it changed."""
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
            cv2.line(frame, (0, HORIZON_Y), (W, HORIZON_Y), (90, 90, 90), 1)
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


# ---- mapping and deciding (pure: --selftest and --robustness exercise these) ------------------------------------
Sighting = collections.namedtuple('Sighting', 'kind x y r frame view area ox oy')   # (ox, oy): seen from here


def sightings_from(blobs, x, y, th, frame, view):
    """Blobs seen from pose (x, y, th) -> floor-plane Sightings in the map frame (blobs without a range are dropped)."""
    out = []
    for b in blobs:
        if b.range is not None:
            a = th + b.bearing
            out.append(Sighting(b.kind, x + b.range * math.cos(a), y + b.range * math.sin(a), b.range, frame, view,
                                b.w * b.h, x, y))
    return out


def same_object(s, m, merge):
    """Could sightings s and m be the same object? Seen from (about) the same spot, compare BEARING, which is precise
    (~0.1 deg/px), and RANGE within its expected error (which grows with range^2 - a fixed radius splits one far wheel
    into two); seen from different spots, compare floor positions."""
    if math.hypot(s.ox - m.ox, s.oy - m.oy) < 0.25:
        da = abs(wrap(math.atan2(s.y - s.oy, s.x - s.ox) - math.atan2(m.y - m.oy, m.x - m.ox)))
        sig = max(range_sigma(s.kind, s.r), range_sigma(s.kind, m.r))
        wide = math.radians(10 if s.kind == 'fire' else 4)          # the box is wide: its blob centre wanders
        return da < wide and abs(s.r - m.r) < max(merge, 3 * sig)
    return math.hypot(s.x - m.x, s.y - m.y) < merge


def cluster(sightings, kind, merge):
    """Group one kind's sightings into objects: [{'x', 'y', 'members'}], each at the MEDIAN of its members (one bad
    sighting cannot drag it)."""
    objs = []
    for s in (s for s in sightings if s.kind == kind):
        # two blobs in ONE photo are two objects, however close: that keeps neighbours apart while the range
        # tolerance stays wide enough not to split one far wheel in two
        hits = [o for o in objs if all(m.frame != s.frame for m in o['members'])
                and any(same_object(s, m, merge) for m in o['members'])]
        if hits:
            o = max(hits, key=lambda o: len(o['members']))
            o['members'].append(s)
            o['x'] = statistics.median(m.x for m in o['members'])
            o['y'] = statistics.median(m.y for m in o['members'])
        else:
            objs.append({'x': s.x, 'y': s.y, 'members': [s]})
    return objs


def build_map(sightings):
    """-> (fire or None, [victims]). The fire is the red cluster with the most evidence (sightings x pixel area): the
    box, not a red speck. Victims need MIN_SIGHTINGS. Each object: {'x', 'y', 'members'}."""
    fires = cluster(sightings, 'fire', FIRE_MERGE_M)
    fire = max(fires, key=lambda f: sum(m.area for m in f['members']), default=None)
    victims = [v for v in cluster(sightings, 'victim', MERGE_M) if len(v['members']) >= MIN_SIGHTINGS]
    return fire, victims


def fire_distance(victim, fire):
    """(distance, sigma, how) from victim to fire. Preferred: measured between the two IN THE SAME FRAME (the robot's
    pose error is common to both and cancels), median over frames; sigma from the range noise model, shrunk by the
    number of independent views. Fallback: between the two map positions."""
    pairs, sig2, views = [], [], set()
    by_frame = collections.defaultdict(list)
    for f in fire['members']:
        by_frame[f.frame].append(f)
    for v in victim['members']:
        if by_frame[v.frame]:
            f = max(by_frame[v.frame], key=lambda f: f.area)
            pairs.append(math.hypot(v.x - f.x, v.y - f.y))
            sig2.append(range_sigma('victim', v.r) ** 2 + range_sigma('fire', f.r) ** 2)
            views.add(v.view)
    if pairs:
        return statistics.median(pairs), math.sqrt(statistics.median(sig2) / len(views)), 'same-photo'
    v_r = statistics.median(m.r for m in victim['members'])
    f_r = statistics.median(m.r for m in fire['members'])
    nv = len({m.view for m in victim['members']})
    sigma = math.sqrt(range_sigma('victim', v_r) ** 2 / nv + range_sigma('fire', f_r) ** 2)
    return math.hypot(victim['x'] - fire['x'], victim['y'] - fire['y']), sigma, 'map'


def rank(victims, fire):
    """Victims sorted closest-to-fire first, each annotated with 'd', 'sigma', 'how'; plus the list of adjacent pairs
    (i, i+1) whose difference is within CLOSE_CALL_SIGMAS of their combined uncertainty."""
    for v in victims:
        v['d'], v['sigma'], v['how'] = fire_distance(v, fire) if fire else (math.hypot(v['x'], v['y']), 0.0, 'robot')
    order = sorted(victims, key=lambda v: v['d'])
    close = [i for i in range(len(order) - 1)
             if order[i + 1]['d'] - order[i]['d'] < CLOSE_CALL_SIGMAS * math.hypot(order[i]['sigma'],
                                                                                   order[i + 1]['sigma'])]
    return order, close


def unlikeness(b1, r1, b2, r2):
    """How implausible it is that (bearing b1, range r1) and (b2, r2) - both seen from one spot - are the same wheel,
    in squared standard deviations: bearing is good to a few degrees (heading drift), range to its range^2 error.
    Bearing alone confuses two wheels that line up; range alone confuses two at the same distance."""
    r_err = max(0.08, 2 * range_sigma('victim', max(r1 or 0, r2 or 0)))
    db = wrap(b1 - b2) / math.radians(3)
    dr = 0 if r1 is None or r2 is None else (r1 - r2) / r_err
    return db * db + dr * dr


def closest_on_segment(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return a[0] + t * dx, a[1] + t * dy


def seg_dist(a, b, p):
    q = closest_on_segment(a, b, p)
    return math.hypot(q[0] - p[0], q[1] - p[1])


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
    """How far ahead a hub's centre is when it reaches CLOSE_FRAC of the image."""
    return floor_range('victim', H * CLOSE_FRAC)


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
    the fire re-anchors; position = dead reckoning (timed legs, arcs), re-anchored on the fire at the start.
    `obstacle` = where the sonar last stopped a leg; `dead` = the motors will not turn the robot."""

    def __init__(self, robot, cam):
        self.robot, self.cam = robot, cam
        self.x = self.y = 0.0
        self.yaw0 = cam.yaw
        self.obstacle = None
        self.dead = False

    @property
    def th(self):
        return wrap(self.cam.yaw - self.yaw0)

    def set_heading(self, th):
        self.yaw0 = self.cam.yaw - th

    def moved(self):
        DASH.moved(self.x, self.y, self.th)

    def pause(self, s=SETTLE_S, state='stopped'):
        hold(self.robot, STATUS, wb.STOP, s, state)

    def look(self, frames=1):
        """Stop, settle, and return the blobs of `frames` fresh frames (a list of lists)."""
        self.pause()
        out = []
        for _ in range(frames):
            out.append(self.cam.snap(time.time()))
        return out

    def _advance(self, dist):
        self.x += dist * math.cos(self.th)
        self.y += dist * math.sin(self.th)

    def turn_to(self, target, state='turning'):
        """ARC turn until the camera-measured heading is within TURN_TOL_DEG of `target` (rad). Forward and backward
        arcs alternate so the robot stays roughly in place. False if it will not turn (self.dead is then set)."""
        tol, lead = math.radians(TURN_TOL_DEG), math.radians(TURN_LEAD_DEG)
        if abs(wrap(target - self.th)) <= tol:
            return True
        DASH.say(f'Turning {math.degrees(wrap(target - self.th)):+.0f} deg (arcs)')
        fwd, stalls = True, 0
        for _ in range(MAX_ARCS):
            err = wrap(target - self.th)
            if abs(err) <= tol or STOP_EVENT.is_set():
                break
            left = err > 0
            powers = ARCS[left][0 if fwd else 1]
            th0, t0, last = self.th, time.time(), time.time()
            while time.time() - t0 < ARC_MAX_S and not STOP_EVENT.is_set():
                self.robot.drive(powers)
                time.sleep(CYCLE_S)
                now = time.time()
                self._advance(ARC_MPS * (now - last) * (1 if fwd else -1))
                last = now
                e = wrap(target - self.th)
                if abs(e) <= lead or (e > 0) != left:              # there (allowing for lag), or crossed over
                    break
                STATUS.state = f'{state}: {math.degrees(e):+.0f} deg to go'
            self.robot.drive(wb.STOP)
            self.pause(0.25, state)                                 # let the camera catch up before judging
            self.moved()
            if abs(wrap(self.th - th0)) < math.radians(STALL_DEG):
                stalls += 1
                if stalls >= MAX_STALLS:
                    DASH.say('The arcs are not turning the robot - stopping (battery? stuck?)')
                    self.dead = True
                    return False
                DASH.say('That arc did not turn the robot - trying again')
                self.pause(1.0)
            else:
                stalls = 0
            fwd = not fwd
        return abs(wrap(target - self.th)) <= tol * 1.5

    def leg(self, dist, heading, backward=False, label=''):
        """Drive `dist` m with the robot FACING `heading`, forward or in reverse, holding that heading with the camera.
        Forward legs stop for an obstacle (self.obstacle is then set). True if the whole distance was covered."""
        self.obstacle = None
        if not self.turn_to(heading, f'lining up for {label}') and self.dead:     # a few degrees off is fine: the
            return False                                                          # leg holds the heading anyway
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
            fire_ahead = [b for b in self.cam.blobs if b.kind == 'fire' and b.range is not None
                          and b.range < FIRE_STOP_M and abs(b.bearing) < math.radians(20)]
            if fire_ahead and not backward:
                self.pause(0.2)
                self.moved()
                DASH.say(f'FIRE {fire_ahead[0].range:.2f} m straight ahead - stopped the leg')
                return False
            if not backward:
                cm = self.robot.ping()
                if cm is not None and cm < OBSTACLE_CM:
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
                  f'tune WEAK_MOTOR / WEAK_RATIO', flush=True)
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


def survey(nav, views_deg=SURVEY_VIEWS_DEG, frame0=0):
    """Look from each heading in views_deg (map frame), FRAMES_PER_VIEW frames each; return the Sightings."""
    seen, frame = [], frame0
    for i, deg in enumerate(views_deg):
        if not nav.turn_to(math.radians(deg), f'survey: turning to {deg:+.0f} deg') and nav.dead:
            DASH.think('I cannot turn, so I can only map what I have already seen.')
            break
        found = []
        for blobs in nav.look(FRAMES_PER_VIEW):
            frame += 1
            got = sightings_from(blobs, nav.x, nav.y, nav.th, frame, (frame0, i))
            seen += got
            found = got
        kinds = ', '.join(f'{s.kind} {s.r:.1f} m' for s in found) or 'nothing'
        DASH.say(f'Looking at {math.degrees(nav.th):+.0f} deg: {kinds}')
    return seen


def relocalise(nav, fire):
    """At (about) the start: face the fire and measure it over several frames. Its BEARING fixes the heading (yaw
    drifts a little) and its RANGE + bearing fix the position. A correction too big to be drift is ignored."""
    nav.turn_to(math.atan2(fire['y'] - nav.y, fire['x'] - nav.x), 'turning to the fire to re-localise')
    obs = []
    for blobs in nav.look(5):
        near = [b for b in blobs if b.kind == 'fire' and b.range is not None
                and abs(b.bearing) < math.radians(ACQUIRE_GATE_DEG)]
        if near:
            b = max(near, key=lambda b: b.w * b.h)
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
    DASH.think(f'Fire seen {r:.2f} m away, {abs(math.degrees(b)):.0f} deg {"left" if b > 0 else "right"}: I am '
               f'{shift:.2f} m and {abs(math.degrees(wrap(th - nav.th))):.0f} deg off my estimate - corrected.')
    nav.x, nav.y = x, y
    nav.set_heading(th)
    nav.moved()
    return True


def home_on(nav, name, bearing, rng):
    """Visual servo onto the victim the map predicts at (bearing, rng) from here: the first lock goes to the blob that
    best fits BOTH (victims look alike), then it is tracked by bearing until its hub nears the bottom of the image;
    then creep CREEP_M blind. True when there; False if it is lost or the fire/an obstacle is in the way."""
    robot, cam = nav.robot, nav.cam
    robot.aim(SONAR_CENTER_DEG)
    nav.obstacle = None
    track, gate, last_seen, last = bearing, math.radians(ACQUIRE_GATE_DEG), time.time(), time.time()
    locked = False
    while not STOP_EVENT.is_set():
        blobs = cam.snap(time.time() - 0.05, timeout=0.3)
        now = time.time()
        mine = [b for b in blobs if b.kind == 'victim' and b.range is not None and abs(b.bearing - track) < gate]
        if not mine:
            robot.drive(wb.STOP)
            last = now
            if now - last_seen > LOST_S:
                DASH.say(f'Lost sight of victim {name}')
                return False
            continue
        b = (min(mine, key=lambda b: abs(b.bearing - track)) if locked else
             min(mine, key=lambda b: unlikeness(b.bearing, b.range, bearing, rng)))
        locked = True
        track, gate, last_seen = b.bearing, math.radians(TRACK_GATE_DEG), now
        if b.y + b.h / 2 >= H * CLOSE_FRAC:
            DASH.say(f'Victim {name} is right in front - creeping the last {CREEP_M * 100:.0f} cm')
            t_end = time.time() + CREEP_M / CRUISE_MPS
            while time.time() < t_end and not STOP_EVENT.is_set():
                robot.drive(wb.drive_powers(CRUISE, CRUISE))
                time.sleep(CYCLE_S)
            nav._advance(CREEP_M)
            nav.pause(0.3, f'at victim {name}')
            nav.moved()
            return True
        for f in blobs:
            if f.kind == 'fire' and f.range is not None and abs(f.bearing) < math.radians(20) \
                    and f.range < FIRE_STOP_M and f.range < b.range:
                nav.pause(0.3, 'FIRE AHEAD - stopping')
                DASH.say(f'FIRE {f.range:.2f} m straight ahead - stopped')
                return False
        cm = robot.ping()
        if cm is not None and cm < OBSTACLE_CM and cm / 100 < b.range - 0.10:
            nav.pause()
            r = cm / 100 + 0.08
            nav.obstacle = (nav.x + r * math.cos(nav.th), nav.y + r * math.sin(nav.th))
            DASH.say(f'OBSTACLE {cm} cm ahead (victim {name} is {b.range:.2f} m) - stopped')
            return False
        steer = max(-70, min(70, K_STEER * track))                  # + bearing (left) -> right side faster
        robot.drive(wb.drive_powers(CRUISE - steer, CRUISE + steer))
        nav._advance(CRUISE_MPS * (now - last))
        last = now
        nav.moved()
        STATUS.state = f'homing on victim {name}: {b.label()}'
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
            DASH.think(f'{label[0].upper() + label[1:]} is only {math.hypot(goal[0] - h[0], goal[1] - h[1]):.2f} m from '
                       f'{h[3]}, so I must pass within {d:.2f} m of it - approaching slowly, watching with the camera.')
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
    """Reverse back through the waypoints passed on the way out, to the start, without turning round; then re-anchor
    on the fire and fix what is left."""
    DASH.set(path=[[nav.x, nav.y]] + [list(p) for p in reversed(passed)] + [[0.0, 0.0]])
    for p in list(reversed(passed)) + [(0.0, 0.0)]:
        if not nav.go_to(p, 'the start point', reverse_ok=True) and nav.dead:
            return
    for _ in range(2):
        if fire is None or not relocalise(nav, fire):
            break
        err = math.hypot(nav.x, nav.y)
        if err <= HOME_TOL_M:
            DASH.think(f'Back at the start point (within {err * 100:.0f} cm).')
            break
        DASH.think(f'I am {err * 100:.0f} cm from the start point - correcting.')
        nav.go_to((0.0, 0.0), 'the start point', reverse_ok=True)
    nav.turn_to(0.0, 'facing the arena again')


def vantage_point(fire):
    """Where close calls are re-measured from: VANTAGE_FRACTION of the way from the start to the fire; None if the
    fire is already close (the start is as good a viewpoint as any)."""
    if math.hypot(fire['x'], fire['y']) < 1.2:
        return None
    return fire['x'] * VANTAGE_FRACTION, fire['y'] * VANTAGE_FRACTION


def recheck_views(pos, fire, who):
    """Headings (deg) to look along from pos: for each contested victim, between it and the fire if both fit in one
    frame (so the distance is a same-photo one), else at the victim."""
    views = []
    for v in who:
        a_v = math.atan2(v['y'] - pos[1], v['x'] - pos[0])
        a_f = math.atan2(fire['y'] - pos[1], fire['x'] - pos[0])
        gap = wrap(a_f - a_v)
        views.append(math.degrees(a_v + gap / 2) if abs(gap) < math.radians(HFOV_DEG - 12) else math.degrees(a_v))
    return views


def remeasure(who, fire, new):
    """Update each contested victim from the sightings `new` taken nearer the fire: its fire distance and its map
    position (both now more precise). New sightings are matched to a victim by BEARING FROM THE START, which the first
    survey measured precisely and its range within its (larger) error - see unlikeness(). Returns
    [(victim, old_d)]."""
    fire2, _ = build_map(new + list(fire['members']))
    near = cluster(new, 'victim', MERGE_M)
    pairs = sorted((unlikeness(math.atan2(c['y'], c['x']), math.hypot(c['x'], c['y']),
                               math.atan2(v['y'], v['x']), math.hypot(v['x'], v['y'])), i, j)
                   for i, v in enumerate(who) for j, c in enumerate(near))
    used_v, used_c, done = set(), set(), []
    for u, i, j in pairs:                                           # one-to-one, most plausible match first
        if u > 9 or i in used_v or j in used_c or fire2 is None:    # 9 = 3 standard deviations
            continue
        v, c = who[i], near[j]
        old = v['d']
        v['d'], v['sigma'], v['how'] = fire_distance(c, fire2)
        v['x'], v['y'] = c['x'], c['y']
        used_v.add(i)
        used_c.add(j)
        done.append((v, old))
    return done


def recheck_close_calls(nav, fire, order, close, names, frame0):
    """Victims whose fire distances are too close to call get measured again from nearer the fire, where a pixel is
    worth far less (range error ~ range^2). Returns the re-ranked order."""
    contested = {id(order[i]) for i in close} | {id(order[i + 1]) for i in close}
    who = [v for v in order if id(v) in contested]
    vantage = vantage_point(fire)
    if vantage is None:
        DASH.think('I am already close to the fire - no nearer viewpoint to re-measure from.')
        return order
    DASH.think('Victims ' + ', '.join(names[id(v)] for v in who) + ' are too close to call from here (their fire '
               f'distances differ by less than my measurement error). I will look again from halfway to the fire, '
               f'({vantage[0]:+.2f}, {vantage[1]:+.2f}): at half the range a pixel of error costs a quarter of the '
               f'depth.')
    hazards = [(fire['x'], fire['y'], FIRE_CLEAR_M)] + [(v['x'], v['y'], VICTIM_CLEAR_M) for v in order]
    passed = []
    for p in plan((nav.x, nav.y), vantage, hazards):
        if not nav.go_to(p, 'the viewpoint'):
            break
        passed.append((nav.x, nav.y))
    new = survey(nav, recheck_views((nav.x, nav.y), fire, who), frame0)
    for v, old in remeasure(who, fire, new):
        DASH.think(f'Victim {names[id(v)]}: re-measured {v["d"]:.2f} m from the fire (+-{v["sigma"]:.2f}), '
                   f'was {old:.2f} m.')
    come_back(nav, passed, fire)
    return sorted(order, key=lambda v: v['d'])


def mission(nav, drive=True):
    DASH.say('Surveying the arena from the start point')
    DASH.think(f'How I judge depth: my camera sits {CAM_HEIGHT_M * 100:.0f} cm above a flat floor, so the further '
               f'below the horizon something touches the floor, the closer it is. I place every sighting on the floor '
               f'that way and measure each victim-to-fire distance inside the same photo, so my own position errors '
               f'cancel.')
    sightings = survey(nav)
    if drive and nav.dead:
        DASH.say('The motors cannot turn the robot - not driving. Check the shield battery and its switch.')
        drive = False
    fire, victims = build_map(sightings)
    victims.sort(key=lambda v: -math.atan2(v['y'], v['x']))         # names A, B, C... from left to right
    names = {id(v): chr(ord('A') + i) for i, v in enumerate(victims)}
    if not victims:
        DASH.think('No victims in sight - nothing to rescue.')
        return
    if fire is None:
        DASH.think('I cannot see the fire, so I cannot rank by danger - I will go nearest first.')
    order, close = rank(victims, fire)

    def publish():
        objs = [{'name': names[id(v)], 'kind': 'victim', 'x': v['x'], 'y': v['y'], 'rank': order.index(v) + 1,
                 'reached': v.get('reached', False)} for v in victims]
        if fire:
            objs.insert(0, {'name': 'FIRE', 'kind': 'fire', 'x': fire['x'], 'y': fire['y'], 'rank': None,
                            'reached': False})
        DASH.set(objs=objs)

    publish()
    if fire:
        DASH.set(hazards=[[fire['x'], fire['y'], FIRE_CLEAR_M, 'the fire']])
        DASH.think(f'Survey done: the fire is {math.hypot(fire["x"], fire["y"]):.2f} m away and I see '
                   f'{len(victims)} victim{"s" * (len(victims) != 1)}.')
        for v in order:
            DASH.think(f'Victim {names[id(v)]}: {v["d"]:.2f} m from the fire (+-{v["sigma"]:.2f} m, measured '
                       f'{"in the same photo as the fire" if v["how"] == "same-photo" else "from my map"}).')
        if close and drive:
            order = recheck_close_calls(nav, fire, order, close, names, frame0=10_000)
            publish()
        elif close:
            DASH.think('Some fire distances are within my measurement error - the full mission re-measures them.')
        DASH.think('Rescue order, closest to the fire first: ' + ' -> '.join(
            f'{names[id(v)]} ({v["d"]:.2f} m)' for v in order) + f'. {names[id(order[0])]} is in the most danger.')
    if not drive:
        return
    reached = 0
    for i, v in enumerate(order, 1):
        if STOP_EVENT.is_set():
            return
        if nav.dead:
            DASH.say('STOPPED: the motors cannot turn the robot any more.')
            return
        name = names[id(v)]
        DASH.set(target=name)
        DASH.say(f'Trip {i}: moving towards victim {name}')
        hazards = ([(fire['x'], fire['y'], FIRE_CLEAR_M, 'the fire')] if fire else []) + \
                  [(o['x'], o['y'], VICTIM_CLEAR_M, f'victim {names[id(o)]}') for o in victims if o is not v]
        DASH.set(hazards=[list(h) for h in hazards])
        passed = go_out(nav, (v['x'], v['y']), hazards, f'victim {name}')
        ok = False
        if passed is not None:
            nav.turn_to(math.atan2(v['y'] - nav.y, v['x'] - nav.x), f'facing victim {name}')
            ok = home_on(nav, name, wrap(math.atan2(v['y'] - nav.y, v['x'] - nav.x) - nav.th),
                         math.hypot(v['x'] - nav.x, v['y'] - nav.y))
        if ok:
            reached += 1
            v['reached'] = True
            publish()
            DASH.say(f'REACHED victim {name}')
            # we are just short of it, facing it: pin the position there (homing steers, so it drifted)
            back = reach_distance() - CREEP_M
            nav.x, nav.y = v['x'] - back * math.cos(nav.th), v['y'] - back * math.sin(nav.th)
            nav.moved()
        else:
            DASH.say(f'Could not reach victim {name}')
            DASH.think(f'Victim {name} is out of reach for now - back to the start, then the next one.')
        DASH.say('Returning to the start point (in reverse, along the same route)')
        come_back(nav, passed or [], fire)
    DASH.set(target=None, path=[])
    DASH.say(f'MISSION COMPLETE: {reached} of {len(order)} victims reached, back at the start point')
    DASH.think(f'All done: {reached} of {len(order)} victims reached, closest to the fire first.')


# ---- calibration and tests ----------------------------------------------------------------------------------------
def calibrate_horizon(cam, dist):
    """One wheel at a tape-measured `dist` metres straight ahead (lens to the wheel's centre, along the floor): its hub
    row then fixes the horizon row, i.e. the camera's tilt - the error that matters most for depth (a 2 px tilt error
    moves a wheel at 2.5 m by ~15%). Writes calibration.json, which every later run loads."""
    print(f'Put ONE wheel {dist:.2f} m straight in front of the camera (floor distance from the lens to the middle of '
          f'the wheel), nothing else yellow in view. Measuring in 3 s...', flush=True)
    if STOP_EVENT.wait(3):
        return
    rows = []
    t_end = time.time() + 6
    while time.time() < t_end and not STOP_EVENT.is_set():
        hubs = [b for b in cam.snap(time.time()) if b.kind == 'victim']
        if hubs:
            b = min(hubs, key=lambda b: abs(b.bearing))
            rows.append(b.y + b.h / 2)
    if len(rows) < 10:
        print(f'Saw the wheel in only {len(rows)} frames - is it in view and lit? Nothing saved.')
        return
    row = statistics.median(rows)
    horizon = row - (CAM_HEIGHT_M - Z_M['victim']) * F_PX / dist
    print(f'hub centre at row {row:.1f} (spread {max(rows) - min(rows):.1f} px over {len(rows)} frames) -> horizon '
          f'row {horizon:.1f} (was {HORIZON_Y})')
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


def simulate_survey(fire_xy, victims_xy, rng, px_noise=PX_NOISE, horizon_err=0.0, heading_err=0.0, pos=(0, 0),
                    views=SURVEY_VIEWS_DEG, frame0=0):
    """Synthetic sightings: what the survey would record from `pos` if the true layout were this, with Gaussian row
    noise per frame, a common horizon error (camera tilt) and a heading error (bad yaw). Mirrors detect()+survey()."""
    out, frame = [], frame0
    objs = [('fire', fire_xy)] + [('victim', v) for v in victims_xy]
    half = {'fire': 0.15, 'victim': 0.035}                          # half-widths: the box, a wheel

    def hidden(kind, o):
        r = math.hypot(o[0] - pos[0], o[1] - pos[1])
        a = math.atan2(o[1] - pos[1], o[0] - pos[0])
        # hidden only if a NEARER object covers this one's whole width as seen from pos (a wheel cannot hide the box)
        return any(math.hypot(q[0] - pos[0], q[1] - pos[1]) < r - 0.05
                   and abs(wrap(math.atan2(q[1] - pos[1], q[0] - pos[0]) - a)) + half[kind] / r
                   < half[k] / math.hypot(q[0] - pos[0], q[1] - pos[1])
                   for k, q in objs if q is not o)

    objs = [(k, o) for k, o in objs if not hidden(k, o)]
    for i, deg in enumerate(views):
        th = math.radians(deg)
        for _ in range(FRAMES_PER_VIEW):
            frame += 1
            for kind, (ox, oy) in objs:
                r = math.hypot(ox - pos[0], oy - pos[1])
                bearing = wrap(math.atan2(oy - pos[1], ox - pos[0]) - th)
                if abs(bearing) > math.radians(HFOV_DEG / 2 - 2):
                    continue
                row = HORIZON_Y + horizon_err + (CAM_HEIGHT_M - Z_M[kind]) * F_PX / r + rng.gauss(0, px_noise)
                rr = floor_range(kind, row)
                if rr is None:
                    continue
                b = bearing + rng.gauss(0, 1 / F_PX)
                a = th + heading_err * i + b                        # heading error grows view by view
                out.append(Sighting(kind, pos[0] + rr * math.cos(a), pos[1] + rr * math.sin(a), rr, frame,
                                    (frame0, i), 400 if kind == 'fire' else 120, pos[0], pos[1]))
    return out


def random_layout(rng):
    """A layout like the real arena: the fire 1.8-2.6 m ahead, three wheels between, >= 0.3 m apart, >= 0.2 m from
    the fire."""
    fire = (rng.uniform(1.8, 2.6), rng.uniform(-0.6, 0.6))
    vs = []
    while len(vs) < 3:
        p = (rng.uniform(0.6, 2.8), rng.uniform(-1.2, 1.2))
        if math.hypot(p[0] - fire[0], p[1] - fire[1]) > 0.2 and all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.3
                                                                     for q in vs):
            vs.append(p)
    return fire, vs


def robustness(trials=2000, seed=7, quiet=False, tilt_px=2.0):
    """Monte-Carlo over random arenas: how often is the rescue order right, versus the smallest gap between the
    victims' TRUE fire distances? Noise: PX_NOISE px per frame, a 2 px camera-tilt (horizon) error shared by every
    picture, 1 deg/view heading drift - and close calls re-measured from nearer the fire, as the robot does."""
    rng = random.Random(seed)
    bins = [(0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 9)]
    stats = {b: [0, 0, 0, 0] for b in bins}                         # arenas, right first look, re-checked, right final
    hidden = 0
    for _ in range(trials):
        fire, vs = random_layout(rng)
        tilt, drift = rng.gauss(0, tilt_px), math.radians(rng.gauss(0, 1))
        clean = simulate_survey(fire, vs, random.Random(0), px_noise=0)
        if len(build_map(clean)[1]) != 3:                       # a wheel hidden behind the box or another wheel
            hidden += 1
            continue
        f, victims = build_map(simulate_survey(fire, vs, rng, horizon_err=tilt, heading_err=drift))
        true_d = sorted(math.hypot(v[0] - fire[0], v[1] - fire[1]) for v in vs)
        gap = min(b - a for a, b in zip(true_d, true_d[1:]))
        b = next(b for b in bins if b[0] <= gap < b[1])
        stats[b][0] += 1
        if f is None or len(victims) != 3:
            continue                                                # counted as wrong

        def right(order):
            # which real wheel is each estimate? the one at the nearest BEARING from the start - that is the wheel
            # the robot would home in on
            est = [min(vs, key=lambda q: unlikeness(math.atan2(q[1], q[0]), math.hypot(*q),
                                                    math.atan2(v['y'], v['x']), math.hypot(v['x'], v['y'])))
                   for v in order]
            return [math.hypot(q[0] - fire[0], q[1] - fire[1]) for q in est] == true_d

        order, close = rank(victims, f)
        stats[b][1] += right(order)
        vp = vantage_point(f)
        if close and vp:
            who = [v for i, v in enumerate(order) if i in close or i - 1 in close]
            new = simulate_survey(fire, vs, rng, horizon_err=tilt, heading_err=drift, pos=vp,
                                  views=recheck_views(vp, f, who), frame0=10_000)
            remeasure(who, f, new)
            order = sorted(order, key=lambda v: v['d'])
            stats[b][2] += 1
        stats[b][3] += right(order)
    if not quiet:
        print("Rescue order right, by the smallest gap between the victims' TRUE distances to the fire:")
        for (lo, hi), (n, first, rc, final) in stats.items():
            if n:
                label = f'>= {lo * 100:.0f} cm' if hi > 1 else f'{lo * 100:2.0f}-{hi * 100:2.0f} cm'
                print(f'  gap {label:9s}: {100 * final / n:5.1f}% right  (first look alone {100 * first / n:5.1f}%; '
                      f'{100 * rc / n:3.0f}% of arenas re-checked from nearer; {n} arenas)')
        print(f'  ({hidden} of {trials} random arenas left out: a wheel stood hidden behind the box or another wheel '
              f'as seen from the start, so no survey from there can find it)')
    return stats


def selftest():
    # closest-to-fire first, even though another victim is nearer the robot
    rng = random.Random(1)
    fire, vs = (2.2, 0.3), [(0.8, -0.6), (1.9, 0.6), (1.4, 0.0)]
    f, victims = build_map(simulate_survey(fire, vs, rng, px_noise=0.3))
    order, close = rank(victims, f)
    near = [min(vs, key=lambda q: math.hypot(q[0] - v['x'], q[1] - v['y'])) for v in order]
    assert near == [vs[1], vs[2], vs[0]], near
    assert abs(f['x'] - fire[0]) < 0.1 and abs(f['y'] - fire[1]) < 0.1, f
    # same-photo distances are immune to the robot's heading error; map distances are not
    s = simulate_survey(fire, vs, random.Random(2), px_noise=0, heading_err=math.radians(8))
    f2, v2 = build_map(s)
    same = 0
    for v in v2:
        d, _, how = fire_distance(v, f2)
        q = min(vs, key=lambda q: abs(math.hypot(*q) - math.hypot(v['x'], v['y'])))   # range: immune to heading
        if how == 'same-photo':
            same += 1
            assert abs(d - math.hypot(q[0] - fire[0], q[1] - fire[1])) < 0.03, (d, q)
    assert same >= 2, same
    # a fire on the straight line forces a detour that keeps its distance
    path = plan((0, 0), (2, 0), [(1, 0.1, FIRE_CLEAR_M)])
    assert len(path) == 2 and path[-1] == (2, 0)
    for a, b in zip([(0, 0)] + path, path):
        assert seg_dist(a, b, (1, 0.1)) > FIRE_CLEAR_M * 0.9, (a, b)
    assert path[0][1] < 0                                           # passes on the side away from the fire
    assert plan((0, 0), (2, 0), [(1, 2, FIRE_CLEAR_M)]) == [(2, 0)]           # fire well off the line: straight
    assert plan((0, 0), (2, 0), [(2.2, 0, FIRE_CLEAR_M)]) == [(2, 0)]         # victim beside the fire: reachable
    hz = [(1, 0.1, FIRE_CLEAR_M), (1.6, -0.5, OBSTACLE_CLEAR_M)]               # fire + a sonar obstacle
    path = plan((0, 0), (2.5, 0), hz)
    for a, b in zip([(0, 0)] + path, path):
        for hx, hy, c in hz:
            assert seg_dist(a, b, (hx, hy)) > c * 0.6, (a, b, hx, hy)
    # perception geometry
    assert abs(Blob('victim', W // 2 - 5, 400, 10, 20).bearing) < 1e-9
    assert Blob('victim', 10, 400, 10, 20).bearing > 0               # left half of the image = + bearing
    assert floor_range('fire', HORIZON_Y + 30) > floor_range('fire', HORIZON_Y + 60)   # lower in the image = nearer
    assert floor_range('fire', HORIZON_Y) is None
    # turning LEFT slides the scene RIGHT: rotation() must call that +
    img = np.float32(cv2.GaussianBlur(np.random.default_rng(1).random((230, 480)), (0, 0), 3))
    a, conf = rotation(img, np.roll(img, 40, axis=1))
    assert math.degrees(a) > 3 and conf > VO_MIN_CONF
    assert math.degrees(rotation(img, np.roll(img, -40, axis=1))[0]) < -3
    # arcs: both arcs of a turn turn the SAME way (left: right side faster forwards, and faster backwards too)
    for left, (fwd, back) in ARCS.items():
        lf, rf = fwd[0] * wb.FORWARD_SIGN[0], fwd[3] * wb.FORWARD_SIGN[3]      # side powers (M1 left, M4 right),
        lb, rb = back[0] * wb.FORWARD_SIGN[0], back[3] * wb.FORWARD_SIGN[3]    # + = that side drives forward
        assert (rf - lf > 0) == left and (rb - lb > 0) == left and lf > 0 and lb < 0, (left, fwd, back)
    assert 0.10 < reach_distance() < 0.25
    # ranking quality: with this camera's noise, wide gaps must essentially always come out right
    st = robustness(400, quiet=True)
    n, _, _, ok = st[(0.3, 9)]
    assert n and ok / n > 0.95, (n, ok)                              # measured 98.8% over 1500 arenas
    json.loads(DASH.json())
    print('selftest: OK')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag in ('--view', '--survey', '--sonar-cal', '--selftest', '--robustness'):
        ap.add_argument(flag, action='store_true')
    ap.add_argument('--calibrate-horizon', type=float, metavar='METRES')
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    if args.robustness:
        robustness()
        return
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP_EVENT.set())
    cam = Camera()
    robot = None
    try:
        if not args.sonar_cal:
            cam.start()
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
        robot.look(SONAR_CENTER_DEG)                                # the Uno resets on every open: servo to centre
        if args.sonar_cal:
            sonar_cal(robot)
            return
        for n in range(wb.COUNTDOWN_S, 0, -1):
            DASH.set(state=f'starting in {n} s')
            print(f'Moving in {n}... (Ctrl+C to cancel)', flush=True)
            if STOP_EVENT.wait(1):
                return
        print(f'camera at {cam.fps:.1f} fps', flush=True)
        mission(Nav(robot, cam), drive=not args.survey)
        DASH.set(state='finished - Ctrl+C to exit (the dashboard stays up until then)')
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
