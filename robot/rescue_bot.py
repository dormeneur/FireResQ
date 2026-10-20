#!/usr/bin/env python3
"""FIRERESQ RESCUE BOT - run ON THE PI, from the same folder as wander_bot.py, wander_fw.hex and dashboard.html.

The arena: a FIRE (red box) and VICTIMS (yellow wheel hubs) on the floor. From the start point the robot looks
around and builds a map of them, then visits the victims in order of how close they are to the fire - closest first -
on routes that keep clear of the fire, coming back to the start after each one, and ends at the start.

    python3 -u rescue_bot.py --view         # camera + detection only, motors untouched
    python3 -u rescue_bot.py --survey       # look around, map, decide the order - no driving
    python3 -u rescue_bot.py --sonar-cal    # flat object 30 cm straight ahead: checks SONAR_CENTER_DEG (no motors)
    python3 -u rescue_bot.py --selftest     # the maths, no hardware
    python3 -u rescue_bot.py                # THE MISSION (5 s countdown)

DASHBOARD: http://pi.local:8090 - live camera, the map, what the robot is doing, and why.

How it moves (measured on this robot, 2026-10-20): a skid spin (one side forward, the other backward) does NOT turn it;
a PIVOT (one side driving, the other stopped) does, ~12 deg per 0.6 s pulse, fading after 2-3 pulses as the AA pack
sags and coming back after a rest. So turns are CLOSED-LOOP on the camera: pulse, stop, measure how far the picture
slid sideways (phase correlation), repeat until the angle is reached. Straight legs are timed (CRUISE_MPS), and the
position is re-measured against the fire after every trip, so the timing error does not pile up.
Map frame = the start pose: x forward, y left, angles CCW (+ = left).
"""
import argparse
import json
import math
import signal
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
CAMERA_FPS = 10
WEB_PORT = 8090
DASHBOARD_HTML = Path(__file__).resolve().parent / 'dashboard.html'
VO_ROWS = (150, 450)                     # the image band used to measure turns (sky and floor carry little texture)

# ---- what to look for: HSV (OpenCV: H 0-179), sampled from this camera in the room's light on 2026-10-20 ----------
COLOURS = {
    'fire':   [((0, 150, 60), (8, 255, 255)), ((170, 150, 60), (179, 255, 255))],     # red shoe box
    'victim': [((20, 150, 70), (33, 255, 255))],     # yellow wheel hubs (cardboard is duller: S 140-170, V 50-77)
}
ROUND = {'victim': (0.5, 2.0)}           # allowed width/height of a blob: a hub is round, a stripe of cardboard is not
MIN_AREA = {'fire': 300, 'victim': 60}   # px of the upright image; smaller blobs are noise
# Range comes from where the object sits on the FLOOR (a tiny hub's pixel height is far too noisy): the ray through
# a point Z_M above the floor - the fire's base, the hub's centre - meets that height at this range.
Z_M = {'fire': 0.0, 'victim': 0.0325}    # wheel radius: the hub centre's height
HORIZON_Y = 318                          # image row of the horizon. Measured: the box's base agrees with its size here
BOX_COLOUR = {'fire': (0, 0, 255), 'victim': (0, 255, 255)}

# ---- motion -----------------------------------------------------------------------------------------------------
# The TT motors are 3-6 V parts and the L293D drops ~2 V, so the PWM is capped to keep the motors at <= ~7.5 V on
# average whatever the pack is: 4xAA (6 V) gets full power, 8xAA in series (12 V) is capped to 75%.
MOTOR_SUPPLY_V = 6.0                     # the pack on the shield's EXT_PWR: 4xAA or 8xAA in PARALLEL = 6.0, 8xAA in series = 12.0
POWER_SCALE = min(1.0, 7.5 / (MOTOR_SUPPLY_V - 2.0))
wb.MAX_POWER = round(255 * POWER_SCALE)  # nothing is ever commanded above this
PIVOT_POWER = 190                        # a fresh pack at 255 spun the robot ~60+ deg per pulse: too far to measure
MIN_CONF = 0.12                          # rotation() confidence below this = the pictures did not overlap
# Each turn direction has two pivots, alternated: one side forward, or the OTHER side backward. Alternating shares
# the load between the motors, and the forward/backward creep of the two cancels out.
PIVOTS = {True: (wb.drive_powers(0, PIVOT_POWER), wb.drive_powers(-PIVOT_POWER, 0)),      # turn LEFT
          False: (wb.drive_powers(PIVOT_POWER, 0), wb.drive_powers(0, -PIVOT_POWER))}     # turn RIGHT
PULSE_S = (0.08, 0.8)                    # (min, max) pivot pulse
FIRST_PULSE_S = 0.12                     # the first pulse of a turn is short whatever the pack: the rate is unknown
STEP_DEG = 12                            # aim for at most this per pulse: the camera must still overlap to measure it
START_RATE_DPS = 60.0                    # first guess at the pivot rate; learned while turning
TURN_TOL_DEG = 4
STALL_DEG = 1.0                          # a pulse that turned less than this stalled (battery sagging)
STALL_REST_S = 4.0                       # rest after a stall: the pack recovers in a few seconds
MAX_STALLS = 5
CRUISE = round(210 * POWER_SCALE)        # forward power for legs and homing
CRUISE_MPS = 0.25                        # TODO(calibrate): forward speed at CRUISE. Re-localising hides most error
SETTLE_S = 0.35                          # stop this long before trusting a frame

# ---- sonar (servo-mounted HC-SR04) ------------------------------------------------------------------------------
SONAR_CENTER_DEG = 128                   # servo angle that points STRAIGHT AHEAD, found by hand (higher = left)
OBSTACLE_CM = 22                         # something this close ahead (and closer than the target) = stop

# ---- mission ----------------------------------------------------------------------------------------------------
SURVEY_VIEWS_DEG = (0, 35, 70, -35, -70)  # headings to look from at the start (FOV 41: neighbours overlap)
MERGE_M = 0.30                           # sightings of one kind closer than this are the same object
FIRE_CLEAR_M = 0.40                      # routes keep this far from the fire...
OBSTACLE_CLEAR_M = 0.25                  # ...from an obstacle the sonar found...
VICTIM_CLEAR_M = 0.18                    # ...and from the victims that are not the current target
MAX_REROUTES = 3
FIRE_STOP_M = 0.25                       # fire straight ahead and this close while homing: stop
K_STEER = 150                            # motor power per radian of bearing error while homing
TRACK_GATE_DEG = 12                      # victims look alike: a tracked blob must stay this close to where it was
ACQUIRE_GATE_DEG = 25                    # ...and this close to the map's prediction when homing starts
LOST_S = 2.0
CLOSE_FRAC = 0.95                        # victim's base this low in the image = reached
HOME_TOL_M = 0.15                        # back at the start when re-localised within this


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


# ---- perception -------------------------------------------------------------------------------------------------
class Blob:
    """One coloured blob in the upright image, with its bearing and (if it can be measured) its range."""

    def __init__(self, kind, x, y, w, h):
        self.kind, self.x, self.y, self.w, self.h = kind, x, y, w, h
        self.bottom = y + h
        self.bearing = math.atan2(W / 2 - (x + w / 2), F_PX)       # rad, + = left of the camera axis
        # floor range; None when the reference point is at/above the horizon (not on the floor, or too far to tell)
        # or, for the fire, when the frame cuts off its base
        ref = y + h / 2 if kind == 'victim' else self.bottom
        drop = ref - HORIZON_Y
        self.range = None if drop < 4 or (kind == 'fire' and self.bottom >= H - 2) \
            else (CAM_HEIGHT_M - Z_M[kind]) * F_PX / drop

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
    """(rad, confidence): how far the camera turned between two grey bands (+ = left; the scene slides right when it
    turns left), and how much to trust it. Confidence is ~0.8 for an unmoved scene and ~0.2-0.4 for a good match after
    a turn; below MIN_CONF the pictures did not overlap (it turned too far) and the angle is meaningless."""
    (dx, _), conf = cv2.phaseCorrelate(prev, cur)
    return math.atan2(dx, F_PX), conf


class Camera:
    """Grabs, rotates and detects continuously, and serves the dashboard: / (page), /stream (MJPEG), /state (JSON).
    snap() hands out the first frame captured after a given time, so nothing acts on a picture taken before the
    robot stopped."""

    def __init__(self):
        self.cond = threading.Condition()
        self.t, self.blobs, self.gray, self.jpeg = 0.0, [], None, None

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
        while not STOP_EVENT.is_set():
            t = time.time()                                         # stamp BEFORE the capture: conservative
            try:
                frame = cv2.rotate(cam.capture_array(wait=3.0), ROTATE)
            except Exception as e:                                  # a frontend timeout (power dip): carry on
                print(f'camera: {e}', flush=True)
                continue
            blobs = detect(frame)
            gray = np.float32(cv2.cvtColor(frame[VO_ROWS[0]:VO_ROWS[1]], cv2.COLOR_BGR2GRAY))
            for b in blobs:
                cv2.rectangle(frame, (b.x, b.y), (b.x + b.w, b.bottom), BOX_COLOUR[b.kind], 2)
                cv2.putText(frame, b.label(), (b.x, max(12, b.y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            BOX_COLOUR[b.kind], 1, cv2.LINE_AA)
            h, s, v = np.median(cv2.cvtColor(frame[H // 2 - 4:H // 2 + 5, W // 2 - 4:W // 2 + 5],
                                             cv2.COLOR_BGR2HSV).reshape(-1, 3), axis=0).astype(int)
            cv2.drawMarker(frame, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
            cv2.putText(frame, f'HSV {h} {s} {v}', (W // 2 + 10, H // 2 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self.cond:
                self.t, self.blobs, self.gray = t, blobs, gray
                if ok:
                    self.jpeg = jpg.tobytes()
                self.cond.notify_all()
            time.sleep(max(0.0, 1.0 / CAMERA_FPS - (time.time() - t)))
        cam.stop()

    def snap(self, after, timeout=3.0):
        """(blobs, grey band) of the first frame captured at or after `after`; ([], None) if none arrives in time."""
        with self.cond:
            self.cond.wait_for(lambda: self.t >= after, timeout)
            return (list(self.blobs), self.gray) if self.t >= after else ([], None)

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


# ---- deciding and planning (pure: --selftest covers these) ------------------------------------------------------
def cluster(sightings):
    """[(kind, x, y)] from every view of a survey -> [[kind, x, y, n]]: one entry per object, at its mean position."""
    objs = []
    for kind, x, y in sightings:
        near = [o for o in objs if o[0] == kind and math.hypot(o[1] - x, o[2] - y) < MERGE_M]
        if near:
            o = near[0]
            o[1], o[2], o[3] = (o[1] * o[3] + x) / (o[3] + 1), (o[2] * o[3] + y) / (o[3] + 1), o[3] + 1
        else:
            objs.append([kind, x, y, 1])
    return objs


def fire_distance(v, fires):
    return min((math.hypot(v[1] - f[1], v[2] - f[2]) for f in fires), default=float('inf'))


def rescue_order(victims, fires):
    """Victims sorted closest-to-the-fire first (no fire: closest to the start first)."""
    if not fires:
        return sorted(victims, key=lambda v: math.hypot(v[1], v[2]))
    return sorted(victims, key=lambda v: fire_distance(v, fires))


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
    """The robot's pose in the map (the start pose's frame), camera-measured turns, timed straight legs.
    `obstacle` is where the sonar last stopped a leg (None if the last leg was not stopped by one)."""

    def __init__(self, robot, cam):
        self.robot, self.cam, self.status = robot, cam, Status()
        self.x = self.y = self.th = 0.0
        self.rate = START_RATE_DPS
        self.obstacle = None
        self.dead = False                                           # the motors gave up: the mission must stop

    def moved(self):
        DASH.moved(self.x, self.y, self.th)

    def pause(self, s=SETTLE_S, state='stopped'):
        hold(self.robot, self.status, wb.STOP, s, state)

    def look(self):
        self.pause()
        return self.cam.snap(time.time())

    def turn(self, ang, state='turning'):
        """Turn by `ang` rad (+ = left), measured by the camera. False if the motors stalled for good."""
        if abs(ang) > math.radians(TURN_TOL_DEG):
            DASH.say(f'Turning {abs(math.degrees(ang)):.0f} deg {"left" if ang > 0 else "right"}')
        done, stalls, k, max_t = 0.0, 0, 0, FIRST_PULSE_S
        prev = self.look()[1]
        while abs(ang - done) > math.radians(TURN_TOL_DEG) and not STOP_EVENT.is_set():
            rem = math.degrees(ang - done)
            t = max(PULSE_S[0], min(max_t, PULSE_S[1], min(abs(rem), STEP_DEG) / self.rate))
            hold(self.robot, self.status, PIVOTS[rem > 0][k % 2], t, f'{state} ({rem:+.0f} deg to go)')
            k += 1
            cur = self.look()[1]
            if prev is None or cur is None:
                DASH.say('No camera frame - cannot measure the turn, stopping it')
                break
            d, conf = rotation(prev, cur)
            prev = cur
            if conf < MIN_CONF:
                # it turned too far for the pictures to overlap: shorten the pulses and assume what we expected
                max_t = max(PULSE_S[0], t / 2)
                d = math.copysign(math.radians(self.rate * t), rem)
                DASH.say(f'Turn overshot the camera view - using shorter pulses ({max_t:.2f} s)')
            else:
                max_t = min(PULSE_S[1], max_t * 1.25) if abs(math.degrees(d)) < STEP_DEG * 1.3 else max_t
            done += d
            self.th = wrap(self.th + d)
            self.moved()
            if conf >= MIN_CONF and abs(math.degrees(d)) < STALL_DEG:
                stalls += 1
                if stalls >= MAX_STALLS:
                    DASH.say('Motors stalled for good - battery flat?')
                    self.dead = True
                    break
                max_t = min(PULSE_S[1], max(t, max_t) * 1.6)        # too short to break the wheels loose: push longer
                DASH.say(f'Pivot did not move the robot - trying a longer pulse ({max_t:.2f} s)')
                self.pause(STALL_REST_S, f'{state}: no movement, trying a longer pulse')
            elif conf >= MIN_CONF:
                stalls = 0
                self.rate = 0.5 * self.rate + 0.5 * abs(math.degrees(d)) / t
        return abs(ang - done) <= math.radians(TURN_TOL_DEG)

    def face(self, p, state='turning'):
        return self.turn(wrap(math.atan2(p[1] - self.y, p[0] - self.x) - self.th), state)

    def forward(self, dist, state='driving'):
        """Drive `dist` m straight (timed). Stops early for an obstacle; the pose gets the distance actually covered."""
        self.robot.aim(SONAR_CENTER_DEG)
        self.obstacle = None
        t0 = time.time()
        end = t0 + dist / CRUISE_MPS
        cm = None
        while time.time() < end and not STOP_EVENT.is_set():
            self.robot.drive(wb.drive_powers(CRUISE, CRUISE))
            self.status.state = state
            cm = self.robot.ping()
            if cm is not None and cm < OBSTACLE_CM:
                break
            time.sleep(0.03)
        moved = (min(time.time(), end) - t0) * CRUISE_MPS
        self.pause(0.2)
        self.x += moved * math.cos(self.th)
        self.y += moved * math.sin(self.th)
        self.moved()
        if cm is not None and cm < OBSTACLE_CM:
            r = cm / 100 + 0.08                                     # the sonar sits ~8 cm behind the robot's nose
            self.obstacle = (self.x + r * math.cos(self.th), self.y + r * math.sin(self.th))
            DASH.say(f'OBSTACLE {cm} cm ahead - stopped')
        return moved >= dist - 0.05

    def go_to(self, p, label):
        d = math.hypot(p[0] - self.x, p[1] - self.y)
        if not self.face(p, f'turning towards {label}'):
            return False
        DASH.say(f'Driving {d:.2f} m towards {label}')
        return self.forward(d, f'driving to {label}')


def survey(nav):
    """Look from each of SURVEY_VIEWS_DEG at the start; return the clustered map [[kind, x, y, n]]."""
    seen = []
    for deg in SURVEY_VIEWS_DEG:
        if not nav.turn(wrap(math.radians(deg) - nav.th), f'survey: turning to {deg:+d} deg'):
            DASH.think('I cannot turn, so I can only map what I have already seen. Check the motor battery.')
            break
        found = [b for b in nav.look()[0] if b.range is not None]
        for b in found:
            a = nav.th + b.bearing
            seen.append((b.kind, nav.x + b.range * math.cos(a), nav.y + b.range * math.sin(a)))
        kinds = ', '.join(f'{b.kind} {b.range:.1f} m' for b in found) or 'nothing'
        DASH.say(f'Looking at {math.degrees(nav.th):+.0f} deg: {kinds}')
    return cluster(seen)


def relocalise(nav, fire, at=None):
    """Face the fire and measure it to correct the pose. With `at` (we have just reached that map point, so the
    position is known but the heading is not) the fire's BEARING fixes the heading; otherwise the heading is trusted
    and the fire's bearing + RANGE fix the position."""
    nav.face(fire[1:3], 'turning to the fire to re-localise')
    fires = [b for b in nav.look()[0] if b.kind == 'fire' and abs(b.bearing) < math.radians(ACQUIRE_GATE_DEG)]
    if not fires:
        DASH.think('I cannot see the fire from here, so I have to trust my dead reckoning.')
        return False
    b = max(fires, key=lambda b: b.w * b.h)
    old = (nav.x, nav.y, nav.th)
    if at is not None:
        nav.x, nav.y = at
        nav.th = wrap(math.atan2(fire[2] - nav.y, fire[1] - nav.x) - b.bearing)
    elif b.range is not None:
        a = nav.th + b.bearing
        nav.x, nav.y = fire[1] - b.range * math.cos(a), fire[2] - b.range * math.sin(a)
    nav.moved()
    rng = '' if b.range is None else f'{b.range:.2f} m away, '
    DASH.think(f'I can see the fire {rng}{abs(math.degrees(b.bearing)):.0f} deg '
               f'{"left" if b.bearing > 0 else "right"} - so I am really at ({nav.x:+.2f}, {nav.y:+.2f}), '
               f'{math.hypot(nav.x - old[0], nav.y - old[1]):.2f} m and '
               f'{abs(math.degrees(wrap(nav.th - old[2]))):.0f} deg off what I thought. Correcting my map position.')
    return True


def home_on(nav, name):
    """Visual servo onto the victim blob nearest straight ahead until its base reaches the bottom of the image.
    True when reached; False if it is lost, or the fire or an obstacle gets in the way (then nav.obstacle is set)."""
    robot, cam = nav.robot, nav.cam
    robot.aim(SONAR_CENTER_DEG)
    nav.obstacle = None
    track, gate, last_seen, t_prev = 0.0, math.radians(ACQUIRE_GATE_DEG), time.time(), time.time()
    while not STOP_EVENT.is_set():
        blobs = cam.snap(time.time() - 0.05, timeout=0.3)[0]
        now = time.time()
        mine = [b for b in blobs if b.kind == 'victim' and b.range is not None and abs(b.bearing - track) < gate]
        if not mine:
            robot.drive(wb.STOP)
            t_prev = now
            if now - last_seen > LOST_S:
                DASH.say(f'Lost sight of victim {name}')
                return False
            continue
        b = min(mine, key=lambda b: abs(b.bearing - track))
        track, gate, last_seen = b.bearing, math.radians(TRACK_GATE_DEG), now
        if b.bottom >= H * CLOSE_FRAC:
            nav.pause(0.3, f'reached victim {name}')
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
        # dead-reckon the homing too (roughly: straight along the current heading), so a reroute starts near the truth
        nav.x += CRUISE_MPS * (now - t_prev) * math.cos(nav.th)
        nav.y += CRUISE_MPS * (now - t_prev) * math.sin(nav.th)
        t_prev = now
        nav.moved()
        nav.status.state = f'homing on victim {name}: {b.label()}'
    return False


def reach_distance():
    """How far ahead a victim is when its base reaches CLOSE_FRAC of the image."""
    return CAM_HEIGHT_M * F_PX / (H * CLOSE_FRAC - HORIZON_Y)


def add_obstacle(hazards, p):
    hazards.append((p[0], p[1], OBSTACLE_CLEAR_M, 'an obstacle'))
    DASH.set(hazards=[list(h) for h in hazards])
    DASH.say(f'Obstacle marked on the map at ({p[0]:+.2f}, {p[1]:+.2f}) - REROUTING')


def explain_route(start, goal, path, hazards, label):
    near = [(seg_dist(start, goal, h[:2]), h) for h in hazards]
    if len(path) > 1:
        d, h = min(near, key=lambda n: n[0] - n[1][2])
        DASH.think(f'A straight line to {label} would pass {d:.2f} m from {h[3]} (I keep {h[2]:.2f} m), so I plan '
                   f'round it: ' + ' -> '.join(f'({x:+.2f},{y:+.2f})' for x, y in path) + '.')
    elif near:
        d, h = min(near, key=lambda n: n[0])
        if d < h[2]:                                                # only possible when the goal itself is that close
            DASH.think(f'{label[0].upper() + label[1:]} is only {math.hypot(goal[0] - h[0], goal[1] - h[1]):.2f} m '
                       f'from {h[3]}, so I have to go within {d:.2f} m of it - approaching directly and slowly, '
                       f'watching for it with the camera.')
        else:
            DASH.think(f'The straight line to {label} stays {d:.2f} m from {h[3]} - safe, going direct.')


def drive_route(nav, goal, hazards, label, last_leg=True):
    """Drive the planned legs to `goal` (all but the last one if not last_leg). An obstacle the sonar stops for joins
    the hazards and the route is planned again from where the robot stopped. False if it cannot get through."""
    for attempt in range(MAX_REROUTES + 1):
        path = plan((nav.x, nav.y), goal, [h[:3] for h in hazards])
        explain_route((nav.x, nav.y), goal, path, hazards, label)
        DASH.set(path=[[nav.x, nav.y]] + [list(p) for p in path])
        legs = path if last_leg else path[:-1]
        if all(nav.go_to(p, label) for p in legs):                 # all() stops at the first leg that fails
            return True
        if nav.obstacle is None:
            return False
        add_obstacle(hazards, nav.obstacle)
    DASH.say(f'Too many obstacles on the way to {label} - giving up this route')
    return False


def visit(nav, name, victim, hazards):
    """Route round the hazards to the victim, then home on it visually. True once reached."""
    goal = (victim[1], victim[2])
    label = f'victim {name}'
    for attempt in range(MAX_REROUTES + 1):
        if not drive_route(nav, goal, hazards, label, last_leg=False):
            return False
        if not nav.face(goal, f'facing {label}'):
            return False
        DASH.say(f'Homing on {label} with the camera')
        heading = nav.th
        if home_on(nav, name):
            # homing steered, so neither heading nor position is known exactly: we are about reach_distance() short
            # of the victim. Pin the position there and let the fire fix the heading.
            nav.th = heading
            nav.x = goal[0] - reach_distance() * math.cos(heading)
            nav.y = goal[1] - reach_distance() * math.sin(heading)
            nav.moved()
            return True
        if nav.obstacle is None:
            return False
        add_obstacle(hazards, nav.obstacle)
    return False


def go_home(nav, hazards, fire):
    DASH.set(target=None)
    for attempt in range(3):
        drive_route(nav, (0.0, 0.0), hazards, 'the start point')
        if fire:
            relocalise(nav, fire)
        err = math.hypot(nav.x, nav.y)
        if err <= HOME_TOL_M:
            DASH.think(f'Back at the start point (within {err:.2f} m).')
            break
        DASH.think(f'I am still {err:.2f} m from the start point - correcting.')
    nav.turn(wrap(-nav.th), 'facing the arena again')


def mission(nav, drive=True):
    DASH.say('Surveying the arena from the start point')
    objs = survey(nav)
    if drive and nav.dead:
        DASH.say('The motors cannot turn the robot - not driving. Check the shield battery (EXT_PWR) and its switch.')
        drive = False
    fires = [o for o in objs if o[0] == 'fire']
    victims = [o for o in objs if o[0] == 'victim']
    victims.sort(key=lambda v: -math.atan2(v[2], v[1]))            # names A, B, C... from left to right
    names = {id(v): chr(ord('A') + i) for i, v in enumerate(victims)}
    fire = max(fires, key=lambda f: f[3]) if fires else None
    order = rescue_order(victims, fires)
    DASH.set(objs=[{'name': 'FIRE', 'kind': 'fire', 'x': f[1], 'y': f[2], 'rank': None, 'reached': False}
                   for f in fires] +
             [{'name': names[id(v)], 'kind': 'victim', 'x': v[1], 'y': v[2], 'rank': order.index(v) + 1,
               'reached': False} for v in victims])
    hazards = [(f[1], f[2], FIRE_CLEAR_M, 'the fire') for f in fires]
    DASH.set(hazards=[list(h) for h in hazards])

    DASH.think(f'Survey done: I see {len(fires)} fire{"s" * (len(fires) != 1)} and {len(victims)} '
               f'victim{"s" * (len(victims) != 1)}.')
    if not victims:
        DASH.think('No victims in sight - nothing to rescue.')
        return
    for v in victims:
        DASH.think(f'Victim {names[id(v)]}: {fire_distance(v, fires):.2f} m from the fire, '
                   f'{math.hypot(v[1], v[2]):.2f} m from me.' if fires else
                   f'Victim {names[id(v)]}: {math.hypot(v[1], v[2]):.2f} m from me.')
    if fires:
        first = order[0]
        DASH.think(f'Victim {names[id(first)]} is closest to the fire ({fire_distance(first, fires):.2f} m), so it '
                   f'is in the most danger: I rescue it first. Order: '
                   + ' -> '.join(names[id(v)] for v in order) + '.')
    else:
        DASH.think('I cannot see the fire, so I cannot rank by danger - nearest first: '
                   + ' -> '.join(names[id(v)] for v in order) + '.')
    if not drive:
        return
    reached = 0
    for i, v in enumerate(order, 1):
        if STOP_EVENT.is_set():
            return
        if nav.dead:
            DASH.say('STOPPED: the motors cannot turn the robot any more. Fresh batteries needed.')
            DASH.think('Without turning I cannot reach anyone safely, so I stop here rather than drive blind.')
            DASH.set(target=None)
            return
        name = names[id(v)]
        DASH.set(target=name)
        if i > 1:
            DASH.think(f'Next is victim {name}: the most at-risk victim I have not reached yet '
                       f'({fire_distance(v, fires):.2f} m from the fire).' if fires else f'Next is victim {name}.')
        DASH.say(f'Moving towards victim {name}')
        others = [(o[1], o[2], VICTIM_CLEAR_M, f'victim {names[id(o)]}') for o in victims if o is not v]
        if visit(nav, name, v, hazards + others):
            reached += 1
            DASH.say(f'RESCUING victim {name} - reached it')
            with DASH.lock:
                for o in DASH.objs:
                    if o['name'] == name:
                        o['reached'] = True
            if fire:
                relocalise(nav, fire, (nav.x, nav.y))
        else:
            DASH.say(f'Could not reach victim {name}')
            DASH.think(f'Victim {name} is out of reach for now - I go back and carry on with the next one.')
        DASH.say('Returning to the start point')
        go_home(nav, hazards + [(o[1], o[2], VICTIM_CLEAR_M, f'victim {names[id(o)]}') for o in victims], fire)
    DASH.set(target=None, path=[])
    DASH.say(f'MISSION COMPLETE: {reached} of {len(order)} victims reached, back at the start point')
    DASH.think(f'All done: {reached} of {len(order)} victims reached in order of danger.')


# ---- calibration and tests ----------------------------------------------------------------------------------------
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


def selftest():
    # closest-to-fire first, even though another victim is nearer the robot
    order = rescue_order([['victim', 0.2, -0.5, 3], ['victim', 2.0, 1.0, 3], ['victim', 0.5, 0.0, 2]],
                         [['fire', 2.2, 1.3, 4]])
    assert [v[1] for v in order] == [2.0, 0.5, 0.2], order
    # a fire on the straight line forces a detour that keeps its distance
    path = plan((0, 0), (2, 0), [(1, 0.1, FIRE_CLEAR_M)])
    assert len(path) == 2 and path[-1] == (2, 0)
    for a, b in zip([(0, 0)] + path, path):
        assert seg_dist(a, b, (1, 0.1)) > FIRE_CLEAR_M * 0.9, (a, b)
    assert path[0][1] < 0                                           # passes on the side away from the fire
    assert plan((0, 0), (2, 0), [(1, 2, FIRE_CLEAR_M)]) == [(2, 0)]           # fire well off the line: straight
    assert plan((0, 0), (2, 0), [(2.2, 0, FIRE_CLEAR_M)]) == [(2, 0)]         # victim beside the fire: reachable
    # a fire and an obstacle the sonar found: the route keeps clear of both
    hz = [(1, 0.1, FIRE_CLEAR_M), (1.6, -0.5, OBSTACLE_CLEAR_M)]
    path = plan((0, 0), (2.5, 0), hz)
    for a, b in zip([(0, 0)] + path, path):
        for hx, hy, c in hz:
            assert seg_dist(a, b, (hx, hy)) > c * 0.6, (a, b, hx, hy)
    # sightings of one object from neighbouring views merge
    objs = cluster([('victim', 1.0, 1.0), ('victim', 1.1, 0.95), ('victim', 3, 3), ('fire', 1.0, 1.0)])
    assert len(objs) == 3 and [o[3] for o in objs] == [2, 1, 1]
    # geometry: centre pixel straight ahead, left half of the image = + bearing
    assert abs(Blob('victim', W // 2 - 5, 400, 10, 20).bearing) < 1e-9
    assert Blob('victim', 10, 400, 10, 20).bearing > 0
    # turning LEFT slides the scene RIGHT: rotation() must call that + (measured on the robot: right pivot = -dx)
    rng = np.random.default_rng(1)
    img = np.float32(cv2.GaussianBlur(rng.random((300, 480)), (0, 0), 3))
    a, conf = rotation(img, np.roll(img, 40, axis=1))
    assert math.degrees(a) > 3 and conf > MIN_CONF
    assert math.degrees(rotation(img, np.roll(img, -40, axis=1))[0]) < -3
    assert rotation(img, np.float32(cv2.GaussianBlur(rng.random((300, 480)), (0, 0), 3)))[1] < MIN_CONF  # no overlap
    assert 0.1 < reach_distance() < 0.5
    json.loads(DASH.json())
    print('selftest: OK')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for flag in ('--view', '--survey', '--sonar-cal', '--selftest'):
        ap.add_argument(flag, action='store_true')
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP_EVENT.set())
    cam = Camera()
    robot = None
    try:
        if not args.sonar_cal:
            cam.start()
        if args.view:
            DASH.set(state='view only - nothing moves')
            print('Viewing only. Ctrl+C to stop.')
            STOP_EVENT.wait()
            return
        robot = Uno(False)
        robot.open()
        robot.look(SONAR_CENTER_DEG)                                # the Uno resets on every open: servo back to centre
        if args.sonar_cal:
            sonar_cal(robot)
            return
        for n in range(wb.COUNTDOWN_S, 0, -1):
            DASH.set(state=f'starting in {n} s')
            print(f'Moving in {n}... (Ctrl+C to cancel)', flush=True)
            if STOP_EVENT.wait(1):
                return
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
