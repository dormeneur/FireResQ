#!/usr/bin/env python3
"""FireResQ WANDER BOT - run ON THE PI. Drives forward, looks ahead with the ultrasonic sensor, and when something is
in the way it stops, looks left and right with the pan servo, backs up if boxed in, and turns toward the clearer side.
Every few seconds of free driving it also makes a short random turn, so it wanders instead of going in straight lines.

    python3 wander_bot.py              # drive (5 s countdown first)
    python3 wander_bot.py --dry        # everything except the motors: robot can sit on a desk. Shows what it WOULD do
    python3 wander_bot.py --sweep      # servo + sonar only: prints distance at 30/90/150 degrees, nothing moves
    python3 wander_bot.py --no-camera  # skip the camera page

The camera page is served on http://pi.local:8090 (any browser on the same network). Ctrl+C stops the robot.

Motor numbers are from docs/MOTOR_CONFIG.md (shield terminals M1 front-right, M2 front-left, M3 back-left,
M4 back-right). The Uno firmware is firmware/wander_controller; if the Uno is running something else this script
flashes ./wander_controller.hex onto it first.
"""
import argparse
import queue
import random
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

STOP_EVENT = threading.Event()           # set by Ctrl+C / SIGTERM: every loop below must notice it

# ---- hardware ---------------------------------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'
BAUD = 115200
BANNER = 'FIRERESQ_WANDER'
HEX = Path(__file__).with_name('wander_controller.hex')

# ---- motor patterns: docs/MOTOR_CONFIG.md, order M1 M2 M3 M4 -----------------------------------------------------
STOP = (0, 0, 0, 0)
FORWARD = (-168, -168, -168, -168)       # negative drives the robot forward (reversed 2026-10-17, see MOTOR_CONFIG.md)
BACKWARD = (168, 168, 168, 168)
TURN_LEFT = (255, -255, -255, 255)       # all four wheels
TURN_RIGHT = (-255, 0, 0, -255)          # only the two right wheels, backward (the left wheels spoil it)

# ---- behaviour --------------------------------------------------------------------------------------------------
CENTER_DEG = 90
LEFT_DEG = 150                           # servo angle that looks to the robot's LEFT. If --sweep shows it the other way
RIGHT_DEG = 30                           # round, swap these two numbers.
STOP_CM = 30                             # something this close ahead: react
CLEAR_CM = 35                            # a side needs at least this much room to be worth turning toward
CONFIRM_READS = 2                        # consecutive close readings before reacting (a single ping can be noise)
CYCLE_S = 0.1                            # command cycle; must stay well under the Uno's 400 ms watchdog
BACKUP_S = 0.6
TURN_S = (0.5, 0.9)                      # random duration of an avoidance turn (no encoders: time is all we have)
WANDER_EVERY_S = (6.0, 12.0)             # random gap between spontaneous turns while the way is clear
WANDER_TURN_S = (0.25, 0.5)
COUNTDOWN_S = 5

CAMERA_PORT = 8090
CAMERA_SIZE = (640, 480)
CAMERA_FPS = 10                          # frames served per second (the Pi 3 has to JPEG-encode each one)


class Robot:
    """The Uno over serial: send M/U lines, read U answers. A reader thread keeps the port drained."""

    def __init__(self, dry):
        self.dry = dry
        self.ser = None
        self.answers = queue.Queue()
        self.applied = STOP
        self.lock = threading.Lock()
        self.last_m = STOP

    def open(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        time.sleep(2.0)                                  # opening the port resets the Uno
        self.ser.reset_input_buffer()
        self._ensure_firmware()
        threading.Thread(target=self._reader, daemon=True).start()

    def _banner(self):
        self.ser.reset_input_buffer()
        self.ser.write(b'?\n')
        end = time.time() + 1.5
        while time.time() < end:
            if self.ser.readline().decode('ascii', 'replace').startswith(BANNER):
                return True
        return False

    def _ensure_firmware(self):
        if self._banner():
            return
        print('The Uno is not running the wander firmware: flashing it...')
        if not HEX.exists():
            sys.exit(f'Missing {HEX}')
        self.ser.close()
        r = subprocess.run(['avrdude', '-p', 'atmega328p', '-c', 'arduino', '-P', PORT, '-b', '115200',
                            '-U', f'flash:w:{HEX}:i'], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit('Flashing failed:\n' + r.stderr[-600:])
        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        time.sleep(2.0)
        if not self._banner():
            sys.exit('Flashed, but the Uno still does not answer as FIRERESQ_WANDER.')
        print('Flashed.')

    def _reader(self):
        while self.ser and self.ser.is_open:
            try:
                text = self.ser.readline().decode('ascii', 'replace').strip()
            except (serial.SerialException, OSError):
                return
            if text.startswith('U '):
                parts = text.split()
                if len(parts) == 3 and parts[1].lstrip('-').isdigit() and parts[2].isdigit():
                    self.answers.put((int(parts[1]), int(parts[2])))
            elif text.startswith('S '):
                try:
                    self.applied = tuple(int(v) for v in text.split()[1:5])
                except ValueError:
                    pass

    def drive(self, powers):
        """Send the motor command. Must be repeated every cycle: the Uno releases the motors after 400 ms of silence."""
        self.last_m = powers
        if not self.dry and self.ser:
            self.ser.write(('M %d %d %d %d\n' % powers).encode())

    def look(self, deg, timeout=8.0):
        """Point the sonar at deg and return the distance in cm (400 = no echo), or None if the Uno did not answer."""
        while not self.answers.empty():
            self.answers.get_nowait()
        self.ser.write(f'U {deg}\n'.encode())
        end = time.time() + timeout
        while time.time() < end and not STOP_EVENT.is_set():
            try:
                got_deg, cm = self.answers.get(timeout=0.05)
            except queue.Empty:
                continue
            if got_deg == deg:
                return cm
        return None

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                for _ in range(3):
                    self.ser.write(b'M 0 0 0 0\n')
                    time.sleep(0.05)
                self.ser.write(b'U 90\n')
                time.sleep(0.3)
            finally:
                self.ser.close()


class Status:
    """What the camera page overlays on the picture."""

    def __init__(self):
        self.state = 'starting'
        self.cm = None
        self.deg = CENTER_DEG
        self.motors = STOP
        self.dry = False


def run_camera(status, stop):
    """Camera + tiny MJPEG web server. Any failure here is reported and the robot carries on without a picture."""
    try:
        import cv2
    except ImportError:
        print('camera: OpenCV is not installed, no picture')
        return
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print('camera: /dev/video0 would not open (is another program using it?), no picture')
        return

    latest = {'jpeg': None}
    cond = threading.Condition()

    def grab():
        interval = 1.0 / CAMERA_FPS
        while not stop.is_set():
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.2)
                continue
            text = f'{status.state}   ahead: {"-" if status.cm is None else ("clear" if status.cm >= 400 else str(status.cm) + " cm")}' \
                   f'   servo {status.deg}' + ('   DRY RUN' if status.dry else '')
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
            cv2.putText(frame, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with cond:
                    latest['jpeg'] = jpg.tobytes()
                    cond.notify_all()
            time.sleep(max(0.0, interval - (time.time() - t0)))
        cap.release()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while not stop.is_set():
                        with cond:
                            cond.wait(timeout=2.0)
                            jpg = latest['jpeg']
                        if jpg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpg))
                            self.wfile.write(jpg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
            else:
                page = (b'<!doctype html><title>FireResQ wander bot</title><body style="margin:0;background:#111;color:#ddd;'
                        b'font:16px sans-serif;text-align:center"><p>FireResQ wander bot - live camera</p>'
                        b'<img src="/stream" style="max-width:100%"></body>')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)

    server = ThreadingHTTPServer(('0.0.0.0', CAMERA_PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=grab, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'camera: open http://pi.local:{CAMERA_PORT} in a browser on your PC')
    stop.wait()
    server.shutdown()


def hold(robot, status, powers, seconds, state):
    """Keep sending one motor command for a while (the Uno's watchdog needs it repeated)."""
    status.state, status.motors = state, powers
    end = time.time() + seconds
    while time.time() < end:
        robot.drive(powers)
        time.sleep(CYCLE_S)


def avoid(robot, status):
    """Something is ahead: stop, look both ways, then back up and/or turn toward the clearer side."""
    hold(robot, status, STOP, 0.3, 'obstacle - looking')
    seen = {}
    for name, deg in (('left', LEFT_DEG), ('right', RIGHT_DEG)):
        status.deg = deg
        seen[name] = robot.look(deg) or 0
        status.cm = seen[name]
        robot.drive(STOP)
    status.deg = CENTER_DEG
    robot.look(CENTER_DEG)
    print(f'  left {seen["left"]} cm, right {seen["right"]} cm')
    best = max(seen, key=seen.get)
    if seen[best] < CLEAR_CM:
        hold(robot, status, BACKWARD, BACKUP_S, 'boxed in - backing up')
        best = 'left'                                      # the left turn is the strong one
    turn = TURN_LEFT if best == 'left' else TURN_RIGHT
    hold(robot, status, turn, random.uniform(*TURN_S), f'turning {best}')
    hold(robot, status, STOP, 0.2, 'looking')


def wander(robot, status, stop):
    close = 0
    next_wander = time.time() + random.uniform(*WANDER_EVERY_S)
    while not stop.is_set():
        robot.drive(FORWARD)
        status.state, status.motors = 'driving', FORWARD
        cm = robot.look(CENTER_DEG)
        status.cm = cm
        if cm is None:                                     # the Uno stopped answering: do not drive blind
            hold(robot, status, STOP, 0.5, 'no sonar answer - stopped')
            continue
        close = close + 1 if cm < STOP_CM else 0
        if close >= CONFIRM_READS:
            close = 0
            print(f'obstacle at {cm} cm')
            avoid(robot, status)
            next_wander = time.time() + random.uniform(*WANDER_EVERY_S)
        elif time.time() >= next_wander:
            turn = random.choice((TURN_LEFT, TURN_RIGHT))
            hold(robot, status, turn, random.uniform(*WANDER_TURN_S), 'wandering')
            next_wander = time.time() + random.uniform(*WANDER_EVERY_S)
        time.sleep(0.02)


def sweep(robot):
    print('Servo + sonar only. Hold your hand about 20 cm from the sensor and see which angle sees it.')
    print(f'The robot\'s LEFT should be {LEFT_DEG} and its RIGHT {RIGHT_DEG}; if not, swap LEFT_DEG/RIGHT_DEG in the script.')
    while not STOP_EVENT.is_set():
        row = []
        for deg in (RIGHT_DEG, CENTER_DEG, LEFT_DEG, CENTER_DEG):
            cm = robot.look(deg)
            row.append(f'{deg:3d}deg: ' + ('---' if cm is None else ('clear' if cm >= 400 else f'{cm:3d} cm')))
        print('   '.join(row))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true', help='do not send any motor command')
    ap.add_argument('--sweep', action='store_true', help='only test the servo and the sonar')
    ap.add_argument('--no-camera', action='store_true')
    args = ap.parse_args()

    stop = STOP_EVENT
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    status = Status()
    status.dry = args.dry
    robot = Robot(args.dry)
    robot.open()
    try:
        if args.sweep:
            sweep(robot)
            return
        if not args.no_camera:
            threading.Thread(target=run_camera, args=(status, stop), daemon=True).start()
        if args.dry:
            print('DRY RUN: no motor commands are sent.')
        else:
            for n in range(COUNTDOWN_S, 0, -1):
                print(f'Driving in {n}... (Ctrl+C to cancel). Put it on the floor with room around it.')
                if stop.wait(1):
                    return
        print('Wandering. Ctrl+C stops it.')
        wander(robot, status, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        robot.close()
        print('Stopped.')


if __name__ == '__main__':
    main()
