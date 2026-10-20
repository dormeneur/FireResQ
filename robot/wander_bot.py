#!/usr/bin/env python3
"""WANDER BOT - run ON THE PI. Drives forward, watches the ultrasonic sensor, and steers around whatever is in the way.

    python3 wander_bot.py                 # drive (5 s countdown first)
    python3 wander_bot.py --dry           # everything except the motors: robot can sit on a desk
    python3 wander_bot.py --sweep         # servo + sonar only: prints the distance at 3 angles, nothing moves
    python3 wander_bot.py --test-motors   # spins each wheel in turn (robot on blocks!) to check the layout below
    python3 wander_bot.py --no-camera

Live camera (ribbon camera, Picamera2): open http://pi.local:8090 in a browser on your PC.

Start-up flashes ./wander_fw.hex onto the Uno; Ctrl+C (or SIGTERM / a dropped ssh session) stops the motors and flashes
the robot's normal firmware (~/base_controller.ino.hex) back, so the wander sketch never stays on the Uno.

Wiring: motors M1 (front-left) M4 (front-right) / M2 (back-left) M3 (back-right) on an Adafruit Motor Shield v1,
servo on SERVO 1 (D10), ultrasonic TRIG = A3, ECHO = A4 (those two pins live in wander_fw.ino).
"""
import argparse
import queue
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

STOP_EVENT = threading.Event()           # set by Ctrl+C / SIGTERM / SIGHUP: every loop below must notice it

# ---- hardware ---------------------------------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'
BAUD = 115200
BANNER = 'WANDER2'
HERE = Path(__file__).resolve().parent
WANDER_HEX = HERE / 'wander_fw.hex'
RESTORE_HEX = Path.home() / 'base_controller.ino.hex'     # the robot's normal firmware, put back on exit

# ---- skid-steer drive -------------------------------------------------------------------------------------------
# Power vector order is (M1, M2, M3, M4). Left side = M1 front + M2 back, right side = M4 front + M3 back.
LEFT_MOTORS = (0, 1)
RIGHT_MOTORS = (3, 2)
MOTOR_NAMES = ('M1 front-left', 'M2 back-left', 'M3 back-right', 'M4 front-right')   # index order
FORWARD_SIGN = (-1, -1, -1, -1)          # sign that makes each wheel push the robot forward (flip one if a wheel runs backwards)
TRIM = (1.0, 1.0, 1.0, 1.0)              # per-motor gain: raise the weak (low-rpm) motor, e.g. 1.15, to even the sides out

# Sweet spot: 255 is full battery voltage, where the strong motors outrun the weak one and the robot pulls to one side.
# ~2/3 power drives straight, and leaves the weak motor headroom (MAX_POWER) for TRIM; turning in place scrubs four
# wheels against the floor, so it needs a bit more than cruising.
CRUISE = 170
SLOW = 130                               # used when something is getting close
TURN = 190
MAX_POWER = 215                          # no motor is ever commanded above this
TEST_POWER = 150

# ---- behaviour --------------------------------------------------------------------------------------------------
CENTER_DEG = 90
LEFT_DEG = 140                           # servo angle that looks to the robot's LEFT; if --sweep shows it the other way
RIGHT_DEG = 40                           # round, swap these two numbers
SLOW_CM = 60                             # start slowing down
STOP_CM = 30                             # something this close ahead: stop and decide
CLEAR_CM = 35                            # a side needs at least this much room to be worth turning toward
CONFIRM_READS = 2                        # consecutive close readings before reacting (a single ping can be noise)
CYCLE_S = 0.08                           # command cycle; must stay well under the Uno's 400 ms watchdog
BACKUP_S = 0.6
TURN_S = (0.5, 0.9)                      # (min, max) turn time; no encoders, so time is all we have
UTURN_S = 1.4
COUNTDOWN_S = 5
SERVO_STEP_S = 0.020                     # must match SERVO_STEP_MS in wander_fw.ino

CAMERA_PORT = 8090
CAMERA_SIZE = (640, 480)
CAMERA_FPS = 10


def drive_powers(left, right):
    """Differential drive: left/right side power (+ forward, - backward) -> signed (M1..M4) powers for the Uno."""
    out = [0] * 4
    for side, motors in ((left, LEFT_MOTORS), (right, RIGHT_MOTORS)):
        for m in motors:
            p = max(-MAX_POWER, min(MAX_POWER, round(side * TRIM[m])))
            out[m] = p * FORWARD_SIGN[m]
    return tuple(out)


STOP = (0, 0, 0, 0)
FORWARD = drive_powers(CRUISE, CRUISE)
FORWARD_SLOW = drive_powers(SLOW, SLOW)
BACKWARD = drive_powers(-SLOW, -SLOW)
SPIN_LEFT = drive_powers(-TURN, TURN)    # left side backward, right side forward
SPIN_RIGHT = drive_powers(TURN, -TURN)


class Robot:
    """The Uno over serial: send M/U/P lines, read U/D answers. A reader thread keeps the port drained."""

    def __init__(self, dry):
        self.dry = dry
        self.ser = None
        self.answers = queue.Queue()

    def open(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        time.sleep(2.0)                                  # opening the port resets the Uno
        self.ser.reset_input_buffer()
        if not self._banner():
            print('Uploading the wander firmware to the Uno...')
            self._flash(WANDER_HEX)
            if not self._banner():
                sys.exit('Uploaded, but the Uno does not answer as WANDER2.')
        threading.Thread(target=self._reader, daemon=True).start()

    def _banner(self):
        self.ser.reset_input_buffer()
        self.ser.write(b'?\n')
        end = time.time() + 1.5
        while time.time() < end:
            if self.ser.readline().decode('ascii', 'replace').startswith(BANNER):
                return True
        return False

    def _flash(self, hexfile):
        """avrdude needs the port to itself: close it, flash, reopen."""
        if not hexfile.exists():
            sys.exit(f'Missing {hexfile}')
        if self.ser and self.ser.is_open:
            self.ser.close()
        r = subprocess.run(['avrdude', '-p', 'atmega328p', '-c', 'arduino', '-P', PORT, '-b', '115200',
                            '-U', f'flash:w:{hexfile}:i'], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f'Flashing {hexfile.name} failed:\n' + r.stderr[-600:])
        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        time.sleep(2.0)

    def _reader(self):
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
        """Send the motor command. Must be repeated every cycle: the Uno releases the motors after 400 ms of silence."""
        if not self.dry and self.ser:
            self.ser.write(('M %d %d %d %d\n' % powers).encode())

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
        """Distance straight ahead of the servo in cm (400 = no echo), or None if the Uno did not answer."""
        return self._ask('P\n', 'D', 0, 0.4)

    def look(self, deg):
        """Point the sonar at deg and return the distance in cm (400 = no echo), or None."""
        return self._ask(f'U {deg}\n', 'U', deg, 5.0)

    def aim(self, deg):
        """Start the servo moving to deg without waiting for it."""
        if self.ser:
            self.ser.write(f'G {deg}\n'.encode())

    def close(self):
        """Stop, centre the servo, then flash the normal firmware back. Ctrl+C is ignored while this runs."""
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if self.ser and self.ser.is_open:
                for _ in range(3):
                    self.ser.write(b'M 0 0 0 0\n')
                    time.sleep(0.05)
                self.ser.write(b'U 90\n')
                time.sleep(0.4)
            if RESTORE_HEX.exists():
                print('Restoring the robot\'s normal firmware on the Uno...')
                self._flash(RESTORE_HEX)
                print('Restored.')
            else:
                print(f'WARNING: {RESTORE_HEX} not found, the wander firmware is still on the Uno.')
        except SystemExit as e:
            print(e)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()


class Status:
    """What the camera page overlays on the picture."""

    def __init__(self):
        self.state = 'starting'
        self.cm = None
        self.deg = CENTER_DEG
        self.dry = False


def run_camera(status, stop):
    """Ribbon camera (Picamera2) + tiny MJPEG web server. Any failure is reported and the robot carries on blind."""
    try:
        import cv2
        from picamera2 import Picamera2
    except ImportError as e:
        print(f'camera: {e}, no picture')
        return
    try:
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(main={'size': CAMERA_SIZE, 'format': 'RGB888'}))
        cam.start()
    except Exception as e:                                # no camera on the ribbon, or another program has it
        print(f'camera: could not start the ribbon camera ({e}), no picture')
        return

    latest = {'jpeg': None}
    cond = threading.Condition()

    def grab():
        interval = 1.0 / CAMERA_FPS
        while not stop.is_set():
            t0 = time.time()
            frame = cam.capture_array()                   # 'RGB888' is BGR in memory: what OpenCV wants
            ahead = '-' if status.cm is None else ('clear' if status.cm >= 400 else f'{status.cm} cm')
            text = f'{status.state}   ahead: {ahead}   servo {status.deg}' + ('   DRY RUN' if status.dry else '')
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
            cv2.putText(frame, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with cond:
                    latest['jpeg'] = jpg.tobytes()
                    cond.notify_all()
            time.sleep(max(0.0, interval - (time.time() - t0)))
        cam.stop()
        cam.close()

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
                page = (b'<!doctype html><title>Wander bot</title><body style="margin:0;background:#111;color:#ddd;'
                        b'font:16px sans-serif;text-align:center"><p>Wander bot - live camera</p>'
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
    status.state = state
    end = time.time() + seconds
    while time.time() < end and not STOP_EVENT.is_set():
        robot.drive(powers)
        time.sleep(CYCLE_S)


def avoid(robot, status):
    """Something is ahead: stop, look both ways, then spin toward the clearer side (or back up and U-turn if boxed in).
    The servo is non-blocking: it re-centres while the robot is already backing up / turning."""
    hold(robot, status, STOP, 0.2, 'obstacle - looking')
    seen = {}
    for name, deg in (('left', LEFT_DEG), ('right', RIGHT_DEG)):
        status.deg = deg
        status.state = f'looking {name}'
        seen[name] = robot.look(deg) or 0
        status.cm = seen[name]
    status.deg = CENTER_DEG
    robot.aim(CENTER_DEG)
    ready_at = time.time() + abs(RIGHT_DEG - CENTER_DEG) * SERVO_STEP_S + 0.2
    print(f'  left {seen["left"]} cm, right {seen["right"]} cm')
    best = max(seen, key=seen.get)
    spin = SPIN_LEFT if best == 'left' else SPIN_RIGHT
    if seen[best] < CLEAR_CM:
        hold(robot, status, BACKWARD, BACKUP_S, 'boxed in - backing up')
        hold(robot, status, spin, UTURN_S, f'boxed in - turning around ({best})')
    else:
        # the more room on the chosen side relative to STOP_CM, the shorter the turn needs to be
        frac = min(1.0, max(0.0, (STOP_CM + 40 - seen[best]) / 40.0))
        hold(robot, status, spin, TURN_S[0] + frac * (TURN_S[1] - TURN_S[0]), f'turning {best}')
    if ready_at > time.time():                             # servo still on its way back to centre
        hold(robot, status, STOP, ready_at - time.time(), 'centring sonar')


def wander(robot, status):
    close = 0
    while not STOP_EVENT.is_set():
        cm = robot.ping()
        status.cm = cm
        if cm is None:                                     # the Uno stopped answering: do not drive blind
            hold(robot, status, STOP, 0.5, 'no sonar answer - stopped')
            continue
        close = close + 1 if cm < STOP_CM else 0
        if close >= CONFIRM_READS:
            close = 0
            print(f'obstacle at {cm} cm')
            avoid(robot, status)
        elif cm < SLOW_CM:
            hold(robot, status, FORWARD_SLOW, CYCLE_S, 'slowing down')
        else:
            hold(robot, status, FORWARD, CYCLE_S, 'driving')


def sweep(robot):
    print('Servo + sonar only. Hold your hand about 20 cm from the sensor and see which angle sees it.')
    print(f'The robot\'s LEFT should be {LEFT_DEG} and its RIGHT {RIGHT_DEG}; if not, swap LEFT_DEG/RIGHT_DEG in the script.')
    while not STOP_EVENT.is_set():
        row = []
        for deg in (RIGHT_DEG, CENTER_DEG, LEFT_DEG, CENTER_DEG):
            cm = robot.look(deg)
            row.append(f'{deg:3d}deg: ' + ('---' if cm is None else ('clear' if cm >= 400 else f'{cm:3d} cm')))
        print('   '.join(row))


def test_motors(robot):
    print('Each wheel turns FORWARD for 1 s. Wheel on blocks! Starting in 5 s...')
    if STOP_EVENT.wait(5):
        return
    for i, name in enumerate(MOTOR_NAMES):
        print(f'{name}: forward')
        powers = [0] * 4
        powers[i] = TEST_POWER * FORWARD_SIGN[i]
        end = time.time() + 1.0
        while time.time() < end and not STOP_EVENT.is_set():
            robot.drive(tuple(powers))
            time.sleep(CYCLE_S)
        robot.drive(STOP)
        STOP_EVENT.wait(0.8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true', help='do not send any motor command')
    ap.add_argument('--sweep', action='store_true', help='only test the servo and the sonar')
    ap.add_argument('--test-motors', action='store_true', help='spin each wheel forward in turn')
    ap.add_argument('--no-camera', action='store_true')
    args = ap.parse_args()

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: STOP_EVENT.set())
    status = Status()
    status.dry = args.dry
    robot = Robot(args.dry)
    robot.open()
    try:
        if args.sweep:
            sweep(robot)
            return
        if args.test_motors:
            test_motors(robot)
            return
        if not args.no_camera:
            threading.Thread(target=run_camera, args=(status, STOP_EVENT), daemon=True).start()
        if args.dry:
            print('DRY RUN: no motor commands are sent.')
        else:
            for n in range(COUNTDOWN_S, 0, -1):
                print(f'Driving in {n}... (Ctrl+C to cancel). Put it on the floor with room around it.')
                if STOP_EVENT.wait(1):
                    return
        print('Running. Ctrl+C stops it and restores the Uno firmware.')
        wander(robot, status)
    except KeyboardInterrupt:
        pass
    finally:
        STOP_EVENT.set()
        robot.close()
        print('Stopped.')


if __name__ == '__main__':
    main()
