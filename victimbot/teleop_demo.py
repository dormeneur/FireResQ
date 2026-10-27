#!/usr/bin/env python3
"""TELEOP DEMO - the robot driven BY HAND from a web page, with operator-chosen narration on the camera feed (a
"Wizard of Oz" set-up: nothing here decides anything by itself). Independent of victimbot.py / rescue3.py. Run ON THE
PI, from the folder that holds dashboard.html:

    python3 -u teleop_demo.py

    http://pi.local:8000   CONTROL (the operator): drive with W/A/S/D or the arrow keys (hold to move), STOP, a small
                           camera preview, and the narration buttons
    http://pi.local:8090   VIEWER (a second laptop): the FireResQ dashboard - live camera with the narration caption
                           and the fire / sticky-note boxes, map, actions, thinking

Motors: the Uno must run victimbot_fw (M a b c d lines, 400 ms watchdog: the robot stops when commands stop). The moves
are the ones measured to work on this robot (the car-control page's settings with front/back and left/right swapped).
"""
import json
import os
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SERIAL_PORT, BAUD = '/dev/ttyUSB0', 115200
CONTROL_PORT, VIEW_PORT = 8000, 8090
MOVES = {'forward': (-255, -255, -255, -255), 'backward': (255, 255, 255, 255),
         'left': (0, 0, -255, -255), 'right': (-255, -255, 0, 0), 'stop': (0, 0, 0, 0)}
COMMAND_TIMEOUT_S = 0.5                  # no command from the page for this long: stop
SEND_HZ = 10                             # the Uno's watchdog releases the motors after 0.4 s of silence

# camera: OV5647 mounted turned 90 deg clockwise
CAM_SIZE = (960, 720)
ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE
W, H = 720, 960
FPS = 8
# what is drawn on the feed: the red box (fire) and the yellow-green sticky notes (victims)
RED_HSV = (((0, 170, 60), (4, 255, 255)), ((174, 170, 60), (179, 255, 255)))
FIRE_MIN_AREA = 600
NOTE_HSV = ((30, 90, 90), (62, 255, 255))
NOTE_MIN_AREA = 10                       # half-size picture
MAGNET_ROW = int(H * 0.86)

# NARRATION: what each button puts on the camera feed (short) and in the dashboard's thinking panel (longer)
SAY = {
    'scan': ('Scanning the scene',
             'Looking at the scene from the safe zone: one fire (the red box) and the victims in front of it.'),
    'fire': ('Fire detected',
             'Fire detected: the red box. Victims close to it are in the most danger.'),
    'found': ('3 victims found',
              'Three victims found. Ranking them by their distance to the fire: closest first.'),
    'going': ('Going towards victim {v}',
              'Victim {v} is the closest to the fire of those left, so it is in the most danger: rescuing it first.'),
    'align': ('Aligning with victim {v}',
              'Turning until victim {v} is in the centre of the picture, so the magnet meets it head on.'),
    'picked': ('Picked up victim {v} - going to the safe zone',
               'Victim {v} is on the magnet. Carrying it back to the safe zone.'),
    'saved': ('Victim {v} saved',
              'Victim {v} is in the safe zone.'),
    'complete': ('Mission complete - all victims saved',
                 'No victims left in front of the fire. All of them are in the safe zone.'),
}


class Robot:
    """Holds the current move and keeps sending it; stops when the page goes quiet."""

    def __init__(self):
        self.move, self.last_cmd, self.ser = 'stop', 0.0, None
        self.lock = threading.Lock()

    def start(self):
        import serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2)
            time.sleep(2.0)                                         # opening the port resets the Uno
            self.ser.reset_input_buffer()
            self.ser.write(b'?\n')
            time.sleep(0.3)
            print('Uno:', self.ser.read(200).decode('ascii', 'replace').strip() or 'no answer', flush=True)
        except Exception as e:                                      # keep the pages up without motors
            print(f'motors: {e} - running without them', flush=True)
            self.ser = None
        threading.Thread(target=self._loop, daemon=True).start()

    def set(self, move):
        with self.lock:
            self.move = move if move in MOVES else 'stop'
            self.last_cmd = time.time()

    def _loop(self):
        while True:
            with self.lock:
                move = self.move if time.time() - self.last_cmd < COMMAND_TIMEOUT_S else 'stop'
            if self.ser:
                try:
                    self.ser.write(('M %d %d %d %d\n' % MOVES[move]).encode())
                    self.ser.reset_input_buffer()
                except Exception as e:
                    print(f'motors: {e}', flush=True)
            time.sleep(1.0 / SEND_HZ)


class Show:
    """The caption on the feed and everything the dashboard shows."""

    def __init__(self):
        self.lock = threading.Lock()
        self.caption = 'FireResQ ready'
        self.log, self.thoughts = [], []

    def say(self, key, victim=None, text=None):
        if key == 'custom':
            short = long = (text or '').strip()[:80]
        else:
            short, long = SAY[key]
            short, long = short.format(v=victim or ''), long.format(v=victim or '')
        if not short:
            return
        stamp = time.strftime('%H:%M:%S')
        with self.lock:
            self.caption = short
            self.log.append([stamp, short])
            self.thoughts.append([stamp, long])
            del self.log[:-200], self.thoughts[:-200]
        print(f'{stamp} {short}', flush=True)

    def state(self):
        with self.lock:
            return json.dumps({'log': self.log, 'thoughts': self.thoughts, 'objs': [], 'hazards': [], 'path': [],
                               'trail': [[0.0, 0.0]], 'pose': [0.0, 0.0, 0.0], 'target': None,
                               'state': self.caption})


def detect(frame):
    """(fire boxes, victim boxes) as (x, y, w, h): the biggest pure-red blob, and the yellow-green sticky notes
    (left to right)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RED_HSV:
        red |= cv2.inRange(hsv, lo, hi)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    fires = [cv2.boundingRect(c) for c in cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
             if cv2.contourArea(c) >= FIRE_MIN_AREA]
    fires = sorted(fires, key=lambda r: -r[2] * r[3])[:1]           # there is ONE fire
    small = cv2.resize(hsv, (W // 2, H // 2), interpolation=cv2.INTER_NEAREST)
    notes = cv2.morphologyEx(cv2.inRange(small, *NOTE_HSV), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(notes)
    victims = sorted((tuple(int(2 * v) for v in stats[i][:4]) for i in range(1, n)
                      if stats[i][4] >= NOTE_MIN_AREA and stats[i][1] + stats[i][3] < MAGNET_ROW // 2),
                     key=lambda r: r[0])
    return fires, victims


def draw(frame, fires, victims, caption):
    for x, y, w, h in fires:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, 'FIRE', (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
    for i, (x, y, w, h) in enumerate(victims):
        cv2.rectangle(frame, (x - 4, y - 4), (x + w + 4, y + h + 4), (255, 255, 0), 2)
        cv2.putText(frame, f'VICTIM {chr(65 + i)}', (x - 4, max(18, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 0), 2, cv2.LINE_AA)
    # the caption: a band across the bottom of the picture
    cv2.rectangle(frame, (0, H - 70), (W, H), (0, 0, 0), -1)
    scale = 0.9 if len(caption) <= 34 else 0.65
    (tw, _), _ = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    cv2.putText(frame, caption, (max(10, (W - tw) // 2), H - 25), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2,
                cv2.LINE_AA)


class Camera:
    def __init__(self, show):
        self.show = show
        self.cond = threading.Condition()
        self.jpeg = None

    def start(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(main={'size': CAM_SIZE, 'format': 'RGB888'}, buffer_count=2))
        cam.start()
        threading.Thread(target=self._grab, args=(cam,), daemon=True).start()

    def _grab(self, cam):
        while True:
            t = time.time()
            try:
                frame = cv2.rotate(cam.capture_array(wait=3.0), ROTATE)
            except Exception as e:
                print(f'camera: {e}', flush=True)
                continue
            fires, victims = detect(frame)
            draw(frame, fires, victims, self.show.caption)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                with self.cond:
                    self.jpeg = jpg.tobytes()
                    self.cond.notify_all()
            time.sleep(max(0.0, 1.0 / FPS - (time.time() - t)))


CONTROL_HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleop Control</title>
<style>
  :root { --bg:#0e1116; --panel:#161b22; --line:#2a313c; --text:#d7dde5; --muted:#8b95a3; --go:#3ddc84; --stop:#ff4d3d; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text); font:15px/1.4 system-ui, sans-serif; }
  header { padding:10px 16px; border-bottom:1px solid var(--line); display:flex; gap:16px; align-items:center; }
  header b { color:var(--stop); } #cap { color:var(--muted); }
  main { display:grid; grid-template-columns:minmax(0, 1fr) minmax(0, 1.3fr); gap:14px; padding:14px 16px; }
  @media (max-width:900px) { main { grid-template-columns:1fr; } }
  section { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:12px; }
  h2 { margin:0 0 10px; font-size:12px; letter-spacing:1px; text-transform:uppercase; color:var(--muted); }
  img { width:100%; max-height:52vh; object-fit:contain; background:#000; border-radius:6px; }
  .pad { display:grid; grid-template-columns:repeat(3, 70px); gap:8px; justify-content:center; margin:10px 0; }
  button { font:inherit; color:var(--text); background:#1f2630; border:1px solid var(--line); border-radius:8px;
           padding:9px 10px; cursor:pointer; }
  button:hover { border-color:var(--muted); } button.on { background:#2b3a4c; border-color:var(--go); }
  .pad button { height:56px; font-size:20px; }
  #stop { width:100%; background:var(--stop); border-color:var(--stop); color:#fff; font-weight:700; font-size:18px; }
  .row { display:flex; flex-wrap:wrap; gap:8px; margin-bottom:10px; align-items:center; }
  .victim { border-top:1px solid var(--line); padding-top:10px; }
  .victim b { width:72px; display:inline-block; }
  input[type=text] { flex:1; min-width:180px; font:inherit; color:var(--text); background:#0e1116; border:1px solid var(--line);
                     border-radius:8px; padding:8px; }
  .hint { color:var(--muted); font-size:13px; text-align:center; }
</style></head><body>
<header><strong>Fire<b>ResQ</b> teleop</strong><span id="cap">-</span></header>
<main>
  <section><h2>Drive (hold)</h2>
    <img src="/stream" alt="camera">
    <div class="pad">
      <span></span><button data-m="forward">&#9650;</button><span></span>
      <button data-m="left">&#9664;</button><button data-m="backward">&#9660;</button><button data-m="right">&#9654;</button>
    </div>
    <button id="stop">STOP</button>
    <p class="hint">Keys: W/A/S/D or arrows (hold). Space = stop. Turns start ~0.6 s after the key goes down.</p>
  </section>
  <section><h2>Narration (shown on the viewer's camera feed)</h2>
    <div class="row">
      <button data-say="scan">Scanning the scene</button><button data-say="fire">Fire detected</button>
      <button data-say="found">3 victims found</button><button data-say="complete">Mission complete</button>
    </div>
    <div id="victims"></div>
    <div class="row victim"><input type="text" id="custom" placeholder="Custom caption"><button id="send">Show</button></div>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
const post = body => fetch('/cmd', {method:'POST', body:JSON.stringify(body)}).catch(() => {});
for (const v of ['A', 'B', 'C']) {
  const row = document.createElement('div'); row.className = 'row victim';
  row.innerHTML = `<b>Victim ${v}</b>` + [['going','Going towards'], ['align','Aligning'], ['picked','Picked up'], ['saved','Saved']]
    .map(([k, t]) => `<button data-say="${k}" data-v="${v}">${t}</button>`).join('');
  $('victims').append(row);
}
document.addEventListener('click', e => {
  const b = e.target.closest('[data-say]'); if (b) post({action:'say', key:b.dataset.say, victim:b.dataset.v || null});
});
$('send').onclick = () => { post({action:'say', key:'custom', text:$('custom').value}); $('custom').value = ''; };
$('custom').addEventListener('keydown', e => { if (e.key === 'Enter') $('send').click(); e.stopPropagation(); });
let move = 'stop';
setInterval(() => post({action:'move', move}), 100);                    // the robot stops if this stops
function go(m) { move = m; for (const b of document.querySelectorAll('[data-m]')) b.classList.toggle('on', b.dataset.m === m); }
for (const b of document.querySelectorAll('[data-m]')) {
  b.addEventListener('pointerdown', () => go(b.dataset.m));
  for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) b.addEventListener(ev, () => go('stop'));
}
$('stop').onclick = () => go('stop');
const KEYS = {w:'forward', arrowup:'forward', s:'backward', arrowdown:'backward', a:'left', arrowleft:'left', d:'right', arrowright:'right'};
addEventListener('keydown', e => { const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); go('stop'); } else if (KEYS[k]) { e.preventDefault(); go(KEYS[k]); } });
addEventListener('keyup', e => { if (KEYS[e.key.toLowerCase()] === move) go('stop'); });
addEventListener('blur', () => go('stop'));
setInterval(async () => { try { $('cap').textContent = JSON.parse(await (await fetch('/state')).text()).state; } catch (e) {} }, 700);
</script></body></html>'''


def handler(robot, show, camera, page):
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
                self._send(show.state().encode(), 'application/json')
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with camera.cond:
                            camera.cond.wait(timeout=2.0)
                            jpg = camera.jpeg
                        if jpg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                             % len(jpg) + jpg + b'\r\n')
                except OSError:
                    pass
            else:
                self._send(page(), 'text/html; charset=utf-8')

        def do_POST(self):
            if self.path != '/cmd' or robot is None:
                self.send_error(404)
                return
            try:
                cmd = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                if cmd.get('action') == 'move':
                    robot.set(cmd.get('move', 'stop'))
                elif cmd.get('action') == 'say' and (cmd.get('key') in SAY or cmd.get('key') == 'custom'):
                    show.say(cmd['key'], cmd.get('victim'), cmd.get('text'))
            except (ValueError, KeyError, TypeError):
                self.send_error(400)
                return
            self._send(b'', 'text/plain')
    return Handler


class DualStackServer(ThreadingHTTPServer):
    address_family = socket.AF_INET6
    daemon_threads = True

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


def stop_other_copies():
    """Only one program can hold the camera and the Uno: stop any other copy of this demo, and the autonomous
    programs (rescue3.py, victimbot.py), before starting."""
    me, others = os.getpid(), []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if int(pid) != me and Path(argv[0].decode(errors='replace')).name.startswith('python') and any(
                a.endswith((b'teleop_demo.py', b'rescue3.py', b'victimbot.py')) for a in argv[1:]):
            others.append(int(pid))
    if not others:
        return
    print(f'Stopping the program already using the camera ({", ".join(map(str, others))})...', flush=True)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for p in others:
            try:
                os.kill(p, sig)
            except ProcessLookupError:
                pass
        for _ in range(30):
            if not any(os.path.exists(f'/proc/{p}') for p in others):
                time.sleep(0.5)                                     # let the camera driver let go
                return
            time.sleep(0.1)


def main():
    stop_other_copies()
    show, robot = Show(), Robot()
    camera = Camera(show)
    camera.start()
    robot.start()
    servers = [DualStackServer(('::', CONTROL_PORT), handler(robot, show, camera, lambda: CONTROL_HTML.encode())),
               DualStackServer(('::', VIEW_PORT), handler(None, show, camera,
                                                          lambda: (HERE / 'dashboard.html').read_bytes()))]
    for s in servers:
        threading.Thread(target=s.serve_forever, daemon=True).start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as u:
            u.connect(('10.255.255.255', 1))
            ip = u.getsockname()[0]
    except OSError:
        ip = 'pi.local'
    print(f'CONTROL: http://{ip}:{CONTROL_PORT}    VIEWER: http://{ip}:{VIEW_PORT}    (Ctrl+C to stop)', flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.set('stop')
        time.sleep(0.3)
        print('Stopped.')


if __name__ == '__main__':
    main()
