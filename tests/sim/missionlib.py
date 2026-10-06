"""Helpers for the integrated rescue-mission tests: watch the mission, and keep Gazebo's TRUE poses to judge it against.

Ground truth is read here and in the tests ONLY to assert; nothing in this file feeds the mission (the rescue node has no access
to it: fire_resq_planning imports no simulator code, and a unit guard says so).
"""
import json
import math
import os
import subprocess
import threading
import time
from collections import defaultdict

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from simlib import WORLD, _quat, yaw_of

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
TERMINAL_STATES = ('MISSION_COMPLETE', 'MISSION_ABORTED')


class TruthLog:
    """Streams the TRUE (x, y, z, yaw) of named models with a wall-clock stamp; keeps the whole history. Test-only."""

    def __init__(self, names):
        self.names = set(names)
        self.lock = threading.Lock()
        self.hist = defaultdict(list)                 # name -> [(wall, x, y, z, yaw)]
        self.p = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info', '--json-output'],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        dec, buf = json.JSONDecoder(), ''
        for chunk in self.p.stdout:
            buf += chunk
            while True:
                buf = buf.lstrip()
                if not buf:
                    break
                try:
                    obj, end = dec.raw_decode(buf)
                except ValueError:
                    break
                buf = buf[end:]
                now = time.time()
                for e in obj.get('pose', []):
                    n = e.get('name')
                    if n in self.names:
                        pos = e.get('position', {})
                        with self.lock:
                            self.hist[n].append((now, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), yaw_of(_quat(e))))

    def latest(self, name):
        with self.lock:
            h = self.hist.get(name)
            return h[-1] if h else None

    def track(self, name, t0=0.0, t1=math.inf):
        with self.lock:
            return [r for r in self.hist.get(name, ()) if t0 <= r[0] <= t1]

    def stop(self):
        self.p.terminate()


class MissionTap:
    """Subscribes to /fire_resq/mission_state (latched JSON) and to /cmd_vel; remembers every distinct state seen with the wall
    time it was first seen, so tests can react to (and time) the mission."""

    def __init__(self, bot):
        self.bot = bot
        self.last = None
        self.first_seen = {}                         # state -> wall time first seen
        self.seen = []                               # ordered distinct (state, wall, snapshot)
        self.cmd = []                                # (wall, vx, wz) from every publisher of /cmd_vel: the FSM AND Nav2
        self.callbacks = []                          # called with (snapshot) on every message
        bot.create_subscription(String, '/fire_resq/mission_state', self._on, LATCHED)
        bot.create_subscription(Twist, '/cmd_vel', lambda m: self.cmd.append((time.time(), m.linear.x, m.angular.z)), 50)

    def _on(self, m):
        snap = json.loads(m.data)
        self.last = snap
        now = time.time()
        if not self.seen or self.seen[-1][0] != snap['state']:
            self.seen.append((snap['state'], now, snap))
            self.first_seen.setdefault(snap['state'], now)
        for cb in list(self.callbacks):
            cb(snap)

    @property
    def state(self):
        return None if self.last is None else self.last['state']

    def history(self):
        return [] if self.last is None else self.last['history']

    def wait(self, cond, timeout, what):
        self.bot.wait_for(lambda: self.last is not None and cond(self.last), timeout, what)
        return self.last


def cpu_seconds(pattern):
    """Total user+system CPU seconds of every process whose command line matches `pattern` (Linux /proc)."""
    total = 0.0
    out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
    for pid in out:
        try:
            f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
            total += (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')
        except (OSError, IndexError):
            pass
    return total


def victim_id_by_position(world, xy, tol=0.6):
    """The world model's id of the victim nearest the map-frame point `xy` (how a test matches a scenario victim to a belief)."""
    best = min(world.victims, key=lambda v: math.hypot(v.position.x - xy[0], v.position.y - xy[1]), default=None)
    if best is None or math.hypot(best.position.x - xy[0], best.position.y - xy[1]) > tol:
        return None
    return best.id
