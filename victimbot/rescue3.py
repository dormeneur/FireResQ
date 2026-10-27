#!/usr/bin/env python3
"""RESCUE3 - the simple mission. Run ON THE PI, in ~/victimbot (it uses victimbot.py's camera, detection, Uno link and
dashboard).

THE SCENE: from the start square (the SAFE SPOT) the camera sees the FIRE (the big red Nike box) and THREE VICTIMS (toy
DC motors standing upright) in front of it, at different distances from it. Everything is inside the camera's view, so
the robot never wanders or scans. For each victim, CLOSEST TO THE FIRE FIRST:
    1. line up: small turns until the victim is in the middle of the picture (the camera checks after every turn),
    2. approach: short hops forward, re-centring in between; the sonar, pointing straight ahead, gives the range when
       it agrees with the camera,
    3. final push: once the victim reaches the magnet's edge of the picture, drive the last few cm onto the magnet,
    4. check: back off a little and look - a victim still standing on the floor ahead was missed: one more try,
    5. return: replay its own moves backwards to the start square, stop 5 s (take the victim off the magnet).

    python3 -u rescue3.py            # THE MISSION (5 s countdown)
    python3 -u rescue3.py --look     # look and decide only: nothing moves
Dashboard: http://pi.local:8090 (live camera, map, actions, thinking).
"""
import argparse
import math
import signal
import statistics
import time

import victimbot as vb

FORWARD = vb.drive_powers(vb.CRUISE, vb.CRUISE)
BACKWARD = vb.drive_powers(-vb.CRUISE, -vb.CRUISE)
LOOK_FRAMES = 6                          # photos per look from the start square
FRONT_MARGIN_PX = 60                     # a victim's base may be this far above the fire's base in the picture (roll)
MAX_VICTIMS = 3                          # there are always three
ALIGN_TOL_DEG = 4.0                      # "in the middle of the picture"
ALIGN_PULSE_S = 0.65                     # first turn pulse: ~0.6 s of the motors doing nothing, then the robot turns
PULSE_LIMITS = (0.5, 1.1)
MAX_ALIGN_TRIES = 15
DEAD_S = 0.5                             # a drive command moves nothing for about this long (first guess, learned)
DEAD_MAX_S = 2.0
STUCK_M = 0.03                           # a hop that shrank the range by less than this did not move the car
SPEED_MPS = 0.15                         # first guess at the forward speed once moving; re-learned from every hop
HOP_M = 0.12                             # longest hop between looks
TRACK_GATE_DEG = 15                      # the victim being approached stays within this of where it was
CLOSE_ROW = int(vb.H * 0.80)             # its floor contact this low in the picture: right in front of the magnet
LOST_NEAR_M = 0.35                       # lost from view while nearer than this = it went under the magnet's edge
MAGNET_REACH_M = vb.MAGNET_REACH_M       # how far ahead of the lens the magnet's face is (TODO(measure))
PUSH_M = 0.03                            # drive this far past "just touching"
STICK_CM = 4                             # the sonar reads this close: something is on the magnet - stop pushing
SONAR_AGREE_M = 0.12                     # the sonar's range counts as the victim's if within this of the camera's
MAX_HOPS = 30
MAX_TRIES = 5                            # approaches on one victim before going back to look again
SAFE_STOP_S = 5


class Mission:
    def __init__(self, robot, cam):
        self.robot, self.cam = robot, cam
        self.moves = []                  # [('turn', left, seconds) | ('drive', seconds)] since leaving the start square
        self.pulse = {True: ALIGN_PULSE_S, False: ALIGN_PULSE_S}   # turn pulse length per direction, learned
        self.speed = SPEED_MPS
        self.dead = DEAD_S
        self.x = self.y = self.th = 0.0  # rough pose for the map: turns from the victim's bearing, drives from range
        self.fire = None
        self.saved = []

    # ---- seeing --------------------------------------------------------------------------------------------------
    def frames(self, n):
        vb.hold(self.robot, vb.STOP, vb.SETTLE_S, vb.DASH.state)
        return [self.cam.snap(time.time()) for _ in range(n)]

    def survey(self):
        """From the start square: the fire and up to three victims (map frame = the start pose), ranked closest to the
        fire first. The camera's horizon (so every camera distance) comes from the SONAR pointed at the fire."""
        photos = self.frames(LOOK_FRAMES)
        fire_blobs = [max((b for b in blobs if b.kind == 'fire'), key=lambda b: b.w * b.h, default=None) for blobs in photos]
        fire_blobs = [b for b in fire_blobs if b is not None]
        fire_seen = len(fire_blobs) * 2 > len(photos)
        if fire_seen:
            base = statistics.median(b.bottom for b in fire_blobs)
            # the victims stand IN FRONT of the fire: their base is lower in the picture than the box's (with a margin
            # for the camera's slight roll). Tape on boxes, shoes against the wall, anything behind it: not victims
            photos = [[b for b in blobs if b.kind == 'fire' or b.bottom >= base - FRONT_MARGIN_PX] for blobs in photos]
        # the victims' size always sets the horizon (the camera mount tilts; the stored row goes stale). The sonar then
        # refines it IF it hears the box clearly - on the low box it mostly gets no echo (2026-10-28: 1 ping in 5)
        vb.update_horizon(photos)
        if fire_seen:
            self.sonar_horizon(base, statistics.median(b.bearing for b in fire_blobs))
        # a far motor is tiny (~8 px) and is found in only some photos: a third of them is enough (there are only three)
        fires, victims = vb.build_scene(photos, (0.0, 0.0, 0.0), need=max(2, len(photos) // 3))
        # THERE IS ONE FIRE: the one seen at the very first look. Later looks keep it (a red blob elsewhere is not it)
        if self.fire is not None:
            fires = [f for f in fires if math.hypot(f['x'] - self.fire['x'], f['y'] - self.fire['y']) < 0.5] \
                or [self.fire]
        victims = sorted(victims, key=lambda v: (-v['n'], -v['score']))[:MAX_VICTIMS]   # the three surest
        self.fire = max(fires, key=lambda f: math.hypot(f['a'][0] - f['b'][0], f['a'][1] - f['b'][1]),
                        default=self.fire)
        if self.fire:
            for v in victims:
                v['d'] = vb.seg_dist(self.fire['a'], self.fire['b'], (v['x'], v['y']))
            victims.sort(key=lambda v: v['d'])
        else:
            victims.sort(key=lambda v: v['r'])
        return victims

    def sonar_horizon(self, base_row, bearing):
        """Point the sonar at the fire (servo), take its range: with the box's base row in the picture, that fixes the
        horizon row, so every distance the camera gives."""
        self.robot.look(vb.servo_deg(math.degrees(bearing)))
        pings = [cm for cm in (self.robot.ping() for _ in range(5)) if cm is not None and 30 <= cm <= 300]
        self.robot.look(vb.SONAR_CENTER_DEG)
        if len(pings) < 3:
            vb.DASH.think('The sonar did not hear the fire clearly; I estimate distances from the victims\' size.')
            return
        r = statistics.median(pings) / 100
        cam_r = vb.floor_range(base_row)
        if cam_r is not None and abs(r - cam_r) > 0.3 * cam_r:      # an echo off the wall/table behind, not the box
            vb.DASH.think(f'The sonar heard {r * 100:.0f} cm but the camera puts the fire at {cam_r * 100:.0f} cm; '
                          f'I keep the camera\'s distances.')
            return
        vb.HORIZON_Y = base_row - vb.CAM_HEIGHT_M * vb.F_PX / r
        vb.DASH.think(f'Sonar pointed at the fire: {r * 100:.0f} cm. Its base is at picture row {base_row:.0f}, so the '
                      f'camera\'s distances are scaled to match (horizon row {vb.HORIZON_Y:.0f}).')

    def find(self, bearing, max_range, n=2, gate_deg=TRACK_GATE_DEG):
        """The victim nearest `bearing` (rad) within TRACK_GATE_DEG and not further than max_range, over n photos:
        (bearing, range, contact row), medians; None if it is in fewer than half of them."""
        hits = []
        for blobs in self.frames(n):
            near = [b for b in blobs if b.kind == 'victim' and b.range is not None and b.range <= max_range
                    and abs(b.bearing - bearing) < math.radians(gate_deg)]
            if near:
                hits.append(min(near, key=lambda b: abs(b.bearing - bearing)))
        if len(hits) * 2 < n:
            return None
        return (statistics.median(b.bearing for b in hits), statistics.median(b.range for b in hits),
                statistics.median(b.bottom for b in hits))

    def sonar_range(self, cam_range):
        """The sonar's range straight ahead (m) if it agrees with the camera's, else None."""
        pings = [cm for cm in (self.robot.ping() for _ in range(3)) if cm is not None and 2 <= cm < 400]
        if len(pings) < 2:
            return None
        r = statistics.median(pings) / 100
        return r if abs(r - cam_range) <= SONAR_AGREE_M else None

    # ---- moving --------------------------------------------------------------------------------------------------
    def turn(self, left, seconds):
        vb.hold(self.robot, vb.SPINS[left], seconds, f'turning {"left" if left else "right"}')
        vb.hold(self.robot, vb.STOP, 0.3, 'stopped')
        self.moves.append(('turn', left, seconds))

    def drive(self, seconds, forward=True, log=True, stop_cm=None):
        """Drive for `seconds`; with stop_cm, stop early when the sonar reads closer than that."""
        end = time.time() + seconds
        while time.time() < end and not vb.STOP_EVENT.is_set():
            self.robot.drive(FORWARD if forward else BACKWARD)
            if stop_cm is not None:
                cm = self.robot.ping()
                if cm is not None and cm < stop_cm:
                    vb.DASH.say(f'Sonar: something {cm} cm ahead - it is on the magnet')
                    break
            else:
                time.sleep(vb.CYCLE_S)
        seconds -= max(0.0, end - time.time())
        vb.hold(self.robot, vb.STOP, 0.3, 'stopped')
        if log:
            self.moves.append(('drive', seconds if forward else -seconds))
        moved = (1 if forward else -1) * self.speed * max(0.0, seconds - self.dead)
        self.x += moved * math.cos(self.th)
        self.y += moved * math.sin(self.th)
        vb.DASH.moved(self.x, self.y, self.th)
        return seconds

    def reacquire(self, name, bearing, rng):
        """The victim was not where expected: look wider, then search with small turns either way (back to where it
        started each time). (bearing, range, row) when found again, else None."""
        seen = self.find(bearing, rng * 1.6 + 0.5, n=3, gate_deg=35)
        for left in (bearing > 0, bearing <= 0, bearing > 0, bearing <= 0):
            if seen is not None or vb.STOP_EVENT.is_set():
                return seen
            vb.DASH.say(f'Searching for victim {name}: a small turn {"left" if left else "right"}')
            self.turn(left, self.pulse[left])
            seen = self.find(0.0, rng * 1.6 + 0.5, n=3, gate_deg=35)
        return seen

    def align(self, name, bearing, rng):
        """Turn until the victim is within ALIGN_TOL_DEG of the picture's centre. The turn pulse per direction is
        learned: lengthened when a pulse did not move the victim in the picture, shortened when it overshot.
        Returns (bearing, range, row) of the victim when lined up, or None if it was lost."""
        seen = self.find(bearing, rng * 1.6 + 0.3)
        for _ in range(MAX_ALIGN_TRIES):
            if seen is None:
                seen = self.reacquire(name, bearing, rng)
            if seen is None or vb.STOP_EVENT.is_set():
                return None
            b, r, row = seen
            if abs(math.degrees(b)) <= ALIGN_TOL_DEG:
                return seen
            left = b > 0                                            # victim to the left: turn left
            vb.DASH.state = f'lining up victim {name}: {math.degrees(b):+.0f} deg off centre'
            self.turn(left, self.pulse[left])
            after = self.find(b, r * 1.6 + 0.3)                     # look near where it was (a turn moves it a little)
            if after is None:
                after = self.find(0.0, r * 1.6 + 0.3, n=3)          # ...or anywhere near the middle
            if after is not None:
                moved = abs(math.degrees(after[0] - b))
                if moved < 1.0:                                     # the turn did nothing: push longer next time
                    self.pulse[left] = min(PULSE_LIMITS[1], self.pulse[left] + 0.05)
                elif (after[0] > 0) != left and abs(math.degrees(after[0])) > ALIGN_TOL_DEG:   # overshot
                    self.pulse[left] = max(PULSE_LIMITS[0], self.pulse[left] - 0.04)
                self.th -= after[0] - b                             # the victim slides the opposite way to the turn
            seen = after
        return seen if seen and abs(math.degrees(seen[0])) <= ALIGN_TOL_DEG * 2 else None

    # ---- one rescue ----------------------------------------------------------------------------------------------
    def rescue(self, name, victim):
        """Line up, approach in hops, push onto the magnet, check. True if the victim is on the magnet."""
        bearing, rng = victim['bearing'], victim['r']
        for _ in range(MAX_HOPS):
            seen = self.align(name, bearing, rng)
            if seen is None:
                if rng <= LOST_NEAR_M:                              # it went under the magnet's edge of the picture
                    return self.push(name, rng)
                vb.DASH.say(f'Lost sight of victim {name}')
                return False
            bearing, rng, row = seen
            son = self.sonar_range(rng)
            if son is not None:
                vb.DASH.think(f'Victim {name}: camera {rng * 100:.0f} cm, sonar {son * 100:.0f} cm - using the sonar.')
                rng = son
            if row >= CLOSE_ROW or rng <= MAGNET_REACH_M + 0.08:
                return self.push(name, rng)
            hop = min(HOP_M, 0.6 * (rng - MAGNET_REACH_M))
            vb.DASH.say(f'Victim {name} centred, {rng * 100:.0f} cm away: hop {hop * 100:.0f} cm')
            t = self.drive(self.dead + hop / self.speed)
            after = self.find(0.0, rng * 1.6 + 0.3)
            if after is not None and t > self.dead + 0.1 and rng - after[1] < STUCK_M:
                # the hop moved nothing: on weak batteries the car needs longer to get rolling (2026-10-28: 1.3 s
                # hops never moved it, 2 s did). Learn the dead time instead of hopping on the spot forever
                self.dead = min(DEAD_MAX_S, self.dead + 0.25)
                vb.DASH.think(f'That hop did not bring victim {name} closer: the motors need longer to start. '
                              f'Dead time now {self.dead:.2f} s.')
            elif after is not None and t > self.dead + 0.1:                     # learn the real speed
                self.speed = max(0.03, min(0.6, 0.5 * self.speed + 0.5 * (rng - after[1]) / (t - self.dead)))
            if after is None:
                if rng <= LOST_NEAR_M:
                    return self.push(name, rng)
                after = self.reacquire(name, 0.0, rng)
                if after is None:
                    vb.DASH.say(f'Lost sight of victim {name}')
                    return False
            bearing, rng = after[0], after[1]
        return False

    def push(self, name, rng):
        dist = max(0.03, rng - MAGNET_REACH_M + PUSH_M)
        vb.DASH.say(f'Victim {name} right in front ({rng * 100:.0f} cm): pushing {dist * 100:.0f} cm onto the magnet')
        self.drive(self.dead + dist / self.speed, stop_cm=STICK_CM)
        return self.picked_up(name)

    def picked_up(self, name):
        """Back off a little and look: a victim still standing on the floor right ahead was NOT picked up."""
        self.drive(self.dead + 0.10 / self.speed, forward=False)
        left = self.find(0.0, 0.45, n=3)
        # a note right at the magnet is the victim ON it (it rides along); one clearly beyond it is still on the floor
        if left is not None and abs(math.degrees(left[0])) < 10 and left[1] > MAGNET_REACH_M + 0.10:
            vb.DASH.think(f'Victim {name} is still on the floor {left[1] * 100:.0f} cm ahead - the magnet missed it.')
            return False
        vb.DASH.think(f'Victim {name} is no longer on the floor ahead: it is on the magnet.')
        return True

    def go_home(self):
        """Replay every move since leaving the start square, backwards and reversed: back to the start square."""
        vb.DASH.say(f'Returning to the safe spot ({len(self.moves)} moves to undo)')
        for move in reversed(self.moves):
            if vb.STOP_EVENT.is_set():
                return
            if move[0] == 'turn':
                vb.hold(self.robot, vb.SPINS[not move[1]], move[2], 'returning: turning back')
                vb.hold(self.robot, vb.STOP, 0.3, 'returning')
            else:
                self.drive(abs(move[1]), forward=move[1] < 0, log=False)
        self.moves = []
        self.x = self.y = self.th = 0.0
        vb.DASH.moved(0.0, 0.0, 0.0)


def publish(m, victims, names, target=None):
    objs = []
    if m.fire:
        f = m.fire
        objs.append({'name': 'FIRE', 'kind': 'fire', 'x': f['x'], 'y': f['y'], 'rank': None, 'reached': False,
                     'ax': f['a'][0], 'ay': f['a'][1], 'bx': f['b'][0], 'by': f['b'][1]})
    for i, v in enumerate(victims):
        objs.append({'name': names[i], 'kind': 'victim', 'x': v['x'], 'y': v['y'], 'rank': i + 1, 'reached': False})
    for name, x, y in m.saved:
        objs.append({'name': name, 'kind': 'victim', 'x': x, 'y': y, 'rank': None, 'reached': True})
    vb.DASH.set(objs=objs, hazards=[], path=[], target=target)


def run(m, move=True):
    vb.DASH.say('Looking at the scene from the safe spot')
    names = iter('ABCDEFGHIJ')
    while not vb.STOP_EVENT.is_set():
        victims = m.survey()
        labels = [next(names) for _ in victims]
        publish(m, victims, labels)
        if not victims:
            vb.DASH.say(f'No victims left in front of me - all {len(m.saved)} saved. Mission complete.')
            vb.DASH.set(state='MISSION COMPLETE')
            return
        if m.fire:
            vb.DASH.think('Fire (the red box) seen. Victims, closest to the fire first:')
            for name, v in zip(labels, victims):
                vb.DASH.think(f'  {name}: {v["d"] * 100:.0f} cm from the fire, {v["r"] * 100:.0f} cm from me, '
                              f'{abs(math.degrees(v["bearing"])):.0f} deg {"left" if v["bearing"] > 0 else "right"}')
            vb.DASH.think(f'Victim {labels[0]} is closest to the fire - the most danger - so I rescue it first.')
        else:
            vb.DASH.think('I cannot see the fire, so I take the nearest victim first.')
        if not move:
            vb.DASH.set(state='look only - nothing moves')
            return
        name, target = labels[0], victims[0]
        vb.DASH.set(target=name)
        ok = False
        for attempt in range(MAX_TRIES):
            vb.DASH.say(f'Going for victim {name}' + (' (second try)' if attempt else ''))
            ok = m.rescue(name, target)
            if ok or vb.STOP_EVENT.is_set():
                break
            vb.DASH.say(f'Victim {name} is not on the magnet yet - lining up again from here')
            target = dict(target, bearing=0.0, r=0.4)               # try again from where it now is: just ahead
        if ok:
            vb.DASH.say(f'Victim {name} PICKED UP by the magnet')
            m.saved.append((name, target['x'], target['y']))
        m.go_home()
        if ok:
            vb.DASH.say(f'Victim {name} SAVED. Stopping {SAFE_STOP_S} s - take it off the magnet now.')
            for n in range(SAFE_STOP_S, 0, -1):
                vb.hold(m.robot, vb.STOP, 1.0, f'SAFE SPOT: victim {name} saved - stopped, {n} s')
        else:
            vb.DASH.say(f'Could not pick up victim {name} - back at the safe spot, looking again.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--look', action='store_true', help='look and decide only: nothing moves')
    args = ap.parse_args()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: vb.STOP_EVENT.set())
    robot = None
    try:
        vb.CAMERA = cam = vb.Camera()
        cam.start()
        robot = vb.Uno(dry=args.look)
        robot.open()
        robot.look(vb.SONAR_CENTER_DEG)                             # sonar straight ahead, and it stays there
        if not args.look:
            for n in range(5, 0, -1):
                vb.DASH.set(state=f'starting in {n} s - Ctrl+C to cancel')
                print(f'Moving in {n} s (Ctrl+C to cancel)...', flush=True)
                if vb.STOP_EVENT.wait(1):
                    return
        vb.STOP_EVENT.wait(1.5)                                     # let the camera settle
        run(Mission(robot, cam), move=not args.look)
        print('Done. Ctrl+C to exit (the dashboard stays up until then).', flush=True)
        vb.STOP_EVENT.wait()
    except KeyboardInterrupt:
        pass
    finally:
        vb.STOP_EVENT.set()
        if robot:
            robot.close()
        print('Stopped.')


if __name__ == '__main__':
    main()
