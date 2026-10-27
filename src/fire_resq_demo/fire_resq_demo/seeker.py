"""The bottle-seeking behaviour: remember every bottle seen, visit the nearest unvisited one, wait, go on.
Pure and deterministic (no ROS, no clock of its own, no OpenCV): `on_frame` takes what the camera saw,
`command` returns the velocity to send, and both are told the time.

    SEARCH   turn on the spot in steps, looking, until an unvisited bottle is in view or a full turn found nothing
    RETURN   a remembered bottle was not seen in a full turn (something hides it): drive to where it was, look again
    TURN     short turn pulses until the chosen bottle is centred
    APPROACH drive at it, steering to keep it centred; a nearer unvisited bottle takes over
    AVOID    something that is not the target is in the way: turn until the way is clear, pass it, look again
    WAIT     arrived: stand still for `wait_s`, then the bottle counts as visited
    DONE     nothing unvisited is left and a full turn showed nothing new

Built for THIS robot, which is why it looks the way it does:
  - Bottles are told apart by COLOUR, not position. The robot's sense of its own position is dead reckoning from
    motor power (no encoders); colour does not drift. So every bottle must be a different colour.
  - Turning is done in short pulses with a look in between ("turn, stop, look"), never by watching the bottle
    while turning. The picture arrives 0.1-0.3 s late and blurred during a turn, and nobody has measured how
    fast this robot turns: the first pulses are a guess, every later one is scaled by what the last one did.
  - A pulse that moves the bottle the WRONG way twice means left and right are swapped in the wiring config;
    the turn direction is flipped (and reported) rather than spinning away for the whole demo.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

from .colours import colour_name, hue_distance

IDLE, SEARCH, TURN, APPROACH, AVOID, WAIT, BACKUP, RETURN, WANDER, DONE = (
    'IDLE', 'SEARCH', 'TURN', 'APPROACH', 'AVOID', 'WAIT', 'BACKUP', 'RETURN', 'WANDER', 'DONE')


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class SeekerParams:
    wait_s: float = 5.0
    expected_bottles: int = 0           # stop as soon as this many are visited; 0 = until a full turn finds nothing new
    # driving (commanded values: what they mean on the floor depends on robot.yaml's uncalibrated numbers)
    drive_speed: float = 0.12           # m/s on the approach
    turn_speed: float = 1.2             # rad/s during a turn pulse
    steer_speed: float = 0.6            # most rad/s added while driving, to keep the bottle centred
    steer_gain: float = 2.5             # rad/s of steering per radian the bottle is off centre
    track_m: float = 0.17               # left-right wheel spacing: caps the steering so the inner side never reverses
    left_boost: float = 2.08            # every turn to the left / to the right is commanded this much harder
    right_boost: float = 2.08           # (found on the floor: this robot needs nearly full power to turn at all)
    # arriving
    arrive_cover_frac: float = 0.70     # ARRIVED = the object covers this much of the whole picture ...
    floor_left_frac: float = 0.40       # ... AND no more than this much of the floor is still in view. The maintainer's
                                        # rule (2026-10-13): "if I can see the white floor the robot has space to move
                                        # ahead". No distance in metres is used: the camera's guessed geometry called
                                        # far objects near, twice
    creep_cover_frac: float = 0.30      # the target covered this much and then vanished (it fills the view, or the
    creep_max_s: float = 2.5            # exposure changed): keep driving straight until the floor is hidden, this long at most
    final_push_s: float = 0.0           # then drive straight on, blind, for this long (0 = don't)
    close_width_frac: float = 0.30      # "next to it": this wide or covering `close_cover_frac`. From then on a
    close_cover_frac: float = 0.15      # half-seen target is good enough to steer at
    floor_gone_frac: float = 0.15       # next to the target and this little floor left in view = arrived as well
    # wandering (nothing to go to: drive about until something turns up). Asked for on 2026-10-13 in place of
    # turning on the spot to search: this robot turns badly (to the right, hardly at all) and drives forward well.
    wander_leg_s: float = 7.0           # drive straight this long, then turn a little to look somewhere else
    wander_turn_s: float = 0.35         # how long that turn (and the turn away from something in the way) lasts
    wander_floor_min: float = 0.20      # less floor than this in view = something is right in front: turn away
    leave_turn_s: float = 0.45          # after a visit: back off, then turn away for this long
    left_turns_only: bool = True        # every turn on the spot the robot chooses itself is to the LEFT (its good side)
    # being stuck (the picture does not change while the robot is told to move)
    stuck_s: float = 2.0                # driving this long, twice in a row, without the picture changing = stuck
    escape_s: float = 0.6               # how long each "try something else" move lasts
    slow_range_m: float = 0.45          # below this, drive at `slow_factor`
    slow_factor: float = 0.7
    backup_s: float = 0.6               # reverse this long after a visit, to see round the bottle (0 = don't)
    # turning
    align_in_deg: float = 8.0           # start driving when the bottle is within this of straight ahead
    align_out_deg: float = 16.0         # stop and re-aim when it drifts beyond this
    steer_deadband_deg: float = 3.0
    turn_rate_guess: float = 1.5        # rad/s the robot really turns at turn_speed: a first guess, then learned
    min_pulse_s: float = 0.10
    max_pulse_s: float = 0.70
    settle_s: float = 0.40              # stand still this long after a pulse before believing the picture ...
    look_s: float = 0.25                # ... then look for this long before deciding
    test_pulse_s: float = 0.12          # the first pulse of a run, made only to measure how fast the robot turns
    search_step_deg: float = 20.0       # under half the camera's 45 degree view: no gaps if the estimate is 2x off
    search_turn_deg: float = 400.0      # a "full turn" with margin, because the turn rate is an estimate ...
    search_max_deg: float = 900.0       # ... and the most to turn while a landmark says the circle is not closed
    still_diff: float = 1.0             # a turn pulse that changes the (tiny, grey) picture by less than this did nothing
    still_detail: float = 2.0           # ... IF the picture has this much left-right detail. A blank wall looks the same
                                        # from every angle and proves nothing: then nothing is concluded
    still_max_scale: float = 2.0        # the most turn power the picture alone may ask for (a bottle in view: 4x)
    max_search_pulses: int = 80         # however that goes, a search ends after this many pulses
    # memory
    confirm_frames: int = 3
    hue_tol: float = 10.0               # OpenCV hue units (x2 = degrees): closer than this is the same bottle
    fresh_s: float = 0.6                # a bottle seen within this long is "in view"
    lost_grace_s: float = 1.2
    switch_ratio: float = 0.75          # change target only for a bottle this much closer
    max_attempts: int = 4
    return_trips: int = 1               # times to drive to where an unseen bottle was remembered before giving it up
    return_max_s: float = 20.0          # ... for at most this long, stopping this far short of the remembered spot:
    return_short_m: float = 0.35
    # obstacles
    avoid: bool = True
    obstacle_range_m: float = 0.30
    half_width_m: float = 0.12
    avoid_forward_s: float = 3.0        # drive this long past an obstacle before turning back to the bottle
    max_avoid_turns: int = 8
    # IR obstacle sensors (2026-10-16): "rely on these" for avoidance and for arrival. Two of them, pointing
    # roughly forward-left and forward-right, each a plain "something is close" switch with no distance and no
    # notion of colour, so they say nothing about WHICH object is close - that is still the camera's job (finding
    # and steering at the right-coloured bottle). The camera's own obstacle/distance heuristics (floor colour,
    # picture coverage) are kept only as a fallback for while the sensors are not reporting (`ir_fresh_s`), so a
    # dead IR topic degrades the robot rather than blinding it.
    ir_fresh_s: float = 1.0             # an IR reading older than this is ignored: fall back to the camera
    # limits
    stale_s: float = 0.8                # no camera frame for this long = stand still
    max_mission_s: float = 600.0
    celebrate: bool = True


@dataclass
class Sighting:
    """One accepted bottle in one frame. Bearings in radians, positive left; `left`/`right` are its edges."""
    hue: Optional[float]
    bearing: float
    range_m: float
    width_frac: float
    left: float = 0.0
    right: float = 0.0
    partial: bool = False               # runs off the side of the picture: bearing and width are rough
    cover_frac: float = 0.0             # how much of the whole picture it covers (its pixels / all pixels)


@dataclass
class Bottle:
    id: int
    hue: Optional[float]
    name: str
    x: float = 0.0
    y: float = 0.0
    bearing: float = 0.0
    range_m: float = math.inf
    width_frac: float = 0.0
    cover_frac: float = 0.0
    left: float = 0.0
    right: float = 0.0
    seen_t: float = -math.inf
    hits: int = 0
    confirmed: bool = False
    visited: bool = False
    lost: bool = False
    attempts: int = 0
    trips: int = 0


class Seeker:
    def __init__(self, params: SeekerParams, tracker):
        self.p, self.tracker = params, tracker
        self.events: List[Tuple[int, str]] = []
        self._event_id = 0
        self.reset()

    # ------------------------------------------------------------------ operator
    def reset(self) -> None:
        """Forget every bottle and stand still."""
        self.state, self.note = IDLE, ''
        self.bottles: List[Bottle] = []
        self.target: Optional[Bottle] = None
        self.wait_until = 0.0
        self.t_start = 0.0
        self.frame_t = -math.inf
        self.scan: Optional[np.ndarray] = None                   # rows of (x forward, y left, bearing); x NaN = clear
        self.look: Optional[np.ndarray] = None                   # the latest picture, tiny and grey: "did the view change?"
        self.floor_visible: Optional[float] = None               # how much of the floor part of the picture shows floor
        self._before: Optional[np.ndarray] = None                # ... and the one from just before the last turn pulse
        self._pulses = 0
        self.turn_rate = self.p.turn_rate_guess
        self.turn_scale, self.turn_sign = 1.0, 1
        self._moves: Deque[Tuple[float, float, float, float]] = deque()
        self._cmd, self._pulse_start, self._move_end, self._settle_end = (0.0, 0.0), 0.0, -math.inf, -math.inf
        self._need_fresh = False
        self._pulse = None                                       # (bottle, bearing before, pulse s, dir, raw yaw before, aiming)
        self._undo = None                                        # (dir, seconds) of an aiming pulse that lost the bottle
        self._turn_samples, self._tested = 0, False
        self._aim_scale = 1.0                                    # < 1 after an aiming pulse lost the bottle: aim more gently
        self._stretch = 1.0                                      # > 1 after a pulse moved nothing: longer pulses
        self._search_dir: Optional[int] = None
        self._last_dir = 1
        self._sweep = None                                       # the turn in progress: dir, turned, start, landmark
        self._sweep_done = False
        self._avoid = None
        self._trip = None                                        # driving to where a bottle was: bottle, turns, until
        self._priority: Optional[Bottle] = None                  # the object the operator clicked: goes first
        self._drive_ref = None                                   # (time, picture, strikes) while driving at a target
        self._wander_from: Optional[float] = None                # when the current straight leg of wandering began
        self._wander_turns, self._wander_push_until = 0, -math.inf
        self._stalls, self._escapes, self._want_escape = 0, 0, False
        self._ir = (False, False)                                # (left, right) triggered, from the latest IR reading
        self._ir_t = -math.inf
        self._next_id = 1

    def start(self, t: float) -> None:
        self.reset()
        self.state, self.t_start = SEARCH, t
        self._say('Looking for bottles')

    def pick(self, t: float, hue: Optional[float], sighting: Optional['Sighting'] = None) -> None:
        """The operator clicked an object: it is a target whatever the detector thinks of it, and it goes first.
        Starts a run if none is going (without forgetting what is already known)."""
        b = next((b for b in self.bottles if hue_distance(b.hue, hue) <= self.p.hue_tol), None)
        if b is None:
            b = Bottle(self._next_id, hue, colour_name(hue))
            self._next_id += 1
            self.bottles.append(b)
        b.confirmed, b.visited, b.lost, b.attempts, b.trips = True, False, False, 0, 0
        b.hits = max(b.hits, self.p.confirm_frames)
        if sighting is not None:
            x, y, yaw = self.tracker.pose
            b.bearing, b.range_m, b.width_frac, b.left, b.right = (sighting.bearing, sighting.range_m, sighting.width_frac,
                                                                    sighting.left, sighting.right)
            b.seen_t, b.cover_frac = t, sighting.cover_frac
            reach = sighting.range_m if math.isfinite(sighting.range_m) else 2.0
            b.x, b.y = x + reach * math.cos(yaw + sighting.bearing), y + reach * math.sin(yaw + sighting.bearing)
        self._priority, self._sweep, self._sweep_done, self._trip = b, None, False, None
        if not self.running:
            self._moves.clear()
            self.state, self.t_start = SEARCH, t
        self._say(f'You picked the {b.name} object: going there first')

    def knows(self, hue: Optional[float]) -> bool:
        """Is this the colour of an object already accepted (so a blob of it need not pass the shape rules again)?"""
        return any(b.confirmed and hue_distance(b.hue, hue) <= self.p.hue_tol for b in self.bottles)

    def stop(self) -> None:
        if self.state not in (IDLE, DONE):
            self._say('Stopped')
        self.state, self.target = IDLE, None
        self._moves.clear()
        self._move_end = self._settle_end = -math.inf

    @property
    def running(self) -> bool:
        return self.state not in (IDLE, DONE)

    @property
    def moving(self) -> bool:
        """Running, or still playing out its last queued moves."""
        return self.running or bool(self._moves)

    def _say(self, text: str) -> None:
        self._event_id += 1
        self.events.append((self._event_id, text))
        del self.events[:-30]

    # ------------------------------------------------------------------ camera in
    def on_frame(self, t: float, sightings: List[Sighting], scan: Optional[np.ndarray],
                 look: Optional[np.ndarray] = None, floor_visible: Optional[float] = None) -> None:
        p = self.p
        self.frame_t, self.scan, self.look, self.floor_visible = t, scan, look, floor_visible
        turning = not self.steady(t)
        x, y, yaw = self.tracker.pose
        matched = set()
        full = [s for s in sightings if not s.partial]
        target = self.target
        if target is not None and self._close(target) and not any(hue_distance(target.hue, s.hue) <= p.hue_tol for s in full):
            half = [s for s in sightings if s.partial and hue_distance(target.hue, s.hue) <= p.hue_tol]
            if half:                                             # next to the target and it runs off the picture: keep
                s = max(half, key=lambda s: s.width_frac)        # steering at what is in view of it
                target.bearing, target.width_frac, target.seen_t = s.bearing, max(s.width_frac, target.width_frac), t
                target.cover_frac = max(s.cover_frac, target.cover_frac)
                target.left, target.right = s.left, s.right
                matched.add(target.id)
        for s in sorted(full, key=lambda s: s.range_m):
            near = [b for b in self.bottles if b.id not in matched and hue_distance(b.hue, s.hue) <= p.hue_tol]
            if any(hue_distance(b.hue, s.hue) <= p.hue_tol for b in self.bottles if b.id in matched):
                continue                                         # a second blob of a colour already matched: ignored
            if near:
                b = min(near, key=lambda b: hue_distance(b.hue, s.hue))
            else:
                b = Bottle(self._next_id, s.hue, colour_name(s.hue))
                self._next_id += 1
                self.bottles.append(b)
            matched.add(b.id)
            if b.hue is not None and b.hue >= 0:                 # follow slow white-balance drift
                b.hue = (b.hue + 0.2 * (((s.hue - b.hue + 90.0) % 180.0) - 90.0)) % 180.0
            b.bearing, b.range_m, b.width_frac, b.left, b.right = s.bearing, s.range_m, s.width_frac, s.left, s.right
            b.seen_t, b.hits, b.cover_frac = t, b.hits + 1, s.cover_frac
            if not turning and math.isfinite(s.range_m) and s.range_m < 4.0:
                bx = x + s.range_m * math.cos(yaw + s.bearing)
                by = y + s.range_m * math.sin(yaw + s.bearing)
                a = 1.0 if b.hits == 1 else 0.35
                b.x, b.y = b.x + a * (bx - b.x), b.y + a * (by - b.y)
            if b.lost:
                b.lost = False
                self._say(f'Found the {b.name} bottle again')
            if not b.confirmed and b.hits >= p.confirm_frames:
                b.confirmed = True
                same = sum(1 for o in self.bottles if o.confirmed and o.name.split(' ')[0] == b.name) - 1
                if same:
                    b.name = f'{b.name} {same + 1}'
                self._say(f"Found {'an' if b.name[0] in 'aeiou' else 'a'} {b.name} bottle")
        self.bottles = [b for b in self.bottles if b.confirmed or t - b.seen_t <= 2.0]

    def on_ir(self, t: float, left: bool, right: bool) -> None:
        """A new reading from the two IR obstacle sensors. Comes on its own topics, independent of camera frames,
        so it can be fed in faster than the ~7-8 fps the camera manages."""
        self._ir, self._ir_t = (bool(left), bool(right)), t

    def _ir_fresh(self, t: float) -> bool:
        return t - self._ir_t <= self.p.ir_fresh_s

    def steady(self, t: float) -> bool:
        """Not turning, and not just after a turn: pictures taken now can be trusted for positions."""
        return t >= self._settle_end or self._cmd[1] == 0.0

    def _close(self, b: Bottle) -> bool:
        return b.width_frac >= self.p.close_width_frac or b.cover_frac >= self.p.close_cover_frac

    def _floor_hidden(self) -> bool:
        """Is the floor (mostly) out of view? Unknown counts as hidden, so it never blocks the cover rule."""
        return self.floor_visible is None or self.floor_visible <= self.p.floor_left_frac

    def _there(self, b: Bottle) -> bool:
        """Reached: the object covers the picture and the floor in front has all but disappeared behind it."""
        return b.cover_frac >= self.p.arrive_cover_frac and self._floor_hidden()

    def _same_picture(self, was: Optional[np.ndarray], now: Optional[np.ndarray], across_only: bool) -> bool:
        """True when the picture has not changed AND has enough detail for that to mean something. A turn can only
        be judged by left-right detail (`across_only`); driving forward, by any detail."""
        if was is None or now is None or was.ndim != 2 or was.shape != now.shape:
            return False
        a = was.astype(np.float32)
        change = now.astype(np.float32) - a
        detail = float(np.abs(np.diff(a, axis=1)).mean())
        if not across_only:
            detail = max(detail, float(np.abs(np.diff(a, axis=0)).mean()))
        return detail >= self.p.still_detail and float(np.abs(change - change.mean()).mean()) < self.p.still_diff

    def _escape(self, t: float) -> Tuple[float, float]:
        """Stuck: the robot is being told to move and the picture is not changing. Try something else, a different
        thing each time: push ahead harder, back up, turn one way, turn the other."""
        p = self.p
        turn = p.turn_speed * self.turn_scale
        moves = (('pushing ahead harder', min(1.6 * p.drive_speed, 0.30), 0.0),
                 ('backing up to try again', -min(1.6 * p.drive_speed, 0.30), 0.0),
                 ('turning left to get free', 0.0, turn), ('turning right to get free', 0.0, -turn))
        name, v, w = moves[self._escapes % len(moves)]
        self._escapes, self._stalls, self._want_escape = self._escapes + 1, 0, False
        self._say(f'Stuck: {name}')
        return self._begin(t, v, w, p.escape_s, p.settle_s)

    def accepts_partial(self, hue: Optional[float]) -> bool:
        """Would a half-visible blob of this colour be used? (Only for the target, and only when next to it.)"""
        b = self.target
        return b is not None and self._close(b) and hue_distance(b.hue, hue) <= self.p.hue_tol

    def _fresh(self, b: Bottle, t: float) -> bool:
        """In view now: seen recently, and in a picture taken after the last turn pulse had stopped."""
        return t - b.seen_t <= self.p.fresh_s and b.seen_t >= self._settle_end

    # ------------------------------------------------------------------ velocity out
    def command(self, t: float) -> Tuple[float, float]:
        """(linear m/s, angular rad/s) to send now. Call at >= 10 Hz."""
        if t < self._move_end:
            return self._cmd
        if t < self._settle_end:
            return 0.0, 0.0
        if self._moves:
            return self._begin(t, *self._moves.popleft())
        if not self.running:
            return 0.0, 0.0
        if t - self.frame_t > self.p.stale_s:
            self.note = 'no camera picture: standing still'
            return 0.0, 0.0
        if self._need_fresh and self.frame_t < self._settle_end + self.p.look_s:
            return 0.0, 0.0                                      # wait for pictures taken after the robot stopped
        self._need_fresh, self.note = False, ''
        return self._decide(t)

    def _turn(self, w: float) -> float:
        """The angular velocity to SEND for an intended one (positive = left): the left boost, then the wiring fix."""
        return w * (self.p.left_boost if w > 0 else self.p.right_boost) * self.turn_sign

    def _begin(self, t: float, v: float, w: float, seconds: float, settle: float) -> Tuple[float, float]:
        self._cmd = (v, self._turn(w))
        self._pulse_start, self._move_end, self._settle_end = t, t + seconds, t + seconds + settle
        self._drive_ref, self._wander_from = None, None
        self._need_fresh = True
        return self._cmd

    def _halt(self, t: float) -> Tuple[float, float]:
        """Stop and take a fresh look before the next decision (the robot was driving: the picture is behind)."""
        return self._begin(t, 0.0, 0.0, 0.0, self.p.settle_s)

    def _turn_pulse(self, t: float, direction: int, angle: float, aim: Optional[Bottle] = None,
                    seconds: Optional[float] = None) -> Tuple[float, float]:
        """Turn about `angle` radians (or exactly `seconds`), then stop and look. Any bottle in view before the
        pulse is a ruler for how far the robot really turned (`_learn_turn`)."""
        p = self.p
        if seconds is None:
            caution = self._aim_scale if aim is not None else 1.0
            seconds = min(max(caution * self._stretch * angle / self.turn_rate, p.min_pulse_s), p.max_pulse_s)
        ruler = aim or min((b for b in self.bottles if b.confirmed and self._fresh(b, t)),
                           key=lambda b: abs(b.bearing), default=None)
        if ruler is not None:
            self._pulse = (ruler, ruler.bearing, seconds, direction, self.tracker.raw_yaw, aim is not None)
        self._before, self._pulses = self.look, self._pulses + 1
        return self._begin(t, 0.0, direction * p.turn_speed * self.turn_scale, seconds, p.settle_s)

    def _decide(self, t: float) -> Tuple[float, float]:
        p = self.p
        if t - self.t_start > p.max_mission_s:
            return self._finish('Time limit reached')
        if self.state == WAIT:
            if t < self.wait_until:
                return 0.0, 0.0
            self.target.visited = True
            self._say(f'Done with the {self.target.name} bottle: moving on')
            self.target, self.state = None, BACKUP               # back off, turn away from it, wander on
            self._moves.append((0.0, p.turn_speed * self.turn_scale, p.leave_turn_s, p.settle_s))
            return self._begin(t, -p.drive_speed, 0.0, max(p.backup_s, 0.05), p.settle_s)
        if self.state == BACKUP:
            self.state = WANDER
        self._learn_turn(t)
        if self._want_escape:                                    # turn pulse after turn pulse moved nothing
            return self._escape(t)
        if self.state == AVOID:
            return self._avoid_step(t)
        if p.expected_bottles and sum(b.visited for b in self.bottles) >= p.expected_bottles:
            return self._finish()

        cur = self.target
        if cur is not None and self.state == APPROACH and not self._fresh(cur, t) and cur.cover_frac >= p.creep_cover_frac:
            # It filled a good part of the picture and then stopped being recognised (it fills the view, or the
            # camera re-exposed for it). It is straight ahead: keep going until the floor is hidden behind it.
            if (self.floor_visible is not None and self._floor_hidden()) or t - cur.seen_t > p.creep_max_s:
                return self._arrive(t, cur)
            return p.drive_speed * p.slow_factor, 0.0
        target = self._choose(t)
        if target is not self.target:
            if target is not None:
                closer = self.target is not None and not self.target.visited and self._fresh(self.target, t)
                self._say(f'A closer bottle: going to the {target.name} one' if closer
                          else f'Going to the {target.name} bottle')
            self.target = target
        if target is None:
            return self._wander(t)
        floor_gone = self.floor_visible is not None and self.floor_visible <= p.floor_gone_frac
        if self.state == APPROACH and self._ir_fresh(t):
            # Already driving straight at this target (camera-steered to stay centred): an IR sensor tripping now
            # is overwhelmingly the target itself, so the sensors alone decide "arrived" here (2026-10-13: "rely on
            # these"). Before APPROACH starts an IR trip means something ELSE is in the way (handled by _blocked
            # below), never this target, which has not been driven at yet.
            if any(self._ir):
                return self._arrive(t, target)
        elif self._there(target) or (self._close(target) and floor_gone):
            return self._arrive(t, target)
        # A target that is in view is DRIVEN at, steering on the way: never aimed at by turning on the spot first.
        # On this robot a turn on the spot is a full-power pivot that swings past the target and (pivoting on a
        # reversing side) backs away from it, so aiming pulses walked it backwards and it never set off (2026-10-13).
        if p.avoid and self._blocked(t, target, p.half_width_m):
            return self._start_avoid(t, target)
        self.state = APPROACH
        if self._going_nowhere(t):
            return self._escape(t)
        v = p.drive_speed * (p.slow_factor if self._close(target) else 1.0)
        steer = 0.0
        if abs(target.bearing) > math.radians(p.steer_deadband_deg):
            most = 2.0 * v / p.track_m                           # at this the inner side stands still: the tightest
            steer = max(-most, min(most, p.steer_gain * target.bearing)) * self.turn_sign    # curve that still goes forward
        return v, steer

    def _going_nowhere(self, t: float) -> bool:
        """Driving, and the picture has not changed over two checks in a row."""
        if self.look is None:
            return False
        if self._drive_ref is None:
            self._drive_ref = (t, self.look, 0)
        elif t - self._drive_ref[0] >= self.p.stuck_s:
            strikes = self._drive_ref[2] + 1 if self._same_picture(self._drive_ref[1], self.look, False) else 0
            if strikes >= 2:
                return True
            self._drive_ref = (t, self.look, strikes)
        return False

    def _wander(self, t: float) -> Tuple[float, float]:
        """Nothing in view to go to: drive about until something turns up. Straight ahead while the way is clear;
        a short turn (to the left, the side this robot turns well) when something is in the way, and every few
        seconds anyway so that it looks somewhere new."""
        p = self.p
        self.state = WANDER
        blocked = p.avoid and self._blocked(t, None, p.half_width_m + 0.03)
        if not self._ir_fresh(t):                                # the sensors are not reporting: fall back to the camera
            blocked = blocked or (self.floor_visible is not None and self.floor_visible < p.wander_floor_min)
        if blocked and t >= self._wander_push_until:
            self._wander_turns += 1
            if self._wander_turns <= 6:
                return self._turn_pulse(t, 1, 0.0, seconds=p.wander_turn_s)
            self._wander_turns, self._wander_push_until = 0, t + 2.0     # "blocked" whichever way it looks: go anyway
        if self._wander_from is None:
            self._wander_from = t
        elif t - self._wander_from > p.wander_leg_s:
            return self._turn_pulse(t, 1, 0.0, seconds=p.wander_turn_s)
        if not blocked:
            self._wander_turns = 0
        if self._going_nowhere(t):
            return self._escape(t)
        return p.drive_speed, 0.0

    def _arrive(self, t: float, target: Bottle) -> Tuple[float, float]:
        p = self.p
        self.state, self.wait_until = WAIT, t + p.final_push_s + p.wait_s
        x, y, yaw = self.tracker.pose
        target.x, target.y = x + 0.15 * math.cos(yaw), y + 0.15 * math.sin(yaw)     # it is just in front of the robot
        self._say(f'Reached the {target.name} bottle')
        if p.final_push_s > 0:
            return self._begin(t, p.drive_speed * p.slow_factor, 0.0, p.final_push_s, 0.0)
        return 0.0, 0.0

    # ------------------------------------------------------------------ choosing
    def _choose(self, t: float) -> Optional[Bottle]:
        x, y, _ = self.tracker.pose

        def cost(b: Bottle) -> float:
            if self._fresh(b, t):
                return b.range_m
            return math.hypot(b.x - x, b.y - y) + 0.3            # out of view: believed distance, a little distrusted

        # Only what is in view NOW: an object that is out of view is found again by wandering, not by trusting where
        # dead reckoning says it was.
        open_ = [b for b in self.bottles if b.confirmed and not b.visited and not b.lost and self._fresh(b, t)]
        if self._priority is not None:
            if self._priority in open_:
                return self._priority                            # the operator's pick comes before "nearest first"
            if self._priority.visited:
                self._priority = None
        if not open_:
            return None
        best = min(open_, key=cost)
        if self.target in open_ and cost(best) > self.p.switch_ratio * cost(self.target):
            return self.target
        return best

    # ------------------------------------------------------------------ searching
    def _search(self, t: float, target: Optional[Bottle]) -> Tuple[float, float]:
        """One step of turning on the spot to look. A sweep ends when the robot has come full circle: by the
        estimate, or sooner/later when a bottle that was in view at the start comes back into view."""
        p = self.p
        self.state = SEARCH
        if any(not b.confirmed and self._fresh(b, t) for b in self.bottles):
            return 0.0, 0.0                                      # something new is in view: look a little longer
        if self._search_dir is None:
            self._search_dir = self._last_dir
            if target is not None:
                x, y, yaw = self.tracker.pose
                self._search_dir = 1 if wrap(math.atan2(target.y - y, target.x - x) - yaw) >= 0 else -1
        self._last_dir = self._search_dir
        sw = self._sweep
        if sw is None or sw['dir'] != self._search_dir:
            mark = min((b for b in self.bottles if b.confirmed and self._fresh(b, t)),
                       key=lambda b: abs(b.bearing), default=None)
            sw = self._sweep = {'dir': self._search_dir, 'turned': 0.0, 'start': t, 'mark': mark, 'gone': False,
                                'pulses': self._pulses, 'last': 0.0}
        closed = False
        if sw['mark'] is not None:
            if not self._fresh(sw['mark'], t):
                sw['gone'] = True
            elif sw['gone'] and sw['turned'] > math.radians(120.0):
                closed = True                                    # the landmark is back: that was a full circle
        limit = p.search_max_deg if sw['mark'] is not None and sw['gone'] else p.search_turn_deg
        if closed or sw['turned'] >= math.radians(limit) or self._pulses - sw['pulses'] >= p.max_search_pulses:
            x, y, _ = self.tracker.pose
            for b in self.bottles:                               # not seen once in a whole circle
                if not (b.confirmed and not b.visited and not b.lost and b.seen_t < sw['start']):
                    continue
                far = math.hypot(b.x - x, b.y - y) > p.return_short_m + 0.15
                if b is target and far and b.trips < p.return_trips and self._trip is None:
                    b.trips += 1                                 # something may be hiding it: go and look from closer
                    self._trip = {'bottle': b, 'turns': 0, 'until': None}
                    self._say(f'Cannot see the {b.name} bottle from here: going to where it was')
                else:
                    b.lost = True
                    self._say(f'Cannot find the {b.name} bottle any more')
            self._sweep, self._sweep_done, self._search_dir = None, self._trip is None, None
            return 0.0, 0.0
        cmd = self._turn_pulse(t, self._search_dir, math.radians(p.search_step_deg))
        sw['last'] = self.turn_rate * (self._move_end - t)
        sw['turned'] += sw['last']
        return cmd

    def _return_step(self, t: float, target: Bottle) -> Tuple[float, float]:
        """Face where the bottle is remembered, drive most of the way there (obstacles still avoided), look again."""
        p, trip = self.p, self._trip
        x, y, yaw = self.tracker.pose
        if trip['until'] is None:
            self.state = RETURN
            off = wrap(math.atan2(target.y - y, target.x - x) - yaw)
            if abs(off) > math.radians(15.0) and trip['turns'] < 8:
                trip['turns'] += 1
                return self._turn_pulse(t, 1 if off > 0 else -1, 0.75 * abs(off))
            reach = math.hypot(target.x - x, target.y - y) - p.return_short_m
            trip['until'] = t + min(max(reach, 0.0) / p.drive_speed, p.return_max_s)
        if p.avoid and self._blocked(t, None, p.half_width_m):
            self._trip = None
            return self._start_avoid(t, target)
        if t >= trip['until']:
            self._trip, self._search_dir = None, None            # there (as far as it can tell): search again from here
            return self._halt(t)
        return p.drive_speed, 0.0

    def _stalled(self, seconds: float, most: float = 4.0) -> None:
        """A turn pulse moved nothing: longer pulses first, then more power (up to `most` times). What the robot
        believes about its turn rate is left alone (a pulse that moved nothing measured nothing), and the search
        does not count it."""
        self._stalls += 1
        if self._stalls >= 3:                                    # longer and stronger pulses are not doing it either
            self._want_escape = True
        if seconds >= 0.9 * self.p.max_pulse_s:
            self.turn_scale = max(self.turn_scale, min(self.turn_scale * 1.3, most))
        else:
            self._stretch = min(self._stretch * 1.6, 10.0)
        if self._sweep is not None:
            self._sweep['turned'] -= self._sweep['last']
            self._sweep['last'] = 0.0

    def _learn_turn(self, t: float) -> None:
        """Scale the next pulses by what the last one really did to a bottle's bearing."""
        (was, self._before), now = (self._before, None), self.look
        unchanged = self._same_picture(was, now, True)
        if was is not None and not unchanged:
            self._stalls = 0
        if self._pulse is None:
            if unchanged:                                        # no bottle to measure by, but the view did not change
                self._stalled(self._move_end - self._pulse_start, self.p.still_max_scale)
            return
        (bottle, before, seconds, direction, raw_before, aiming), self._pulse = self._pulse, None
        if not self._fresh(bottle, t):
            if unchanged:
                return self._stalled(seconds, self.p.still_max_scale)
            if aiming:                                           # lost it: turned too far, or the wrong way. Undo, and
                self._aim_scale = max(self._aim_scale * 0.5, 0.25)   # aim more gently until a pulse can be measured
                self._stretch = 1.0                              # (it did move, so the long pulses are over)
                self._undo = (direction, seconds)
            return
        turned = direction * (before - bottle.bearing)           # radians, positive = the way it was told to turn
        if turned < -math.radians(5.0):
            self.turn_sign = -self.turn_sign
            self.tracker.yaw_gain = -self.tracker.yaw_gain
            self._say('Turning the wrong way: left and right look swapped, reversing turns')
            return
        if turned < math.radians(1.5):
            return self._stalled(seconds)
        weight = 1.0 if self._turn_samples == 0 else 0.5         # the first measurement replaces the guess outright
        self._turn_samples, self._aim_scale, self._stretch = self._turn_samples + 1, 1.0, 1.0
        self.turn_rate = min(max((1.0 - weight) * self.turn_rate + weight * turned / seconds, 0.2), 8.0)
        raw = self.tracker.raw_yaw - raw_before
        if abs(raw) > math.radians(2.0):                         # how much the robot REALLY turns per radian of /odom
            gain = (before - bottle.bearing) / raw
            if 0.05 < abs(gain) < 20.0:
                self.tracker.yaw_gain = (1.0 - weight) * self.tracker.yaw_gain + weight * gain

    # ------------------------------------------------------------------ obstacles
    def _blocked(self, t: float, target: Optional[Bottle], half_width: float) -> bool:
        """Is something in the way? The IR sensors decide this whenever they are reporting (2026-10-16: "rely on
        these" - they see what is actually there, not a colour heuristic, and cannot be fooled by a dark wall or
        a far object that merely looks wide). The camera's floor-scan is only a fallback for while the IR topics
        have gone quiet (sensor/node fault), so losing them degrades the robot instead of blinding it."""
        if self._ir_fresh(t):
            left, right = self._ir
            return left or right
        if self.scan is None or not len(self.scan):
            return False
        x, y, bearing = self.scan[:, 0], self.scan[:, 1], self.scan[:, 2]
        hit = np.isfinite(x) & (x <= self.p.obstacle_range_m) & (np.abs(y) <= half_width)
        if target is not None:
            margin = 0.4 * (target.left - target.right) + math.radians(2.0)
            hit &= ~((bearing >= target.right - margin) & (bearing <= target.left + margin))
        return int(hit.sum()) >= 2

    def _start_avoid(self, t: float, target: Bottle) -> Tuple[float, float]:
        target.attempts += 1
        if target.attempts > self.p.max_attempts:
            target.lost = True
            self._say(f'The way to the {target.name} bottle is blocked: skipping it')
            return 0.0, 0.0
        if self._ir_fresh(t):
            ir_left, ir_right = self._ir                         # turn AWAY from whichever side tripped
            direction = -1 if ir_left and not ir_right else (1 if ir_right and not ir_left else 1)
        elif self.scan is not None and len(self.scan):
            free = np.where(np.isfinite(self.scan[:, 0]), np.minimum(self.scan[:, 0], 1.5), 1.5)
            left, right = free[self.scan[:, 2] > 0], free[self.scan[:, 2] <= 0]
            direction = 1 if (left.mean() if len(left) else 0.0) >= (right.mean() if len(right) else 0.0) else -1
        else:
            direction = 1
        if self.p.left_turns_only:
            direction = 1
        self.state, self._avoid = AVOID, {'dir': direction, 'turns': 0, 'phase': 'turn', 'until': 0.0}
        self._say(f"Obstacle ahead: going round it on the {'left' if direction > 0 else 'right'}")
        return self._halt(t)

    def _avoid_step(self, t: float) -> Tuple[float, float]:
        p, a = self.p, self._avoid
        if a['phase'] == 'turn':
            if not self._blocked(t, None, p.half_width_m + 0.06):
                a['phase'], a['until'] = 'drive', t + p.avoid_forward_s
            elif a['turns'] >= p.max_avoid_turns:
                self.state, self._search_dir = SEARCH, -a['dir']
                return 0.0, 0.0
            else:
                a['turns'] += 1
                return self._turn_pulse(t, a['dir'], math.radians(p.search_step_deg))
        if self._blocked(t, None, p.half_width_m):
            a['phase'] = 'turn'
            return self._halt(t)
        if t >= a['until']:
            self.state, self._search_dir = SEARCH, -a['dir']     # past it: the bottle is back the other way
            return self._halt(t)
        return p.drive_speed, 0.0

    # ------------------------------------------------------------------ finishing
    def _finish(self, why: str = '') -> Tuple[float, float]:
        visited = sum(b.visited for b in self.bottles)
        found = sum(b.confirmed for b in self.bottles)
        self.state, self.target = DONE, None
        self._say(why or (f'Finished: visited {visited} of {found} bottles' if found else 'Finished: no bottles found'))
        if self.p.celebrate and visited:
            w = self.p.turn_speed * self.turn_scale
            self._moves.extend([(0.0, w, 0.2, 0.15), (0.0, -w, 0.4, 0.15), (0.0, w, 0.2, 0.0)])
        return 0.0, 0.0

    # ------------------------------------------------------------------ for the dashboard
    def headline(self, t: float) -> str:
        name = self.target.name if self.target else ''
        if self.state == IDLE:
            return 'Ready'
        if self.state == WANDER:
            return 'Wandering, looking for objects'
        if self.state == SEARCH:
            return f'Turning to find the {name} bottle again' if name else 'Looking for bottles'
        if self.state == TURN:
            return f'Aiming at the {name} bottle'
        if self.state == APPROACH:
            return f'Going to the {name} bottle'
        if self.state == AVOID:
            return 'Obstacle ahead: going round it'
        if self.state == WAIT:
            return f'At the {name} bottle: waiting {max(0.0, self.wait_until - t):.0f} s'
        if self.state == BACKUP:
            return 'Backing away'
        if self.state == RETURN:
            return f'Going to where the {name} bottle was'
        visited = sum(b.visited for b in self.bottles)
        return f'Finished: visited {visited} of {sum(b.confirmed for b in self.bottles)} bottles'

    def status_of(self, b: Bottle) -> str:
        if b.visited:
            return 'visited'
        if b is self.target:
            return 'target'
        return 'lost' if b.lost else 'seen'
