"""4WD skid-steer drive model: (v, w) <-> motor powers, and dead reckoning. Pure, no ROS, no serial.

THIS ROBOT HAS NO ENCODERS. Nothing here measures motion. `twist_of` is the model's BELIEF about what a motor
power does, from two numbers measured with a tape measure (`full_speed_mps`, `min_power`) and the track width.
Odometry integrated from it is open-loop dead reckoning: it cannot see wheel slip, a stalled motor, sagging
batteries or a robot pushed against a wall, and skid-steer turning is mostly slip. It exists so the rest of the
stack has an `odom` frame and knows whether the robot is being told to move - not as a position source.
Localisation has to come from somewhere that looks at the world.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from .protocol import MAX_POWER, MOTORS, Powers

# How a turn ON THE SPOT (no forward speed) is made. Curving while driving is always both sides forward at
# different speeds. Turning left is shown; turning right is the mirror image.
TURN_STYLES = {
    'skid': 'spin: left side backward, right side forward',
    'pivot_back': 'pivot: left side backward, right side stopped',
    'pivot_forward': 'pivot: right side forward, left side stopped',
    'pivot_right': 'the RIGHT side only, both ways: forward to turn left, backward to turn right; left side stopped',
    'pivot_rear': 'rear wheels only (left back, right forward), front wheels stopped',
    'pivot_front': 'front wheels only (left back, right forward), rear wheels stopped',
    'pivot_split': 'LEFT turns with the rear wheels only, RIGHT turns with the front wheels only',
    'mixed': 'LEFT: spin (left side back, right side forward). RIGHT: left side forward only, right side stopped',
}


@dataclass(frozen=True)
class MotorMap:
    """Which shield terminals (1..4) drive which side, and each terminal's polarity (+1, or -1 if that motor
    turns backwards on a positive power). Wiring facts, found with `motor_check`, never guessed in code."""
    left: Tuple[int, ...]
    right: Tuple[int, ...]
    signs: Tuple[int, ...]
    front: Tuple[int, ...] = (1, 2)         # the front axle's terminals (only the pivot_front/pivot_rear turns use it)

    def __post_init__(self):
        if sorted(self.left + self.right) != list(range(1, MOTORS + 1)) or not self.left or not self.right:
            raise ValueError(f'left {self.left} + right {self.right} must be terminals 1..{MOTORS}, each exactly once')
        if len(self.signs) != MOTORS or any(s not in (1, -1) for s in self.signs):
            raise ValueError(f'signs must be {MOTORS} values of +1/-1, got {self.signs}')
        if not (set(self.front) & set(self.left) and set(self.front) & set(self.right)) or len(set(self.front)) != MOTORS // 2:
            raise ValueError(f'front {self.front} must be one left and one right terminal')


@dataclass(frozen=True)
class DriveModel:
    motors: MotorMap
    track_m: float                 # EFFECTIVE left-right wheel separation (skid steer: larger than the geometric one)
    full_speed_mps: float          # wheel speed over the ground at MAX_POWER, measured
    min_power: int                 # the power at which the loaded wheels just start to turn, measured
    stop_fraction: float = 0.02    # side speeds below this fraction of full speed are "stopped"
    turn_style: str = 'skid'       # how a turn on the spot is made: one of TURN_STYLES
    right_turn_idle: Tuple[int, ...] = ()   # terminals left unpowered in a RIGHT turn on the spot (a motor that spoils it)

    def __post_init__(self):
        if self.track_m <= 0.0 or self.full_speed_mps <= 0.0:
            raise ValueError('track_m and full_speed_mps must be positive')
        if self.turn_style not in TURN_STYLES:
            raise ValueError(f'turn_style must be one of {sorted(TURN_STYLES)}, got {self.turn_style!r}')
        if not 0 <= self.min_power < MAX_POWER:
            raise ValueError(f'min_power must be within 0..{MAX_POWER - 1}')

    # -- command -> power
    def _power(self, speed: float) -> int:
        frac = abs(speed) / self.full_speed_mps
        if frac < self.stop_fraction:
            return 0
        p = self.min_power + (MAX_POWER - self.min_power) * min(frac, 1.0)
        return int(math.copysign(round(p), speed))

    def side_powers(self, v: float, w: float) -> Tuple[int, int]:
        left, right = v - w * self.track_m / 2.0, v + w * self.track_m / 2.0
        over = max(abs(left), abs(right)) / self.full_speed_mps
        if over > 1.0:                                           # keep the commanded curvature when saturated
            left, right = left / over, right / over
        return self._power(left), self._power(right)

    def motor_powers(self, v: float, w: float) -> Powers:
        out = [0] * MOTORS
        on_the_spot = abs(v) < self.stop_fraction * self.full_speed_mps and w != 0.0
        if on_the_spot and self.turn_style != 'skid':
            for terminal, speed in self._pivot(w).items():
                out[terminal - 1] = self._power(speed) * self.motors.signs[terminal - 1]
        else:
            left, right = self.side_powers(v, w)
            for side, power in ((self.motors.left, left), (self.motors.right, right)):
                for terminal in side:
                    out[terminal - 1] = power * self.motors.signs[terminal - 1]
        if on_the_spot and w < 0:
            for terminal in self.right_turn_idle:
                out[terminal - 1] = 0
        return tuple(out)

    def _pivot(self, w: float) -> dict:
        """Wheel speeds (terminal -> m/s, the rest stopped) for a turn on the spot at w rad/s in a pivot style."""
        m, full = self.motors, self.full_speed_mps
        if self.turn_style == 'mixed':
            if w > 0:                                            # left: the full four-wheel spin, which works
                speed = min(w * self.track_m / 2.0, full)
                return {**{t: -speed for t in m.left}, **{t: speed for t in m.right}}
            return {t: min(-w * self.track_m, full) for t in m.left}    # right: no motor is asked to run backward
        if self.turn_style in ('pivot_back', 'pivot_forward', 'pivot_right'):
            speed = min(abs(w) * self.track_m, full)             # one side does all the turning: twice the skid speed
            if self.turn_style == 'pivot_right':                 # always the same wheels, whichever way it turns
                return {t: math.copysign(speed, w) for t in m.right}
            if self.turn_style == 'pivot_back':                  # the side on the inside of the turn reverses
                return {t: -speed for t in (m.left if w > 0 else m.right)}
            return {t: speed for t in (m.right if w > 0 else m.left)}
        speed = math.copysign(min(abs(w) * self.track_m / 2.0, full), w)
        front = self.turn_style == 'pivot_front' or (self.turn_style == 'pivot_split' and w < 0)
        axle = set(m.front) if front else set(m.left + m.right) - set(m.front)
        return {t: (-speed if t in m.left else speed) for t in axle}

    # -- applied power -> believed motion
    def _speed(self, power: float) -> float:
        if abs(power) <= self.min_power:
            return 0.0
        return math.copysign((abs(power) - self.min_power) / (MAX_POWER - self.min_power) * self.full_speed_mps, power)

    def twist_of(self, powers: Sequence[int]) -> Tuple[float, float]:
        """(v, w) the model believes these applied powers produce. See the module docstring before trusting it."""
        def side(terminals):
            return sum(self._speed(powers[t - 1] * self.motors.signs[t - 1]) for t in terminals) / len(terminals)
        left, right = side(self.motors.left), side(self.motors.right)
        return (left + right) / 2.0, (right - left) / self.track_m


def integrate(x: float, y: float, yaw: float, v: float, w: float, dt: float) -> Tuple[float, float, float]:
    """One dead-reckoning step (midpoint heading); yaw stays within (-pi, pi]."""
    mid = yaw + w * dt / 2.0
    yaw = yaw + w * dt
    return x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), math.atan2(math.sin(yaw), math.cos(yaw))
