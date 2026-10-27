"""A pretend room and a pretend robot, for rehearsing the demo and testing it without hardware. Pure.

The room is a flat floor with upright cylinders on it (bottles and obstacles). The robot is deliberately a BAD
one, like the real one: it is told a velocity, believes it did exactly that (which is what /odom reports), and
really did something else (`turn_gain`, `speed_gain`, swapped turning, a dead band, a delay).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .seeker import Sighting
from .vision import Camera


@dataclass
class Thing:
    """An upright cylinder on the floor. hue None + bottle = a black bottle; bottle False = an obstacle."""
    x: float
    y: float
    radius: float = 0.035
    height: float = 0.22
    bgr: Tuple[int, int, int] = (40, 40, 220)
    hue: Optional[float] = 0.0
    bottle: bool = True


@dataclass
class FakeRobot:
    turn_gain: float = 1.0              # real turn rate / commanded
    speed_gain: float = 1.0
    swapped: bool = False               # left and right wired the wrong way round
    min_turn: float = 0.0               # commanded |rad/s| below which the robot does not turn at all
    delay_s: float = 0.2                # command -> motors
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    odom: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    _queue: deque = field(default_factory=deque)

    def step(self, t: float, v: float, w: float, dt: float) -> None:
        """Advance by dt; (v, w) is what is being commanded NOW and reaches the motors `delay_s` later."""
        self._queue.append((t + self.delay_s, v, w))
        applied = (0.0, 0.0)
        while self._queue and self._queue[0][0] <= t:
            _, *applied = self._queue.popleft()
            self._applied = tuple(applied)
        v, w = getattr(self, '_applied', (0.0, 0.0))
        self.odom[0] += v * math.cos(self.odom[2]) * dt          # what the driver believes (it has no encoders)
        self.odom[1] += v * math.sin(self.odom[2]) * dt
        self.odom[2] += w * dt
        true_w = 0.0 if abs(w) < self.min_turn else w * self.turn_gain * (-1.0 if self.swapped else 1.0)
        self.x += v * self.speed_gain * math.cos(self.yaw) * dt
        self.y += v * self.speed_gain * math.sin(self.yaw) * dt
        self.yaw += true_w * dt


class FakeWorld:
    def __init__(self, things: List[Thing], cam: Camera = Camera(), robot: Optional[FakeRobot] = None,
                 floor_bgr=(196, 204, 198), wall_bgr=(96, 124, 132)):
        self.things, self.cam, self.robot = things, cam, robot or FakeRobot()
        self.floor_bgr, self.wall_bgr = floor_bgr, wall_bgr

    def _relative(self, thing: Thing) -> Tuple[float, float]:
        """(forward, left) of the thing's centre in the robot frame."""
        dx, dy = thing.x - self.robot.x, thing.y - self.robot.y
        c, s = math.cos(self.robot.yaw), math.sin(self.robot.yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def distance_to(self, thing: Thing) -> float:
        return math.hypot(thing.x - self.robot.x, thing.y - self.robot.y)

    # -- what a perfect detector would report (fast: the behaviour tests use this)
    def observe(self, strips: int = 32) -> Tuple[List[Sighting], np.ndarray]:
        cam = self.cam
        half = math.atan2(cam.width / 2.0, cam.f)
        u = (np.arange(strips) + 0.5) * cam.width / strips
        bearings = np.arctan2(cam.cx - u, cam.f)
        scan = np.full((strips, 3), np.nan, np.float32)
        scan[:, 2] = bearings
        nearest = [None] * strips
        for thing in self.things:
            fx, fy = self._relative(thing)
            for i, b in enumerate(bearings):                     # ray from the robot along bearing b
                along = fx * math.cos(b) + fy * math.sin(b)
                off = abs(-fx * math.sin(b) + fy * math.cos(b))
                if along <= 0 or off > thing.radius:
                    continue
                d = along - math.sqrt(thing.radius ** 2 - off ** 2)
                if d > 0 and (not np.isfinite(scan[i, 0]) or d * math.cos(b) < scan[i, 0]):
                    scan[i, 0], scan[i, 1], nearest[i] = d * math.cos(b), d * math.sin(b), thing
        sightings = []
        for thing in self.things:
            fx, fy = self._relative(thing)
            bearing, rng = math.atan2(fy, fx), math.hypot(fx, fy) - thing.radius
            if not thing.bottle or fx <= 0.05 or abs(bearing) > half or rng > 4.0:
                continue
            i = int(np.argmin(np.abs(bearings - bearing)))
            if nearest[i] not in (None, thing):
                continue                                         # hidden behind something
            wide = math.atan2(thing.radius, max(fx, 0.05))
            face = max(fx - thing.radius, 0.03)
            base = min(cam.horizon_row + cam.f * cam.height_m / face, cam.height)
            top = max(cam.horizon_row - cam.f * (thing.height - cam.height_m) / face, 0.0)
            width = 2 * thing.radius * cam.f / max(fx, 0.05) / cam.width
            sightings.append(Sighting(thing.hue, bearing, max(rng, 0.0), width, bearing + wide, bearing - wide,
                                      cover_frac=min(width, 1.0) * (base - top) / cam.height))
        return sightings, scan

    # -- what the camera would show (the rehearsal, and the tests of the detector itself)
    def render(self, noise: float = 2.0, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        cam = self.cam
        img = np.empty((cam.height, cam.width, 3), np.uint8)
        horizon = int(cam.horizon_row)
        img[:horizon] = self.wall_bgr
        img[horizon:] = self.floor_bgr
        shade = np.linspace(0.82, 1.0, cam.height - horizon)[:, None, None]      # the floor is darker far away
        img[horizon:] = (img[horizon:] * shade).astype(np.uint8)
        for fx, fy, thing in sorted(((*self._relative(t), t) for t in self.things), key=lambda r: -r[0]):
            if fx <= 0.03:
                continue
            u = cam.cx - cam.f * fy / fx
            half = cam.f * thing.radius / fx
            face = max(fx - thing.radius, 0.03)                  # the part of the base nearest the camera
            bottom = cam.horizon_row + cam.f * cam.height_m / face
            top = cam.horizon_row - cam.f * (thing.height - cam.height_m) / face
            x0, x1 = int(round(u - half)), int(round(u + half))
            y0, y1 = int(round(top)), int(round(bottom))
            if x1 < 0 or x0 >= cam.width or y1 < 0:
                continue
            cv2.rectangle(img, (x0, y0), (x1, y1), thing.bgr, -1)
            edge = tuple(int(c * 0.75) for c in thing.bgr)       # a little shading so it reads as round
            cv2.rectangle(img, (x0, y0), (x0 + max(1, (x1 - x0) // 6), y1), edge, -1)
        if noise > 0:
            rng = rng or np.random.default_rng(0)
            img = np.clip(img.astype(np.int16) + rng.normal(0.0, noise, img.shape), 0, 255).astype(np.uint8)
        return img


def demo_room() -> List[Thing]:
    """The rehearsal layout: four bottles, one of them hidden behind the robot at the start, and a box in the way."""
    return [
        Thing(1.10, 0.25, bgr=(40, 40, 215), hue=0.0),                       # red
        Thing(1.90, -0.55, bgr=(200, 110, 30), hue=106.0),                   # blue
        Thing(0.70, -1.20, bgr=(40, 200, 235), hue=27.0),                    # yellow, off to the right
        Thing(-1.00, 0.80, bgr=(25, 25, 25), hue=None),                      # black, behind the start
        Thing(1.45, -0.20, radius=0.09, height=0.16, bgr=(96, 100, 104), hue=None, bottle=False),   # a grey box
    ]
