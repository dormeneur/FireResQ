"""Only trust RGB<->depth<->TF fusion while the camera is (nearly) not turning.

Measured in Phase 3/4: the RGB image is offset ~40 ms from depth and TF (~10 px at 0.4 rad/s).
Rotation is what hurts: a 0.1 rad/s turn costs ~1.4 px at 87 deg, translation at driving speed
costs a centimetre. So the gate looks at angular rate mainly, from TF (odom -> base_link), which
exists on hardware too - not from any simulator topic.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Tuple


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MotionGate:
    def __init__(self, max_angular_rate: float = 0.1, max_linear_speed: float = 0.3,
                 window_s: float = 0.3, min_dt: float = 0.02):
        self.max_w, self.max_v, self.window_s, self.min_dt = max_angular_rate, max_linear_speed, window_s, min_dt
        self._s: Deque[Tuple[float, float, float, float]] = deque()      # (t, yaw, x, y)

    def update(self, t: float, yaw: float, x: float, y: float) -> None:
        self._s.append((t, yaw, x, y))
        while self._s and t - self._s[0][0] > self.window_s:
            self._s.popleft()

    def rates(self) -> Tuple[float, float]:
        """Worst angular rate (rad/s) and linear speed (m/s) between samples in the window."""
        w = v = 0.0
        for (t0, y0, x0, yy0), (t1, y1, x1, yy1) in zip(self._s, list(self._s)[1:]):
            dt = t1 - t0
            if dt >= self.min_dt:
                w = max(w, abs(_wrap(y1 - y0)) / dt)
                v = max(v, math.hypot(x1 - x0, yy1 - yy0) / dt)
        return w, v

    def allows(self) -> bool:
        """True only with enough history to know the camera is steady."""
        if len(self._s) < 2 or self._s[-1][0] - self._s[0][0] < 2 * self.min_dt:
            return False
        w, v = self.rates()
        return w <= self.max_w and v <= self.max_v
