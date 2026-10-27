"""Bottle colours: a hue (OpenCV's 0..179 scale), or None for black. Pure."""
from __future__ import annotations

import colorsys
import math
from typing import Optional, Tuple

BLACK = 'black'
PICKED = -1.0                           # not a hue: "the object the operator clicked on" (matched by its own colour model)
HUE_NAMES = ((8, 'red'), (20, 'orange'), (34, 'yellow'), (85, 'green'), (100, 'cyan'), (130, 'blue'),
             (150, 'purple'), (170, 'pink'), (180, 'red'))           # upper bound, name


def colour_name(hue: Optional[float]) -> str:
    if hue is None:
        return BLACK
    return 'picked' if hue < 0 else next(name for bound, name in HUE_NAMES if hue % 180.0 < bound)


def colour_rgb(hue: Optional[float]) -> Tuple[int, int, int]:
    """A display colour for a bottle of this hue."""
    if hue is None:
        return (45, 45, 45)
    if hue < 0:
        return (255, 70, 220)
    r, g, b = colorsys.hsv_to_rgb((hue % 180.0) / 180.0, 0.9, 0.92)
    return int(r * 255), int(g * 255), int(b * 255)


def hue_distance(a: Optional[float], b: Optional[float]) -> float:
    """Circular distance in hue units (0..90); black only matches black."""
    if a is None or b is None or a < 0 or b < 0:
        return 0.0 if a == b else math.inf
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)
