"""A synthetic camera and scene for testing perception WITHOUT a simulator.

Known 3D objects are projected through a known camera into an image (and a registered depth image),
so an estimator's maths can be checked against the exact answer. Geometry only - Gazebo is
exercised separately in tests/sim/test_perception.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from fire_resq_perception.config import class_config
from fire_resq_perception.types import CameraModel, Transform

FLOOR_OFFSET = 0.0325                     # wheel radius: the floor is this far below base_link
HFOV = 1.518                              # the 87 deg navigation camera
W, H = 640, 480
FX = W / (2 * math.tan(HFOV / 2))
CAM = CameraModel(FX, FX, W / 2, H / 2, W, H)
# camera optical frame -> base_link: optical z (forward) = base x, optical x (right) = -base y, optical y (down) = -base z
CAM_TO_BASE = Transform(np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]), np.array([0.05, 0.0, 0.055]))

RGB = {'victim': (38, 89, 255), 'fire': (255, 71, 0), 'floor': (150, 150, 150), 'clutter': (90, 92, 100), 'green': (25, 178, 51)}

VICTIM = class_config('victim', (100, 100, 40), (125, 255, 255), top_height_m=0.315, bottom_height_m=0.055,
                      axis_offset_m=0.05, bottom_axis_offset_m=0.05)
FIRE = class_config('fire', (0, 200, 200), (30, 255, 255), top_height_m=0.47, bottom_height_m=0.06,
                    axis_offset_m=0.115, bottom_axis_offset_m=0.13)
CLASSES = {'victim': VICTIM, 'fire': FIRE}


def to_optical(p_base: np.ndarray) -> np.ndarray:
    return CAM_TO_BASE.inverse().apply(p_base)


def project(p_base: np.ndarray):
    p = to_optical(p_base)
    return CAM.fx * p[0] / p[2] + CAM.cx, CAM.fy * p[1] / p[2] + CAM.cy, p[2]


@dataclass
class Obj:
    cls: str
    x: float                    # axis position in base_link (forward, left)
    y: float
    z0: float                   # height range above the floor of the coloured region
    z1: float
    r: float                    # radius (half width) of the coloured region


def victim_at(x, y):
    return Obj('victim', x, y, VICTIM.bottom_height_m, VICTIM.top_height_m, 0.05)


def fire_at(x, y):
    return Obj('fire', x, y, FIRE.bottom_height_m, FIRE.top_height_m, 0.10)


def render(objs, background=6.0, clutter=True):
    """Return (rgb uint8 HxWx3, depth float32 HxW). Each object is a rectangle from its top row to
    its bottom row (exact projections of the axis top/bottom points); depth is the front surface."""
    rgb = np.full((H, W, 3), RGB['floor'], np.uint8)
    depth = np.full((H, W), background, np.float32)
    if clutter:                                            # things that must NOT be detected
        rgb[300:340, 40:90] = RGB['clutter']
        rgb[380:420, 500:560] = RGB['green']
    for o in objs:
        top = np.array([o.x, o.y, -FLOOR_OFFSET + o.z1])                  # the crown lies over the axis
        d = np.array([o.x, o.y]) - CAM_TO_BASE.t[:2]
        d = d / np.hypot(*d)
        rim = np.array([o.x - o.r * d[0], o.y - o.r * d[1], -FLOOR_OFFSET + o.z0])   # the bottom edge is the FRONT RIM
        u, v_top, z = project(top)
        _, v_bot, _ = project(rim)
        half = CAM.fx * o.r / z
        y0, y1 = int(round(v_top)), int(round(v_bot))
        x0, x1 = int(round(u - half)), int(round(u + half))
        rgb[y0:y1, x0:x1] = RGB[o.cls]
        depth[y0:y1, x0:x1] = z - o.r                       # front surface (z-depth)
    return rgb, depth
