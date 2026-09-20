"""RGB-only spatial estimation from KNOWN OBJECT HEIGHT (no depth).

A pixel row is a ray; a horizontal plane at a known height meets it at exactly one point. Which
edge of the blob to use matters enormously, and the plan's original choice (the blob's BASE on the
floor) is badly conditioned for this camera: it sits 8.75 cm above the floor looking level, and a
victim's blue region starts 5.5 cm up, so the camera is only ~3 cm above that plane and one pixel
of edge error is tens of percent of range. The blob's TOP is 20-40 cm above the camera, so the same
pixel error is a few percent. `anchor='top'` (default) uses the top; `anchor='bottom'` is kept so
that claim can be measured (tests/sim/experiments/perception_accuracy.py).

The estimate is in base_link, whose z is vertical and whose floor is `floor_offset_m` below it
(the wheel radius, from parameters.xacro - never hardcoded here).

TODO(hardware): camera mounting height/tilt is an open decision; a down-tilted camera would make
    the bottom anchor viable too. Lens distortion is not modelled (needs the real camera's calibration).
TODO(future): occlusion handling - a blob whose top is hidden behind an obstacle reads too far.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..config import ClassConfig
from ..types import CameraModel, DepthFrame, Detection2D, SpatialEstimate, Transform
from .base import SpatialEstimator


class KnownHeightEstimator(SpatialEstimator):
    needs_depth = False

    def __init__(self, floor_offset_m: float, anchor: str = 'top', max_range_m: float = 8.0):
        if anchor not in ('top', 'bottom'):
            raise ValueError("anchor must be 'top' or 'bottom'")
        self.floor_offset_m, self.anchor, self.max_range_m = floor_offset_m, anchor, max_range_m
        self.name = 'known_height' if anchor == 'top' else 'known_height_bottom'

    def estimate(self, det: Detection2D, cam: CameraModel, cam_to_base: Transform, cfg: ClassConfig,
                 depth: Optional[DepthFrame] = None) -> Optional[SpatialEstimate]:
        if self.anchor == 'top':
            v, height, axis_offset = float(det.y), cfg.top_height_m, 0.0      # apex/crown lies over the axis
        else:
            v, height, axis_offset = float(det.y + det.h), cfg.bottom_height_m, cfg.bottom_axis_offset_m
        if height <= 0.0:
            return None                                                        # no height prior for this class
        u = det.x + det.w / 2.0
        ray = cam_to_base.rotate(cam.ray(u, v))
        origin = cam_to_base.t
        z_plane = -self.floor_offset_m + height
        if abs(ray[2]) < 1e-6:
            return None
        s = (z_plane - origin[2]) / ray[2]
        if s <= 0.0:
            return None                                                        # the plane is behind the ray
        p = origin + s * ray
        d = p[:2] - origin[:2]
        rng = float(np.hypot(d[0], d[1]))
        if rng > self.max_range_m or rng < 1e-6:
            return None
        if axis_offset:
            p[:2] += axis_offset * d / rng
        return SpatialEstimate(p, rng, self.name)
