"""RGB-D spatial estimation: depth over the detection's own pixels.

Depth is used ONLY for range; the detection itself comes from RGB. Two things about the data shape
this:
  * RGB and depth are separate sensors and their content is offset (~40 ms, ~10 px at 0.4 rad/s).
    A single depth pixel at the centroid can land on the wall BEHIND a thin object, so this takes a
    low percentile over the blob's mask (an object is always nearer than its backdrop) and the
    pipeline only calls it while the camera is steady (motion_gate.py).
  * The depth image must be REGISTERED to the RGB image (same size and intrinsics), as a RealSense
    provides with align-to-colour and the simulator provides by construction. Anything else returns
    None instead of a wrong number.

TODO(hardware): RealSense depth is noisier, has holes and a ~0.3 m minimum range; re-tune the
    validity thresholds against the real device.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..config import ClassConfig
from ..types import CameraModel, DepthFrame, Detection2D, SpatialEstimate, Transform
from .base import SpatialEstimator


class DepthEstimator(SpatialEstimator):
    name = 'depth'
    needs_depth = True

    def __init__(self, percentile: float = 25.0, surface_tol_m: float = 0.03, min_valid_px: int = 8,
                 min_valid_fraction: float = 0.3, min_range_m: float = 0.1, max_range_m: float = 8.0):
        self.percentile, self.surface_tol_m = percentile, surface_tol_m
        self.min_valid_px, self.min_valid_fraction = min_valid_px, min_valid_fraction
        self.min_range_m, self.max_range_m = min_range_m, max_range_m

    def estimate(self, det: Detection2D, cam: CameraModel, cam_to_base: Transform, cfg: ClassConfig,
                 depth: Optional[DepthFrame] = None) -> Optional[SpatialEstimate]:
        if depth is None or depth.depth.shape != (cam.height, cam.width):
            return None
        crop = depth.depth[det.y:det.y + det.h, det.x:det.x + det.w]
        ok = det.mask & np.isfinite(crop) & (crop >= self.min_range_m) & (crop <= self.max_range_m)
        n_ok = int(ok.sum())
        if n_ok < self.min_valid_px or n_ok < self.min_valid_fraction * det.area:
            return None
        near = ok & (crop <= np.percentile(crop[ok], self.percentile) + self.surface_tol_m)
        ys, xs = np.nonzero(near)
        z = float(np.median(crop[near]))
        u, v = det.x + xs.mean() + 0.5, det.y + ys.mean() + 0.5               # pixel index -> continuous
        p = cam_to_base.apply(cam.ray(u, v) * z)
        d = p[:2] - cam_to_base.t[:2]
        rng = float(np.hypot(d[0], d[1]))
        if rng < 1e-6:
            return None
        p[:2] += cfg.axis_offset_m * d / rng                                   # seen surface -> the object's axis
        return SpatialEstimate(p, rng, self.name)
