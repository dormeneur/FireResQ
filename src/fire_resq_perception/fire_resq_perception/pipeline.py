"""RGB (+ depth) -> detections with positions in base_link. Pure logic: no ROS, no Gazebo.

Order of trust, and why a position is withheld rather than guessed:
  no camera transform            -> no position (nothing to project through)
  camera is turning              -> no position (RGB/depth/TF are offset ~40 ms; see motion_gate.py)
  blob touches the image border  -> no position (its extent, and so the position, is biased)
  estimator returns None         -> no position
The 2D detection (class, confidence, box) is ALWAYS reported: it does not depend on any of that.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from .config import ClassConfig
from .detectors.base import Detector
from .spatial.base import SpatialEstimator
from .types import CameraModel, DepthFrame, Detection3D, Transform

BACKENDS = ('auto', 'depth', 'known_height')


class PerceptionPipeline:
    def __init__(self, detector: Detector, classes: Dict[str, ClassConfig],
                 depth_estimator: SpatialEstimator, plane_estimator: SpatialEstimator,
                 backend: str = 'auto', min_confidence: float = 0.0):
        if backend not in BACKENDS:
            raise ValueError(f'backend must be one of {BACKENDS}')
        self.detector, self.classes = detector, classes
        self.depth_estimator, self.plane_estimator = depth_estimator, plane_estimator
        self.backend, self.min_confidence = backend, min_confidence

    def _chain(self, depth: Optional[DepthFrame]) -> List[SpatialEstimator]:
        if self.backend == 'depth':
            return [self.depth_estimator]
        if self.backend == 'known_height':
            return [self.plane_estimator]
        return [self.depth_estimator, self.plane_estimator] if depth is not None else [self.plane_estimator]

    def process(self, rgb: np.ndarray, cam: CameraModel, cam_to_base: Optional[Transform],
                depth: Optional[DepthFrame], motion_ok: bool) -> List[Detection3D]:
        out: List[Detection3D] = []
        for det in self.detector.detect(rgb):
            if det.confidence < self.min_confidence:
                continue
            res = Detection3D(det)
            if cam_to_base is None:
                res.reason = 'no camera transform'
            elif not motion_ok:
                res.reason = 'camera moving: RGB/depth/TF are offset in time'
            elif det.truncated:
                res.reason = 'blob touches the image border'
            else:
                for est in self._chain(depth):
                    e = est.estimate(det, cam, cam_to_base, self.classes[det.class_id], depth)
                    if e is not None:
                        res.position_base, res.range_m, res.backend, res.valid = e.position, e.range_m, e.method, True
                        break
                else:
                    res.reason = 'no estimator could place it'
            out.append(res)
        return out
