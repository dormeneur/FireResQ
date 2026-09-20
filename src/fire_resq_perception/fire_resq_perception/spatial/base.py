"""The spatial-estimation interface: 2D detection -> a point in base_link.

An estimator sees only the detection, a CameraModel, the camera->base transform and (optionally)
a depth frame registered to the RGB image. It never sees which camera or simulator produced them,
which is how RGB and RGB-D backends stay interchangeable (Architecture.md section 4)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..config import ClassConfig
from ..types import CameraModel, DepthFrame, Detection2D, SpatialEstimate, Transform


class SpatialEstimator(ABC):
    name = 'estimator'
    needs_depth = False

    @abstractmethod
    def estimate(self, det: Detection2D, cam: CameraModel, cam_to_base: Transform, cfg: ClassConfig,
                 depth: Optional[DepthFrame] = None) -> Optional[SpatialEstimate]:
        """Return None when no trustworthy estimate exists (never guess)."""
