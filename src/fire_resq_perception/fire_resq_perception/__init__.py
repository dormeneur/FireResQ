"""Perception layer - camera to camera-agnostic detections.

    Responsibility (Architecture.md section 3, Implementation_Plan.md section 7):
        image (+ depth) -> Detector -> 2D detections
                        -> SpatialEstimator -> 3D points in map
                        -> fire_resq_interfaces/DetectionArray

    The whole point of this package is that NOTHING above it can tell which camera
    backend produced a detection. RGB and RGB-D are interchangeable.

    Implemented in Phase 5. Empty by design until then.

    TODO(phase-5): Detector interface + ColorBlobDetector (OpenCV HSV).
    TODO(phase-5): SpatialEstimator interface + GroundPlaneEstimator (RGB)
                   and DepthEstimator (RGB-D).
    TODO(phase-5b): YoloDetector behind the same Detector interface.
    TODO(future): propagate geometric uncertainty into detection confidence
                  rather than passing the detector score through unchanged.
"""
