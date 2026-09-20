"""Perception: camera image (+ depth) -> camera-agnostic detections with positions.

    RGB image -> Detector -> 2D detections -> SpatialEstimator -> position in base_link -> TF -> map

Both stages are interfaces, which is how camera independence is enforced rather than hoped for:
the ColorBlobDetector (OpenCV, the MVP) can be replaced by a YOLO-family detector, and the
DepthEstimator (RGB-D) and KnownHeightEstimator (RGB-only) are interchangeable. Nothing above this
package may look at `Detection.source_backend`.

Detection uses the RGB image only; depth is used only to place a detection in space. RGB and depth
are separate sensors whose content is offset in time (~40 ms), so positions are withheld while the
camera turns (motion_gate.py).

Modules: types, config, image_utils, motion_gate, detectors/, spatial/, pipeline, perception_node.

TODO(future): uncertainty - a position covariance in Detection.msg and calibrated confidence.
TODO(future): active perception - deliberately viewing an uncertain region (information gain).
TODO(phase-5b / hardware): YoloDetector if colour blobs prove brittle on real cameras.
TODO(hardware): real-camera intrinsics/distortion; RealSense depth registration and noise.
TODO(future): obstacle and safe-zone detection; occlusion handling; marker-assisted estimation.
"""
