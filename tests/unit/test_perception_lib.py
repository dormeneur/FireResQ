"""The perception library, without ROS or Gazebo: detector, both spatial estimators, the motion
gate, the pipeline that ties them together, and the guarantees that make RGB and RGB-D
interchangeable. Ground truth is a synthetic scene projected through a known camera."""
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import perception_scene as S
from fire_resq_perception.config import classes_from_dict
from fire_resq_perception.detectors import ColorBlobDetector, Detector
from fire_resq_perception.image_utils import to_depth_metres, to_rgb
from fire_resq_perception.motion_gate import MotionGate
from fire_resq_perception.pipeline import PerceptionPipeline
from fire_resq_perception.spatial import DepthEstimator, KnownHeightEstimator, SpatialEstimator
from fire_resq_perception.types import DepthFrame

REPO = Path(__file__).resolve().parents[2]
CFG = REPO / 'src' / 'fire_resq_perception' / 'config' / 'perception.yaml'
CAM_XY = S.CAM_TO_BASE.t[:2]


def place(rng, bearing_deg=0.0):
    a = math.radians(bearing_deg)
    return CAM_XY[0] + rng * math.cos(a), CAM_XY[1] + rng * math.sin(a)


def detect(objs, **kw):
    rgb, depth = S.render(objs, **kw)
    return ColorBlobDetector(list(S.CLASSES.values())).detect(rgb), rgb, depth


def one(dets, cls):
    found = [d for d in dets if d.class_id == cls]
    assert len(found) == 1, f'expected one {cls}, got {len(found)}'
    return found[0]


# ================================================================ detector
def test_detector_finds_each_class_and_ignores_clutter():
    dets, _, _ = detect([S.victim_at(*place(2.0, 10)), S.fire_at(*place(3.0, -15))])
    assert sorted(d.class_id for d in dets) == ['fire', 'victim']       # grey and GREEN clutter must not match
    v = one(dets, 'victim')
    assert v.w > 5 and v.h > 20 and not v.truncated and 0.5 < v.confidence <= 1.0
    assert v.mask.shape == (v.h, v.w) and v.mask.dtype == bool and v.mask.sum() == v.area


def test_detector_bbox_is_the_exact_rendered_rectangle():
    o = S.victim_at(*place(2.5, 0))
    _, v_top, z = S.project(np.array([o.x, o.y, -S.FLOOR_OFFSET + o.z1]))
    d = one(detect([o])[0], 'victim')
    assert d.y == round(v_top)
    assert abs((d.x + d.w / 2) - S.CAM.cx) <= 1.0                       # dead ahead => centred


def test_detector_separates_two_victims_and_sorts_largest_first():
    dets, _, _ = detect([S.victim_at(*place(1.5, -10)), S.victim_at(*place(4.0, 15))])
    v = [d for d in dets if d.class_id == 'victim']
    assert len(v) == 2 and v[0].area > v[1].area


def test_detector_min_area_drops_specks():
    rgb = np.full((S.H, S.W, 3), S.RGB['floor'], np.uint8)
    rgb[100:105, 100:105] = S.RGB['victim']                              # 25 px < min_area 30
    rgb[200:210, 200:210] = S.RGB['victim']                              # 100 px
    v = ColorBlobDetector([S.VICTIM]).detect(rgb)
    assert len(v) == 1 and v[0].area == 100


def test_detector_flags_truncated_blobs_and_halves_their_confidence():
    rgb = np.full((S.H, S.W, 3), S.RGB['floor'], np.uint8)
    rgb[100:200, 0:40] = S.RGB['victim']                                  # touches the left border
    rgb[100:200, 300:340] = S.RGB['victim']
    a, b = sorted(ColorBlobDetector([S.VICTIM]).detect(rgb), key=lambda d: d.x)
    assert a.truncated and not b.truncated
    assert a.confidence == pytest.approx(b.confidence * 0.5, rel=1e-6)


def test_detector_supports_hue_wrap_with_several_ranges():
    red = replace(S.VICTIM, name='red', hsv_ranges=(((0, 100, 100), (8, 255, 255)), ((172, 100, 100), (179, 255, 255))))
    rgb = np.full((S.H, S.W, 3), S.RGB['floor'], np.uint8)
    rgb[50:100, 50:100] = (255, 0, 10)                                    # hue ~179 (wraps)
    rgb[150:200, 150:200] = (255, 30, 0)                                  # hue ~4
    assert len(ColorBlobDetector([red]).detect(rgb)) == 2


def test_detector_rejects_bad_input_and_needs_a_class():
    with pytest.raises(ValueError):
        ColorBlobDetector([S.VICTIM]).detect(np.zeros((10, 10), np.uint8))
    with pytest.raises(ValueError):
        ColorBlobDetector([])


def test_detector_implements_the_swappable_interface():
    assert isinstance(ColorBlobDetector([S.VICTIM]), Detector)


# ================================================================ known-height (RGB only)
@pytest.mark.parametrize('k', [20, 30, 40, 60])
def test_known_height_top_anchor_is_exact_when_the_edge_lands_on_a_pixel_row(k):
    """Choose a range so the top edge falls exactly k rows above the horizon: the maths must return the truth."""
    dz = (-S.FLOOR_OFFSET + S.VICTIM.top_height_m) - S.CAM_TO_BASE.t[2]
    rng = S.FX * dz / k
    o = S.victim_at(CAM_XY[0] + rng, 0.0)
    d = one(detect([o])[0], 'victim')
    e = KnownHeightEstimator(S.FLOOR_OFFSET, 'top').estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM)
    assert e.position[:2] == pytest.approx([o.x, o.y], abs=1e-3)
    assert e.position[2] == pytest.approx(-S.FLOOR_OFFSET + S.VICTIM.top_height_m, abs=1e-6)
    assert e.range_m == pytest.approx(rng, abs=1e-3)


@pytest.mark.parametrize('k', [5, 8, 10])
def test_known_height_bottom_anchor_is_exact_and_adds_the_rim_offset(k):
    dz = S.CAM_TO_BASE.t[2] - (-S.FLOOR_OFFSET + S.VICTIM.bottom_height_m)
    rng_rim = S.FX * dz / k                                                # to the FRONT RIM
    o = S.victim_at(CAM_XY[0] + rng_rim + 0.05, 0.0)                       # axis is one radius behind the rim
    d = one(detect([o])[0], 'victim')
    e = KnownHeightEstimator(S.FLOOR_OFFSET, 'bottom').estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM)
    assert e.position[:2] == pytest.approx([o.x, o.y], abs=2e-3)


@pytest.mark.parametrize('rng', [1.5, 2.5, 3.5, 4.5, 5.5])
@pytest.mark.parametrize('bearing', [-30, 0, 30])
def test_known_height_top_anchor_stays_within_one_pixel_of_quantisation(rng, bearing):
    """Between the exact rows, a top edge is quantised to a whole pixel: the error must stay inside
    what one pixel of edge position can cause (d^2 / (fx * dz))."""
    for make, cfg in ((S.victim_at, S.VICTIM), (S.fire_at, S.FIRE)):
        o = make(*place(rng, bearing))
        d = one(detect([o])[0], cfg.name)
        e = KnownHeightEstimator(S.FLOOR_OFFSET).estimate(d, S.CAM, S.CAM_TO_BASE, cfg)
        dz = (-S.FLOOR_OFFSET + cfg.top_height_m) - S.CAM_TO_BASE.t[2]
        bound = 1.5 * rng ** 2 / (S.FX * dz) + 0.01
        assert np.hypot(e.position[0] - o.x, e.position[1] - o.y) <= bound, (cfg.name, rng, bearing)


def test_the_bottom_anchor_is_badly_conditioned_for_this_low_camera_and_the_top_is_not():
    """The reason the top is the default: a 1-pixel edge error costs far more range from the bottom."""
    rng = 3.0
    dz_top = (-S.FLOOR_OFFSET + S.VICTIM.top_height_m) - S.CAM_TO_BASE.t[2]
    dz_bot = abs(S.CAM_TO_BASE.t[2] - (-S.FLOOR_OFFSET + S.VICTIM.bottom_height_m))
    per_px_top, per_px_bot = rng ** 2 / (S.FX * dz_top), rng ** 2 / (S.FX * dz_bot)
    assert per_px_bot > 5 * per_px_top           # the camera is 7x closer in height to the bottom plane
    assert per_px_bot > 0.25 * rng               # one pixel of bottom-edge error = over a QUARTER of the range at 3 m
    assert per_px_top < 0.05 * rng               # ... but under 5 % from the top


def test_known_height_refuses_instead_of_guessing():
    d = one(detect([S.victim_at(*place(2.0))])[0], 'victim')
    est = KnownHeightEstimator(S.FLOOR_OFFSET)
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, replace(S.VICTIM, top_height_m=0.0)) is None      # no height prior
    assert KnownHeightEstimator(S.FLOOR_OFFSET, max_range_m=1.0).estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM) is None
    # a top edge BELOW the horizon cannot be a plane above the camera: the ray points away from it
    low = replace(S.VICTIM, top_height_m=0.01)
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, low) is None
    with pytest.raises(ValueError):
        KnownHeightEstimator(S.FLOOR_OFFSET, 'middle')


# ================================================================ depth (RGB-D)
@pytest.mark.parametrize('rng', [1.0, 2.0, 3.5, 5.0, 6.5])
@pytest.mark.parametrize('bearing', [-35, 0, 35])
def test_depth_estimator_recovers_the_axis(rng, bearing):
    o = S.victim_at(*place(rng, bearing))
    dets, _, depth = detect([o])
    d = one(dets, 'victim')
    e = DepthEstimator().estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(depth, 0.0))
    assert np.hypot(e.position[0] - o.x, e.position[1] - o.y) <= 0.04, (rng, bearing)


def test_depth_is_robust_to_background_pixels_inside_the_blob():
    """The RGB/depth offset puts BACKDROP depth under part of an RGB blob. A low percentile over the
    mask must still find the object, where a centroid pixel would not."""
    o = S.victim_at(*place(3.0, 0))
    dets, _, depth = detect([o])
    d = one(dets, 'victim')
    shifted = depth.copy()
    shifted[d.y:d.y + d.h, d.x:d.x + d.w // 2 + 1] = 6.0                  # half the blob sees the wall behind it
    est = DepthEstimator().estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(shifted, 0.0))
    assert est is not None and np.hypot(est.position[0] - o.x, est.position[1] - o.y) <= 0.06
    lone = depth.copy()
    lone[d.y:d.y + d.h, d.x:d.x + d.w] = 6.0                              # ALL of it: nothing to find, but it must not crash
    DepthEstimator().estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(lone, 0.0))


def test_depth_estimator_refuses_bad_depth():
    o = S.victim_at(*place(3.0))
    dets, _, depth = detect([o])
    d = one(dets, 'victim')
    est = DepthEstimator()
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, None) is None
    holes = depth.copy()
    holes[:] = 0.0                                                         # no return anywhere (a RealSense hole)
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(holes, 0.0)) is None
    nan = depth.copy()
    nan[:] = np.nan
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(nan, 0.0)) is None
    assert est.estimate(d, S.CAM, S.CAM_TO_BASE, S.VICTIM, DepthFrame(depth[::2, ::2], 0.0)) is None   # NOT registered to the RGB


# ================================================================ interchangeability
def make_pipeline(backend='auto'):
    return PerceptionPipeline(ColorBlobDetector(list(S.CLASSES.values())), S.CLASSES, DepthEstimator(),
                              KnownHeightEstimator(S.FLOOR_OFFSET), backend)


def test_both_estimators_are_spatial_estimators():
    assert isinstance(DepthEstimator(), SpatialEstimator) and isinstance(KnownHeightEstimator(0.03), SpatialEstimator)


def test_rgb_and_rgbd_backends_agree_so_cognition_cannot_tell_them_apart():
    """Camera independence: the same scene through either estimator lands at the same place, to
    within what the RGB-only estimate can resolve. Same output type either way."""
    for rng, bearing in [(1.5, -20), (2.5, 0), (3.0, 25)]:
        o = S.victim_at(*place(rng, bearing))
        rgb, depth = S.render([o])
        a = make_pipeline('depth').process(rgb, S.CAM, S.CAM_TO_BASE, DepthFrame(depth, 0.0), True)[0]
        b = make_pipeline('known_height').process(rgb, S.CAM, S.CAM_TO_BASE, None, True)[0]
        assert a.valid and b.valid and (a.backend, b.backend) == ('depth', 'known_height')
        assert type(a.position_base) is type(b.position_base) and a.position_base.shape == b.position_base.shape == (3,)
        dz = (-S.FLOOR_OFFSET + S.VICTIM.top_height_m) - S.CAM_TO_BASE.t[2]
        assert np.linalg.norm(a.position_base[:2] - b.position_base[:2]) <= 1.5 * rng ** 2 / (S.FX * dz) + 0.05


def test_auto_prefers_depth_and_falls_back_to_rgb_only():
    rgb, depth = S.render([S.victim_at(*place(2.5))])
    assert make_pipeline().process(rgb, S.CAM, S.CAM_TO_BASE, DepthFrame(depth, 0.0), True)[0].backend == 'depth'
    assert make_pipeline().process(rgb, S.CAM, S.CAM_TO_BASE, None, True)[0].backend == 'known_height'
    unusable = DepthFrame(np.zeros_like(depth), 0.0)                        # depth exists but is all holes
    assert make_pipeline().process(rgb, S.CAM, S.CAM_TO_BASE, unusable, True)[0].backend == 'known_height'


def test_strict_backends_do_not_fall_back():
    rgb, _ = S.render([S.victim_at(*place(2.5))])
    r = make_pipeline('depth').process(rgb, S.CAM, S.CAM_TO_BASE, None, True)[0]
    assert not r.valid and r.position_base is None and r.reason
    with pytest.raises(ValueError):
        make_pipeline('lidar')


# ================================================================ withholding positions
def test_the_2d_detection_is_always_reported_even_when_no_position_can_be_computed():
    rgb, depth = S.render([S.victim_at(*place(2.5))])
    pipe = make_pipeline()
    for tf, moving, why in ((None, True, 'transform'), (S.CAM_TO_BASE, False, 'moving')):
        r = pipe.process(rgb, S.CAM, tf, DepthFrame(depth, 0.0), moving)
        assert len(r) == 1 and r[0].det.class_id == 'victim' and not r[0].valid and r[0].position_base is None
        assert why in r[0].reason


def test_truncated_blobs_get_no_position():
    rgb = np.full((S.H, S.W, 3), S.RGB['floor'], np.uint8)
    rgb[150:300, 0:60] = S.RGB['victim']
    r = make_pipeline().process(rgb, S.CAM, S.CAM_TO_BASE, None, True)
    assert len(r) == 1 and r[0].det.truncated and not r[0].valid and 'border' in r[0].reason


def test_low_confidence_detections_can_be_dropped():
    rgb = np.full((S.H, S.W, 3), S.RGB['floor'], np.uint8)
    rgb[100:106, 100:106] = S.RGB['victim']                                 # 36 px: detected but weak
    pipe = make_pipeline()
    assert len(pipe.process(rgb, S.CAM, S.CAM_TO_BASE, None, True)) == 1
    pipe.min_confidence = 0.5
    assert pipe.process(rgb, S.CAM, S.CAM_TO_BASE, None, True) == []


# ================================================================ the timing-offset gate
def feed(gate, rate, hz=25.0, seconds=1.0, yaw0=0.0, speed=0.0):
    n = int(seconds * hz)
    for i in range(n + 1):
        t = i / hz
        gate.update(t, yaw0 + rate * t, speed * t, 0.0)
    return gate


def test_gate_allows_a_steady_camera_and_blocks_a_turning_one():
    assert feed(MotionGate(), 0.0).allows()
    assert feed(MotionGate(), 0.05).allows()                                # slow drift: ~0.7 px, fine
    assert not feed(MotionGate(), 0.4).allows()                             # the measured problem case: ~10 px
    assert not feed(MotionGate(), -1.2).allows()                            # Nav2's rotate-to-heading


def test_gate_tolerates_driving_straight_but_not_fast():
    assert feed(MotionGate(), 0.0, speed=0.24).allows()                     # Nav2 cruise speed
    assert not feed(MotionGate(max_linear_speed=0.1), 0.0, speed=0.24).allows()


def test_gate_needs_history_and_handles_angle_wraparound():
    g = MotionGate()
    assert not g.allows()
    g.update(0.0, 0.0, 0, 0)
    assert not g.allows()                                                   # one sample is not evidence
    w = feed(MotionGate(), 0.0, yaw0=math.pi - 0.001)
    assert w.allows()
    turning = MotionGate()
    for i in range(20):
        turning.update(i * 0.04, math.pi - 0.05 + 0.4 * i * 0.04, 0, 0)     # crosses +-pi while turning
    assert not turning.allows()


def test_gate_recovers_once_the_turn_leaves_the_window():
    g = MotionGate(window_s=0.3)
    for i in range(10):
        g.update(i * 0.04, 0.4 * i * 0.04, 0, 0)
    assert not g.allows()
    for i in range(10, 30):
        g.update(i * 0.04, 0.4 * 9 * 0.04, 0, 0)                            # stopped
    assert g.allows()


# ================================================================ images
def fake_image(encoding, arr, step=None):
    a = np.ascontiguousarray(arr)
    row = a.shape[1] * (a.shape[2] if a.ndim == 3 else 1) * a.itemsize
    data = a.tobytes()
    if step and step > row:                                                 # rows padded, as some drivers do
        data = b''.join(a[r].tobytes() + b'\x00' * (step - row) for r in range(a.shape[0]))
    return SimpleNamespace(encoding=encoding, height=a.shape[0], width=a.shape[1], step=step or row, data=data)


def test_image_conversion_handles_the_encodings_real_and_simulated_cameras_use():
    rgb = np.random.default_rng(0).integers(0, 255, (6, 8, 3), dtype=np.uint8)
    assert np.array_equal(to_rgb(fake_image('rgb8', rgb)), rgb)
    assert np.array_equal(to_rgb(fake_image('bgr8', rgb[..., ::-1])), rgb)
    assert np.array_equal(to_rgb(fake_image('rgb8', rgb, step=8 * 3 + 5)), rgb)              # padded rows
    d = np.random.default_rng(1).random((6, 8), dtype=np.float32) * 5
    assert np.allclose(to_depth_metres(fake_image('32FC1', d)), d)                            # Gazebo: metres
    mm = (d * 1000).astype(np.uint16)
    assert np.allclose(to_depth_metres(fake_image('16UC1', mm)), mm * 0.001, atol=1e-6)      # RealSense: millimetres
    with pytest.raises(ValueError):
        to_rgb(fake_image('mono8', rgb[..., 0]))
    with pytest.raises(ValueError):
        to_depth_metres(fake_image('rgb8', rgb))


# ================================================================ configuration
def shipped():
    p = yaml.safe_load(CFG.read_text())['perception_node']['ros__parameters']
    return p, {c.name: c for c in classes_from_dict(p['class_names'], p['classes'])}


def test_shipped_config_loads_and_matches_the_phase3_colour_contract():
    p, cs = shipped()
    assert set(cs) == {'victim', 'fire'} and p['target_frame'] == 'map' and p['base_frame'] == 'base_link'
    assert cs['victim'].hsv_ranges[0] == ((100, 100, 40), (125, 255, 255))     # docs: victim = blue, hue 109-110
    assert cs['fire'].hsv_ranges[0][1][0] == 30 and cs['fire'].hsv_ranges[0][0][1] == 200
    assert p['max_angular_rate'] <= 0.15, 'the RGB/depth offset limit must stay tight'


def test_class_settings_that_would_match_everything_are_rejected():
    with pytest.raises(ValueError, match='hsv_hi'):
        classes_from_dict(['x'], {'x': {'hsv_lo': [0, 0, 0]}})
    with pytest.raises(ValueError, match='no settings'):
        classes_from_dict(['x'], {})


def test_the_priors_match_the_simulation_models_so_they_cannot_drift_apart():
    """Object-geometry priors are facts about the models. If someone edits the victim or fire model,
    the RGB-only estimate would silently go wrong - so this fails instead."""
    _, cs = shipped()
    sdf = REPO / 'simulation' / 'models'

    def cyl_or_sphere(root, name):
        for tag in ('collision', 'visual'):
            for e in root.iter(tag):
                if e.get('name') == name:
                    z = float(e.findtext('pose').split()[2])
                    g = e.find('geometry')
                    return z, g
        raise AssertionError(name)

    v = ET.parse(sdf / 'victim' / 'model.sdf').getroot()
    zt, gt = cyl_or_sphere(v, 'torso')
    zh, gh = cyl_or_sphere(v, 'head')
    torso_r, torso_len = float(gt.findtext('cylinder/radius')), float(gt.findtext('cylinder/length'))
    head_top = zh + float(gh.findtext('sphere/radius'))
    assert cs['victim'].top_height_m == pytest.approx(head_top, abs=0.005)
    assert cs['victim'].bottom_height_m == pytest.approx(zt - torso_len / 2, abs=0.005)
    assert cs['victim'].axis_offset_m == pytest.approx(torso_r, abs=0.005)

    f = ET.parse(sdf / 'fire' / 'model.sdf').getroot()
    zo, go = cyl_or_sphere(f, 'flame_outer')
    length, radius = float(go.findtext('cone/length')), float(go.findtext('cone/radius'))
    base, apex = zo - length / 2, zo + length / 2
    assert cs['fire'].top_height_m == pytest.approx(apex, abs=0.02)
    assert cs['fire'].bottom_height_m == pytest.approx(base, abs=0.005)
    assert cs['fire'].bottom_axis_offset_m == pytest.approx(radius, abs=0.005)
    centroid_h = base + length / 3                                               # a triangle's centroid
    # Bounded by the geometry (cone radius at the centroid .. at the base) and tuned inside it by measurement:
    # the surface the estimators actually hit is the lower cone (perception_accuracy.py, Phase 5).
    assert radius * (apex - centroid_h) / length <= cs['fire'].axis_offset_m <= radius


def test_nothing_above_perception_reads_the_estimator_that_produced_a_position():
    """Detection.source_backend is provenance for debugging. Cognition must not branch on it
    (Architecture.md section 4)."""
    src = REPO / 'src'
    bad = []
    for f in src.rglob('*.py'):
        if 'fire_resq_perception' in f.parts or '__pycache__' in f.parts:
            continue
        code = re.sub(r'(?m)#.*$', '', f.read_text())
        if 'source_backend' in code:
            bad.append(str(f))
    assert not bad, f'only perception may touch source_backend: {bad}'
