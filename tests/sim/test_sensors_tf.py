"""TF tree, camera contract and sensor geometry - checked against the scenario's known objects.

The projection test is the important one: it finds coloured objects in the RGB image, reads
their depth, back-projects through the camera intrinsics and the TF chain into `odom`, converts
to the world frame, and checks the result lands ON the real object. A wrong optical-frame
rotation, a wrong intrinsic, or a mis-mounted camera fails it (see the negative control).
"""
import math
import subprocess

import numpy as np
import pytest

from simlib import P, class_masks, match_entities

EXPECTED_FRAMES = {'odom', 'base_link', 'left_wheel_link', 'right_wheel_link', 'caster_link',
                   'magnet_link', 'camera_link', 'camera_optical_frame'}


# ---------------------------------------------------------------- TF
@pytest.fixture(scope='module')
def tf_edges(arena):
    arena.bot.tf_dyn.clear()
    arena.bot.spin_wall(6.0)
    return arena.bot.tf_dyn, arena.bot.tf_static


def test_tf_tree_structure(tf_edges):
    dyn, static = tf_edges
    edges = list(dyn) + list(static)
    parents = {}
    for p, c in edges:
        parents.setdefault(c, set()).add(p)
    frames = set(parents) | {p for p, _ in edges}
    assert frames - set(parents) == {'odom'}, 'exactly one root, and it is odom'
    assert all(len(v) == 1 for v in parents.values()), 'every frame has exactly one parent'
    assert frames == EXPECTED_FRAMES
    assert 'map' not in frames, 'map->odom belongs to SLAM (Phase 4); it must not be faked'


def test_tf_dynamic_vs_static_edges(tf_edges):
    dyn, static = tf_edges
    assert set(dyn) == {('odom', 'base_link'), ('base_link', 'left_wheel_link'), ('base_link', 'right_wheel_link')}
    assert static == {('base_link', 'camera_link'), ('base_link', 'caster_link'),
                      ('base_link', 'magnet_link'), ('camera_link', 'camera_optical_frame')}
    ts = dyn[('odom', 'base_link')]
    assert 45 <= len(ts) / (max(ts) - min(ts)) <= 55, 'odom->base_link should be ~50 Hz'


def test_optical_frame_follows_rep103(arena):
    """optical +z (forward) -> base +x ; +x (right) -> -y ; +y (down) -> -z."""
    R, T = arena.bot.cam_to_odom(arena.bot.tfbuf.get_latest_common_time('odom', 'camera_optical_frame'))
    assert np.allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-6)
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-6)
    assert np.allclose(R @ [0, 1, 0], [0, 0, -1], atol=1e-6)
    assert T == pytest.approx([P['camera_x'], 0.0, P['camera_z']], abs=1e-3)   # from parameters.xacro


# ---------------------------------------------------------------- camera contract
def test_camera_message_contract(arena):
    m = arena.bot.msgs
    for k in ('rgb', 'rgb_info', 'depth', 'depth_info'):
        assert m[k].header.frame_id == 'camera_optical_frame', k
    w, h = int(P['cam_width']), int(P['cam_height'])
    assert (m['rgb'].width, m['rgb'].height, m['rgb'].encoding) == (w, h, 'rgb8')
    assert (m['depth'].width, m['depth'].height, m['depth'].encoding) == (w, h, '32FC1')
    fx = w / (2 * math.tan(P['cam_hfov'] / 2))
    K, Kd = np.array(m['rgb_info'].k).reshape(3, 3), np.array(m['depth_info'].k).reshape(3, 3)
    assert K[0, 0] == pytest.approx(fx, abs=1.0) and K[1, 1] == pytest.approx(fx, abs=1.0)
    assert K[0, 2] == pytest.approx(w / 2, abs=1.5) and K[1, 2] == pytest.approx(h / 2, abs=1.5)
    assert np.allclose(K, Kd, atol=0.5), 'RGB and depth must share intrinsics so pixels align'


def test_topic_rates(arena):
    bot = arena.bot
    bot.counts.clear()
    t0 = bot.simt
    bot.wait_for(lambda: bot.simt - t0 >= 6.0, 30, 'rate window')
    rate = {k: v / 6.0 for k, v in bot.counts.items()}
    assert rate['rgb'] >= 15 and rate['depth'] >= 12, rate           # 30 requested; measured 17-26
    assert 45 <= rate['odom'] <= 55, rate
    assert 200 <= rate['joint_states'] <= 300, rate                  # 4 ms physics step -> 250 Hz


def test_no_ground_truth_leaks_into_ros(arena):
    topics = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout.split()
    assert not [t for t in topics if any(s in t for s in ('pose/info', 'ground_truth', 'dynamic_pose'))]
    assert {'/cmd_vel', '/odom', '/joint_states', '/tf', '/camera/image_raw',
            '/camera/depth/image_raw'} <= set(topics)


# ---------------------------------------------------------------- geometry
def test_colour_classes_are_separable(arena):
    """The perception contract: fire, victim and safe zone occupy disjoint hue classes and
    everything else (floor, walls, obstacles, sky, steel ring) is too grey to match any."""
    rgb = arena.bot.rgb_array(arena.bot.msgs['rgb'])
    masks = class_masks(rgb)
    for name in ('fire', 'victim', 'safe_zone'):
        assert masks[name].sum() > 200, f'{name} not visible from the start pose'
    stack = np.stack(list(masks.values())).sum(0)
    assert stack.max() == 1, 'a pixel belongs to two colour classes'
    import cv2
    s = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[..., 1]
    anyclass = np.any(list(masks.values()), axis=0)
    assert s[~anyclass].max() < 60, 'something un-classed is saturated enough to confuse a detector'


def test_back_projection_lands_on_the_real_objects(arena, scenario):
    """pixel + depth + intrinsics + TF -> world, checked against scenario ground truth."""
    bot = arena.bot
    seen_any = []
    for _ in range(6):
        bot.spin_wall(0.4)
        seen_any += bot.detect(scenario)
    assert seen_any, 'nothing detected from the start pose'
    seen, unmatched = match_entities(scenario, seen_any)
    assert {'fire', 'victim_1'} <= seen, f'expected fire and victim_1 in view, saw {seen}'
    assert not unmatched, f'detections that are NOT on any real object: {unmatched}'


def test_x_mirrored_optical_frame_would_fail_the_projection(arena, scenario):
    """Negative control: proves the projection test can fail, i.e. it is really checking geometry."""
    bot = arena.bot
    dets = []
    for _ in range(4):
        bot.spin_wall(0.4)
        dets += bot.detect(scenario)
    s = scenario.robot_start
    mirrored = []
    for cls, wx, wy, z, a in dets:                         # reflect each point across the robot's forward axis
        dx, dy = wx - s.x, wy - s.y
        c, sn = math.cos(-s.yaw), math.sin(-s.yaw)
        lx, ly = dx * c - dy * sn, dx * sn + dy * c
        ly = -ly
        c2, s2 = math.cos(s.yaw), math.sin(s.yaw)
        mirrored.append((cls, s.x + lx * c2 - ly * s2, s.y + lx * s2 + ly * c2, z, a))
    seen, unmatched = match_entities(scenario, mirrored)
    assert unmatched, 'a mirrored frame should put detections off the objects'
