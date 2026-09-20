"""Navigation Step 1: depth image -> /scan, and the TF the navigation stack relies on.

The scan is judged against a ray-cast of the TRUE scenario from the robot's TRUE pose: if the
depth->scan pipeline (or its frame conventions) were wrong, or the scan saw the floor, or a sign
were flipped, the error would be tens of centimetres, not millimetres.
"""
import math
import re
import subprocess

import numpy as np
import pytest

from fire_resq_simulation.scenario import raycast
from simlib import P

NAV_HFOV = 1.518     # arena_nav.launch.py default (87 deg); the plain arena launch stays at 60 deg


def stationary(env, seconds=1.0):
    env.bot.stop(0.5)
    env.bot.spin_wall(seconds)


def latest_scan(env):
    n = env.bot.counts['scan']
    env.bot.wait_for(lambda: env.bot.counts['scan'] >= n + 3, 15, 'fresh scans')
    return env.bot.scans[-1]


def expected_ranges(env, scenario, scan):
    g = env.gt.get()
    cx, cy = g[0] + P['camera_x'] * math.cos(g[5]), g[1] + P['camera_x'] * math.sin(g[5])
    ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    return np.array([raycast(scenario, cx, cy, math.cos(g[5] + a), math.sin(g[5] + a)) for a in ang])


# ---------------------------------------------------------------- depth image
def test_depth_image_flows_at_the_navigation_fov(navsim):
    bot = navsim.bot
    assert bot.counts['depth'] > 20 and bot.msgs['depth'].encoding == '32FC1'
    fx = 640 / (2 * math.tan(NAV_HFOV / 2))
    K = np.array(bot.msgs['depth_info'].k).reshape(3, 3)
    assert K[0, 0] == pytest.approx(fx, abs=1.0), 'the navigation launch must run the 87 deg camera'
    assert bot.msgs['depth'].header.frame_id == 'camera_optical_frame'


# ---------------------------------------------------------------- /scan contract
def test_scan_message_contract(navsim):
    s = latest_scan(navsim)
    assert s.header.frame_id == 'camera_link', 'x-forward/z-up frame, so the LaserScan angle convention holds'
    assert len(s.ranges) == 640
    fov = math.degrees(s.angle_max - s.angle_min)
    assert fov == pytest.approx(math.degrees(NAV_HFOV), abs=1.0)
    assert s.angle_increment > 0 and s.angle_min < 0 < s.angle_max
    assert (s.range_min, s.range_max) == (pytest.approx(0.2), pytest.approx(8.0))


def test_scan_rate(navsim):
    bot = navsim.bot
    bot.counts.clear()
    t0 = bot.simt
    bot.wait_for(lambda: bot.simt - t0 >= 6.0, 30, 'rate window')
    # SLAM gates on 0.3 m of travel (about one scan per second at 0.3 m/s), so it needs far less than
    # this; 12 Hz is a floor that flags a stalled or overloaded pipeline. Measured 15-18 Hz under the
    # FULL stack (Environment.md) and ~20 Hz on its own.
    assert bot.counts['scan'] / 6.0 >= 12, f"scan rate {bot.counts['scan'] / 6.0:.1f} Hz"
    assert bot.counts['scan'] <= bot.counts['depth'] * 1.1 + 5, 'more scans than depth frames'
    assert bot.counts['scan'] >= 0.6 * bot.counts['depth'], 'a large share of depth frames produced no scan'


def test_scan_timestamps_are_the_depth_frames_timestamps(navsim):
    """Scans inherit the depth stamp exactly (so TF lookups at the scan time are correct), and
    time is monotonic."""
    bot = navsim.bot
    bot.spin_wall(2.0)
    stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in bot.scans}
    depth = set(bot.depth_stamps)
    # This client subscribes best-effort and can MISS depth frames the scan node received, so demand
    # that nearly all scan stamps are depth stamps rather than every one. A scan built at a different
    # time would match ~none of them (stamps are quantised to the camera period).
    matched = len(stamps & depth) / len(stamps)
    assert stamps and matched >= 0.9, f'only {100 * matched:.0f}% of scan stamps are depth-frame stamps'
    order = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in bot.scans]
    assert order == sorted(order)


# ---------------------------------------------------------------- geometry
@pytest.mark.parametrize('turn_deg', [0, 90, 180, 270], ids=lambda d: f'facing+{d}deg')
def test_scan_matches_a_raycast_of_the_true_scenario(navsim, scenario, turn_deg):
    """Median error < 1 cm and p95 < 5 cm, from four headings (four different views of the room)."""
    if turn_deg:
        navsim.bot.turn_by(math.pi / 2, rate=1.0, settle=1.0)
    stationary(navsim)
    s = latest_scan(navsim)
    r = np.array(s.ranges)
    exp = expected_ranges(navsim, scenario, s)
    finite = np.isfinite(r)
    assert finite.mean() > 0.85, f'only {100 * finite.mean():.0f}% of beams returned'
    both = finite & np.isfinite(exp) & (exp < s.range_max)
    err = np.abs(r[both] - exp[both])
    assert np.median(err) < 0.01, f'median error {np.median(err) * 100:.1f} cm'
    assert np.percentile(err, 95) < 0.05, f'p95 error {np.percentile(err, 95) * 100:.1f} cm'
    assert r[finite].min() >= s.range_min and r[finite].max() <= s.range_max


def test_scan_sees_no_phantom_floor_returns(navsim, scenario):
    """A scan band that is too tall starts to see the FLOOR far away and reports a wall that is
    not there. Every beam that returns must correspond to something real within range."""
    stationary(navsim)
    s = latest_scan(navsim)
    r = np.array(s.ranges)
    exp = expected_ranges(navsim, scenario, s)
    phantom = np.isfinite(r) & ~(exp < s.range_max + 0.05)
    assert phantom.sum() == 0, f'{phantom.sum()} beams returned where nothing real is within range'


# ---------------------------------------------------------------- TF
def test_scan_frame_is_reachable_through_the_navigation_tf_chain(navsim):
    bot = navsim.bot
    s = latest_scan(navsim)
    for target in ('base_link', 'odom'):
        assert bot.tfbuf.can_transform(target, s.header.frame_id, s.header.stamp), \
            f'no {target} <- {s.header.frame_id} transform at the scan time'
    tr = bot.tfbuf.lookup_transform('base_link', 'camera_link', s.header.stamp).transform.translation
    assert (tr.x, tr.y, tr.z) == pytest.approx((P['camera_x'], 0.0, P['camera_z']), abs=1e-3)


def tf_publishers():
    out = subprocess.run(['ros2', 'topic', 'info', '-v', '/tf'], capture_output=True, text=True).stdout
    section = out.split('Subscription count')[0]
    return sorted(set(re.findall(r'Node name:\s*(\S+)', section)))


def test_only_the_expected_nodes_publish_tf(navsim):
    """No duplicate TF publishers: the bridge (odom->base_link) and robot_state_publisher (the
    rest of the robot). The scan node must not publish any TF."""
    assert tf_publishers() == ['bridge_base', 'robot_state_publisher']


def test_every_frame_has_exactly_one_parent(navsim):
    bot = navsim.bot
    bot.tf_dyn.clear()
    bot.spin_wall(4.0)
    parents = {}
    for p, c in list(bot.tf_dyn) + list(bot.tf_static):
        parents.setdefault(c, set()).add(p)
    assert all(len(v) == 1 for v in parents.values()), {c: sorted(v) for c, v in parents.items() if len(v) != 1}
