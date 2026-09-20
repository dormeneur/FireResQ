"""Robot physics and odometry against Gazebo ground truth.

Regression guards for three real bugs found in Phase 2 (each of which odometry ALONE could not
see, because odometry is derived from wheel-joint motion and happily reports a robot that is
not moving):
  * robot rested on its magnet, not its wheels     -> test_rests_on_its_wheels, test_forward_*
  * base_link was not at the drive-axle midpoint   -> test_spin_in_place_does_not_translate
  * flat-cylinder wheel contact over-rotated 18 %  -> test_spin_*  (truth yaw == odom yaw)

Tests run in definition order on ONE simulation; the robot wanders a little near its start
(the safe zone is clear of every wall and obstacle by more than any test drives).
"""
import math

import pytest

from simlib import MAX_ANG, MAX_LIN, SEP, WHEEL_R, wrap


def fwd(a, b, heading):
    return (b[0] - a[0]) * math.cos(heading) + (b[1] - a[1]) * math.sin(heading)


def run(env, vx, wz, dur):
    """Command for `dur` sim-seconds, coast, and return truth-vs-odom deltas."""
    bot, gt = env.bot, env.gt
    bot.peak_v = bot.peak_w = 0.0
    g0, o0, j0, mark = gt.get(), bot.pose(), dict(bot.js), gt.mark()
    bot.spin_sim(dur, vx, wz)
    bot.stop(2.0)
    g1, o1, j1, hist = gt.get(), bot.pose(), dict(bot.js), gt.since(mark)
    return dict(
        gt_dist=fwd(g0, g1, g0[5]), odom_dist=fwd(o0, o1, o0[2]),
        gt_moved=math.hypot(g1[0] - g0[0], g1[1] - g0[1]),
        gt_yaw=math.degrees(wrap(g1[5] - g0[5])), odom_yaw=math.degrees(wrap(o1[2] - o0[2])),
        dl=j1['left_wheel_joint'] - j0['left_wheel_joint'], dr=j1['right_wheel_joint'] - j0['right_wheel_joint'],
        max_tilt=max(max(abs(math.degrees(h[3])), abs(math.degrees(h[4]))) for h in hist),
        z=(min(h[2] for h in hist), max(h[2] for h in hist)),
        peak_v=bot.peak_v, peak_w=bot.peak_w)


def assert_odometry_matches_truth(r, allow_translation=True):
    assert abs(r['odom_dist'] - r['gt_dist']) <= 0.01 + 0.03 * abs(r['gt_dist']), r
    assert abs(r['odom_yaw'] - r['gt_yaw']) <= 2.0 + 0.03 * abs(r['gt_yaw']), r
    assert r['max_tilt'] < 1.0, r


def test_rests_on_its_wheels(arena):
    """The wheels - not the magnet or anything else - must carry the robot."""
    gt, bot = arena.gt, arena.bot
    m = gt.mark()
    bot.stop(4.0)
    hist = gt.since(m)
    zs = [h[2] for h in hist]
    assert max(zs) <= WHEEL_R + 1e-4, f'axle z {max(zs):.5f} > wheel radius {WHEEL_R}: robot is propped up by something else'
    assert min(zs) > WHEEL_R - 2e-3, 'robot is sinking into the floor'
    assert max(max(abs(h[3]), abs(h[4])) for h in hist) < math.radians(0.5)
    assert math.hypot(hist[-1][0] - hist[0][0], hist[-1][1] - hist[0][1]) < 0.005


def test_forward(arena):
    r = run(arena, 0.2, 0.0, 3.0)
    assert_odometry_matches_truth(r)
    assert r['gt_dist'] == pytest.approx(0.6, abs=0.03)
    for w in (r['dl'], r['dr']):                       # kinematics come from parameters.xacro
        assert w == pytest.approx(0.6 / WHEEL_R, rel=0.05)
    assert r['z'][1] <= WHEEL_R + 1e-4


def test_backward(arena):
    r = run(arena, -0.2, 0.0, 3.0)
    assert_odometry_matches_truth(r)
    assert r['gt_dist'] == pytest.approx(-0.6, abs=0.03)
    for w in (r['dl'], r['dr']):
        assert w == pytest.approx(-0.6 / WHEEL_R, rel=0.05)


@pytest.mark.parametrize('sign', [1, -1], ids=['ccw', 'cw'])
def test_spin_in_place(arena, sign):
    angle = math.pi / 2
    r = run(arena, 0.0, 1.0 * sign, angle)
    assert_odometry_matches_truth(r)
    assert r['gt_yaw'] == pytest.approx(sign * 90.0, abs=5.0)
    expect = sign * (angle * SEP / 2) / WHEEL_R                       # left -, right + for CCW
    assert r['dl'] == pytest.approx(-expect, rel=0.05)
    assert r['dr'] == pytest.approx(expect, rel=0.05)


def test_spin_in_place_does_not_translate(arena):
    """base_link must be the drive-axle midpoint. Anywhere else, a spin drags it sideways
    (Phase 2: a 4 cm offset gave 5 cm of phantom motion per 90 degrees)."""
    r = run(arena, 0.0, 1.0, math.pi)
    assert r['gt_moved'] < 0.01, f"base_link moved {r['gt_moved']*100:.1f} cm during an in-place spin"
    assert abs(r['gt_yaw'] - r['odom_yaw']) <= 2.0 + 0.03 * abs(r['gt_yaw']), \
        'truth and odometry disagree on rotation: wheel contact width is distorting the track'
    run(arena, 0.0, -1.0, math.pi)                                    # undo


def test_arc(arena):
    r = run(arena, 0.2, 0.5, 2.0)
    assert_odometry_matches_truth(r)
    assert r['gt_yaw'] == pytest.approx(math.degrees(0.5 * 2.0), abs=5.0)


def test_linear_speed_is_capped(arena):
    r = run(arena, 1.0, 0.0, 2.0)
    assert r['peak_v'] <= MAX_LIN * 1.03, r
    assert_odometry_matches_truth(r)


def test_angular_speed_is_capped(arena):
    r = run(arena, 0.0, 5.0, 1.0)
    assert r['peak_w'] <= MAX_ANG * 1.03, r
    assert_odometry_matches_truth(r)


def test_stops_on_zero_command(arena):
    arena.bot.spin_sim(1.0)
    v = arena.bot.odom.twist.twist
    assert abs(v.linear.x) < 0.01 and abs(v.angular.z) < 0.02
