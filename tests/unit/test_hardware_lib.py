"""The hardware package's pure logic: drive model, firmware wire format, camera calibration.

No ROS, no serial port, no camera. The firmware source and the robot config are read as text so the Python side
cannot drift from them.
"""
import math
import re
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
HW = REPO / 'src' / 'fire_resq_hardware'
sys.path.insert(0, str(HW))
from fire_resq_hardware import protocol  # noqa: E402
from fire_resq_hardware.calibration import load_calibration  # noqa: E402
from fire_resq_hardware.drive import DriveModel, MotorMap, integrate  # noqa: E402

FIRMWARE = (HW / 'firmware' / 'base_controller' / 'base_controller.ino').read_text()
ROBOT = yaml.safe_load((HW / 'config' / 'robot.yaml').read_text())
BASE, CAMERA = ROBOT['base_node']['ros__parameters'], ROBOT['camera_node']['ros__parameters']

PLAIN = MotorMap(left=(1, 3), right=(2, 4), signs=(1, 1, 1, 1))


def model(motors=PLAIN, track=0.2, full=0.5, min_power=100):
    return DriveModel(motors, track_m=track, full_speed_mps=full, min_power=min_power)


# ---------------------------------------------------------------- drive model
def test_stop_is_all_zero_and_tiny_commands_do_not_buzz_the_motors():
    m = model()
    assert m.motor_powers(0.0, 0.0) == (0, 0, 0, 0)
    assert m.motor_powers(0.004, 0.0) == (0, 0, 0, 0)            # below 2 % of full speed


def test_forward_drives_every_wheel_forward_and_a_left_turn_reverses_the_left_side():
    m = model()
    assert all(p > 0 for p in m.motor_powers(0.2, 0.0))
    p = m.motor_powers(0.0, 1.0)                                 # +w = counter-clockwise = left
    assert p[0] < 0 and p[2] < 0 and p[1] > 0 and p[3] > 0
    assert p[0] == p[2] == -p[1] == -p[3]


def test_a_moving_wheel_never_gets_less_than_the_start_up_power():
    m = model(min_power=100)
    for v in (0.02, 0.05, 0.1, 0.3, 0.5):
        assert all(100 <= p <= 255 for p in m.motor_powers(v, 0.0)), v
    assert m.motor_powers(0.5, 0.0) == (255, 255, 255, 255)


def test_wiring_map_and_polarity_are_applied_per_terminal():
    front_back = MotorMap(left=(2, 4), right=(1, 3), signs=(1, -1, -1, 1))
    left, right = model(front_back).side_powers(0.2, 0.5)
    assert left < right
    assert model(front_back).motor_powers(0.2, 0.5) == (right, -left, -right, left)


def test_saturation_keeps_the_commanded_curvature():
    m = model(min_power=0)
    left, right = m.side_powers(0.5, 2.0)                        # asks the right side for 0.7 m/s of a 0.5 m/s robot
    assert right == 255 and 0 < left < 255
    assert left / right == pytest.approx((0.5 - 0.2) / (0.5 + 0.2), abs=0.01)


@pytest.mark.parametrize('v,w', [(0.2, 0.0), (0.0, 1.5), (0.15, -0.8), (-0.2, 0.3), (0.0, 0.0)])
def test_believed_motion_of_a_command_is_that_command(v, w):
    """The odometry model is the exact inverse of the command model (up to one power step)."""
    m = model()
    got_v, got_w = m.twist_of(m.motor_powers(v, w))
    step = m.full_speed_mps / (255 - m.min_power)
    assert got_v == pytest.approx(v, abs=step) and got_w == pytest.approx(w, abs=2 * step / m.track_m)


def test_released_motors_are_believed_stationary_whatever_was_commanded():
    assert model().twist_of((0, 0, 0, 0)) == (0.0, 0.0)
    assert model(min_power=100).twist_of((60, 60, 60, 60)) == (0.0, 0.0)      # below start-up power: no motion claimed


@pytest.mark.parametrize('left,right,signs', [((1, 2), (2, 4), (1, 1, 1, 1)), ((1,), (2, 4), (1, 1, 1, 1)), ((), (1, 2, 3, 4), (1, 1, 1, 1)),
                                              ((1, 3), (2, 4), (1, 1, 0, 1)), ((1, 3), (2, 5), (1, 1, 1, 1)), ((1, 3), (2, 4), (1, 1, 1))])
def test_an_impossible_wiring_map_is_refused(left, right, signs):
    with pytest.raises(ValueError):
        MotorMap(left, right, signs)


def test_dead_reckoning_geometry():
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(100):
        x, y, yaw = integrate(x, y, yaw, 0.2, 0.0, 0.05)
    assert (x, y, yaw) == pytest.approx((1.0, 0.0, 0.0))
    x, y, yaw = 0.0, 0.0, 0.0                                    # one full circle of radius 0.5 m comes home
    for _ in range(2000):
        x, y, yaw = integrate(x, y, yaw, 0.5, 1.0, 2 * math.pi / 2000)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-6) and abs(yaw) < 1e-6


# ---------------------------------------------------------------- wire format vs the firmware source
def test_python_and_firmware_agree_on_the_protocol():
    assert int(re.search(r'const int PROTOCOL = (\d+);', FIRMWARE).group(1)) == protocol.PROTOCOL
    assert int(re.search(r'Serial\.begin\((\d+)\)', FIRMWARE).group(1)) == protocol.BAUD
    assert int(re.search(r'const uint8_t MOTORS = (\d+);', FIRMWARE).group(1)) == protocol.MOTORS


def test_the_driver_feeds_the_firmware_watchdog_with_margin():
    """A drive command must be repeated at least twice per watchdog period, or one late packet stops the robot."""
    watchdog_s = int(re.search(r'const unsigned long WATCHDOG_MS = (\d+);', FIRMWARE).group(1)) / 1000.0
    assert 1.0 / BASE['send_rate_hz'] <= watchdog_s / 2
    assert BASE['cmd_timeout_s'] <= 1.0                          # /cmd_vel silence must stop the robot promptly


def test_drive_lines_and_what_is_refused():
    assert protocol.format_drive((120, -120, 0, 255)) == b'M 120 -120 0 255\n'
    assert len(protocol.format_drive((-255,) * 4)) < int(re.search(r'char line\[(\d+)\]', FIRMWARE).group(1))
    for bad in ((256, 0, 0, 0), (0, 0, 0), (0, 0, 0, 0, 0), (1.5, 0, 0, 0)):
        with pytest.raises(ValueError):
            protocol.format_drive(bad)


def test_status_and_banner_parsing_ignores_everything_else():
    assert protocol.parse_status('S 120 -120 0 255') == (120, -120, 0, 255)
    assert protocol.parse_banner('FIRERESQ_BASE 2 wd=400') == 2
    for junk in ('', 'S', 'S 1 2 3', 'S 1 2 3 4 5', 'S a b c d', 'S 300 0 0 0', 'M 1 2 3 4', 'ARM READY'):
        assert protocol.parse_status(junk) is None, junk
    for junk in ('', 'ARM READY', 'FIRERESQ_BASE', 'FIRERESQ_BASE x', 'S 0 0 0 0', 'Press S to start movement.'):
        assert protocol.parse_banner(junk) is None, junk


def test_shipped_wiring_config_is_a_valid_map():
    MotorMap(tuple(BASE['left_motors']), tuple(BASE['right_motors']), tuple(BASE['motor_signs']), tuple(BASE['front_motors']))


# ---------------------------------------------------------------- camera calibration
def test_shipped_calibration_matches_the_configured_camera_mode():
    cal = load_calibration(HW / 'config' / 'camera_c270.yaml')
    assert cal.k[0] == cal.p[0] and cal.k[4] == cal.p[5] and cal.k[2] == cal.p[2] and cal.k[5] == cal.p[6]
    cal.scaled_to(CAMERA['width'], CAMERA['height'])             # raises if the configured mode has another aspect ratio


def test_scaling_a_calibration_and_refusing_a_cropped_mode():
    cal = load_calibration(HW / 'config' / 'camera_c270.yaml')
    half = cal.scaled_to(320, 240)
    assert (half.width, half.height) == (320, 240)
    assert [half.k[i] for i in (0, 2, 4, 5)] == [cal.k[i] / 2 for i in (0, 2, 4, 5)]
    assert half.k[8] == 1.0 and half.p[10] == 1.0 and half.d == cal.d
    with pytest.raises(ValueError, match='aspect ratio'):
        cal.scaled_to(640, 360)


@pytest.mark.parametrize('text', ['[]', 'image_width: 640', 'image_width: 640\nimage_height: 480',
                                  'image_width: 640\nimage_height: 480\ncamera_matrix: {data: [1, 2, 3]}',
                                  'image_width: 640\nimage_height: 480\ncamera_matrix: {data: [0, 0, 1, 0, 0, 1, 0, 0, 1]}'])
def test_a_broken_calibration_file_is_refused(tmp_path, text):
    f = tmp_path / 'cal.yaml'
    f.write_text(text)
    with pytest.raises(ValueError):
        load_calibration(f)


# ---------------------------------------------------------------- the layer rule, from this side
def test_only_the_hardware_package_touches_devices():
    """Hardware.md section 8: device access lives in fire_resq_hardware and nowhere else under src/."""
    for p in (REPO / 'src').rglob('*.py'):
        if HW in p.parents or '__pycache__' in p.parts:
            continue
        assert not re.search(r'^\s*(import|from)\s+(serial|RPi|gpiozero|smbus)\b', p.read_text(), re.M), p
        assert 'VideoCapture' not in p.read_text(), p


# ------------------------------------------------------------------------------------- turning on the spot
def test_every_turn_style_turns_the_right_way_and_stops_the_wheels_it_says_it_stops():
    """M1 front-left, M2 front-right, M3 rear-left, M4 rear-right. Turning LEFT (w > 0), then right."""
    from fire_resq_hardware.drive import TURN_STYLES
    expect_left = {                       # sign of each terminal's power, M1..M4
        'skid': (-1, 1, -1, 1),
        'pivot_back': (-1, 0, -1, 0),     # left side reverses, right side stopped
        'pivot_forward': (0, 1, 0, 1),    # right side drives, left side stopped
        'pivot_right': (0, 1, 0, 1),      # the right side only (a left turn looks like pivot_forward ...)
        'pivot_rear': (0, 0, -1, 1),      # rear wheels only
        'pivot_front': (-1, 1, 0, 0),     # front wheels only
        'pivot_split': (0, 0, -1, 1),     # left with the rear wheels ...
        'mixed': (-1, 1, -1, 1),          # left as skid ...
    }
    assert set(expect_left) == set(TURN_STYLES)
    for style, signs in expect_left.items():
        m = DriveModel(PLAIN, track_m=0.17, full_speed_mps=0.3, min_power=110, turn_style=style)
        left, right = m.motor_powers(0.0, 1.2), m.motor_powers(0.0, -1.2)
        assert tuple((p > 0) - (p < 0) for p in left) == signs, (style, left)
        mirrored = (signs[1], signs[0], signs[3], signs[2])
        if style == 'pivot_right':        # ... and a right turn is the SAME wheels reversed)
            mirrored = (0, -1, 0, -1)
        if style == 'pivot_split':        # ... right with the front wheels
            mirrored = (1, -1, 0, 0)
        if style == 'mixed':              # ... right with the left side driving forward, nothing in reverse
            mirrored = (1, 0, 1, 0)
        assert tuple((p > 0) - (p < 0) for p in right) == mirrored, (style, right)
        assert all(abs(p) >= 110 for p in left if p), 'a driven wheel gets at least the power that moves it'
        assert m.twist_of(left)[1] > 0 > m.twist_of(right)[1], f'{style}: /odom must turn the way the robot is told to'
        assert m.motor_powers(0.0, 0.0) == (0, 0, 0, 0)


def test_a_pivot_asks_one_side_for_twice_the_skid_speed_and_respects_polarity():
    skid = DriveModel(PLAIN, track_m=0.17, full_speed_mps=0.3, min_power=110)
    pivot = DriveModel(PLAIN, track_m=0.17, full_speed_mps=0.3, min_power=110, turn_style='pivot_back')
    assert abs(pivot.motor_powers(0.0, 1.0)[0]) > abs(skid.motor_powers(0.0, 1.0)[0])
    assert pivot.twist_of(pivot.motor_powers(0.0, 1.0))[1] == pytest.approx(1.0, rel=0.03), 'same turn rate as asked'
    assert pivot.twist_of(pivot.motor_powers(0.0, 1.0))[0] < 0, 'pivot_back backs the robot away a little as it turns'
    flipped = MotorMap(left=(1, 3), right=(2, 4), signs=(-1, 1, 1, 1))
    m = DriveModel(flipped, track_m=0.17, full_speed_mps=0.3, min_power=110, turn_style='pivot_back')
    p = m.motor_powers(0.0, 1.0)
    assert p[0] > 0 > p[2] and p[1] == p[3] == 0, 'a reversed motor still runs backward'


def test_turn_style_only_changes_turns_on_the_spot():
    for style in ('skid', 'pivot_back', 'pivot_rear'):
        m = DriveModel(PLAIN, track_m=0.17, full_speed_mps=0.3, min_power=110, turn_style=style)
        straight = m.motor_powers(0.15, 0.0)
        assert len(set(straight)) == 1 and 170 < straight[0] < 195, 'straight is straight in every style'
        curve = m.motor_powers(0.15, 0.6)
        assert 0 < curve[0] == curve[2] < curve[1] == curve[3], 'curving while driving: both sides forward'
    with pytest.raises(ValueError):
        DriveModel(PLAIN, track_m=0.17, full_speed_mps=0.3, min_power=110, turn_style='crab')
    with pytest.raises(ValueError):
        MotorMap(left=(1, 3), right=(2, 4), signs=(1, 1, 1, 1), front=(1, 3))


def test_the_configured_turn_style_exists():
    from fire_resq_hardware.drive import TURN_STYLES
    assert BASE['turn_style'] in TURN_STYLES


def test_the_shipped_wiring_pivots_the_way_it_was_seen_to_on_the_floor():
    """2026-10-13, from the maintainer: M1, M2 front; M3 back-left, M4 back-right. Turns use the back wheels only."""
    motors = MotorMap(tuple(BASE['left_motors']), tuple(BASE['right_motors']), tuple(BASE['motor_signs']), tuple(BASE['front_motors']))
    m = DriveModel(motors, track_m=0.17, full_speed_mps=BASE['full_speed_mps'], min_power=BASE['min_power'], turn_style=BASE['turn_style'])
    sign = lambda powers: tuple((p > 0) - (p < 0) for p in powers)  # noqa: E731
    assert sign(m.motor_powers(0.0, 1.2)) == (1, -1, -1, 1), 'left: M2, M3 (the left wheels) back, M1, M4 forward'
    assert sign(m.motor_powers(0.0, -1.2)) == (-1, 1, 1, -1), 'right: the opposite, all four wheels ...'
    idle = DriveModel(motors, track_m=0.17, full_speed_mps=BASE['full_speed_mps'], min_power=BASE['min_power'],
                      turn_style=BASE['turn_style'], right_turn_idle=tuple(BASE['right_turn_idle']))
    assert sign(idle.motor_powers(0.0, -1.2)) == (-1, 0, 0, -1), '... except that only M1 and M4 are powered in a right turn'
    assert sign(idle.motor_powers(0.0, 1.2)) == (1, -1, -1, 1) and sign(idle.motor_powers(0.12, -0.4)) == (1, 1, 1, 1)
    assert sign(m.motor_powers(0.12, 0.0)) == (1, 1, 1, 1)
