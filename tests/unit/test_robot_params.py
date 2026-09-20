"""Navigation reads the robot's footprint and limits from parameters.xacro - never its own copy."""
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src' / 'fire_resq_navigation'))
from fire_resq_navigation.robot_params import (  # noqa: E402
    convex_hull, load_robot_params, read_properties)

XACRO = REPO / 'src' / 'fire_resq_description' / 'urdf' / 'parameters.xacro'


@pytest.fixture(scope='module')
def rp():
    return load_robot_params(XACRO)


def test_limits_come_straight_from_the_description(rp):
    p = read_properties(XACRO)
    assert rp.max_linear_velocity == p['max_linear_velocity']
    assert rp.max_angular_velocity == p['max_angular_velocity']
    assert rp.max_linear_acceleration == p['max_linear_acceleration']
    assert rp.wheel_separation == p['wheel_separation'] and rp.wheel_radius == p['wheel_radius']


def test_footprint_is_a_ccw_convex_polygon_containing_every_part(rp):
    fp = rp.footprint
    n = len(fp)
    assert n >= 6
    for i in range(n):                                    # every turn is a left turn => convex, CCW
        (x1, y1), (x2, y2), (x3, y3) = fp[i], fp[(i + 1) % n], fp[(i + 2) % n]
        assert (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2) > 0
    p = read_properties(XACRO)
    xs, ys = [q[0] for q in fp], [q[1] for q in fp]
    assert min(xs) == pytest.approx(p['chassis_x_offset'] - p['chassis_length'] / 2, abs=1e-3)   # rear of chassis
    assert max(xs) == pytest.approx(p['magnet_x'] + p['magnet_length'] / 2, abs=1e-3)            # magnet is the front
    assert max(ys) == pytest.approx(p['wheel_separation'] / 2 + p['wheel_width'] / 2, abs=1e-3)  # wheels are the widest


def test_radii_are_consistent(rp):
    assert 0 < rp.inscribed_radius < rp.circumscribed_radius
    assert rp.circumscribed_radius < 0.25                 # small robot; a TurtleBot default (0.22) would be wrong


def test_footprint_string_is_nav2_format(rp):
    s = rp.footprint_string()
    assert re.fullmatch(r'\[(\[-?\d+\.\d{3}, -?\d+\.\d{3}\](, )?)+\]', s), s


def test_changing_the_description_changes_the_footprint(tmp_path):
    """The linkage that matters: edit parameters.xacro and navigation follows."""
    copy = tmp_path / 'parameters.xacro'
    copy.write_text(XACRO.read_text().replace('name="wheel_separation" value="0.17"', 'name="wheel_separation" value="0.30"'))
    wide = load_robot_params(copy)
    assert max(y for _, y in wide.footprint) == pytest.approx(0.15 + 0.013, abs=1e-3)
    assert wide.circumscribed_radius > load_robot_params(XACRO).circumscribed_radius


def test_convex_hull_basics():
    sq = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.2, 0.8)]
    assert sorted(convex_hull(sq)) == [(0, 0), (0, 1), (1, 0), (1, 1)]
