"""The Phase 9 magnet contact geometry: pure math, no ROS, no simulator - and the recursive xacro
macro that turns a scenario's victim list into one DetachableJoint plugin per victim."""
import math
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'simulation'))
sys.path.insert(0, str(REPO / 'src' / 'fire_resq_description'))
from fire_resq_description.geometry import read_properties  # noqa: E402
from fire_resq_simulation.magnet_geometry import (  # noqa: E402
    CONTACT_TOLERANCE_M, VICTIM_RING_RADIUS_M, contact_gap_m, is_in_contact, magnet_contact_distance_m,
    magnet_face_world_xy)

PROPS = read_properties()


def test_the_ring_radius_constant_matches_the_victim_model():
    text = (REPO / 'simulation/models/victim/model.sdf').read_text()
    radius = float(re.search(r'<cylinder><radius>([\d.]+)</radius><length>0.055</length>', text)[1])
    assert VICTIM_RING_RADIUS_M == pytest.approx(radius)


def test_contact_distance_matches_the_documented_0_131_m():
    """The same number Phase 7/8's standoff-feasibility guard derives independently
    (tests/unit/test_cognition_cache.py) - both must agree, since they describe the same robot."""
    d = magnet_contact_distance_m(PROPS['magnet_x'], PROPS['magnet_length'])
    assert d == pytest.approx(0.131, abs=1e-3)


def test_the_magnet_face_is_ahead_of_base_link_along_whatever_the_robot_faces():
    reach = PROPS['magnet_x'] + PROPS['magnet_length'] / 2
    fx, fy = magnet_face_world_xy((1.0, 2.0), 0.0, PROPS['magnet_x'], PROPS['magnet_length'])
    assert (fx, fy) == pytest.approx((1.0 + reach, 2.0))
    fx, fy = magnet_face_world_xy((0.0, 0.0), math.pi / 2, PROPS['magnet_x'], PROPS['magnet_length'])
    assert (fx, fy) == pytest.approx((0.0, reach), abs=1e-9)


def test_the_gap_is_zero_at_exactly_the_contact_distance_from_any_bearing():
    d = magnet_contact_distance_m(PROPS['magnet_x'], PROPS['magnet_length'])
    for bearing in (0.0, math.pi / 3, math.pi, -math.pi / 2):
        rx = -d * math.cos(bearing)
        ry = -d * math.sin(bearing)
        gap = contact_gap_m((rx, ry), bearing, (0.0, 0.0), PROPS['magnet_x'], PROPS['magnet_length'])
        assert gap == pytest.approx(0.0, abs=1e-6), f'bearing {bearing}: gap {gap}'


def test_is_in_contact_respects_the_tolerance_boundary():
    d = magnet_contact_distance_m(PROPS['magnet_x'], PROPS['magnet_length'])
    just_inside = (-(d + CONTACT_TOLERANCE_M - 0.005), 0.0)
    just_outside = (-(d + CONTACT_TOLERANCE_M + 0.005), 0.0)
    assert is_in_contact(just_inside, 0.0, (0.0, 0.0), PROPS['magnet_x'], PROPS['magnet_length'])
    assert not is_in_contact(just_outside, 0.0, (0.0, 0.0), PROPS['magnet_x'], PROPS['magnet_length'])


def test_a_victim_two_metres_away_is_never_in_contact():
    assert not is_in_contact((0.0, 0.0), 0.0, (2.0, 0.0), PROPS['magnet_x'], PROPS['magnet_length'])


# ------------------------------------------------------------------------------ xacro rendering
XACRO = REPO / 'simulation' / 'urdf' / 'gz_magnet.xacro'


def _render(victim_names: str) -> str:
    import xacro
    # gz_magnet.xacro only defines a macro; invoke it directly rather than editing the robot file.
    wrapper = REPO / 'simulation' / 'urdf' / '__test_wrapper__.xacro'
    wrapper.write_text(
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="w">\n'
        f'  <xacro:include filename="{XACRO}"/>\n'
        f'  <xacro:victim_magnet_plugins names="{victim_names}"/>\n'
        '</robot>\n')
    try:
        doc = xacro.process_file(str(wrapper))
        return doc.toxml()
    finally:
        wrapper.unlink()


def test_empty_victim_names_produces_no_plugin_blocks():
    assert _render('').count('DetachableJoint') == 0


def test_one_plugin_block_per_victim_name_in_order():
    xml = _render('victim_1 victim_2 victim_3')
    assert xml.count('DetachableJoint') == 3
    for name in ('victim_1', 'victim_2', 'victim_3'):
        assert f'<child_model>{name}</child_model>' in xml
        assert f'/model/fire_resq/magnet/{name}/attach' in xml
        assert f'/model/fire_resq/magnet/{name}/detach' in xml
        assert f'/model/fire_resq/magnet/{name}/state' in xml


def test_the_plugin_anchors_on_base_link_not_magnet_link():
    """Measured, Phase 9: spawning the robot from URDF lumps every fixed-joint child link
    (magnet_link included) into base_link, so a DetachableJoint naming magnet_link fails to
    initialise ("Link with name magnet_link not found"). See gz_magnet.xacro for the full story."""
    xml = _render('victim_1')
    assert '<parent_link>base_link</parent_link>' in xml
    assert 'magnet_link' not in xml


def test_a_single_victim_name_with_no_trailing_space_works():
    assert _render('only_one').count('DetachableJoint') == 1
