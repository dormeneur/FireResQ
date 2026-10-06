"""Phase 9: the electromagnet abstraction. Judged against Gazebo's TRUE pose, not just the
SetMagnet response - a service that says "attached" while the joint never actually formed would
pass every topic-level check and still leave Phase 10 building on a false assumption (this is
exactly why this module exists: see docs/Implementation_Plan.md Phase 9).

Positions are TELEPORTED into contact (simlib.teleport), not driven with cmd_vel: Nav2 goals end
6-14 cm from where they were aimed (Phase 7, measured) and the magnet's own contact tolerance is
2 cm, so an approach precise enough to test attach/refuse needs ground truth, not a live planner.
That is a test-only shortcut - ground truth is never fed back into a ROS topic - and Phase 10's
own ALIGN creep is exactly the open-loop problem of closing this gap for real.
"""
import math
import time

import rclpy
from fire_resq_description.geometry import read_properties
from fire_resq_interfaces.msg import MagnetState
from fire_resq_interfaces.srv import SetMagnet
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from simlib import WHEEL_R, entity_poses, teleport

from fire_resq_simulation.magnet_geometry import magnet_contact_distance_m

PROPS = read_properties()
CONTACT_M = magnet_contact_distance_m(PROPS['magnet_x'], PROPS['magnet_length'])
_LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def call_set_magnet(env, attach, timeout=6.0):
    fut = env.set_magnet.call_async(SetMagnet.Request(attach=attach))
    rclpy.spin_until_future_complete(env.bot, fut, timeout_sec=timeout)
    assert fut.done(), 'SetMagnet did not answer in time'
    return fut.result()


def teleport_and_settle(x, y, yaw, timeout=10.0):
    """teleport() plus confirmation that ground truth actually reflects it: sim_magnet_bridge.py
    reads pose on its OWN independent stream, and a slow host can leave it briefly stale (the same
    class of CLI/transport slowness CLAUDE.md documents elsewhere) - calling SetMagnet against a
    stale robot pose would test the race, not the magnet."""
    teleport('fire_resq', x, y, yaw, z=WHEEL_R)
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = entity_poses(['fire_resq'])['fire_resq']
        if math.hypot(p[0] - x, p[1] - y) < 0.02:
            return
        time.sleep(0.3)
    raise AssertionError(f'fire_resq never settled at the teleported pose ({x:.3f}, {y:.3f})')


def place_at(env, victim_id, gap_m, bearing=0.0):
    """Teleport the robot so its magnet face is `gap_m` from `victim_id`'s ring surface (0 =
    exactly flush, negative = past it), approaching from `bearing` radians (0 = the victim's
    -x side, robot facing +x - the ring is radially symmetric, so any bearing is equally valid;
    Phase 4 goal tolerance and Phase 8's approach pose both already rely on this)."""
    vx, vy, *_ = entity_poses([victim_id])[victim_id]
    d = CONTACT_M + gap_m
    teleport_and_settle(vx - d * math.cos(bearing), vy - d * math.sin(bearing), bearing)
    return vx, vy


def latest_state(env, timeout=5.0):
    box = {}
    sub = env.bot.create_subscription(MagnetState, '/fire_resq/magnet/state', lambda m: box.setdefault('m', m), _LATCHED)
    env.bot.wait_for(lambda: 'm' in box, timeout=timeout, what='a MagnetState')
    env.bot.destroy_subscription(sub)
    return box['m']


# ---------------------------------------------------------------- bring-up
def test_no_victim_starts_attached(magnetsim, scenario):
    """KNOWN GAZEBO BEHAVIOUR (measured, Phase 9): every declared DetachableJoint instance starts
    ATTACHED by default. sim_magnet_bridge must detach all of them before this fixture returns."""
    poses = entity_poses([v.id for v in scenario.victims])
    for v in scenario.victims:
        vx, vy, *_ = poses[v.id]
        d = math.hypot(vx - v.x, vy - v.y)
        assert d < 0.01, f'{v.id} is {d * 100:.1f} cm from its scenario spawn pose - started attached?'


def test_the_magnet_interface_is_available(magnetsim):
    assert magnetsim.set_magnet.service_is_ready()
    s = latest_state(magnetsim)
    assert s.energized is False and s.attached is False and s.victim_id == ''


# ---------------------------------------------------------------- refusal
def test_attach_is_refused_with_no_victim_in_range(magnetsim, scenario):
    v = scenario.victims[0]
    place_at(magnetsim, v.id, gap_m=0.5)                          # well outside the 2 cm tolerance
    magnetsim.bot.stop(0.5)
    res = call_set_magnet(magnetsim, True)
    print(f'MEASURED refused attach: success={res.success}, state={res.state}')
    assert res.success is False
    assert res.state.energized is True and res.state.attached is False and res.state.victim_id == ''
    assert call_set_magnet(magnetsim, False).success is True         # de-energize for the next test


# ---------------------------------------------------------------- attach, tow, identity
def test_attach_succeeds_in_contact_range_and_the_carried_victim_tows_while_the_others_dont(magnetsim, scenario):
    target = scenario.victims[0]
    others = [v.id for v in scenario.victims if v.id != target.id]
    place_at(magnetsim, target.id, gap_m=-0.01)
    magnetsim.bot.stop(0.5)

    res = call_set_magnet(magnetsim, True)
    print(f'MEASURED attach: success={res.success}, state={res.state}')
    assert res.success is True
    assert res.state.attached is True and res.state.victim_id == target.id

    before = entity_poses([target.id] + others)
    magnetsim.bot.spin_sim(2.5, vx=0.2)
    magnetsim.bot.stop(0.5)
    after = entity_poses([target.id] + others)

    towed = math.hypot(after[target.id][0] - before[target.id][0], after[target.id][1] - before[target.id][1])
    print(f'MEASURED tow distance: {towed:.3f} m')
    assert towed > 0.15, f'the carried victim barely moved ({towed * 100:.1f} cm) - is it really attached?'
    for o in others:
        drift = math.hypot(after[o][0] - before[o][0], after[o][1] - before[o][1])
        assert drift < 0.01, f'{o} moved {drift * 100:.1f} cm though it was never attached - duplicate association?'

    assert call_set_magnet(magnetsim, False).success is True         # leave it released for the next test


def test_a_second_attach_while_already_carrying_one_victim_does_not_grab_another(magnetsim, scenario):
    a, b = scenario.victims[0], scenario.victims[1]
    place_at(magnetsim, a.id, gap_m=-0.01)
    magnetsim.bot.stop(0.5)
    first = call_set_magnet(magnetsim, True)
    assert first.success is True and first.state.victim_id == a.id

    # drive to b WHILE STILL CARRYING a, then ask to attach again
    bx, by, *_ = entity_poses([b.id])[b.id]
    teleport_and_settle(bx - CONTACT_M, by, 0.0)
    magnetsim.bot.stop(0.5)
    second = call_set_magnet(magnetsim, True)
    print(f'MEASURED second attach request while carrying {a.id}: {second}')
    # a is towed along wherever the robot goes (including next to b), so a and b are close together
    # by construction here - the actual claim is about IDENTITY, not position: still exactly a, never b.
    assert second.success is True and second.state.attached is True
    assert second.state.victim_id == a.id, f'a second attach grabbed {second.state.victim_id!r} too'

    assert call_set_magnet(magnetsim, False).success is True


# ---------------------------------------------------------------- release
def test_release_drops_the_victim_in_place_and_the_robot_can_still_drive(magnetsim, scenario):
    v = scenario.victims[2]
    place_at(magnetsim, v.id, gap_m=-0.01)
    magnetsim.bot.stop(0.5)
    assert call_set_magnet(magnetsim, True).success is True
    magnetsim.bot.spin_sim(1.0, vx=0.2)                            # tow it a little way off its spawn pose first
    magnetsim.bot.stop(0.5)

    before = entity_poses([v.id])[v.id]
    res = call_set_magnet(magnetsim, False)
    print(f'MEASURED release: success={res.success}, state={res.state}')
    assert res.success is True and res.state.attached is False and res.state.victim_id == ''
    magnetsim.bot.stop(0.5)
    just_released = entity_poses([v.id])[v.id]
    assert math.hypot(just_released[0] - before[0], just_released[1] - before[1]) < 0.01, \
        'the victim moved on release alone, before the robot moved'

    r0 = magnetsim.bot.pose()
    magnetsim.bot.spin_sim(2.0, vx=-0.2)                           # away from the victim it just set down
    magnetsim.bot.stop(0.5)
    r1 = magnetsim.bot.pose()
    moved = math.hypot(r1[0] - r0[0], r1[1] - r0[1])
    print(f'MEASURED robot travel after release: {moved:.3f} m')
    assert moved > 0.2, 'the robot could not drive after releasing the victim'

    still = entity_poses([v.id])[v.id]
    assert math.hypot(still[0] - just_released[0], still[1] - just_released[1]) < 0.01, \
        'the released victim moved on its own while the robot drove away'


def test_a_carried_victim_is_held_off_the_floor_and_never_pins_the_robot(magnetsim, scenario):
    """Phase 10 regression. A weld made while the victim rests on the floor freezes it a fraction of a millimetre INTO the
    floor's soft contact; the solver's push-out, passed to the robot through the weld, pinned robot and victim together
    (measured: 13 of 29 attach-then-turn trials, every victim, the robot could not turn - while its odometry still said it
    did). sim_magnet_bridge.py now lifts the victim and only counts a weld made clear of the floor (0 of 30 after)."""
    target = scenario.victims[1]
    turned = []
    for i in range(6):
        for v in scenario.victims:                       # earlier tests leave victims wherever they released them
            teleport(v.id, v.x, v.y, v.yaw, z=0.0)
        magnetsim.bot.stop(0.5)
        place_at(magnetsim, target.id, gap_m=0.005, bearing=0.6 + 0.05 * i)
        magnetsim.bot.stop(1.0)
        assert call_set_magnet(magnetsim, True).state.attached
        magnetsim.bot.stop(0.5)
        p1 = entity_poses(['fire_resq', target.id])
        magnetsim.bot.spin_sim(1.5, wz=0.5)
        magnetsim.bot.stop(0.5)
        p2 = entity_poses(['fire_resq'])
        dyaw = abs(math.atan2(math.sin(p2['fire_resq'][5] - p1['fire_resq'][5]), math.cos(p2['fire_resq'][5] - p1['fire_resq'][5])))
        turned.append((round(p1[target.id][2] * 1000, 1), round(dyaw, 2)))
        assert call_set_magnet(magnetsim, False).success
        magnetsim.bot.stop(0.5)
    print(f'MEASURED (victim height mm while held, robot turn rad) per trial: {turned}')
    assert all(z >= 1.0 for z, _ in turned), f'a victim was welded resting on the floor: {turned}'
    assert all(dyaw > 0.5 for _, dyaw in turned), f'the robot could not turn while carrying (commanded 0.75 rad): {turned}'
