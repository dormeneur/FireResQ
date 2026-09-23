"""Control - the stable ROS-level boundary above the drivers.

    Responsibility (Hardware.md section 8, Implementation_Plan.md section 12):
        ATTACH/RELEASE  ->  MagnetBackend  ->  simulation topic OR ESP32 GPIO
        cmd_vel / odom plumbing and limits

    This is the layer that makes sim-to-hardware a driver swap - but the swap point is the ROS
    TOPIC boundary, not a Python class: magnet_node (magnet_node.py) exposes SetMagnet and
    publishes MagnetState, backed by RosTopicMagnetBackend (magnet_backend.py), which itself only
    ever talks to two standard topics (/fire_resq/magnet/energize, /fire_resq/magnet/state) -
    exactly the shape cmd_vel/odom already use. Nothing in this package imports Gazebo or GPIO
    code; the driver on the other end of those two topics is simulation-specific
    (simulation/scripts/sim_magnet_bridge.py, fire_resq_simulation) today and will be
    hardware-specific (fire_resq_hardware's ESP32 bridge) from Phase 12 - each is a new NODE, not
    a new class here.

    Implemented (Phase 9): MagnetBackend ABC + RosTopicMagnetBackend, magnet_node.

    TODO(phase-12): the ESP32 bridge (fire_resq_hardware) implementing the same two topics on
    real GPIO, plus the cmd_vel watchdog gz_drive.xacro already calls out.
    TODO(hardware): velocity limits matched to real motor capability.
"""
