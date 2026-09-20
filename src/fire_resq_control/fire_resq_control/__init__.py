"""Control - the stable ROS-level boundary above the drivers.

    Responsibility (Hardware.md section 8, Implementation_Plan.md section 12):
        ATTACH/RELEASE  ->  MagnetBackend  ->  simulation topic OR ESP32 GPIO
        cmd_vel / odom plumbing and limits

    This is the layer that makes sim-to-hardware a driver swap. The magnet node is
    written against a backend interface from the start precisely so Phase 12 adds a
    class instead of editing a node.

    No GPIO access here - that belongs in fire_resq_hardware.

    Implemented in Phase 9. Empty by design until then.

    TODO(phase-9): MagnetBackend interface + SimMagnetBackend (Gazebo detachable joint).
    TODO(phase-9): magnet_node exposing SetMagnet and publishing MagnetState.
    TODO(phase-12): Esp32MagnetBackend behind the same interface.
    TODO(hardware): velocity limits matched to real motor capability.
"""
