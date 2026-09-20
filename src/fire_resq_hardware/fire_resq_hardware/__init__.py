"""Hardware layer - the ONLY package permitted to know about physical devices.

    Responsibility (Hardware.md sections 3 and 8):
        laptop ROS 2  <->  ESP32  ->  motor driver / encoders / electromagnet

    Architecture rule: cognition and planning must never import from this package.
    A guard test in Phase 10 asserts exactly that.

    Implemented in Phase 12, and only after the simulation MVP is complete.
    Empty by design until then.

    TODO(phase-12): esp32_bridge_node - cmd_vel to serial, encoder ticks to odom + TF.
    TODO(phase-12): ESP32 firmware (motor PWM/direction, encoder ISR, magnet switching).
    TODO(phase-12): Esp32MagnetBackend implementing the fire_resq_control interface.
    TODO(hardware): encoder calibration - ticks/rev, wheel radius, wheel separation.
    TODO(hardware): physical motor controller configuration and current limits.
    TODO(hardware): watchdog / failsafe. BLOCKING for any autonomous physical test
                    (Hardware_Integration.md).
"""
