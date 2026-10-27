"""Hardware layer - the ONLY package permitted to know about physical devices.

    Responsibility (Hardware.md sections 3 and 8):
        PC (station)  <-ROS 2 over Wi-Fi->  Raspberry Pi 3B  <-serial->  Arduino Uno + L293D shield -> 4 motors
                                                   `- USB camera (Logitech C270)

    Architecture rule: cognition and planning must never import from this package (unit-guarded).

    Robot side (launch/robot.launch.py, runs on the Pi):
        camera_node          the camera's own MJPEG frames -> /camera/image_raw/compressed + /camera/camera_info
        base_node            /cmd_vel -> firmware/base_controller; open-loop /odom + TF odom->base_link
        motor_check          one-off wiring check (which shield terminal is which wheel)
    Station side (launch/station.launch.py, runs on the PC):
        image_decoder_node   /camera/image_raw/compressed -> /camera/image_raw, what perception reads
    Pure, unit-tested: drive.py (skid-steer model, dead reckoning), protocol.py (firmware wire format),
        calibration.py (camera calibration file).

    TODO(hardware): a localisation source. There are no encoders and no depth camera, so /odom is dead reckoning
                    from commanded power and nothing provides `map` yet (see drive.py).
    TODO(hardware): the pickup actuator and its driver (/fire_resq/magnet/energize, /fire_resq/magnet/state).
    TODO(hardware): measure the placeholders in config/robot.yaml and calibrate the camera.
"""
