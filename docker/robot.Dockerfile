# ROBOT image - Raspberry Pi 3B (arm64). ROS 2 Jazzy + the FireResQ drivers (camera, motors, two IR sensors), nothing else.
# Built ON the Pi by docker/deploy_robot.sh (context = the repo subset that script copies over).
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial python3-opencv v4l-utils python3-libgpiod \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/fire_resq_description src/fire_resq_description
COPY src/fire_resq_hardware src/fire_resq_hardware
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select fire_resq_description fire_resq_hardware

COPY docker/entrypoint.sh /entrypoint.sh
# The real robot lives on its own ROS domain: a simulation on the PC (domain 0) can never drive these motors.
ENV ROS_DOMAIN_ID=42
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fire_resq_hardware", "robot.launch.py"]
