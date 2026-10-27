# STATION image - the PC. ROS 2 Jazzy desktop + everything the workspace needs (the PC's own OS has no Jazzy
# packages). The repository is bind-mounted at /ws and built there; nothing of it is baked in.
FROM osrf/ros:jazzy-desktop

# Real-robot station + unit/node tests.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv python3-serial python3-pytest \
        ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-vision-msgs ros-jazzy-nav2-msgs \
        ros-jazzy-tf2-ros-py ros-jazzy-teleop-twist-keyboard ros-jazzy-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

# Navigation + simulation (Nav2, SLAM Toolbox, Gazebo Harmonic): the sim regression suite. Large.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
        ros-jazzy-depthimage-to-laserscan ros-jazzy-ros-gz ros-jazzy-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
