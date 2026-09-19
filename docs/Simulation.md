# FireResQ — Simulation Plan

## Environment
Initial target: approximately 5 m × 5 m indoor rescue arena.

Include:
- Walls.
- Three victim models.
- Fire model.
- Safe zone.
- Obstacles.
- Differential-drive robot.

## Software
Target:
- Ubuntu 24.04.
- ROS 2 Jazzy.
- Gazebo.
- RViz.
- Nav2.
- SLAM Toolbox or another suitable ROS 2 SLAM solution.
- TF2.
- OpenCV.
- Suitable YOLO-family detector where practical.

## Simulation Stages
1. Robot spawns and moves with `/cmd_vel`.
2. Simulated camera/depth and odometry work.
3. SLAM builds a map and localization works.
4. Perception produces fire/victim/obstacle detections.
5. World model receives detections.
6. Decision engine selects a victim dynamically.
7. Nav2 moves to it.
8. Simulated electromagnet attaches/releases the victim.
9. Robot returns to safe zone.
10. World model marks victim rescued and selects the next target.

## Principle
Use deterministic simulation data for early testing where useful, but keep interfaces realistic enough that physical RGB/RGB-D sensors can replace simulated sensors.

TODO: realistic sensor noise.
TODO: perception uncertainty.
TODO: dynamic obstacles.
TODO: failed detection/navigation recovery.
