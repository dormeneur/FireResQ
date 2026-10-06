# FireResQ — Product Requirements Document

## 1. Project Overview
FireResQ is a low-cost cognitive search-and-rescue robot for a controlled indoor fire-emergency scenario. It perceives a mapped environment containing a fire source and multiple victims, reasons about relative danger and rescue cost, selects a rescue target, navigates autonomously, retrieves the victim using an electromagnet, returns the victim to a safe zone, and reassesses the remaining situation.

This is an academic cognitive-robotics prototype, not a real firefighting system.

## 2. Core Cognitive Problem
The robot must not simply follow a predefined path or always rescue the nearest victim. It maintains an internal representation of the environment and makes decisions from imperfect observations.

**Perception → Mapping/Localization → World Model → Reasoning → Decision → Planning → Action → Reassessment**

## 3. Objectives
- Build and validate the complete system in simulation before hardware integration.
- Use SLAM and autonomous navigation as core capabilities.
- Detect fire, victims, and relevant obstacles visually.
- Maintain a world model containing entities, positions, confidence, and rescue state.
- Prioritize victims using risk, accessibility, distance, and estimated rescue cost.
- Navigate to selected victims without predefined rescue paths.
- Simulate and later implement electromagnetic pickup/release.
- Reassess the environment after every rescue.
- Keep camera and hardware interfaces replaceable.
- Leave explicit extension points for uncertainty, Bayesian reasoning, and active perception.

## 4. Minimum Viable System
1. Launch a Gazebo rescue arena.
2. Spawn a differential-drive robot, fire source, three victims, obstacles, and safe zone.
3. Provide simulated camera/depth data and odometry.
4. Detect/identify victims and fire.
5. Build and use a SLAM map.
6. Localize and navigate using ROS 2/Nav2.
7. Maintain a world model.
8. Calculate victim priority dynamically.
9. Select victims without hardcoded rescue ordering.
10. Navigate to the selected victim.
11. Simulate electromagnetic pickup.
12. Return to the safe zone.
13. Release the victim.
14. Update the world model.
15. Repeat for remaining victims.

## 5. Cognitive Extensions
These are not required for the first MVP but must have explicit TODOs in the codebase:
- TODO: perception confidence and uncertainty propagation.
- TODO: Bayesian belief updates.
- TODO: active perception/information-gathering actions.
- TODO: dynamic fire-risk estimation.
- TODO: battery/resource-aware decision-making.
- TODO: compare alternative decision algorithms.
- TODO: recovery from failed detection, pickup, or navigation.

## 6. Environment
Initial target: approximately 5 m × 5 m indoor arena containing a simulated fire, three lightweight metal victim models, walls/obstacles, and a robot starting/safe zone.

Gazebo provides the initial deterministic scenario. A real visual fire representation may replace the simulated fire during physical testing.

## 7. Perception
The perception interface must support:
- RGB camera.
- RGB-D/depth camera such as Intel RealSense.

Candidate tools include OpenCV and a suitable YOLO-family object detector. The exact detector and sensor are intentionally not hard-fixed.

Expected detections: fire, victims, obstacles/relevant objects.

## 8. Navigation
Use a ROS 2-compatible autonomous navigation stack, with Nav2 as the planned baseline. The robot should map/localize, plan collision-free paths, navigate to victims, return to the safe zone, and support reasonable recovery behavior.

**SLAM is a core project capability.**

## 9. Cognitive Decision
For each unrescued victim, evaluate factors such as:
- Fire risk.
- Distance to fire.
- Robot-to-victim distance.
- Accessibility.
- Estimated rescue/travel cost.
- Perception confidence.

The MVP may use a transparent weighted utility model. The interface must allow a more advanced model later. The rescue order must never be hardcoded.

## 10. Physical Rescue
Planned hardware:
- Differential-drive chassis.
- DC geared motors.
- Motor driver.
- ESP32/Arduino-class controller.
- Wheel encoders.
- Electromagnet.
- MOSFET/relay switching.
- Battery and regulator.

The robot approaches a lightweight metal victim, activates the electromagnet, transports the victim to the safe zone, deactivates it, and updates the victim as rescued.

## 11. Evaluation
Metrics:
- Victim detection accuracy.
- Spatial estimation accuracy.
- SLAM/localization performance.
- Navigation success rate.
- Victim-prioritization accuracy.
- Successful rescue rate.
- Collision-free operation.
- Rescue completion time.
- Correct rescue state transitions.

Compare the cognitive prioritization strategy with a nearest-victim baseline.

## 12. Constraints
- Low-cost hardware.
- Laptop is the main computer.
- ESP32 handles low-level physical control.
- No dependency on expensive LiDAR.
- Camera hardware remains interchangeable.
- Simulation must work before hardware arrives.
- Do not assume a fixed victim arrangement.
- Avoid unnecessary complexity until the MVP works.

## 13. Out of Scope
Real human rescue, fire suppression, human-scale loads, safety certification, production emergency response, and dependence on one specific camera.

## 14. Definition of Done
The simulated robot can perceive the scenario, use SLAM/localization, construct its world model, choose a rescue target from current observations, autonomously navigate to it, perform simulated pickup, return to the safe zone, release the victim, update state, and repeat for the remaining victims.
