# FireResQ — System Architecture

## 1. Principle
FireResQ is modular. Hardware, perception, cognition, planning, and control remain separated so simulation drivers can later be replaced by physical hardware without rewriting the cognitive layer.

## 2. High-Level Flow
```text
Sensors
  ↓
Perception
  ↓
World Model / Belief State
  ↓
Cognitive Decision Engine
  ↓
Task / Motion Planner
  ↓
Navigation + Control
  ↓
Robot / Rescue Actuator
  ↓
New Observations ─────────→ World Model
```

## 3. ROS 2 Layers

### Perception
Converts RGB/depth sensor data into semantic detections. Candidate tools: OpenCV, YOLO-family detector, and RealSense ROS drivers when available. Publish camera-independent detection messages.

### Mapping and Localization
SLAM is core. Planned ecosystem: SLAM Toolbox or another suitable ROS 2 SLAM solution, Nav2, TF2, and wheel odometry.

### World Model
Maintains robot pose, fire, victims, obstacles, safe zone, rescue status, confidence, current target, and relevant resource/state information.

### Cognition
Handles candidate generation, risk estimation, victim prioritization, decision selection, and reassessment. MVP: transparent weighted multi-criteria utility model. Future options: Bayesian reasoning, probabilistic decisions, active perception, and alternative policies.

### Planning
Converts a selected action into:
```text
SELECT VICTIM
→ NAVIGATE
→ ALIGN
→ PICK UP
→ RETURN
→ RELEASE
→ MARK RESCUED
→ REASSESS
```

### Control
Simulation uses Gazebo controllers. Physical control uses ROS 2 ↔ ESP32 ↔ motor driver/encoders.

### Rescue Actuator
Expose an electromagnet abstraction. Physical path:
```text
ROS 2 → ESP32 → MOSFET/Relay → Electromagnet
```

## 4. Camera Abstraction
```text
Perception Interface
       ↑
 ┌─────┴─────────┐
 RGB Backend   RGB-D Backend
```
The cognition layer must not depend on which backend produced spatial estimates.

## 5. Coordinate Frames
Expected frames:
- `map`
- `odom`
- `base_link`
- camera frame(s)

Use TF2. Do not hardcode coordinates in cognitive code.

## 6. Interfaces
Prefer custom ROS 2 interfaces for semantic information such as:
- Detection.
- VictimState.
- WorldState.
- RescueTarget.
- RescueAction.

Exact definitions may evolve.

## 7. Failure Handling
Architecture should allow lost victims, low-confidence detections, navigation failure, failed pickup/release, and sensor unavailability.

TODO: implement recovery policies after MVP.

## 8. Simulation-to-Hardware Rule
Keep ROS-level interfaces stable. Replace underlying sensor/actuator drivers rather than higher-level cognition.

## 9. Package Structure
```text
src/
├── fire_resq_interfaces/
├── fire_resq_description/
├── fire_resq_navigation/
├── fire_resq_perception/
├── fire_resq_world_model/
├── fire_resq_cognition/
├── fire_resq_planning/
├── fire_resq_control/
└── fire_resq_hardware/

simulation/
├── worlds/
├── models/
├── config/
└── launch/

tests/
docs/
```

`fire_resq_description` holds the reusable robot description (URDF/xacro) and robot model assets. It is independent of Gazebo-specific simulation logic so that both simulation and physical hardware bringup can depend on it.

`fire_resq_navigation` holds the depth→scan pipeline, SLAM/localization and Nav2 configuration and launch files. It consumes only stable ROS interfaces (depth image, `odom`, TF, `/scan`) and exposes `/scan`, `/map`, `map→odom` and the Nav2 actions, so it is shared unchanged by simulation and hardware and the mapping/localization backend can be replaced without touching cognition. Layering: description → simulation → navigation → cognition.

## 10. Architecture TODOs
- TODO: uncertainty propagation.
- TODO: Bayesian belief update.
- TODO: active perception.
- TODO: battery/resource model.
- TODO: compare priority algorithms.
- TODO: recovery behaviors.
- TODO: validate RGB backend.
- TODO: validate RealSense backend.
- TODO: tune physical navigation.
