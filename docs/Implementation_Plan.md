# FireResQ Implementation Plan

> Planning document. Nothing in this file is implemented yet. Source of truth for scope remains [../PRD.md](../PRD.md), [../Architecture.md](../Architecture.md), and [../Hardware.md](../Hardware.md); this document says *how* and *in what order*, not *what*.

## 1. Implementation Goal

Take the repository from design-docs-only to a simulated robot that closes the full cognitive loop autonomously, then swap simulated drivers for physical ones without touching cognition.

The deliverable that defines success is [../PRD.md](../PRD.md) §14: the simulated robot perceives the arena, runs SLAM/localization, builds a world model, **chooses** a rescue target from current observations, navigates there, performs simulated pickup, returns to the safe zone, releases, updates state, and repeats for the remaining victims.

Two properties matter more than feature count:

- **Explainability.** Every target selection must be inspectable — which victim, what score, which factors produced it. A demo where the robot rescues victims but cannot explain why it picked that order has failed the point of the project.
- **Substitutability.** Camera backend, SLAM implementation, decision model, and motor/magnet driver are each behind an interface. None of them is a rewrite when it changes.

Non-goal: anything in §17. Those stay TODOs in code.

## 2. Current Repository State

Design docs only — no packages, no build system, no code. `src/`, `simulation/`, and `tests/` do not exist yet.

The toolchain, however, is already installed and usable in this WSL2 Ubuntu 24.04 checkout. Measured versions and sensor performance are recorded in [Environment.md](Environment.md). Verified present:

| Dependency | State | Role |
| --- | --- | --- |
| ROS 2 Jazzy (`/opt/ros/jazzy`) | installed | Core |
| Gazebo Harmonic (`gz sim` 8.15.0) | installed | Simulator |
| `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image` | installed | Gazebo↔ROS bridging |
| `gz-sim8` `diff-drive`, `detachable-joint`, `sensors`, `joint-state-publisher`, `pose-publisher` system plugins | installed | Drive, magnet, sensors |
| `slam_toolbox` | installed | 2D SLAM candidate |
| `depthimage_to_laserscan` | installed | Depth→scan for the above |
| `nav2_bringup`, `nav2_simple_commander` | installed | Navigation |
| `robot_state_publisher`, `joint_state_publisher`, `xacro` | installed | Robot description / TF |
| `cv_bridge`, `vision_msgs`, `image_transport`, OpenCV 4.6.0 | installed | Perception |
| `robot_localization` | installed | Optional odometry fusion (not MVP) |
| `colcon`, Python 3.12.3, `pytest` 7.4.4 | installed | Build and test |

Verified **missing**, and deliberately not required by the plan below: `ros2_control` / `gz_ros2_control` (the Gazebo `diff-drive` system plugin covers the MVP without it), `rtabmap_slam` (visual-SLAM fallback, apt-installable if needed), `ultralytics` (see §7 — the MVP detector does not need it), `micro_ros_agent` (Phase 12 only), `twist_mux`, `camera_info_manager`.

**The practical consequence:** Phases 1–11 need no new system dependencies. That is a deliberate constraint on the plan, not a coincidence.

## 3. System Architecture Summary

The loop from [../PRD.md](../PRD.md) §2, annotated with which package owns each stage:

```text
PERCEPTION                fire_resq_perception   camera → detections
    ↓
SLAM / LOCALIZATION       (3rd party + control)  map, map→odom→base_link TF
    ↓
WORLD MODEL               fire_resq_world_model  entities, confidence, rescue state
    ↓
REASONING                 fire_resq_cognition    risk, accessibility, cost per victim
    ↓
DECISION                  fire_resq_cognition    utility → one RescueTarget
    ↓
PLANNING                  fire_resq_planning     rescue FSM + Nav2 goals
    ↓
ACTION                    fire_resq_control      cmd_vel, magnet attach/release
    ↓
UPDATED SITUATION AWARENESS → back to WORLD MODEL
```

Data flows one way through the stages; only the reassessment edge closes the loop, and it closes it *into the world model*, never directly into planning. That is what stops the system from degenerating into a scripted sequence: the planner never holds a list of victims, it asks cognition for the next target and cognition answers from whatever the world model currently believes.

Three architectural boundaries carry the project's hard rules:

1. **`DetectionArray` between perception and everything above it** — camera-agnostic, so RGB and RGB-D are interchangeable ([../Architecture.md](../Architecture.md) §4).
2. **`/scan` + `map`→`odom` TF between localization and navigation** — so the SLAM implementation is swappable (§8).
3. **`cmd_vel` / `odom` / magnet command+state between planning and hardware** — so sim drivers and ESP32 drivers are interchangeable ([../Hardware.md](../Hardware.md) §8).

## 4. ROS 2 Package Plan

| Package | Build type | Responsibility | Depends on |
| --- | --- | --- | --- |
| `fire_resq_interfaces` | `ament_cmake` + `rosidl` | All custom msg/srv/action definitions. No logic. | `std_msgs`, `geometry_msgs`, `builtin_interfaces`, `vision_msgs` |
| `fire_resq_description` | `ament_cmake` | URDF/xacro for the 2WD robot, sensor frames, magnet link. Shared by sim and hardware. | `xacro`, `robot_state_publisher` |
| `fire_resq_perception` | `ament_python` | Detector backends, spatial estimation, publishes detections. | interfaces, `cv_bridge`, `image_transport`, `tf2_ros` |
| `fire_resq_world_model` | `ament_python` | Entity registry, association, confidence, rescue status. Sole owner of world state. | interfaces, `tf2_ros` |
| `fire_resq_cognition` | `ament_python` | Prioritizer interface + weighted-utility and nearest-victim implementations. | interfaces, `nav2_msgs` |
| `fire_resq_planning` | `ament_python` | Rescue FSM, Nav2 client, approach-pose computation. | interfaces, `nav2_msgs`, `nav2_simple_commander` |
| `fire_resq_control` | `ament_python` | Magnet abstraction, odometry plumbing, velocity limits. Sim and hardware share this layer's *interface*. | interfaces |
| `fire_resq_hardware` | `ament_python` | ESP32 bridge, physical magnet driver. Stub until Phase 12. | `fire_resq_control` |
| `fire_resq_simulation` (at `simulation/`) | `ament_cmake` | Worlds, models, sim config, sim launch files. | `ros_gz_sim`, `ros_gz_bridge`, description |

Two notes on layout, both flagged rather than assumed:

- **`simulation/` as a package.** [../Architecture.md](../Architecture.md) §9 places `simulation/` at the repo root, outside `src/`. That works as-is: `colcon` scans the whole workspace recursively for `package.xml`, so `simulation/package.xml` builds and becomes launchable as `ros2 launch fire_resq_simulation …`. No doc change needed.
- **`fire_resq_description` is an addition.** It is not in [../Architecture.md](../Architecture.md) §9's list. It earns its place because the URDF is needed by *both* sim and hardware — folding it into `simulation/` would make the physical robot depend on the simulation package, which inverts the §8 sim-to-hardware rule. **This needs your approval as a one-line amendment to Architecture.md §9.** I have not edited that doc. See §16.

## 5. Node and Interface Plan

### Nodes

| Node | Package | Responsibility |
| --- | --- | --- |
| `detector_node` | perception | Runs the active `Detector` backend on the image stream, emits 2D detections with class + confidence |
| `spatial_estimator_node` | perception | 2D detection → 3D point in `map` via the active `SpatialEstimator` backend (depth lookup or ground-plane projection) |
| `world_model_node` | world_model | Consumes detections, associates to tracked entities, maintains confidence and rescue status, publishes `WorldState`, serves status updates |
| `prioritizer_node` | cognition | On request, scores all unrescued victims via the configured model and returns one `RescueTarget` with its score breakdown |
| `rescue_manager_node` | planning | The FSM. Owns the `ExecuteRescue` action, requests targets, drives Nav2, commands the magnet |
| `magnet_node` | control | Translates `ATTACH`/`RELEASE` into the active backend (Gazebo topic in sim, ESP32 command later), publishes true magnet state |
| `esp32_bridge_node` | hardware | Phase 12 only. `cmd_vel`→serial, encoder ticks→`odom`, magnet GPIO |

Third-party nodes: `gz sim` server, `ros_gz_bridge`, `robot_state_publisher`, `depthimage_to_laserscan`, `slam_toolbox`, Nav2 stack.

### Custom interfaces (`fire_resq_interfaces`)

```text
msg/Detection.msg          header, class_id, confidence, bbox (vision_msgs/BoundingBox2D),
                           position (geometry_msgs/Point, map frame), position_valid,
                           source_backend  # provenance for debug ONLY — cognition must not branch on it
msg/DetectionArray.msg     header, Detection[] detections
msg/VictimState.msg        id, position, confidence, status, risk, accessibility,
                           rescue_cost, last_seen
                           status ∈ {UNKNOWN, DETECTED, TARGETED, CARRIED, RESCUED, UNREACHABLE}
msg/WorldState.msg         header, VictimState[] victims, fire_position, fire_confidence,
                           safe_zone_pose, robot_pose, current_target_id
msg/ScoreBreakdown.msg     fire_risk, distance_to_fire, robot_distance, accessibility,
                           rescue_cost, perception_confidence, weights[], total
msg/RescueTarget.msg       victim_id, approach_pose, utility, breakdown, rationale
msg/MagnetState.msg        header, energized, attached, victim_id

srv/SelectTarget.srv       ---> RescueTarget target, bool found, string reason
srv/UpdateVictimStatus.srv string victim_id, uint8 new_status ---> bool success
srv/SetMagnet.srv          bool attach ---> bool success, MagnetState state

action/ExecuteRescue.action  string victim_id  # "" = let cognition choose
                             ---
                             bool success, uint8 final_status, float32 duration_s
                             ---
                             string phase, string detail
```

`rationale` and `ScoreBreakdown` exist purely so the robot can explain itself — they are MVP scope, not polish.

### Topics

| Topic | Type | Producer → Consumer |
| --- | --- | --- |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | bridge → perception |
| `/camera/depth/image_raw`, `/camera/depth/camera_info` | same | bridge → perception, depth→scan *(when RGB-D active)* |
| `/scan` | `sensor_msgs/LaserScan` | `depthimage_to_laserscan` → SLAM, Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / planning → sim or ESP32 |
| `/odom` | `nav_msgs/Odometry` | sim or ESP32 → SLAM, Nav2 |
| `/joint_states` | `sensor_msgs/JointState` | sim → `robot_state_publisher` |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM → Nav2, cognition (accessibility) |
| `/fire_resq/detections` | `DetectionArray` | perception → world model |
| `/fire_resq/world_state` | `WorldState` | world model → cognition, planning, RViz |
| `/fire_resq/rescue_target` | `RescueTarget` | cognition → planning, RViz *(latched, for introspection)* |
| `/fire_resq/magnet/state` | `MagnetState` | control → planning |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | SLAM, `robot_state_publisher`, odometry |

Actions consumed: `/navigate_to_pose`, `/compute_path_to_pose` (Nav2).

### TF frames

```text
map ──(SLAM)──> odom ──(wheel odometry)──> base_link ──(static, from URDF)──> left_wheel_link
                                                                          ├─> right_wheel_link
                                                                          ├─> caster_link
                                                                          ├─> magnet_link
                                                                          └─> camera_link ──> camera_optical_frame
```

One publisher per edge, no exceptions. `camera_optical_frame` follows the REP-103 optical convention (z forward) and is where depth maths happens; `camera_link` is the mechanical mount. Getting this pair wrong is the single most common cause of detections landing in the wrong place, so Phase 3 verifies it explicitly.

## 6. Simulation Plan

One world, `rescue_arena.sdf`: 5 m × 5 m floor, four bounding walls, 2–3 interior obstacles, a safe-zone patch, one fire model, three victim models.

| Element | Implementation |
| --- | --- |
| 2WD differential drive | `gz-sim8-diff-drive-system` on two wheel joints; publishes odometry and consumes `cmd_vel` |
| Passive caster | Sphere link with low friction on a free joint — no actuation, matching [../Hardware.md](../Hardware.md) §2 |
| Wheel encoders | `joint-state-publisher-system` → `/joint_states`; `diff-drive` plugin's odometry is the `/odom` source |
| RGB camera | `<sensor type="camera">` via `sensors-system`, bridged with `ros_gz_image` |
| RGB-D camera (optional) | `<sensor type="rgbd_camera">`, enabled by a `use_depth` launch argument — **off does not break the system**, it switches the spatial-estimation backend |
| Fire source | Static model, visually distinct emissive material; a light marker, not a physics fire |
| 3 victims | Small boxes with a distinct material and an attachment link; **poses set in the world file, never read by cognitive code** |
| Obstacles / walls | Static geometry |
| Safe zone | Visual floor patch; its pose is a *configured parameter* of the world model, not a constant in cognition |
| Electromagnet | `detachable-joint-system`, verified to support both `attach_topic` and `detach_topic` in 8.15.0, driven by `magnet_node` |

Victim poses living in the world file is not a violation of the no-hardcoding rule — that is ground truth the simulator owns. The rule binds the *robot's* code: cognition may only learn victim positions through perception → world model. Phase 11 exploits this by randomising world victim poses and confirming rescue order changes accordingly.

Ground-truth `pose-publisher-system` output is bridged but used **only** as an evaluation reference and as a temporary localization stand-in during Phases 2–3. Phase 4 replaces it with SLAM; a guard test in Phase 10 asserts no cognitive node subscribes to it.

## 7. Perception Plan

```text
image (+ depth)  →  Detector  →  2D detections  →  SpatialEstimator  →  3D points (map)  →  world model
```

Both stages are interfaces with swappable implementations, which is how camera independence is actually enforced rather than merely intended:

| Interface | MVP implementation | Later implementations |
| --- | --- | --- |
| `Detector` | `ColorBlobDetector` — OpenCV HSV threshold + contour/area filter | `YoloDetector` (Phase 5b / hardware) |
| `SpatialEstimator` | `GroundPlaneEstimator` (RGB) and `DepthEstimator` (RGB-D), selected by param | Marker-assisted; stereo |

**Why colour-blob detection is the MVP detector, not YOLO.** No pretrained YOLO model detects "small metal rescue block." A YOLO path needs a labelled dataset generated from the sim before it can detect anything at all, which front-loads days of work onto the phase that everything downstream is waiting on. The simulated victims and fire are deliberately distinct in colour, so HSV thresholding is reliable, runs on CPU, and ships in hours. [../PRD.md](../PRD.md) §7 lists OpenCV and a YOLO-family detector as *candidates*, so this is doc-compliant. `ultralytics` is not installed and is not needed until the detector interface is proven.

This also protects the real risk: the hard part of perception here is **spatial estimation**, not classification. Fixing the detector cheaply lets Phase 5 focus on getting positions right.

`GroundPlaneEstimator` back-projects the bounding-box base through the camera intrinsics onto the `z=0` plane using the `camera_optical_frame` TF — valid because victims sit on the floor. `DepthEstimator` reads the median depth over the bounding box interior, rejecting zeros. Both emit the same `Detection.position` in `map`, so nothing above perception can tell which ran.

Confidence is the detector's own score, propagated unchanged. Fusing it with geometric uncertainty is a §17 TODO.

## 8. SLAM / Localization Plan

### Options considered

| Option | Verdict |
| --- | --- |
| **`depthimage_to_laserscan` + `slam_toolbox`** | **Selected for the simulation MVP.** Both installed. `slam_toolbox` is the ROS 2 reference 2D SLAM and feeds Nav2's costmaps natively. Produces the `map`→`odom` TF and an `OccupancyGrid` that cognition reuses for accessibility. Honours the no-LiDAR rule: the scan is synthesised from the depth camera. |
| RTAB-Map visual SLAM | Best RGB-only / RGB-D-with-loop-closure option and the leading hardware candidate. Rejected for the MVP: needs an apt install, is heavier to tune, and its richer output buys nothing the 5×5 m arena needs yet. |
| AMCL on a pre-built map | Useful Phase 11 comparison baseline, but pre-supplying a map contradicts SLAM being a core capability ([../PRD.md](../PRD.md) §8). Not the primary path. |
| Wheel odometry only | Adequate for Phases 2–3 bring-up. Drifts without correction; not a localization answer. |
| Marker-assisted (ArUco) | Real fallback if visual odometry disappoints on hardware. Optional per [../Hardware.md](../Hardware.md) §10, so not MVP. |

### The flexibility mechanism

`/scan` + the `map`→`odom` TF is the interface. Everything above it — Nav2, cognition, planning — consumes only those two things and cannot tell what produced them. Swapping SLAM means changing one launch file and one config file:

```text
[depth camera] → depthimage_to_laserscan → /scan ─┐
[RGB camera]   → RTAB-Map ───────────────────────┼→ map→odom TF → Nav2 / cognition
[real LiDAR, if ever]  → /scan ──────────────────┘
```

Phase 4 delivers this as an explicitly parameterised choice (`localization_backend:=slam_toolbox|rtabmap`) with only the first implemented, so the second is a config addition rather than a refactor.

### The honest caveat

A depth camera has a ~60–90° horizontal FOV against a LiDAR's 360°. The synthesised scan sees a narrow wedge, which means weak loop closure and a map that only fills in if the robot deliberately turns to look around. Mitigations, in order of preference: an initial in-place rotation during Phase 4 bring-up to seed the map; `slam_toolbox` tuned for a small arena (fine resolution, short range); and accepting that mapping needs an exploration pass before the rescue loop starts. **This is the strongest argument for RGB-D over RGB-only on the physical robot, and it is a hardware-dependent decision that stays open — see §16.**

## 9. World Model Plan

`world_model_node` is the single owner of believed state. Nothing else writes it.

Responsibilities:

- **Association.** Match each incoming detection to a tracked entity by nearest-neighbour within a gating radius (param, ~0.3 m); otherwise create a new entity. Three victims in 5×5 m makes this sufficient; a full tracker is unnecessary complexity.
- **Position estimate.** Exponential moving average over accepted detections. Not a Kalman filter — that is a §17 TODO, and the EMA is honest about being a placeholder.
- **Confidence.** Latest detector confidence, decayed with time-since-last-seen so stale entities lose standing in the utility score naturally.
- **Rescue status.** Owns the `VictimState.status` lifecycle and serves `UpdateVictimStatus`. `RESCUED` is terminal and permanently excludes a victim from candidacy — this is what makes the loop terminate.
- **Static-ish entities.** Fire position and confidence tracked the same way. Safe-zone pose comes from a parameter (it is known infrastructure, not something to discover).

Publishes `WorldState` at a modest fixed rate (~5 Hz) plus RViz markers for every entity, colour-coded by status. The marker output is how the system gets demonstrated, so it is Phase 6 scope, not an afterthought.

Deliberate omissions, as TODOs: no occupancy/obstacle memory beyond the SLAM grid, no negative evidence ("I looked and saw nothing"), no probabilistic occupancy.

## 10. Cognitive Decision Plan

### The interface

```python
class VictimPrioritizer(ABC):
    @abstractmethod
    def score(self, world: WorldState, ctx: PlanningContext) -> list[ScoredVictim]: ...
    def select(self, world, ctx) -> RescueTarget | None:   # default: argmax of score()
```

`PlanningContext` carries the robot pose, the occupancy grid, and a path-query callable — so implementations get Nav2 access without importing Nav2.

Selected by a single ROS param, `decision_model`:

| Value | Implementation | Purpose |
| --- | --- | --- |
| `weighted_utility` | `WeightedUtilityPrioritizer` | MVP decision model |
| `nearest` | `NearestVictimPrioritizer` | The [../PRD.md](../PRD.md) §11 comparison baseline |

The baseline is not extra work — it is the second implementation that proves the interface is real, and it is required by the evaluation anyway. One mechanism, two requirements.

### The MVP utility

For each victim with status ∉ {`RESCUED`, `CARRIED`}:

```text
U = w_risk        · fire_risk
  + w_fire_prox   · (1 − norm(d_victim→fire))     # closer to fire ⇒ more urgent
  + w_reach       · (1 − norm(d_robot→victim))    # closer to robot ⇒ cheaper
  + w_access      · accessibility
  + w_conf        · perception_confidence
  − w_cost        · norm(rescue_cost)
```

- Weights are ROS params with documented defaults. No weight is a literal in the scoring code.
- Normalisation is by arena diagonal (a configured extent, not a coordinate) so scores stay comparable across worlds.
- `accessibility` ∈ {0,1} from whether Nav2's `ComputePathToPose` returns a path; `rescue_cost` is that path's length plus the victim→safe-zone leg. **One planner query per candidate yields both**, which is why Nav2 must precede cognition in the phase order (§13).
- Every score emits a `ScoreBreakdown` and a human-readable `rationale` string. The node logs the full comparison table at each decision.

Sign discipline matters here: proximity to fire *raises* priority while distance from the robot *lowers* it, and those two pull in opposite directions when the fire is far away. Phase 8's unit tests pin exactly that case.

### What stays out

Bayesian updates, uncertainty propagation, information-gain/active perception, dynamic fire-risk evolution, battery-aware reasoning, learned policies. Each gets a TODO at the code location where it would attach — `fire_risk` is a static function of distance-to-fire in the MVP, with the TODO for dynamic modelling sitting on that function.

## 11. Navigation and Rescue Plan

Nav2 runs with the standard bringup: `slam_toolbox` supplying `map`, `/scan` feeding both costmaps, a differential-drive-compatible controller, and tuned footprint/inflation for a small robot in a 5 m arena.

`rescue_manager_node` implements the FSM behind the `ExecuteRescue` action:

```text
        ┌──────────────────────────────────────────────────┐
        ↓                                                  │
IDLE → SELECT_VICTIM → NAVIGATE → ALIGN → PICK_UP → RETURN → RELEASE → MARK_RESCUED → REASSESS
            │              │        │        │                                            │
            │ none found    │ fail   │ fail   │ fail                                       │
            ↓              ↓        ↓        ↓                                            ↓
          DONE ←───────── FAILED (TODO: recovery policies) ──────────────┘         more victims? ─→ SELECT_VICTIM
```

| State | Action |
| --- | --- |
| `SELECT_VICTIM` | Call `SelectTarget`. No target ⇒ `DONE`. Mark `TARGETED`. |
| `NAVIGATE` | Send the computed approach pose to `/navigate_to_pose`. |
| `ALIGN` | Creep forward along the robot→victim bearing to magnet standoff. Open-loop in the MVP; TODO for closed-loop visual servoing. |
| `PICK_UP` | `SetMagnet(attach)`, confirm via `MagnetState.attached`, set `CARRIED`. |
| `RETURN` | Navigate to the safe-zone pose from the world model. |
| `RELEASE` | `SetMagnet(release)`, confirm detach. |
| `MARK_RESCUED` | `UpdateVictimStatus(RESCUED)` — terminal. |
| `REASSESS` | Re-enter `SELECT_VICTIM`. **Cognition re-runs against the updated world model.** |

The approach pose is *computed*, never stored: victim position from the world model, offset back along the robot→victim vector by the magnet standoff distance, yaw facing the victim. No waypoint lists, no path constants, no ordering — the FSM holds exactly one victim id at a time and re-derives everything else.

That `REASSESS` → `SELECT_VICTIM` edge is the whole cognitive claim of the project. Re-deciding there (rather than computing an order once at startup) is what makes the robot responsive to a world that changed while it was busy.

## 12. Hardware Integration Plan

The boundary is already specified in [../Hardware.md](../Hardware.md) §8. This plan's job is to not violate it before Phase 12 arrives.

```text
LAPTOP   ROS 2 · perception · SLAM/localization · Nav2 · world model · cognition · planning
   ↓      stable interfaces: cmd_vel · odom · TF · image · depth (when present) · magnet cmd/state
ESP32    motor PWM+direction · encoder acquisition · electromagnet switching
```

What changes at Phase 12, and nothing else:

| Interface | Simulation source | Hardware source |
| --- | --- | --- |
| `cmd_vel` consumer | `gz` `diff-drive` plugin | `esp32_bridge_node` → serial → motor driver |
| `/odom` + `odom`→`base_link` | `diff-drive` plugin odometry | Encoder ticks → differential kinematics |
| Camera streams | `ros_gz_image` bridge | `v4l2_camera` or RealSense driver |
| Magnet backend | `detachable-joint` topics | GPIO → MOSFET |

`magnet_node` is written from Phase 9 with a backend interface (`SimMagnetBackend`, `Esp32MagnetBackend`) precisely so Phase 12 adds a class rather than editing the node. Hardware-specific code is confined to `fire_resq_hardware`; a Phase 10 guard test asserts no cognition/planning module imports it.

A watchdog/failsafe is mandatory before any autonomous physical run ([../docs/Hardware_Integration.md](Hardware_Integration.md)) and is Phase 12 scope, not optional polish.

## 13. Phased Implementation Roadmap

**One change from your proposed ordering: Nav2 moves before cognitive prioritization** (your Phase 8 → Phase 7, and vice versa). The utility model's `accessibility` and `rescue_cost` factors are defined in terms of planner path queries, so building cognition first would mean either stubbing those two factors and reworking them later, or building the scorer against an interface nothing implements. Nav2 first makes Phase 8 testable the day it is written. Everything else keeps your order.

---

### Phase 0 — Repository / dependency validation ✅ COMPLETE
- **Result:** See [Environment.md](Environment.md). Accelerated rendering available via `GALLIUM_DRIVER=d3d12` + `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`; RTF 1.00 on both software and GPU paths; depth→scan pipeline validated early at 17 Hz / 59.9° FOV.
- **Goal:** Confirm the toolchain end-to-end before writing project code.
- **Creates:** `.gitignore` (`build/ install/ log/`), a short `docs/Environment.md` recording verified versions.
- **Depends on:** nothing.
- **Interfaces:** none.
- **Verify:** `gz sim` launches an empty world with a GUI from WSL2; `ros2 topic list` works; `colcon build` succeeds on an empty `src/`; the `ros_gz_bridge` round-trips one topic.
- **Done when:** a GUI Gazebo window renders and a bridged topic echoes.
- **TODO:** confirm WSL2 GPU/software rendering is adequate for camera sensors — if `rgbd_camera` is unusably slow, this is the phase that must discover it.

### Phase 1 — ROS 2 package skeleton
- **Goal:** All nine packages build, empty but wired.
- **Creates:** the package tree from §4; every msg/srv/action from §5 in `fire_resq_interfaces`.
- **Depends on:** Phase 0.
- **Interfaces:** all custom types compile and are introspectable.
- **Verify:** `colcon build --symlink-install`; `ros2 interface show fire_resq_interfaces/msg/WorldState` for each type.
- **Done when:** clean build from scratch and every custom type resolves.
- **TODO:** revisit `Detection` for a covariance field when uncertainty propagation lands.

### Phase 2 — Gazebo robot and arena
- **Goal:** The robot spawns and drives.
- **Creates:** `fire_resq_description` xacro (base, two wheels, caster, camera mount, magnet link); `simulation/worlds/rescue_arena.sdf`; victim/fire/safe-zone models; `simulation/launch/sim.launch.py`.
- **Depends on:** Phase 1.
- **Interfaces:** `/cmd_vel` in, `/odom` out, `/joint_states`, `/tf`.
- **Verify:** `teleop_twist_keyboard` drives it; it rotates in place; the caster does not snag; RViz shows the model matching the sim.
- **Done when:** [../docs/Simulation.md](Simulation.md) stage 1 passes — the robot moves via `/cmd_vel`.
- **TODO:** inertia/friction values are placeholders until real chassis mass is known.

### Phase 3 — Sensors + TF + odometry
- **Goal:** Trustworthy sensing and frames.
- **Creates:** camera + optional `rgbd_camera` sensors in the URDF; bridge config; `use_depth` launch arg.
- **Depends on:** Phase 2.
- **Interfaces:** `/camera/image_raw`, `/camera/camera_info`, depth pair, complete `odom`→`base_link`→`camera_optical_frame` chain.
- **Verify:** `ros2 run tf2_tools view_frames` shows one publisher per edge and no gaps; images render in RViz; a known-position object back-projects to the right place; odometry after a 2 m drive and a 360° turn is within tolerance.
- **Done when:** [../docs/Simulation.md](Simulation.md) stage 2 passes and the optical-frame convention is confirmed correct.
- **TODO:** sensor noise is absent; add it before trusting any accuracy number.

### Phase 4 — SLAM / localization
- **Goal:** A map and a correct `map`→`odom` transform, LiDAR-free.
- **Creates:** `depthimage_to_laserscan` + `slam_toolbox` config; `localization.launch.py` with `localization_backend`.
- **Depends on:** Phase 3 (needs depth + TF).
- **Interfaces:** `/scan`, `/map`, `map`→`odom`.
- **Verify:** drive the arena and confirm walls appear in roughly correct geometry; revisit a spot and check drift; compare against bridged ground-truth pose.
- **Done when:** stage 3 passes — a usable occupancy grid exists and localization holds over a full arena circuit.
- **TODO:** narrow-FOV loop closure is the known weakness (§8); RTAB-Map remains the unimplemented alternative behind the same interface.

### Phase 5 — Perception
- **Goal:** Victims and fire detected and placed in `map`.
- **Creates:** `Detector` ABC + `ColorBlobDetector`; `SpatialEstimator` ABC + `GroundPlaneEstimator` and `DepthEstimator`; the two nodes.
- **Depends on:** Phase 3 (Phase 4 not strictly required — positions can be checked in `odom` first).
- **Interfaces:** `/fire_resq/detections`.
- **Verify:** unit tests on synthetic images; compare estimated victim positions against sim ground truth (target: within a few centimetres with depth); **run both estimators on the same scene and confirm consumers need no changes.**
- **Done when:** stage 4 passes and the RGB/RGB-D swap is demonstrated, not just designed.
- **TODO:** `YoloDetector` unimplemented; confidence is not yet geometric.

### Phase 6 — World model
- **Goal:** Stable beliefs from noisy detections.
- **Creates:** entity registry, association, EMA smoothing, confidence decay, status lifecycle, `world_model_node`, RViz markers.
- **Depends on:** Phases 4 + 5.
- **Interfaces:** `/fire_resq/world_state`, `UpdateVictimStatus`.
- **Verify:** three victims converge to three stable entities with no duplicates or identity swaps across a full arena drive; confidence decays when a victim leaves view; `RESCUED` persists.
- **Done when:** stage 5 passes and the RViz view is demo-legible.
- **TODO:** Bayesian updates, negative evidence, covariance tracking.

### Phase 7 — Nav2 planning *(moved ahead of cognition)*
- **Goal:** Reliable point-to-point autonomy plus the path-query capability cognition needs.
- **Creates:** Nav2 params, `navigation.launch.py`, a path-query helper wrapping `ComputePathToPose`.
- **Depends on:** Phase 4.
- **Interfaces:** `/navigate_to_pose`, `/compute_path_to_pose`.
- **Verify:** RViz goals reached without collision from several start poses; the helper returns sane lengths and correctly reports unreachable poses.
- **Done when:** the robot navigates to arbitrary reachable goals, and path length/reachability are queryable programmatically.
- **TODO:** recovery behaviours left at Nav2 defaults; controller tuning is sim-only.

### Phase 8 — Cognitive prioritization
- **Goal:** Dynamic, explainable target selection.
- **Creates:** `VictimPrioritizer` ABC, `WeightedUtilityPrioritizer`, `NearestVictimPrioritizer`, `prioritizer_node`, weight config.
- **Depends on:** Phases 6 + 7.
- **Interfaces:** `SelectTarget`, `/fire_resq/rescue_target`.
- **Verify:** unit tests on hand-built `WorldState` fixtures — nearest-but-safe vs distant-but-endangered, a victim behind an obstacle scoring lower on accessibility, low confidence deprioritised, `RESCUED` never selected; the two models provably disagree on a constructed scene.
- **Done when:** stage 6 passes — selection is dynamic, and each decision emits a breakdown explaining itself.
- **TODO:** static `fire_risk`; no uncertainty, no information gain, no battery term.

### Phase 9 — Rescue mechanism simulation
- **Goal:** Commandable pickup and release.
- **Creates:** `detachable-joint` wiring in the victim models; `MagnetBackend` ABC + `SimMagnetBackend`; `magnet_node`.
- **Depends on:** Phase 2.
- **Interfaces:** `SetMagnet`, `/fire_resq/magnet/state`.
- **Verify:** manual attach/release by service call; the victim tracks the robot while carried and stays put after release; state reporting matches reality including a failed attach at excessive distance.
- **Done when:** stage 8's mechanism works in isolation, ahead of FSM integration.
- **TODO:** `Esp32MagnetBackend` unimplemented; no holding-force model.

### Phase 10 — Complete autonomous rescue loop
- **Goal:** The [../PRD.md](../PRD.md) §14 deliverable.
- **Creates:** `rescue_manager_node` FSM, `ExecuteRescue` server, approach-pose computation, `full_system.launch.py`.
- **Depends on:** Phases 6, 7, 8, 9.
- **Interfaces:** `ExecuteRescue`; consumes everything above.
- **Verify:** all three victims rescued unattended from a cold start; state transitions logged in order; **re-run with victim poses moved and confirm the rescue order changes**; guard tests that no cognitive module imports hardware or subscribes to ground truth.
- **Done when:** stages 9–10 pass and the full loop repeats until no victims remain.
- **TODO:** every failure path currently terminates rather than recovering — the three recovery TODOs attach to the `FAILED` transitions.

### Phase 11 — Evaluation and comparison
- **Goal:** Evidence, per [../PRD.md](../PRD.md) §11.
- **Creates:** a metrics recorder node, a scenario runner over randomised victim/fire layouts, a results summariser.
- **Depends on:** Phase 10.
- **Interfaces:** consumes `WorldState`, `RescueTarget`, `/odom`, ground truth.
- **Verify:** N randomised trials per decision model; report detection accuracy, spatial error, navigation success, rescue success, collisions, completion time, and prioritization agreement with a documented expected ordering.
- **Done when:** `weighted_utility` and `nearest` are compared on identical scenarios with a written interpretation of where and why they differ.
- **TODO:** no statistical significance testing; ground truth only exists in sim.

### Phase 12 — Hardware integration
- **Goal:** Same cognition, physical robot.
- **Creates:** `esp32_bridge_node`, ESP32 firmware, `Esp32MagnetBackend`, real camera driver config, `hardware.launch.py`, watchdog.
- **Depends on:** Phase 10 complete; physical parts per [../Hardware.md](../Hardware.md) §10; the §16 decisions resolved.
- **Interfaces:** identical to simulation — that is the whole test.
- **Verify:** the [../Hardware.md](../Hardware.md) §11 phase ladder, then full-loop comparison against sim behaviour.
- **Done when:** the physical robot completes one full rescue cycle with unmodified cognition/planning packages.
- **TODO:** all of [../Hardware.md](../Hardware.md) §14, with the watchdog blocking any autonomous run.

## 14. Testing Strategy

Three tiers, cheapest first:

1. **Pure unit tests (`pytest`, no ROS, no sim).** The scorer, normalisation, approach-pose geometry, association logic, and depth/ground-plane maths are all pure functions over fixtures. This is where the cognitive claims get pinned down, and it must stay fast enough to run constantly:
   ```bash
   python3 -m pytest src/fire_resq_cognition/test/test_priority.py -k endangered_beats_nearest -v
   ```
2. **Node-level integration (`launch_testing`).** Publish synthetic detections, assert `WorldState`; stub a world model, assert the FSM's transition sequence. No Gazebo, so these can gate commits.
3. **Full-stack simulation runs (headless `gz sim`).** The Phase 10 acceptance run and Phase 11 trials. Slow, non-deterministic, run deliberately rather than on every change.

Per-phase verification is specified in §13 and is the real gate — a phase is not done because code exists but because its check passed. Two invariants get permanent guard tests: no cognitive module imports `fire_resq_hardware`, and no cognitive module subscribes to ground-truth pose.

## 15. Evaluation Strategy

Metrics come from [../PRD.md](../PRD.md) §11; the comparison that matters is `weighted_utility` vs `nearest`.

Scenarios are generated, not authored: randomise victim and fire poses within the arena across trials. For each trial record detection accuracy, spatial error against ground truth, localization drift, navigation success rate, collisions, per-victim and total rescue time, rescue success, and the selected order against a documented expected order.

The interesting result is not "utility wins." It is *characterising when the two diverge* — a layout where the nearest victim is safe and a distant one sits beside the fire is where the cognitive model earns its existence, and a layout where they agree is worth reporting honestly too. Randomised layouts also double as the anti-hardcoding proof: order must change with the world.

## 16. Known Risks and Unresolved Decisions

### Genuinely blocking: nothing

No contradiction between the documents blocks implementation.

- ~~**`fire_resq_description` is not in [../Architecture.md](../Architecture.md) §9.**~~ **RESOLVED (approved).** [../Architecture.md](../Architecture.md) §9 now lists `fire_resq_description`, scoped to the reusable robot description/URDF and model assets, independent of Gazebo-specific simulation logic.
- **Phase 0 is complete.** Results recorded in [Environment.md](Environment.md). Gazebo rendering under WSL2 is acceptable; no blocking limitation found.

### Hardware-dependent decisions, deliberately left open

| Decision | Status | When it must be resolved | What it affects |
| --- | --- | --- | --- |
| **RGB vs RGB-D** | Open. Sim supports both; MVP is developed with depth available and RGB-only exercised via the estimator swap. | Before Phase 12 purchase | Which `SpatialEstimator` is primary; heavily constrains the SLAM choice |
| **SLAM implementation** | `slam_toolbox` + depth→scan for the sim MVP only. Hardware choice open. | Phase 12 | Coupled to the camera decision — RGB-only realistically forces RTAB-Map or markers |
| **Exact camera model** | Open | Phase 12 | FOV, intrinsics, mounting height, depth range |
| **Encoder resolution** | Open | Phase 12 | Odometry accuracy, `odom` covariance, ESP32 interrupt load |
| **Battery** | Open by design ([../Hardware.md](../Hardware.md) §7) | After motor/magnet current measurement | Runtime, regulator sizing, chassis mass |
| **Electromagnet spec** | Open | Phase 12 | Holding force vs victim mass, standoff distance, `ALIGN` tolerance |

None of these blocks Phases 0–11, which is the point of putting them behind interfaces.

### Technical risks

| Risk | Severity | Mitigation |
| --- | --- | --- |
| Narrow-FOV scan gives poor loop closure and patchy maps (§8) | **High** — the most likely phase to overrun | Initial rotation to seed the map; tune for a small arena; accept an exploration pass; RTAB-Map as the fallback behind the same interface |
| WSL2 rendering too slow for camera/depth sensors | Medium — would slow every later phase | Discovered in Phase 0 deliberately; fall back to reduced resolution/rate or headless |
| Spatial estimation error exceeds magnet alignment tolerance | Medium | Depth estimator preferred; measure in Phase 5 before the FSM depends on it; TODO for closed-loop visual align |
| Open-loop `ALIGN` unreliable in sim physics | Medium | Keep standoff generous; confirm attach via `MagnetState` rather than assuming success |
| Colour-blob detection is brittle on real cameras (lighting) | Low in sim, **high on hardware** | Accepted MVP tradeoff behind the `Detector` interface; `YoloDetector` is the hardware answer |
| `detachable-joint` re-attach semantics differ from expectation | Low | `attach_topic` support confirmed present in 8.15.0; Phase 9 tests it standalone before integration |
| Scope creep from §17 into the MVP | Medium | TODOs live at their attachment points; phase definitions-of-done are the gate |

## 17. Future TODOs

Not MVP scope. Each lives as an explicit `TODO` at the code location where it would attach, so the extension point is discoverable without this document.

| Extension | Code location |
| --- | --- |
| Uncertainty propagation | `Detection` covariance field; world model position estimator |
| Bayesian belief updates | World model — replaces EMA smoothing and confidence decay |
| Active perception / information gain | Cognition — a new `VictimPrioritizer` plus an inspection action in the FSM |
| Dynamic fire-risk modelling | Cognition `fire_risk()`, currently static in distance-to-fire |
| Battery / resource-aware reasoning | `PlanningContext` + a utility term; hardware state topic |
| Alternative decision algorithms | New `VictimPrioritizer` implementations — the interface already exists |
| Failed-detection recovery | FSM `SELECT_VICTIM` no-target branch |
| Failed-pickup recovery | FSM `PICK_UP` failure branch |
| Failed-navigation recovery | FSM `NAVIGATE` failure branch; Nav2 recovery config |
| Sensor noise, dynamic obstacles | Simulation world and sensor definitions |
| Hardware watchdog / failsafe | `fire_resq_hardware` — **blocking for autonomous physical testing** |

## Recommended Starting Point

> **Status: Phase 0 complete.** The next task is Phase 1 (ROS 2 package skeleton). The original reasoning is kept below for the record.

**Phase 0, and specifically: verify that Gazebo Harmonic renders camera and RGB-D sensors at usable frame rates under WSL2.**

Concretely — launch `gz sim` with a trivial world containing a camera sensor and an `rgbd_camera` sensor, bridge both image topics into ROS 2, and measure with `ros2 topic hz`. Then commit a `.gitignore` covering `build/ install/ log/` and record the measured versions and frame rates in `docs/Environment.md`.

This is first because it is the only unknown that can invalidate the plan's shape. Every phase from 3 onward depends on simulated camera and depth streams, and the entire SLAM strategy in §8 assumes depth images arrive fast enough to synthesise scans. WSL2 GPU passthrough is the one variable I could not confirm by inspecting installed packages — everything else in §2 I verified directly. If depth rendering turns out to be unusably slow, that is worth discovering in an afternoon against a throwaway world rather than in Phase 4 with six packages already built on the assumption.

It is also nearly free: no project code, no design commitment, and the `.gitignore` is needed regardless.

Once that passes, Phase 1 follows immediately — and Phase 1 wants the `fire_resq_description` question in §16 answered first, since it determines the package tree that gets created.
