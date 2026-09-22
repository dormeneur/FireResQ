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

One publisher per edge, no exceptions. `camera_optical_frame` follows the REP-103 optical convention (z forward) and is where depth maths happens; `camera_link` is the mechanical mount. Getting this pair wrong is the single most common cause of detections landing in the wrong place, so Phase 2 verified it explicitly (back-projected known objects into `odom`; see Phase 2).

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
| `SpatialEstimator` | `KnownHeightEstimator` (RGB) and `DepthEstimator` (RGB-D), selected by param | Marker-assisted; stereo |

**Why colour-blob detection is the MVP detector, not YOLO.** No pretrained YOLO model detects "small metal rescue block." A YOLO path needs a labelled dataset generated from the sim before it can detect anything at all, which front-loads days of work onto the phase that everything downstream is waiting on. The simulated victims and fire are deliberately distinct in colour, so HSV thresholding is reliable, runs on CPU, and ships in hours. [../PRD.md](../PRD.md) §7 lists OpenCV and a YOLO-family detector as *candidates*, so this is doc-compliant. `ultralytics` is not installed and is not needed until the detector interface is proven.

This also protects the real risk: the hard part of perception here is **spatial estimation**, not classification. Fixing the detector cheaply lets Phase 5 focus on getting positions right.

`KnownHeightEstimator` (RGB-only) intersects the ray through the blob's **top edge** with the horizontal plane at that object class's known top height. It replaced the planned ground-plane estimator, which back-projected the blob's *base*: with the camera 8.75 cm above the floor the base ray meets the floor at a grazing angle, so one pixel of error moves the range by tens of centimetres (measured: 13–44 cm on victims, 57 cm on the fire; see Phase 5). The top edge sits well above the camera, so the same pixel costs ~1–4 cm. The bottom anchor is kept as a parameter (`known_height_anchor: bottom`) purely so that measurement stays reproducible. `DepthEstimator` takes a low percentile of the depth over the blob's own mask (never one pixel) and back-projects the blob's centre at that depth. Both emit the same `Detection.position` in the target frame, so nothing above perception can tell which ran (`source_backend` is provenance only, and a unit guard stops cognition reading it).

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

Phase 4 delivers this as an explicit choice (`localization:=slam|amcl|none`, and `slam_preset:=scan_matching|odom_only`) with exactly one `map→odom` publisher enforced; RTAB-Map remains an unimplemented alternative behind the same `/scan`/`/map`/`map→odom` interface.

### The honest caveat (updated with Phase 4's measurements)

A depth camera has a ~60–90° horizontal FOV against a LiDAR's 360°, and the synthesised scan is a narrow wedge. This section originally proposed an initial in-place rotation to seed the map; **that was wrong**. Measured in Phase 4 (see Step 0 and the Phase 4 section): SLAM Toolbox ignores rotation-only motion by default (this robot pivots exactly about `base_link`), and when forced to match every scan it **diverges** on the wedge, at 60° and at 87°, even on noise-free odometry. What works is skipping rotation-only scans with a 0.3 m translation gate on an 87° camera — genuine scan-matching SLAM, but one that **maps only when the robot translates**. Loop closure made every narrow-FOV configuration worse and is off. Localizing against a saved map (AMCL) avoids the scan-to-scan drift and handles rotation. This is the strongest argument for RGB-D over RGB-only on the physical robot, and the camera choice and the hardware SLAM approach stay open (§16).

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

**As built (Phase 6):** see the Phase 6 section in §13 for what was implemented and the five decisions that differ from or add to the text above (tentative `UNKNOWN` confirmation, the 0.2 confidence floor, the configured safe-zone region and its "never creates a victim" rule, no publication without localisation, one process).

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

**As built (Phase 8): see the Phase 8 section in §13** — the formula below is implemented with a graded accessibility, two planner queries per candidate and a hazard-radius fire risk; six decisions differ from or add to this text.

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

### Phase 1 — ROS 2 package skeleton ✅ COMPLETE
- **Result:** All nine packages build clean (~23 s from scratch) and are discoverable via `ros2 pkg list`. All 11 custom interfaces generate. Robot description launches standalone in RViz with a correct TF tree; `camera_optical_frame` verified against REP-103. Robot spawns in Gazebo. `colcon test` runs clean (0 tests — test files begin in Phase 5).
- **Known Phase 1 gap (expected):** wheel-joint TF is absent under Gazebo because nothing publishes `/joint_states` yet; closed in Phase 2. The standalone `display.launch.py` path has complete TF via `joint_state_publisher`.
- **Correction (found in Phase 2):** this phase originally reported the robot as "resting on its wheels at z=0.0327 m". That was wrong. The magnet cylinder's lowest point (−0.033 m) was below the wheel contact plane (−0.0325 m), so the robot rested on the magnet and caster while the wheels hovered; the 0.0327 rest height was `0.033 − 0.3 mm penetration`, not the wheel radius. Two further description faults surfaced at the same time (base_link not at the axle midpoint; flat-cylinder wheel contact). All three are fixed — see Phase 2. Lesson kept: a settling height that merely *looks* right is not evidence; Phase 2's tests compare against Gazebo ground truth.
- **Goal:** All nine packages build, empty but wired.
- **Creates:** the package tree from §4; every msg/srv/action from §5 in `fire_resq_interfaces`.
- **Depends on:** Phase 0.
- **Interfaces:** all custom types compile and are introspectable.
- **Verify:** `colcon build --symlink-install`; `ros2 interface show fire_resq_interfaces/msg/WorldState` for each type.
- **Done when:** clean build from scratch and every custom type resolves.
- **TODO:** revisit `Detection` for a covariance field when uncertainty propagation lands.

### Phase 2 — Gazebo robot control and sensors ✅ COMPLETE
> **Scope note.** As executed, Phase 2 is *drive + joint states + odometry + RGB/depth sensors*, with the arena deferred to Phase 3. This swaps what the original plan split between Phases 2 and 3; nothing was dropped, and the sensor/TF verification originally listed under Phase 3 was done here.

- **Goal:** The simulated robot is physically controllable and publishes the sensor interfaces that SLAM, navigation and perception will consume.
- **Created:** `simulation/urdf/gz_drive.xacro` (DiffDrive + JointStatePublisher), `gz_sensors.xacro` (RGB camera + optional depth camera), `simulation/config/bridge_base.yaml` / `bridge_depth.yaml` / `sim.rviz`, `simulation/launch/sim.launch.py` (replaces `spawn_robot.launch.py`; adds bridge, sim time, `use_depth`, and a working `gui` switch).
- **Gazebo systems:** `gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`, `gz-sim-sensors-system` (ogre2, in the world file). All stock — no custom workaround nodes.
- **Interfaces:** `/cmd_vel` in; `/odom`, `/joint_states`, `/tf` (`odom→base_link`), `/clock`, `/camera/image_raw`, `/camera/camera_info`, and — when `use_depth:=true` — `/camera/depth/image_raw`, `/camera/depth/camera_info`. `use_depth:=false` removes the depth sensor, its bridge node and its topics entirely.
- **Verified against Gazebo ground truth** (test-only reference read via the gz CLI, never on a ROS topic; test scripts were throwaway and are not in the repo):

| Check | Result |
| --- | --- |
| Rest state | axle z = 0.0325 m = wheel radius, roll/pitch 0.00°, zero drift over 8 s |
| Forward / backward 0.2 m/s × 5 s | truth ±1.0000 m, odom ±1.0000 m |
| Wheel joint angles | 30.769 rad for 1 m = 1/0.0325 exactly; ±4.132 rad for a 90° spin (kinematics from `parameters.xacro`) |
| Spin ±1 rad/s × π/2 s | truth 90.53°, odom 90.53°; base_link displacement 0.0000 m |
| Arc 0.2 m/s + 0.5 rad/s × 4 s | truth (0.3596 m, 114.59°) vs odom (0.3590 m, 114.59°) |
| Speed limits | 1.0 m/s request → peak 0.300; 5 rad/s request → peak 1.500 |
| Zero command | odom velocity 0.0000 |
| TF | one root (`odom`), one parent per frame, 8 frames, no `map`; `odom→base_link` 50 Hz; wheels dynamic; REP-103 optical frame confirmed |
| Sensor contract | RGB 640×480 `rgb8`, depth 640×480 `32FC1`; all four headers `camera_optical_frame`; K matches 60°/640 px analytically (fx 554.38); RGB and depth intrinsics identical |
| Back-projection (pixel + depth + K + TF → `odom`) | red box **0.0 mm** from its surface, green sphere **1.2 mm**; left/right and height correct; an x-mirrored optical frame would miss by 776 mm (negative control) |
| Rates (headless) | RGB ~23–26 Hz, depth ~22–25 Hz, `/odom` 50 Hz, `/joint_states` 250 Hz; RTF 1.00 |
| `use_depth:=false` | no depth topics or bridge node; RGB 21 Hz; driving unaffected |

- **Faults found and fixed** (all in the Phase 1 description; the motion tests against ground truth are what exposed them):
  1. *Robot rested on the magnet, not its wheels.* Wheels hovered, could not drive the body, and wheel-derived odometry reported motion that never happened (odom said 1.39 m; truth said 0 m). Fixed: magnet raised (`magnet_z` −0.015 → 0.0, bottom now 14.5 mm clear); constraint documented in `parameters.xacro`.
  2. *`base_link` was 4 cm behind the axle midpoint.* Differential-drive odometry reports the pose of the axle midpoint as `base_link`, so a spin produced ~5 cm of phantom motion. Fixed: `base_link` is now the axle midpoint (wheels at x = 0; chassis, caster, camera, magnet shifted by the same 4 cm). This is a hardware-relevant convention — the ESP32 odometry must publish the same point.
  3. *Simulated robot over-rotated by 18%.* A flat-cylinder wheel collision turned about its inner edge (0.085 − 0.013 = 0.072 m; 0.085/0.072 = 1.181 vs the measured 1.180). Fixed: the sim wrapper selects a single-contact sphere collision through an xacro arg; the pure description keeps the faithful cylinder by default.
- **Physics step 1 ms → 4 ms.** The stock joint-state plugin has no rate limit and publishes every step; at 1 ms that was a 1 kHz `/joint_states` costing ~40% of a core in the bridge. At 4 ms it is 250 Hz, bridge CPU roughly halved, and every test above was run at 4 ms.
- **Finding for hardware:** Gazebo's DiffDrive holds the last `cmd_vel` indefinitely (measured: 0.200 m/s two seconds after the publisher exited). Sim-harmless, but it is precisely why the ESP32 needs a `cmd_vel` watchdog. Recorded as `TODO(hardware)` in `gz_drive.xacro`.
- **TODOs (remain):** sensor noise models (renders are noise-free, so accuracy figures are optimistic); encoder quantisation (simulated odometry derives from continuous joint motion); real camera / RealSense integration; ESP32 communication; measured inertia, friction and motor limits; camera intrinsics are placeholders for a camera not yet chosen.

### Phase 3 — Rescue arena and world ✅ COMPLETE
- **Goal:** The 5 m × 5 m rescue environment the whole project operates in, launchable repeatably and deterministically, with the robot, sensors, TF and odometry unchanged.
- **Created:**
  - `simulation/fire_resq_simulation/scenario.py` — pure-Python scenario loader, validator and world generator (no ROS/Gazebo import); `simulation/config/scenarios/default.yaml`; `simulation/scripts/generate_world.py` (layout printer / world writer).
  - `simulation/models/victim/` and `simulation/models/fire/` — the two reusable models. Walls, obstacles and the safe-zone slab are generated inline from the scenario.
  - `simulation/launch/arena.launch.py` — generates the world, then includes `sim.launch.py` unchanged (so the robot is not forked or duplicated).
  - `pytest.ini`, `tests/unit/` (39 tests), `tests/sim/` (29 tests + 2 slow).
- **Arena:** 5.0 × 5.0 m inner free space, enclosed by four 0.10 m walls (0.5 m high). Robot starts inside a 1 × 1 m safe zone in the south-west corner facing north-east. One fire, three victims, a 1.4 m partition and two blocks. Coordinates and the design intent of each victim are in CLAUDE.md ("The rescue arena").
- **Fire:** static emissive orange cone (embers + outer + inner flame) with a cylindrical collision column so the robot treats it as an obstacle. Replaceable: the scenario names the model, and only the colour/obstacle contract is relied on. No dynamic fire behaviour (TODO).
- **Victims:** one reusable `victim` model, dynamic (0.22 kg, low centre of mass), blue torso + head, steel attachment ring on the bottom 55 mm at the electromagnet's height, single `base_link` and an `attachment_point` frame for Phase 9. Identity (`victim_1..3`) and pose come from the scenario only; all three look identical, like the physical victims.
- **Safe zone:** 4 mm green floor slab, visual only, so the robot drives over it.
- **No hardcoding:** placement exists only in the scenario YAML; unit-test guards assert robot code never reads it, never consumes Gazebo ground truth, and cognition/planning never import hardware (guards were confirmed to fail on an injected violation). No rescue order or path exists anywhere.
- **Perception preparation (colour contract, measured):** victim hue 109–110 (S 149–159, V 100–205), fire hue 13–21 (S 255, V ≥ 236), safe zone hue 68 (S ~153, V 172–183); every other pixel has S ≤ 18. Classes are disjoint. Back-projected detections land on the real objects (see below).
- **Verification (all automated in `tests/`):**

| Check | Result |
| --- | --- |
| Arena loads | 15 entities present at their scenario poses (±2 mm); **0 `[Err]`** in the Gazebo log (pre-existing Ogre-material `[Wrn]` from the robot wrapper remain) |
| Robot | spawns at the scenario pose inside the safe zone; axle z = wheel radius; all Phase 2 motion/odometry/TF/sensor checks re-run **inside the arena** and pass (10 robot tests, 9 sensor/TF tests) |
| Victims / fire | rest at z ≈ 0, tilt < 1°, no drift after settling; fire static |
| Visibility from the start | a full survey finds fire, `victim_1`, `victim_2` and **not** `victim_3` (hidden behind the partition, as designed); **every detection sits on a real object** — no phantoms |
| Exploration | after driving round the partition (2.55 m + turn + 1.1 m, ±10 cm of plan, no tilt, odometry within 3% of truth) `victim_3` is found |
| Safe zone | visible from inside it; depth-projected green pixels fall inside the zone rectangle and on the floor plane |
| Launch modes | headless, GUI + RViz (RGB 22 Hz, depth 22.6 Hz with everything rendering), `use_depth:=false` (no depth topics or bridge) |
| Determinism | two independent launches → identical world file (SHA-256), identical entity poses (±1 mm), and the same scripted drive ends within 1 cm / 0.5° |

- **Findings that constrain later phases:**
  1. **RGB and depth are separate sensors whose content is offset by ~40 ms** (~10 px at 0.4 rad/s even with matching timestamps). *Correction from Phase 4:* the depth stream — and the scan derived from it — is exactly time-aligned to the true pose (0.06 cm ray-cast error at +0 ms), so it is the **RGB** image that is offset. A single depth pixel at an RGB blob's centroid from a *moving* camera can land on the wall behind a thin object — the first survey produced phantom "victims" on the walls exactly this way. Phase 5 must look while stationary/slow, or use aligned depth with a robust depth over the blob's pixels. The tests use step-and-look.
  2. **Depth is the stream that suffers under load** — see Environment.md.
- **TODOs (remain):** scenario randomisation for Phase 11 (the YAML + validator already support generated layouts); fire is not dynamic; victim mass/ring geometry must match the real victims and magnet holding force; wall/obstacle appearance is not realistic by design; sensor noise still absent.

### Phase 4 — Step 0 findings (measured; reproduce with `tests/sim/experiments/slam_fov_spike.py`)
Before building Phase 4, a viability experiment ran SLAM Toolbox at the placeholder 60° depth FOV and at a RealSense-like 87°, on the arena, from one start pose over one route (three 360° spins, 2.55 m, turn, 1.1 m), scored against Gazebo ground truth. **The plan's original assumption — that SLAM Toolbox plus an initial in-place rotation would work — was wrong.**

| Configuration | 60° final yaw / pos | 87° final yaw / pos | 87° map cells ≤10 cm from a real surface |
| --- | --- | --- | --- |
| A default-style matcher, every scan | 147° / 428 cm | 107° / 378 cm | ~25% |
| B tight matcher, every scan | 64° / 226 cm | 41° / 138 cm | ~29% |
| C default + loop closure | 61° / 543 cm (`map→odom` jump 343 cm) | 79° / 301 cm (jump 166 cm) | ~30% |
| **E rotation-only scans skipped, default matcher** | 2.8° / 36 cm (jump 26 cm) | **0.3° / 13.6 cm (jump 6.8 cm)** | **100%** |
| **F rotation-only scans skipped, tight matcher** | 1.1° / 15 cm (jump 48 cm) | **0.0° / 8.9 cm (jump 10.5 cm)** | **100%** |
| D matching off (odometry only — **not SLAM**) | 0 / 0 | 0 / 0 | 100% |

Findings:
1. The depth→`/scan` pipeline is geometrically exact (median 0.09 cm vs a ray-cast of the true scenario at 60°, 0.17 cm at 87°) and time-aligned to the true pose (+0 ms). The SLAM problem is the scan matcher, not the scan.
2. SLAM Toolbox gates scans on translation of `base_link`; this robot pivots exactly about `base_link`, so with default settings **in-place rotation adds nothing to the map**. Forcing `minimum_travel_distance: 0` integrates rotation — and then the matcher **diverges systematically on a narrow wedge** (yaw under-estimated every spin). Widening 60°→87° reduces but does not remove this.
3. The divergence is specific to rotation-only motion (in config A the yaw error changes only during spins). **Skipping rotation-only scans** (translation gate, heading gate disabled) gives genuine scan-matching SLAM that works, and works well at 87°.
4. Caveats, stated plainly: in-place spins do not map (the robot must translate to see new areas, and Nav2's spin recovery adds no map); one route, one start pose; simulated odometry is noise-free, so these errors are the matcher's own and hardware — with drifting encoder odometry and no IMU — will differ; run-to-run noise is about ±3 cm.
5. **Mapping with trusted simulator odometry is not scan-matching SLAM.** It is kept only as a labelled baseline preset.
6. Rejected/deferred alternatives: RTAB-Map (new dependency; the simulated arena is textureless, so visual odometry has nothing to track and ICP would face the same wedge degeneracy); an IMU (deferred by decision — encoder-only heading drift is recorded as a hardware risk); autonomous exploration (deferred to the rescue loop). AMCL against a saved map was probed at 60° (2–8 cm typical, one 60 cm transient) and is the Phase 4 localization option.

**Lap tuning (14.8 m lap, 87°, translation-gated tight matcher; `slam_fov_spike.py lap|lap2`).** The first Step-0 preset (0.1 m translation gate) drifted on a longer route — 19.8 cm / 6.3° final, 87% of map cells within 10 cm. Varying the matcher on that lap:

| Variant | Final pos / yaw | Worst waypoint | Cells ≤10 cm | Max `map→odom` jump |
| --- | --- | --- | --- | --- |
| gate 0.1 m (Step-0 preset) | 19.8 cm / +6.3° | 19.8 cm | 86.8% | 12.2 cm |
| gate 0.2 m, no barycenter | 10.7 cm / +2.7° | 10.7 cm | 99.2% | 13.7 cm |
| **gate 0.3 m** (3 runs) | **1.0 / 0.3 / 2.3 cm, ≤ +1.5°** | 2.6 / 3.6 / 4.8 cm | ≥ 99.1% | 7.8 cm |
| gate 0.4 m | 1.5 cm / +2.0° | 2.4 cm | 100% | 7.3 cm |
| gate 0.5 m | 2.8 cm / +0.9° | 2.8 cm | 100% | 7.8 cm |
| tighter search (dim 0.1, penalties 0.1) | 88.8 cm / +23.9° | 88.8 cm | 40.9% | 23.6 cm |
| scan buffer 3 | 33.8 cm / −3.1° | 33.8 cm | 60.9% | 39.8 cm |

Larger gates accumulate less bias; 0.3 m is the smallest in the good regime and is the shipped preset. **Tighter is not monotonically better** — the very tight matcher and the small scan buffer were much worse. The spread across variants (1 cm to 89 cm) shows this behaviour is sensitive, which is why the shipped value was confirmed by three repeats and is re-checked by the mapping test on every run. Still noise-free odometry on a single route family, so hardware will differ.

**Decision:** the navigation stack runs with an 87° depth camera and SLAM Toolbox with rotation-only scans skipped and a tightened matcher. The Phase 2–3 launch defaults stay 60° so their behaviour and tests are unchanged.

### Phase 4 — SLAM, localization and Nav2 ✅ COMPLETE
> **What "complete" means here, stated plainly.** The stack maps the arena with genuine scan-matching SLAM, localizes against a saved map with AMCL, and Nav2 drives to supplied goals. It does **not** run Nav2 concurrently with scan-matching SLAM reliably (a measured limitation, below), the results are on noise-free simulated odometry, and there is no autonomous exploration. The Step 0 findings above overturned the original plan (SLAM Toolbox + an in-place spin); this section records what was built instead.

- **Goal:** `/scan`, `/map` and `map→odom` from depth alone (no LiDAR), and Nav2 navigation to goals — behind interfaces that hardware and any replacement backend keep.
- **Created:** package `fire_resq_navigation` (Architecture.md §9 amended): `scan.launch.py` (depth→`/scan`), `mapping.launch.py` (SLAM Toolbox, presets `scan_matching` and `odom_only`), `localization.launch.py` (map_server + AMCL), `navigation.launch.py` (controller, planner, behaviours, BT navigator, lifecycle manager), `nav_stack.launch.py` (composes them; `localization:=slam|amcl|none` — exactly one `map→odom` publisher, invalid combinations rejected), `robot_params.py` / `nav2_params.py` (footprint and limits derived from `parameters.xacro`, never restated), `rviz/navigation.rviz`. `simulation/launch/arena_nav.launch.py` composes it with the arena; `camera_hfov` became a launch argument (default unchanged at 60°; the navigation launch uses 87°).
- **Architecture decisions (each measured):**
  1. **87° depth camera** in the navigation configuration (a RealSense-like FOV); Phase 2–3 launches stay 60°.
  2. **SLAM Toolbox, scan-matching preset:** rotation-only scans skipped, 0.3 m translation gate, tightened matcher, no loop closure (tuning table above).
  3. **Mapped-arena mode:** map with SLAM, save, then **AMCL** on the saved map for navigation. AMCL handles in-place rotation and Nav2's arcs; scan-matching SLAM does not (see the limitation).
  4. **Nav2 composed from four servers**, Regulated Pure Pursuit + NavFn, behaviours `spin`/`wait` only, a BT that replans only when the path becomes invalid. No BackUp/reversing/recovery: the robot cannot see behind itself.
  5. **Rolling 10 × 10 m global costmap** and **circular** costmap footprint (two integration failures found and fixed, below).
- **Tests added** (all reuse `run_sim`/`Bot`): unit — `test_robot_params`, `test_nav2_params`, `test_nav_config`, scenario ray-cast/nearest-surface helpers; sim — `test_nav_scan` (12), `test_nav_mapping` (8), `test_nav_localization` (5), `test_nav2` (9 + 1 xfail). The Phase 1–3 tests are unchanged and pass.
- **Verification results** (ground truth = Gazebo pose and the scenario geometry):

| Requirement | Result |
| --- | --- |
| Depth image exists; `/scan` valid | depth 32FC1 at the 87° intrinsics; `/scan` 640 beams, 86.9°, `camera_link`, ≥ 15 Hz; **median ray-cast error < 1 cm and p95 < 5 cm from four headings**, no phantom floor returns; scan stamps equal depth stamps |
| TF has no conflicting publishers | one parent per frame; `map→odom` published by exactly one node (`slam_toolbox` or `amcl`), asserted for each |
| SLAM produces a map; `map` frame exists | `/map` at 0.05 m from the first scan; SLAM Toolbox lifecycle `active` |
| SLAM localizes | 14.8 m mapping lap: over two full test runs the worst waypoint error was **4.1 and 3.5 cm**, the final error 1.7 and 3.5 cm, yaw ≤ 1.8°; largest single `map→odom` step 7.3 and 10.0 cm (≤ 1.6°). Across the tuning repeats at this setting, final error 0.3–2.3 cm |
| Map quality | **100% of occupied cells within 10 cm** of a real surface (both runs), 76% and 92% within 5 cm, median 2.5 cm (~730 cells); **97% of the true wall length mapped**; 24.5–24.6 m² of the 25 m² arena known |
| Localization against the saved map | on the saved map, from the mission start pose: **3.0 and 2.5 cm** (≤ 0.3°); on a *different* route with in-place spins: median **5.4 and 1.7 cm**, worst **9.7 and 7.2 cm**, yaw ≤ 2.5° (the SLAM preset would skip those spins; AMCL handles them) |
| Nav2 lifecycle active; goal accepted; robot moves toward it; completes within tolerance | four servers `active`; under AMCL on the saved map, three goals, identical in two full runs: **G1 succeeded in 9.2–9.3 s** (13 cm from the goal in the real world), **G2 — a detour round obstacle_2 — in 15.5–16.1 s** (9 cm from goal, ≥ 48 cm clear of the obstacle), **G3 home in 16.6–16.7 s** (16 and 8 cm); all ≤ the 25 cm test tolerance, which is Nav2's 10 cm goal tolerance plus localization error; robot upright and on its wheels throughout. SLAM and AMCL are non-deterministic, so figures vary run to run |
| No hardcoded scenario coordinates in navigation/cognition | unit guards: nothing under `src/` (now including `fire_resq_navigation`) reads the scenario or Gazebo ground truth |
| Existing Phase 1–3 tests | all pass unchanged |

- **Performance** (headless, D3D12; `nav_performance.py`; detail in Environment.md): real-time factor **1.00** throughout; depth 15–18 Hz, `/scan` 16–18.5 Hz, `/map` updates every 1.0 s; whole stack **1.7 cores** (Gazebo 0.9, bridges 0.3, SLAM + Nav2 + scan < 0.6); Nav2 drove a detour goal in 14–16 s.
- **Integration failures found and fixed** (each was diagnosed from logs and data, not guessed):
  1. *Nav2 aborted every goal in 0.03 s:* `Start Coordinates … was outside bounds`. The SLAM map covers only what has been seen, and the robot's own position was just outside it, so a map-sized global costmap did not contain the robot. Fixed: rolling 10 × 10 m global costmap.
  2. *Controller aborted with "collision ahead":* the planner plans for a point and the controller checks the footprint; with wheels protruding (9.8 cm) past the inscribed radius (7.5 cm) they disagreed. Fixed: a circle at the circumscribed radius (derived from the description). Slightly conservative; a tighter polygon is a TODO for the victim approach.
  3. *Stock SLAM Toolbox/Nav2 defaults were wrong for this robot* (`base_footprint` frame, 0.7 m inflation, TurtleBot MPPI controller, ten servers): all replaced explicitly, with unit tests that fail if they return.
  4. *Test-infrastructure bugs, also real:* the stray-process guard matched its own launcher command line and a script name; `use_rviz` leaked from `arena_nav.launch.py` into the arena launch and opened two RViz windows (launch arguments are global across included files).
- **KNOWN LIMITATION — Nav2 concurrent with scan-matching SLAM.** With SLAM Toolbox localizing and Nav2 driving, the pose stayed within 2 cm through Nav2's in-place rotation, then jumped **+44° in a single step** at the first scan processed after it (2.2 cm → 16 cm, growing linearly afterwards). Odometry matched ground truth to < 0.4 cm / 1.1° and no victim moved, so it is SLAM's correction, not physics or a collision. The identical goals all succeed with odometry-only mapping (8, 17, 18 s), which isolates it from the Nav2 configuration. The mechanism is not fully understood (a 44° step is outside the matcher's search window). Recorded as an `xfail` (non-strict) that will announce a fix; `tests/sim/experiments/nav2_localization_diag.py` reproduces it. Mitigation: navigate under AMCL on a saved map.
- **Other limitations:** in-place spins add nothing to the SLAM map (asserted); tuning evidence is one route family on noise-free odometry, so hardware will differ; the Nav2 goal tolerance (10 cm) and the AMCL/SLAM errors add; goals inside a victim's inflation radius will be rejected by Nav2 — relevant to the Phase 8–10 approach; unknown space is plannable (`allow_unknown`) because nothing explores yet.
- **TODOs (explicit in code and docs):** sensor and odometry noise so SLAM/AMCL are measured under realistic error; encoder-only heading drift on hardware (Hardware.md TODO; IMU deferred, not required); RGB-only visual-SLAM alternative; RealSense integration and validation of the depth-derived scan; better loop closure; dynamic environments; recovery from failed navigation; autonomous exploration (rescue loop); decoupled RGB/depth FOV once the camera is chosen; publish the mission start pose to AMCL automatically.

### Phase 5 — Perception ✅ COMPLETE
> **What "complete" means here, stated plainly.** Fire and victims are detected by colour and placed in a TF frame with centimetre-level error, from depth (RGB-D) or from object-height priors (RGB-only), through one message type; the RGB/depth timing offset is respected by refusing to claim a position while the camera turns. Positions are verified in `odom` (the arena launch has no SLAM); the `map` frame is the same code with `target_frame:=map`, exercised at node level with a synthetic transform but **not yet against a live SLAM/AMCL `map→odom`** (Phase 6 consumes it). Everything is on noise-free simulated sensors. Obstacles and the safe zone are not detected (nothing needs them yet).

- **Goal:** victims and fire detected and placed in a TF frame, camera-agnostically.
- **Created:** in `fire_resq_perception`: `Detector` ABC + `ColorBlobDetector` (HSV ranges → connected components; no morphology, which would erode the fire's apex; a blob touching the image border is flagged truncated), `SpatialEstimator` ABC + `DepthEstimator` and `KnownHeightEstimator`, `MotionGate` (camera-rotation/speed from TF over a short window), `PerceptionPipeline` (estimator chain: `auto` = depth when available, else known-height; `depth` / `known_height` are strict and never fall back), `PerceptionNode`, `config/perception.yaml` (colours, size priors, gate), `launch/perception.launch.py`. `fire_resq_description` gained a small Python module (`geometry.py`) that reads `parameters.xacro`, used by navigation and perception launches so the floor offset (wheel radius) is never restated. `arena.launch.py` / `arena_nav.launch.py` gained opt-in `perception:=true` (frames `odom` / `map` by default respectively).
- **Interface:** `/fire_resq/detections` (`DetectionArray`), stamped with the image time, `frame_id` = target frame. Every detection carries its bbox and confidence; `position_valid=false` means "seen but not placed" (the 2D detection is still reported). Camera-agnostic: needs only an RGB stream, `CameraInfo`, TF, and optionally depth.
- **Deviations from the plan (each needs your sign-off, see the report):** (1) **one node** with a pipeline, not two nodes (detector, estimator): the split lives in the library interfaces, and one process avoids shipping two full-rate image streams between nodes; (2) **`KnownHeightEstimator` (top edge) replaced `GroundPlaneEstimator`**, on measurement; (3) the description Python module; (4) node-level tests run rclpy in-process under pytest rather than `launch_testing`.
- **The RGB/depth offset, handled:** the RGB frame's own stamp is used to look up TF; a depth frame is paired only if it is within `depth_pair_max_dt` (0.1 s); positions are **withheld while the camera turns faster than 0.1 rad/s or drives faster than 0.3 m/s** (from `odom→base_link` at the image stamp), and the depth value is a low percentile over the blob mask, never one pixel. Blobs touching the image border get no position (a clipped blob has no reliable top or centre).
- **Tests added:** unit — `test_perception_lib` (65: camera model, transforms, detector, both estimators, motion gate, pipeline chain, config vs the real model SDFs), architecture guards (perception has no simulator/hardware imports; cognition cannot read `source_backend`), launch rules (perception opt-in, started once by the navigation launch); node — `tests/node/test_perception_node.py` (8; synthetic RGB/depth/CameraInfo/TF in-process: frame and stamp, both backends, strict backend never falls back, gating while turning and resuming, missing transform, bad config); sim — `test_perception` (8), `test_perception_nav` (87° camera through `arena_nav`), `test_perception_rgb` (RGB-only, slow). Reproducible measurement: `tests/sim/experiments/perception_accuracy.py`.
- **Verification results** (ground truth = scenario positions, compared in `odom`; `perception_accuracy.py`, five estimator configurations side by side, 12 headings × 6 stations, stopping at every heading):

| Requirement | Result |
| --- | --- |
| Depth backend accuracy | **victims: median 0.9–1.4 cm** from 0.5 to 9 m (p90 ≤ 5 cm; a few 12 cm outliers at 4–9 m with the 87° camera, where a victim is ~12 px wide); **fire: median 0.8–3.9 cm**, radial bias ≈ 0 after tuning |
| RGB-only backend accuracy | top anchor: **victims median 0.2–1.3 cm to 3 m, ~4 cm at 3–4 m, up to 11 cm beyond 4 m** (error grows with range², 1 px ≈ 4 cm at 3 m); fire 1–4 cm, biased 1–4 cm too near |
| The bottom-anchor (ground-plane) method it replaced | victims **2 cm at 1–2 m, 15 cm at 2–3 m, 44 cm at 4–9 m**, fire **57 cm**, with 100–200 detections placed at no real object: measured worse than useless beyond 2 m, hence the top anchor |
| Same message from both backends | node test + sim: identical fields, only `source_backend` differs; RGB-only robot (`use_depth:=false`) finds fire, victim_1, victim_2 within `0.05 m + 2 %·range` |
| No false positives | every valid detection in a full 360° look, in three scenarios, is within 0.5 m of a real fire/victim (floor, walls, obstacles, safe zone never match: the colour contract) |
| Hidden victim | victim_3 (behind the partition) is not reported from the start pose and is found within tolerance once the robot has line of sight |
| Timing offset respected | while turning at 0.4 rad/s: 2D detections continue, **no position is published**, positions resume within 2 s of stopping; driving at 0.15 m/s still places objects |
| What the gate costs and buys (ungated nodes, same scene) | turning 0.4 rad/s ungated: depth **median 1.2–1.6 cm, max 3.5 cm** (vs 1.0 cm still); driving 0.2 m/s: 1.7 cm. **The offset is real but small for these compact objects; the gate is conservative** and its thresholds are parameters |
| Rates and cost | ~22–25 Hz output, stamps within 0.25 s of the image; node **0.49 cores**, 5.6–5.9 ms of processing per frame (pipeline itself 4–6 ms) |
| Existing Phase 1–4 tests | all 131 Phase 1–4 tests and the 1 recorded xfail pass unchanged. **Full suite after a clean rebuild: 217 passed, 1 xfail in 25 min 33 s** (134 unit, 8 node, 76 sim; 86 tests added by this phase) |

- **Measurement-driven changes:** the fire's `axis_offset_m` was 0.085 (cone radius at the silhouette centroid) and left a **−3 cm bias** in both estimators; the surface a depth ray or top edge finds is the lower cone, so it is now 0.115 (a unit test bounds it between the centroid radius and the base radius). A first version ran under a `MultiThreadedExecutor` and cost **1.24 cores** (rclpy wait-set bookkeeping, 24.7 ms per frame for 5 ms of work); a single-threaded node with the TF listener on its own thread costs 0.49.
- **Limitations:**
  1. **Close objects get no position.** A victim nearer than roughly 0.5–1 m overflows the 60°/87° camera's vertical field (the camera is 8.75 cm above the floor); its blob touches the image border and is reported 2D-only. The Phase 9–10 approach must rely on the world model's last good position, not on live perception, for the last metre.
  2. **Colour is the whole detector.** It relies on the Phase 3 colour contract in simulation; on a real camera lighting will break it. `YoloDetector` sits behind the same interface for that case; not built, because nothing here shows the MVP detector insufficient.
  3. **No occlusion handling:** a partly hidden object gives a biased blob; only border truncation is caught.
  4. **RGB-only accuracy is range-limited by pixels** (≈ 4 cm/px at 3 m, ~11 cm beyond 4 m): fine for approach planning, marginal for the final magnet alignment.
  5. **Priors are per model.** Heights and axis offsets in `perception.yaml` describe the current victim and fire models (a unit test checks them against the SDFs); a new fire model needs new priors.
  6. **Noise-free sensors; `odom`-frame verification only** (see the box above). Depth noise, lens distortion and camera calibration error are untested.
  7. Confidence is the detector's own pixel-count score; it is not geometric.
  8. **One unexplained intermittent test failure seen during this phase.** Twice, `test_spin_in_place` (a Phase 2 test; 84.7° and −20° for a commanded 90°) failed when run first on a freshly spawned robot under `-k`; it passed on the untouched Phase 4 commit, passed in the full module, passed in two immediate reruns of the identical command on this tree, and passed in the final full run. No change of this phase touches robot physics (the perception launch options are off by default), but the cause was not found. If it recurs, suspect the first-command-after-spawn settling window under machine load.
- **TODOs (explicit in code and docs):** uncertainty/covariance on positions; active perception (turn to centre a clipped blob); `YoloDetector`; real-camera calibration and RealSense validation; obstacle and safe-zone detection; occlusion handling; sensor noise.

### Phase 6 — World model ✅ COMPLETE
> **What "complete" means here, stated plainly.** The world model turns placed detections into a stable, persistent belief — three victims stay three entities with fixed identities, a victim first seen late is added without disturbing the others, confidence decays when a victim is out of view, and `RESCUED` is permanent — and publishes it in the `map` frame with a rescue-status service. It was verified in the real simulation against ground truth with live SLAM providing `map→odom`. It does **not** prioritise, plan or explore, its tracker is a smoothed nearest-neighbour placeholder, and the sensors are still noise-free.

- **Goal:** stable beliefs from noisy detections, owned in one place.
- **Created:** in `fire_resq_world_model` — a **pure-Python library** (`config`, `entities`, `world_model`, `markers`: explicit time, no ROS, so every rule is unit-tested deterministically) and a thin `world_model_node`, `config/world_model.yaml`, `launch/world_model.launch.py`. `arena_nav.launch.py` gained opt-in `world_model:=true` (needs `perception:=true` and a `map` frame); the navigation RViz view gained a MarkerArray display for `/fire_resq/world_markers`. **No interface changed:** `WorldState`, `VictimState` and `UpdateVictimStatus` were sufficient as defined.
- **Interfaces:** consumes `/fire_resq/detections` and TF; publishes `/fire_resq/world_state` (5 Hz, `map`, robot pose from `map→base_link`) and `/fire_resq/world_markers`; serves `/fire_resq/update_victim_status`.
- **How it behaves (each rule has a unit test):**
  - *Association:* only **placed** detections (`position_valid`) are used; nearest track within a 0.3 m gate (the scenario validator keeps victims 0.5 m apart), strongest detection first, so two blobs of one split victim merge into one track and count as one observation. Positions are an EMA (α 0.3). No detection ever reads which camera backend produced it.
  - *Confidence:* the latest detection's, published as `latest · exp(−unseen / 30 s)`; never deleted, just less credible. Detections below 0.2 (perception's own floor) are dropped.
  - *Identity and new victims:* ids are `V1, V2, …` in creation order, never reused, and deliberately unlike the scenario's `victim_N` (the model does not know those). A new track is `UNKNOWN` (tentative) until a second message confirms it → `DETECTED`; a track never confirmed within 5 s is a ghost and is dropped. Confirmed victims are never dropped.
  - *Status lifecycle* (`UpdateVictimStatus`): `DETECTED→TARGETED/UNREACHABLE`, `TARGETED→DETECTED/CARRIED/UNREACHABLE`, `CARRIED→RESCUED/DETECTED`, `UNREACHABLE→DETECTED` (explicit retry); `UNKNOWN` cannot be requested and **`RESCUED` has no exits**. One active target (`TARGETED`/`CARRIED`) at a time, published as `current_target_id`. Refusals return a reason. A `RESCUED` or `CARRIED` track absorbs detections without moving, so seeing it again cannot resurrect it or create a duplicate.
  - *Safe zone:* configured infrastructure (a rectangle in the `map` frame, `world_model.yaml`), never discovered. A victim detected **inside** it never creates a track — otherwise a released victim would appear as a phantom fourth candidate.
  - *Fire:* one entity, same EMA/confidence rules, its own 0.5 m gate; a second far fire is ignored and counted (TODO).
- **Tests added:** unit — `test_world_model_lib` (36: creation, association, fragment merging, identity under noise, absence and return, late victims, EMA, confidence decay, validity, fire, the whole lifecycle, RESCUED terminal, safe zone, markers, agreement with `VictimState.msg`) and `test_world_model_config` (7: valid config, every parameter declared, **config safe zone equals the scenario's**, layering guards, library imports no ROS, launch opt-in, RViz display); node — `test_world_model_node` (15, in-process, with a `map→odom` that is offset *and* rotated so a wrong transform gives wrong numbers; a mutation that disables the transform is caught); sim — `test_world_model` (7, arena_nav + scan-matching SLAM + perception + world model).
- **Verification results** (ground truth = scenario positions; `map` starts at the start pose, so truth is `world_to_odom`, up to SLAM's error):

| Requirement | Result |
| --- | --- |
| Beliefs in a real `map` frame | `WorldState` in `map`, 5.0 Hz; robot pose within 5 cm of the test's own `map→base_link`; safe zone equal to the configured one |
| Visible victims and fire believed where they are | from the start pose exactly **two** victims (the third is hidden) and the fire: **victims 1.2 and 1.0 cm, fire 0.4 cm** from truth; each real victim believed exactly once |
| No duplicates, stable identity | a full 360° look (12 stops): same ids, nobody moved > 6 cm |
| Confidence decays out of view, recovers in view | facing the wall for one time constant (30 s sim): every victim below 0.6 × its earlier confidence, none deleted; back in view: recovered to ≥ 0.8 × |
| Hidden → visible | driving a translation route to line of sight: **a third entity appears, the original two keep their ids**, new entity **7.5–8.8 cm** from victim_3 (two runs; the larger error is the map frame after driving, where SLAM has moved) |
| Rescued stays rescued | after `TARGETED→CARRIED→RESCUED`, 8 s of the victim still in view: three entities, still `RESCUED`, no target; `RESCUED→DETECTED` refused as terminal; an illegal `DETECTED→CARRIED` refused |
| Cost | **0.23–0.27 cores** (the Python TF listener dominates), `WorldState` at 5 Hz |
| Existing Phase 1–5 tests | **No regression attributable to this phase, but a clean full-suite pass was not obtained.** The last complete run (clean rebuild, 59 min) gave 275 passed, 1 xfail, **7 failed**: six Phase 2–4 timing tests (`test_scan_rate`, both `test_spin_in_place`, both speed-cap tests, `test_topic_rates`) and one Phase 6 wall-clock rate test. The simulator ran at ≈ 0.3× real time then (the Phase 6 test read 1.5 Hz from a 5 Hz timer). Reruns on a quiet host: robot module 9–10 of 10 (which tests fail changes run to run); Nav2 module 8 passed + xfail + 1 goal aborted with the controller "missed its desired rate". **A/B on the untouched Phase 5 commit in a clean worktree: 4 of 10 robot tests failed in one run and 0 in the next.** All Phase 6 unit (43), node (15) and simulation (7) tests pass; the rate test now counts per *simulation* second |

- **Decisions and deviations from §9 (flagged in the report):**
  1. `UNKNOWN` is used as the *tentative* status (confirmation needs two observations; `confirm_hits: 1` disables it). Not in §9; it makes ghosts and one-frame false positives invisible to cognition, which only needs to consider `DETECTED` and above.
  2. **`min_confidence` is 0.2, not the first-guess 0.3.** Detector confidence is a pixel-count proxy: a victim ~4.7 m away at 87° reads 0.32, so a 0.3 floor would make victims beyond ~5 m impossible to create. (Found by the simulation test; ghosts are filtered by confirmation instead.)
  3. **The safe zone is configuration** in the `map` frame (the map origin is the mission start pose, inside the zone) — the plan said "a parameter". A unit test fails if it drifts from the scenario, because the model cannot read the scenario.
  4. Nothing is published until `map→base_link` exists: the belief is expressed in `map`, so without localisation it would be wrong, not merely uncertain.
  5. `world_model_node` is one process; the marker builder is a pure function so demos can be unit-tested.
- **Limitations:**
  1. **Unplaced detections are dropped entirely.** A victim closer than ~0.5–1 m is only seen 2D (Phase 5), so while the robot is beside it its confidence *decays* though it is right there; there is no negative evidence either way. Phase 9–10's approach must not read a low confidence next to the target as "lost".
  2. **The tracker is nearest-neighbour + EMA.** Two victims closer than about twice the gate could merge; the scenario validator prevents that here, hardware layouts must respect it. No covariance, no motion model; a moved victim is a new victim unless it moves within the gate.
  3. **`CARRIED` tracks are frozen**, not following the robot (Phase 9 attaches here).
  4. **Tracks are stored in `map` at observation time.** SLAM's later corrections are blended in by the EMA but old tracks are not re-anchored; there is no loop closure to cause big jumps in this configuration.
  5. **One fire, no obstacles** (`WorldState` has no obstacle field; the SLAM grid is the only obstacle memory).
  6. **Confidence is the detector's pixel-count score**, low for far victims: honest, but it means Phase 8 may penalise a far victim twice (its confidence and its distance); weigh that when tuning.
  7. **RViz rendering not verified by eye** (screen capture is unavailable under WSLg, as in Phase 4): marker frames, types, colours, labels and lifetimes are asserted in tests; whether they *look* right in RViz has not been seen.
  8. The status service has no caller authentication; any node may change a status.
  9. **Test-suite reliability on this host.** The Phase 2–4 timing tests (robot motion, topic and scan rates, Nav2 goals) are intermittently unreliable when the host runs the simulator below real time; the cause of the slowdown (Windows-side contention; the C: drive was 93 % full) was not established. Check `gz topic -e -t /stats -n 4` (real-time factor ≈ 1.0) before trusting a failure, and A/B against a clean worktree of the last accepted commit before suspecting code. Not fixed here: weakening those tests was out of scope.
- **TODOs (explicit in code and docs):** Bayesian belief updates and covariance tracking; negative evidence; active perception; multiple fires and obstacle memory; a `CARRIED` track following the robot; re-anchoring tracks after map corrections; battery/resource state and dynamic fire risk (cognition's, not this package's).

### Phase 7 — Nav2 planning and reliability ✅ COMPLETE *(after cognition, which was built first)*
> **What "complete" means here, stated plainly.** The intermittent Phase 4 detour-goal failure was reproduced, root-caused to a controller latch, and fixed with a one-line configuration change that was measured (60 of 60 goals, against 12–44 % failing before). The planner interface cognition uses is more reliable (a path must end at its goal, "unreachable" and "unknown" stay apart, repeated questions are answered from memory, a missing planner is noticed once). Navigation to what the rescue loop will ask for — every victim's approach pose, out and home — is validated. It does **not** add recovery behaviours, exploration or anything for the executable approach (Phase 10), and the SLAM + Nav2 concurrency limitation and the second flaky Phase 4 test (map precision) are unchanged and recorded below.

- **Goal:** point-to-point autonomy that is reliable enough to serve the rescue loop, plus path length and reachability that cognition can trust.
- **Root cause of the intermittent goal failure** (`tests/sim/experiments/nav_goal_diag.py` reproduces it; one AMCL+Nav2 session on one saved map, AMCL re-initialised at the mission start pose before each trial to get fresh-session randomness):
  - *Reproduction, before touching anything:* over five runs on the same map the detour goal G2 failed **8 of 66 goals (12 %)** and **G1 35 of 80 (44 %)** (G3, the goal home, 0 of 80); a single long session *without* re-initialisation failed 0 of 30, which is why it looked rare. Signature: the robot creeps in at the controller's minimum approach speed, stops about 10 cm from the goal, and from then on commands **linear velocity exactly 0** with angular commands of ±0.13–0.29 rad/s alternating (in-place rotation) until the progress checker aborts 15 s later ("Failed to make progress"). At the moment it stops, AMCL's distance to the goal is the same in passes and failures (mean 9.74 cm); failures then end at 10.0–12.8 cm, passes at about 9.4 cm.
  - *Mechanism* (upstream Nav2 Jazzy `regulated_pure_pursuit_controller.cpp`, present in the installed 1.3.13; `stateful` confirmed in the library): `shouldRotateToGoalHeading` **latches** `has_reached_xy_tolerance_` the first control cycle the controller's own distance to the goal drops below the goal checker's `xy_goal_tolerance`, and while latched the controller only rotates in place. The goal checker independently needs *its own* evaluation to see the robot inside the same 10 cm, a control cycle (and, once the robot starts turning, an AMCL update of 0.5–1.4 cm) apart. When the checker's estimate is a fraction of a centimetre outside at that moment, the robot is frozen outside the tolerance and nothing can move it back in. Success is therefore decided by the sign of a sub-centimetre difference.
  - *Ruled out with data:* the world drifting over a session (victims and fire displaced **0.0 cm** over 14 trials); the planner's path ending short of the goal (24 random goals: the path ends exactly at the goal, 0.00 cm, for every free goal); a stale goal transform in the goal checker (a predicted distance under that hypothesis did not separate passes from failures, so it was dropped). *Not concluded:* two controller-parameter sweeps (collision detection, velocity regulation, rotate settings) were **invalidated by a flaw in the experiment** — a variant that strands the robot leaves AMCL initialised at the map origin while the robot is elsewhere, and every later trial aborts within a second — so nothing is claimed from them.
- **The fix:** `FollowPath.stateful: false` in `nav2_params.yaml` (with the rationale in the file). The controller re-evaluates "am I inside the tolerance" every cycle instead of latching, so if the estimate drifts outward it resumes creeping in until the goal checker — which does latch, correctly, on its own evaluation — is satisfied. *Measured after (same harness, same map, fresh-AMCL every trial, live parameter confirmed):* **G1 20/20, G2 20/20, G3 20/20 (60 of 60)**; one G1 took 31.6 s, which is the recovery working (before, that trial aborted). A false-alarm test in the other direction: `use_rotate_to_heading` stays true and the progress checker still aborts a genuinely stuck robot (unit-guarded).
- **Regression tests** (each shown able to fail): unit `test_the_controller_does_not_latch_arrival_at_the_goal_tolerance` and `test_the_progress_checker_still_aborts_a_robot_that_really_is_stuck`; simulation `test_repeated_goals_at_the_tolerance_edge_never_deadlock` (five out-and-back rounds of the goal that failed most: **with the bug put back it failed; with the fix all ten goals succeed**).
- **The planning interface** (`fire_resq_cognition`; cognition still sees only `PathInfo` and a callable):
  - *Valid / unreachable / unknown stay apart.* New: a path is `reachable` only if its last pose is within one costmap cell (5 cm) of the goal asked about. NavFn's 0.2 m `tolerance` had returned "success" with a path to the nearest free cell for a goal inside an obstacle (measured: the centre of the 15 cm partition came back reachable, a 2.27 m path); it is now `unreachable` with the shortfall in the reason. Free goals end at exactly 0.00 cm, so no valid goal is rejected.
  - *Fewer planner calls.* `CachedPathQuery` remembers a **reachable** answer for 15 s (start and goal matched to 5 cm); `unreachable` and `unknown` are never cached (they are the transient ones), and a retry or the periodic refresh bypasses the cache. `SelectTarget` right after an automatic decision costs the planner nothing (node test). The adapter notices a missing planner once and answers `unknown` at once for 2 s instead of waiting 0.5 s per question (6 questions: first ≥ 0.4 s, the next six < 0.2 s in total).
  - *Approach pose.* `approach_pose(robot, victim, standoff)` returns the standoff point and a heading that faces the victim, and the node uses it for `RescueTarget.approach_pose`. **Contract** for Phase 10: a point 0.4 m from the victim's centre on the robot→victim line, in `map`, facing the victim; Nav2 can plan and drive to it; the last stretch to the magnet (`ALIGN`) is open-loop and Phase 10's. Derived from the robot's own description (a unit guard fails if the geometry or the victim model changes it): Nav2 can plan no closer than **0.265 m** to a victim's centre (circumscribed radius 0.159 + ring 0.056 + one cell), magnet contact is at **0.131 m** (magnet face 0.075 m ahead of `base_link` + ring 0.056), so the 0.4 m standoff leaves a **0.269 m** creep. Cognition and planning still share nothing but the message.
- **Navigation validated** (`tests/sim/test_nav_planning.py`, AMCL on the saved map, judged against Gazebo's truth; 10 tests): planned lengths are never shorter than the straight line; route directness (1 = straight) **victim_1 0.99, victim_2 0.99, victim_3 0.62** (the partition's detour); every victim's approach point is reachable out and home; the same question gives the same length. *Driving to each approach pose* (position and facing heading), from the start: victim_1 7 s, 12.1 cm from the approach point, 52.1 cm from the victim; victim_2 21 s, 11.6 cm, 49.2 cm; victim_3 (the detour) 25 s, 7.2 cm, 42.2 cm; **no victim moved (< 1 cm)** and the robot never came within 25 cm of one; home again after each. The original goals: G1 8.9 s, G2 (detour) 16.5 s, G3 16.8 s.
- **The second flaky Phase 4 test, `test_occupied_cells_lie_on_real_surfaces`, now measures aligned map shape, not registration offset (maintainer-approved change, applied).** Eleven earlier maps (`tests/sim/experiments/map_precision_diag.py`) showed the share of occupied cells within 5 cm of a true surface at **68.0–92.5 % raw (mean 80.1)** — the 70 % threshold sits in the tail, two of eleven at or under it — with a systematic **~4 cm rigid offset from Gazebo's frame** in every map (e.g. (−1, −4), (0, −6), (−2, −6) cm; cause not isolated), and **87.0–98.5 % (mean 94.1)** once that offset is removed. The 10 cm criterion was similarly thin raw (95.8 %) and comfortable aligned (≥ 97.8 %). **The fix:** the test now grid-searches the best-fit rigid translation (±8 cm, 1 cm step — the same search `map_precision_diag.py` uses) and applies the SAME 70 % / 95 % thresholds, unchanged since Phase 4, to the aligned distances, plus a new hard bound that the offset itself must be **≤ 10 cm** — so a map that is genuinely mis-registered, not merely quantised, still fails. *Verification run (`best_rigid_offset`, post-change):* offset **(−1.0, −4.0) cm = 4.1 cm** (well inside the 10 cm bound), aligned precision **90.0 % within 5 cm, 100.0 % within 10 cm**.
- **Limitations (unchanged or found here):**
  1. **Known and unchanged:** Nav2 concurrent with scan-matching SLAM (the recorded `xfail`); no recovery behaviours, no reversing; the robot is blind behind and within ~0.5 m ahead.
  2. **Unknown space is plannable** (`allow_unknown: true`): a goal at the centre of a 0.5 m block (whose interior the map never saw) comes back reachable with a detour into it. Recorded as an `xfail`; cognition is unaffected (approach points are outside victims).
  3. **Goals end at the tolerance edge by design:** the controller stops as soon as it is inside 10 cm, so truth is 6–14 cm from the goal (true position is judged with a 25 cm test tolerance; goals that need centimetre accuracy, e.g. the magnet alignment, cannot rely on this).
  4. Path lengths come from the costmap at the moment of the question; a stale costmap gives a stale answer for up to the 15 s cache time.
  5. The controller's `stateful: false` trades the deadlock for possible brief rotate/creep hunting at the boundary under AMCL jitter; bounded by the progress checker.
  6. The saved map is ~4 cm offset from Gazebo's frame (above), which the tests' world↔map conversions inherit.
- **Tests and results.** *First complete clean run, before the map-precision metric change (clean rebuild 15 s, real-time factor 1.0 throughout):* **401 passed, 2 xfailed, 0 failed, 0 errors in 34 min 19 s** (403 tests: 255 unit, 48 node, 100 sim). *Final complete run, after the metric change (clean rebuild, real-time factor 1.0 throughout):* **401 passed, 2 xfailed, 0 failed, 0 errors in 33 min 46 s.** The two xfails, both runs, are the recorded limitations: SLAM Toolbox + Nav2 concurrency (`test_known_limitation_nav2_under_scan_matching_slam`, unchanged) and a goal inside a solid block being reported reachable (`test_known_limitation_a_goal_inside_a_block_is_reported_reachable`, caused by `allow_unknown`). **`test_occupied_cells_lie_on_real_surfaces` is now fixed, not just lucky:** the final run measured a map offset of (+1.0, −3.0) cm = 3.2 cm (well inside the new 10 cm bound) and aligned precision 99.9 % within 5 cm, 100.0 % within 10 cm — comfortably over the unchanged 70 %/95 % thresholds. **`test_scan_rate` remains intermittent and environmental, threshold unchanged:** `tests/sim/experiments/sensor_rate_diag.py` measured the depth/`/scan` render rate per *simulation* second over 24 windows at 11–28 Hz on both this tree and the accepted Phase 6 commit (the test's floor is 12 Hz), so a low window is the renderer, not the navigation code; it passed in both final runs. An earlier complete run this phase (before the harness hardening below) gave 391 passed, 2 xfailed, 1 failed (`test_scan_rate`), 9 errors (Nav2 module: AMCL lifecycle bring-up timeout on a stalling host). **Harness hardening made for the errors (no test threshold touched):** `simlib.lifecycle_state()` asks the lifecycle `GetState` service instead of the `ros2` CLI (returns `unknown` on a slow answer so wait loops keep polling); the AMCL bring-up timeout now raises with the lifecycle states and the launch-log tail; the stray-process guard also covers the perception, world-model and cognition nodes; `FIRE_RESQ_TEST_MAP` reuses one saved map for A/B runs. Measured values from the final run: goal G1 succeeded in 9.2 s (closest 0.12 m); detour G2 succeeded in 16.2 s, 10.5 cm from the goal, 50 cm clear of `obstacle_2`; home G3 succeeded in 16.9 s, 7.0 cm from the goal; route directness victim_1/2/3 = 0.99 / 0.99 / 0.62; wall recall 96.2 %; approach-pose navigation succeeded for all three victims (12.7, 11.8, 6.7 cm from the approach point; 52.7, 50.1, 41.8 cm from the victim); standoff 0.4 m against a Nav2 floor of 0.265 m, magnet contact 0.131 m, ALIGN creep 0.269 m.
- **TODOs (explicit):** recovery behaviours; tighten `allow_unknown`; a final-approach controller for `ALIGN` (Phase 10); `Nav2PathQuery` stays inside `fire_resq_cognition` for now (maintainer's decision: revisit only if another package needs the same adapter) — not moved.

### Phase 8 — Cognitive prioritization ✅ COMPLETE *(built before Phase 7, on the maintainer's instruction; the session called it "Phase 7")*
> **What "complete" means here, stated plainly.** The robot decides *which victim to rescue next* from its current world model, with the real Nav2 planner supplying path lengths and reachability, and every decision explains itself. It **selects; it does not act** — marking a victim `TARGETED`, navigating and the rest of the loop are Phase 10. The decision quality is only as good as the defaults, which are a stated stance and not a fitted result (see "Decisions" below). **Phase 7 (Nav2 reliability and the path-query helper) is not done**: only the narrow query adapter cognition needs was built.

- **Goal:** dynamic, explainable target selection behind a replaceable interface, with the PRD §11 baseline.
- **Created:** in `fire_resq_cognition` — a **ROS-free library** (`types`, `config`, `factors`, `prioritizer`: the `VictimPrioritizer` ABC, `WeightedUtilityPrioritizer`, `NearestVictimPrioritizer`, a factory keyed by the `decision_model` parameter), `nav2_path_query.py` (the only place cognition touches navigation, and it only *asks*), `prioritizer_node`, `config/prioritizer.yaml`, `launch/prioritizer.launch.py`. `arena_nav.launch.py` gained opt-in `cognition:=true` and `decision_model:=`. **No interface changed:** `SelectTarget`, `RescueTarget`, `ScoreBreakdown` are used as defined.
- **Interfaces:** consumes `/fire_resq/world_state` only; asks Nav2's `/compute_path_to_pose`; publishes `/fire_resq/rescue_target` (latched; an empty `victim_id` means nothing selectable, reason in `rationale`), `/fire_resq/decision_report` (latched JSON, below) and serves `/fire_resq/select_target`.
- **The decision model** (`weighted_utility`). For each candidate (status `DETECTED` or `TARGETED`, reachable out *and* back):

```text
U =  w_risk           · fire_risk(d_fire)             1 inside the hazard radius (1 m), exp(−(d−1)/1) beyond
   + w_fire_proximity · (1 − d_fire / extent)         linear over the arena diagonal (7.07 m)
   + w_reach          · (1 − trip_out / extent)       trip_out = Nav2 path, robot → approach point
   + w_accessibility  · straight / path               directness of that route, (0, 1]
   + w_confidence     · perception confidence         from the world model (already decayed)
   − w_cost           · (trip_out + trip_back) / (2 · extent)     back = approach point → safe zone, also a Nav2 path
```

  Default weights are priority tiers, **not** fitted numbers: risk 3 (safety) · fire-proximity, reach, cost 1 (practical) · accessibility, confidence 0.5 (tie-breakers). Every weight is a parameter (`weights.*`; a unit test forbids literals in the scoring code); unknown keys, negatives and missing weights fail at start-up. Ties break by cost, then id, deterministically. `nearest` ranks by the same planner's path length, with the same eligibility rules, so the comparison isolates the ranking; it reports the fire/confidence factors but weights none.
- **Eligibility is explicit.** Not selectable, each with its reason in the decision: `UNKNOWN` (tentative), `CARRIED`, `RESCUED`, `UNREACHABLE` (marked by the rescue side); **no path out or back** (`unreachable` = the planner said so: no path / goal occupied / outside the map); **planner could not be asked** (`unknown`: server missing, timeout, TF error, an exception) — *not* assumed reachable. Nothing eligible ⇒ an empty target and the reasons.
- **Explanation.** Each decision carries, for every candidate: raw factors (metres, scores), the weighted contributions (which sum exactly to the utility), the rationale string and any exclusion; plus the weights, the selected id, why it decided (world changed / robot moved / retry / refresh) and **the exact world it decided on** (so any decision can be replayed from its own report — a live test does). The node logs the comparison table each time.
- **Reassessment.** Automatic re-decision when the world model changes (a victim's id/status/position by ≥ 0.15 m, the fire, a new victim), when the robot has moved ≥ 0.5 m (the paths changed), when the last decision was **incomplete** (nothing selectable, or the planner could not be asked: retried every 3 s), and as a slow refresh (15 s) so paths follow the costmap; rate-limited to 1 Hz; also on demand via `SelectTarget`. `TARGETED` stays a candidate, so a re-decision can confirm or change the target (no hysteresis yet: TODO).
- **Found by the live test, fixed:** the first decision came while Nav2's costmap was still starting ("goal outside the map"), the world then stayed unchanged, and the node kept its empty answer. That is what retry/refresh are for (two node tests pin it). And a multithreaded executor cost **~0.9 cores** under the 250 Hz simulated clock; the planner client now has its own node and thread so the cognition node is single-threaded (**~0.3 cores each**; 0.01 without sim time).
- **Tests added:** unit — `test_cognition_lib` (47: factors, hand-worked utility, contributions sum to utility, near-safe vs distant-endangered, sign discipline, obstacle → lower accessibility, confidence, cost, no dependence on ids or order, order recomputed each time, every ineligible status, unreachable / no way back / planner unknown or raising, world change → decision change, tie-breaks, explanation JSON, the baseline on its own, factory, agreement with `VictimState.msg`), `test_cognition_config` (8: shipped config valid and equal to the code defaults, every parameter declared, decision logic imports no ROS, no Gazebo/hardware/camera/detector/scenario/perception/world-model/navigation imports, **no victim-id, coordinate or rescue-order literals**, no weight literals, subscribes to `WorldState` only, launch opt-in), `test_prioritization_scenario` (10: the deterministic evaluation below); node — `test_prioritizer_node` (19: a fake Nav2 `ComputePathToPose` server with the real adapter and node — published target and explanation, planner protocol, model by parameter, weights by parameter, reassessment, no decision spam, retry, refresh, `SelectTarget`, unreachable/unknown/missing planner, the adapter's classification, start-up errors); sim — `test_cognition` (7, the whole real stack).
- **Deterministic evaluation** (`tests/sim/experiments/prioritization_eval.py [--sweep]`; no simulator; the scenario's ground truth as the world — all victims known, `DETECTED`, confidence 1.0 — with A* over the true geometry as the path oracle, *evaluation tooling that the robot never uses*). It reports; it scores and ranks nothing:

| model | selects | victim (ground-truth id) | U | d_fire | risk | trip out | rescue cost | directness |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| weighted utility | **victim_2** | victim_2 | +4.374 | 1.08 m | 0.93 | 4.40 m | 8.80 m | 0.99 |
| | | victim_1 | +3.043 | 2.48 m | 0.23 | 0.97 m | 1.93 m | 0.98 |
| | | victim_3 | +2.220 | 2.53 m | 0.22 | 3.50 m | 7.00 m | 0.84 |
| nearest-first | **victim_1** | victim_1 / victim_3 / victim_2 | −0.97 / −3.50 / −4.40 | (same factor values as above) | | | | |

  They **differ**. Measured causes and sensitivities: take the fire away and both choose victim_1; rescue victim_2 and both then choose victim_1; rescue victim_1 instead and the weighted model chooses victim_2 while nearest chooses victim_3; the weighted choice flips to victim_1 only when `weights.risk` falls to **≈ 1.1** (default 3.0; hazard radius 0.25–2 m and decay 0.25–4 m never flip it). The defaults were fixed before this evaluation was run and not changed afterwards.
- **Live evaluation** (real world model + real Nav2 planner under AMCL; `test_cognition.py -rP`; the world model's own ids `V1..` matched to ground truth by position only). Two victims known: MVP → victim_2, nearest → victim_1. All three known, robot at the start: MVP **victim_2**, nearest **victim_1**, the same as offline. Live factors (MVP run): victim_2 U +4.00, d_fire 1.09, trip 4.40, cost 8.80, confidence 0.32; victim_1 U +3.05, trip 0.97, cost 1.92, confidence 1.00; victim_3 U +1.50, trip **4.64 m against ~2.9 m straight** (directness **0.63**: the partition's detour, measured by Nav2). After `TARGETED→CARRIED→RESCUED` on the selected victim both nodes re-decided by themselves (MVP and nearest both → victim_1) and listed the rescued one as excluded.
- **Verification summary:**

| Requirement | Result |
| --- | --- |
| Consumes the world model, produces a `RescueTarget` | live: target id is one the world model believes in (`V2`), frame `map`, breakdown, weights, rationale naming every candidate |
| Asks Nav2, does not plan | live paths ≥ straight line; two queries per candidate; unreachable/unknown/missing planner are excluded with reasons (fake-planner node tests) |
| Node = library | live: replaying the library on each decision's own reported world and planner answers reproduces the selection and every utility to 1e-6, for both models |
| Dynamic, not a fixed order | order recomputed each call; label/order permutations give the same choice; a rescue, a new victim, a moved fire or a moved robot each change it |
| Existing Phase 1–6 tests | **No regression attributable to this phase; two intermittent Phase 4 failures remain, so a fully green run was not obtained.** Complete run after a clean rebuild on a freshly rebooted, healthy host: **372 passed, 1 xfail, 2 failed in 30 min 32 s** (375 tests: 242 unit, 43 node, 90 sim). Every Phase 5–8 test passed. The two failures: `test_navigation_avoids_an_obstacle_it_must_go_round` (the Nav2 detour goal ends ~9 cm from the goal and aborts with "Failed to make progress" after ~28 s) and `test_occupied_cells_lie_on_real_surfaces` (65 % of occupied cells within 5 cm against a 70 % threshold). **Both were measured to be intermittent and independent of this change.** *Same saved map, same host, alternating runs of the whole Nav2 module:* the detour goal passed 2 of 3 and failed 1 of 3 on this tree, and passed 2 of 3 and failed 1 of 3 on the accepted Phase 6 commit. *Map precision over six mapping runs, two per tree:* Phase 5 93.2 / 80.1 %, Phase 6 75.0 / 77.4 %, Phase 7 91.3 / 78.5 % within 5 cm (Phase 4 recorded 76 and 92 %), so the 70 % threshold sits in the low tail of a wide run-to-run spread. Earlier runs in this phase failed more (the Phase 2 motion tests, topic and scan rates, fixture start-up) while the host was stalling (real-time factor swinging 0.3–3, `ros2` CLI calls > 15 s, an 8.5-minute clean build against 17 s after a reboot); those did not recur on the rebooted host. Harness changes made for this: `lifecycle_state()` returns `unknown` on a slow CLI call instead of aborting the wait; message-rate tests count per *simulation* second; `FIRE_RESQ_TEST_MAP` reuses a saved map for A/B experiments. No existing test was weakened or removed |

- **Decisions (flagged in the report):**
  1. **Numbering.** The plan puts cognition at Phase 8 *after* Nav2 planning (Phase 7); it was built first on request. Only the path-query adapter (`Nav2PathQuery`, the narrow part cognition needs) exists; Phase 7's reliability verification and helper are still open.
  2. **Two planner queries per candidate** (robot → approach point, approach point → safe zone) where §10 said one: the return leg is a real path, so a victim with no way home is excluded.
  3. **Accessibility is graded** (straight ÷ planned) instead of {0, 1}: with unreachable victims excluded, a binary term would be constant, and §13's own test needs "behind an obstacle scores lower".
  4. **`fire_risk` and the weights are stated stances**, not fitted: risk is 1 within a 1 m hazard radius then decays; tiers 3 : 1 : 0.5. The choice is sensitive to `weights.risk` (flip ≈ 1.1). **The maintainer should own these numbers.**
  5. **Paths are queried to an approach point 0.4 m short of the victim**, not the victim (Nav2 rejects goals inside its inflation; Phase 4). `RescueTarget.approach_pose` carries that point facing the victim as a *provisional* value; the executable approach pose is the FSM's (Phase 10).
  6. **The full comparison is an untyped JSON topic** (`decision_report`), since `ScoreBreakdown` describes only the selected victim; a typed message is a TODO if a consumer needs one (no interface was changed).
- **Limitations:** decisions are one-shot with no hysteresis (a near tie can flip the target between re-decisions — `TARGETED` stays a candidate); `fire_risk` is static; confidence is the world model's pixel-count proxy (a victim 5 m away reads ~0.3, so distance is penalised twice: by confidence and by reach/cost — measured, not tuned away); path lengths come from Nav2's costmap, so a victim not yet mapped as an obstacle or a stale costmap gives stale answers until the refresh; two nodes cost ~0.65 cores in simulation (rclpy handling the 250 Hz clock), one node ~0.3; reachability is only as good as the costmap (a planner that says "outside the map" while starting is now retried, but a wrong verdict after start-up is not detected).
- **TODOs (explicit in code):** Bayesian/uncertainty-aware reasoning; active perception; dynamic fire-risk modelling; battery/resource awareness; decision hysteresis; a typed decision message; learned policy; the executable approach pose (Phase 10).

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

**Status (Phase 3):** tiers 1 and 3 exist for the simulation layer. `tests/unit/` (scenario validation/generation, geometry, architecture guards; ~0.2 s) and `tests/sim/` (each module launches its own headless simulation, in its own process group, and judges the robot against Gazebo ground truth; ~5 min fast, ~8 min with the slow RGB-only and determinism tests). Commands are in CLAUDE.md. Two corrections to the paragraph above: scenario startup *is* deterministic (verified: identical world, poses and scripted-drive outcome across launches), so it is the perception/timing-dependent behaviour that varies; and the ground-truth guard is now enforced by `tests/unit/test_architecture_guards.py` rather than left as an intention. Tier 2 (node-level) arrived with Phase 5 as `tests/node/`: the real node run in-process under pytest against synthetic ROS traffic (no `launch_testing`; simpler, and it gates commits in ~25 s).

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
| Narrow-FOV scan-matching SLAM diverges (§8) | **High — realised in Phase 4, then mitigated** | Skip rotation-only scans, 0.3 m translation gate, 87° camera, no loop closure; AMCL on a saved map for navigation. Residual: noise-free odometry flattered the numbers; hardware will differ |
| Nav2 concurrent with scan-matching SLAM injects a heading jump | **Medium — measured, not fixed** | Navigate under AMCL on a saved map; xfail test records it; diagnostic script reproduces it |
| Encoder-only heading drift on hardware (no IMU) | Medium — unmeasured | Recorded as a Hardware.md TODO; IMU deliberately deferred and not required; measure on the real robot first |
| The robot is blind behind itself (forward-only depth wedge) | Medium | No reversing/BackUp anywhere; recovery from failed navigation is a later TODO that needs a sensing answer first |
| WSL2 rendering too slow for camera/depth sensors | Medium — would slow every later phase | Discovered in Phase 0 deliberately; fall back to reduced resolution/rate or headless |
| Spatial estimation error exceeds magnet alignment tolerance | Medium — measured in Phase 5 | Depth: ~1 cm median. RGB-only: ~1–4 cm to 4 m, ~11 cm beyond. Close objects (< ~0.5–1 m) get *no* position (clipped blob), so the last metre must use the world model's stored position; closed-loop visual align stays a TODO |
| Open-loop `ALIGN` unreliable in sim physics | Medium | Keep standoff generous; confirm attach via `MagnetState` rather than assuming success |
| Colour-blob detection is brittle on real cameras (lighting) | Low in sim, **high on hardware** | Accepted MVP tradeoff behind the `Detector` interface; `YoloDetector` is the hardware answer |
| `detachable-joint` re-attach semantics differ from expectation | Low | `attach_topic` support confirmed present in 8.15.0; Phase 9 tests it standalone before integration |
| RGB/depth content lag (~40 ms) corrupts fused positions from a moving camera | Medium — found in Phase 3 | Perception processes while stationary/slow or compensates; robust depth over the blob mask; on hardware prefer aligned depth. Tests use step-and-look |
| **The Phase 4 detour-goal flake** | ~~Medium~~ **RESOLVED (plan Phase 7)** | Root cause: RPP's `stateful` latch froze the robot in rotate-only mode outside the goal checker's tolerance. Fix `FollowPath.stateful: false`; 60 of 60 goals after, 12–44 % failing before; regression tests added (see Phase 7) |
| **`test_occupied_cells_lie_on_real_surfaces` (map precision)** | ~~Medium~~ **RESOLVED (plan Phase 7, maintainer-approved)** | The 5 cm criterion mostly measured a ~4 cm rigid map offset from Gazebo's frame (raw 68–93 %, aligned 87–98.5 %, 11 maps), not map shape. Fixed by scoring the best-fit-aligned distances against the SAME 70 %/95 % thresholds, plus a new ≤ 10 cm bound on the offset itself so a genuinely mis-registered map still fails. Cause of the offset still not isolated (harmless: AMCL localises against the same map) |
| **`test_scan_rate` (depth/scan render rate)** | Low — KNOWN FLAKINESS, environmental, threshold unchanged | Render rate varies 11–28 Hz per simulation second on both this tree and Phase 6 (24 windows, `sensor_rate_diag.py`) against a 12 Hz floor; not a navigation defect |
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

> **Status: Phases 0–4 complete.** The next task is Phase 5 (perception). The original Phase 0 reasoning is kept below for the record.

**Phase 0, and specifically: verify that Gazebo Harmonic renders camera and RGB-D sensors at usable frame rates under WSL2.**

Concretely — launch `gz sim` with a trivial world containing a camera sensor and an `rgbd_camera` sensor, bridge both image topics into ROS 2, and measure with `ros2 topic hz`. Then commit a `.gitignore` covering `build/ install/ log/` and record the measured versions and frame rates in `docs/Environment.md`.

This is first because it is the only unknown that can invalidate the plan's shape. Every phase from 3 onward depends on simulated camera and depth streams, and the entire SLAM strategy in §8 assumes depth images arrive fast enough to synthesise scans. WSL2 GPU passthrough is the one variable I could not confirm by inspecting installed packages — everything else in §2 I verified directly. If depth rendering turns out to be unusably slow, that is worth discovering in an afternoon against a throwaway world rather than in Phase 4 with six packages already built on the assumption.

It is also nearly free: no project code, no design commitment, and the `.gitignore` is needed regardless.

Once that passes, Phase 1 follows immediately — and Phase 1 wants the `fire_resq_description` question in §16 answered first, since it determines the package tree that gets created.
