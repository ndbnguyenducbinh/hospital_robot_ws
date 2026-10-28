
# Hospital Reception & Guidance Robot — ROS 2 Navigation System

Autonomous navigation and motion control stack for the **Hospital Reception Robot** running on **NVIDIA Jetson Orin** with **ROS 2**[cite: 1]. 

In this **Phase 1 Prototype (1-Month Sprint)**, Human-Robot Interaction (HRI), voice/LLM processing, and user interface tasks are fully offloaded to an external **Tablet Application**. The ROS 2 stack on the Jetson Orin is strictly responsible for receiving semantic destination commands from the Tablet, executing autonomous navigation via `Nav2`[cite: 1], and controlling the differential drive base (MCU + Dual Hub Motors)[cite: 1].

---

## 1. System Architecture & Scope (Phase 1 Prototype)

```text
[Tablet App (UI / Voice AI / Cloud)]
         │
         │ HTTP POST / WebSocket (JSON: {"destination_id": "BLOOD_LAB_01"})
         ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ NVIDIA JETSON ORIN (ROS 2 Workspace)                                    │
│                                                                         │
│  ┌──────────────────────┐   /tablet/command/goal_id   ┌──────────────┐  │
│  │ 1_robot_interface    │ ──────────────────────────► │ 4_navigation │  │
│  │ (tablet_bridge_node) │                             │(task_manager)│  │
│  └──────────────────────┘                             └──────┬───────┘  │
│                                                              │          │
│  ┌──────────────────────┐   /scan, /depth/points      ┌──────▼───────┐  │
│  │ 2_robot_sensors &    │ ──────────────────────────► │  Nav2 Stack  │  │
│  │ 3_robot_perception   │                             │ (AMCL + Plan)│  │
│  └──────────────────────┘                             └──────┬───────┘  │
│                                                              │ /cmd_vel │
│  ┌───────────────────────────────────────────────────────────▼───────┐  │
│  │ 5_robot_control (twist_mux + safety_supervisor + ros2_control)    │  │
│  └───────────────────────────────────────────────────────────┬───────┘  │
└──────────────────────────────────────────────────────────────┼──────────┘
                                                               │ Serial/CAN
                                                               ▼
                                          ┌────────────────────────────────┐
                                          │ MCU / Motion Controller        │
                                          │  ├── Left Driver & Hub Motor   │
                                          │  └── Right Driver & Hub Motor  │
                                          └────────────────────────────────┘

```

### Key Design Decisions for Phase 1

* **Decoupled HRI:** The Jetson Orin does **not** process microphone audio, TTS, or touch UI during Phase 1. The Tablet handles all patient interactions.
* **One-Way Command Priority:** Communication prioritizes Tablet $\rightarrow$ Jetson goal dispatch. Video streaming, live map rendering on the tablet, and complex bi-directional telemetry are deferred to Phase 2.
* **Canonical Topic Mapping:** **Hardcoding ROS 2 topic names inside node source code is strictly prohibited.** All nodes must resolve topic names dynamically through `config/topic_map.yaml`.

---

## 2. Workspace Directory Structure

```text
hospital_robot_ws/
└── src/
    ├── robot_bringup/                # Global launch files & master configurations
    │   ├── config/
    │   │   ├── topic_map.yaml        # Canonical ROS 2 topic mapping (Single Source of Truth)
    │   │   └── hospital_locations.yaml # Semantic ID to (x, y, yaw) coordinate lookup
    │   ├── launch/
    │   │   └── prototype_bringup.launch.py
    │   └── robot_bringup/
    │       └── topic_mapper.py       # Shared Python helper to resolve topic keys
    │
    ├── 1_robot_interface/            # Gateway between Tablet App and ROS 2
    │   └── tablet_bridge_node.py     # REST API / WebSocket server -> ROS 2 publisher
    │
    ├── 2_robot_sensors/              # Hardware drivers (2D LiDAR, Depth Camera, IMU)
    │
    ├── 3_robot_perception/           # LaserScan filtering & Depth pointcloud processing
    │
    ├── 4_robot_navigation/           # Nav2 configs, AMCL, maps, and Task Manager
    │   └── task_manager_node.py      # Translates Semantic ID -> Nav2 NavigateToPose Action
    │
    └── 5_robot_control/              # URDF, ros2_control, MCU serial bridge, Safety Supervisor
        └── safety_supervisor_node.py # Monitors hardware faults & E-Stop triggers

```

---

## 3. Canonical Topic Map (`config/topic_map.yaml`)

All developers must adhere to the subsystem hierarchy defined below. If a feature is out of scope for the 1-month prototype, its value remains `null`.

```yaml
# Hospital Reception Robot - Canonical Topic Map
#
# Hierarchy:
#   /robot_{id}/interface/  — Tablet App, Cloud & HRI communication
#   /robot_{id}/sensors/    — Raw hardware data (LiDAR, Depth Cam, Encoders)
#   /robot_{id}/perception/ — Obstacle filtering & future human tracking
#   /robot_{id}/navigation/ — Localization, Mapping, Nav2 Planning & Mission State
#   /robot_{id}/control/    — Safety supervisor, ros2_control & MCU telemetry

# ── 1. Interface (/robot_{id}/interface/) ───────────────────────────────────
interface:
  tablet:
    goal_id: "/tablet/command/goal_id"       # [ACTIVE] std_msgs/String (e.g., "BLOOD_LAB_01")
    cancel_task: "/tablet/command/cancel"    # [ACTIVE] std_msgs/Empty
    robot_status: null                       # [PHASE 2] Feedback state to Tablet
    battery_level: null                      # [PHASE 2] Battery percentage
  hri_audio:
    tts_prompt: null                         # [PHASE 2] Handled by Tablet in Phase 1
    mic_raw: null                            # [PHASE 2] Handled by Tablet in Phase 1
  cloud_bridge:
    task_request: null                       # [PHASE 2] Direct Hospital Cloud sync

# ── 2. Sensors (/robot_{id}/sensors/) ───────────────────────────────────────
sensors:
  lidar_2d:
    scan_raw: "/scan"                        # [ACTIVE] sensor_msgs/LaserScan
  camera_depth:
    color_raw: null                          # [PHASE 2] RGB streaming disabled in Phase 1
    depth_raw: "/camera/depth/image_rect_raw" # [ACTIVE] sensor_msgs/Image
    camera_info: "/camera/depth/camera_info" # [ACTIVE] sensor_msgs/CameraInfo
    pointcloud: "/camera/depth/color/points" # [ACTIVE] sensor_msgs/PointCloud2 (3D obstacle avoidance)
  imu:
    data_raw: "/imu/data_raw"                # [OPTIONAL] sensor_msgs/Imu
  wheel_odom:
    joint_states: "/joint_states"            # [ACTIVE] sensor_msgs/JointState (from MCU encoders)

# ── 3. Perception (/robot_{id}/perception/) ─────────────────────────────────
perception:
  obstacle_processing:
    filtered_scan: "/scan_filtered"          # [ACTIVE] sensor_msgs/LaserScan (robot footprint cropped)
    depth_obstacles: "/perception/depth_obstacles" # [OPTIONAL] Processed 3D obstacles
  human_tracking:
    patient_detections: null                 # [PHASE 2] Follow-me / patient tracking
    person_bbox_debug: null                  # [PHASE 2] Vision debug

# ── 4. Navigation (/robot_{id}/navigation/) ─────────────────────────────────
navigation:
  localization:
    odom_wheel: "/diff_drive_controller/odom" # [ACTIVE] nav_msgs/Odometry
    odom_filtered: "/odometry/filtered"      # [OPTIONAL] EKF fused odometry
    amcl_pose: "/amcl_pose"                  # [ACTIVE] geometry_msgs/PoseWithCovarianceStamped
    initial_pose: "/initialpose"             # [ACTIVE] geometry_msgs/PoseWithCovarianceStamped
  mapping:
    static_map: "/map"                       # [ACTIVE] nav_msgs/OccupancyGrid
    local_costmap: "/local_costmap/costmap"  # [ACTIVE] nav_msgs/OccupancyGrid
    global_costmap: "/global_costmap/costmap" # [ACTIVE] nav_msgs/OccupancyGrid
  planning:
    global_plan: "/plan"                     # [ACTIVE] nav_msgs/Path
    local_plan: "/local_plan"                # [ACTIVE] nav_msgs/Path
    goal_pose: "/goal_pose"                  # [ACTIVE] geometry_msgs/PoseStamped
  mission:
    active_goal_id: "/task_manager/current_goal" # [ACTIVE] std_msgs/String
    navigation_state: "/task_manager/state"  # [ACTIVE] Internal state (IDLE, NAVIGATING, ERROR)

# ── 5. Control (/robot_{id}/control/) ───────────────────────────────────────
control:
  command:
    cmd_vel_nav: "/cmd_vel_nav"              # [ACTIVE] geometry_msgs/Twist (from Nav2)
    cmd_vel_teleop: "/cmd_vel_teleop"        # [ACTIVE] geometry_msgs/Twist (manual override)
    cmd_vel_out: "/diff_drive_controller/cmd_vel_unstamped" # [ACTIVE] Final Twist to MCU
  safety:
    emergency_stop: "/safety/e_stop"         # [ACTIVE] std_msgs/Bool
    collision_warning: null                  # [PHASE 2] Proximity slow-down trigger
  mcu_telemetry:
    motor_status: "/mcu/motor_status"        # [ACTIVE] Driver/Hub motor fault codes & encoder health
    diagnostics: "/diagnostics"              # [ACTIVE] diagnostic_msgs/DiagnosticArray

```

---

## 4. Coding Guidelines: Using `TopicMapper`

**Do not hardcode topic strings in your nodes.** Import the `TopicMapper` utility to load topics dynamically from `topic_map.yaml`:

```python
import yaml
from rclpy.node import Node
from std_msgs.msg import String

class TopicMapper:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            self._map = yaml.safe_load(f)

    def resolve(self, key_path: str) -> str | None:
        keys = key_path.split('.')
        val = self._map
        for k in keys:
            if isinstance(val, dict) and k in val:
                val = val[k]
            else:
                raise KeyError(f"Topic key '{key_path}' not found in topic_map.yaml")
        return val

# Example Usage inside a ROS 2 Node:
class TaskManagerNode(Node):
    def __init__(self, topic_map_path: str):
        super().__init__('task_manager_node')
        self.mapper = TopicMapper(topic_map_path)

        goal_topic = self.mapper.resolve("interface.tablet.goal_id")
        if goal_topic:
            self.sub_goal = self.create_subscription(
                String, goal_topic, self.goal_callback, 10
            )

```

---

## 5. Tablet $\rightarrow$ Jetson Interface Specification

The `tablet_bridge_node` exposes a lightweight HTTP REST API (port `8000`) on the local network connecting the Tablet and Jetson Orin.

### 5.1. Send Navigation Goal

* **Endpoint:** `POST http://<JETSON_IP>:8000/api/v1/navigate`
* **Payload:**
```json
{
  "destination_id": "BLOOD_LAB_01"
}

```


* **Response (`200 OK`):**
```json
{
  "status": "ACCEPTED",
  "destination_id": "BLOOD_LAB_01",
  "message": "Navigation goal dispatched to Nav2."
}

```



### 5.2. Cancel Current Goal / Stop Robot

* **Endpoint:** `POST http://<JETSON_IP>:8000/api/v1/cancel`
* **Response (`200 OK`):**
```json
{
  "status": "CANCELLED",
  "message": "Robot stopping."
}

```



### 5.3. Semantic Location Lookup (`config/hospital_locations.yaml`)

The `task_manager_node` maps the `destination_id` string received from the Tablet to map coordinates `(x, y, yaw)` in the `map` frame:

```yaml
locations:
  RECEPTION_DESK:
    frame_id: "map"
    x: 0.00
    y: 0.00
    yaw: 0.00
  BLOOD_LAB_01:
    frame_id: "map"
    x: 12.45
    y: -3.20
    yaw: 1.57

```

---

## 6. Getting Started & Build Instructions

### Prerequisites

* **OS:** Ubuntu 22.04 LTS (JetPack 6.x on Jetson Orin)


* **ROS 2 Distribution:** ROS 2 Humble Hawksbill
* **Dependencies:**
```bash
sudo apt update && sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-twist-mux \
  python3-fastapi \
  python3-uvicorn \
  python3-yaml

```
---

## 7. 4-Week Implementation Roadmap

* **Week 1 — Hardware Bringup & Interface Freeze:**
* Finalize REST API specification with the Tablet App team.
* Verify `ros2_control` differential drive interface with MCU, Hub Motor drivers, and wheel encoders. Validate `/diff_drive_controller/odom` accuracy via teleop.




* **Week 2 — URDF, Sensor Drivers & SLAM:**
* Configure robot URDF (`base_link` $\rightarrow$ `laser_frame`, `camera_link`).
* Integrate 2D LiDAR (`/scan_filtered`) and Depth Camera (`/camera/depth/color/points`).


* Build the static hospital floor map using `slam_toolbox` and record coordinates in `hospital_locations.yaml`.


* **Week 3 — Nav2 Tuning & Core Nodes:**
* Implement `tablet_bridge_node` and `task_manager_node` using `topic_map.yaml`.
* Configure `Nav2` local/global costmaps (Obstacle Layer + Voxel Layer for 3D overhanging obstacles like hospital beds/tables) and limit max linear velocity to `0.4 – 0.6 m/s` for patient safety.


* **Week 4 — System Integration & Fault Handling:**
* Implement basic hardware fault monitoring in `safety_supervisor_node` (stuck detection, motor error logging).


* End-to-end testing with the physical Tablet mounted on the robot chassis.





```

```