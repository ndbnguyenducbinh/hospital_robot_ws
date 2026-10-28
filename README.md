# Hospital Reception & Guidance Robot — ROS 2 Navigation System

Hệ thống điều hướng tự động và kiểm soát chuyển động dành cho **Robot Tiếp đón & Dẫn đường Bệnh viện** chạy trên nền tảng **NVIDIA Jetson Orin** với **ROS 2 Humble**.

---

## 1. Tổng quan hệ thống (Phase 1 Prototype)

Trong giai đoạn **Phase 1 Prototype (Sprint 1 tháng)**:
- **Tách biệt tương tác người-máy (Decoupled HRI):** Mọi tác vụ giao tiếp giọng nói (Voice AI / LLM) và giao diện cảm ứng cho bệnh nhân được xử lý hoàn toàn trên **Ứng dụng Tablet gắn ngoài**.
- **Nhiệm vụ của Jetson Orin:** Nhận điểm đến ngữ nghĩa (`destination_id`) từ Tablet qua HTTP REST API (port `8000`), tra cứu tọa độ bản đồ, điều phối điều hướng tự động qua `Nav2`, và giám sát an toàn động cơ base vi sai (MCU Dual Hub Motors).
- **Nguyên tắc Canonical Topic Mapping:** Tuyệt đối không hardcode tên topic ROS 2 trong source code. Tất cả các node bắt buộc phải phân giải topic động qua `config/topic_map.yaml`.

---

## 2. Cấu trúc Workspace

```text
hospital_robot_ws/
├── README.md                          # Tài liệu hướng dẫn cài đặt & vận hành
├── node_structure.md                  # Hướng dẫn chi tiết kiến trúc Node & Canonical Topic Map
├── requirement.md                     # Đặc tả kỹ thuật hệ thống
└── src/
    ├── robot_bringup/                 # Master launch, canonical topic map & cấu hình tọa độ
    │   ├── config/
    │   │   ├── topic_map.yaml         # Single Source of Truth cho toàn bộ topic ROS 2
    │   │   └── hospital_locations.yaml# Bảng tra cứu Semantic ID -> tọa độ (x, y, yaw)
    │   ├── launch/
    │   │   └── prototype_bringup.launch.py
    │   └── robot_bringup/
    │       └── topic_mapper.py        # Thư viện helper TopicMapper
    │
    ├── 1_robot_interface/             # Gateway HTTP REST API kết nối với Tablet
    │   ├── robot_interface/
    │   │   └── tablet_bridge_node.py  # Server FastAPI (port 8000) -> ROS 2 Publisher
    │   └── tablet_bridge_node.py      # Executable wrapper
    │
    ├── 2_robot_sensors/               # Driver cảm biến (2D LiDAR, Depth Camera, IMU)
    │
    ├── 3_robot_perception/            # Xử lý & lọc dữ liệu cảm biến (LaserScan, PointCloud)
    │
    ├── 4_robot_navigation/            # Bộ điều phối nhiệm vụ Nav2 & AMCL
    │   ├── robot_navigation/
    │   │   └── task_manager_node.py   # Chuyển đổi Semantic ID -> Nav2 NavigateToPose Action
    │   └── task_manager_node.py       # Executable wrapper
    │
    └── 5_robot_control/               # Giám sát an toàn & MCU Telemetry
        ├── robot_control/
        │   └── safety_supervisor_node.py # Giám sát E-Stop, lỗi động cơ MCU & Diagnostics
        └── safety_supervisor_node.py  # Executable wrapper
```

> **Lưu ý về tên package:** Các thư mục con trong `src/` được đánh số (`1_...`, `4_...`, `5_...`) theo yêu cầu tổ chức subsystem, nhưng tên package đăng ký trong `package.xml` là tên hợp lệ theo chuẩn ROS 2 (`robot_interface`, `robot_navigation`, `robot_control`) để đảm bảo không bị cảnh báo và hỗ trợ import Python hoàn hảo.

---

## 3. Yêu cầu môi trường (Prerequisites)

* **Hệ điều hành:** Ubuntu 22.04 LTS (JetPack 6.x trên Jetson Orin hoặc máy x86 tương đương)
* **ROS 2 Distribution:** ROS 2 Humble Hawksbill
* **Cài đặt ROS 2 packages:**
  ```bash
  sudo apt update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-twist-mux \
    python3-yaml
  ```
* **Cài đặt Python dependencies cho Tablet Bridge:**
  ```bash
  python3 -m pip install --user fastapi uvicorn "websockets>=12.0" pydantic
  ```

---

## 4. Hướng dẫn Build Workspace

Mở terminal tại thư mục gốc của workspace:

```bash
cd ~/hospital_robot_ws

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Build tất cả các packages
colcon build --symlink-install

# 3. Source overlay workspace
source install/setup.bash
```

---

## 5. Hướng dẫn Chạy (Run)

### 5.1. Khởi động toàn bộ Prototype Stack
Chạy lệnh master launch để khởi động đồng thời `tablet_bridge_node`, `task_manager_node` và `safety_supervisor_node`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_bringup prototype_bringup.launch.py
```

Khi chạy thành công, log sẽ hiển thị:
- `tablet_bridge_node` mở HTTP Server tại `http://0.0.0.0:8000`
- `task_manager_node` tải danh sách tọa độ các phòng từ `hospital_locations.yaml`
- `safety_supervisor_node` bắt đầu broadcast `/diagnostics` định kỳ 1Hz.

---

## 6. Hướng dẫn Kiểm thử (Mocking Tablet Commands)

Mở một terminal mới để gửi lệnh giả lập từ Tablet (hoặc qua `curl` / Postman):

### 6.1. Kiểm tra trạng thái REST API
```bash
curl -X GET http://localhost:8000/api/v1/health
```
**Phản hồi:**
```json
{
  "status": "OK",
  "service": "tablet_bridge_node",
  "goal_topic": "/tablet/command/goal_id",
  "cancel_topic": "/tablet/command/cancel"
}
```

### 6.2. Gửi lệnh điều hướng đến phòng bệnh (Navigate Goal)
```bash
curl -X POST http://localhost:8000/api/v1/navigate \
     -H "Content-Type: application/json" \
     -d '{"destination_id": "BLOOD_LAB_01"}'
```
**Phản hồi (`200 OK`):**
```json
{
  "status": "ACCEPTED",
  "destination_id": "BLOOD_LAB_01",
  "message": "Navigation goal dispatched to Nav2."
}
```

### 6.3. Hủy nhiệm vụ hiện tại / Dừng robot (Cancel Goal)
```bash
curl -X POST http://localhost:8000/api/v1/cancel
```
**Phản hồi (`200 OK`):**
```json
{
  "status": "CANCELLED",
  "message": "Robot stopping."
}
```

### 6.4. Kiểm tra các topic ROS 2 đang hoạt động
```bash
# Kiểm tra danh sách topic
ros2 topic list

# Theo dõi điểm đến hiện tại
ros2 topic echo /task_manager/current_goal

# Theo dõi trạng thái máy trạng thái dẫn đường (IDLE, NAVIGATING, CANCELLED,...)
ros2 topic echo /task_manager/state

# Theo dõi dữ liệu chẩn đoán an toàn
ros2 topic echo /diagnostics
```

### 6.5. Kiểm thử tính năng Dừng khẩn cấp (Emergency Stop)
Thử kích hoạt E-Stop từ console ROS 2:
```bash
# Bật E-Stop
ros2 topic pub --once /safety/e_stop std_msgs/msg/Bool "{data: true}"
```
Quan sát terminal chạy launch: `safety_supervisor_node` sẽ phát hiện ngay lập tức, ghi log `EMERGENCY STOP TRIGGERED!` và gửi lệnh vận tốc 0 đến `/diff_drive_controller/cmd_vel_unstamped`.

---

## 7. Tài liệu chi tiết khác

- Chi tiết về cách thức kết nối các node và nguyên tắc hoạt động của **Canonical Topic Map**: xem [node_structure.md](file:///home/binhnguyenduc/Documents/hospital_robot_ws/node_structure.md).
- Đặc tả yêu cầu 4 tuần sprint và phần cứng: xem [requirement.md](file:///home/binhnguyenduc/Documents/hospital_robot_ws/requirement.md).

