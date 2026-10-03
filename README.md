# First Project - ROS2 Odometry for Bunker Pro

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow)](https://www.python.org/)

**First Project** for the **Robotics course** — an odometry estimation system for a skid-steering robot (Bunker Pro) using ROS2.  
The system computes wheel-based odometry, compares it against ground truth, and provides a reset service.

<img width="1375" height="738" alt="image" src="https://github.com/user-attachments/assets/f6e52fea-5dd9-40b4-a026-026bc06e0dc3" />

---

## 📦 Features

- **Odometer node**: subscribes to `/bunker_status`, converts RPM to linear velocity, integrates pose, and publishes:
  - `/project_odom` (custom odometry)
  - `/tf` from `odom` to `base_link2`
- **TF Error node**: compares ground truth TF vs. estimated odometry and publishes error metrics.
- **Reset service**: resets odometry state to zero via `/reset` (`std_srvs/Empty`).
- **RViz visualization**: ready-to-use config with TF display and trajectory tracking.

---

## 🧪 Parameters

| Parameter      | Value   | Description |
|----------------|---------|-------------|
| `k`            | 0.001170 | RPM → m/s conversion factor (calibrated from straight-line bag data) |
| `wheel_base`   | 0.720 m  | Apparent baseline for skid-steering (calibrated empirically; physical baseline is 0.635 m) |

---

## 🚀 Installation

### 1. Prerequisites

- ROS2 Humble (or later)
- `bunker_msgs` package (provides `/bunker_status` messages)

```bash
sudo apt install ros-humble-bunker-msgs   # or install from source in your workspace
```

### 2. Clone the repository

```bash
cd ~/colcon_ws/src
git clone https://github.com/<your-username>/first_project_ros2.git
```

### 3. Build the package

```bash
cd ~/colcon_ws
colcon build --packages-select first_project --symlink-install
source install/setup.bash
```

---

## ▶️ Usage

### Launch the full system

```bash
ros2 launch first_project first_project.launch.py
```

This launches:
- `odometer` node
- `tf_error` node
- `rviz2` with the preconfigured view

### Play a bag file (in a separate terminal)

```bash
ros2 bag play ~/bags/bags_fixed/<bag_name> --clock
```

> ⚠️ The `--clock` flag is **mandatory** because all nodes use `use_sim_time := True`.

### Reset odometry

```bash
ros2 service call /reset std_srvs/srv/Empty {}
```

---

## 📊 Available Bag Files

Place your bag files in `~/bags/bags_fixed/`.  
Example bags:

- `rosbag2_2026_04_08-16_38_55_fixed`
- `rosbag2_2026_04_08-16_41_35_fixed`
- `rosbag2_2026_04_08-16_44_32_fixed`
- `rosbag2_2026_04_08-16_47_51_fixed`
- `rosbag2_2026_04_08-16_51_38_fixed`
- `rosbag2_2026_04_08-17_03_17_fixed`

---

## 🧠 Nodes Overview

### `odometer`

- **Subscribes to:**
  - `/bunker_status` (`bunker_msgs/BunkerStatus`) — wheel RPM data
  - `/odom` (`nav_msgs/Odometry`) — initial pose (used once for alignment)
- **Publishes:**
  - `/project_odom` (`nav_msgs/Odometry`)
  - `/tf` — `odom` → `base_link2`
- **Service:**
  - `/reset` (`std_srvs/Empty`) — resets odometry to zero

### `tf_error`

- **Subscribes to:**
  - `/project_odom` (`nav_msgs/Odometry`)
  - `/tf` — ground truth (`odom` → `base_link`) and estimated (`odom` → `base_link2`)
- **Publishes:**
  - `/tf_error_msg` (`first_project/TfErrorMsg`) containing:
    - `header`
    - `tf_error` (Euclidean distance [m])
    - `time_from_start` [s]
    - `travelled_distance` [m]

---

## 🖥️ RViz Configuration

RViz is automatically launched with:

- **Fixed Frame:** `odom`
- **Displays:**
  - TF: `base_link` (ground truth) and `base_link2` (our odometry)
  - Top-down orthographic view
  - Trajectory history: 100000 poses

Config file: `first_project/rviz/first_project.rviz`

---

## 📂 Custom Message

`first_project/msg/TfErrorMsg.msg`:

```
std_msgs/Header header
float64 tf_error
float64 time_from_start
float64 travelled_distance
```

---

## 🧪 Calibration Notes

- `k` was estimated from straight-line sections of the bag data.
- `wheel_base` was empirically tuned to minimize odometry error vs. ground truth.  
  The physical baseline is **0.635 m**, but skid-steering slip increases the effective baseline to **0.720 m**.

---

## 📌 Important Notes

- Always **source** your workspace after building:

  ```bash
  source ~/colcon_ws/install/setup.bash
  ```

- All nodes use **simulated time**. Ensure your bag playback includes `--clock`.

- The launch file uses `get_package_share_directory()` to locate the RViz config — no absolute paths.

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

---

## ✍️ Author

- Francesco Monticone — [@francescomonticone](https://github.com/francescomonticone)  
  Robotics Course — Politecnico di Milano
