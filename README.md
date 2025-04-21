# 🤖 Indoor Adaptive Patrol Robot

A ROS2-based autonomous robot designed to patrol indoor environments and adapt its behavior based on detected human activity.

This project is being developed as a **long-term skill-building portfolio project** by a robotics engineer with expertise in control systems, AI, and systems design.

---

## 📌 Project Goals

- ✅ **Autonomous indoor patrol** with dynamic route adaptation
- ✅ **Human activity detection** (vision-based)
- ✅ **Sensor fusion** with LiDAR, Camera, IMU
- ✅ **Modular architecture** with clean ROS2-native practices
- ✅ **Real-time visualization** in RViz2
- ✅ **Simulation** in Gazebo

---

## 📅 Status: End of Day 1

| Feature                             | Status |
|------------------------------------|--------|
| ROS2 Workspace                     | ✅ Done |
| ROS2 Package `patrol_robot`       | ✅ Done |
| Dummy Nodes (3 total)             | ✅ `patrol_node`, `detection_node`, `visualization_node` |
| Multi-node Launch System          | ✅ Done |
| GitHub Setup + Version Control    | ✅ Done |

---

## 🚀 Getting Started

### 🔧 Build & Launch

```bash
cd ~/indoor_patrol_ws
colcon build
source install/setup.bash
ros2 launch patrol_robot main.launch.py
```  

### 🧱 Workspace Structure

```plaintext
indoor_patrol_ws/
├── build/                      # Build outputs (ignored in git)
├── install/                    # Installed ROS2 setup (ignored in git)
├── log/                        # Colcon logs (ignored in git)
├── src/
│   └── patrol_robot/
│       ├── patrol_robot/       # Python ROS2 nodes
│       │   ├── patrol_node.py
│       │   ├── detection_node.py
│       │   └── visualization_node.py
│       ├── launch/             # Launch files (e.g. main.launch.py)
│       ├── config/             # RViz configs, YAML parameters
│       ├── msg/                # Custom messages (if any)
│       ├── srv/                # Custom services (if any)
│       ├── action/             # Custom actions (if any)
│       ├── package.xml         # ROS2 package manifest
│       ├── setup.py            # Python entry point config
│       └── README.md           # Package-level docs
```

## 📈 Roadmap

This roadmap outlines the planned weekly development and integration milestones:

| Week | Focus Area                 | Key Deliverables                                   |
|------|----------------------------|----------------------------------------------------|
| 1    | ROS2 Core Setup            | Workspace, package structure, nodes, launch system |
| 2    | URDF + Gazebo Modeling     | Simulated robot model, base sensors (LiDAR/IMU)    |
| 3    | Navigation Stack (Nav2)    | Mapping, localization, static patrol path          |
| 4    | FSM / Behavior Tree Logic  | Basic patrol loop, zone switching                  |
| 5    | Sensor Data Integration    | Real LiDAR & camera data in RViz2                  |
| 6    | AI-based Human Detection   | YOLO/OpenCV-based detection, publish events        |
| 7    | Activity Heatmaps          | Zone-level activity scoring, visualization         |
| 8    | Adaptive Patrol Logic      | Route updates based on events/human presence       |
| 9    | Visual Debugging Layer     | RViz markers, zone overlays, diagnostics           |
| 10   | Demo Packaging             | GitHub final polish, README, launch demos          |

✅ Tasks marked as **Done** will be checked off in future versions of this roadmap.

## 🧠 Engineering Principles

This project follows strict software and systems engineering principles to ensure quality, scalability, and maintainability:

- **ROS2-Native Design**
  - Use of lifecycle nodes (when possible), parameters, modular launch files
  - Custom interfaces (`.msg`, `.srv`, `.action`) for clean inter-node communication

- **Modular Architecture**
  - Clearly separated nodes for perception, control, visualization, etc.
  - Launch files and config files organized by subsystem

- **Visualization & Observability**
  - Every behavior must have corresponding RViz2 output (markers, paths, heatmaps)
  - Clear ROS2 logging at all critical points (`self.get_logger().info/debug/warn`)

- **Parameter-Driven Development**
  - No hardcoded logic — every key variable is exposed via `params.yaml`

- **Testable & Extensible**
  - Each node is self-contained and testable independently
  - Easily swappable AI models, planners, or perception backends

## 📘 Daily Logs

This project is developed using a daily engineering logbook approach.

Each day has its own markdown file under the [`daily_logs/`](daily_logs/) directory, documenting:

- ✅ Objectives and what was planned
- 📦 What was implemented or configured
- 🐞 Debugging steps and lessons learned
- 💡 Design decisions and architectural notes
- 📊 Progress made vs. roadmap

### Example

```bash
daily_logs/
├── day1.md   # ROS2 setup, dummy nodes, launch config
├── day2.md   # URDF + Gazebo robot modeling
├── ...
```
