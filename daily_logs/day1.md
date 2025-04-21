# 🗓️ Day 1 Log – April 21, 2025

## 🎯 Goals
- Set up full ROS2 workspace from scratch
- Create package `patrol_robot`
- Implement 3 dummy ROS2 nodes:
  - `patrol_node`
  - `detection_node`
  - `visualization_node`
- Create a working multi-node launch file
- Push workspace to GitHub with proper structure
- Create initial `README.md` and `.gitignore`

---

## ✅ What I Did

- Installed ROS2 Humble and set up environment
- Created `indoor_patrol_ws` and `patrol_robot` package
- Wrote minimal Python ROS2 nodes (1 per behavior)
- Added `setup.py` with `console_scripts`
- Verified each node with `ros2 run`
- Built and tested a multi-node `main.launch.py`
- Registered launch files in `setup.py` for install
- Created GitHub repo and pushed clean initial commit
- Built a professional-grade `README.md`

---

## 🐞 Issues Faced

| Issue | Resolution |
|-------|------------|
| `rclpy` module not found | Sourced ROS2 correctly and installed missing packages |
| `LifecycleNode` import error | Replaced with standard `Node` (Lifecycle only in C++) |
| `ros2 run` couldn't find package | Fixed incorrect folder structure and entry points |
| Launch file not found | Added it to `data_files` in `setup.py` |
| GitHub rejected password push | Used Personal Access Token (PAT) instead |

---

## 💡 Notes & Decisions

- Python LifecycleNodes aren't supported → will simulate lifecycle behavior manually
- Keeping logs in `daily_logs/` as a clean audit trail of progress
- `.gitignore` now excludes build/install/log and dev clutter
- All nodes must always log meaningful info and be visualized in RViz later

---

## 🔜 Next Up (Day 2 Preview)

- Design URDF model for robot base
- Add LiDAR + IMU plugins
- Spawn robot into Gazebo
- Begin RViz2 + Gazebo debug cycle
