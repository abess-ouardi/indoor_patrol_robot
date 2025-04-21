# Indoor Adaptive Patrol Robot

ROS2-based simulation of an indoor robot that adapts its patrol based on human activity.

## Current Status (Day 1)

✅ ROS2 Workspace  
✅ Package `patrol_robot`  
✅ Dummy nodes: `patrol_node`, `detection_node`, `visualization_node`  
✅ Multi-node Launch File  

## To Run

```bash
cd ~/indoor_patrol_ws
colcon build
source install/setup.bash
ros2 launch patrol_robot main.launch.py
