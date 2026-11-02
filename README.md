## 1. Mapping waypoint

```bash
# T1: sim + RViz (default use_rviz:=true)
ros2 launch robot_navigation mapping.launch.py

# T2: manual steering wheel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=robot -p use_sim_time:=true -r cmd_vel:=/diff_drive_controller/cmd_vel

# T3: recorder manual (q = record point, e = done)
# Recorded points will immediately appear as markers in RViz
ros2 run robot_navigation waypoint_mapping
```

## 2. Navigation

```bash
# Running the entire system (Gazebo simulation + waypoint navigation + vision + RViz)
ros2 launch robot_navigation navigation.launch.py

ros2 launch robot_navigation navigation.launch.py use_rviz:=false
```
