# SLAM Mapping with Four-Wheel Robot (ROS 2)

This repository contains a 2D SLAM-ready simulation environment using a custom four-wheel mobile robot in Gazebo, integrated with SLAM Toolbox and RViz.

---

## SLAM Mapping (2D)

This package supports **2D SLAM mapping** using [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox).

---

## How to Run SLAM

### 1. Launch the Gazebo Simulation

```bash
ros2 launch four_wheel_2d gazebo_sim_turtle_world.launch.py
```

This will start Gazebo with a custom world and spawn your four-wheel robot. RViz will also open if configured.

### 2. Start SLAM Toolbox (Online Async Mode)

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

This node starts SLAM Toolbox in asynchronous online mode, subscribing to `/scan` and `/tf` to build a 2D map in real-time.

> If your LiDAR publishes to a different topic (e.g. `/lidar/scan`), you may need to remap the topic.

### 3. Teleoperate the Robot

Use the keyboard teleop package or your own teleoperation node to move the robot and explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

### 4. Save the Map

Once you’ve completed mapping:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/four_wheel_2d/maps/my_map
```

This will save `my_map.pgm` and `my_map.yaml` inside the `maps/` directory.

> You can visualize and edit the resulting map files using RViz or image editors.

---

More documentation coming soon!
