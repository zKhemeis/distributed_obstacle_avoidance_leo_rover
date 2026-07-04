# Leo PyBullet Simulation

A ROS 2 Humble simulation of the Leo Rover using the Bullet Physics Engine.

The simulator provides:

- Physics-based robot simulation
- Differential drive controller
- 2D LiDAR simulation
- Odometry publishing
- TF publishing
- RViz visualization
- Configurable environments using YAML world files

---

# Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Bullet Physics
- colcon
- RViz2

---

# Build

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select leo_pybullet

source install/setup.bash
```

---

# Run

Launch the simulator with the default world:

```bash
ros2 launch leo_pybullet pybullet_sim.launch.py
```

Launch a predefined world:

```bash
ros2 launch leo_pybullet pybullet_sim.launch.py world:=boxes
```

Launch an empty world:

```bash
ros2 launch leo_pybullet pybullet_sim.launch.py world:=empty
```

Launch a custom YAML world:

```bash
ros2 launch leo_pybullet pybullet_sim.launch.py \
world:=/root/leo_ws/src/leo_pybullet/worlds/my_world.yaml
```

---

# Robot Control

Drive forward:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Drive backward:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -0.3}, angular: {z: 0.0}}"
```

Turn left:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Turn right:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: -0.5}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

# RViz

Start RViz:

```bash
rviz2
```

Add the following displays:

- MarkerArray
- LaserScan
- TF
- Odometry

Topics:

| Display | Topic |
|----------|-------|
| Robot | /leo_markers |
| LiDAR | /scan |
| TF | /tf |
| Odometry | /odom |

---

# LiDAR Test

Run the scan debug node:

```bash
ros2 run leo_pybullet scan_debug_node
```

Example output:

```
Closest obstacle: 1.65 m
```

---

# Odometry Test

Display one odometry message:

```bash
ros2 topic echo /odom --once
```

Check publishing frequency:

```bash
ros2 topic hz /odom
```

Expected:

```
10 Hz
```

---

# TF Test

Display the transform:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

---

# World Configuration

Worlds are stored in

```
src/leo_pybullet/worlds/
```

Current worlds:

```
boxes_world.yaml
empty_world.yaml
```

The launch file accepts either

```
world:=boxes
```

or

```
world:=empty
```

or the full path to any YAML file.

---

# Creating a New World

Create a new YAML file inside

```
src/leo_pybullet/worlds/
```

Example:

```yaml
obstacles:

  - name: box_1
    x: 2.0
    y: 0.0
    z: 0.25
    size_x: 0.5
    size_y: 0.5
    size_z: 0.5

  - name: wall
    x: 4.0
    y: 1.5
    z: 0.5
    size_x: 0.2
    size_y: 4.0
    size_z: 1.0
```

Each obstacle represents an axis-aligned box.

| Field | Description |
|--------|-------------|
| name | obstacle identifier |
| x | center x position (m) |
| y | center y position (m) |
| z | center height (m) |
| size_x | box length (m) |
| size_y | box width (m) |
| size_z | box height (m) |

For objects resting on the ground:

```
z = size_z / 2
```

Example:

```
size_z = 0.5

↓

z = 0.25
```

---

# ROS Topics

## Published

| Topic | Type |
|--------|------|
| /odom | nav_msgs/Odometry |
| /scan | sensor_msgs/LaserScan |
| /tf | tf2_msgs/TFMessage |
| /leo_markers | visualization_msgs/MarkerArray |

## Subscribed

| Topic | Type |
|--------|------|
| /cmd_vel | geometry_msgs/Twist |

---

# Project Structure

```
leo_pybullet/

├── launch/
│   └── pybullet_sim.launch.py
│
├── src/
│   └── pybullet_sim_node.cpp
│
├── worlds/
│   ├── boxes_world.yaml
│   └── empty_world.yaml
│
├── rviz/
│
├── include/
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

# Current Features

- Physics-based simulation using Bullet
- Differential drive rover
- Wheel motor control
- LiDAR simulation
- Odometry
- TF broadcasting
- RViz visualization
- YAML-configurable environments
- Runtime world selection

---

# Planned Features

- Goal marker
- Random world generation
- IMU simulation
- Camera simulation
- Reinforcement Learning interface
- Domain randomization
- PyBullet GUI mode (optional)
