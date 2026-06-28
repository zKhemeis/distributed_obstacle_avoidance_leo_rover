# How to Create a Custom Gazebo Map

This guide explains how to create a custom map from the empty Leo Gazebo world.

## 1. Launch the empty world with GUI

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

In Gazebo, add boxes or obstacles manually.

Do not use the Gazebo `Save World` button, because it may not work correctly inside Docker.

## 2. Export the current Gazebo world

Open a second terminal inside Docker:

```bash
docker exec -it leo_dev bash
```

Then run:

```bash
cd /root/leo_ws/src/leo_simulator-ros2/leo_gz_worlds/worlds

ign service \
  -s /world/leo_empty/generate_world_sdf \
  --reqtype ignition.msgs.SdfGeneratorConfig \
  --reptype ignition.msgs.StringMsg \
  --timeout 10000 \
  --req "" \
  > my_map.txt
```

Replace `my_map.txt` with your desired raw export name.

## 3. Convert the exported world to a clean SDF map

```bash
python3 /root/leo_ws/tools/export_gui_world.py my_map.txt my_map.sdf
```

This script removes the spawned robot and keeps the obstacle objects.

## 4. Rebuild the worlds package

```bash
cd /root/leo_ws

colcon build --symlink-install --packages-select leo_gz_worlds leo_gz_bringup

source install/setup.bash
```

## 5. Launch the custom map

Headless:

```bash
ros2 launch leo_gz_bringup leo_gz_headless.launch.py world:=my_map.sdf
```

With GUI:

```bash
ros2 launch leo_gz_bringup leo_gz.launch.py world:=my_map.sdf
```

## 6. Commit the map

```bash
cd /root/leo_ws

git add src/leo_simulator-ros2/leo_gz_worlds/worlds/my_map.sdf
git commit -m "Add custom Gazebo map"
git push
```
