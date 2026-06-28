# Leo Rover ROS 2 Humble Setup on Windows

This project uses:

- ROS 2 Humble
- Gazebo / Ignition Gazebo
- Docker
- WSL2 through Docker Desktop

All required Leo Rover ROS 2 packages are already included inside this repository under `src/`.

---

# 1. Install Docker Desktop

Install Docker Desktop for Windows:

```text
https://www.docker.com/products/docker-desktop/
```

During installation, enable WSL2 support if requested.

After installation, restart Windows if required.

Verify Docker in PowerShell:

```powershell
docker --version
docker run hello-world
```

---

# 2. Open the Project Repository

Open PowerShell in the repository root.

Example:

```powershell
cd C:\Users\<your-user>\Desktop\distributed_obstacle_avoidance_leo_rover
```

Check that the repository contains:

```powershell
dir
```

Expected files/folders include:

```text
Dockerfile
setup_guide_windows.md
src
```

---

# 3. Build the Docker Image

In PowerShell, from the repository root:

```powershell
docker build -t leo_humble_gazebo .
```

This may take several minutes the first time.

---

# 4. Start the Docker Container

In PowerShell, from the repository root:

```powershell
docker run -it --name leo_dev -v ${PWD}:/root/leo_ws leo_humble_gazebo
```

Important:

Run this command only once.

After the container has been created, do not run this command again.

Use this command later to reopen the existing container:

```powershell
docker start -ai leo_dev
```

The important mount is:

```text
-v ${PWD}:/root/leo_ws
```

This connects the local repository to the Docker workspace.

Changes made inside Docker immediately appear in the local repository, and changes made locally immediately appear inside Docker.

---

# 5. Verify the Workspace

Inside the container:

```bash
ls /root/leo_ws
```

Expected files/folders include:

```text
Dockerfile
setup_guide_windows.md
src
```

Check the source folder:

```bash
ls /root/leo_ws/src
```

Expected packages include:

```text
leo_common-ros2
leo_random_walk_cpp
leo_simulator-ros2
```

Important:

The Leo Rover packages are already included in this repository.  
Do not clone `leo_common-ros2` or `leo_simulator-ros2` manually.

---

# 6. Build the ROS Workspace

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

Note:

The warning

```text
running 'rosdep update' as root is not recommended
```

can be ignored inside the Docker container.

Verify that the Leo packages are available:

```bash
ros2 pkg list | grep leo
```

Expected packages include:

```text
leo
leo_description
leo_gz_bringup
leo_gz_plugins
leo_gz_worlds
leo_msgs
leo_random_walk_cpp
leo_simulator
leo_teleop
```

---

# 7. Launch Gazebo With GUI

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

Note:

Gazebo GUI forwarding from Docker on Windows can be difficult. If the GUI does not open, use headless mode or ask a teammate using Ubuntu to test the GUI.

---

# 8. Launch Gazebo Headless

Headless mode runs the simulation without opening the Gazebo GUI.

This is useful if Gazebo GUI does not work on Windows:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz_headless.launch.py
```

---

# 9. Move the Robot Manually

Open a second PowerShell terminal and enter the running container:

```powershell
docker exec -it leo_dev bash
```

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

Check available topics:

```bash
ros2 topic list
```

Move the robot:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Stop the publisher:

```text
Ctrl+C
```

Send a zero-velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

# 10. Run the Random Walk Node

Open a second PowerShell terminal and enter the running container:

```powershell
docker exec -it leo_dev bash
```

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run leo_random_walk_cpp random_walk_node
```

---

# 11. Reopen the Container Later

Terminal 1:

```powershell
docker start -ai leo_dev
```

Terminal 2:

```powershell
docker exec -it leo_dev bash
```

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# 12. Development Workflow

All packages are normal folders inside this repository.

When changing ROS code, rebuild inside Docker:

```bash
cd /root/leo_ws

colcon build --symlink-install

source install/setup.bash
```

Then commit from the main project repository:

```bash
git status
git add .
git commit -m "Describe the change"
git push
```

Branch usage:

```text
main                 stable base
gazebo_simulation    Gazebo-specific work
pybullet_simulation  PyBullet-specific work
```

---

# 13. Common Problems

## Problem: Docker Desktop is not running

Start Docker Desktop manually and wait until it says Docker is running.

Then retry:

```powershell
docker run hello-world
```

---

## Problem: container name "leo_dev" is already in use

The container already exists.

Start it:

```powershell
docker start -ai leo_dev
```

Do not create another container with the same name.

---

## Problem: Gazebo GUI does not open

GUI forwarding from Docker on Windows can be complicated.

First verify that the container works:

```powershell
docker exec -it leo_dev bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/leo_ws/install/setup.bash
ros2 pkg list | grep leo
```

If ROS works but the Gazebo GUI does not open, use headless mode:

```bash
ros2 launch leo_gz_bringup leo_gz_headless.launch.py
```

---

## Problem: ros2 command not found

Source ROS:

```bash
source /opt/ros/humble/setup.bash
```

Then:

```bash
ros2 pkg list
```

---

## Problem: Leo packages are not found

Source the workspace:

```bash
cd /root/leo_ws
source install/setup.bash
```

Then:

```bash
ros2 pkg list | grep leo
```
