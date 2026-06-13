# Leo Rover ROS 2 Humble Setup

This project uses:

* ROS 2 Humble
* Gazebo / Ignition Gazebo
* Docker

The Docker setup ensures that all team members use the same environment independent of their host operating system.

---

# 1. Install Docker Desktop

Install Docker Desktop:

```text
https://www.docker.com/products/docker-desktop/
```

During installation, enable WSL2 support if requested.

Verify the installation:

```powershell
docker --version
docker run hello-world
```

---

# 2. Build the Docker Image

Open a terminal in the repository root and run:

```powershell
docker build -t leo_humble_gazebo .
```

This may take several minutes the first time.

---

# 3. Start the Docker Container

Open a terminal in the repository root and run:

```powershell
docker run -it --name leo_dev -v ${PWD}:/root/leo_ws leo_humble_gazebo
```

---

# 4. Build the ROS Workspace

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.bash
```

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
leo_simulator
leo_teleop
```

---

# 5. Launch Gazebo

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

---

# 6. Move the Robot

Open a second terminal and enter the running container:

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

Stop with:

```text
Ctrl+C
```

Send a zero-velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

# 7. Reopen the Container Later

Restart the container:

```powershell
docker start -ai leo_dev
```

Open a second terminal into the running container:

```powershell
docker exec -it leo_dev bash
```



