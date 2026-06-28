# Leo Rover ROS 2 Humble Setup on Ubuntu

This project uses:

- ROS 2 Humble
- Gazebo / Ignition Gazebo
- Docker

All required Leo Rover ROS 2 packages are already included inside this repository under `src/`.

---

# 1. Install Docker Engine on Ubuntu

On Ubuntu, Docker Desktop is not required.

Update the package list and install the required tools:

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg
```

Add Docker's official GPG key:

```bash
sudo install -m 0755 -d /etc/apt/keyrings

sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc

sudo chmod a+r /etc/apt/keyrings/docker.asc
```

Add the Docker repository:

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME}) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

Install Docker:

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Verify the installation:

```bash
docker --version
docker run hello-world
```

---

# 2. Allow Docker Without sudo

Add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
```

Apply the group change:

```bash
newgrp docker
```

Or simply log out and log in again.

Verify:

```bash
docker run hello-world
```

---

# 3. Build the Docker Image

Open a terminal in the repository root.

Example:

```bash
cd ~/Desktop/distributed_obstacle_avoidance_leo_rover
```

Build the Docker image:

```bash
docker build -t leo_humble_gazebo .
```

This may take several minutes the first time.

---

# 4. Allow Gazebo GUI From Docker

Because Gazebo runs inside Docker but displays on Ubuntu, allow local containers to access the display:

```bash
xhost +local:
```

You only need to run this once per login session.

Optional: remove the permission after finishing:

```bash
xhost -local:
```

---

# 5. Start the Docker Container

Open a terminal in the repository root and run:

```bash
docker run -it \
  --name leo_dev \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ${PWD}:/root/leo_ws \
  leo_humble_gazebo
```

Important:

Run this command only once.

After the container has been created, do not run this command again.

Use this command later to reopen the existing container:

```bash
docker start -ai leo_dev
```

The important mount is:

```text
--volume ${PWD}:/root/leo_ws
```

This connects the local repository to the Docker workspace.

Changes made inside Docker immediately appear in the local repository, and changes made locally immediately appear inside Docker.

---

# 6. Verify the Workspace

Inside the container:

```bash
ls /root/leo_ws
```

Expected files/folders include:

```text
Dockerfile
setup_guide_ubuntu.md
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

# 7. Build the ROS Workspace

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

# 8. Launch Gazebo With GUI

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

If everything works, Gazebo should open on the Ubuntu desktop.

---

# 9. Launch Gazebo Headless

Headless mode runs the simulation without opening the Gazebo GUI.

This is useful for better performance:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz_headless.launch.py
```

---

# 10. Move the Robot Manually

Open a second terminal and enter the running container:

```bash
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

# 11. Run the Random Walk Node

Open a second terminal and enter the running container:

```bash
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

# 12. Reopen the Container Later

Terminal 1:

```bash
docker start -ai leo_dev
```

Terminal 2:

```bash
docker exec -it leo_dev bash
```

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# 13. Development Workflow

All packages are normal folders inside this repository.

When changing ROS code, rebuild inside Docker:

```bash
cd /root/leo_ws

colcon build --symlink-install

source install/setup.bash
```

Then commit from the main project repository on the host machine:

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

# 14. Common Problems

## Problem: permission denied while trying to connect to the Docker daemon socket

Run:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Then test again:

```bash
docker run hello-world
```

If it still does not work, log out and log in again.

---

## Problem: container name "leo_dev" is already in use

The container already exists.

Start it:

```bash
docker start -ai leo_dev
```

Do not create another container with the same name.

---

## Problem: Gazebo does not open

Allow Docker to use the Ubuntu display:

```bash
xhost +local:
```

Verify:

```bash
echo $DISPLAY
```

It should not be empty.

The container must have been started with:

```text
--env DISPLAY=$DISPLAY
--env QT_X11_NO_MITSHM=1
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw
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

---

## Problem: Git says permission denied or dubious ownership

This can happen because Docker runs as root and the project folder is mounted into the container.

On the Ubuntu host, run:

```bash
sudo chown -R $USER:$USER ~/Desktop/distributed_obstacle_avoidance_leo_rover
```

If your project is stored somewhere else, replace the path.
