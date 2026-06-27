# Leo Rover ROS 2 Humble Setup on Ubuntu

This project uses:

* ROS 2 Humble
* Gazebo / Ignition Gazebo
* Docker

The Docker setup ensures that all team members use the same environment independent of their host operating system.

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

Use:

```bash
docker start -ai leo_dev
```

to reopen the existing container.

Explanation of the important options:

```text
--env DISPLAY=$DISPLAY
```

Forwards the display variable from Ubuntu to the container.

```text
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw
```

Allows graphical applications inside the container to open windows on Ubuntu.

```text
--volume ${PWD}:/root/leo_ws
```

Mounts the current repository folder into the container at:

```text
/root/leo_ws
```

Changes made inside Docker immediately appear in the local repository and vice versa.

---

# 6. Verify the Workspace

Inside the container:

```bash
ls /root/leo_ws
```

Expected:

```text
Dockerfile
setup_guide_ubuntu.md
src
```

Check the workspace source folder:

```bash
ls /root/leo_ws/src
```

Expected:

```text
leo_random_walk_cpp
```

---

# 7. Clone Leo Rover Dependencies

Only execute this step once.

Inside the container:

```bash
cd /root/leo_ws/src

git clone -b humble https://github.com/LeoRover/leo_common-ros2.git

git clone -b humble https://github.com/LeoRover/leo_simulator-ros2.git
```

Verify:

```bash
ls
```

Expected:

```text
leo_common-ros2
leo_random_walk_cpp
leo_simulator-ros2
```

---

# 8. Build the ROS Workspace

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

# 9. Launch Gazebo

Inside the container:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

If everything works, Gazebo should open on the Ubuntu desktop.

---

# 10. Move the Robot

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

# 11. Reopen the Container Later

Restart the existing container:

```bash
docker start -ai leo_dev
```

Open a second terminal into the running container:

```bash
docker exec -it leo_dev bash
```

Source the workspace again:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# 12. Common Problems

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

Do not create another container.

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

# 13. Development Workflow

Whenever you modify ROS packages:

```bash
cd /root/leo_ws

colcon build --symlink-install

source install/setup.bash
```

Because the repository is mounted into Docker:

```text
Host Repository  <----->  Docker Workspace (/root/leo_ws)
```

any file changes made inside Docker immediately appear in the local repository, and any changes made locally immediately appear inside Docker.

