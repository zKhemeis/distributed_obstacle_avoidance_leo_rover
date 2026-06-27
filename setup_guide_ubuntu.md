# Leo Rover ROS 2 Humble Setup on Ubuntu
---

# 1. Install Docker Engine on Ubuntu

On Ubuntu, Docker Desktop is not required.  
Use Docker Engine directly.

Update the package list and install required tools:

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
sudo docker --version

sudo docker run hello-world
```

---

# 2. Allow Docker Without `sudo`

Add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
```

Apply the group change:

```bash
newgrp docker
```

Or log out and log in again.

Verify that Docker works without `sudo`:

```bash
docker run hello-world
```

---

# 3. Install Git

If Git is not installed yet:

```bash
sudo apt update

sudo apt install -y git
```

Verify:

```bash
git --version
```

---

# 4. Build the Docker Image

Open a terminal in the repository root.

Example:

```bash
cd ~/Desktop/ros_projects/leo_rover/RobotRepoLeo/project_ws
```

Build the Docker image:

```bash
docker build -t leo_humble_gazebo .
```

This may take several minutes the first time.

---

# 5. Allow Gazebo GUI From Docker

Because Gazebo runs inside Docker but displays on the Ubuntu desktop, allow local Docker containers to access the X11 display:

```bash
xhost +local:docker
```

You only need to run this once per login session.

Optional: after finishing your work, you can close the permission again:

```bash
xhost -local:docker
```

---

# 6. Start the Docker Container

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

Explanation of the important parts:

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

Mounts the current project folder into the container at `/root/leo_ws`.

---

# 7. Clone Leo Rover Dependencies

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

Open a second terminal on Ubuntu and enter the running container:

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

Stop with:

```text
Ctrl+C
```

Send a zero-velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

# 11. Reopen the Container Later

If the container already exists, restart it:

```bash
docker start -ai leo_dev
```

Open a second terminal into the running container:

```bash
docker exec -it leo_dev bash
```

Then source the workspace again:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

source install/setup.bash
```

---

# 12. Common Problems

## Problem: `permission denied while trying to connect to the Docker daemon socket`

This means your user is not allowed to run Docker without `sudo`.

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

## Problem: `docker: Error response from daemon: Conflict. The container name "/leo_dev" is already in use`

This means the container already exists.

Start the existing container:

```bash
docker start -ai leo_dev
```

Or remove the old container and create a new one:

```bash
docker rm leo_dev
```

Then run the `docker run` command again.

---

## Problem: Gazebo does not open

First, allow Docker to use the Ubuntu display:

```bash
xhost +local:docker
```

Then make sure the container was started with these options:

```bash
--env DISPLAY=$DISPLAY
--env QT_X11_NO_MITSHM=1
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw
```

You can also test GUI forwarding with:

```bash
docker exec -it leo_dev bash
```

Inside the container:

```bash
echo $DISPLAY
```

If `$DISPLAY` is empty, the container was started without display forwarding.

---

## Problem: `ros2: command not found`

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Then try again:

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

Then check again:

```bash
ros2 pkg list | grep leo
```

---

# 13. Notes

Always start work from the repository root before running Docker commands.

Example:

```bash
cd ~/Desktop/ros_projects/leo_rover/RobotRepoLeo/project_ws
```

If you make changes to ROS packages, rebuild the workspace inside the container:

```bash
cd /root/leo_ws

colcon build

source install/setup.bash
```
