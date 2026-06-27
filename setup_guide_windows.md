# Leo Rover ROS 2 Humble Setup on Windows

This project uses:

* ROS 2 Humble
* Gazebo / Ignition Gazebo
* Docker
* WSL2 through Docker Desktop

The Docker setup ensures that all team members use the same environment independent of their host operating system.

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

Use:

```powershell
docker start -ai leo_dev
```

to reopen the existing container.

Explanation:

```text
-v ${PWD}:/root/leo_ws
```

mounts the current repository folder into the container at:

```text
/root/leo_ws
```

Changes made inside Docker immediately appear in the local repository and vice versa.

---

# 5. Verify the Workspace

Inside the container:

```bash
ls /root/leo_ws
```

Expected:

```text
Dockerfile
setup_guide_windows.md
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

# 6. Clone Leo Rover Dependencies

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

Note:

```text
src/leo_common-ros2
src/leo_simulator-ros2
```

are separate Git repositories nested inside the main project repository.

Any modifications inside these folders must be committed and pushed from within those repositories.

---

# 7. Optional: Modifying Leo Rover Dependencies

The folders

```text
src/leo_common-ros2
src/leo_simulator-ros2
```

are independent Git repositories inside the main project repository.

Project structure:

```text
distributed_obstacle_avoidance_leo_rover
├── src/leo_common-ros2
├── src/leo_random_walk_cpp
└── src/leo_simulator-ros2
```

This means:

* Commits inside `src/leo_common-ros2` are separate from the main project repository.
* Commits inside `src/leo_simulator-ros2` are separate from the main project repository.
* Running `git push` in the main project repository will not push changes made inside these folders.

If you only want to run the simulation, no additional steps are required.

If you want to modify these repositories and share your changes with the team, use the team's forks or your own forks:

```text
https://github.com/<your-account>/leo_common-ros2
https://github.com/<your-account>/leo_simulator-ros2
```

Then change the remotes.

For `leo_common-ros2`:

```bash
cd /root/leo_ws/src/leo_common-ros2
git remote set-url origin https://github.com/<your-account>/leo_common-ros2.git
git remote -v
```

For `leo_simulator-ros2`:

```bash
cd /root/leo_ws/src/leo_simulator-ros2
git remote set-url origin https://github.com/<your-account>/leo_simulator-ros2.git
git remote -v
```

After this, commits can be pushed normally from inside each repository:

```bash
git add .
git commit -m "Description of the changes"
git push
```

GitHub may offer to create a Pull Request to the official Leo Rover repositories. This is optional and can be ignored unless you want to contribute your changes back to the Leo Rover project.

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

If everything works, Gazebo should launch.

Note:

Depending on the Windows/Docker/WSL2 setup, Gazebo GUI support may require additional display forwarding configuration. If the GUI does not open, use the Ubuntu guide on a Linux machine or run the simulation in headless mode.

---

# 10. Move the Robot

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

# 11. Reopen the Container Later

Restart the existing container:

```powershell
docker start -ai leo_dev
```

Open a second terminal into the running container:

```powershell
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

Do not create another container.

---

## Problem: Gazebo does not open

GUI forwarding from Docker on Windows can be more complicated than on Ubuntu.

First verify that the container works:

```powershell
docker exec -it leo_dev bash
```

Inside the container:

```bash
ros2 pkg list | grep leo
```

If the ROS packages are available but Gazebo GUI does not open, use headless mode or ask a teammate using Ubuntu to test the GUI.

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

Remember:

```text
Changes in main project files      -> commit from main project repository
Changes in leo_common-ros2         -> commit from src/leo_common-ros2
Changes in leo_simulator-ros2      -> commit from src/leo_simulator-ros2
```

