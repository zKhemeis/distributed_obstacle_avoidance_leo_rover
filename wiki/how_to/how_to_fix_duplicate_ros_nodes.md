# Fix Duplicate ROS Nodes and Changing Goal Positions

## Cause

Previous simulation sessions were not closed correctly. Multiple PPO nodes publish different goals and velocity commands, causing the goal to change position or the robot to stop moving.

## Check for duplicate nodes

```bash
ros2 node list | sort | uniq -c

ros2 topic info /rl_goal

ros2 topic info /cmd_vel
```

Each topic should have only one publisher.

## Find running simulation processes

```bash
pgrep -af \
  'leo_rl_navigation.ros_policy_node|pybullet_sim_node|rviz2'
```

## Stop old processes

Replace the numbers with the actual process IDs:

```bash
kill -TERM PID_1 PID_2
```

Example:

```bash
kill -TERM 12293 12373
```

## Restart ROS discovery

```bash
ros2 daemon stop

ros2 daemon start
```

## Verify

```bash
ros2 node list | sort | uniq -c

ros2 topic info /rl_goal

ros2 topic info /cmd_vel
```

## Prevention

Always press `Ctrl+C` and wait for the current simulation to stop before launching another map. Do not run multiple non-namespaced simulations simultaneously.
