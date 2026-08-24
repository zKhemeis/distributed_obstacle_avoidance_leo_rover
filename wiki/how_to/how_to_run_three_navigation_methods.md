# Run the Three LEO Rover Navigation Methods

This guide explains how to run:

1. LiDAR-only Dijkstra navigation.
2. PPO trained with one robot.
3. PPO trained using four parallel robots sharing one policy.

All three methods use:

```text
The same Bullet simulator.

The same ROS 2 environment.

The same generated map.

The same robot start position.

The same goal position.

RViz visualization.
```

---

## 1. Prepare the Docker environment

Whenever you open a new Docker terminal:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

source /root/leo_ws/install/setup.bash

source /root/leo_ws/.venv_rl/bin/activate
```

Check the branch:

```bash
git branch --show-current
```

If needed:

```bash
git status --short

git switch feature/three-method-navigation-comparison
```

Do not switch branches while important changes are uncommitted.

After switching branches, source the environment again:

```bash
source /opt/ros/humble/setup.bash

source /root/leo_ws/install/setup.bash

source /root/leo_ws/.venv_rl/bin/activate
```

---

## 2. Build the packages if required

A rebuild is needed after changing package source code or switching to a branch with different package contents.

It is not necessary merely because a new YAML map was generated.

```bash
cd /root/leo_ws

deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    leo_rl_navigation \
    leo_heuristic_navigation
```

Then reactivate the workspace:

```bash
source /root/leo_ws/install/setup.bash

source /root/leo_ws/.venv_rl/bin/activate
```

---

## 3. Define the frozen PPO policies

```bash
SINGLE_MODEL=/root/leo_ws/deployment/policies/ppo_single/model.zip

SINGLE_CONFIG=/root/leo_ws/deployment/policies/ppo_single/config.yaml

MULTI_MODEL=/root/leo_ws/deployment/policies/ppo_multi/model.zip

MULTI_CONFIG=/root/leo_ws/deployment/policies/ppo_multi/config.yaml
```

Define the Dijkstra configurations:

```bash
DIJKSTRA_CONFIG=/root/leo_ws/src/leo_heuristic_navigation/config/dijkstra_lidar_v2.yaml

LAB_DIJKSTRA_CONFIG=/root/leo_ws/src/leo_heuristic_navigation/config/dijkstra_lidar_lab_3x2.yaml
```

Check the required files:

```bash
for REQUIRED_FILE in \
  "$SINGLE_MODEL" \
  "$SINGLE_CONFIG" \
  "$MULTI_MODEL" \
  "$MULTI_CONFIG" \
  "$DIJKSTRA_CONFIG"
do
  if test -f "$REQUIRED_FILE"; then
    echo "FOUND: $REQUIRED_FILE"
  else
    echo "MISSING: $REQUIRED_FILE"
  fi
done
```

Check the laboratory-specific Dijkstra configuration:

```bash
if test -f "$LAB_DIJKSTRA_CONFIG"; then
  echo "lab_dijkstra_config=FOUND"
else
  echo "ERROR: laboratory Dijkstra configuration missing"
fi
```

The laboratory Dijkstra configuration is necessary when the standard configuration cannot plan correctly inside a 3×2-meter arena.

---

## 4. Select a generated 10×10-meter map

Example:

```bash
CUSTOM_MAP=/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11003.yaml
```

Verify that the map exists:

```bash
test -f "$CUSTOM_MAP" \
  && echo "custom_map=FOUND" \
  || echo "ERROR: custom map missing"
```

The corresponding manifest is:

```bash
CUSTOM_MANIFEST=/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11003_manifest.csv
```

Check it:

```bash
test -f "$CUSTOM_MANIFEST" \
  && echo "custom_manifest=FOUND" \
  || echo "ERROR: custom manifest missing"
```

---

## 5. Run LiDAR-only Dijkstra on a 10×10-meter map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=dijkstra \
  map:="$CUSTOM_MAP" \
  config:="$DIJKSTRA_CONFIG"
```

Important:

The Bullet simulator loads the YAML world because it must create the simulated obstacles.

The Dijkstra navigation node must not receive the obstacle YAML.

Instead, it discovers obstacles from:

```text
/scan
```

and constructs an occupancy grid from the observed measurements.

Stop the launch with:

```text
Ctrl+C
```

before starting another method.

---

## 6. Run single-robot PPO on the same 10×10-meter map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-single \
  map:="$CUSTOM_MAP"
```

To specify the frozen model and configuration explicitly:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-single \
  map:="$CUSTOM_MAP" \
  model:="$SINGLE_MODEL" \
  config:="$SINGLE_CONFIG"
```

Stop the launch with:

```text
Ctrl+C
```

before starting another method.

---

## 7. Run four-robot shared PPO on the same 10×10-meter map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-multi \
  map:="$CUSTOM_MAP"
```

To specify the frozen model and configuration explicitly:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-multi \
  map:="$CUSTOM_MAP" \
  model:="$MULTI_MODEL" \
  config:="$MULTI_CONFIG"
```

Important:

```text
ppo-multi
```

does not display four robots together in the same simulator world.

It deploys one robot using a PPO policy trained from experiences collected by four parallel Bullet/Gymnasium environments.

The four environments shared one PPO policy during training.

---

## 8. Select a generated 3×2-meter laboratory map

Example:

```bash
LAB_MAP=/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12002.yaml

LAB_MANIFEST=/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12002_manifest.csv
```

Verify that the files exist:

```bash
for REQUIRED_FILE in \
  "$LAB_MAP" \
  "$LAB_MANIFEST"
do
  if test -f "$REQUIRED_FILE"; then
    echo "FOUND: $REQUIRED_FILE"
  else
    echo "MISSING: $REQUIRED_FILE"
  fi
done
```

---

## 9. Run LiDAR-only Dijkstra on a 3×2-meter laboratory map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=dijkstra \
  map:="$LAB_MAP" \
  config:="$LAB_DIJKSTRA_CONFIG"
```

Use the laboratory-specific configuration because the default Dijkstra occupancy-grid settings were designed for larger maps.

If the terminal reports:

```text
Online planning failed: Dijkstra could not find a collision-free path
```

check:

```text
The laboratory configuration exists.

The occupancy-grid limits match the 3×2-meter arena.

The occupancy-grid resolution is appropriate.

The start and goal are inside the arena.

The map actually contains a collision-free path.
```

Stop the launch with:

```text
Ctrl+C
```

---

## 10. Run single-robot PPO on a 3×2-meter laboratory map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-single \
  map:="$LAB_MAP"
```

Explicit model and configuration:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-single \
  map:="$LAB_MAP" \
  model:="$SINGLE_MODEL" \
  config:="$SINGLE_CONFIG"
```

---

## 11. Run four-robot shared PPO on a 3×2-meter laboratory map

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-multi \
  map:="$LAB_MAP"
```

Explicit model and configuration:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-multi \
  map:="$LAB_MAP" \
  model:="$MULTI_MODEL" \
  config:="$MULTI_CONFIG"
```

---

## 12. Run a method on an existing benchmark map

Define the benchmark manifest:

```bash
HARD_MANIFEST=/root/leo_ws/src/leo_pybullet/worlds/benchmark_v4_hard/manifest.csv
```

Run Dijkstra:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=dijkstra \
  manifest:="$HARD_MANIFEST" \
  split:=validation \
  world_index:=14 \
  config:="$DIJKSTRA_CONFIG"
```

Run single-robot PPO:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-single \
  manifest:="$HARD_MANIFEST" \
  split:=validation \
  world_index:=14
```

Run four-robot PPO:

```bash
ros2 launch \
  leo_rl_navigation \
  navigation_comparison.launch.py \
  method:=ppo-multi \
  manifest:="$HARD_MANIFEST" \
  split:=validation \
  world_index:=14
```

Choose another benchmark map by changing:

```text
world_index:=8
```

```text
world_index:=14
```

```text
world_index:=29
```

---

## 13. Verify that Dijkstra uses LiDAR only

Keep the Dijkstra launch running in the first terminal.

Open a second Docker terminal and prepare its environment:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

source /root/leo_ws/install/setup.bash

source /root/leo_ws/.venv_rl/bin/activate
```

Confirm that the navigation node is running:

```bash
ros2 node list \
  | grep -Fx '/leo_dijkstra_navigation_node'
```

Check the LiDAR topic:

```bash
ros2 topic info /scan
```

Check the discovered occupancy-grid topic:

```bash
ros2 topic info /dijkstra_discovered_map
```

Check that the Dijkstra node did not receive a world-file parameter:

```bash
if ! ros2 node list \
  | grep -Fxq '/leo_dijkstra_navigation_node'
then
  echo "ERROR: Dijkstra node is not running"

elif ros2 param list /leo_dijkstra_navigation_node \
  | grep -Eq '^[[:space:]]*world_path$'
then
  echo "ERROR: Dijkstra received a world YAML path"

else
  echo "dijkstra_lidar_only_check=PASS"
fi
```

Expected:

```text
dijkstra_lidar_only_check=PASS
```

Important:

If the node is not running, the verification must not be interpreted as a successful LiDAR-only check.

---

## 14. Save evaluation results for the three methods

Define a shared output directory:

```bash
COMPARISON_RESULTS=/root/leo_ws/results/three_method_comparison

mkdir -p "$COMPARISON_RESULTS"
```

For a standard 10×10-meter map:

```bash
EVALUATION_MANIFEST="$CUSTOM_MANIFEST"

EVALUATION_DIJKSTRA_CONFIG="$DIJKSTRA_CONFIG"
```

For a laboratory 3×2-meter map:

```bash
EVALUATION_MANIFEST="$LAB_MANIFEST"

EVALUATION_DIJKSTRA_CONFIG="$LAB_DIJKSTRA_CONFIG"
```

Choose one of the two configurations above before continuing.

Evaluate Dijkstra:

```bash
python -m leo_heuristic_navigation.evaluate_dijkstra \
  --config "$EVALUATION_DIJKSTRA_CONFIG" \
  --manifest "$EVALUATION_MANIFEST" \
  --split validation \
  --episodes  1 \
  --seed 42 \
  --output "$COMPARISON_RESULTS/dijkstra.csv"
```

Evaluate single-robot PPO:

```bash
python -m leo_rl_navigation.evaluate_policy \
  --config "$SINGLE_CONFIG" \
  --model "$SINGLE_MODEL" \
  --manifest "$EVALUATION_MANIFEST" \
  --split validation \
  --episodes 1 \
  --seed 42 \
  --device cpu \
  --disable-safety-shield \
  --output "$COMPARISON_RESULTS/ppo_single.csv"
```

Evaluate four-robot PPO:

```bash
python -m leo_rl_navigation.evaluate_policy \
  --config "$MULTI_CONFIG" \
  --model "$MULTI_MODEL" \
  --manifest "$EVALUATION_MANIFEST" \
  --split validation \
  --episodes 1 \
  --seed 42 \
  --device cpu \
  --disable-safety-shield \
  --output "$COMPARISON_RESULTS/ppo_multi.csv"
```

The resulting CSV files are:

```text
/root/leo_ws/results/three_method_comparison/dijkstra.csv

/root/leo_ws/results/three_method_comparison/ppo_single.csv

/root/leo_ws/results/three_method_comparison/ppo_multi.csv
```

If you evaluate a different map, use different output filenames to avoid overwriting previous results.

---

## 15. Prevent duplicate ROS nodes

Always terminate an existing launch before starting another:

```text
Ctrl+C
```

If multiple launches remain active, possible symptoms include:

```text
The goal marker jumps between two locations.

The robot receives conflicting velocity commands.

The robot does not move.

RViz shows inconsistent robot behavior.

Multiple policy nodes remain active.
```

Check for duplicate nodes:

```bash
ros2 node list \
  | sort \
  | uniq -c
```

Check the velocity command topic:

```bash
ros2 topic info /cmd_vel
```

A normal single-robot navigation launch should have one active navigation-command publisher.

Inspect remaining processes:

```bash
pgrep -af \
  'navigation_comparison.launch.py|leo_rl_navigation.ros_policy_node|leo_dijkstra_navigation_node|pybullet_sim_node|rviz2' \
  || true
```

Return to the terminal containing the old launch and stop it using:

```text
Ctrl+C
```

Then launch only one navigation method again.

---

## 16. Fair-comparison requirements

For a meaningful comparison, all three methods must use:

```text
The same world.

The same start position.

The same start orientation.

The same goal position.

The same Bullet simulation environment.

Comparable success, collision, timeout, and stuck metrics.
```

Additional requirements:

```text
Dijkstra must discover obstacles through LiDAR measurements.

The Dijkstra navigation node must not read obstacle locations from the world YAML.

PPO evaluations must have the safety shield disabled.

The PPO methods must not receive external heuristic interventions.

The four-robot PPO method represents shared-policy parallel training, not four robots driving simultaneously in the same world.
```
