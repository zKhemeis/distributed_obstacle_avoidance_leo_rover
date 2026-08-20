# Leo Stage 1: Random Map and Benchmark Tools

These scripts generate reproducible, axis-aligned box worlds for the current
`leo_pybullet` YAML loader. They do not modify the Bullet simulator.

## 1. Copy into the ROS workspace

Copy this folder to:

```text
/root/leo_ws/src/leo_pybullet/map_tools
```

Run all following commands inside the ROS Humble container.

## 2. Install the only dependency

```bash
apt-get update
apt-get install -y python3-yaml
```

## 3. Adjust the arena, start and goal

Edit:

```text
/root/leo_ws/src/leo_pybullet/map_tools/config/map_generation.yaml
```

The configured arena must describe the usable floor in the Bullet world. The
start pose must match the pose created in `createRobot()`. The goal is used for
map validation now and will later be sent to the PPO deployment node.

The generator creates four physical boundary-wall boxes because the current
simulator has an infinite ground plane. In the configuration, `x_min`, `x_max`,
`y_min` and `y_max` are the inner wall faces. If your simulator later creates
its own physical walls, set `boundary_walls.enabled` to `false`.

## 4. Run unit tests

```bash
cd /root/leo_ws/src/leo_pybullet/map_tools
python3 -m unittest discover -s tests -v
```

## 5. Generate one test world

```bash
cd /root/leo_ws/src/leo_pybullet/map_tools
python3 scripts/generate_world.py \
  --config config/map_generation.yaml \
  --seed 42 \
  --difficulty medium \
  --output /root/leo_ws/src/leo_pybullet/worlds/generated_test.yaml
```

To force an exact number of obstacles, add for example:

```bash
--num-obstacles 7
```

## 6. Validate the world independently

```bash
python3 scripts/validate_world.py \
  --config config/map_generation.yaml \
  --world /root/leo_ws/src/leo_pybullet/worlds/generated_test.yaml
```

## 7. Create an SVG preview

```bash
python3 scripts/visualize_world.py \
  --config config/map_generation.yaml \
  --world /root/leo_ws/src/leo_pybullet/worlds/generated_test.yaml \
  --output /root/leo_ws/src/leo_pybullet/worlds/generated_test.svg
```

The solid red rectangles are the physical obstacles. The light-red rectangles
include the conservative rover clearance. The blue line is the A* validation
path; it is not a path that will be given to the PPO policy.

## 8. Load it in Bullet and RViz

```bash
cd /root/leo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run leo_pybullet pybullet_sim_node \
  --ros-args \
  -p world:=/root/leo_ws/src/leo_pybullet/worlds/generated_test.yaml
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source /root/leo_ws/install/setup.bash
rviz2
```

Confirm that every box is on the floor, markers match LiDAR returns, the rover
does not start inside an obstacle and the map matches the SVG preview.

## 9. Generate disjoint benchmark datasets

Start with a small smoke-test set:

```bash
cd /root/leo_ws/src/leo_pybullet/map_tools
python3 scripts/generate_dataset.py \
  --config config/map_generation.yaml \
  --output-root /root/leo_ws/src/leo_pybullet/worlds/benchmark_smoke \
  --train 12 \
  --validation 6 \
  --test 6
```

After verifying those files in Bullet, generate the full initial set:

```bash
python3 scripts/generate_dataset.py \
  --config config/map_generation.yaml \
  --output-root /root/leo_ws/src/leo_pybullet/worlds/benchmark_v1 \
  --train 300 \
  --validation 60 \
  --test 90
```

Default seed ranges are deliberately disjoint:

- Training: 0 and above
- Validation: 100000 and above
- Testing: 200000 and above

Never tune the policy using the test results. Use the validation set during
development and run the test set only for the final comparison.

The dataset script refuses to overwrite an existing manifest or results file.
Use a new dataset version such as `benchmark_v2` when the configuration changes.
Only use `--force` when you intentionally want to replace an unneeded dataset.

### LiDAR curriculum worlds

`policy_lidar_curriculum_10x10.yaml` creates a deterministic mixture of two
labelled scenarios:

- `direct_block`: the rover initially faces the goal, a substantial obstacle
  blocks the direct route, an A* detour exists, and a minimum path-stretch
  ratio is enforced.
- `clear`: the direct route is verified to remain open so the policy retains
  efficient goal-directed motion.

Generate a curriculum dataset in a new output directory; never overwrite an
existing benchmark:

```bash
python3 scripts/generate_dataset.py \
  --config config/policy_lidar_curriculum_10x10.yaml \
  --output-root /root/leo_ws/src/leo_pybullet/worlds/benchmark_v3_curriculum \
  --train 300 \
  --validation 60 \
  --test 90
```

The scenario label, straight-line distance, A* path length and path-stretch
ratio are recorded in both metadata and `manifest.csv`.

## 10. Evaluation files

`generate_dataset.py` creates:

- `manifest.csv`: immutable description of every benchmark world.
- `evaluation_results.csv`: empty results table for later policy runs.
- `dataset_info.json`: dataset configuration and seed ranges.
- One `.meta.json` beside every YAML world.

The later ROS evaluation node should append one row to
`evaluation_results.csv` after each episode. A row records success, collision,
timeout, duration, path length, minimum LiDAR distance, average speeds and
safety interventions.

After results have been added:

```bash
python3 scripts/summarize_results.py \
  --results /root/leo_ws/src/leo_pybullet/worlds/benchmark_v1/evaluation_results.csv
```

## Important limitation

The current C++ loader has no obstacle yaw field. Do not generate rotated boxes
until `BoxObstacle`, `loadWorldFromYaml()`, `addBoxObstacle()` and the RViz
marker orientation are extended together. This first version exactly matches
the current loader and is therefore safer for the initial PPO baseline.
