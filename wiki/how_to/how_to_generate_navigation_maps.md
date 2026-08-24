# Generate Navigation Maps for LEO Rover

This guide explains how to generate valid navigation maps for the three supported methods:

- LiDAR-only Dijkstra.
- PPO trained with one robot.
- PPO trained with four parallel robots.

Each generated map includes:

- A YAML world file.
- A JSON metadata file.
- A CSV manifest containing the robot start position and goal position.

The generator also verifies that a valid path exists.

---

## 1. Prepare the Docker environment

Run these commands whenever you open a new Docker terminal:

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash

source /root/leo_ws/install/setup.bash

source /root/leo_ws/.venv_rl/bin/activate
```

Check the current Git branch:

```bash
git branch --show-current
```

If necessary, switch to the integrated comparison branch:

```bash
git status --short

git switch feature/three-method-navigation-comparison
```

Do not switch branches if `git status --short` shows changes that you have not committed.

---

## 2. Define the map-generation paths

```bash
MAP_SCRIPT=/root/leo_ws/src/leo_pybullet/map_tools/scripts/generate_navigation_map.py

STANDARD_CONFIG=/root/leo_ws/src/leo_pybullet/map_tools/config/policy_lidar_hard_10x10.yaml

LAB_CONFIG=/root/leo_ws/src/leo_pybullet/map_tools/config/policy_lidar_lab_3x2.yaml

STANDARD_MAP_DIRECTORY=/root/leo_ws/src/leo_pybullet/worlds/custom_maps

LAB_MAP_DIRECTORY=/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2

mkdir -p "$STANDARD_MAP_DIRECTORY"

mkdir -p "$LAB_MAP_DIRECTORY"
```

Verify the generator and configurations:

```bash
for REQUIRED_FILE in \
  "$MAP_SCRIPT" \
  "$STANDARD_CONFIG" \
  "$LAB_CONFIG"
do
  if test -f "$REQUIRED_FILE"; then
    echo "FOUND: $REQUIRED_FILE"
  else
    echo "MISSING: $REQUIRED_FILE"
  fi
done
```

Show the available generator options:

```bash
python "$MAP_SCRIPT" --help
```

Available options include:

```text
--seed
--difficulty
--obstacles
--scenario
--output-dir
--generator-config
--start
--goal
--start-yaw
```

---

## 3. Understand the generation options

Difficulty levels:

```text
easy
medium
hard
```

Obstacle-placement scenarios:

```text
random
clear
direct_block
double_block
```

Definitions:

```text
random:
    Obstacles are distributed according to the generator configuration.

clear:
    The direct route is generally less obstructed.

direct_block:
    An obstacle blocks the direct route toward the goal.

double_block:
    Two structured obstacles create a more difficult route.
```

The `--seed` value determines the generated map.

Changing the seed creates a different obstacle arrangement:

```text
--seed 11001
--seed 11002
--seed 11003
```

Use a new seed when the output file already exists.

---

## 4. Generate an easy 10×10-meter map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11001 \
  --difficulty easy \
  --obstacles 4 \
  --scenario clear
```

Generated files:

```text
/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11001.yaml

/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11001.meta.json

/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11001_manifest.csv
```

---

## 5. Generate a medium 10×10-meter map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11002 \
  --difficulty medium \
  --obstacles 8 \
  --scenario direct_block
```

---

## 6. Generate a hard 10×10-meter map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11003 \
  --difficulty hard \
  --obstacles 12 \
  --scenario direct_block
```

---

## 7. Generate a very hard 10×10-meter map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11004 \
  --difficulty hard \
  --obstacles 14 \
  --scenario double_block
```

---

## 8. Generate an extra-hard 10×10-meter map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11005 \
  --difficulty hard \
  --obstacles 18 \
  --scenario double_block
```

Important:

A successfully generated map is physically solvable according to the generator’s validation.

However, a trained PPO policy might still collide or fail on that map.

This indicates a policy limitation, not necessarily a map-generation error.

---

## 9. Generate a 10×10-meter map with a fixed start and goal

```bash
python "$MAP_SCRIPT" \
  --generator-config "$STANDARD_CONFIG" \
  --output-dir "$STANDARD_MAP_DIRECTORY" \
  --seed 11006 \
  --difficulty hard \
  --obstacles 10 \
  --scenario direct_block \
  --start -3.0 0.0 \
  --goal 3.0 0.0 \
  --start-yaw 0.0
```

Meaning:

```text
Start:

x = -3.0 m
y = 0.0 m

Goal:

x = 3.0 m
y = 0.0 m

Initial orientation:

yaw = 0.0 radians
```

Example orientation values:

```text
0.0:
    Facing the positive X direction.

1.5708:
    Facing the positive Y direction.

3.1416:
    Facing the negative X direction.

-1.5708:
    Facing the negative Y direction.
```

---

## 10. Understand the 3×2-meter laboratory map

The laboratory configuration represents an arena with:

```text
Width along X: 3.0 meters.

Height along Y: 2.0 meters.
```

With the coordinate origin at the center:

```text
Left wall:

x = -1.5

Right wall:

x = 1.5

Bottom wall:

y = -1.0

Top wall:

y = 1.0
```

A typical start and goal are:

```text
Start:

x = -1.0
y = 0.0

Goal:

x = 1.0
y = 0.0
```

Because the laboratory arena is much smaller, use fewer obstacles than in a 10×10-meter map.

Typical laboratory obstacle counts:

```text
Easy:

1 obstacle.

Medium:

2 obstacles.

Hard:

2 or 3 obstacles.
```

---

## 11. Generate an easy 3×2-meter laboratory map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12001 \
  --difficulty easy \
  --obstacles 1 \
  --scenario clear \
  --start -1.0 0.0 \
  --goal 1.0 0.0 \
  --start-yaw 0.0
```

Generated files:

```text
/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12001.yaml

/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12001.meta.json

/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12001_manifest.csv
```

---

## 12. Generate a medium 3×2-meter laboratory map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12002 \
  --difficulty medium \
  --obstacles 2 \
  --scenario direct_block \
  --start -1.0 0.0 \
  --goal 1.0 0.0 \
  --start-yaw 0.0
```

---

## 13. Generate a hard 3×2-meter laboratory map

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12003 \
  --difficulty hard \
  --obstacles 3 \
  --scenario direct_block \
  --start -1.0 0.0 \
  --goal 1.0 0.0 \
  --start-yaw 0.0
```

If generation fails, use another seed:

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12013 \
  --difficulty hard \
  --obstacles 3 \
  --scenario direct_block \
  --start -1.0 0.0 \
  --goal 1.0 0.0 \
  --start-yaw 0.0
```

Alternatively, reduce the obstacle count:

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12014 \
  --difficulty hard \
  --obstacles 2 \
  --scenario direct_block \
  --start -1.0 0.0 \
  --goal 1.0 0.0 \
  --start-yaw 0.0
```

---

## 14. Generate a 3×2-meter double-block map

For a small laboratory arena, do not force the start and goal positions when using `double_block`.

Allow the generator to find a feasible start and goal automatically:

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 9710 \
  --difficulty hard \
  --obstacles 3 \
  --scenario double_block
```

If seed `9710` already exists, use another seed:

```bash
python "$MAP_SCRIPT" \
  --generator-config "$LAB_CONFIG" \
  --output-dir "$LAB_MAP_DIRECTORY" \
  --seed 12004 \
  --difficulty hard \
  --obstacles 3 \
  --scenario double_block
```

Important:

The following combination may be impossible in a 3×2-meter arena:

```text
Fixed start:

(-1.0, 0.0)

Fixed goal:

(1.0, 0.0)

Scenario:

double_block
```

The rover footprint, obstacle inflation, and two blocking obstacles can leave insufficient navigable space.

If the generator reports:

```text
could not generate a valid hard world after 400 attempts
```

Choose one of the following:

```text
Use direct_block instead of double_block.

Reduce the number of obstacles.

Choose a different seed.

Allow randomized start and goal positions.

Increase the arena dimensions.
```

---

## 15. Inspect the generated metadata

Example for a laboratory map:

```bash
python -m json.tool \
  "$LAB_MAP_DIRECTORY/map_seed_12002.meta.json"
```

Example for a standard map:

```bash
python -m json.tool \
  "$STANDARD_MAP_DIRECTORY/map_seed_11003.meta.json"
```

The metadata includes information such as:

```text
Seed.

Difficulty.

Scenario.

Obstacle count.

Robot start position.

Goal position.

Whether the direct route is blocked.

Estimated feasible path length.
```

---

## 16. Inspect the generated manifest

Example for a laboratory map:

```bash
cat \
  "$LAB_MAP_DIRECTORY/map_seed_12002_manifest.csv"
```

Example for a standard map:

```bash
cat \
  "$STANDARD_MAP_DIRECTORY/map_seed_11003_manifest.csv"
```

The manifest contains the world path, start position, start orientation, and goal position.

All three navigation methods can use the same generated map and manifest.

---

## 17. Create another arena size

The generator does not currently provide dedicated `--width` and `--height` flags.

To create another arena size, copy an existing generator configuration:

```bash
cp \
  "$LAB_CONFIG" \
  /root/leo_ws/src/leo_pybullet/map_tools/config/policy_lidar_custom_size.yaml
```

Edit the copied configuration:

```bash
gedit \
  /root/leo_ws/src/leo_pybullet/map_tools/config/policy_lidar_custom_size.yaml
```

Change the arena boundaries according to the required dimensions.

For example:

```text
4×3 meters centered at the origin:

x_min = -2.0

x_max = 2.0

y_min = -1.5

y_max = 1.5
```

Another example:

```text
2×3 meters centered at the origin:

x_min = -1.0

x_max = 1.0

y_min = -1.5

y_max = 1.5
```

Generate a map using the new configuration:

```bash
CUSTOM_SIZE_CONFIG=/root/leo_ws/src/leo_pybullet/map_tools/config/policy_lidar_custom_size.yaml

CUSTOM_SIZE_DIRECTORY=/root/leo_ws/src/leo_pybullet/worlds/custom_size_maps

mkdir -p "$CUSTOM_SIZE_DIRECTORY"

python "$MAP_SCRIPT" \
  --generator-config "$CUSTOM_SIZE_CONFIG" \
  --output-dir "$CUSTOM_SIZE_DIRECTORY" \
  --seed 13001 \
  --difficulty medium \
  --obstacles 2 \
  --scenario direct_block
```

Depending on the arena size, you may also need to adjust the configuration’s obstacle sizes, start/goal separation, and boundary clearances.

---

## 18. Reuse a generated map

Once a map exists, do not regenerate it.

Simply use its YAML path:

```bash
CUSTOM_MAP=/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_11003.yaml
```

Or:

```bash
LAB_MAP=/root/leo_ws/src/leo_pybullet/worlds/custom_lab_3x2/map_seed_12002.yaml
```

Then follow:

```text
how_to_run_three_navigation_methods.md
```

The same map can be reused for:

```text
Dijkstra.

Single-robot PPO.

Four-robot shared PPO.
```
