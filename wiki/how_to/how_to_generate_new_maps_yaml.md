# Generate New Maps and Evaluate the Trained PPO Policy

## 1. Prepare the environment

```bash
cd /root/leo_ws

source /opt/ros/humble/setup.bash
source /root/leo_ws/install/setup.bash
source /root/leo_ws/.venv_rl/bin/activate
```

## 2. Define the trained policy

```bash
PPO_MODEL=/root/leo_ws/results/rl_v8_pure_ppo/ppo_maneuver_focused_smoke_60k_seed1042_002/checkpoints/ppo_leo_60000_steps.zip

PPO_CONFIG=/root/leo_ws/src/leo_rl_navigation/config/ppo_lidar_maneuver_focused_v8.yaml

MAP_TOOLS=/root/leo_ws/src/leo_pybullet/map_tools

GENERATOR_CONFIG="$MAP_TOOLS/config/policy_lidar_hard_10x10.yaml"

WORLD_DIRECTORY=/root/leo_ws/src/leo_pybullet/worlds/custom_maps

mkdir -p "$WORLD_DIRECTORY"
```

Verify that the model exists:

```bash
test -f "$PPO_MODEL" \
  && echo "PPO model found" \
  || echo "ERROR: PPO model missing"
```

## 3. Choose the map difficulty

Adjust the following values for every new map:

```bash
MAP_SEED=9301

MAP_DIFFICULTY=hard

MAP_OBSTACLES=14

MAP_SCENARIO=double_block
```

Available examples:

```text
Easy:

MAP_DIFFICULTY=easy
MAP_OBSTACLES=4
MAP_SCENARIO=clear

Medium:

MAP_DIFFICULTY=medium
MAP_OBSTACLES=8
MAP_SCENARIO=direct_block

Hard:

MAP_DIFFICULTY=hard
MAP_OBSTACLES=12
MAP_SCENARIO=direct_block

Very hard:

MAP_DIFFICULTY=hard
MAP_OBSTACLES=14
MAP_SCENARIO=double_block
```

Supported scenarios:

```text
random
clear
direct_block
double_block
```

Use a new `MAP_SEED` to generate a different map.

## 4. Define the map output files

```bash
WORLD_FILE="$WORLD_DIRECTORY/map_seed_${MAP_SEED}.yaml"

METADATA_FILE="$WORLD_DIRECTORY/map_seed_${MAP_SEED}.meta.json"

PREVIEW_FILE="$WORLD_DIRECTORY/map_seed_${MAP_SEED}.svg"

MANIFEST_FILE="$WORLD_DIRECTORY/map_seed_${MAP_SEED}_manifest.csv"

PREVIEW_CONFIG="/tmp/leo_map_seed_${MAP_SEED}_preview.yaml"

RESULT_FILE="$WORLD_DIRECTORY/map_seed_${MAP_SEED}_result.csv"
```

## 5. Generate the map

```bash
cd "$MAP_TOOLS"

python3 scripts/generate_world.py \
  --config "$GENERATOR_CONFIG" \
  --seed "$MAP_SEED" \
  --difficulty "$MAP_DIFFICULTY" \
  --num-obstacles "$MAP_OBSTACLES" \
  --scenario "$MAP_SCENARIO" \
  --output "$WORLD_FILE"
```

The generator creates:

```text
map_seed_9301.yaml

map_seed_9301.meta.json
```

If the world already exists, choose another seed.

## 6. Create the preview configuration and manifest

```bash
export GENERATOR_CONFIG

export WORLD_FILE

export METADATA_FILE

export MANIFEST_FILE

export PREVIEW_CONFIG
```

```bash
python - <<'PY'
import csv
import json
import os
from pathlib import Path

import yaml

world = Path(
    os.environ["WORLD_FILE"]
)

metadata_path = Path(
    os.environ["METADATA_FILE"]
)

manifest_path = Path(
    os.environ["MANIFEST_FILE"]
)

generator_config_path = Path(
    os.environ["GENERATOR_CONFIG"]
)

preview_config_path = Path(
    os.environ["PREVIEW_CONFIG"]
)

with metadata_path.open(
    encoding="utf-8"
) as stream:
    metadata = json.load(stream)

with generator_config_path.open(
    encoding="utf-8"
) as stream:
    configuration = yaml.safe_load(stream)

start = metadata["start"]
goal = metadata["goal"]

configuration["start"].update(start)
configuration["goal"].update(goal)

with preview_config_path.open(
    "w",
    encoding="utf-8",
) as stream:
    yaml.safe_dump(
        configuration,
        stream,
        sort_keys=False,
    )

row = {
    "split": "validation",
    "world_file": str(world.resolve()),
    "seed": metadata["seed"],
    "difficulty": metadata["difficulty"],
    "scenario": metadata["scenario"],
    "random_obstacle_count": metadata[
        "random_obstacle_count"
    ],
    "direct_path_blocked": metadata[
        "direct_path_blocked"
    ],
    "path_stretch_ratio": metadata[
        "path_stretch_ratio"
    ],
    "start_x": float(start["x"]),
    "start_y": float(start["y"]),
    "start_yaw": float(
        start.get("yaw", 0.0)
    ),
    "goal_x": float(goal["x"]),
    "goal_y": float(goal["y"]),
}

with manifest_path.open(
    "w",
    newline="",
    encoding="utf-8",
) as stream:
    writer = csv.DictWriter(
        stream,
        fieldnames=list(row),
    )

    writer.writeheader()
    writer.writerow(row)

print("World:", world)
print("Manifest:", manifest_path)
print("Difficulty:", metadata["difficulty"])
print("Scenario:", metadata["scenario"])
print("Obstacles:", metadata["random_obstacle_count"])
print("Path stretch:", metadata["path_stretch_ratio"])
print("Start:", start)
print("Goal:", goal)
PY
```

## 7. Validate the generated map

```bash
python3 scripts/validate_world.py \
  --config "$PREVIEW_CONFIG" \
  --world "$WORLD_FILE"
```

Expected:

```text
VALID: ...
A* path exists: yes
```

A* is used only to verify that the generated map is solvable. The PPO policy does not follow the A* path.

## 8. Generate the SVG preview

```bash
python3 scripts/visualize_world.py \
  --config "$PREVIEW_CONFIG" \
  --world "$WORLD_FILE" \
  --output "$PREVIEW_FILE"
```

The preview is saved as:

```text
/root/leo_ws/src/leo_pybullet/worlds/custom_maps/map_seed_9301.svg
```

## 9. Evaluate the existing PPO policy

```bash
cd /root/leo_ws

python -m leo_rl_navigation.evaluate_policy \
  --config "$PPO_CONFIG" \
  --model "$PPO_MODEL" \
  --manifest "$MANIFEST_FILE" \
  --split validation \
  --episodes 1 \
  --seed 42 \
  --device cpu \
  --disable-safety-shield \
  --output "$RESULT_FILE"
```

A successful run reports:

```text
successes=1
collisions=0
safety_shield=False
safety_intervention_steps=0
```

## 10. Visualize the robot in RViz

```bash
ros2 launch leo_rl_navigation \
  ppo_rviz_demo.launch.py \
  config:="$PPO_CONFIG" \
  manifest:="$MANIFEST_FILE" \
  split:=validation \
  world_index:=0 \
  model:="$PPO_MODEL" \
  explicit_front_clearance:=true \
  safety_shield:=false \
  detailed_model:=true \
  rviz:=true
```

`world_index:=0` is used because the custom manifest contains only one map.

To create another map, change:

```bash
MAP_SEED
MAP_DIFFICULTY
MAP_OBSTACLES
MAP_SCENARIO
```

Then repeat Steps 4–10.
