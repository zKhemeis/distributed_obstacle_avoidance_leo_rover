#!/usr/bin/env python3

"""Generate and validate one reusable navigation map."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import tempfile

from pathlib import Path

import yaml


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a map, validate that a path exists, "
            "and create a manifest usable by all navigation methods."
        ),
    )

    parser.add_argument(
        "--seed",
        type=int,
        required=True,
        help="Unique seed used to generate the map.",
    )

    parser.add_argument(
        "--difficulty",
        choices=(
            "easy",
            "medium",
            "hard",
        ),
        default="medium",
        help="Map difficulty.",
    )

    parser.add_argument(
        "--obstacles",
        "--num-obstacles",
        dest="obstacles",
        type=int,
        default=None,
        help="Number of randomly generated obstacles.",
    )

    parser.add_argument(
        "--scenario",
        choices=(
            "random",
            "clear",
            "direct_block",
            "double_block",
        ),
        default="direct_block",
        help="Obstacle-placement scenario.",
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional custom output directory.",
    )

    parser.add_argument(
        "--generator-config",
        type=Path,
        default=None,
        help="Optional custom generator configuration.",
    )

    parser.add_argument(
        "--start",
        nargs=2,
        type=float,
        metavar=(
            "X",
            "Y",
        ),
        default=None,
        help="Optional fixed robot start position.",
    )

    parser.add_argument(
        "--goal",
        nargs=2,
        type=float,
        metavar=(
            "X",
            "Y",
        ),
        default=None,
        help="Optional fixed goal position.",
    )

    parser.add_argument(
        "--start-yaw",
        type=float,
        default=0.0,
        help="Initial robot yaw when --start is specified.",
    )

    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()

    if arguments.seed < 0:
        raise SystemExit(
            "ERROR: --seed cannot be negative."
        )

    if (
        arguments.obstacles is not None
        and arguments.obstacles < 0
    ):
        raise SystemExit(
            "ERROR: --obstacles cannot be negative."
        )

    if (
        arguments.start is None
    ) != (
        arguments.goal is None
    ):
        raise SystemExit(
            "ERROR: provide --start and --goal together."
        )

    script_path = Path(
        __file__
    ).resolve()

    map_tools = script_path.parents[1]

    workspace = script_path.parents[4]

    generator_config = (
        arguments.generator_config
        or (
            map_tools
            / "config"
            / "policy_lidar_hard_10x10.yaml"
        )
    ).expanduser().resolve()

    output_directory = (
        arguments.output_dir
        or (
            workspace
            / "src"
            / "leo_pybullet"
            / "worlds"
            / "custom_maps"
        )
    ).expanduser().resolve()

    generator_script = (
        map_tools
        / "scripts"
        / "generate_world.py"
    )

    validator_script = (
        map_tools
        / "scripts"
        / "validate_world.py"
    )

    required_files = {
        "generator configuration": generator_config,
        "generator script": generator_script,
        "validator script": validator_script,
    }

    for description, path in required_files.items():
        if not path.is_file():
            raise SystemExit(
                f"ERROR: missing {description}: {path}"
            )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    prefix = (
        f"map_seed_{arguments.seed}"
    )

    world_path = (
        output_directory
        / f"{prefix}.yaml"
    )

    metadata_path = (
        world_path.with_suffix(
            ".meta.json"
        )
    )

    manifest_path = (
        output_directory
        / f"{prefix}_manifest.csv"
    )

    if (
        world_path.exists()
        or metadata_path.exists()
        or manifest_path.exists()
    ):
        raise SystemExit(
            f"ERROR: seed {arguments.seed} already exists. "
            "Choose another seed."
        )

    with generator_config.open(
        encoding="utf-8",
    ) as stream:
        configuration = yaml.safe_load(
            stream
        )

    with tempfile.TemporaryDirectory(
        prefix="leo_navigation_map_",
    ) as temporary_directory:
        temporary_root = Path(
            temporary_directory
        )

        generation_config = (
            generator_config
        )

        if arguments.start is not None:
            configuration["start"].update(
                {
                    "x": arguments.start[0],
                    "y": arguments.start[1],
                    "yaw": arguments.start_yaw,
                }
            )

            configuration["goal"].update(
                {
                    "x": arguments.goal[0],
                    "y": arguments.goal[1],
                }
            )

            configuration.setdefault(
                "start_goal_randomization",
                {},
            )["enabled"] = False

            generation_config = (
                temporary_root
                / "generation.yaml"
            )

            with generation_config.open(
                "w",
                encoding="utf-8",
            ) as stream:
                yaml.safe_dump(
                    configuration,
                    stream,
                    sort_keys=False,
                )

        command = [
            sys.executable,
            str(
                generator_script
            ),
            "--config",
            str(
                generation_config
            ),
            "--seed",
            str(
                arguments.seed
            ),
            "--difficulty",
            arguments.difficulty,
            "--scenario",
            arguments.scenario,
            "--output",
            str(
                world_path
            ),
        ]

        if arguments.obstacles is not None:
            command.extend(
                [
                    "--num-obstacles",
                    str(
                        arguments.obstacles
                    ),
                ]
            )

        subprocess.run(
            command,
            cwd=map_tools,
            check=True,
        )

        with metadata_path.open(
            encoding="utf-8",
        ) as stream:
            metadata = json.load(
                stream
            )

        start = metadata[
            "start"
        ]

        goal = metadata[
            "goal"
        ]

        configuration[
            "start"
        ].update(
            start
        )

        configuration[
            "goal"
        ].update(
            goal
        )

        configuration.setdefault(
            "start_goal_randomization",
            {},
        )["enabled"] = False

        validation_config = (
            temporary_root
            / "validation.yaml"
        )

        with validation_config.open(
            "w",
            encoding="utf-8",
        ) as stream:
            yaml.safe_dump(
                configuration,
                stream,
                sort_keys=False,
            )

        subprocess.run(
            [
                sys.executable,
                str(
                    validator_script
                ),
                "--config",
                str(
                    validation_config
                ),
                "--world",
                str(
                    world_path
                ),
            ],
            cwd=map_tools,
            check=True,
        )

    manifest_row = {
        "split": "validation",
        "world_file": str(
            world_path
        ),
        "seed": metadata[
            "seed"
        ],
        "difficulty": metadata[
            "difficulty"
        ],
        "scenario": metadata[
            "scenario"
        ],
        "structured_blocker_count": metadata.get(
            "structured_blocker_count",
            0,
        ),
        "random_obstacle_count": metadata[
            "random_obstacle_count"
        ],
        "boundary_wall_count": metadata.get(
            "boundary_wall_count",
            0,
        ),
        "total_box_count": metadata.get(
            "total_box_count",
            0,
        ),
        "direct_path_blocked": metadata[
            "direct_path_blocked"
        ],
        "straight_distance_m": metadata.get(
            "straight_distance_m",
            "",
        ),
        "astar_path_length_m": metadata.get(
            "astar_path_length_m",
            "",
        ),
        "path_stretch_ratio": metadata[
            "path_stretch_ratio"
        ],
        "start_x": start[
            "x"
        ],
        "start_y": start[
            "y"
        ],
        "start_yaw": start.get(
            "yaw",
            0.0,
        ),
        "goal_x": goal[
            "x"
        ],
        "goal_y": goal[
            "y"
        ],
    }

    with manifest_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(
                manifest_row
            ),
        )

        writer.writeheader()

        writer.writerow(
            manifest_row
        )

    print()
    print("Map:", world_path)
    print("Manifest:", manifest_path)
    print("Difficulty:", metadata["difficulty"])
    print("Scenario:", metadata["scenario"])
    print(
        "Obstacles:",
        metadata["random_obstacle_count"],
    )
    print("Start:", start)
    print("Goal:", goal)
    print()
    print("map_generation_and_validation=PASS")


if __name__ == "__main__":
    main()
