#!/usr/bin/env python3
"""Generate one reproducible leo_pybullet YAML world."""

from __future__ import annotations

import argparse
from pathlib import Path

from world_tools import generate_boxes, read_yaml, write_metadata, write_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--difficulty", choices=("easy", "medium", "hard"), default="medium")
    parser.add_argument("--num-obstacles", type=int, default=None)
    parser.add_argument(
        "--scenario",
        choices=("random", "clear", "direct_block"),
        default=None,
    )
    parser.add_argument("--force", action="store_true", help="allow overwriting output files")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metadata_path = args.output.with_suffix(".meta.json")
    if not args.force and (args.output.exists() or metadata_path.exists()):
        raise SystemExit("output already exists; choose another path or add --force")
    config = read_yaml(args.config)
    boxes, metadata = generate_boxes(
        config,
        args.seed,
        args.difficulty,
        args.num_obstacles,
        scenario=args.scenario,
    )
    write_world(args.output, boxes)
    metadata["world_file"] = str(args.output)
    write_metadata(metadata_path, metadata)
    print(f"Generated: {args.output}")
    print(f"Metadata:  {metadata_path}")
    print(f"Random obstacles: {metadata['random_obstacle_count']}")
    print(f"Boundary walls:   {metadata['boundary_wall_count']}")
    print(f"Direct path blocked: {metadata['direct_path_blocked']}")
    print(f"Scenario: {metadata['scenario']}")
    print(f"Path stretch: {metadata['path_stretch_ratio']:.3f}")
    print(f"A* path length: {metadata['astar_path_length_m']:.2f} m")


if __name__ == "__main__":
    main()
