#!/usr/bin/env python3
"""Generate separate train, validation and test world sets plus CSV manifests."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from world_tools import generate_boxes, read_yaml, write_metadata, write_world


RESULT_FIELDS = (
    "run_id",
    "timestamp",
    "split",
    "world_file",
    "seed",
    "difficulty",
    "policy_name",
    "success",
    "collision",
    "timeout",
    "duration_s",
    "path_length_m",
    "minimum_scan_m",
    "mean_linear_speed_mps",
    "mean_abs_angular_speed_rps",
    "safety_interventions",
    "notes",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--train", type=int, default=100)
    parser.add_argument("--validation", type=int, default=20)
    parser.add_argument("--test", type=int, default=40)
    parser.add_argument("--train-seed-base", type=int, default=0)
    parser.add_argument("--validation-seed-base", type=int, default=100000)
    parser.add_argument("--test-seed-base", type=int, default=200000)
    parser.add_argument("--force", action="store_true", help="allow overwriting an existing manifest/results template")
    args = parser.parse_args()

    config = read_yaml(args.config)
    args.output_root.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_root / "manifest.csv"
    results_path = args.output_root / "evaluation_results.csv"
    if not args.force and (manifest_path.exists() or results_path.exists()):
        raise SystemExit(
            "dataset manifest/results already exist; choose another output root or add --force"
        )
    manifest_rows: list[dict[str, object]] = []
    split_specs = (
        ("train", args.train, args.train_seed_base),
        ("validation", args.validation, args.validation_seed_base),
        ("test", args.test, args.test_seed_base),
    )
    difficulties = ("easy", "medium", "hard")

    for split, amount, seed_base in split_specs:
        split_dir = args.output_root / split
        split_dir.mkdir(parents=True, exist_ok=True)
        for index in range(amount):
            seed = seed_base + index
            difficulty = difficulties[index % len(difficulties)]
            boxes, metadata = generate_boxes(config, seed, difficulty)
            world_path = split_dir / f"world_{difficulty}_seed_{seed:06d}.yaml"
            write_world(world_path, boxes)
            metadata["world_file"] = str(world_path.resolve())
            write_metadata(world_path.with_suffix(".meta.json"), metadata)
            manifest_rows.append(
                {
                    "split": split,
                    "world_file": str(world_path.resolve()),
                    "seed": seed,
                    "difficulty": difficulty,
                    "random_obstacle_count": metadata["random_obstacle_count"],
                    "boundary_wall_count": metadata["boundary_wall_count"],
                    "total_box_count": metadata["total_box_count"],
                    "direct_path_blocked": metadata["direct_path_blocked"],
                    "astar_path_length_m": metadata["astar_path_length_m"],
                    "start_x": config["start"]["x"],
                    "start_y": config["start"]["y"],
                    "start_yaw": config["start"]["yaw"],
                    "goal_x": config["goal"]["x"],
                    "goal_y": config["goal"]["y"],
                }
            )

    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(manifest_rows[0].keys()))
        writer.writeheader()
        writer.writerows(manifest_rows)

    with results_path.open("w", newline="", encoding="utf-8") as stream:
        csv.DictWriter(stream, fieldnames=RESULT_FIELDS).writeheader()

    dataset_info = {
        "config": str(args.config.resolve()),
        "train": args.train,
        "validation": args.validation,
        "test": args.test,
        "seed_bases": {
            "train": args.train_seed_base,
            "validation": args.validation_seed_base,
            "test": args.test_seed_base,
        },
    }
    with (args.output_root / "dataset_info.json").open("w", encoding="utf-8") as stream:
        json.dump(dataset_info, stream, indent=2)
        stream.write("\n")

    print(f"Generated {len(manifest_rows)} valid worlds in {args.output_root}")
    print(f"Manifest: {manifest_path}")
    print(f"Results template: {results_path}")


if __name__ == "__main__":
    main()
