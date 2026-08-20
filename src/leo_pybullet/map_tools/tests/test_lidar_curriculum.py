#!/usr/bin/env python3

from __future__ import annotations

import copy
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from world_tools import generate_boxes, read_yaml, validate_boxes  # noqa: E402


class LidarCurriculumTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = read_yaml(
            ROOT / "config" / "policy_lidar_curriculum_10x10.yaml"
        )

    def active_config(self, metadata: dict) -> dict:
        config = copy.deepcopy(self.config)
        config["start"] = metadata["start"]
        config["goal"] = metadata["goal"]
        return config

    def test_direct_block_worlds_are_reproducible_and_solvable(self) -> None:
        for difficulty in ("easy", "medium", "hard"):
            for seed in range(4):
                first_boxes, first_meta = generate_boxes(
                    self.config,
                    seed=seed,
                    difficulty=difficulty,
                    scenario="direct_block",
                )
                second_boxes, second_meta = generate_boxes(
                    self.config,
                    seed=seed,
                    difficulty=difficulty,
                    scenario="direct_block",
                )
                self.assertEqual(first_boxes, second_boxes)
                self.assertEqual(first_meta, second_meta)
                self.assertTrue(first_meta["direct_path_blocked"])
                self.assertGreaterEqual(
                    first_meta["path_stretch_ratio"],
                    self.config["difficulties"][difficulty][
                        "minimum_path_stretch"
                    ],
                )
                errors, path = validate_boxes(
                    first_boxes,
                    self.active_config(first_meta),
                )
                self.assertEqual(errors, [])
                self.assertIsNotNone(path)

                start = first_meta["start"]
                goal = first_meta["goal"]
                goal_heading = math.atan2(
                    goal["y"] - start["y"],
                    goal["x"] - start["x"],
                )
                heading_error = math.atan2(
                    math.sin(start["yaw"] - goal_heading),
                    math.cos(start["yaw"] - goal_heading),
                )
                self.assertLessEqual(
                    abs(math.degrees(heading_error)),
                    5.0001,
                )

    def test_clear_worlds_keep_the_direct_route_open(self) -> None:
        for difficulty in ("easy", "medium", "hard"):
            boxes, metadata = generate_boxes(
                self.config,
                seed=100 + len(difficulty),
                difficulty=difficulty,
                scenario="clear",
            )
            self.assertFalse(metadata["direct_path_blocked"])
            errors, path = validate_boxes(
                boxes,
                self.active_config(metadata),
            )
            self.assertEqual(errors, [])
            self.assertIsNotNone(path)


if __name__ == "__main__":
    unittest.main()
