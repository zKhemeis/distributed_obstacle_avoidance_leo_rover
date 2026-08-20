#!/usr/bin/env python3

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from world_tools import generate_boxes, read_yaml, validate_boxes  # noqa: E402


class LidarHardCurriculumTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = read_yaml(
            ROOT / "config" / "policy_lidar_hard_10x10.yaml"
        )

    def active_config(self, metadata: dict) -> dict:
        config = copy.deepcopy(self.config)
        config["start"] = metadata["start"]
        config["goal"] = metadata["goal"]
        return config

    def test_double_block_worlds_are_reproducible_and_solvable(self) -> None:
        for difficulty in ("easy", "medium", "hard"):
            for seed in range(3):
                first_boxes, first_meta = generate_boxes(
                    self.config,
                    seed=300000 + seed,
                    difficulty=difficulty,
                    scenario="double_block",
                )
                second_boxes, second_meta = generate_boxes(
                    self.config,
                    seed=300000 + seed,
                    difficulty=difficulty,
                    scenario="double_block",
                )

                self.assertEqual(first_boxes, second_boxes)
                self.assertEqual(first_meta, second_meta)
                self.assertEqual(first_meta["structured_blocker_count"], 2)
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

    def test_clear_worlds_remain_solvable_and_direct(self) -> None:
        for difficulty in ("easy", "medium", "hard"):
            boxes, metadata = generate_boxes(
                self.config,
                seed=400000 + len(difficulty),
                difficulty=difficulty,
                scenario="clear",
            )
            self.assertFalse(metadata["direct_path_blocked"])
            self.assertEqual(metadata["structured_blocker_count"], 0)
            errors, path = validate_boxes(
                boxes,
                self.active_config(metadata),
            )
            self.assertEqual(errors, [])
            self.assertIsNotNone(path)


if __name__ == "__main__":
    unittest.main()
