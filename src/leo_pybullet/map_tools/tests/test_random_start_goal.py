#!/usr/bin/env python3

from __future__ import annotations

import copy
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from world_tools import generate_boxes, read_yaml  # noqa: E402


class RandomStartGoalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = read_yaml(ROOT / "config" / "map_generation.yaml")

    def test_random_start_goal_is_reproducible_and_varied(self) -> None:
        config = copy.deepcopy(self.config)
        config["start_goal_randomization"] = {
            "enabled": True,
            "minimum_distance": 1.5,
            "maximum_distance": 6.0,
            "boundary_clearance": 0.65,
            "randomize_start_yaw": True,
            "max_sampling_attempts": 1000,
        }
        first_boxes, first_meta = generate_boxes(
            config, seed=21, difficulty="medium"
        )
        second_boxes, second_meta = generate_boxes(
            config, seed=21, difficulty="medium"
        )
        _, other_meta = generate_boxes(config, seed=22, difficulty="medium")

        self.assertEqual(first_boxes, second_boxes)
        self.assertEqual(first_meta, second_meta)
        self.assertNotEqual(first_meta["start"], other_meta["start"])
        self.assertNotEqual(first_meta["goal"], other_meta["goal"])

        distance = math.hypot(
            first_meta["goal"]["x"] - first_meta["start"]["x"],
            first_meta["goal"]["y"] - first_meta["start"]["y"],
        )
        self.assertGreaterEqual(distance, 1.5)
        self.assertLessEqual(distance, 6.0)
        self.assertGreaterEqual(first_meta["start"]["yaw"], -math.pi)
        self.assertLessEqual(first_meta["start"]["yaw"], math.pi)


if __name__ == "__main__":
    unittest.main()
