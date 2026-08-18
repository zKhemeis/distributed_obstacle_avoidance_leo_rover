#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from world_tools import Box, generate_boxes, read_yaml, validate_boxes  # noqa: E402


class WorldToolsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = read_yaml(ROOT / "config" / "map_generation.yaml")

    def test_generation_is_reproducible(self) -> None:
        first, first_meta = generate_boxes(self.config, seed=42, difficulty="medium")
        second, second_meta = generate_boxes(self.config, seed=42, difficulty="medium")
        self.assertEqual(first, second)
        self.assertEqual(first_meta, second_meta)

    def test_generated_world_is_valid(self) -> None:
        boxes, _ = generate_boxes(self.config, seed=7, difficulty="hard")
        errors, path = validate_boxes(boxes, self.config)
        self.assertEqual(errors, [])
        self.assertIsNotNone(path)
        for box in boxes:
            self.assertAlmostEqual(box.z, box.size_z / 2.0, places=6)

    def test_overlap_is_rejected(self) -> None:
        a = Box("a", 2.0, 1.0, 0.25, 0.5, 0.5, 0.5)
        b = Box("b", 2.1, 1.0, 0.25, 0.5, 0.5, 0.5)
        errors, _ = validate_boxes([a, b], self.config)
        self.assertTrue(any("overlaps" in error for error in errors))


if __name__ == "__main__":
    unittest.main()

