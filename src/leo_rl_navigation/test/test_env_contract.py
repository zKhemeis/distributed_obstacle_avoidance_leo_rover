#!/usr/bin/env python3

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from leo_rl_navigation import LeoRoverEnv


class EnvironmentContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.world = root / 'world.yaml'
        self.manifest = root / 'manifest.csv'
        self.world.write_text(
            'obstacles:\n'
            '- name: blocking_box\n'
            '  x: 2.0\n'
            '  y: 0.0\n'
            '  z: 0.25\n'
            '  size_x: 0.5\n'
            '  size_y: 0.5\n'
            '  size_z: 0.5\n',
            encoding='utf-8',
        )
        with self.manifest.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    'split',
                    'world_file',
                    'start_x',
                    'start_y',
                    'start_yaw',
                    'goal_x',
                    'goal_y',
                ),
            )
            writer.writeheader()
            writer.writerow({
                'split': 'train',
                'world_file': self.world,
                'start_x': 0.0,
                'start_y': 0.0,
                'start_yaw': 0.0,
                'goal_x': 5.0,
                'goal_y': 0.0,
            })

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def test_observation_contract_and_determinism(self) -> None:
        environment = LeoRoverEnv(self.manifest, split='train')
        first, first_info = environment.reset(seed=4)
        second, second_info = environment.reset(seed=4)
        self.assertEqual(first.shape, (53,))
        self.assertTrue(np.isfinite(first).all())
        self.assertTrue(environment.observation_space.contains(first))
        self.assertTrue(np.allclose(first, second))
        self.assertEqual(first_info['world_file'], second_info['world_file'])
        environment.close()

    def test_success_and_timeout_are_distinct(self) -> None:
        stop = np.array([-1.0, 0.0], dtype=np.float32)
        success_environment = LeoRoverEnv(self.manifest, split='train')
        success_environment.reset(options={'goal_x': 0.0, 'goal_y': 0.0})
        _, _, terminated, truncated, info = success_environment.step(stop)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info['is_success'])
        success_environment.close()

        timeout_environment = LeoRoverEnv(
            self.manifest,
            split='train',
            maximum_episode_steps=2,
        )
        timeout_environment.reset()
        timeout_environment.step(stop)
        _, _, terminated, truncated, info = timeout_environment.step(stop)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertTrue(info['timeout'])
        timeout_environment.close()

    def test_collision_terminates_episode(self) -> None:
        environment = LeoRoverEnv(
            self.manifest,
            split='train',
            maximum_episode_steps=100,
        )
        environment.reset()
        forward = np.array([1.0, 0.0], dtype=np.float32)
        for _ in range(100):
            _, _, terminated, truncated, info = environment.step(forward)
            if terminated or truncated:
                break
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info['collision'])
        self.assertFalse(info['is_success'])
        environment.close()


if __name__ == '__main__':
    unittest.main()
