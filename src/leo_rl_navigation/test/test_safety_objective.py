#!/usr/bin/env python3

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from leo_rl_navigation import LeoRoverEnv


class SafetyObjectiveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.world = root / 'world.yaml'
        self.manifest = root / 'manifest.csv'
        self.world.write_text(
            'obstacles:\n'
            '- name: distant_box\n'
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

    def test_timeout_has_terminal_penalty(self) -> None:
        environment = LeoRoverEnv(
            self.manifest,
            maximum_episode_steps=1,
            timeout_penalty=7.0,
        )
        environment.reset()
        stop = np.array([-1.0, 0.0], dtype=np.float32)
        _, _, terminated, truncated, info = environment.step(stop)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertTrue(info['timeout'])
        self.assertEqual(info['reward_timeout'], -7.0)
        environment.close()

    def test_proximity_penalizes_unsafe_forward_motion(self) -> None:
        environment = LeoRoverEnv(
            self.manifest,
            safety_distance=3.0,
            proximity_penalty_weight=0.25,
            unsafe_speed_penalty_weight=0.50,
        )
        environment.reset()
        forward = np.array([1.0, 0.0], dtype=np.float32)
        _, _, _, _, info = environment.step(forward)
        self.assertLess(info['reward_proximity'], 0.0)
        self.assertLess(info['reward_unsafe_speed'], 0.0)
        environment.close()

    def test_stuck_motion_terminates_episode(self) -> None:
        environment = LeoRoverEnv(
            self.manifest,
            maximum_episode_steps=10,
            stuck_window_steps=2,
            stuck_displacement_threshold=0.20,
            stuck_yaw_threshold=0.20,
            stuck_minimum_command_speed=0.0,
            stuck_penalty=9.0,
        )
        environment.reset()
        stop = np.array([-1.0, 0.0], dtype=np.float32)
        environment.step(stop)
        _, _, terminated, truncated, info = environment.step(stop)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info['stuck'])
        self.assertFalse(info['collision'])
        self.assertEqual(info['reward_stuck'], -9.0)
        environment.close()


if __name__ == '__main__':
    unittest.main()
