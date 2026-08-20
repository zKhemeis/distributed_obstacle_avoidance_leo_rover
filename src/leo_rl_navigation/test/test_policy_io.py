#!/usr/bin/env python3

import math
import unittest

import numpy as np

from leo_rl_navigation.policy_io import action_to_command, build_observation


class PolicyIoTests(unittest.TestCase):
    def test_observation_contract(self) -> None:
        ranges = np.full(500, np.inf, dtype=np.float32)
        ranges[0:10] = 3.0
        ranges[10:20] = np.linspace(2.0, 4.0, 10)

        observation, measurements = build_observation(
            ranges,
            pose_x=1.0,
            pose_y=2.0,
            pose_yaw=math.pi / 2.0,
            goal_x=4.0,
            goal_y=6.0,
            number_of_rays=500,
            n_sectors=50,
            range_min=0.05,
            range_max=12.0,
            maximum_goal_distance=math.sqrt(200.0),
        )

        self.assertEqual(observation.shape, (53,))
        self.assertEqual(observation.dtype, np.float32)
        self.assertTrue(np.isfinite(observation).all())
        self.assertAlmostEqual(float(observation[0]), 3.0 / 12.0)
        self.assertAlmostEqual(float(observation[1]), 2.0 / 12.0)
        self.assertAlmostEqual(float(observation[2]), 1.0)
        self.assertAlmostEqual(measurements['distance_to_goal'], 5.0)
        self.assertAlmostEqual(measurements['front_minimum_scan'], 2.0)
        self.assertAlmostEqual(float(observation[50]), 5.0 / math.sqrt(200.0))

    def test_scan_is_canonicalized_to_forward_angle(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        ranges[250] = 0.40

        observation, measurements = build_observation(
            ranges,
            pose_x=0.0,
            pose_y=0.0,
            pose_yaw=0.0,
            goal_x=1.0,
            goal_y=0.0,
            number_of_rays=500,
            n_sectors=50,
            range_min=0.05,
            range_max=12.0,
            maximum_goal_distance=math.sqrt(200.0),
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 500.0,
        )

        self.assertAlmostEqual(float(observation[0]), 0.40 / 12.0)
        self.assertAlmostEqual(measurements['front_minimum_scan'], 0.40)

    def test_front_cone_wraps_across_scan_ends(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        ranges[-1] = 0.30

        _, measurements = build_observation(
            ranges,
            pose_x=0.0,
            pose_y=0.0,
            pose_yaw=0.0,
            goal_x=1.0,
            goal_y=0.0,
            number_of_rays=500,
            n_sectors=50,
            range_min=0.05,
            range_max=12.0,
            maximum_goal_distance=math.sqrt(200.0),
            angle_min=0.0,
            angle_increment=2.0 * math.pi / 500.0,
        )

        self.assertAlmostEqual(measurements['front_minimum_scan'], 0.30)

    def test_action_contract(self) -> None:
        linear, angular = action_to_command(
            np.array([-1.0, -1.0], dtype=np.float32),
            linear_speed_max=0.25,
            angular_speed_max=0.8,
        )
        self.assertAlmostEqual(linear, 0.0)
        self.assertAlmostEqual(angular, -0.8)

        linear, angular = action_to_command(
            np.array([1.0, 1.0], dtype=np.float32),
            linear_speed_max=0.25,
            angular_speed_max=0.8,
        )
        self.assertAlmostEqual(linear, 0.25)
        self.assertAlmostEqual(angular, 0.8)

        linear, angular = action_to_command(
            np.array([0.0, 0.0], dtype=np.float32),
            linear_speed_max=0.25,
            angular_speed_max=0.8,
        )
        self.assertAlmostEqual(linear, 0.125)
        self.assertAlmostEqual(angular, 0.0)

    def test_invalid_lidar_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            build_observation(
                np.zeros(499, dtype=np.float32),
                pose_x=0.0,
                pose_y=0.0,
                pose_yaw=0.0,
                goal_x=1.0,
                goal_y=0.0,
                number_of_rays=500,
                n_sectors=50,
                range_min=0.05,
                range_max=12.0,
                maximum_goal_distance=math.sqrt(200.0),
            )


if __name__ == '__main__':
    unittest.main()
