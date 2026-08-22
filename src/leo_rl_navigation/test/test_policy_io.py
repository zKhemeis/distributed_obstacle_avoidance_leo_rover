#!/usr/bin/env python3

import math
import unittest

import numpy as np

from leo_rl_navigation.policy_io import (
    action_to_command,
    apply_footprint_safety,
    build_observation,
)


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

    def test_explicit_front_clearance_observation(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        ranges[:5] = 0.40
        ranges[-5:] = 0.40

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
            include_front_clearance=True,
            front_normalization_distance=0.80,
        )

        self.assertEqual(observation.shape, (54,))
        self.assertAlmostEqual(float(observation[50]), 0.50)
        self.assertAlmostEqual(
            measurements['normalized_front_clearance'],
            0.50,
        )
        self.assertAlmostEqual(
            float(observation[51]),
            1.0 / math.sqrt(200.0),
        )

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

    def test_reversible_action_contract(self) -> None:
        reverse, angular = action_to_command(
            np.array([-1.0, -0.5], dtype=np.float32),
            linear_speed_max=0.25,
            angular_speed_max=0.8,
            linear_reverse_speed_max=0.10,
        )
        self.assertAlmostEqual(reverse, -0.10)
        self.assertAlmostEqual(angular, -0.40)

        forward, _ = action_to_command(
            np.array([1.0, 0.0], dtype=np.float32),
            linear_speed_max=0.25,
            angular_speed_max=0.8,
            linear_reverse_speed_max=0.10,
        )
        self.assertAlmostEqual(forward, 0.25)

    def test_directional_escape_features_use_side_sector_medians(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        ranges[:20] = 0.45
        ranges[-20:] = 0.45
        ranges[35:120] = 3.5
        ranges[-120:-35] = 1.2

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
            include_front_clearance=True,
            include_directional_clearance=True,
            directional_normalization_distance=3.0,
            use_footprint_clearance=True,
        )

        self.assertEqual(observation.shape, (56,))
        self.assertAlmostEqual(
            float(observation[50]),
            measurements['normalized_front_clearance'],
        )
        self.assertAlmostEqual(
            float(observation[51]),
            measurements['normalized_left_clearance'],
        )
        self.assertAlmostEqual(
            float(observation[52]),
            measurements['normalized_right_clearance'],
        )
        self.assertGreater(float(observation[51]), float(observation[52]))
        self.assertAlmostEqual(
            float(observation[53]),
            1.0 / math.sqrt(200.0),
        )

    def test_directional_features_require_front_feature(self) -> None:
        with self.assertRaises(ValueError):
            build_observation(
                np.full(500, 12.0, dtype=np.float32),
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
                include_directional_clearance=True,
            )

    def test_footprint_clearance_detects_front_wheel_corner(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        angle_increment = 2.0 * math.pi / 500.0
        corner_index = int(round(math.radians(66.0) / angle_increment))
        ranges[corner_index] = 0.295

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
            front_half_angle_deg=30.0,
            include_front_clearance=True,
            front_normalization_distance=0.50,
            use_footprint_clearance=True,
            footprint_half_angle_deg=90.0,
        )

        self.assertEqual(observation.shape, (54,))
        self.assertAlmostEqual(measurements['front_minimum_scan'], 12.0)
        self.assertLess(measurements['footprint_minimum_clearance'], 0.03)
        self.assertGreater(measurements['footprint_nearest_angle_deg'], 60.0)
        self.assertLess(float(observation[50]), 0.06)

    def test_footprint_clearance_respects_sensor_front_offset(self) -> None:
        ranges = np.full(500, 12.0, dtype=np.float32)
        ranges[0] = 0.247

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
            use_footprint_clearance=True,
        )

        self.assertAlmostEqual(
            measurements['footprint_minimum_clearance'],
            0.247 - (0.215 - 0.10),
            places=5,
        )

    def test_safety_shield_stops_and_turns_toward_clearer_side(self) -> None:
        linear, angular, active = apply_footprint_safety(
            0.25,
            0.10,
            {
                'footprint_minimum_clearance': 0.02,
                'footprint_left_clearance': 0.06,
                'footprint_right_clearance': 0.80,
            },
            enabled=True,
            stop_distance=0.08,
            slowdown_distance=0.35,
            minimum_turn_speed=0.40,
            angular_speed_max=0.80,
        )

        self.assertTrue(active)
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, -0.40)

    def test_safety_shield_scales_speed_before_body_contact(self) -> None:
        linear, angular, active = apply_footprint_safety(
            0.25,
            0.20,
            {
                'footprint_minimum_clearance': 0.215,
                'footprint_left_clearance': 1.0,
                'footprint_right_clearance': 1.0,
            },
            enabled=True,
            stop_distance=0.08,
            slowdown_distance=0.35,
            minimum_turn_speed=0.40,
            angular_speed_max=0.80,
        )

        self.assertTrue(active)
        self.assertAlmostEqual(linear, 0.125)
        self.assertAlmostEqual(angular, 0.20)

    def test_disabled_safety_shield_preserves_legacy_action(self) -> None:
        linear, angular, active = apply_footprint_safety(
            0.25,
            -0.30,
            {},
            enabled=False,
            stop_distance=0.08,
            slowdown_distance=0.35,
            minimum_turn_speed=0.40,
            angular_speed_max=0.80,
        )

        self.assertFalse(active)
        self.assertAlmostEqual(linear, 0.25)
        self.assertAlmostEqual(angular, -0.30)

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
