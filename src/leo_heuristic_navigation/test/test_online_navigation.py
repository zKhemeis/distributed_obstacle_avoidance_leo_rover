"""Tests for online Dijkstra replanning from newly observed obstacles."""

import math
import unittest

from leo_heuristic_navigation.online_navigation import (
    OnlineDijkstraNavigator,
)


class OnlineNavigationTests(unittest.TestCase):
    """Verify that planning uses discoveries rather than hidden map data."""

    @staticmethod
    def _navigator():
        planner = {
            'arena_x_min': 0.0,
            'arena_x_max': 5.0,
            'arena_y_min': 0.0,
            'arena_y_max': 5.0,
            'grid_resolution': 0.1,
            'obstacle_inflation_radius': 0.2,
            'allow_diagonal': True,
        }
        mapping = {
            'lidar_offset_x': 0.1,
            'lidar_offset_y': 0.0,
            'maximum_mapping_range': 3.0,
            'ray_stride': 1,
            'update_interval_steps': 1,
            'periodic_replan_steps': 20,
        }
        controller = {
            'linear_speed_max': 0.25,
            'angular_speed_max': 0.8,
            'heading_gain': 1.8,
            'pivot_heading_threshold': 0.55,
            'lookahead_distance': 0.4,
            'waypoint_tolerance': 0.2,
            'lidar_front_half_angle_deg': 45.0,
            'lidar_emergency_stop_distance': 0.2,
            'lidar_slowdown_distance': 0.5,
            'emergency_replan_steps': 5,
        }
        return OnlineDijkstraNavigator(
            planner=planner,
            mapping=mapping,
            controller=controller,
            goal_x=4.5,
            goal_y=2.5,
        )

    @staticmethod
    def _update(navigator, front_range):
        ranges = [math.inf] * 8
        ranges[0] = front_range
        return navigator.update(
            robot_x=0.5,
            robot_y=2.5,
            robot_yaw=0.0,
            ranges=ranges,
            angle_min=0.0,
            angle_increment=2.0 * math.pi / len(ranges),
            range_min=0.05,
            range_max=12.0,
        )

    def test_constructor_starts_with_zero_known_obstacles(self):
        navigator = self._navigator()
        self.assertEqual(len(navigator.mapper.obstacle_hits), 0)
        self.assertIsNone(navigator.plan)

    def test_new_obstacle_blocks_path_and_triggers_replan(self):
        navigator = self._navigator()
        initial = self._update(navigator, math.inf)
        self.assertTrue(initial.plan_changed)
        self.assertEqual(navigator.replans, 0)

        discovered = self._update(navigator, 1.0)
        self.assertTrue(discovered.plan_changed)
        self.assertGreater(discovered.new_obstacle_cells, 0)
        self.assertEqual(navigator.replans, 1)
        assert navigator.plan is not None
        self.assertTrue(any(
            abs(y - 2.5) > 0.05
            for _, y in navigator.plan.path
        ))


if __name__ == '__main__':
    unittest.main()
