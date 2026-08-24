"""Tests for deterministic path tracking and LiDAR emergency behavior."""

import math
import unittest

from leo_heuristic_navigation.path_controller import (
    ControllerConfig,
    PathController,
    scan_clearances,
)


class PathControllerTests(unittest.TestCase):
    """Verify bounded commands for representative situations."""

    def setUp(self):
        self.config = ControllerConfig()
        self.path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
        self.clear_scan = [5.0] * 361

    def _command(self, controller, yaw=0.0, ranges=None):
        return controller.command(
            x=0.0,
            y=0.0,
            yaw=yaw,
            ranges=self.clear_scan if ranges is None else ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 360.0,
            range_max=12.0,
        )

    def test_clear_aligned_path_drives_forward(self):
        command = self._command(PathController(self.path, self.config))
        self.assertGreater(command.linear, 0.0)
        self.assertAlmostEqual(command.angular, 0.0)
        self.assertFalse(command.emergency)

    def test_large_heading_error_pivots(self):
        command = self._command(
            PathController(self.path, self.config),
            yaw=math.pi / 2.0,
        )
        self.assertAlmostEqual(command.linear, 0.0)
        self.assertLess(command.angular, 0.0)

    def test_close_front_obstacle_triggers_emergency_turn(self):
        ranges = list(self.clear_scan)
        ranges[180] = 0.15
        command = self._command(
            PathController(self.path, self.config),
            ranges=ranges,
        )
        self.assertAlmostEqual(command.linear, 0.0)
        self.assertNotEqual(command.angular, 0.0)
        self.assertTrue(command.emergency)

    def test_emergency_tie_turns_toward_planned_path(self):
        path = [(0.0, 0.0), (0.5, -0.5), (1.0, -1.0)]
        ranges = list(self.clear_scan)
        ranges[180] = 0.15
        command = self._command(
            PathController(path, self.config),
            ranges=ranges,
        )
        self.assertLess(command.angular, 0.0)
        self.assertTrue(command.emergency)

    def test_scan_clearance_uses_forward_angle(self):
        ranges = list(self.clear_scan)
        ranges[180] = 0.3
        front, _, _ = scan_clearances(
            ranges,
            angle_min=-math.pi,
            angle_increment=2.0 * math.pi / 360.0,
            range_max=12.0,
            front_half_angle_deg=45.0,
        )
        self.assertAlmostEqual(front, 0.3)


if __name__ == '__main__':
    unittest.main()
