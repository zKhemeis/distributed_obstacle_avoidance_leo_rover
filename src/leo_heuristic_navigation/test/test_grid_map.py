"""Tests for conversion of YAML obstacles to an inflated grid."""

import unittest

from leo_heuristic_navigation.grid_map import BoxObstacle, build_grid_map


class GridMapTests(unittest.TestCase):
    """Verify deterministic occupancy and coordinate conversions."""

    def test_obstacle_and_inflation_are_occupied(self):
        grid = build_grid_map(
            [BoxObstacle(x=0.0, y=0.0, size_x=1.0, size_y=1.0)],
            x_min=-2.0,
            x_max=2.0,
            y_min=-2.0,
            y_max=2.0,
            resolution=0.1,
            inflation_radius=0.2,
        )
        self.assertFalse(grid.is_free(grid.world_to_cell((0.0, 0.0))))
        self.assertFalse(grid.is_free(grid.world_to_cell((0.65, 0.0))))
        self.assertTrue(grid.is_free(grid.world_to_cell((0.85, 0.0))))

    def test_world_cell_round_trip_is_bounded_by_half_cell(self):
        grid = build_grid_map(
            [],
            x_min=-5.0,
            x_max=5.0,
            y_min=-5.0,
            y_max=5.0,
            resolution=0.08,
            inflation_radius=0.0,
        )
        original = (1.234, -2.345)
        recovered = grid.cell_to_world(grid.world_to_cell(original))
        self.assertLessEqual(abs(recovered[0] - original[0]), 0.04)
        self.assertLessEqual(abs(recovered[1] - original[1]), 0.04)


if __name__ == '__main__':
    unittest.main()
