"""Tests for deterministic Dijkstra search."""

import math
import unittest

from leo_heuristic_navigation.dijkstra_planner import plan_path
from leo_heuristic_navigation.grid_map import build_grid_map


class DijkstraPlannerTests(unittest.TestCase):
    """Verify shortest paths and failure behavior."""

    @staticmethod
    def _grid():
        return build_grid_map(
            [],
            x_min=0.0,
            x_max=4.0,
            y_min=0.0,
            y_max=4.0,
            resolution=1.0,
            inflation_radius=0.0,
        )

    def test_open_diagonal_path_has_expected_cost(self):
        result = plan_path(
            self._grid(),
            (0.0, 0.0),
            (2.0, 2.0),
            allow_diagonal=True,
        )
        self.assertAlmostEqual(result.cost_m, 2.0 * math.sqrt(2.0))
        self.assertEqual(result.path[0], (0.0, 0.0))
        self.assertEqual(result.path[-1], (2.0, 2.0))

    def test_diagonal_cannot_cut_an_occupied_corner(self):
        grid = self._grid()
        grid.occupied.update({(1, 0), (0, 1)})
        with self.assertRaises(RuntimeError):
            plan_path(
                grid,
                (0.0, 0.0),
                (2.0, 2.0),
                allow_diagonal=True,
            )

    def test_occupied_start_is_rejected(self):
        grid = self._grid()
        grid.occupied.add((0, 0))
        with self.assertRaises(ValueError):
            plan_path(grid, (0.0, 0.0), (2.0, 2.0))


if __name__ == '__main__':
    unittest.main()
