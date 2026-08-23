"""Tests for obstacle discovery using LiDAR and odometry only."""

import math
import unittest

from leo_heuristic_navigation.lidar_mapping import (
    LidarOccupancyMap,
    raster_line,
)


class LidarMappingTests(unittest.TestCase):
    """Verify that no obstacles exist until a scan actually observes them."""

    @staticmethod
    def _mapper():
        return LidarOccupancyMap(
            x_min=-3.0,
            x_max=3.0,
            y_min=-3.0,
            y_max=3.0,
            resolution=0.1,
            inflation_radius=0.2,
            lidar_offset_x=0.1,
            lidar_offset_y=0.0,
            maximum_mapping_range=2.0,
            ray_stride=1,
        )

    def test_map_starts_without_obstacles(self):
        mapper = self._mapper()
        self.assertEqual(mapper.obstacle_hits, set())
        self.assertEqual(mapper.grid.occupied, set())

    def test_lidar_hit_uses_sensor_offset_and_robot_pose(self):
        mapper = self._mapper()
        update = mapper.integrate_scan(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            ranges=[1.0],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.05,
            range_max=12.0,
        )
        hit = mapper.grid.world_to_cell((1.1, 0.0))
        inflated = mapper.grid.world_to_cell((1.2, 0.0))
        self.assertEqual(update.new_obstacle_cells, 1)
        self.assertIn(hit, mapper.obstacle_hits)
        self.assertIn(inflated, mapper.grid.occupied)

    def test_robot_yaw_rotates_lidar_hit_into_world_frame(self):
        mapper = self._mapper()
        mapper.integrate_scan(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=math.pi / 2.0,
            ranges=[1.0],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.05,
            range_max=12.0,
        )
        self.assertIn(
            mapper.grid.world_to_cell((0.0, 1.1)),
            mapper.obstacle_hits,
        )

    def test_infinite_range_discovers_free_space_only(self):
        mapper = self._mapper()
        update = mapper.integrate_scan(
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            ranges=[math.inf],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.05,
            range_max=12.0,
        )
        self.assertEqual(update.new_obstacle_cells, 0)
        self.assertEqual(mapper.obstacle_hits, set())
        self.assertGreater(len(mapper.observed_free), 0)

    def test_bresenham_line_includes_both_endpoints(self):
        cells = raster_line((1, 1), (4, 3))
        self.assertEqual(cells[0], (1, 1))
        self.assertEqual(cells[-1], (4, 3))


if __name__ == '__main__':
    unittest.main()
