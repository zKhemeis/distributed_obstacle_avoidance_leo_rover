"""Build an obstacle map exclusively from LiDAR and robot odometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from leo_heuristic_navigation.grid_map import Cell, GridMap, build_grid_map


def raster_line(start: Cell, end: Cell) -> list[Cell]:
    """Return the integer cells traversed by one Bresenham ray."""
    x, y = start
    end_x, end_y = end
    delta_x = abs(end_x - x)
    delta_y = abs(end_y - y)
    step_x = 1 if x < end_x else -1
    step_y = 1 if y < end_y else -1
    error = delta_x - delta_y
    cells: list[Cell] = []

    while True:
        cells.append((x, y))
        if x == end_x and y == end_y:
            return cells
        doubled = 2 * error
        if doubled > -delta_y:
            error -= delta_y
            x += step_x
        if doubled < delta_x:
            error += delta_x
            y += step_y


@dataclass(frozen=True)
class MappingUpdate:
    """Report what one LiDAR integration added to the discovered map."""

    new_obstacle_cells: int
    total_obstacle_cells: int
    total_observed_free_cells: int


class LidarOccupancyMap:
    """Maintain an initially unknown map without reading obstacle geometry."""

    def __init__(
        self,
        *,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        resolution: float,
        inflation_radius: float,
        lidar_offset_x: float = 0.10,
        lidar_offset_y: float = 0.0,
        maximum_mapping_range: float = 5.0,
        ray_stride: int = 2,
    ) -> None:
        if maximum_mapping_range <= 0.0:
            raise ValueError('Maximum mapping range must be positive')
        if ray_stride <= 0:
            raise ValueError('LiDAR ray stride must be positive')

        self.grid = build_grid_map(
            [],
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            resolution=resolution,
            inflation_radius=0.0,
        )
        self.inflation_radius = float(inflation_radius)
        self.lidar_offset_x = float(lidar_offset_x)
        self.lidar_offset_y = float(lidar_offset_y)
        self.maximum_mapping_range = float(maximum_mapping_range)
        self.ray_stride = int(ray_stride)
        self.obstacle_hits: set[Cell] = set()
        self.observed_free: set[Cell] = set()
        self.version = 0
        self._inflation_offsets = self._make_inflation_offsets()

    def _make_inflation_offsets(self) -> tuple[Cell, ...]:
        if self.inflation_radius < 0.0:
            raise ValueError('Inflation radius must be non-negative')
        maximum = int(math.ceil(
            self.inflation_radius / self.grid.resolution))
        offsets = []
        for delta_x in range(-maximum, maximum + 1):
            for delta_y in range(-maximum, maximum + 1):
                distance = math.hypot(
                    delta_x * self.grid.resolution,
                    delta_y * self.grid.resolution,
                )
                if distance <= self.inflation_radius + 1e-9:
                    offsets.append((delta_x, delta_y))
        return tuple(offsets)

    def _add_hit(self, cell: Cell) -> bool:
        if not self.grid.in_bounds(cell) or cell in self.obstacle_hits:
            return False
        self.obstacle_hits.add(cell)
        self.observed_free.discard(cell)
        for delta_x, delta_y in self._inflation_offsets:
            inflated = (cell[0] + delta_x, cell[1] + delta_y)
            if self.grid.in_bounds(inflated):
                self.grid.occupied.add(inflated)
        self.version += 1
        return True

    def integrate_scan(
        self,
        *,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> MappingUpdate:
        """Fuse scan endpoints and free rays into the persistent grid."""
        cosine = math.cos(robot_yaw)
        sine = math.sin(robot_yaw)
        lidar_x = (
            robot_x + cosine * self.lidar_offset_x -
            sine * self.lidar_offset_y
        )
        lidar_y = (
            robot_y + sine * self.lidar_offset_x +
            cosine * self.lidar_offset_y
        )
        origin = self.grid.world_to_cell((lidar_x, lidar_y))
        usable_range = min(float(range_max), self.maximum_mapping_range)
        new_hits = 0

        for index in range(0, len(ranges), self.ray_stride):
            measured = float(ranges[index])
            finite_hit = (
                math.isfinite(measured) and
                float(range_min) <= measured < float(range_max) and
                measured <= usable_range
            )
            distance = measured if finite_hit else usable_range
            angle = robot_yaw + angle_min + index * angle_increment
            endpoint = self.grid.world_to_cell((
                lidar_x + distance * math.cos(angle),
                lidar_y + distance * math.sin(angle),
            ))
            cells = raster_line(origin, endpoint)
            free_cells = cells[:-1] if finite_hit else cells
            for cell in free_cells:
                if (
                    self.grid.in_bounds(cell) and
                    cell not in self.obstacle_hits
                ):
                    self.observed_free.add(cell)
            if finite_hit and self._add_hit(endpoint):
                new_hits += 1

        robot_cell = self.grid.world_to_cell((robot_x, robot_y))
        if robot_cell not in self.obstacle_hits:
            self.grid.occupied.discard(robot_cell)
            self.observed_free.add(robot_cell)

        return MappingUpdate(
            new_obstacle_cells=new_hits,
            total_obstacle_cells=len(self.obstacle_hits),
            total_observed_free_cells=len(self.observed_free),
        )

    def path_is_blocked(
        self,
        grid_path: Sequence[Cell],
        *,
        start_index: int = 0,
    ) -> bool:
        """Check whether discovered obstacles invalidate a planned path."""
        return any(
            cell in self.grid.occupied
            for cell in grid_path[max(0, start_index):]
        )

    def occupancy_values(self) -> list[int]:
        """Serialize the map as ROS OccupancyGrid values."""
        values = [-1] * (self.grid.width * self.grid.height)
        for x, y in self.observed_free:
            values[y * self.grid.width + x] = 0
        for x, y in self.grid.occupied:
            values[y * self.grid.width + x] = 100
        return values
