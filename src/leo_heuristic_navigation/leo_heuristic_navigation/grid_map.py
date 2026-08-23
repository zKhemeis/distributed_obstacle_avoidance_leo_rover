"""Build an inflated occupancy grid from a Leo YAML world."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


Cell = tuple[int, int]
Point = tuple[float, float]


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned rectangular obstacle in world coordinates."""

    x: float
    y: float
    size_x: float
    size_y: float


@dataclass
class GridMap:
    """Regular occupancy grid with deterministic coordinate conversion."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    resolution: float
    width: int
    height: int
    occupied: set[Cell]

    def in_bounds(self, cell: Cell) -> bool:
        """Return whether a grid cell is inside the configured arena."""
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def is_free(self, cell: Cell) -> bool:
        """Return whether a cell is inside the arena and unoccupied."""
        return self.in_bounds(cell) and cell not in self.occupied

    def world_to_cell(self, point: Point) -> Cell:
        """Convert a world point to its nearest grid cell."""
        return (
            int(round((point[0] - self.x_min) / self.resolution)),
            int(round((point[1] - self.y_min) / self.resolution)),
        )

    def cell_to_world(self, cell: Cell) -> Point:
        """Convert a grid cell to world coordinates."""
        return (
            self.x_min + cell[0] * self.resolution,
            self.y_min + cell[1] * self.resolution,
        )


def load_obstacles(world_path: str | Path) -> list[BoxObstacle]:
    """Load axis-aligned boxes from a simulator YAML world."""
    path = Path(world_path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(
            data.get('obstacles'), list):
        raise ValueError(f'Invalid YAML world: {path}')

    obstacles: list[BoxObstacle] = []
    for index, item in enumerate(data['obstacles']):
        if not isinstance(item, dict):
            raise ValueError(f'obstacles[{index}] must be a mapping')
        try:
            obstacles.append(BoxObstacle(
                x=float(item['x']),
                y=float(item['y']),
                size_x=float(item['size_x']),
                size_y=float(item['size_y']),
            ))
        except KeyError as error:
            raise ValueError(
                f'obstacles[{index}] is missing {error.args[0]}') from error
    return obstacles


def build_grid_map(
    obstacles: list[BoxObstacle],
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    resolution: float,
    inflation_radius: float,
) -> GridMap:
    """Rasterize obstacles after conservative circular-radius inflation."""
    if resolution <= 0.0:
        raise ValueError('Grid resolution must be positive')
    if inflation_radius < 0.0:
        raise ValueError('Inflation radius must be non-negative')
    if x_max <= x_min or y_max <= y_min:
        raise ValueError('Arena bounds are invalid')

    width = int(math.floor((x_max - x_min) / resolution)) + 1
    height = int(math.floor((y_max - y_min) / resolution)) + 1
    occupied: set[Cell] = set()

    for obstacle in obstacles:
        left = obstacle.x - obstacle.size_x / 2.0 - inflation_radius
        right = obstacle.x + obstacle.size_x / 2.0 + inflation_radius
        bottom = obstacle.y - obstacle.size_y / 2.0 - inflation_radius
        top = obstacle.y + obstacle.size_y / 2.0 + inflation_radius

        ix_min = max(0, int(math.ceil((left - x_min) / resolution)))
        ix_max = min(
            width - 1,
            int(math.floor((right - x_min) / resolution)),
        )
        iy_min = max(0, int(math.ceil((bottom - y_min) / resolution)))
        iy_max = min(
            height - 1,
            int(math.floor((top - y_min) / resolution)),
        )
        for ix in range(ix_min, ix_max + 1):
            for iy in range(iy_min, iy_max + 1):
                occupied.add((ix, iy))

    return GridMap(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        resolution=resolution,
        width=width,
        height=height,
        occupied=occupied,
    )


def load_grid_map(
    world_path: str | Path,
    **grid_arguments: float,
) -> GridMap:
    """Load a YAML world and return its inflated occupancy grid."""
    return build_grid_map(load_obstacles(world_path), **grid_arguments)
