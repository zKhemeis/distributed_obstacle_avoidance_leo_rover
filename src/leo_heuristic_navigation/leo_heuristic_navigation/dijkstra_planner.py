"""Deterministic Dijkstra search on an inflated occupancy grid."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time

from leo_heuristic_navigation.grid_map import Cell, GridMap, Point


@dataclass(frozen=True)
class PlanResult:
    """Dijkstra output and planning diagnostics."""

    path: list[Point]
    grid_path: list[Cell]
    cost_m: float
    expanded_cells: int
    planning_time_s: float


def _neighbors(
    grid: GridMap,
    cell: Cell,
    allow_diagonal: bool,
) -> list[tuple[Cell, float]]:
    motions = [(-1, 0), (0, -1), (0, 1), (1, 0)]
    if allow_diagonal:
        motions.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

    neighbors: list[tuple[Cell, float]] = []
    for dx, dy in motions:
        candidate = (cell[0] + dx, cell[1] + dy)
        if not grid.is_free(candidate):
            continue
        if dx != 0 and dy != 0:
            if not grid.is_free((cell[0] + dx, cell[1])):
                continue
            if not grid.is_free((cell[0], cell[1] + dy)):
                continue
        step_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
        neighbors.append((candidate, step_cost * grid.resolution))
    return neighbors


def plan_path(
    grid: GridMap,
    start: Point,
    goal: Point,
    *,
    allow_diagonal: bool = True,
) -> PlanResult:
    """Find the lowest-cost path from start to goal using Dijkstra."""
    started = time.perf_counter()
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    if not grid.is_free(start_cell):
        raise ValueError(f'Start cell is occupied or outside: {start_cell}')
    if not grid.is_free(goal_cell):
        raise ValueError(f'Goal cell is occupied or outside: {goal_cell}')

    serial = itertools.count()
    frontier: list[tuple[float, int, Cell]] = [(0.0, next(serial), start_cell)]
    cost: dict[Cell, float] = {start_cell: 0.0}
    parent: dict[Cell, Cell] = {}
    expanded = 0

    while frontier:
        current_cost, _, current = heapq.heappop(frontier)
        if current_cost > cost[current] + 1e-12:
            continue
        expanded += 1
        if current == goal_cell:
            break
        for neighbor, edge_cost in _neighbors(
                grid, current, allow_diagonal):
            candidate_cost = current_cost + edge_cost
            if candidate_cost + 1e-12 >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = candidate_cost
            parent[neighbor] = current
            heapq.heappush(
                frontier,
                (candidate_cost, next(serial), neighbor),
            )

    if goal_cell not in cost:
        raise RuntimeError('Dijkstra could not find a collision-free path')

    grid_path = [goal_cell]
    while grid_path[-1] != start_cell:
        grid_path.append(parent[grid_path[-1]])
    grid_path.reverse()

    path = [grid.cell_to_world(cell) for cell in grid_path]
    path[0] = (float(start[0]), float(start[1]))
    path[-1] = (float(goal[0]), float(goal[1]))
    return PlanResult(
        path=path,
        grid_path=grid_path,
        cost_m=cost[goal_cell],
        expanded_cells=expanded,
        planning_time_s=time.perf_counter() - started,
    )
