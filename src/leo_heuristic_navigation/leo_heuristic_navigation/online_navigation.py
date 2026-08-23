"""Repeated Dijkstra planning over obstacles discovered by LiDAR."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from leo_heuristic_navigation.dijkstra_planner import PlanResult, plan_path
from leo_heuristic_navigation.lidar_mapping import (
    LidarOccupancyMap,
    MappingUpdate,
)
from leo_heuristic_navigation.path_controller import (
    CommandResult,
    ControllerConfig,
    PathController,
)


@dataclass(frozen=True)
class NavigationUpdate:
    """Return the current command and online-planning diagnostics."""

    command: CommandResult
    plan_changed: bool
    map_changed: bool
    new_obstacle_cells: int


class OnlineDijkstraNavigator:
    """Drive using only known goal, robot odometry, and LiDAR observations."""

    def __init__(
        self,
        *,
        planner: dict[str, Any],
        mapping: dict[str, Any],
        controller: dict[str, Any],
        goal_x: float,
        goal_y: float,
    ) -> None:
        self.goal = (float(goal_x), float(goal_y))
        self.allow_diagonal = bool(planner['allow_diagonal'])
        self.update_interval_steps = int(mapping['update_interval_steps'])
        self.periodic_replan_steps = int(mapping['periodic_replan_steps'])
        self.emergency_replan_steps = int(
            controller['emergency_replan_steps'])
        if self.update_interval_steps <= 0:
            raise ValueError('Map update interval must be positive')
        if self.periodic_replan_steps <= 0:
            raise ValueError('Periodic replan interval must be positive')
        if self.emergency_replan_steps <= 0:
            raise ValueError('Emergency replan threshold must be positive')

        self.mapper = LidarOccupancyMap(
            x_min=float(planner['arena_x_min']),
            x_max=float(planner['arena_x_max']),
            y_min=float(planner['arena_y_min']),
            y_max=float(planner['arena_y_max']),
            resolution=float(planner['grid_resolution']),
            inflation_radius=float(planner['obstacle_inflation_radius']),
            lidar_offset_x=float(mapping['lidar_offset_x']),
            lidar_offset_y=float(mapping['lidar_offset_y']),
            maximum_mapping_range=float(mapping['maximum_mapping_range']),
            ray_stride=int(mapping['ray_stride']),
        )
        self.controller_config = ControllerConfig(
            linear_speed_max=float(controller['linear_speed_max']),
            angular_speed_max=float(controller['angular_speed_max']),
            heading_gain=float(controller['heading_gain']),
            pivot_heading_threshold=float(
                controller['pivot_heading_threshold']),
            lookahead_distance=float(controller['lookahead_distance']),
            waypoint_tolerance=float(controller['waypoint_tolerance']),
            lidar_front_half_angle_deg=float(
                controller['lidar_front_half_angle_deg']),
            lidar_emergency_stop_distance=float(
                controller['lidar_emergency_stop_distance']),
            lidar_slowdown_distance=float(
                controller['lidar_slowdown_distance']),
        )
        self.plan: PlanResult | None = None
        self.controller: PathController | None = None
        self.initial_plan_cost_m = 0.0
        self.total_planning_time_s = 0.0
        self.total_expanded_cells = 0
        self.replans = 0
        self.steps = 0
        self._last_planning_step = 0
        self._last_planning_map_version = 0
        self._consecutive_emergency_steps = 0

    def _make_plan(self, x: float, y: float) -> None:
        if self.plan is not None:
            self.replans += 1
        self.plan = plan_path(
            self.mapper.grid,
            (x, y),
            self.goal,
            allow_diagonal=self.allow_diagonal,
        )
        if self.replans == 0:
            self.initial_plan_cost_m = self.plan.cost_m
        self.total_planning_time_s += self.plan.planning_time_s
        self.total_expanded_cells += self.plan.expanded_cells
        self.controller = PathController(
            self.plan.path,
            self.controller_config,
        )
        self._last_planning_step = self.steps
        self._last_planning_map_version = self.mapper.version

    def _path_needs_replanning(self) -> bool:
        if self.plan is None or self.controller is None:
            return True
        if self.mapper.path_is_blocked(
                self.plan.grid_path,
                start_index=self.controller.progress_index):
            return True
        map_changed = self.mapper.version != self._last_planning_map_version
        overdue = (
            self.steps - self._last_planning_step >=
            self.periodic_replan_steps
        )
        return map_changed and overdue

    def update(
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
    ) -> NavigationUpdate:
        """Integrate a scan, replan if necessary, and return one command."""
        mapping_update = MappingUpdate(
            new_obstacle_cells=0,
            total_obstacle_cells=len(self.mapper.obstacle_hits),
            total_observed_free_cells=len(self.mapper.observed_free),
        )
        map_updated = (
            self.plan is None or
            self.steps % self.update_interval_steps == 0
        )
        if map_updated:
            mapping_update = self.mapper.integrate_scan(
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
            )

        plan_changed = self._path_needs_replanning()
        if plan_changed:
            self._make_plan(robot_x, robot_y)

        assert self.controller is not None
        command = self.controller.command(
            x=robot_x,
            y=robot_y,
            yaw=robot_yaw,
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_max=range_max,
        )
        if command.emergency:
            self._consecutive_emergency_steps += 1
        else:
            self._consecutive_emergency_steps = 0
        if (
            self._consecutive_emergency_steps >=
            self.emergency_replan_steps
        ):
            self._make_plan(robot_x, robot_y)
            assert self.controller is not None
            command = self.controller.command(
                x=robot_x,
                y=robot_y,
                yaw=robot_yaw,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                range_max=range_max,
            )
            self._consecutive_emergency_steps = int(command.emergency)
            plan_changed = True

        self.steps += 1
        return NavigationUpdate(
            command=command,
            plan_changed=plan_changed,
            map_changed=map_updated,
            new_obstacle_cells=mapping_update.new_obstacle_cells,
        )
