"""Deterministic path tracker with a small LiDAR emergency behavior."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


Point = tuple[float, float]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def scan_clearances(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_max: float,
    front_half_angle_deg: float,
) -> tuple[float, float, float]:
    """Return minimum front, left-front and right-front scan distances."""
    half_angle = math.radians(front_half_angle_deg)
    front: list[float] = []
    left: list[float] = []
    right: list[float] = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance):
            distance = range_max
        distance = _clamp(distance, 0.0, range_max)
        angle = wrap_angle(angle_min + index * angle_increment)
        if abs(angle) <= half_angle:
            front.append(distance)
            if angle >= 0.0:
                left.append(distance)
            if angle <= 0.0:
                right.append(distance)
    return (
        min(front, default=range_max),
        min(left, default=range_max),
        min(right, default=range_max),
    )


@dataclass(frozen=True)
class ControllerConfig:
    """Path-tracking and LiDAR emergency parameters."""

    linear_speed_max: float = 0.25
    angular_speed_max: float = 0.80
    heading_gain: float = 1.8
    pivot_heading_threshold: float = 0.55
    lookahead_distance: float = 0.45
    waypoint_tolerance: float = 0.20
    lidar_front_half_angle_deg: float = 45.0
    lidar_emergency_stop_distance: float = 0.20
    lidar_slowdown_distance: float = 0.50


@dataclass(frozen=True)
class CommandResult:
    """Controller command and diagnostic state."""

    linear: float
    angular: float
    target_index: int
    heading_error: float
    front_clearance: float
    emergency: bool


class PathController:
    """Follow a dense grid path using a deterministic lookahead target."""

    def __init__(
        self,
        path: Sequence[Point],
        config: ControllerConfig,
    ) -> None:
        if not path:
            raise ValueError('Path must contain at least one point')
        self.path = [(float(x), float(y)) for x, y in path]
        self.config = config
        self.progress_index = 0

    def _target_index(self, x: float, y: float) -> int:
        search_start = max(0, self.progress_index - 3)
        nearest = min(
            range(search_start, len(self.path)),
            key=lambda index: math.hypot(
                self.path[index][0] - x,
                self.path[index][1] - y,
            ),
        )
        self.progress_index = max(self.progress_index, nearest)
        target = self.progress_index
        while target < len(self.path) - 1:
            distance = math.hypot(
                self.path[target][0] - x,
                self.path[target][1] - y,
            )
            if distance >= self.config.lookahead_distance:
                break
            target += 1
        return target

    def command(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_max: float,
    ) -> CommandResult:
        """Calculate one bounded velocity command."""
        target_index = self._target_index(x, y)
        target_x, target_y = self.path[target_index]
        desired_heading = math.atan2(target_y - y, target_x - x)
        error = wrap_angle(desired_heading - yaw)
        front, left, right = scan_clearances(
            ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_max=range_max,
            front_half_angle_deg=self.config.lidar_front_half_angle_deg,
        )

        if front <= self.config.lidar_emergency_stop_distance:
            if left > right + 1e-6:
                turn_sign = 1.0
            elif right > left + 1e-6:
                turn_sign = -1.0
            else:
                turn_sign = 1.0 if error >= 0.0 else -1.0
            return CommandResult(
                linear=0.0,
                angular=turn_sign * self.config.angular_speed_max,
                target_index=target_index,
                heading_error=error,
                front_clearance=front,
                emergency=True,
            )

        angular = _clamp(
            self.config.heading_gain * error,
            -self.config.angular_speed_max,
            self.config.angular_speed_max,
        )
        if abs(error) >= self.config.pivot_heading_threshold:
            linear = 0.0
        else:
            heading_scale = max(
                0.25,
                1.0 - abs(error) / self.config.pivot_heading_threshold,
            )
            linear = self.config.linear_speed_max * heading_scale
            if front < self.config.lidar_slowdown_distance:
                interval = (
                    self.config.lidar_slowdown_distance -
                    self.config.lidar_emergency_stop_distance
                )
                clearance_scale = (
                    (front - self.config.lidar_emergency_stop_distance) /
                    max(interval, 1e-9)
                )
                linear *= _clamp(clearance_scale, 0.0, 1.0)

        return CommandResult(
            linear=linear,
            angular=angular,
            target_index=target_index,
            heading_error=error,
            front_clearance=front,
            emergency=False,
        )
