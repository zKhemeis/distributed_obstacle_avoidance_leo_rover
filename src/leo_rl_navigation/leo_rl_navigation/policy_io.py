"""Shared observation and action transformations for training and deployment."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _footprint_measurements(
    canonical_ranges: np.ndarray,
    *,
    angle_increment: float,
    range_max: float,
    footprint_half_length: float,
    footprint_half_width: float,
    lidar_offset_x: float,
    lidar_offset_y: float,
    footprint_half_angle_deg: float,
    directional_inner_angle_deg: float,
    directional_outer_angle_deg: float,
) -> dict[str, float]:
    """Measure obstacle clearance from the rectangular wheel footprint."""
    if footprint_half_length <= 0.0 or footprint_half_width <= 0.0:
        raise ValueError('Footprint half dimensions must be positive')
    if abs(lidar_offset_x) >= footprint_half_length:
        raise ValueError('LiDAR x offset must lie inside the footprint')
    if abs(lidar_offset_y) >= footprint_half_width:
        raise ValueError('LiDAR y offset must lie inside the footprint')
    if not 0.0 < footprint_half_angle_deg <= 180.0:
        raise ValueError('Footprint half angle must be in (0, 180]')
    if not 0.0 <= directional_inner_angle_deg < directional_outer_angle_deg <= 180.0:
        raise ValueError(
            'Directional escape angles must satisfy 0 <= inner < outer <= 180')

    angles = np.arange(canonical_ranges.size, dtype=np.float64)
    angles *= angle_increment
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    cosine = np.cos(angles)
    sine = np.sin(angles)

    x_boundary = np.where(
        cosine >= 0.0,
        footprint_half_length - lidar_offset_x,
        footprint_half_length + lidar_offset_x,
    )
    y_boundary = np.where(
        sine >= 0.0,
        footprint_half_width - lidar_offset_y,
        footprint_half_width + lidar_offset_y,
    )
    x_distance = np.divide(
        x_boundary,
        np.abs(cosine),
        out=np.full_like(cosine, np.inf),
        where=np.abs(cosine) > 1e-12,
    )
    y_distance = np.divide(
        y_boundary,
        np.abs(sine),
        out=np.full_like(sine, np.inf),
        where=np.abs(sine) > 1e-12,
    )
    body_boundary = np.minimum(x_distance, y_distance)
    clearances = np.maximum(
        canonical_ranges.astype(np.float64) - body_boundary,
        0.0,
    )
    forward = (
        np.abs(angles) <= math.radians(footprint_half_angle_deg) + 1e-12)
    nearest_index = int(np.flatnonzero(forward)[
        np.argmin(clearances[forward])])
    left = forward & (angles > 1e-12)
    right = forward & (angles < -1e-12)
    inner_angle = math.radians(directional_inner_angle_deg)
    outer_angle = math.radians(directional_outer_angle_deg)
    escape_left = (angles >= inner_angle) & (angles <= outer_angle)
    escape_right = (angles <= -inner_angle) & (angles >= -outer_angle)

    return {
        'footprint_minimum_clearance': float(clearances[nearest_index]),
        'footprint_nearest_angle_deg': float(
            math.degrees(angles[nearest_index])),
        'footprint_left_clearance': float(
            clearances[left].min() if np.any(left) else range_max),
        'footprint_right_clearance': float(
            clearances[right].min() if np.any(right) else range_max),
        # A minimum on either side is dominated by the same head-on obstacle.
        # A side-sector median instead describes the space available to escape.
        'footprint_left_escape_clearance': float(
            np.median(clearances[escape_left])
            if np.any(escape_left) else range_max),
        'footprint_right_escape_clearance': float(
            np.median(clearances[escape_right])
            if np.any(escape_right) else range_max),
    }


def build_observation(
    ranges: Any,
    *,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    goal_x: float,
    goal_y: float,
    number_of_rays: int,
    n_sectors: int,
    range_min: float,
    range_max: float,
    maximum_goal_distance: float,
    angle_min: float = 0.0,
    angle_increment: float | None = None,
    front_half_angle_deg: float = 30.0,
    include_front_clearance: bool = False,
    front_normalization_distance: float = 0.80,
    include_directional_clearance: bool = False,
    directional_normalization_distance: float = 3.0,
    directional_inner_angle_deg: float = 25.0,
    directional_outer_angle_deg: float = 85.0,
    use_footprint_clearance: bool = False,
    footprint_half_length: float = 0.215,
    footprint_half_width: float = 0.259,
    lidar_offset_x: float = 0.10,
    lidar_offset_y: float = 0.0,
    footprint_half_angle_deg: float = 90.0,
) -> tuple[np.ndarray, dict[str, float]]:
    """Create the policy observation used by both Gym and ROS."""
    if number_of_rays <= 0 or n_sectors <= 0:
        raise ValueError('number_of_rays and n_sectors must be positive')
    if number_of_rays % n_sectors != 0:
        raise ValueError(
            f'{number_of_rays} rays must be divisible by {n_sectors} sectors')
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError('LiDAR range limits are invalid')
    if maximum_goal_distance <= 0.0:
        raise ValueError('maximum_goal_distance must be positive')
    if not 0.0 < front_half_angle_deg < 180.0:
        raise ValueError('front_half_angle_deg must be in (0, 180)')
    if front_normalization_distance <= 0.0:
        raise ValueError('front_normalization_distance must be positive')
    if directional_normalization_distance <= 0.0:
        raise ValueError('directional_normalization_distance must be positive')
    if include_directional_clearance and not include_front_clearance:
        raise ValueError(
            'Directional clearance observations require explicit front clearance')

    if angle_increment is None:
        angle_increment = 2.0 * math.pi / number_of_rays
    if not math.isfinite(angle_min):
        raise ValueError('LiDAR angle_min must be finite')
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError('LiDAR angle_increment must be finite and positive')

    range_array = np.asarray(ranges, dtype=np.float32)
    if range_array.shape != (number_of_rays,):
        raise ValueError(
            f'Expected {number_of_rays} rays, received {range_array.shape}')
    if np.isnan(range_array).any() or np.isneginf(range_array).any():
        raise ValueError('LiDAR contains NaN or negative infinity')

    range_array = range_array.copy()
    range_array[np.isposinf(range_array)] = range_max
    range_array = np.clip(range_array, range_min, range_max)

    # Canonicalize every scanner so index zero points forward. Bullet scans
    # start at zero radians, while many real LaserScan messages start at -pi.
    front_index = int(round(-angle_min / angle_increment)) % number_of_rays
    canonical_ranges = np.roll(range_array, -front_index)
    sector_ranges = canonical_ranges.reshape(n_sectors, -1).min(axis=1)
    normalized_ranges = sector_ranges / range_max

    front_half_rays = max(
        1,
        int(math.ceil(
            math.radians(front_half_angle_deg) / angle_increment)),
    )
    front_half_rays = min(front_half_rays, number_of_rays // 2)
    if front_half_rays == number_of_rays // 2:
        front_ranges = canonical_ranges
    else:
        front_ranges = np.concatenate((
            canonical_ranges[:front_half_rays + 1],
            canonical_ranges[-front_half_rays:],
        ))

    delta_x = float(goal_x - pose_x)
    delta_y = float(goal_y - pose_y)
    distance = math.hypot(delta_x, delta_y)
    goal_angle = math.atan2(delta_y, delta_x)
    relative_bearing = wrap_angle(goal_angle - pose_yaw)
    normalized_distance = min(distance / maximum_goal_distance, 1.0)

    front_minimum_scan = float(front_ranges.min())
    footprint = _footprint_measurements(
        canonical_ranges,
        angle_increment=angle_increment,
        range_max=range_max,
        footprint_half_length=footprint_half_length,
        footprint_half_width=footprint_half_width,
        lidar_offset_x=lidar_offset_x,
        lidar_offset_y=lidar_offset_y,
        footprint_half_angle_deg=footprint_half_angle_deg,
        directional_inner_angle_deg=directional_inner_angle_deg,
        directional_outer_angle_deg=directional_outer_angle_deg,
    )
    observed_clearance = (
        footprint['footprint_minimum_clearance']
        if use_footprint_clearance else front_minimum_scan
    )
    normalized_front_clearance = min(
        observed_clearance / front_normalization_distance,
        1.0,
    )
    normalized_left_clearance = min(
        footprint['footprint_left_escape_clearance'] /
        directional_normalization_distance,
        1.0,
    )
    normalized_right_clearance = min(
        footprint['footprint_right_escape_clearance'] /
        directional_normalization_distance,
        1.0,
    )
    navigation_features = [
        normalized_distance,
        math.sin(relative_bearing),
        math.cos(relative_bearing),
    ]
    if include_front_clearance:
        navigation_features.insert(0, normalized_front_clearance)
    if include_directional_clearance:
        navigation_features[1:1] = [
            normalized_left_clearance,
            normalized_right_clearance,
        ]

    observation = np.concatenate((
        normalized_ranges,
        np.asarray(navigation_features, dtype=np.float32),
    )).astype(np.float32, copy=False)

    measurements = {
        'distance_to_goal': float(distance),
        'relative_bearing': float(relative_bearing),
        'minimum_scan': float(canonical_ranges.min()),
        'front_minimum_scan': front_minimum_scan,
        'normalized_front_clearance': float(
            normalized_front_clearance),
        'normalized_left_clearance': float(normalized_left_clearance),
        'normalized_right_clearance': float(normalized_right_clearance),
        'pose_x': float(pose_x),
        'pose_y': float(pose_y),
        'pose_yaw': float(pose_yaw),
    }
    measurements.update(footprint)
    return observation, measurements


def action_to_command(
    action: Any,
    *,
    linear_speed_max: float,
    angular_speed_max: float,
    linear_reverse_speed_max: float = 0.0,
) -> tuple[float, float]:
    """Map the normalized policy action to a velocity command."""
    if linear_speed_max <= 0.0 or angular_speed_max <= 0.0:
        raise ValueError('Speed limits must be positive')
    if linear_reverse_speed_max < 0.0:
        raise ValueError('Reverse speed limit must be non-negative')

    action_array = np.asarray(action, dtype=np.float32)
    if action_array.shape != (2,):
        raise ValueError(f'Action must have shape (2,), got {action_array.shape}')
    if not np.isfinite(action_array).all():
        raise ValueError('Action contains NaN or infinity')
    action_array = np.clip(action_array, -1.0, 1.0)

    linear_velocity = (
        (float(action_array[0]) + 1.0) * 0.5 *
        (linear_speed_max + linear_reverse_speed_max) -
        linear_reverse_speed_max)
    angular_velocity = float(action_array[1]) * angular_speed_max
    return linear_velocity, angular_velocity


def apply_footprint_safety(
    linear_velocity: float,
    angular_velocity: float,
    measurements: dict[str, float],
    *,
    enabled: bool,
    stop_distance: float,
    slowdown_distance: float,
    minimum_turn_speed: float,
    angular_speed_max: float,
) -> tuple[float, float, bool]:
    """Slow near the body boundary and turn toward the clearer side."""
    if not enabled:
        return float(linear_velocity), float(angular_velocity), False
    if stop_distance < 0.0 or slowdown_distance <= stop_distance:
        raise ValueError('Safety distances must satisfy 0 <= stop < slowdown')
    if minimum_turn_speed < 0.0 or angular_speed_max <= 0.0:
        raise ValueError('Safety turn speeds must be valid')

    clearance = float(measurements['footprint_minimum_clearance'])
    if clearance >= slowdown_distance or linear_velocity <= 0.0:
        return float(linear_velocity), float(angular_velocity), False

    if clearance <= stop_distance:
        left = float(measurements['footprint_left_clearance'])
        right = float(measurements['footprint_right_clearance'])
        if abs(left - right) < 1e-9 and abs(angular_velocity) > 1e-9:
            direction = math.copysign(1.0, angular_velocity)
        else:
            direction = 1.0 if left >= right else -1.0
        turn_speed = min(
            angular_speed_max,
            max(abs(angular_velocity), minimum_turn_speed),
        )
        return 0.0, float(direction * turn_speed), True

    scale = (
        (clearance - stop_distance) /
        (slowdown_distance - stop_distance)
    )
    return (
        float(linear_velocity * np.clip(scale, 0.0, 1.0)),
        float(angular_velocity),
        True,
    )
