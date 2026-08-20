"""Shared observation and action transformations for training and deployment."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


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
    normalized_front_clearance = min(
        front_minimum_scan / front_normalization_distance,
        1.0,
    )
    navigation_features = [
        normalized_distance,
        math.sin(relative_bearing),
        math.cos(relative_bearing),
    ]
    if include_front_clearance:
        navigation_features.insert(0, normalized_front_clearance)

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
        'pose_x': float(pose_x),
        'pose_y': float(pose_y),
        'pose_yaw': float(pose_yaw),
    }
    return observation, measurements


def action_to_command(
    action: Any,
    *,
    linear_speed_max: float,
    angular_speed_max: float,
) -> tuple[float, float]:
    """Map the normalized policy action to a velocity command."""
    if linear_speed_max <= 0.0 or angular_speed_max <= 0.0:
        raise ValueError('Speed limits must be positive')

    action_array = np.asarray(action, dtype=np.float32)
    if action_array.shape != (2,):
        raise ValueError(f'Action must have shape (2,), got {action_array.shape}')
    if not np.isfinite(action_array).all():
        raise ValueError('Action contains NaN or infinity')
    action_array = np.clip(action_array, -1.0, 1.0)

    linear_velocity = (
        (float(action_array[0]) + 1.0) * 0.5 * linear_speed_max)
    angular_velocity = float(action_array[1]) * angular_speed_max
    return linear_velocity, angular_velocity
