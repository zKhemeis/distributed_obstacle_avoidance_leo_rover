"""Gymnasium environment backed directly by the C++ Bullet simulation core."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from leo_bullet_sim import (
    BulletSim,
    CONTROL_PHYSICS_STEPS,
    LIDAR_RANGE_MAX,
    LIDAR_RANGE_MIN,
    NUMBER_OF_RAYS,
    PHYSICS_RATE,
)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LeoRoverEnv(gym.Env):
    """Goal navigation with LiDAR and a continuous velocity action."""

    metadata = {'render_modes': []}

    def __init__(
        self,
        manifest_path: str | Path,
        split: str = 'train',
        n_sectors: int = 50,
        linear_speed_max: float = 0.25,
        angular_speed_max: float = 0.80,
        maximum_goal_distance: float = math.sqrt(200.0),
        goal_tolerance: float = 0.25,
        maximum_episode_steps: int = 400,
        progress_weight: float = 10.0,
        time_penalty: float = 0.01,
        goal_reward: float = 20.0,
        collision_penalty: float = 20.0,
        settle_physics_steps: int = CONTROL_PHYSICS_STEPS,
        sequential_worlds: bool = False,
    ) -> None:
        super().__init__()

        if NUMBER_OF_RAYS % n_sectors != 0:
            raise ValueError(
                f'{NUMBER_OF_RAYS} LiDAR rays must be divisible by '
                f'n_sectors={n_sectors}')
        if linear_speed_max <= 0.0 or angular_speed_max <= 0.0:
            raise ValueError('Speed limits must be positive')
        if maximum_goal_distance <= 0.0:
            raise ValueError('maximum_goal_distance must be positive')
        if maximum_episode_steps <= 0:
            raise ValueError('maximum_episode_steps must be positive')

        self.manifest_path = Path(manifest_path).expanduser().resolve()
        self.split = split
        self.n_sectors = int(n_sectors)
        self.linear_speed_max = float(linear_speed_max)
        self.angular_speed_max = float(angular_speed_max)
        self.maximum_goal_distance = float(maximum_goal_distance)
        self.goal_tolerance = float(goal_tolerance)
        self.maximum_episode_steps = int(maximum_episode_steps)
        self.progress_weight = float(progress_weight)
        self.time_penalty = float(time_penalty)
        self.goal_reward = float(goal_reward)
        self.collision_penalty = float(collision_penalty)
        self.settle_physics_steps = int(settle_physics_steps)
        self.sequential_worlds = bool(sequential_worlds)

        self._worlds = self._read_manifest()
        self._simulation = BulletSim()
        self._goal = np.zeros(2, dtype=np.float64)
        self._previous_distance = 0.0
        self._episode_steps = 0
        self._world_file = ''
        self._manifest_index = -1
        self._next_world_index = 0

        observation_low = np.concatenate((
            np.zeros(self.n_sectors + 1, dtype=np.float32),
            np.array([-1.0, -1.0], dtype=np.float32),
        ))
        observation_high = np.concatenate((
            np.ones(self.n_sectors + 1, dtype=np.float32),
            np.array([1.0, 1.0], dtype=np.float32),
        ))
        self.observation_space = spaces.Box(
            low=observation_low,
            high=observation_high,
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

    def _read_manifest(self) -> list[dict[str, str]]:
        if not self.manifest_path.is_file():
            raise FileNotFoundError(
                f'Manifest does not exist: {self.manifest_path}')

        required = {
            'split',
            'world_file',
            'start_x',
            'start_y',
            'start_yaw',
            'goal_x',
            'goal_y',
        }
        with self.manifest_path.open(
                'r', newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            missing = required.difference(reader.fieldnames or ())
            if missing:
                raise ValueError(
                    f'Manifest is missing columns: {sorted(missing)}')
            rows = [row for row in reader if row['split'] == self.split]

        if not rows:
            raise ValueError(
                f'Manifest contains no rows for split={self.split!r}')
        return rows

    def _resolve_world_file(self, row: dict[str, str]) -> Path:
        configured = Path(row['world_file']).expanduser()
        if configured.is_file():
            return configured.resolve()

        portable = self.manifest_path.parent / self.split / configured.name
        if portable.is_file():
            return portable.resolve()

        raise FileNotFoundError(
            f'World from manifest does not exist: {configured}; '
            f'portable candidate: {portable}')

    def _select_world(self, options: dict[str, Any]) -> tuple[int, dict[str, str]]:
        if 'world_index' in options:
            index = int(options['world_index'])
            if index < 0 or index >= len(self._worlds):
                raise IndexError(
                    f'world_index={index} outside [0, {len(self._worlds)})')
        elif self.sequential_worlds:
            index = self._next_world_index
            self._next_world_index = (
                self._next_world_index + 1) % len(self._worlds)
        else:
            index = int(self.np_random.integers(len(self._worlds)))
        return index, self._worlds[index]

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        options = {} if options is None else dict(options)
        index, row = self._select_world(options)
        world_file = self._resolve_world_file(row)

        start_x = float(options.get('start_x', row['start_x']))
        start_y = float(options.get('start_y', row['start_y']))
        start_yaw = float(options.get('start_yaw', row['start_yaw']))
        goal_x = float(options.get('goal_x', row['goal_x']))
        goal_y = float(options.get('goal_y', row['goal_y']))

        self._simulation.reset(
            str(world_file), start_x, start_y, start_yaw)
        self._simulation.set_command(0.0, 0.0)
        if self.settle_physics_steps > 0:
            self._simulation.step(self.settle_physics_steps)

        self._goal[:] = (goal_x, goal_y)
        self._episode_steps = 0
        self._world_file = str(world_file)
        self._manifest_index = index

        observation, measurements = self._observation_and_measurements()
        self._previous_distance = measurements['distance_to_goal']
        info = self._build_info(
            measurements,
            success=False,
            collision=self._simulation.has_collision(),
            timeout=False,
            reward_terms=None,
        )
        return observation, info

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action_array = np.asarray(action, dtype=np.float32)
        if action_array.shape != (2,):
            raise ValueError(f'Action must have shape (2,), got {action_array.shape}')
        if not np.isfinite(action_array).all():
            raise ValueError('Action contains NaN or infinity')
        action_array = np.clip(
            action_array, self.action_space.low, self.action_space.high)

        linear_velocity = (
            (float(action_array[0]) + 1.0) * 0.5 * self.linear_speed_max)
        angular_velocity = float(action_array[1]) * self.angular_speed_max
        self._simulation.set_command(linear_velocity, angular_velocity)
        self._simulation.step(CONTROL_PHYSICS_STEPS)
        self._episode_steps += 1

        observation, measurements = self._observation_and_measurements()
        distance = measurements['distance_to_goal']
        collision = bool(self._simulation.has_collision())
        success = bool(distance <= self.goal_tolerance and not collision)
        terminated = collision or success
        truncated = (
            self._episode_steps >= self.maximum_episode_steps and
            not terminated
        )

        reward_progress = self.progress_weight * (
            self._previous_distance - distance)
        reward_time = -self.time_penalty
        reward_goal = self.goal_reward if success else 0.0
        reward_collision = -self.collision_penalty if collision else 0.0
        reward = (
            reward_progress + reward_time + reward_goal + reward_collision)
        self._previous_distance = distance

        if terminated or truncated:
            self._simulation.set_command(0.0, 0.0)

        reward_terms = {
            'reward_progress': float(reward_progress),
            'reward_time': float(reward_time),
            'reward_goal': float(reward_goal),
            'reward_collision': float(reward_collision),
        }
        info = self._build_info(
            measurements,
            success=success,
            collision=collision,
            timeout=truncated,
            reward_terms=reward_terms,
        )
        return observation, float(reward), terminated, truncated, info

    def _observation_and_measurements(
        self,
    ) -> tuple[np.ndarray, dict[str, float]]:
        state = self._simulation.robot_state()
        scan = self._simulation.laser_scan()
        ranges = np.asarray(scan.ranges, dtype=np.float32)

        if ranges.shape != (NUMBER_OF_RAYS,):
            raise RuntimeError(
                f'Expected {NUMBER_OF_RAYS} rays, received {ranges.shape}')
        if np.isnan(ranges).any() or np.isneginf(ranges).any():
            raise RuntimeError('LiDAR contains NaN or negative infinity')

        ranges = ranges.copy()
        ranges[np.isposinf(ranges)] = LIDAR_RANGE_MAX
        ranges = np.clip(ranges, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX)
        sector_ranges = ranges.reshape(self.n_sectors, -1).min(axis=1)
        normalized_ranges = sector_ranges / LIDAR_RANGE_MAX

        delta_x = float(self._goal[0] - state.transform.x)
        delta_y = float(self._goal[1] - state.transform.y)
        distance = math.hypot(delta_x, delta_y)
        goal_angle = math.atan2(delta_y, delta_x)
        relative_bearing = _wrap_angle(goal_angle - state.yaw)
        normalized_distance = min(distance / self.maximum_goal_distance, 1.0)

        observation = np.concatenate((
            normalized_ranges,
            np.array([
                normalized_distance,
                math.sin(relative_bearing),
                math.cos(relative_bearing),
            ], dtype=np.float32),
        )).astype(np.float32, copy=False)

        measurements = {
            'distance_to_goal': float(distance),
            'relative_bearing': float(relative_bearing),
            'minimum_scan': float(ranges.min()),
            'pose_x': float(state.transform.x),
            'pose_y': float(state.transform.y),
            'pose_yaw': float(state.yaw),
        }
        return observation, measurements

    def _build_info(
        self,
        measurements: dict[str, float],
        *,
        success: bool,
        collision: bool,
        timeout: bool,
        reward_terms: dict[str, float] | None,
    ) -> dict[str, Any]:
        info: dict[str, Any] = {
            'is_success': bool(success),
            'collision': bool(collision),
            'timeout': bool(timeout),
            'episode_step': self._episode_steps,
            'simulation_time': float(self._simulation.simulation_time),
            'world_file': self._world_file,
            'world_index': self._manifest_index,
            'goal_x': float(self._goal[0]),
            'goal_y': float(self._goal[1]),
        }
        info.update(measurements)
        if reward_terms is not None:
            info.update(reward_terms)
        return info

    @property
    def world_count(self) -> int:
        return len(self._worlds)

    @property
    def control_period(self) -> float:
        return CONTROL_PHYSICS_STEPS / PHYSICS_RATE

    def close(self) -> None:
        self._simulation.set_command(0.0, 0.0)
