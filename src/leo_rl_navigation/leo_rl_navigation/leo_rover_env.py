"""Gymnasium environment backed directly by the C++ Bullet simulation core."""

from __future__ import annotations

import csv
from collections import deque
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
from leo_rl_navigation.policy_io import (
    DISCRETE_MANEUVER_TABLE,
    action_to_command,
    apply_footprint_safety,
    build_observation,
)


class LeoRoverEnv(gym.Env):
    """Goal navigation with continuous or PPO-selected maneuver actions."""

    metadata = {'render_modes': []}

    def __init__(
        self,
        manifest_path: str | Path,
        split: str = 'train',
        n_sectors: int = 50,
        linear_speed_max: float = 0.25,
        angular_speed_max: float = 0.80,
        linear_reverse_speed_max: float = 0.0,
        action_mode: str = 'continuous',
        maximum_goal_distance: float = math.sqrt(200.0),
        goal_tolerance: float = 0.25,
        maximum_episode_steps: int = 400,
        progress_weight: float = 10.0,
        time_penalty: float = 0.01,
        goal_reward: float = 20.0,
        collision_penalty: float = 20.0,
        timeout_penalty: float = 0.0,
        safety_distance: float = 0.0,
        proximity_penalty_weight: float = 0.0,
        unsafe_speed_penalty_weight: float = 0.0,
        front_half_angle_deg: float = 30.0,
        include_front_clearance: bool = False,
        front_normalization_distance: float = 0.80,
        include_directional_clearance: bool = False,
        directional_normalization_distance: float = 3.0,
        directional_inner_angle_deg: float = 25.0,
        directional_outer_angle_deg: float = 85.0,
        front_safety_distance: float = 0.0,
        front_unsafe_speed_penalty_weight: float = 0.0,
        escape_turn_reward_weight: float = 0.0,
        clearance_progress_reward_weight: float = 0.0,
        escape_reverse_reward_weight: float = 0.0,
        reverse_penalty_weight: float = 0.0,
        use_footprint_clearance: bool = False,
        footprint_half_length: float = 0.215,
        footprint_half_width: float = 0.259,
        lidar_offset_x: float = 0.10,
        lidar_offset_y: float = 0.0,
        footprint_half_angle_deg: float = 90.0,
        enable_safety_shield: bool = False,
        safety_stop_distance: float = 0.08,
        safety_slowdown_distance: float = 0.35,
        safety_minimum_turn_speed: float = 0.40,
        safety_intervention_penalty_weight: float = 0.0,
        stuck_window_steps: int = 0,
        stuck_displacement_threshold: float = 0.05,
        stuck_yaw_threshold: float = 0.10,
        stuck_minimum_command_speed: float = 0.05,
        stuck_penalty: float = 0.0,
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
        if action_mode not in {'continuous', 'discrete_primitives'}:
            raise ValueError(f'Unknown PPO action mode: {action_mode}')
        if maximum_goal_distance <= 0.0:
            raise ValueError('maximum_goal_distance must be positive')
        if maximum_episode_steps <= 0:
            raise ValueError('maximum_episode_steps must be positive')
        nonnegative_values = {
            'timeout_penalty': timeout_penalty,
            'linear_reverse_speed_max': linear_reverse_speed_max,
            'safety_distance': safety_distance,
            'proximity_penalty_weight': proximity_penalty_weight,
            'unsafe_speed_penalty_weight': unsafe_speed_penalty_weight,
            'front_safety_distance': front_safety_distance,
            'front_normalization_distance': front_normalization_distance,
            'directional_normalization_distance': (
                directional_normalization_distance),
            'front_unsafe_speed_penalty_weight': (
                front_unsafe_speed_penalty_weight),
            'escape_turn_reward_weight': escape_turn_reward_weight,
            'clearance_progress_reward_weight': (
                clearance_progress_reward_weight),
            'escape_reverse_reward_weight': escape_reverse_reward_weight,
            'reverse_penalty_weight': reverse_penalty_weight,
            'footprint_half_length': footprint_half_length,
            'footprint_half_width': footprint_half_width,
            'safety_stop_distance': safety_stop_distance,
            'safety_slowdown_distance': safety_slowdown_distance,
            'safety_minimum_turn_speed': safety_minimum_turn_speed,
            'safety_intervention_penalty_weight': (
                safety_intervention_penalty_weight),
            'stuck_displacement_threshold': stuck_displacement_threshold,
            'stuck_yaw_threshold': stuck_yaw_threshold,
            'stuck_minimum_command_speed': stuck_minimum_command_speed,
            'stuck_penalty': stuck_penalty,
        }
        for name, value in nonnegative_values.items():
            if value < 0.0:
                raise ValueError(f'{name} must be non-negative')
        if stuck_window_steps < 0:
            raise ValueError('stuck_window_steps must be non-negative')
        if not 0.0 < front_half_angle_deg < 180.0:
            raise ValueError('front_half_angle_deg must be in (0, 180)')

        self.manifest_path = Path(manifest_path).expanduser().resolve()
        self.split = split
        self.n_sectors = int(n_sectors)
        self.linear_speed_max = float(linear_speed_max)
        self.angular_speed_max = float(angular_speed_max)
        self.linear_reverse_speed_max = float(linear_reverse_speed_max)
        self.action_mode = str(action_mode)
        self.maximum_goal_distance = float(maximum_goal_distance)
        self.goal_tolerance = float(goal_tolerance)
        self.maximum_episode_steps = int(maximum_episode_steps)
        self.progress_weight = float(progress_weight)
        self.time_penalty = float(time_penalty)
        self.goal_reward = float(goal_reward)
        self.collision_penalty = float(collision_penalty)
        self.timeout_penalty = float(timeout_penalty)
        self.safety_distance = float(safety_distance)
        self.proximity_penalty_weight = float(proximity_penalty_weight)
        self.unsafe_speed_penalty_weight = float(
            unsafe_speed_penalty_weight)
        self.front_half_angle_deg = float(front_half_angle_deg)
        self.include_front_clearance = bool(include_front_clearance)
        self.front_normalization_distance = float(
            front_normalization_distance)
        self.include_directional_clearance = bool(
            include_directional_clearance)
        self.directional_normalization_distance = float(
            directional_normalization_distance)
        self.directional_inner_angle_deg = float(
            directional_inner_angle_deg)
        self.directional_outer_angle_deg = float(
            directional_outer_angle_deg)
        self.front_safety_distance = float(front_safety_distance)
        self.front_unsafe_speed_penalty_weight = float(
            front_unsafe_speed_penalty_weight)
        self.escape_turn_reward_weight = float(escape_turn_reward_weight)
        self.clearance_progress_reward_weight = float(
            clearance_progress_reward_weight)
        self.escape_reverse_reward_weight = float(
            escape_reverse_reward_weight)
        self.reverse_penalty_weight = float(reverse_penalty_weight)
        self.use_footprint_clearance = bool(use_footprint_clearance)
        self.footprint_half_length = float(footprint_half_length)
        self.footprint_half_width = float(footprint_half_width)
        self.lidar_offset_x = float(lidar_offset_x)
        self.lidar_offset_y = float(lidar_offset_y)
        self.footprint_half_angle_deg = float(footprint_half_angle_deg)
        self.enable_safety_shield = bool(enable_safety_shield)
        self.safety_stop_distance = float(safety_stop_distance)
        self.safety_slowdown_distance = float(safety_slowdown_distance)
        self.safety_minimum_turn_speed = float(safety_minimum_turn_speed)
        self.safety_intervention_penalty_weight = float(
            safety_intervention_penalty_weight)
        if (
            self.enable_safety_shield and
            self.safety_slowdown_distance <= self.safety_stop_distance
        ):
            raise ValueError(
                'Safety slowdown distance must exceed stop distance')
        self.stuck_window_steps = int(stuck_window_steps)
        self.stuck_displacement_threshold = float(
            stuck_displacement_threshold)
        self.stuck_yaw_threshold = float(stuck_yaw_threshold)
        self.stuck_minimum_command_speed = float(
            stuck_minimum_command_speed)
        self.stuck_penalty = float(stuck_penalty)
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
        self._latest_measurements: dict[str, float] = {}
        self._pose_history: deque[tuple[float, float, float]] = deque(
            maxlen=max(self.stuck_window_steps + 1, 1))

        nonnegative_features = self.n_sectors + 1
        if self.include_front_clearance:
            nonnegative_features += 1
        if self.include_directional_clearance:
            nonnegative_features += 2
        observation_low = np.concatenate((
            np.zeros(nonnegative_features, dtype=np.float32),
            np.array([-1.0, -1.0], dtype=np.float32),
        ))
        observation_high = np.concatenate((
            np.ones(nonnegative_features, dtype=np.float32),
            np.array([1.0, 1.0], dtype=np.float32),
        ))
        self.observation_space = spaces.Box(
            low=observation_low,
            high=observation_high,
            dtype=np.float32,
        )
        if self.action_mode == 'discrete_primitives':
            self.action_space = spaces.Discrete(len(DISCRETE_MANEUVER_TABLE))
        else:
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
        self._latest_measurements = dict(measurements)
        self._previous_distance = measurements['distance_to_goal']
        self._pose_history.clear()
        self._pose_history.append((
            measurements['pose_x'],
            measurements['pose_y'],
            measurements['pose_yaw'],
        ))
        info = self._build_info(
            measurements,
            success=False,
            collision=self._simulation.has_collision(),
            timeout=False,
            stuck=False,
            reward_terms=None,
        )
        return observation, info

    def step(
        self,
        action: np.ndarray | int,
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        previous_measurements = dict(self._latest_measurements)
        requested_linear, requested_angular = action_to_command(
            action,
            linear_speed_max=self.linear_speed_max,
            angular_speed_max=self.angular_speed_max,
            linear_reverse_speed_max=self.linear_reverse_speed_max,
            action_mode=self.action_mode,
        )
        linear_velocity, angular_velocity, shield_active = (
            apply_footprint_safety(
                requested_linear,
                requested_angular,
                self._latest_measurements,
                enabled=self.enable_safety_shield,
                stop_distance=self.safety_stop_distance,
                slowdown_distance=self.safety_slowdown_distance,
                minimum_turn_speed=self.safety_minimum_turn_speed,
                angular_speed_max=self.angular_speed_max,
            )
        )
        self._simulation.set_command(linear_velocity, angular_velocity)
        self._simulation.step(CONTROL_PHYSICS_STEPS)
        self._episode_steps += 1

        observation, measurements = self._observation_and_measurements()
        self._latest_measurements = dict(measurements)
        measurements['command_linear'] = float(linear_velocity)
        measurements['command_angular'] = float(angular_velocity)
        measurements['requested_command_linear'] = float(requested_linear)
        measurements['requested_command_angular'] = float(requested_angular)
        measurements['safety_shield_active'] = bool(shield_active)
        distance = measurements['distance_to_goal']
        collision = bool(self._simulation.has_collision())
        success = bool(distance <= self.goal_tolerance and not collision)
        self._pose_history.append((
            measurements['pose_x'],
            measurements['pose_y'],
            measurements['pose_yaw'],
        ))
        stuck = self._is_stuck(linear_velocity) and not collision and not success
        terminated = collision or success or stuck
        truncated = (
            self._episode_steps >= self.maximum_episode_steps and
            not terminated
        )

        reward_progress = self.progress_weight * (
            self._previous_distance - distance)
        reward_time = -self.time_penalty
        reward_goal = self.goal_reward if success else 0.0
        reward_collision = -self.collision_penalty if collision else 0.0
        reward_timeout = -self.timeout_penalty if truncated else 0.0
        reward_stuck = -self.stuck_penalty if stuck else 0.0
        clearance_ratio = 0.0
        if self.safety_distance > 0.0:
            clearance_ratio = max(
                0.0,
                (self.safety_distance - measurements['minimum_scan']) /
                self.safety_distance,
            )
        normalized_linear_speed = (
            max(requested_linear, 0.0) / self.linear_speed_max)
        reward_proximity = (
            -self.proximity_penalty_weight * clearance_ratio ** 2)
        reward_unsafe_speed = (
            -self.unsafe_speed_penalty_weight *
            normalized_linear_speed * clearance_ratio)
        front_clearance_ratio = 0.0
        if self.front_safety_distance > 0.0:
            front_clearance = (
                measurements['footprint_minimum_clearance']
                if self.use_footprint_clearance
                else measurements['front_minimum_scan']
            )
            front_clearance_ratio = max(
                0.0,
                (self.front_safety_distance - front_clearance) /
                self.front_safety_distance,
            )
        reward_front_unsafe_speed = (
            -self.front_unsafe_speed_penalty_weight *
            normalized_linear_speed * front_clearance_ratio)
        previous_front_clearance = (
            previous_measurements['footprint_minimum_clearance']
            if self.use_footprint_clearance
            else previous_measurements['front_minimum_scan']
        )
        previous_front_blocked_ratio = 0.0
        if self.front_safety_distance > 0.0:
            previous_front_blocked_ratio = float(np.clip(
                (self.front_safety_distance - previous_front_clearance) /
                self.front_safety_distance,
                0.0,
                1.0,
            ))
        directional_advantage = float(np.clip(
            (
                previous_measurements['footprint_left_escape_clearance'] -
                previous_measurements['footprint_right_escape_clearance']
            ) / self.directional_normalization_distance,
            -1.0,
            1.0,
        ))
        reward_escape_turn = (
            self.escape_turn_reward_weight *
            previous_front_blocked_ratio *
            directional_advantage *
            requested_angular / self.angular_speed_max
        )
        current_front_clearance = (
            measurements['footprint_minimum_clearance']
            if self.use_footprint_clearance
            else measurements['front_minimum_scan']
        )
        clearance_progress = float(np.clip(
            (current_front_clearance - previous_front_clearance) /
            max(self.front_safety_distance, 1e-9),
            -1.0,
            1.0,
        ))
        reward_clearance_progress = (
            self.clearance_progress_reward_weight *
            previous_front_blocked_ratio *
            clearance_progress
        )
        reverse_ratio = 0.0
        if self.linear_reverse_speed_max > 0.0:
            reverse_ratio = (
                max(-requested_linear, 0.0) /
                self.linear_reverse_speed_max
            )
        reward_escape_reverse = (
            self.escape_reverse_reward_weight *
            previous_front_blocked_ratio *
            reverse_ratio
        )
        reward_reverse = (
            -self.reverse_penalty_weight *
            (1.0 - previous_front_blocked_ratio) *
            reverse_ratio
        )
        reward_safety_intervention = (
            -self.safety_intervention_penalty_weight *
            max(requested_linear - linear_velocity, 0.0) /
            self.linear_speed_max
        )
        reward = (
            reward_progress + reward_time + reward_goal + reward_collision +
            reward_timeout + reward_stuck + reward_proximity +
            reward_unsafe_speed + reward_front_unsafe_speed +
            reward_escape_turn + reward_clearance_progress +
            reward_escape_reverse + reward_reverse +
            reward_safety_intervention)
        self._previous_distance = distance

        if terminated or truncated:
            self._simulation.set_command(0.0, 0.0)

        reward_terms = {
            'reward_progress': float(reward_progress),
            'reward_time': float(reward_time),
            'reward_goal': float(reward_goal),
            'reward_collision': float(reward_collision),
            'reward_timeout': float(reward_timeout),
            'reward_stuck': float(reward_stuck),
            'reward_proximity': float(reward_proximity),
            'reward_unsafe_speed': float(reward_unsafe_speed),
            'reward_front_unsafe_speed': float(
                reward_front_unsafe_speed),
            'reward_escape_turn': float(reward_escape_turn),
            'reward_clearance_progress': float(reward_clearance_progress),
            'reward_escape_reverse': float(reward_escape_reverse),
            'reward_reverse': float(reward_reverse),
            'reward_safety_intervention': float(
                reward_safety_intervention),
        }
        info = self._build_info(
            measurements,
            success=success,
            collision=collision,
            timeout=truncated,
            stuck=stuck,
            reward_terms=reward_terms,
        )
        return observation, float(reward), terminated, truncated, info

    def _is_stuck(self, linear_velocity: float) -> bool:
        if self.stuck_window_steps <= 0:
            return False
        if len(self._pose_history) < self.stuck_window_steps + 1:
            return False
        if abs(linear_velocity) < self.stuck_minimum_command_speed:
            return False

        old_x, old_y, old_yaw = self._pose_history[0]
        new_x, new_y, new_yaw = self._pose_history[-1]
        displacement = math.hypot(new_x - old_x, new_y - old_y)
        yaw_change = abs(math.atan2(
            math.sin(new_yaw - old_yaw),
            math.cos(new_yaw - old_yaw),
        ))
        return (
            displacement < self.stuck_displacement_threshold and
            yaw_change < self.stuck_yaw_threshold
        )

    def _observation_and_measurements(
        self,
    ) -> tuple[np.ndarray, dict[str, float]]:
        state = self._simulation.robot_state()
        scan = self._simulation.laser_scan()
        return build_observation(
            scan.ranges,
            pose_x=state.transform.x,
            pose_y=state.transform.y,
            pose_yaw=state.yaw,
            goal_x=float(self._goal[0]),
            goal_y=float(self._goal[1]),
            number_of_rays=NUMBER_OF_RAYS,
            n_sectors=self.n_sectors,
            range_min=LIDAR_RANGE_MIN,
            range_max=LIDAR_RANGE_MAX,
            maximum_goal_distance=self.maximum_goal_distance,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            front_half_angle_deg=self.front_half_angle_deg,
            include_front_clearance=self.include_front_clearance,
            front_normalization_distance=(
                self.front_normalization_distance),
            include_directional_clearance=(
                self.include_directional_clearance),
            directional_normalization_distance=(
                self.directional_normalization_distance),
            directional_inner_angle_deg=self.directional_inner_angle_deg,
            directional_outer_angle_deg=self.directional_outer_angle_deg,
            use_footprint_clearance=self.use_footprint_clearance,
            footprint_half_length=self.footprint_half_length,
            footprint_half_width=self.footprint_half_width,
            lidar_offset_x=self.lidar_offset_x,
            lidar_offset_y=self.lidar_offset_y,
            footprint_half_angle_deg=self.footprint_half_angle_deg,
        )

    def _build_info(
        self,
        measurements: dict[str, float],
        *,
        success: bool,
        collision: bool,
        timeout: bool,
        stuck: bool,
        reward_terms: dict[str, float] | None,
    ) -> dict[str, Any]:
        info: dict[str, Any] = {
            'is_success': bool(success),
            'collision': bool(collision),
            'timeout': bool(timeout),
            'stuck': bool(stuck),
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
