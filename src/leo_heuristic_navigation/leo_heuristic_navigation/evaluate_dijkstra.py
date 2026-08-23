"""Evaluate LiDAR-only Dijkstra navigation in the reusable Bullet core."""

from __future__ import annotations

import argparse
from collections import deque
import csv
import math
from pathlib import Path
from statistics import mean
from typing import Any

import yaml
from leo_bullet_sim import BulletSim, CONTROL_PHYSICS_STEPS

from leo_heuristic_navigation.online_navigation import (
    OnlineDijkstraNavigator,
)
from leo_heuristic_navigation.path_controller import (
    scan_clearances,
    wrap_angle,
)


RESULT_FIELDS = (
    'episode',
    'split',
    'world_file',
    'method_name',
    'obstacle_source',
    'initial_known_obstacle_cells',
    'success',
    'collision',
    'timeout',
    'stuck',
    'planning_failure',
    'steps',
    'duration_s',
    'path_length_m',
    'planned_path_length_m',
    'planning_time_s',
    'expanded_cells',
    'replans',
    'discovered_obstacle_cells',
    'observed_free_cells',
    'minimum_scan_m',
    'minimum_front_scan_m',
    'mean_linear_speed_mps',
    'mean_abs_angular_speed_rps',
    'pivot_steps',
    'emergency_steps',
    'final_distance_m',
)


def _load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).expanduser().resolve().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('Dijkstra configuration must be a mapping')
    for section in ('planner', 'mapping', 'controller', 'evaluation'):
        if not isinstance(data.get(section), dict):
            raise ValueError(
                f'LiDAR-only configuration is missing section: {section}')
    return data


def _manifest_rows(
    manifest_path: Path,
    split: str,
) -> list[dict[str, str]]:
    with manifest_path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {
            'split', 'world_file', 'start_x', 'start_y', 'start_yaw',
            'goal_x', 'goal_y',
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f'Manifest is missing columns: {sorted(missing)}')
        return [row for row in reader if row['split'] == split]


def _resolve_world(
    manifest_path: Path,
    split: str,
    configured: str,
) -> Path:
    path = Path(configured).expanduser()
    if path.is_file():
        return path.resolve()
    portable = manifest_path.parent / split / path.name
    if portable.is_file():
        return portable.resolve()
    raise FileNotFoundError(
        f'World does not exist: {path}; portable candidate: {portable}')


def _is_stuck(
    poses: deque[tuple[float, float, float]],
    linear_velocity: float,
    *,
    window_steps: int,
    displacement_threshold: float,
    yaw_threshold: float,
    minimum_command_speed: float,
) -> bool:
    if window_steps <= 0 or len(poses) < window_steps + 1:
        return False
    if abs(linear_velocity) < minimum_command_speed:
        return False
    oldest = poses[0]
    newest = poses[-1]
    displacement = math.hypot(newest[0] - oldest[0], newest[1] - oldest[1])
    yaw_change = abs(wrap_angle(newest[2] - oldest[2]))
    return displacement < displacement_threshold and yaw_change < yaw_threshold


def _write_results(path: Path, rows: list[dict[str, Any]]) -> None:
    if path.exists():
        raise FileExistsError(f'Results file already exists: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Evaluate each world using only goal, odometry, and LiDAR scans."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--manifest', required=True)
    parser.add_argument(
        '--split', choices=('validation', 'test'), default='validation')
    parser.add_argument('--episodes', type=int, default=0)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    config = _load_config(args.config)
    planner = config['planner']
    mapping = config['mapping']
    controller_values = config['controller']
    evaluation = config['evaluation']
    manifest_path = Path(args.manifest).expanduser().resolve()
    rows = _manifest_rows(manifest_path, args.split)
    if args.episodes > 0:
        rows = rows[:args.episodes]
    if not rows:
        raise ValueError(f'No {args.split} worlds found in {manifest_path}')

    maximum_steps = int(evaluation['maximum_episode_steps'])
    settle_steps = int(evaluation.get(
        'settle_physics_steps', CONTROL_PHYSICS_STEPS))
    stuck_window = int(evaluation['stuck_window_steps'])
    simulation = BulletSim()
    results: list[dict[str, Any]] = []

    for episode, row in enumerate(rows):
        world = _resolve_world(
            manifest_path,
            args.split,
            row['world_file'],
        )
        start = (float(row['start_x']), float(row['start_y']))
        start_yaw = float(row['start_yaw'])
        goal = (float(row['goal_x']), float(row['goal_y']))

        # Only the physics engine sees the world file. The navigator does not.
        simulation.reset(str(world), start[0], start[1], start_yaw)
        simulation.set_command(0.0, 0.0)
        simulation.step(settle_steps)

        navigator = OnlineDijkstraNavigator(
            planner=planner,
            mapping=mapping,
            controller=controller_values,
            goal_x=goal[0],
            goal_y=goal[1],
        )
        initial_known_obstacles = len(navigator.mapper.obstacle_hits)
        if initial_known_obstacles != 0:
            raise AssertionError('LiDAR-only map must start without obstacles')

        state = simulation.robot_state()
        previous_x = float(state.transform.x)
        previous_y = float(state.transform.y)
        path_length = 0.0
        minimum_scan = math.inf
        minimum_front_scan = math.inf
        linear_speeds: list[float] = []
        angular_speeds: list[float] = []
        pivot_steps = 0
        emergency_steps = 0
        success = False
        collision = False
        timeout = False
        stuck = False
        planning_failure = False
        poses: deque[tuple[float, float, float]] = deque(
            maxlen=max(stuck_window + 1, 1))
        poses.append((previous_x, previous_y, float(state.yaw)))
        steps = 0

        while steps < maximum_steps:
            state = simulation.robot_state()
            distance = math.hypot(
                goal[0] - state.transform.x,
                goal[1] - state.transform.y,
            )
            collision = bool(simulation.has_collision())
            success = distance <= float(controller_values['goal_tolerance'])
            if collision or success:
                break

            scan = simulation.laser_scan()
            finite_ranges = [
                float(value) for value in scan.ranges
                if math.isfinite(float(value))
            ]
            minimum_scan = min(
                minimum_scan,
                min(finite_ranges, default=float(scan.range_max)),
            )
            front, _, _ = scan_clearances(
                scan.ranges,
                angle_min=float(scan.angle_min),
                angle_increment=float(scan.angle_increment),
                range_max=float(scan.range_max),
                front_half_angle_deg=float(
                    controller_values['lidar_front_half_angle_deg']),
            )
            minimum_front_scan = min(minimum_front_scan, front)

            try:
                update = navigator.update(
                    robot_x=float(state.transform.x),
                    robot_y=float(state.transform.y),
                    robot_yaw=float(state.yaw),
                    ranges=scan.ranges,
                    angle_min=float(scan.angle_min),
                    angle_increment=float(scan.angle_increment),
                    range_min=float(scan.range_min),
                    range_max=float(scan.range_max),
                )
            except (RuntimeError, ValueError):
                planning_failure = True
                break

            command = update.command
            if command.emergency:
                emergency_steps += 1
            simulation.set_command(command.linear, command.angular)
            simulation.step(CONTROL_PHYSICS_STEPS)
            steps += 1
            linear_speeds.append(float(command.linear))
            angular_speeds.append(abs(float(command.angular)))
            if abs(command.linear) <= 1e-6 and abs(command.angular) > 1e-6:
                pivot_steps += 1

            state = simulation.robot_state()
            current_x = float(state.transform.x)
            current_y = float(state.transform.y)
            path_length += math.hypot(
                current_x - previous_x,
                current_y - previous_y,
            )
            previous_x = current_x
            previous_y = current_y
            poses.append((current_x, current_y, float(state.yaw)))
            stuck = _is_stuck(
                poses,
                command.linear,
                window_steps=stuck_window,
                displacement_threshold=float(
                    evaluation['stuck_displacement_threshold']),
                yaw_threshold=float(evaluation['stuck_yaw_threshold']),
                minimum_command_speed=float(
                    evaluation['stuck_minimum_command_speed']),
            )
            if stuck:
                break

        final_state = simulation.robot_state()
        final_distance = math.hypot(
            goal[0] - final_state.transform.x,
            goal[1] - final_state.transform.y,
        )
        collision = bool(simulation.has_collision())
        success = bool(
            final_distance <= float(controller_values['goal_tolerance']) and
            not collision and not planning_failure
        )
        timeout = bool(
            steps >= maximum_steps and
            not success and not collision and not stuck and
            not planning_failure
        )
        simulation.set_command(0.0, 0.0)
        results.append({
            'episode': episode,
            'split': args.split,
            'world_file': str(world),
            'method_name': 'dijkstra_lidar_v2',
            'obstacle_source': 'lidar_only',
            'initial_known_obstacle_cells': initial_known_obstacles,
            'success': int(success),
            'collision': int(collision),
            'timeout': int(timeout),
            'stuck': int(stuck),
            'planning_failure': int(planning_failure),
            'steps': steps,
            'duration_s': round(
                steps / float(controller_values['control_hz']), 6),
            'path_length_m': round(path_length, 6),
            'planned_path_length_m': round(
                navigator.initial_plan_cost_m, 6),
            'planning_time_s': round(navigator.total_planning_time_s, 9),
            'expanded_cells': navigator.total_expanded_cells,
            'replans': navigator.replans,
            'discovered_obstacle_cells': len(
                navigator.mapper.obstacle_hits),
            'observed_free_cells': len(navigator.mapper.observed_free),
            'minimum_scan_m': round(
                minimum_scan if math.isfinite(minimum_scan) else 0.0, 6),
            'minimum_front_scan_m': round(
                minimum_front_scan
                if math.isfinite(minimum_front_scan) else 0.0,
                6,
            ),
            'mean_linear_speed_mps': round(mean(linear_speeds), 6)
            if linear_speeds else 0.0,
            'mean_abs_angular_speed_rps': round(mean(angular_speeds), 6)
            if angular_speeds else 0.0,
            'pivot_steps': pivot_steps,
            'emergency_steps': emergency_steps,
            'final_distance_m': round(final_distance, 6),
        })

    output_path = Path(args.output).expanduser().resolve()
    _write_results(output_path, results)
    successes = sum(int(row['success']) for row in results)
    collisions = sum(int(row['collision']) for row in results)
    timeouts = sum(int(row['timeout']) for row in results)
    stuck_count = sum(int(row['stuck']) for row in results)
    planning_failures = sum(int(row['planning_failure']) for row in results)
    print('method=dijkstra_lidar_v2')
    print('obstacle_source=lidar_only')
    print('initial_known_obstacle_cells=0')
    print(f'split={args.split}')
    print(f'episodes={len(results)}')
    print(f'successes={successes}')
    print(f'collisions={collisions}')
    print(f'timeouts={timeouts}')
    print(f'stuck={stuck_count}')
    print(f'planning_failures={planning_failures}')
    print(f'success_rate={successes / len(results):.6f}')
    print(f'collision_rate={collisions / len(results):.6f}')
    print(f'output={output_path}')


if __name__ == '__main__':
    main()
