"""Evaluate PPO or a random policy on an immutable benchmark split."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO

from leo_rl_navigation import LeoRoverEnv
from leo_rl_navigation.training_utils import environment_kwargs, load_config


RESULT_FIELDS = (
    'episode',
    'split',
    'world_file',
    'policy_name',
    'success',
    'collision',
    'timeout',
    'stuck',
    'steps',
    'duration_s',
    'episode_return',
    'path_length_m',
    'minimum_scan_m',
    'minimum_front_scan_m',
    'minimum_footprint_clearance_m',
    'mean_linear_speed_mps',
    'mean_requested_linear_speed_mps',
    'reverse_steps',
    'mean_reverse_speed_mps',
    'front_blocked_steps',
    'mean_front_blocked_linear_speed_mps',
    'safety_intervention_steps',
    'mean_abs_angular_speed_rps',
    'final_distance_m',
)


def _write_results(path: Path, rows: list[dict[str, Any]]) -> None:
    if path.exists():
        raise FileExistsError(f'Results file already exists: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument(
        '--config',
        help='Optional training config defining the environment contract.',
    )
    parser.add_argument(
        '--split', choices=('validation', 'test'), default='validation')
    policy = parser.add_mutually_exclusive_group(required=True)
    policy.add_argument('--model')
    policy.add_argument('--random-policy', action='store_true')
    parser.add_argument('--output', required=True)
    parser.add_argument('--episodes', type=int, default=0)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--device', default='cpu')
    parser.add_argument(
        '--disable-safety-shield',
        action='store_true',
        help='Evaluate the raw PPO policy without the configured safety shield.',
    )
    args = parser.parse_args()

    if args.config:
        arguments = environment_kwargs(load_config(args.config))
        arguments['manifest_path'] = args.manifest
        if args.disable_safety_shield:
            arguments['enable_safety_shield'] = False
        environment = LeoRoverEnv(split=args.split, **arguments)
    else:
        environment = LeoRoverEnv(
            manifest_path=args.manifest,
            split=args.split,
        )
    episode_count = environment.world_count
    if args.episodes > 0:
        episode_count = min(args.episodes, episode_count)

    model = None
    if args.model:
        model = PPO.load(args.model, device=args.device)
        policy_name = Path(args.model).stem
    else:
        policy_name = 'random_policy'
        environment.action_space.seed(args.seed)

    rows: list[dict[str, Any]] = []
    for episode in range(episode_count):
        observation, info = environment.reset(
            seed=args.seed + episode,
            options={'world_index': episode},
        )
        previous_x = info['pose_x']
        previous_y = info['pose_y']
        episode_return = 0.0
        path_length = 0.0
        minimum_scan = info['minimum_scan']
        minimum_front_scan = info['front_minimum_scan']
        minimum_footprint_clearance = info['footprint_minimum_clearance']
        linear_speeds: list[float] = []
        requested_linear_speeds: list[float] = []
        reverse_speeds: list[float] = []
        front_blocked_linear_speeds: list[float] = []
        angular_speeds: list[float] = []
        safety_interventions = 0

        while True:
            if model is None:
                action = environment.action_space.sample()
            else:
                action, _ = model.predict(observation, deterministic=True)
            action = np.asarray(action, dtype=np.float32)

            observation, reward, terminated, truncated, info = (
                environment.step(action)
            )
            episode_return += reward
            path_length += math.hypot(
                info['pose_x'] - previous_x,
                info['pose_y'] - previous_y,
            )
            previous_x = info['pose_x']
            previous_y = info['pose_y']
            minimum_scan = min(minimum_scan, info['minimum_scan'])
            minimum_front_scan = min(
                minimum_front_scan,
                info['front_minimum_scan'],
            )
            minimum_footprint_clearance = min(
                minimum_footprint_clearance,
                info['footprint_minimum_clearance'],
            )
            linear_speed = float(info['command_linear'])
            linear_speeds.append(linear_speed)
            if linear_speed < -1e-6:
                reverse_speeds.append(abs(linear_speed))
            requested_linear_speeds.append(float(
                info['requested_command_linear']))
            safety_interventions += int(info['safety_shield_active'])
            blocked_clearance = (
                info['footprint_minimum_clearance']
                if environment.use_footprint_clearance
                else info['front_minimum_scan']
            )
            if (
                environment.front_safety_distance > 0.0 and
                blocked_clearance < environment.front_safety_distance
            ):
                front_blocked_linear_speeds.append(linear_speed)
            angular_speeds.append(
                abs(float(info['command_angular']))
            )

            if terminated or truncated:
                break

        rows.append({
            'episode': episode,
            'split': args.split,
            'world_file': info['world_file'],
            'policy_name': policy_name,
            'success': int(info['is_success']),
            'collision': int(info['collision']),
            'timeout': int(info['timeout']),
            'stuck': int(info['stuck']),
            'steps': info['episode_step'],
            'duration_s': round(
                info['episode_step'] * environment.control_period, 6),
            'episode_return': round(episode_return, 6),
            'path_length_m': round(path_length, 6),
            'minimum_scan_m': round(minimum_scan, 6),
            'minimum_front_scan_m': round(minimum_front_scan, 6),
            'minimum_footprint_clearance_m': round(
                minimum_footprint_clearance, 6),
            'mean_linear_speed_mps': round(
                float(np.mean(linear_speeds)), 6),
            'mean_requested_linear_speed_mps': round(
                float(np.mean(requested_linear_speeds)), 6),
            'reverse_steps': len(reverse_speeds),
            'mean_reverse_speed_mps': round(
                float(np.mean(reverse_speeds)) if reverse_speeds else 0.0,
                6,
            ),
            'front_blocked_steps': len(front_blocked_linear_speeds),
            'mean_front_blocked_linear_speed_mps': round(
                float(np.mean(front_blocked_linear_speeds))
                if front_blocked_linear_speeds else 0.0,
                6,
            ),
            'safety_intervention_steps': safety_interventions,
            'mean_abs_angular_speed_rps': round(
                float(np.mean(angular_speeds)), 6),
            'final_distance_m': round(info['distance_to_goal'], 6),
        })

    output_path = Path(args.output).expanduser().resolve()
    _write_results(output_path, rows)
    successes = sum(row['success'] for row in rows)
    collisions = sum(row['collision'] for row in rows)
    timeouts = sum(row['timeout'] for row in rows)
    stuck = sum(row['stuck'] for row in rows)
    reverse_steps = sum(row['reverse_steps'] for row in rows)
    safety_interventions = sum(
        row['safety_intervention_steps'] for row in rows)
    print(f'policy={policy_name}')
    print(f'safety_shield={environment.enable_safety_shield}')
    print(f'split={args.split}')
    print(f'episodes={len(rows)}')
    print(f'successes={successes}')
    print(f'collisions={collisions}')
    print(f'timeouts={timeouts}')
    print(f'stuck={stuck}')
    print(f'reverse_steps={reverse_steps}')
    print(f'safety_intervention_steps={safety_interventions}')
    print(f'success_rate={successes / max(len(rows), 1):.6f}')
    print(f'collision_rate={collisions / max(len(rows), 1):.6f}')
    print(f'stuck_rate={stuck / max(len(rows), 1):.6f}')
    print(f'output={output_path}')
    environment.close()


if __name__ == '__main__':
    main()
