"""Validate and benchmark the headless Leo Gymnasium environment."""

from __future__ import annotations

import argparse
import time

import numpy as np
from stable_baselines3.common.env_checker import check_env

from leo_rl_navigation import LeoRoverEnv
from leo_rl_navigation.training_utils import environment_kwargs, load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument(
        '--config',
        help='Optional training config defining the environment contract.',
    )
    parser.add_argument('--split', default='train')
    parser.add_argument('--steps', type=int, default=500)
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()

    if args.steps <= 0:
        parser.error('--steps must be positive')

    if args.config:
        arguments = environment_kwargs(load_config(args.config))
        arguments['manifest_path'] = args.manifest
        environment = LeoRoverEnv(split=args.split, **arguments)
    else:
        environment = LeoRoverEnv(
            manifest_path=args.manifest,
            split=args.split,
        )
    print(f'worlds={environment.world_count}')
    print(f'observation_shape={environment.observation_space.shape}')
    print(f'action_shape={environment.action_space.shape}')
    print(f'action_mode={environment.action_mode}')
    if hasattr(environment.action_space, 'n'):
        print(f'action_count={environment.action_space.n}')

    check_env(environment, warn=True, skip_render_check=True)
    print('check_env=PASS')

    environment.action_space.seed(args.seed)
    observation, info = environment.reset(seed=args.seed)
    print(f'reset_world={info["world_file"]}')
    print(f'reset_goal=({info["goal_x"]:.3f}, {info["goal_y"]:.3f})')
    print(f'reset_distance={info["distance_to_goal"]:.6f}')
    print(f'reset_minimum_scan={info["minimum_scan"]:.6f}')
    if observation.shape != environment.observation_space.shape:
        raise RuntimeError('Reset observation has an unexpected shape')
    if not np.isfinite(observation).all():
        raise RuntimeError('Reset observation is not finite')

    completed_episodes = 0
    collisions = 0
    successes = 0
    stuck = 0
    wall_start = time.perf_counter()
    for step_index in range(args.steps):
        action = environment.action_space.sample()
        observation, _, terminated, truncated, info = environment.step(action)
        if not environment.observation_space.contains(observation):
            raise RuntimeError('Step observation is outside its declared space')
        if terminated or truncated:
            completed_episodes += 1
            collisions += int(info['collision'])
            successes += int(info['is_success'])
            stuck += int(info['stuck'])
            environment.reset(seed=args.seed + step_index + 1)

    wall_seconds = time.perf_counter() - wall_start
    steps_per_second = args.steps / max(wall_seconds, 1e-12)
    simulated_seconds = args.steps * environment.control_period
    real_time_factor = simulated_seconds / max(wall_seconds, 1e-12)

    print(f'rollout_steps={args.steps}')
    print(f'completed_episodes={completed_episodes}')
    print(f'collisions={collisions}')
    print(f'successes={successes}')
    print(f'stuck={stuck}')
    print(f'wall_time_s={wall_seconds:.6f}')
    print(f'steps_per_second={steps_per_second:.3f}')
    print(f'rtf={real_time_factor:.3f}')
    environment.close()


if __name__ == '__main__':
    main()
