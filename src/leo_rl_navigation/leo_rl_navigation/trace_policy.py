"""Trace one deterministic PPO episode without changing its actions."""

from __future__ import annotations

import argparse

import numpy as np
from stable_baselines3 import PPO

from leo_rl_navigation import LeoRoverEnv
from leo_rl_navigation.policy_io import DISCRETE_MANEUVER_TABLE
from leo_rl_navigation.training_utils import environment_kwargs, load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--split', choices=('train', 'validation', 'test'))
    parser.add_argument('--world-index', required=True, type=int)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--every', type=int, default=5)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()

    if args.every <= 0:
        raise ValueError('--every must be positive')

    arguments = environment_kwargs(load_config(args.config))
    arguments['manifest_path'] = args.manifest
    arguments['enable_safety_shield'] = False
    environment = LeoRoverEnv(split=args.split or 'validation', **arguments)
    model = PPO.load(args.model, device=args.device)
    if model.observation_space != environment.observation_space:
        raise ValueError(
            'Model and configuration have different observation spaces: '
            f'{model.observation_space} versus {environment.observation_space}')

    observation, info = environment.reset(
        seed=args.seed,
        options={'world_index': args.world_index},
    )
    print(f'world={info["world_file"]}')
    print(f'safety_shield={environment.enable_safety_shield}')
    print(f'action_mode={environment.action_mode}')
    print(
        f'{"STEP":>6} {"MANEUVER":>16} {"GOAL":>7} {"FRONT":>7} '
        f'{"LEFT":>7} {"RIGHT":>7} {"V":>7} {"W":>7} '
        f'{"TURN R":>8} {"CLEAR R":>8}'
    )

    while True:
        action, _ = model.predict(observation, deterministic=True)
        selected_action = (
            int(np.asarray(action).reshape(-1)[0])
            if environment.action_mode == 'discrete_primitives'
            else np.asarray(action, dtype=np.float32)
        )
        observation, _, terminated, truncated, info = environment.step(
            selected_action)
        if (
            info['episode_step'] == 1 or
            info['episode_step'] % args.every == 0 or
            terminated or truncated
        ):
            maneuver_name = (
                DISCRETE_MANEUVER_TABLE[selected_action][0]
                if environment.action_mode == 'discrete_primitives'
                else 'continuous'
            )
            print(
                f'{info["episode_step"]:>6} '
                f'{maneuver_name:>16} '
                f'{info["distance_to_goal"]:>7.3f} '
                f'{info["footprint_minimum_clearance"]:>7.3f} '
                f'{info["footprint_left_escape_clearance"]:>7.3f} '
                f'{info["footprint_right_escape_clearance"]:>7.3f} '
                f'{info["command_linear"]:>7.3f} '
                f'{info["command_angular"]:>7.3f} '
                f'{info["reward_escape_turn"]:>8.3f} '
                f'{info["reward_clearance_progress"]:>8.3f}'
            )
        if terminated or truncated:
            break

    print(f'success={info["is_success"]}')
    print(f'collision={info["collision"]}')
    print(f'timeout={info["timeout"]}')
    print(f'stuck={info["stuck"]}')
    environment.close()


if __name__ == '__main__':
    main()
