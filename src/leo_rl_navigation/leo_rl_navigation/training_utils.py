"""Shared configuration and environment factories for PPO tools."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import gymnasium as gym
import yaml
from stable_baselines3.common.monitor import Monitor

from leo_rl_navigation import LeoRoverEnv


INFO_KEYWORDS = (
    'is_success',
    'collision',
    'timeout',
    'stuck',
    'distance_to_goal',
    'minimum_scan',
)


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f'{config_path} must contain a YAML mapping')
    for section in ('environment', 'ppo', 'training'):
        if section not in config or not isinstance(config[section], dict):
            raise ValueError(f'Configuration is missing section {section!r}')
    return config


def environment_kwargs(config: dict[str, Any]) -> dict[str, Any]:
    values = dict(config['environment'])
    values['manifest_path'] = str(
        Path(values['manifest_path']).expanduser().resolve())
    return values


def make_environment(
    config: dict[str, Any],
    *,
    split: str,
    seed: int,
    rank: int,
    sequential_worlds: bool = False,
) -> Callable[[], gym.Env]:
    kwargs = environment_kwargs(config)
    kwargs['sequential_worlds'] = sequential_worlds

    def factory() -> gym.Env:
        environment = LeoRoverEnv(split=split, **kwargs)
        environment.action_space.seed(seed + rank)
        return Monitor(environment, info_keywords=INFO_KEYWORDS)

    return factory
