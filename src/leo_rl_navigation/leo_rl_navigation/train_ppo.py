"""Train PPO on the headless C++ Bullet environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.vec_env import DummyVecEnv

from leo_rl_navigation.training_utils import load_config, make_environment


def _positive(value: int, name: str) -> int:
    if value <= 0:
        raise ValueError(f'{name} must be positive')
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--total-timesteps', type=int)
    parser.add_argument('--n-envs', type=int)
    parser.add_argument('--eval-frequency', type=int)
    parser.add_argument('--checkpoint-frequency', type=int)
    parser.add_argument('--evaluation-episodes', type=int)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--device', default='cpu')
    parser.add_argument(
        '--initial-model',
        help=(
            'Optional PPO model whose policy parameters initialize this run. '
            'A new optimizer and timestep counter are still created.'
        ),
    )
    args = parser.parse_args()

    config = load_config(args.config)
    training = config['training']
    ppo_config = config['ppo']

    total_timesteps = _positive(
        int(args.total_timesteps or training['total_timesteps']),
        'total_timesteps',
    )
    n_envs = _positive(int(args.n_envs or training['n_envs']), 'n_envs')
    seed = int(training['seed'] if args.seed is None else args.seed)
    evaluation_frequency = _positive(
        int(args.eval_frequency or training['evaluation_frequency']),
        'evaluation_frequency',
    )
    checkpoint_frequency = _positive(
        int(args.checkpoint_frequency or training['checkpoint_frequency']),
        'checkpoint_frequency',
    )
    evaluation_episodes = _positive(
        int(args.evaluation_episodes or training['evaluation_episodes']),
        'evaluation_episodes',
    )

    run_directory = Path(args.run_dir).expanduser().resolve()
    if run_directory.exists() and any(run_directory.iterdir()):
        raise FileExistsError(
            f'Run directory is not empty: {run_directory}')
    checkpoint_directory = run_directory / 'checkpoints'
    best_directory = run_directory / 'best_model'
    evaluation_directory = run_directory / 'evaluation'
    tensorboard_directory = run_directory / 'tensorboard'
    for directory in (
            checkpoint_directory,
            best_directory,
            evaluation_directory,
            tensorboard_directory):
        directory.mkdir(parents=True, exist_ok=True)

    train_environment = DummyVecEnv([
        make_environment(
            config,
            split='train',
            seed=seed,
            rank=rank,
        )
        for rank in range(n_envs)
    ])
    validation_environment = DummyVecEnv([
        make_environment(
            config,
            split='validation',
            seed=seed + 100000,
            rank=0,
            sequential_worlds=True,
        )
    ])
    train_environment.seed(seed)
    validation_environment.seed(seed + 100000)

    checkpoint_callback = CheckpointCallback(
        save_freq=max(checkpoint_frequency // n_envs, 1),
        save_path=str(checkpoint_directory),
        name_prefix='ppo_leo',
    )
    evaluation_callback = EvalCallback(
        validation_environment,
        best_model_save_path=str(best_directory),
        log_path=str(evaluation_directory),
        eval_freq=max(evaluation_frequency // n_envs, 1),
        n_eval_episodes=evaluation_episodes,
        deterministic=True,
        render=False,
    )

    policy_kwargs = {
        'net_arch': list(ppo_config['net_arch']),
    }
    model = PPO(
        'MlpPolicy',
        train_environment,
        learning_rate=float(ppo_config['learning_rate']),
        n_steps=int(ppo_config['n_steps']),
        batch_size=int(ppo_config['batch_size']),
        n_epochs=int(ppo_config['n_epochs']),
        gamma=float(ppo_config['gamma']),
        gae_lambda=float(ppo_config['gae_lambda']),
        clip_range=float(ppo_config['clip_range']),
        ent_coef=float(ppo_config['ent_coef']),
        vf_coef=float(ppo_config['vf_coef']),
        max_grad_norm=float(ppo_config['max_grad_norm']),
        target_kl=(
            None if ppo_config.get('target_kl') is None
            else float(ppo_config['target_kl'])
        ),
        policy_kwargs=policy_kwargs,
        tensorboard_log=str(tensorboard_directory),
        seed=seed,
        device=args.device,
        verbose=1,
    )

    initial_model_path = None
    initial_model_timesteps = None
    if args.initial_model:
        initial_model_path = Path(args.initial_model).expanduser().resolve()
        if not initial_model_path.is_file():
            raise FileNotFoundError(
                f'Initial model does not exist: {initial_model_path}')
        initial_model = PPO.load(initial_model_path, device=args.device)
        if initial_model.observation_space != model.observation_space:
            raise ValueError('Initial model observation space is incompatible')
        if initial_model.action_space != model.action_space:
            raise ValueError('Initial model action space is incompatible')
        model.policy.load_state_dict(
            initial_model.policy.state_dict(),
            strict=True,
        )
        initial_model_timesteps = int(initial_model.num_timesteps)
        print(
            f'initialized_policy={initial_model_path} '
            f'source_timesteps={initial_model_timesteps}'
        )

    resolved = {
        'config': config,
        'resolved_training': {
            'total_timesteps': total_timesteps,
            'n_envs': n_envs,
            'seed': seed,
            'evaluation_frequency': evaluation_frequency,
            'checkpoint_frequency': checkpoint_frequency,
            'evaluation_episodes': evaluation_episodes,
            'device': args.device,
            'initial_model': (
                None if initial_model_path is None
                else str(initial_model_path)
            ),
            'initial_model_timesteps': initial_model_timesteps,
        },
    }
    with (run_directory / 'resolved_config.json').open(
            'w', encoding='utf-8') as stream:
        json.dump(resolved, stream, indent=2)
        stream.write('\n')

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=CallbackList([
                checkpoint_callback,
                evaluation_callback,
            ]),
            tb_log_name='ppo',
            reset_num_timesteps=True,
            progress_bar=False,
        )
        model.save(run_directory / 'final_model')
        print(f'final_model={run_directory / "final_model.zip"}')
    finally:
        train_environment.close()
        validation_environment.close()


if __name__ == '__main__':
    main()
