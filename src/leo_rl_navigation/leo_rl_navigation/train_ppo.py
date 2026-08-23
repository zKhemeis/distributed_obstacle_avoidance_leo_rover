"""Train PPO on the headless C++ Bullet environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
)

from leo_rl_navigation.policy_io import DISCRETE_MANEUVER_TABLE
from leo_rl_navigation.training_utils import load_config, make_environment


def _positive(value: int, name: str) -> int:
    if value <= 0:
        raise ValueError(f'{name} must be positive')
    return value


def _reset_linear_action_head(
    model: PPO,
    initial_bias: float,
) -> None:
    """Reset only the forward-speed row of the Gaussian action mean."""
    if not -1.0 < initial_bias < 1.0:
        raise ValueError('linear action initial bias must be in (-1, 1)')

    action_net = model.policy.action_net
    if action_net.out_features != 2:
        raise ValueError(
            'Expected a two-output continuous action head, got '
            f'{action_net.out_features}')

    with torch.no_grad():
        action_net.weight[0].zero_()
        action_net.bias[0].fill_(initial_bias)


def _expand_policy_observation(
    source_model: PPO,
    target_model: PPO,
    inserted_index: int,
) -> int:
    """Copy a policy into a model with inserted observation features.

    The new input columns are initialized to zero. Consequently, the expanded
    policy initially produces the same outputs as the source policy while the
    new feature remains available for subsequent fine-tuning.
    """
    source_shape = source_model.observation_space.shape
    target_shape = target_model.observation_space.shape
    if source_shape is None or target_shape is None:
        raise ValueError('Observation spaces must have fixed shapes')
    if len(source_shape) != 1 or len(target_shape) != 1:
        raise ValueError('Only one-dimensional observations are supported')

    source_width = int(source_shape[0])
    target_width = int(target_shape[0])
    inserted_features = target_width - source_width
    if inserted_features <= 0:
        raise ValueError(
            'Expanded target observation must contain additional features: '
            f'source={source_width}, target={target_width}')
    if not 0 <= inserted_index <= source_width:
        raise ValueError(
            f'Inserted observation index must be in [0, {source_width}]')
    if source_model.action_space != target_model.action_space:
        raise ValueError('Initial model action space is incompatible')

    source_state = source_model.policy.state_dict()
    target_state = target_model.policy.state_dict()
    if list(source_state) != list(target_state):
        raise ValueError('Initial and target policy parameters are incompatible')

    expanded_state = {}
    expanded_parameters = 0
    for name, target_tensor in target_state.items():
        source_tensor = source_state[name]
        if source_tensor.shape == target_tensor.shape:
            expanded_state[name] = source_tensor.detach().clone()
            continue

        can_expand_input = (
            source_tensor.ndim == 2 and
            target_tensor.ndim == 2 and
            source_tensor.shape[0] == target_tensor.shape[0] and
            source_tensor.shape[1] == source_width and
            target_tensor.shape[1] == target_width
        )
        if not can_expand_input:
            raise ValueError(
                f'Cannot expand policy parameter {name}: '
                f'{tuple(source_tensor.shape)} -> '
                f'{tuple(target_tensor.shape)}')

        expanded_tensor = torch.zeros_like(target_tensor)
        expanded_tensor[:, :inserted_index] = (
            source_tensor[:, :inserted_index])
        expanded_tensor[:, inserted_index + inserted_features:] = (
            source_tensor[:, inserted_index:])
        expanded_state[name] = expanded_tensor
        expanded_parameters += 1

    if expanded_parameters == 0:
        raise ValueError('No observation-input policy parameters were expanded')

    target_model.policy.load_state_dict(expanded_state, strict=True)
    return expanded_parameters


def _transfer_continuous_navigation_backbone(
    source_model: PPO,
    target_model: PPO,
) -> int:
    """Reuse PPO navigation and value features with a categorical action head.

    The source Gaussian head cannot be copied into a categorical policy. Its
    learned angular row seeds the new maneuver logits while all matching
    actor/critic feature and value parameters are copied exactly.
    """
    if source_model.observation_space != target_model.observation_space:
        raise ValueError('Initial model observation space is incompatible')
    if source_model.policy.action_net.out_features != 2:
        raise ValueError('Source model must have two continuous action outputs')
    target_head = target_model.policy.action_net
    if target_head.out_features != len(DISCRETE_MANEUVER_TABLE):
        raise ValueError('Target model must use the discrete maneuver head')

    source_state = source_model.policy.state_dict()
    target_state = target_model.policy.state_dict()
    copied_parameters = 0
    for name, target_tensor in target_state.items():
        if name.startswith('action_net.'):
            continue
        source_tensor = source_state.get(name)
        if source_tensor is None or source_tensor.shape != target_tensor.shape:
            raise ValueError(
                f'Cannot transfer navigation parameter {name}: '
                f'{None if source_tensor is None else tuple(source_tensor.shape)} '
                f'-> {tuple(target_tensor.shape)}')
        target_state[name] = source_tensor.detach().clone()
        copied_parameters += 1

    if copied_parameters == 0:
        raise ValueError('No navigation or value parameters were transferred')
    target_model.policy.load_state_dict(target_state, strict=True)

    source_head = source_model.policy.action_net
    with torch.no_grad():
        for index, (_, linear_fraction, angular_fraction) in enumerate(
                DISCRETE_MANEUVER_TABLE):
            target_head.weight[index].copy_(
                0.8 * angular_fraction * source_head.weight[1])
            speed_prior = (
                0.50 * max(linear_fraction, 0.0) -
                0.20 * max(-linear_fraction, 0.0) -
                0.10 * abs(angular_fraction)
            )
            target_head.bias[index].copy_(
                0.8 * angular_fraction * source_head.bias[1] + speed_prior)

    return copied_parameters

def _resolve_vectorized_backend(
    requested_backend: str,
    n_envs: int,
) -> str:
    """Choose the vectorized-environment implementation."""
    if n_envs <= 0:
        raise ValueError('n_envs must be positive')

    valid_backends = {'auto', 'dummy', 'subproc'}

    if requested_backend not in valid_backends:
        raise ValueError(
            f'Unsupported vectorized backend: {requested_backend}'
        )

    if requested_backend == 'auto':
        return 'dummy' if n_envs == 1 else 'subproc'

    return requested_backend


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--total-timesteps', type=int)
    parser.add_argument('--n-envs', type=int)
    parser.add_argument(
        '--vec-env',
        choices=('auto', 'dummy', 'subproc'),
        default='auto',
        help=(
            'Vectorized environment backend. Auto selects DummyVecEnv '
            'for one robot and SubprocVecEnv for multiple robots.'
        ),
    )
    parser.add_argument(
        '--rollout-steps',
        type=int,
        help=(
            'PPO rollout steps per environment. Allows equal total '
            'rollout batch sizes across different robot counts.'
        ),
    )
    parser.add_argument(
        '--subproc-start-method',
        choices=('forkserver', 'spawn', 'fork'),
        default='forkserver',
        help=(
            'Multiprocessing start method used by SubprocVecEnv. '
            'Default: forkserver.'
        ),
    )
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
    parser.add_argument(
        '--reset-linear-action-head',
        action='store_true',
        help=(
            'After loading --initial-model, reset only the forward-speed '
            'action-mean row so it can be relearned without clipping.'
        ),
    )
    parser.add_argument(
        '--transfer-discrete-action-backbone',
        action='store_true',
        help=(
            'Initialize a categorical maneuver PPO policy from a continuous '
            'PPO actor/critic backbone and its learned angular action row.'
        ),
    )
    parser.add_argument(
        '--linear-action-initial-bias',
        type=float,
        default=0.5,
        help=(
            'Raw normalized forward-action mean used with '
            '--reset-linear-action-head. Default 0.5 maps to 75 percent '
            'of linear_speed_max.'
        ),
    )
    parser.add_argument(
        '--expand-observation-at-index',
        type=int,
        help=(
            'Expand a one-dimensional --initial-model observation by the '
            'required number of features at this zero-based index. Inserted weights '
            'start at zero, preserving the source policy exactly.'
        ),
    )
    parser.add_argument(
        '--save-initialized-model-only',
        action='store_true',
        help=(
            'Create and save initialized_model.zip without learning. Requires '
            '--initial-model and is intended for equivalence auditing.'
        ),
    )
    args = parser.parse_args()

    if args.reset_linear_action_head and not args.initial_model:
        raise ValueError(
            '--reset-linear-action-head requires --initial-model')
    if args.expand_observation_at_index is not None and not args.initial_model:
        raise ValueError(
            '--expand-observation-at-index requires --initial-model')
    if args.transfer_discrete_action_backbone and not args.initial_model:
        raise ValueError(
            '--transfer-discrete-action-backbone requires --initial-model')
    if args.transfer_discrete_action_backbone and args.reset_linear_action_head:
        raise ValueError(
            'Discrete backbone transfer cannot reset a continuous action head')
    if (
        args.transfer_discrete_action_backbone and
        args.expand_observation_at_index is not None
    ):
        raise ValueError(
            'Discrete backbone transfer requires matching observation widths')
    if args.save_initialized_model_only and not args.initial_model:
        raise ValueError(
            '--save-initialized-model-only requires --initial-model')
    config = load_config(args.config)
    training = config['training']
    ppo_config = config['ppo']

    total_timesteps = _positive(
        int(args.total_timesteps or training['total_timesteps']),
        'total_timesteps',
    )
    n_envs = _positive(int(args.n_envs or training['n_envs']), 'n_envs')
    vectorized_backend = _resolve_vectorized_backend(
        args.vec_env,
        n_envs,
    )
    rollout_steps = _positive(
        int(args.rollout_steps or ppo_config['n_steps']),
        'rollout_steps',
    )
    rollout_batch_size = rollout_steps * n_envs
    minibatch_size = int(ppo_config['batch_size'])

    if rollout_batch_size < minibatch_size:
        raise ValueError(
            'Combined rollout batch must not be smaller than the '
            f'PPO minibatch size: rollout={rollout_batch_size}, '
            f'minibatch={minibatch_size}'
        )

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

    environment_factories = [
        make_environment(
            config,
            split='train',
            seed=seed,
            rank=rank,
        )
        for rank in range(n_envs)
    ]

    if vectorized_backend == 'subproc':
        train_environment = SubprocVecEnv(
            environment_factories,
            start_method=args.subproc_start_method,
        )
        worker_process_ids = [
            int(process.pid)
            for process in train_environment.processes
        ]
    else:
        train_environment = DummyVecEnv(environment_factories)
        worker_process_ids = []

    print(
        f'vectorized_environment={vectorized_backend} '
        f'robot_count={n_envs} '
        f'rollout_steps_per_robot={rollout_steps} '
        f'combined_rollout_batch={rollout_batch_size} '
        f'worker_process_ids={worker_process_ids}',
        flush=True,
    )

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
        n_steps=rollout_steps,
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
    expanded_policy_parameters = None
    transferred_navigation_parameters = None
    if args.initial_model:
        initial_model_path = Path(args.initial_model).expanduser().resolve()
        if not initial_model_path.is_file():
            raise FileNotFoundError(
                f'Initial model does not exist: {initial_model_path}')
        initial_model = PPO.load(initial_model_path, device=args.device)
        if args.transfer_discrete_action_backbone:
            transferred_navigation_parameters = (
                _transfer_continuous_navigation_backbone(
                    initial_model,
                    model,
                )
            )
            print(
                'transferred_discrete_action_backbone=true '
                f'copied_parameters={transferred_navigation_parameters} '
                f'action_count={len(DISCRETE_MANEUVER_TABLE)}'
            )
        elif args.expand_observation_at_index is None:
            if initial_model.observation_space != model.observation_space:
                raise ValueError(
                    'Initial model observation space is incompatible')
            if initial_model.action_space != model.action_space:
                raise ValueError('Initial model action space is incompatible')
            model.policy.load_state_dict(
                initial_model.policy.state_dict(),
                strict=True,
            )
        else:
            expanded_policy_parameters = _expand_policy_observation(
                initial_model,
                model,
                args.expand_observation_at_index,
            )
            inserted_features = (
                model.observation_space.shape[0] -
                initial_model.observation_space.shape[0]
            )
            print(
                'expanded_observation=true '
                f'inserted_index={args.expand_observation_at_index} '
                f'inserted_features={inserted_features} '
                f'expanded_parameters={expanded_policy_parameters}'
            )
        if args.reset_linear_action_head:
            _reset_linear_action_head(
                model,
                args.linear_action_initial_bias,
            )
            print(
                'reset_linear_action_head=true '
                f'initial_bias={args.linear_action_initial_bias}'
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
            'vectorized_environment': vectorized_backend,
            'subproc_start_method': (
                args.subproc_start_method
                if vectorized_backend == 'subproc'
                else None
            ),
            'rollout_steps_per_environment': rollout_steps,
            'combined_rollout_batch_size': rollout_batch_size,
            'worker_process_ids': worker_process_ids,
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
            'reset_linear_action_head': bool(
                args.reset_linear_action_head),
            'linear_action_initial_bias': (
                float(args.linear_action_initial_bias)
                if args.reset_linear_action_head else None
            ),
            'expand_observation_at_index': (
                args.expand_observation_at_index
            ),
            'expanded_policy_parameters': expanded_policy_parameters,
            'transfer_discrete_action_backbone': bool(
                args.transfer_discrete_action_backbone),
            'transferred_navigation_parameters': (
                transferred_navigation_parameters),
            'save_initialized_model_only': bool(
                args.save_initialized_model_only),
        },
    }
    with (run_directory / 'resolved_config.json').open(
            'w', encoding='utf-8') as stream:
        json.dump(resolved, stream, indent=2)
        stream.write('\n')

    if args.initial_model:
        initialized_model_path = run_directory / 'initialized_model'
        model.save(initialized_model_path)
        print(f'initialized_model={initialized_model_path}.zip')

    if args.save_initialized_model_only:
        train_environment.close()
        validation_environment.close()
        print('learning_skipped=true')
        return

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
