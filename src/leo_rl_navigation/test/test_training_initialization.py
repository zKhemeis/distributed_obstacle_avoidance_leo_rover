#!/usr/bin/env python3

from __future__ import annotations

import unittest

import torch

from leo_rl_navigation.train_ppo import (
    _expand_policy_observation,
    _reset_linear_action_head,
    _transfer_continuous_navigation_backbone,
)


class _Policy:
    def __init__(self) -> None:
        self.action_net = torch.nn.Linear(4, 2)


class _Model:
    def __init__(self) -> None:
        self.policy = _Policy()


class _Space:
    def __init__(self, width: int) -> None:
        self.shape = (width,)


class _ExpandablePolicy(torch.nn.Module):
    def __init__(self, width: int) -> None:
        super().__init__()
        self.policy_input = torch.nn.Linear(width, 3)
        self.value_input = torch.nn.Linear(width, 3)
        self.action_net = torch.nn.Linear(3, 2)


class _ExpandableModel:
    def __init__(self, width: int, action_space: object) -> None:
        self.observation_space = _Space(width)
        self.action_space = action_space
        self.policy = _ExpandablePolicy(width)


class _ManeuverPolicy(torch.nn.Module):
    def __init__(self, action_count: int) -> None:
        super().__init__()
        self.policy_input = torch.nn.Linear(3, 4)
        self.value_input = torch.nn.Linear(3, 4)
        self.action_net = torch.nn.Linear(4, action_count)


class _ManeuverModel:
    def __init__(self, action_count: int) -> None:
        self.observation_space = (3,)
        self.policy = _ManeuverPolicy(action_count)


class TrainingInitializationTests(unittest.TestCase):
    def test_only_linear_action_row_is_reset(self) -> None:
        model = _Model()
        with torch.no_grad():
            model.policy.action_net.weight.fill_(2.0)
            model.policy.action_net.bias.fill_(3.0)

        angular_weight = model.policy.action_net.weight[1].detach().clone()
        angular_bias = model.policy.action_net.bias[1].detach().clone()

        _reset_linear_action_head(model, initial_bias=0.5)

        self.assertTrue(torch.equal(
            model.policy.action_net.weight[0],
            torch.zeros_like(model.policy.action_net.weight[0]),
        ))
        self.assertAlmostEqual(
            float(model.policy.action_net.bias[0]),
            0.5,
        )
        self.assertTrue(torch.equal(
            model.policy.action_net.weight[1],
            angular_weight,
        ))
        self.assertTrue(torch.equal(
            model.policy.action_net.bias[1],
            angular_bias,
        ))

    def test_initial_bias_must_be_unsaturated(self) -> None:
        model = _Model()
        with self.assertRaises(ValueError):
            _reset_linear_action_head(model, initial_bias=1.0)

    def test_observation_expansion_preserves_policy_outputs(self) -> None:
        action_space = object()
        source = _ExpandableModel(3, action_space)
        target = _ExpandableModel(4, action_space)

        with torch.no_grad():
            for index, parameter in enumerate(source.policy.parameters()):
                parameter.copy_(torch.arange(
                    parameter.numel(),
                    dtype=parameter.dtype,
                ).reshape(parameter.shape) + index)

        expanded = _expand_policy_observation(
            source,
            target,
            inserted_index=2,
        )
        self.assertEqual(expanded, 2)

        source_observation = torch.tensor([[0.2, -0.4, 0.7]])
        for inserted_value in (-1.0, 0.0, 0.75, 1.0):
            target_observation = torch.tensor([[
                0.2,
                -0.4,
                inserted_value,
                0.7,
            ]])
            self.assertTrue(torch.equal(
                source.policy.policy_input(source_observation),
                target.policy.policy_input(target_observation),
            ))
            self.assertTrue(torch.equal(
                source.policy.value_input(source_observation),
                target.policy.value_input(target_observation),
            ))

        self.assertTrue(torch.equal(
            target.policy.policy_input.weight[:, 2],
            torch.zeros(3),
        ))
        self.assertTrue(torch.equal(
            target.policy.value_input.weight[:, 2],
            torch.zeros(3),
        ))

    def test_observation_expansion_preserves_two_directional_features(self) -> None:
        action_space = object()
        source = _ExpandableModel(3, action_space)
        target = _ExpandableModel(5, action_space)

        with torch.no_grad():
            for index, parameter in enumerate(source.policy.parameters()):
                parameter.copy_(torch.arange(
                    parameter.numel(),
                    dtype=parameter.dtype,
                ).reshape(parameter.shape) + index)

        expanded = _expand_policy_observation(
            source,
            target,
            inserted_index=2,
        )
        self.assertEqual(expanded, 2)

        source_observation = torch.tensor([[0.2, -0.4, 0.7]])
        target_observation = torch.tensor([[0.2, -0.4, 0.1, 0.9, 0.7]])
        self.assertTrue(torch.equal(
            source.policy.policy_input(source_observation),
            target.policy.policy_input(target_observation),
        ))
        self.assertTrue(torch.equal(
            source.policy.value_input(source_observation),
            target.policy.value_input(target_observation),
        ))
        self.assertTrue(torch.equal(
            target.policy.policy_input.weight[:, 2:4],
            torch.zeros((3, 2)),
        ))

    def test_observation_expansion_requires_additional_features(self) -> None:
        action_space = object()
        source = _ExpandableModel(3, action_space)
        target = _ExpandableModel(3, action_space)

        with self.assertRaises(ValueError):
            _expand_policy_observation(
                source,
                target,
                inserted_index=2,
            )

    def test_discrete_transfer_preserves_actor_and_value_features(self) -> None:
        source = _ManeuverModel(2)
        target = _ManeuverModel(13)
        with torch.no_grad():
            source.policy.policy_input.weight.fill_(1.25)
            source.policy.value_input.weight.fill_(2.50)
            source.policy.action_net.weight[1].fill_(0.50)

        transferred = _transfer_continuous_navigation_backbone(
            source,
            target,
        )
        self.assertGreater(transferred, 0)
        self.assertTrue(torch.equal(
            source.policy.policy_input.weight,
            target.policy.policy_input.weight,
        ))
        self.assertTrue(torch.equal(
            source.policy.value_input.weight,
            target.policy.value_input.weight,
        ))
        self.assertEqual(target.policy.action_net.out_features, 13)


if __name__ == '__main__':
    unittest.main()
