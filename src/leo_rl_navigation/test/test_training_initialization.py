#!/usr/bin/env python3

from __future__ import annotations

import unittest

import torch

from leo_rl_navigation.train_ppo import _reset_linear_action_head


class _Policy:
    def __init__(self) -> None:
        self.action_net = torch.nn.Linear(4, 2)


class _Model:
    def __init__(self) -> None:
        self.policy = _Policy()


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


if __name__ == '__main__':
    unittest.main()
