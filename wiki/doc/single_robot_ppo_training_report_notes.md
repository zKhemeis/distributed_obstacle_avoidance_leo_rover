# Single-Robot PPO Obstacle-Avoidance Training: Report Notes

## Project objective

- Compare three navigation methods:

  - Heuristic obstacle avoidance.
  - PPO obstacle avoidance with one robot.
  - PPO obstacle avoidance with multiple robots sharing training information.

- The task is to reach a target position while avoiding obstacles.

- The reported single-robot method must remain pure PPO:

  - No active safety shield.
  - No heuristic controller modifying actions.
  - PPO selects every executed maneuver.

## Simulation infrastructure

- Robot: Leo Rover.

- Middleware: ROS 2 Humble.

- Simulator: PyBullet with a custom C++ Bullet simulation core.

- Reinforcement-learning environment: Gymnasium.

- Learning algorithm: Stable-Baselines3 PPO.

- Neural-network framework: PyTorch.

- Visualization: RViz.

- Physics frequency:

  ```text
  240 Hz
  ```

- Physics steps per control action:

  ```text
  24
  ```

- Policy-control frequency:

  ```text
  240 / 24 = 10 Hz
  ```

- Simulated LiDAR:

  ```text
  Number of rays: 500

  Minimum range: 0.05 m

  Maximum range: 12.0 m

  Number of observation sectors: 50
  ```

## Benchmark datasets

- Benchmark V2:

  ```text
  Training maps: 300

  Validation maps: 60

  Test maps: 90
  ```

- Benchmark V3:

  ```text
  Training maps: 300

  Validation maps: 60

  Test maps: 90
  ```

- Benchmark V4:

  ```text
  Training maps: 450

  Validation maps: 90

  Test maps: 120
  ```

- Generated scenarios include:

  - Clear paths.
  - Random obstacle arrangements.
  - One obstacle blocking the direct route.
  - Two obstacles blocking the direct route.
  - Randomized start positions.
  - Randomized start orientations.
  - Randomized goal positions.

- A* verifies whether a generated map is solvable.

- A* is not used to control or guide the PPO policy.

## Initial baseline policy

- The first PPO observation contained:

  ```text
  50 LiDAR sectors

  1 normalized goal distance

  2 goal-direction values

  Total: 53 observations
  ```

- Goal direction was encoded as:

  ```text
  sin(relative_goal_angle)

  cos(relative_goal_angle)
  ```

- The initial action space contained:

  ```text
  Continuous forward velocity

  Continuous angular velocity
  ```

- Initial V2 validation performance:

  ```text
  Successes: 49 / 60

  Collisions: 10 / 60

  Timeouts: 1 / 60
  ```

- Problem:

  - The policy frequently drove toward the goal at maximum speed.
  - The forward action became saturated.
  - Steering changed near obstacles, but forward speed often remained high.

## Safety-aware reward experiments

- Added penalties for:

  - Collisions.
  - Timeouts.
  - Obstacle proximity.
  - Unsafe forward speed.
  - Stuck behavior.

- Training a new policy from scratch with strong safety penalties performed poorly.

- Typical failure:

  - The robot avoided movement.
  - Episodes ended with timeouts.
  - Goal-reaching performance collapsed.

- Warm-start training was introduced:

  - Load parameters from an existing PPO policy.
  - Start a new optimizer.
  - Restart the training-step counter.
  - Continue learning with a modified reward.

- Warm-starting preserved navigation ability better than restarting from random weights.

## Explicit front-clearance feature

- Added an explicit feature describing the obstacle distance in front of the rover.

- Observation size increased from:

  ```text
  53 → 54
  ```

- New observation:

  ```text
  50 LiDAR sectors

  1 front-clearance feature

  1 normalized goal distance

  2 goal-direction features

  Total: 54 observations
  ```

- A policy trained from scratch with 54 observations performed poorly.

- Example:

  ```text
  4 successes / 60 validation episodes
  ```

- A transfer method was implemented to expand an existing policy:

  - Copy the original network parameters.
  - Insert the additional observation feature.
  - Initialize its new input weights to zero.
  - Preserve the existing policy output initially.
  - Fine-tune the expanded network with PPO.

## LiDAR orientation correction

- LiDAR scans were canonicalized so the forward direction is interpreted consistently.

- This avoids inconsistencies when different scan messages start at different angles.

- Verified LiDAR frame:

  ```text
  base_scan
  ```

- Verified transform relative to `base_link`:

  ```text
  x = 0.10 m

  y = 0.00 m

  z = 0.08 m
  ```

- The LiDAR measures distance from the sensor location rather than the robot body boundary.

## Robot-footprint clearance

- Added body-relative obstacle clearance.

- Robot footprint parameters:

  ```text
  Half length: 0.215 m

  Half width: 0.259 m

  LiDAR forward offset: 0.10 m
  ```

- Conceptually:

  ```text
  body_clearance =
      measured_LiDAR_distance
      -
      distance_from_LiDAR_to_robot_boundary
  ```

- This explains why a LiDAR reading around 0.25 m can correspond to an obstacle very close to the rover body.

## Curriculum training

- Training maps were organized by difficulty.

- The policy progressed through:

  ```text
  General maps

  Blocked-path maps

  Hard blocked-path maps

  Double-obstacle maps

  Focused difficult scenarios
  ```

- Purpose:

  - Prevent the policy from learning only straight-line goal reaching.
  - Increase exposure to situations requiring obstacle detours.
  - Improve generalization to difficult maps.

- Example curriculum improvement:

  ```text
  Initial curriculum test:

  62 successes / 90 episodes

  Curriculum-trained policy:

  65 successes / 90 episodes
  ```

## Hard-map training and forgetting

- Initial hard-map performance:

  ```text
  48 successes / 90 episodes

  42 collisions / 90 episodes
  ```

- Longer hard-map training improved difficult-scenario performance.

- However, training exclusively on hard maps sometimes reduced performance on earlier benchmarks.

- This effect is known as catastrophic forgetting.

- Example:

  ```text
  Hard benchmark:

  64 successes / 90 episodes

  V3 benchmark:

  51 successes / 60 episodes

  V2 benchmark:

  38 successes / 60 episodes
  ```

- Balanced training and recovery experiments were tested to reduce this trade-off.

## Directional escape features

- Front clearance only indicates that an obstacle exists.

- Additional features were introduced to indicate which side offers more free space.

- Added:

  ```text
  Left escape clearance

  Right escape clearance
  ```

- Directional measurements typically consider side angles between:

  ```text
  25° and 85°
  ```

- Side-sector median clearance is used to estimate usable escape space.

- The final observation contains:

  ```text
  50 LiDAR sectors

  1 front/body clearance

  1 left escape clearance

  1 right escape clearance

  1 normalized goal distance

  1 sin(relative_goal_angle)

  1 cos(relative_goal_angle)

  Total: 56 observations
  ```

- Verified model observation space:

  ```text
  Box(..., shape=(56,), dtype=float32)
  ```

## Reverse and recovery maneuvers

- Reverse motion was added to help escape blocked positions.

- Typical reverse-speed limit:

  ```text
  0.10 m/s
  ```

- Reverse motion is rewarded when the robot is blocked.

- Unnecessary reverse motion in open space is penalized.

## Discrete PPO maneuver space

- The original continuous action space was replaced with:

  ```text
  Discrete(13)
  ```

- PPO chooses one of 13 actuator commands.

- The available maneuvers are:

  ```text
  Index  Maneuver           Linear velocity    Angular velocity

  0      reverse_left       -0.10 m/s          +0.80 rad/s

  1      reverse_straight   -0.10 m/s           0.00 rad/s

  2      reverse_right      -0.10 m/s          -0.80 rad/s

  3      pivot_left          0.00 m/s          +0.80 rad/s

  4      pivot_right         0.00 m/s          -0.80 rad/s

  5      crawl_left         +0.075 m/s         +0.80 rad/s

  6      crawl_right        +0.075 m/s         -0.80 rad/s

  7      slow_straight      +0.10 m/s           0.00 rad/s

  8      forward_left       +0.15 m/s          +0.52 rad/s

  9      forward_right      +0.15 m/s          -0.52 rad/s

  10     gentle_left        +0.20 m/s          +0.28 rad/s

  11     gentle_right       +0.20 m/s          -0.28 rad/s

  12     forward_straight   +0.25 m/s           0.00 rad/s
  ```

- Important:

  - The maneuver table defines available actions.
  - It does not decide which action should be executed.
  - The PPO policy selects the action.
  - Therefore, this remains a pure PPO controller.

## Neural-network architecture

- PPO policy:

  ```text
  ActorCriticPolicy
  ```

- Feature extractor:

  ```text
  FlattenExtractor
  ```

- Input dimension:

  ```text
  56
  ```

- Output:

  ```text
  Categorical probability distribution over 13 actions
  ```

- Base maneuver configuration:

  ```text
  Hidden layers:

  [256, 256]
  ```

- The actor selects the action.

- The critic estimates the value of the current state.

## PPO loss function

- Define the probability ratio:

  ```text
  r_t(theta) =
      pi_theta(a_t | s_t)
      /
      pi_theta_old(a_t | s_t)
  ```

- PPO clipped objective:

  ```text
  L_clip(theta) =
      E[
          min(
              r_t(theta) * A_t,

              clip(
                  r_t(theta),
                  1 - epsilon,
                  1 + epsilon
              )
              *
              A_t
          )
      ]
  ```

- Value-function loss:

  ```text
  L_value =
      E[
          (
              V_theta(s_t)
              -
              R_t
          )²
      ]
  ```

- Entropy term:

  ```text
  H(
      pi_theta(. | s_t)
  )
  ```

- Combined minimization objective:

  ```text
  L_total =
      -L_clip
      +
      c_value * L_value
      -
      c_entropy * H
  ```

- Where:

  ```text
  A_t:
      Estimated advantage.

  epsilon:
      PPO clipping coefficient.

  V_theta:
      Critic value estimate.

  R_t:
      Estimated return.

  c_value:
      Value-loss coefficient.

  c_entropy:
      Entropy coefficient.
  ```

## Generalized advantage estimation

- Temporal-difference error:

  ```text
  delta_t =
      reward_t
      +
      gamma * V(s_t+1)
      -
      V(s_t)
  ```

- Generalized advantage:

  ```text
  A_t =
      delta_t
      +
      gamma * lambda * delta_t+1
      +
      (gamma * lambda)² * delta_t+2
      +
      ...
  ```

## Reward function

- The total reward combines:

  ```text
  reward_total =
      reward_progress
      +
      reward_time
      +
      reward_goal
      +
      reward_collision
      +
      reward_timeout
      +
      reward_stuck
      +
      reward_proximity
      +
      reward_unsafe_speed
      +
      reward_front_unsafe_speed
      +
      reward_escape_turn
      +
      reward_clearance_progress
      +
      reward_escape_reverse
      +
      reward_reverse
  ```

- Goal progress:

  ```text
  reward_progress =
      progress_weight
      *
      (
          previous_goal_distance
          -
          current_goal_distance
      )
  ```

- Time penalty:

  ```text
  reward_time =
      -time_penalty
  ```

- Goal reward:

  ```text
  reward_goal =
      goal_reward
      if goal reached
      else 0
  ```

- Collision penalty:

  ```text
  reward_collision =
      -collision_penalty
      if collision
      else 0
  ```

- Timeout penalty:

  ```text
  reward_timeout =
      -timeout_penalty
      if timeout
      else 0
  ```

- Stuck penalty:

  ```text
  reward_stuck =
      -stuck_penalty
      if stuck
      else 0
  ```

- General obstacle-proximity ratio:

  ```text
  rho =
      max(
          0,
          (
              safety_distance
              -
              minimum_lidar_distance
          )
          /
          safety_distance
      )
  ```

- Obstacle proximity penalty:

  ```text
  reward_proximity =
      -proximity_penalty_weight
      *
      rho²
  ```

- Normalized forward speed:

  ```text
  u_forward =
      max(
          requested_forward_speed,
          0
      )
      /
      maximum_forward_speed
  ```

- Unsafe forward-speed penalty:

  ```text
  reward_unsafe_speed =
      -unsafe_speed_penalty_weight
      *
      u_forward
      *
      rho
  ```

- Front-obstacle ratio:

  ```text
  beta =
      max(
          0,
          (
              front_safety_distance
              -
              front_body_clearance
          )
          /
          front_safety_distance
      )
  ```

- Front-specific unsafe-speed penalty:

  ```text
  reward_front_unsafe_speed =
      -front_unsafe_speed_penalty_weight
      *
      u_forward
      *
      beta
  ```

- Directional escape advantage:

  ```text
  escape_advantage =
      clip(
          (
              left_escape_clearance
              -
              right_escape_clearance
          )
          /
          directional_normalization_distance,
          -1,
          +1
      )
  ```

- Reward turning toward the clearer side:

  ```text
  reward_escape_turn =
      escape_turn_reward_weight
      *
      previous_front_blocked_ratio
      *
      escape_advantage
      *
      (
          requested_angular_speed
          /
          maximum_angular_speed
      )
  ```

- Reward increasing obstacle clearance:

  ```text
  reward_clearance_progress =
      clearance_progress_reward_weight
      *
      previous_front_blocked_ratio
      *
      clearance_progress
  ```

- Reward reversing when blocked:

  ```text
  reward_escape_reverse =
      escape_reverse_reward_weight
      *
      previous_front_blocked_ratio
      *
      normalized_reverse_speed
  ```

- Penalize reversing unnecessarily:

  ```text
  reward_reverse =
      -reverse_penalty_weight
      *
      (
          1
          -
          previous_front_blocked_ratio
      )
      *
      normalized_reverse_speed
  ```

## Safety constraints

- The implementation does not use:

  - Constrained Policy Optimization.
  - A separate Lagrangian constraint.
  - An online safety controller.
  - A heuristic obstacle-avoidance override.

- Safety is encouraged through:

  - Reward shaping.
  - Collision penalties.
  - Stuck penalties.
  - Obstacle-clearance penalties.
  - Limited actuator commands.
  - Collision-based episode termination.

- A conceptual safety-cost function is:

  ```text
  safety_cost =
      collision_penalty * collision_indicator
      +
      proximity_penalty_weight * rho²
      +
      unsafe_speed_penalty_weight
      *
      normalized_forward_speed
      *
      rho
      +
      front_unsafe_speed_penalty_weight
      *
      normalized_forward_speed
      *
      beta
      +
      stuck_penalty * stuck_indicator
  ```

- This cost is incorporated into the reward.

- It is not a separately optimized constrained-MDP objective.

- PPO clipping limits excessively large policy updates:

  ```text
  clip(
      r_t(theta),
      1 - epsilon,
      1 + epsilon
  )
  ```

- `target_kl` can additionally stop overly large updates.

- Neither PPO clipping nor `target_kl` guarantees collision-free navigation.

## Episode termination

- Success:

  ```text
  distance_to_goal <= goal_tolerance

  and no collision
  ```

- Typical goal tolerance:

  ```text
  0.25 m
  ```

- Failure:

  ```text
  Collision.
  ```

- Failure:

  ```text
  Stuck behavior.
  ```

- Timeout:

  ```text
  Maximum episode length reached.
  ```

## Training hyperparameters

- Important PPO parameters:

  ```text
  learning_rate

  n_steps

  batch_size

  n_epochs

  gamma

  gae_lambda

  clip_range

  ent_coef

  vf_coef

  max_grad_norm

  target_kl

  net_arch
  ```

- Verified base V8 maneuver configuration:

  ```text
  learning_rate = 0.000075

  n_steps = 1024

  batch_size = 256

  n_epochs = 5

  gamma = 0.997

  gae_lambda = 0.97

  clip_range = 0.15

  ent_coef = 0.02

  vf_coef = 0.5

  max_grad_norm = 0.5

  target_kl = 0.025

  net_arch = [256, 256]
  ```

- The focused run can override the base values.

- The exact final values must be taken from:

  ```text
  /root/leo_ws/results/rl_v8_pure_ppo/ppo_maneuver_focused_smoke_60k_seed1042_002/resolved_config.json
  ```

## Final selected model

- Selected checkpoint:

  ```text
  /root/leo_ws/results/rl_v8_pure_ppo/ppo_maneuver_focused_smoke_60k_seed1042_002/checkpoints/ppo_leo_60000_steps.zip
  ```

- Checkpoint SHA-256:

  ```text
  3383999ef34da5629fd0d23bcfc6ac4f00baa176b1e6463eabda2974721e2f16
  ```

- Configuration:

  ```text
  /root/leo_ws/src/leo_rl_navigation/config/ppo_lidar_maneuver_focused_v8.yaml
  ```

- Configuration SHA-256:

  ```text
  85dc4a9ddfabdbeb60c9063c3d50ca7328ba9ea19c4d104f4b89e28d222aefb8
  ```

- Verified code commit:

  ```text
  f0a7886b50400103086cc0ab2413ca10ea663b62
  ```

- Recorded checkpoint timesteps:

  ```text
  60000
  ```

- Important:

  - The checkpoint was obtained after 60,000 steps of the final focused run.
  - Earlier training and policy-transfer stages also contributed.
  - It must not be described as a model trained entirely from scratch in only 60,000 total interactions.

## Final hard-benchmark results

- Dataset:

  ```text
  benchmark_v4_hard
  ```

- Split:

  ```text
  validation
  ```

- Evaluation episodes:

  ```text
  90
  ```

- Results:

  ```text
  Successes:
  68 / 90

  Collisions:
  14 / 90

  Timeouts:
  5 / 90

  Stuck:
  3 / 90

  Safety interventions:
  0
  ```

- Rates:

  ```text
  Success rate:
  75.56%

  Collision rate:
  15.56%

  Timeout rate:
  5.56%

  Stuck rate:
  3.33%
  ```

- Difficult demonstration worlds:

  ```text
  World index 8:
  success

  World index 14:
  success

  World index 29:
  success
  ```

- The policy was evaluated with:

  ```text
  safety_shield = false
  ```

## Model-selection trade-off

- Earlier maneuver checkpoint:

  ```text
  69 successes / 90 episodes

  16 collisions / 90 episodes
  ```

- Selected focused checkpoint:

  ```text
  68 successes / 90 episodes

  14 collisions / 90 episodes
  ```

- Interpretation:

  - One fewer successful episode.
  - Two fewer collisions.
  - Successful execution on targeted worlds 8, 14, and 29.

## Evaluation metrics

- Success rate.

- Collision rate.

- Timeout rate.

- Stuck rate.

- Time required to reach the goal.

- Number of simulation steps.

- Travel distance.

- Path length.

- Minimum LiDAR distance.

- Minimum body-relative obstacle clearance.

- Average linear velocity.

- Average angular velocity.

- Number of pivot maneuvers.

- Number of reverse maneuvers.

- Number of slow-forward maneuvers.

- Number of safety-shield interventions.

- Performance across difficulty levels.

- Performance on unseen generated maps.

## Comparison with the heuristic method

- Evaluate both methods using identical:

  - Robot model.
  - LiDAR sensor.
  - Maps.
  - Initial positions.
  - Goal positions.
  - Control frequency.
  - Episode limits.
  - Goal tolerance.
  - Collision definitions.

- A heuristic controller explicitly follows hand-coded rules.

- Example:

  ```text
  IF obstacle ahead:

      turn toward the clearer side

  ELSE:

      move toward the goal
  ```

- The PPO policy selects the maneuver based on learned model parameters.

- Report:

  - Success rate.
  - Collision rate.
  - Navigation time.
  - Path length.
  - Computational cost.
  - Behavior on unseen maps.

## Comparison with multi-robot PPO

- Preserve the same:

  - Observation space.
  - Action space.
  - Reward function.
  - PPO loss.
  - Robot dynamics.
  - Evaluation maps.
  - Safety-shield setting.

- The main experimental difference should be distributed experience collection and communication.

- Possible distributed architecture:

  - Multiple robots collect experience.
  - Each robot performs local PPO learning.
  - Robots exchange policy parameters periodically.
  - No central navigation controller is required.

- Important:

  ```text
  n_envs = 4
  ```

  does not automatically mean a distributed multi-robot system.

- Vectorized environments are multiple simulation instances for collecting experience.

- A distributed multi-robot method requires explicit communication or parameter-sharing between robot workers.

- Compare:

  - Total environment interactions.
  - Interactions per robot.
  - Wall-clock training time.
  - Communication overhead.
  - Convergence speed.
  - Final success rate.
  - Collision rate.
  - Generalization to unseen maps.

## Scientific validity

- Worlds 8, 14, and 29 were repeatedly inspected during development.

- They should be described as targeted development scenarios.

- They should not be presented as untouched test data.

- Final comparison should use:

  - Previously unused test maps.
  - Newly generated random seeds.
  - Identical evaluation settings.
  - Frozen trained policies.

## Extract exact focused-training parameters

```bash
RUN=/root/leo_ws/results/rl_v8_pure_ppo/ppo_maneuver_focused_smoke_60k_seed1042_002

export RUN

python - <<'PY'
import json
import os
from pathlib import Path

path = (
    Path(os.environ["RUN"])
    /
    "resolved_config.json"
)

with path.open(
    encoding="utf-8"
) as stream:
    data = json.load(stream)

for section in (
    "environment",
    "ppo",
    "training",
):
    print()
    print(section.upper())

    for key, value in data[
        "config"
    ][section].items():
        print(
            f"{key}: {value}"
        )

print()
print("RESOLVED TRAINING")

for key, value in data[
    "resolved_training"
].items():
    print(
        f"{key}: {value}"
    )
PY
```
