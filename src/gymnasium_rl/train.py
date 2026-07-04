"""
Train an obstacle-avoidance + goal-reaching policy on LeoRover2DEnv with PPO.

    pip install "stable-baselines3[extra]" gymnasium matplotlib

PPO is a solid, robust default for continuous control and is the algorithm to
reach for first (don't sink time into picking the "perfect" one early). SAC is a
reasonable alternative if you want more sample efficiency.
"""

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

from leo_rover_env import LeoRover2DEnv


def make_env():
    # Turn on small lidar_noise once training works -> helps sim-to-real transfer.
    return Monitor(LeoRover2DEnv(n_obstacles=5, lidar_noise=0.05))


if __name__ == "__main__":
    # 8 parallel envs makes PPO converge much faster.
    vec_env = make_vec_env(make_env, n_envs=8)

    model = PPO(
        "MlpPolicy",
        vec_env,
        n_steps=1024,
        batch_size=256,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=0.01,
        learning_rate=3e-4,
        verbose=1,
        tensorboard_log="./tb_logs",
    )
    model.learn(total_timesteps=1_000_000)
    model.save("leo_ppo")
    print("saved -> leo_ppo.zip")

    # ---- quick visual evaluation of the trained policy ----
    env = LeoRover2DEnv(render_mode="human")
    for ep in range(5):
        obs, _ = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            env.render()
            done = terminated or truncated
        print(f"episode {ep}: success={info['is_success']}  final_dist={info['dist']:.2f}")
    env.close()