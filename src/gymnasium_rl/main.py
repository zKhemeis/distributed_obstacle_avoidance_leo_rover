from stable_baselines3 import PPO
from leo_rover_env import LeoRover2DEnv

model = PPO.load("leo_ppo")
env = LeoRover2DEnv(render_mode="human")

for ep in range(10):
    obs, _ = env.reset()
    done = False
    while not done:
        action, _ = model.predict(obs, deterministic=True)   # greedy, no exploration
        obs, reward, terminated, truncated, info = env.step(action)
        # env.render()
        done = terminated or truncated
    print(f"episode {ep}: success={info['is_success']} final_dist={info['dist']:.2f}")
env.close()