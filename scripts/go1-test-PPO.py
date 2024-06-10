from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from go1_env import Go1Env

# Create the environment
env = Go1Env()

# Check the environment
check_env(env)

# Create the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=100000)

# Save the model
model.save("ppo_go1")

# To load the model and continue training or evaluation
# model = PPO.load("ppo_go1", env=env)
