from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

env = Go1Env()
check_env(env)

model = PPO('CnnPolicy', env, verbose=1)
model.learn(total_timesteps=100000)
model.save("ppo_go1_stability")

# 在训练之后，加载模型并测试
model = PPO.load("ppo_go1_stability")
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    if dones:
        obs = env.reset()
