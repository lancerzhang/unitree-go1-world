# 初始化环境
from go1_env import Go1Env

env = Go1Env()

# 重置环境，获取初始观察
observation = env.reset()

total_episodes = 10
for episode in range(total_episodes):
    observation = env.reset()
    done = False
    while not done:
        # 选择动作（这里假设是随机选择动作）
        action = env.action_space.sample()

        # 在环境中采取动作，获取新的状态和奖励
        next_observation, reward, done, info = env.step(action)

        # 这里进行算法的学习和更新逻辑

        # 更新当前状态
        observation = next_observation

    # 记录或打印回合结果（可选）
    print(f"Episode {episode} finished")
