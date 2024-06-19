import rospy

from example_go1_env import Go1Env


class CustomTrainingAlgorithm:
    def __init__(self):
        self.env = Go1Env()
        self.rate = rospy.Rate(25)  # 25 Hz

    def train(self, total_episodes):
        for episode in range(total_episodes):
            observation = self.env.reset()
            done = False
            while not done:
                # 选择动作（这里假设是随机选择动作）
                action = self.env.action_space.sample()

                # 在环境中采取动作，获取新的状态和奖励
                next_observation, reward, done, info = self.env.step(action)

                # 这里进行算法的学习和更新逻辑

                # 更新当前状态
                observation = next_observation

                # 控制循环频率为25Hz
                self.rate.sleep()

            # 记录或打印回合结果（可选）
            print(f"Episode {episode} finished")


if __name__ == "__main__":
    rospy.init_node('custom_training_algorithm_node')
    trainer = CustomTrainingAlgorithm()
    trainer.train(total_episodes=100)
