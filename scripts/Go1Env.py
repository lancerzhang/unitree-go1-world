import gym
from gym import spaces
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from unitree_legged_msgs.msg import MotorCmd, MotorState


class Go1Env(gym.Env):
    def __init__(self):
        super(Go1Env, self).__init__()
        self.bridge = CvBridge()
        rospy.init_node('go1_rl_env', anonymous=True)
        rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)

        self.publishers = {name: rospy.Publisher(f'/go1_gazebo/{name}_controller/command', MotorCmd, queue_size=10)
                           for group in ['calf', 'hip', 'thigh']
                           for name in [f'{group}_{i}' for i in ['FR', 'FL', 'RR', 'RL']]}

        self.action_space = spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)

        self.image = None
        self.state = None

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def step(self, action):
        assert self.image is not None, "No image data received yet!"

        # 将动作映射到电机指令
        for i, name in enumerate(self.publishers.keys()):
            motor_cmd = MotorCmd()
            motor_cmd.mode = 10
            motor_cmd.q = action[i]
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0
            motor_cmd.Kp = 300.0 if 'calf' in name else 180.0
            motor_cmd.Kd = 15.0 if 'calf' in name else 8.0
            self.publishers[name].publish(motor_cmd)

        # 计算奖励
        reward = self.calculate_reward()

        # 更新状态
        self.state = self.image

        # 判断是否结束
        done = self.check_done()

        return self.state, reward, done, {}

    def reset(self):
        self.image = None
        self.state = None
        rospy.wait_for_message('/camera_face/color/image_raw', Image)
        return self.image

    def calculate_reward(self):
        # 自定义奖励函数
        return 0.0

    def check_done(self):
        # 自定义终止条件
        return False
