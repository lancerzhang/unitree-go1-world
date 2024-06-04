import gym
from gym import spaces
import numpy as np
import rospy
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from unitree_legged_msgs.msg import MotorCmd, MotorState
import cv2


class Go1Env(gym.Env):
    def __init__(self):
        super(Go1Env, self).__init__()
        self.bridge = CvBridge()
        rospy.init_node('go1_rl_env', anonymous=True)
        rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/trunk_imu', Imu, self.imu_callback)

        self.publishers = {name: rospy.Publisher(f'/go1_gazebo/{name}_controller/command', MotorCmd, queue_size=10)
                           for group in ['calf', 'hip', 'thigh']
                           for name in [f'{group}_{i}' for i in ['FR', 'FL', 'RR', 'RL']]}

        self.action_space = spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255, shape=(64, 64, 3), dtype=np.uint8)

        self.image = None
        self.imu_data = None
        self.state = None
        self.start_time = rospy.get_time()

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        resized_image = cv2.resize(cv_image, (64, 64))
        self.image = resized_image

    def imu_callback(self, data):
        self.imu_data = data

    def step(self, action):
        assert self.image is not None, "No image data received yet!"
        assert self.imu_data is not None, "No IMU data received yet!"

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
        self.start_time = rospy.get_time()
        rospy.wait_for_message('/camera_face/color/image_raw', Image)
        rospy.wait_for_message('/imu', Imu)
        return self.image

    def calculate_reward(self):
        # 姿态稳定性奖励
        roll, pitch, yaw = self.get_orientation_from_imu(self.imu_data)
        stability_reward = -abs(roll) - abs(pitch)  # Roll和Pitch越接近0，奖励越高

        # 前进速度奖励
        velocity_reward = 1.0  # 可根据前进速度设计，例如 target_velocity - abs(current_velocity - target_velocity)

        # 轨迹跟踪奖励
        tracking_reward = 1.0  # 可根据轨迹偏离情况设计

        return stability_reward + velocity_reward + tracking_reward

    def check_done(self):
        roll, pitch, _ = self.get_orientation_from_imu(self.imu_data)
        if abs(roll) > 0.5 or abs(pitch) > 0.5:  # 设定的姿态角度阈值
            return True
        if rospy.get_time() - self.start_time > 30:  # 时间限制
            return True
        return False

    def get_orientation_from_imu(self, imu_data):
        # 从IMU数据中提取Roll, Pitch, Yaw
        orientation_q = imu_data.orientation
        # 转换四元数到欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(orientation_q)
        return roll, pitch, yaw

    def quaternion_to_euler(self, q):
        # 四元数转欧拉角
        x, y, z, w = q.x, q.y, q.z, q.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw
