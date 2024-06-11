import rospy
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from sensor_msgs.msg import Imu
from unitree_legged_msgs.msg import MotorCmd, MotorState
from transforms3d.euler import quat2euler

go1_Hip_max = 1.047
go1_Hip_min = -1.047
go1_Thigh_max = 2.966
go1_Thigh_min = -0.663
go1_Calf_max = -0.837
go1_Calf_min = -2.721

class Go1Env(gym.Env):
    def __init__(self):
        super(Go1Env, self).__init__()
        rospy.init_node('go1_gym_env', anonymous=True)

        self.joint_groups = {
            'calf': ['FR_calf', 'FL_calf', 'RR_calf', 'RL_calf'],
            'hip': ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip'],
            'thigh': ['FR_thigh', 'FL_thigh', 'RR_thigh', 'RL_thigh']
        }
        self.publishers = {name: rospy.Publisher(f'/go1_gazebo/{name}_controller/command', MotorCmd, queue_size=10) for
                           group in self.joint_groups.values() for name in group}
        self.motor_states = {name: MotorState() for group in self.joint_groups.values() for name in group}

        for group in self.joint_groups.values():
            for name in group:
                rospy.Subscriber(f'/go1_gazebo/{name}_controller/state', MotorState, self.create_callback(name))

        rospy.Subscriber('/trunk_imu', Imu, self.imu_callback)

        self.rate = rospy.Rate(500)  # 500 Hz

        # Define action and observation space
        # Actions: desired angles for all 12 joints
        self.action_space = spaces.Box(low=np.array([go1_Hip_min, go1_Thigh_min, go1_Calf_min] * 4),
                                       high=np.array([go1_Hip_max, go1_Thigh_max, go1_Calf_max] * 4),
                                       dtype=np.float32)

        # Observations: motor states (position, velocity) + IMU data
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12 * 2 + 10,), dtype=np.float32)

        self.current_imu = None
        self.flipped_threshold = 1.0  # Threshold to determine if the robot is flipped

    def create_callback(self, name):
        def callback(msg):
            self.motor_states[name] = msg

        return callback

    def imu_callback(self, msg):
        self.current_imu = msg

    def reset(self, seed=None, options=None):
        # Reset the environment to an initial state
        super().reset(seed=seed)
        rospy.sleep(1.0)
        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action):
        # Apply action
        for i, group in enumerate(self.joint_groups.values()):
            for j, name in enumerate(group):
                self.publishers[name].publish(self.create_motor_cmd(10, action[i * 3 + j], 0.0, 0.0, 300.0, 15.0))

        self.rate.sleep()

        # Get observation
        obs = self._get_obs()

        # Calculate reward
        reward = self._compute_reward(obs, action)

        # Check if done
        terminated = self._is_flipped()
        truncated = False

        # Additional info
        info = {}

        if terminated:
            obs, info = self.reset()

        return obs, float(reward), terminated, truncated, info

    def _get_obs(self):
        obs = []
        for group in self.joint_groups.values():
            for name in group:
                state = self.motor_states[name]
                obs.extend([state.q, state.dq])

        if self.current_imu:
            imu_data = self.current_imu
            obs.extend([imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w])
            obs.extend([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
            obs.extend([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
            rpy = quat2euler([imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z])
            obs.extend(rpy)
        else:
            obs.extend([0.0] * 4)  # quaternion
            obs.extend([0.0] * 3)  # gyroscope
            obs.extend([0.0] * 3)  # accelerometer
            obs.extend([0.0] * 3)  # rpy

        # Ensure the observation array has the correct length
        return np.array(obs, dtype=np.float32)[:34]

    def _compute_reward(self, obs, action):
        # Define a reward function
        return -np.sum(np.square(action))  # Simple reward for minimizing action effort

    def _is_flipped(self):
        if self.current_imu:
            rpy = quat2euler([self.current_imu.orientation.w, self.current_imu.orientation.x, self.current_imu.orientation.y, self.current_imu.orientation.z])
            rospy.loginfo(f'rpy {rpy}')
            roll, pitch, yaw = rpy
            if abs(roll) > self.flipped_threshold or abs(pitch) > self.flipped_threshold:
                rospy.loginfo(f'is_flipped')
                return True
        return False

    def create_motor_cmd(self, mode, q, dq, tau, Kp, Kd):
        motor_cmd = MotorCmd()
        motor_cmd.mode = mode
        motor_cmd.q = q
        motor_cmd.dq = dq
        motor_cmd.tau = tau
        motor_cmd.Kp = Kp
        motor_cmd.Kd = Kd
        return motor_cmd

if __name__ == "__main__":
    rospy.init_node('go1_env_test')
    env = Go1Env()
    rospy.spin()  # Keep the node running
