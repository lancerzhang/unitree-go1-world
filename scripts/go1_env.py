import rospy
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from unitree_legged_msgs.msg import MotorCmd, MotorState, IMU
from std_srvs.srv import Empty

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

        rospy.Subscriber('/trunk_imu', IMU, self.imu_callback)

        self.rate = rospy.Rate(500)  # 500 Hz

        # Define action and observation space
        # Actions: desired angles for all 12 joints
        self.action_space = spaces.Box(low=np.array([go1_Hip_min, go1_Thigh_min, go1_Calf_min] * 4),
                                       high=np.array([go1_Hip_max, go1_Thigh_max, go1_Calf_max] * 4),
                                       dtype=np.float32)

        # Observations: motor states (position, velocity) + IMU data
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12 * 2 + 10,), dtype=np.float32)

        self.current_imu = None

        # Initialize Gazebo reset service
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def create_callback(self, name):
        def callback(msg):
            self.motor_states[name] = msg

        return callback

    def imu_callback(self, msg):
        self.current_imu = msg

    def reset(self, seed=None, options=None):
        # Reset the environment to an initial state
        super().reset(seed=seed)
        self.reset_simulation()  # Reset the Gazebo simulation

        # Ensure ROS time is stable after reset
        stable_time = False
        while not stable_time:
            try:
                rospy.sleep(0.1)
                stable_time = True
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                stable_time = False

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
        terminated = self._is_fallen()
        truncated = False

        if terminated:
            reward -= 100  # Penalize for falling
            self.reset()  # Reset the environment

        # Additional info
        info = {}

        return obs, float(reward), terminated, truncated, info

    def _get_obs(self):
        obs = []
        for group in self.joint_groups.values():
            for name in group:
                state = self.motor_states[name]
                obs.extend([state.q, state.dq])

        if self.current_imu:
            imu_data = self.current_imu
            obs.extend(imu_data.quaternion)
            obs.extend(imu_data.gyroscope)
            obs.extend(imu_data.accelerometer)
            obs.extend(imu_data.rpy)
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

    def _is_fallen(self):
        # Check if the robot has fallen based on IMU data
        if self.current_imu:
            roll, pitch = self.current_imu.rpy[:2]
            if abs(roll) > 1.0 or abs(pitch) > 1.0:  # Thresholds for determining if the robot has fallen
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
