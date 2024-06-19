import rospy
from std_srvs.srv import Empty

from env_utils import reset_env


class Go1Env():
    def __init__(self):
        super(Go1Env, self).__init__()
        rospy.init_node('go1_rl_env', anonymous=True)
        self.reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    def reset(self, seed=None, options=None):
        return reset_env(self)

    def step(self):
        terminated = False
        return terminated


if __name__ == "__main__":
    rospy.init_node('go1_env_test')
    env = Go1Env()
    rospy.spin()  # Keep the node running
