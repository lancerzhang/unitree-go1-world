import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from collections import deque

from env_utils import reset_env
from utils import get_synced_images


class Go1Env:
    def __init__(self):
        super(Go1Env, self).__init__()
        rospy.init_node('go1_env', anonymous=True)
        self.reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        # Initialize image subscribers and queues for each scale
        self.scales = [1, 2, 4, 8, 16, 32]
        self.last_image_time = None
        self.image_queues = {scale: deque(maxlen=2) for scale in self.scales}
        for scale in self.scales:
            rospy.Subscriber(f'/camera_face/color/image_resized_{scale}x', Image, self.image_callback,
                             callback_args=scale)

    def image_callback(self, msg, scale):
        self.image_queues[scale].append(msg)

    def step(self):
        synced_images, self.last_image_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        if synced_images is not None:
            # Process images here
            pass
        terminated = False
        return terminated

    def reset(self, seed=None, options=None):
        return reset_env(self)


if __name__ == "__main__":
    rospy.init_node('go1_env_test')
    env = Go1Env()
    rospy.spin()  # Keep the node running
