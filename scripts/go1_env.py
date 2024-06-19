import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from collections import deque
import threading

from env_utils import reset_env


class Go1Env():
    def __init__(self):
        super(Go1Env, self).__init__()
        rospy.init_node('go1_rl_env', anonymous=True)
        self.reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # Initialize image subscribers and queues for each scale
        self.scales = [1, 2, 4, 8, 16, 32]
        self.image_queues = {scale: deque(maxlen=2) for scale in self.scales}
        self.image_locks = {scale: threading.Lock() for scale in self.scales}
        for scale in self.scales:
            rospy.Subscriber(f'/camera_face/color/image_resized_{scale}x', Image, self.image_callback,
                             callback_args=scale)

        self.last_image_time = None

    def image_callback(self, msg, scale):
        with self.image_locks[scale]:
            self.image_queues[scale].append(msg)

    def get_synced_images(self):
        synced_images = {}
        for scale in self.scales:
            with self.image_locks[scale]:
                if not self.image_queues[scale]:
                    return None  # Skip step if any queue is empty
                for image in self.image_queues[scale]:
                    if self.last_image_time is None or image.header.stamp > self.last_image_time:
                        synced_images[scale] = image
                        break
                if scale not in synced_images:
                    return None  # Skip step if no new image found for any scale
        if len(synced_images) == len(self.scales):
            self.last_image_time = synced_images[self.scales[0]].header.stamp
            return synced_images
        return None

    def step(self):
        synced_images = self.get_synced_images()
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
