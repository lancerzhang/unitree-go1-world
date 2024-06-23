#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        scales = [1, 2, 4, 8, 16, 32]
        for scale in scales:
            rospy.Subscriber(f'/camera_face/color/image_resized_{scale}x', Image,
                             self.resized_image_callback, callback_args=scale)

    def resized_image_callback(self, msg, scale):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_path = f"{scale}x_image.jpg"
        cv2.imwrite(image_path, cv_image)


if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    subscriber = ImageSubscriber()
    rospy.spin()
