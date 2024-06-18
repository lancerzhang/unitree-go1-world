#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.resized_1x_sub = rospy.Subscriber('/camera_face/color/image_resized_1x', Image,
                                               self.resized_image_callback, callback_args=1)
        self.resized_2x_sub = rospy.Subscriber('/camera_face/color/image_resized_2x', Image,
                                               self.resized_image_callback, callback_args=2)
        self.resized_4x_sub = rospy.Subscriber('/camera_face/color/image_resized_4x', Image,
                                               self.resized_image_callback, callback_args=4)
        self.resized_8x_sub = rospy.Subscriber('/camera_face/color/image_resized_8x', Image,
                                               self.resized_image_callback, callback_args=8)
        self.resized_16x_sub = rospy.Subscriber('/camera_face/color/image_resized_16x', Image,
                                                self.resized_image_callback, callback_args=16)

    def resized_image_callback(self, msg, scale):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_path = f"{scale}x_image.jpg"
        cv2.imwrite(image_path, cv_image)


if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    subscriber = ImageSubscriber()
    rospy.spin()
