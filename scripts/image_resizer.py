#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageResizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)

        # 读取参数
        self.scale_factors = rospy.get_param('~scale_factors', [1, 2, 4, 8, 16, 32])
        self.publishers = []

        # 为每个缩放倍数创建一个发布者
        for scale in self.scale_factors:
            pub = rospy.Publisher(f'/camera_face/color/image_resized_{scale}x', Image, queue_size=1)
            self.publishers.append((scale, pub))

        # 创建线程池
        self.executor = ThreadPoolExecutor(max_workers=len(self.scale_factors))

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 并行处理图像缩放和发布
        futures = []
        for scale, pub in self.publishers:
            futures.append(self.executor.submit(self.resize_and_publish, cv_image, scale, pub))

        # 等待所有线程完成
        for future in futures:
            future.result()

    def resize_and_publish(self, cv_image, scale, pub):
        if scale == 1:
            resized_image = cv_image
        else:
            resized_image = cv2.resize(cv_image, (cv_image.shape[1] // scale, cv_image.shape[0] // scale))
        resized_image_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
        pub.publish(resized_image_msg)


if __name__ == '__main__':
    rospy.init_node('image_resizer', anonymous=True)
    resizer = ImageResizer()
    rospy.spin()
