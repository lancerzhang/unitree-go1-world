#!/usr/bin/env python

import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2


class CameraProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_count = 0
        self.start_time = time.time()
        self.last_print_time = time.time()

        rospy.init_node('camera_processor', anonymous=True)
        rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Get image resolution
        height, width, _ = cv_image.shape
        # rospy.loginfo(f"Image resolution: {width}x{height}")

        # Save the image to disk
        image_path = f"saved_image.jpg"
        cv2.imwrite(image_path, cv_image)
        # rospy.loginfo(f"Image saved to: {image_path}")

        # Update image count
        self.image_count += 1

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # Print statistics every 2 seconds
        if time.time() - self.last_print_time > 2:
            avg_images_per_sec = self.image_count / elapsed_time
            rospy.loginfo(f"Elapsed time: {elapsed_time:.2f} seconds")
            rospy.loginfo(f"Total images received: {self.image_count}")
            rospy.loginfo(f"Average images per second: {avg_images_per_sec:.2f}")
            self.last_print_time = time.time()


if __name__ == '__main__':
    try:
        CameraProcessor()
    except rospy.ROSInterruptException:
        pass
