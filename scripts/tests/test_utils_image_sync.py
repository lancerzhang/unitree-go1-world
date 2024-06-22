# test_image_sync.py

import unittest
from collections import deque
from unittest.mock import MagicMock

from sensor_msgs.msg import Image

from utils import get_synced_images


class TestImageSync(unittest.TestCase):
    def setUp(self):
        self.scales = [1, 2, 4, 8, 16, 32]
        self.image_queues = {scale: deque(maxlen=2) for scale in self.scales}
        self.last_image_time = None

    def create_image(self, stamp):
        image = MagicMock(spec=Image)
        image.header.stamp = stamp
        return image

    def test_no_images(self):
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNone(synced_images)
        self.assertIsNone(new_time)

    def test_insufficient_images(self):
        self.image_queues[1].append(self.create_image(1))
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNone(synced_images)
        self.assertIsNone(new_time)

    def test_single_set_of_synced_images(self):
        for scale in self.scales:
            self.image_queues[scale].append(self.create_image(1))
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNotNone(synced_images)
        self.assertEqual(new_time, 1)
        self.assertEqual(len(synced_images), len(self.scales))

    def test_multiple_sets_of_synced_images(self):
        for scale in self.scales:
            self.image_queues[scale].append(self.create_image(1))
            self.image_queues[scale].append(self.create_image(2))
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNotNone(synced_images)
        self.assertEqual(new_time, 1)
        self.assertEqual(len(synced_images), len(self.scales))

    def test_ignore_old_images(self):
        self.last_image_time = 1
        for scale in self.scales:
            self.image_queues[scale].append(self.create_image(1))
            self.image_queues[scale].append(self.create_image(2))
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNotNone(synced_images)
        self.assertEqual(new_time, 2)
        self.assertEqual(len(synced_images), len(self.scales))

    def test_no_synced_images_after_last_time(self):
        self.last_image_time = 2
        for scale in self.scales:
            self.image_queues[scale].append(self.create_image(1))
            self.image_queues[scale].append(self.create_image(2))
        synced_images, new_time = get_synced_images(self.image_queues, self.last_image_time, self.scales)
        self.assertIsNone(synced_images)
        self.assertEqual(new_time, 2)


if __name__ == '__main__':
    unittest.main()
