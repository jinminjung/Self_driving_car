#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)

        self.get_logger().info('Subscribing Image')
        self.bridge = CvBridge()

    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    vision_subscriber = VisionNode()

    rclpy.spin(vision_subscriber)

    vision_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()