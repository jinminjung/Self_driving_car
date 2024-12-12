#!/usr/bin/env python3

import cv2
import rclpy
import time
import subprocess  # 추가: 외부 프로세스 실행을 위한 모듈

from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from .Drive_Bot import Car, Debugging


class Video_feed_in(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 20)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        
        # lidar scan subscription 
        self.subscriber2 = self.create_subscription(LaserScan, 'lidar/lidar_scan', self.lidar_distance, 10)

        self.velocity = Twist()
        self.bridge = CvBridge()  # converting ros images to opencv data
        self.Debug = Debugging()
        self.Car = Car()

        self.min_range = 20.0
        self.start_time = 0
        self.count = 0
        self.obstacle_detected = False
        self.driver_triggered = False  # `SimpleHatchbackDriver` 실행 여부

    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)

    def process_data(self, data):
        """Processes the data stream from the sensor (camera) and passes on to the
           Self Drive Algorithm which computes and executes the appropriate control
           (Steering and speed) commands.

        Args:
            data (img_msg): image data from the camera received as a ros message
        """
        self.Debug.setDebugParameters()
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # performing conversion

        if self.min_range < 12:  # 장애물 감지
            if not self.obstacle_detected:
                self.start_time = time.time()  # 정지 타이머 시작
                self.obstacle_detected = True
                self.get_logger().info("Obstacle detected. Stopping and waiting for 2 seconds.")

        # 장애물 감지 후 2초 대기
        if self.obstacle_detected and not self.driver_triggered:
            if time.time() - self.start_time > 2:  # 2초 후 실행
                self.driver_triggered = True
                self.run_hatchback_driver()
                self.get_logger().info("SimpleHatchbackDriver executed.")
                return  # 이후 로직 실행하지 않음

        # 장애물 감지 시 차량 정지
        if self.obstacle_detected and not self.driver_triggered:
            self.Car.stop()
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0
            self.publisher.publish(self.velocity)
            return

        # 정상 주행
        Angle, Speed, img = self.Car.driveCar(frame, self.min_range)
        self.velocity.angular.z = Angle
        self.velocity.linear.x = Speed

        cv2.imshow("Frame", img)
        cv2.waitKey(1)

    def lidar_distance(self, data):
        """Processes LiDAR distance data and updates the minimum range."""
        ranges = data.ranges  # 거리 값 리스트
        if ranges:
            self.min_range = min(ranges)  # 최단 거리
            self.get_logger().info(f"min_range: {self.min_range}")

    def run_hatchback_driver(self):
        """Run the SimpleHatchbackDriver node as a subprocess."""
        subprocess.Popen(['python3', '/home/jinmin/auto_ws/src/self_driving_car_pkg/self_driving_car_pkg/hatchback_node.py'])  # SimpleHatchbackDriver 실행


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Video_feed_in()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
