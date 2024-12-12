#!/usr/bin/env python3

import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
import signal

from .Drive_Bot import Car, Debugging


class VideoFeedRecorder(Node):
    def __init__(self):
        super().__init__('record_node')

        # Subscribers and publishers
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5; self.timer = self.create_timer(timer_period, self.send_cmd_vel)

        # Twist message
        self.velocity = Twist()

        # Utilities
        self.bridge = CvBridge()  # For converting ROS images to OpenCV format
        self.Debug = Debugging()
        self.Car = Car()

        # Video writer setup
        self.video_writer = None
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 20.0
        self.video_filename = "recorded_video.avi"

        # Graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown)

    def setup_video_writer(self):
        """Initialize the video writer."""
        self.video_writer = cv2.VideoWriter(
            self.video_filename,
            cv2.VideoWriter_fourcc(*'XVID'),
            self.fps,
            (self.frame_width, self.frame_height)
        )

    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)

    def process_data(self, data):
        """Processes the data stream from the sensor (camera) and passes it to
        the self-driving algorithm for control commands.

        Args:
            data (Image): Image data from the camera received as a ROS message.
        """
        self.Debug.setDebugParameters()

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Get angle, speed, and processed frame
        Angle, Speed, img = self.Car.driveCar(frame)

        # Update velocity
        self.velocity.angular.z = Angle
        self.velocity.linear.x = Speed

        # Initialize video writer if not already initialized
        if self.video_writer is None:
            self.frame_height, self.frame_width = img.shape[:2]
            self.setup_video_writer()

        # Write frame to video
        self.video_writer.write(img)

        # Display frame
        cv2.imshow("Frame", img)
        cv2.waitKey(1)

    def shutdown(self, signum, frame):
        """Handles the SIGINT signal (Ctrl+C) and releases resources."""
        self.get_logger().info("Shutting down gracefully...")
        
        # Release video writer
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved as {self.video_filename}")

        # Destroy OpenCV windows
        cv2.destroyAllWindows()

        # Shutdown ROS node
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    video_recorder = VideoFeedRecorder()
    rclpy.spin(video_recorder)


if __name__ == '__main__':
    main()
