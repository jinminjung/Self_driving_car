import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SimpleHatchbackDriver(Node):
    def __init__(self):
        super().__init__('simple_hatchback_driver')
        self.publisher = self.create_publisher(Twist, '/hatchback/hatchback_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.drive_loop)
        self.start_time = self.get_clock().now()
        self.is_stopped = False  # 중복 정지 방지 플래그

    def drive_loop(self):
        if self.is_stopped:
            return  # 타이머가 멈춘 상태에서 실행되지 않도록 방지

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        msg = Twist()

        # 처음 1.5초: 직진
        if elapsed_time < 2.1:
            msg.linear.x = 2.0
            msg.angular.z = 0.0

        # 1.5초~4.3초: 90도 우회전
        elif elapsed_time < 4.8:
            msg.linear.x = 1.5  # 회전 중 속도 감소
            msg.angular.z = -math.pi / 4.0  # 일정한 각속도로 회전

        # 4.3초~10.0초: 직진
        elif elapsed_time < 10.0:
            msg.linear.x = 4.0
            msg.angular.z = 0.0

        # 10.0초 이후: 완전 정지
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.is_stopped = True  # 중복 실행 방지 플래그 설정
            self.timer.cancel()  # 타이머 종료

        # 속도 명령 퍼블리시
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver = SimpleHatchbackDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
