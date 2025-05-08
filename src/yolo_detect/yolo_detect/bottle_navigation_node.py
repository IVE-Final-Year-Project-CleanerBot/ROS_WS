from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node


class BottleNavigationNode(Node):
    def __init__(self):
        super().__init__('bottle_navigation_node')

        # 初始化电机控制发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 发布是否到达瓶子面前的状态
        self.bottle_at_target_publisher = self.create_publisher(Bool, '/bottle_at_target', 10)

        # 订阅瓶子检测结果和位置
        self.create_subscription(Bool, '/bottle_detected', self.bottle_detected_callback, 10)
        self.create_subscription(Point, '/bottle_position', self.bottle_position_callback, 10)

        # 可调整参数
        self.x_tolerance = 50  # 中心点 x 的容忍范围（像素）
        self.y_threshold_factor = 1.5 / 3  # 中心点 y 的阈值比例（图像高度的 1/2）
        self.linear_speed = 0.13  # 线速度
        self.fixed_angular_speed = 0.3  # 固定角速度

        self.bottle_detected = False
        self.bottle_position = None

        self.get_logger().info("BottleNavigationNode has been started.")

    def bottle_detected_callback(self, msg):
        """处理瓶子检测结果"""
        self.bottle_detected = msg.data

    def bottle_position_callback(self, msg):
        """处理瓶子位置"""
        self.bottle_position = msg
        if self.bottle_detected and self.bottle_position:
            self.navigate_to_bottle()

    def navigate_to_bottle(self):
        """根据瓶子位置控制机器人移动"""
        if not self.bottle_detected or not self.bottle_position:
            self.get_logger().warn("Bottle not detected or position unavailable. Stopping navigation.")
            return
        twist = Twist()

        # 计算 x 和 y 偏移量
        offset_x = self.bottle_position.x - 320  # 假设图像宽度为 640
        y_threshold = 480 * self.y_threshold_factor  # 假设图像高度为 480

        # 阶段 1：水平对齐
        if abs(offset_x) > self.x_tolerance:
            twist.angular.z = self.fixed_angular_speed if offset_x < 0 else -self.fixed_angular_speed
            twist.linear.x = 0.0
        # 阶段 2：向前移动
        elif self.bottle_position.y < y_threshold:
            twist.angular.z = 0.0
            twist.linear.x = self.linear_speed
        # 阶段 3：停止并发布到达状态
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.bottle_at_target_publisher.publish(Bool(data=True))
            self.get_logger().info("Bottle is at target position.")

        # 发布速度指令
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Driving to target: linear.x={twist.linear.x}, angular.z={twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = BottleNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()