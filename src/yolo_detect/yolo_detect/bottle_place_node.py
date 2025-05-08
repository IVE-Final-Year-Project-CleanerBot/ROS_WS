from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node


class BottlePlaceNode(Node):
    def __init__(self):
        super().__init__('bottle_place_node')

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

        # 发布放置完成状态
        self.bottle_place_done_publisher = self.create_publisher(Bool, '/bottle_place_done', 10)

        # 订阅是否到达放置位置的状态
        self.create_subscription(Bool, '/start_reached', self.start_reached_callback, 10)

        # 订阅瓶子检测状态
        self.create_subscription(Bool, '/bottle_detected', self.bottle_detected_callback, 10)

        self.start_reached = False
        self.bottle_detected = False  # 标志变量，记录是否检测到瓶子
        self.place_completed = False  # 标志变量，记录是否已经完成放置
        self.get_logger().info("BottlePlaceNode has been started.")

    def bottle_detected_callback(self, msg):
        """处理瓶子检测状态"""
        self.bottle_detected = msg.data
        if not self.bottle_detected:
            self.get_logger().info("No bottle detected. Skipping place operation.")

    def start_reached_callback(self, msg):
        """处理是否到达放置位置的状态"""
        self.start_reached = msg.data
        if self.start_reached and self.bottle_detected and not self.place_completed:
            self.place_bottle()
        elif self.start_reached and not self.bottle_detected:
            self.get_logger().info("Start position reached but no bottle detected. Skipping place operation.")
            self.reset_flags()

    def reset_flags(self):
        """重置标志变量"""
        self.start_reached = False
        self.bottle_detected = False
        self.place_completed = False
        self.get_logger().info("All flags have been reset for the next task.")

    def place_bottle(self):
        """控制机械臂放置瓶子"""
        self.arm_command_publisher.publish(String(data="move_to_place"))
        self.get_logger().info("Sent place command to the arm.")

        # 模拟放置完成，发布放置完成状态
        self.bottle_place_done_publisher.publish(Bool(data=True))
        self.get_logger().info("Bottle place completed.")

        # 设置放置完成标志，防止重复执行
        self.place_completed = True


def main(args=None):
    rclpy.init(args=args)
    node = BottlePlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()