from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node


class BottlePickupNode(Node):
    def __init__(self):
        super().__init__('bottle_pickup_node')

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

        # 发布拾取完成状态
        self.bottle_pickup_done_publisher = self.create_publisher(Bool, '/bottle_pickup_done', 10)

        # 订阅是否到达瓶子面前的状态
        self.create_subscription(Bool, '/bottle_at_target', self.bottle_at_target_callback, 10)

        self.bottle_at_target = False
        self.pickup_completed = False  # 标志变量，记录是否已经完成拾取
        self.get_logger().info("BottlePickupNode has been started.")

    def bottle_at_target_callback(self, msg):
        """处理是否到达瓶子面前的状态"""
        self.bottle_at_target = msg.data
        if self.bottle_at_target and not self.pickup_completed:
            self.pick_up_bottle()
        elif not self.bottle_at_target:
            self.get_logger().info("Bottle not at target position. Skipping pick-up operation.")
            self.reset_flags()

    def reset_flags(self):
        """重置标志变量"""
        self.bottle_at_target = False
        self.pickup_completed = False
        self.get_logger().info("All flags have been reset for the next task.")

    def pick_up_bottle(self):
        """控制机械臂拾取瓶子"""
        self.arm_command_publisher.publish(String(data="move_to_pick"))
        self.get_logger().info("Sent pick-up command to the arm.")

        # 模拟拾取完成，发布拾取完成状态
        self.bottle_pickup_done_publisher.publish(Bool(data=True))
        self.get_logger().info("Bottle pickup completed.")

        # 设置拾取完成标志，防止重复执行
        self.pickup_completed = True


def main(args=None):
    rclpy.init(args=args)
    node = BottlePickupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()