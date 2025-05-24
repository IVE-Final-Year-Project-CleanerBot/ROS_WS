from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node

class BottlePickupNode(Node):
    def __init__(self):
        super().__init__('bottle_pickup_node')

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

        # 订阅是否到达瓶子面前的状态
        self.create_subscription(Bool, '/bottle_at_target', self.bottle_at_target_callback, 10)

        self.get_logger().info("BottlePickupNode has been started.")

    def bottle_at_target_callback(self, msg):
        if msg.data:
            self.pick_up_bottle()
        else:
            self.get_logger().info("Bottle not at target position. Skipping pick-up operation.")

    def pick_up_bottle(self):
        """控制机械臂拾取瓶子"""
        self.arm_command_publisher.publish(String(data="move_to_pick"))
        self.get_logger().info("Sent pick-up command to the arm.")

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