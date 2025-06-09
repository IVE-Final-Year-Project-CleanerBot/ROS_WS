from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node

class BottlePlaceNode(Node):
    def __init__(self):
        super().__init__('bottle_place_node')

        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)
        self.create_subscription(Bool, '/start_reached', self.start_reached_callback, 10)
        self.create_subscription(Bool, '/gripper_has_bottle', self.gripper_callback, 10)

        self.has_bottle = False

        self.get_logger().info("BottlePlaceNode has been started.")

    def gripper_callback(self, msg):
        self.has_bottle = msg.data

    def start_reached_callback(self, msg):
        if msg.data and self.has_bottle:
            self.arm_command_publisher.publish(String(data="move_to_place"))
            self.get_logger().info("Bottle place completed.")
        elif msg.data and not self.has_bottle:
            self.get_logger().warn("Start reached, but no bottle in gripper. Skip placing.")

def main(args=None):
    rclpy.init(args=args)
    node = BottlePlaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()