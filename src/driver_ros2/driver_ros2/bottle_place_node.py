from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node

class BottlePlaceNode(Node):
    def __init__(self):
        super().__init__('bottle_place_node')

        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)
        self.create_subscription(Bool, '/start_reached', self.start_reached_callback, 10)

        self.get_logger().info("BottlePlaceNode has been started.")

    def start_reached_callback(self, msg):
        if msg.data:
            self.arm_command_publisher.publish(String(data="move_to_place"))
            self.get_logger().info("Bottle place completed.")

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