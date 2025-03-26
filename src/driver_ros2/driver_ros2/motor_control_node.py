import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ros_robot_controller_sdk import Board

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.board = Board()
        self.board.enable_reception()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_motor',
            self.motor_callback,
            10
        )
        self.get_logger().info("MotorControlNode has been started.")

    def motor_callback(self, msg):
        motor_speeds = [[i + 1, speed] for i, speed in enumerate(msg.data)]
        self.board.set_motor_duty(motor_speeds)
        self.get_logger().info(f"Set motor speeds: {motor_speeds}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()