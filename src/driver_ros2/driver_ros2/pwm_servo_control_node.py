import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from driver_ros2.ros_robot_controller_sdk import Board

class PWMServoControlNode(Node):
    def __init__(self):
        super().__init__('pwm_servo_control_node')
        self.board = Board()
        self.board.enable_reception()

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/cmd_pwm_servo',
            self.pwm_servo_callback,
            10
        )
        self.get_logger().info("PWMServoControlNode has been started.")

    def pwm_servo_callback(self, msg):
        positions = []
        for i in range(0, len(msg.data), 2):
            servo_id = msg.data[i]
            angle = msg.data[i + 1]

            # 将角度转换为脉宽
            pulse_width = int(11.1 * angle + 500)

            # 限制脉宽范围在 500 到 2500 之间
            pulse_width = max(500, min(2500, pulse_width))

            positions.append([servo_id, pulse_width])
        
        # 设置舵机位置
        self.board.pwm_servo_set_position(1, positions)
        self.get_logger().info(f"Set PWM servo positions: {positions}")

def main(args=None):
    rclpy.init(args=args)
    node = PWMServoControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()