import rclpy
from rclpy.node import Node
from driver_ros2.ros_robot_controller_sdk import Board
from driver_ros2.servo_controller import ServoController
from driver_ros2.motor_controller import MotorController
from driver_ros2.battery_monitor import BatteryMonitor

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')

        # 初始化扩展板通信
        self.board = Board()
        self.board.enable_reception()

        # 初始化功能模块
        self.servo_controller = ServoController(self, self.board)
        self.motor_controller = MotorController(self, self.board)
        self.battery_monitor = BatteryMonitor(self, self.board)

        self.get_logger().info("DriverNode has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # 确保 rclpy 仍然处于运行状态
            rclpy.shutdown()

if __name__ == '__main__':
    main()