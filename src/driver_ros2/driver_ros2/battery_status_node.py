import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from driver_ros2.ros_robot_controller_sdk import Board

class BatteryStatusNode(Node):
    def __init__(self):
        super().__init__('battery_status_node')
        self.board = Board()
        self.board.enable_reception()

        self.battery_publisher = self.create_publisher(Float32, '/battery_voltage', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_voltage)
        self.get_logger().info("BatteryStatusNode has been started.")

    def publish_battery_voltage(self):
        voltage = self.board.get_battery()
        if voltage is not None:
            msg = Float32()
            msg.data = voltage / 1000.0
            self.battery_publisher.publish(msg)
            self.get_logger().info(f"Battery voltage: {msg.data} V")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()