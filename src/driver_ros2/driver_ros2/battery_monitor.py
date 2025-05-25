from std_msgs.msg import Float32

class BatteryMonitor:
    def __init__(self, node, board):
        self.node = node
        self.board = board

        # 发布电池状态
        self.battery_publisher = self.node.create_publisher(Float32, '/battery_voltage', 10)
        self.battery_timer = self.node.create_timer(1.0, self.publish_battery_voltage)

    def publish_battery_voltage(self):
        """发布电池电压"""
        voltage = self.board.get_battery()
        if voltage is not None:
            msg = Float32()
            msg.data = voltage / 1000.0
            self.battery_publisher.publish(msg)
            self.node.get_logger().info(f"Battery voltage: {msg.data} V")
        else:
            self.node.get_logger().warn("No battery voltage read from board!")