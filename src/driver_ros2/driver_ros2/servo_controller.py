from std_msgs.msg import Int32MultiArray

class ServoController:
    def __init__(self, node, board):
        self.node = node
        self.board = board

        # 订阅舵机控制指令
        self.servo_subscription = self.node.create_subscription(
            Int32MultiArray,
            '/cmd_pwm_servo',
            self.pwm_servo_callback,
            10
        )

    def pwm_servo_callback(self, msg):
        """处理舵机控制指令"""
        positions = []
        for i in range(0, len(msg.data), 2):
            servo_id = msg.data[i]
            angle = msg.data[i + 1]

            # 将角度转换为脉宽
            pulse_width = int(angle * (2500 - 500) / 180 + 500)

            # 限制脉宽范围在 500 到 2500 之间
            pulse_width = max(500, min(2500, pulse_width))

            positions.append([servo_id, pulse_width])

        # 设置舵机位置
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info(f"Set PWM servo positions: {positions}")