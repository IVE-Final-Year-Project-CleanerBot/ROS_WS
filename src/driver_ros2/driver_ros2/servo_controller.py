from std_msgs.msg import Int32MultiArray, String

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

        # 订阅高层次机械臂动作指令
        self.arm_command_subscription = self.node.create_subscription(
            String,
            '/arm_command',
            self.arm_command_callback,
            10
        )

    def pwm_servo_callback(self, msg):
        """处理舵机控制指令"""
        positions = []
        for i in range(0, len(msg.data), 2):
            servo_id = msg.data[i]
            angle = msg.data[i + 1]

            # 将角度转换为脉宽
            pulse_width = self.angle_to_pulse_width(angle)

            positions.append([servo_id, pulse_width])

        # 设置舵机位置
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info(f"Set PWM servo positions: {positions}")

    def arm_command_callback(self, msg):
        """处理高层次机械臂动作指令"""
        command = msg.data
        if command == "move_to_pick":
            self.move_to_pick_position()
        elif command == "move_to_place":
            self.move_to_place_position()
        elif command == "reset":
            self.reset_arm_position()

    def move_to_pick_position(self):
        """移动机械臂到拾取位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)],  # 舵机 1 设置为 90°
            [2, self.angle_to_pulse_width(135)], # 舵机 2 设置为 120°
            [3, self.angle_to_pulse_width(135)], # 舵机 3 设置为 100°
            [4, self.angle_to_pulse_width(85)],  # 舵机 4 设置为 85°
            [5, self.angle_to_pulse_width(90)],  # 舵机 5 设置为 45°（夹爪）
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Moved arm to pick position.")

    def move_to_place_position(self):
        """移动机械臂到放置位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)],  # 舵机 1 设置为 45°
            [2, self.angle_to_pulse_width(90)],  # 舵机 2 设置为 90°
            [3, self.angle_to_pulse_width(80)],  # 舵机 3 设置为 80°
            [4, self.angle_to_pulse_width(60)],  # 舵机 4 设置为 60°
            [5, self.angle_to_pulse_width(90)],   # 舵机 5 设置为 0°（夹爪打开）
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Moved arm to place position.")

    def reset_arm_position(self):
        """重置机械臂到初始位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)],  # 舵机 1 设置为 90°
            [2, self.angle_to_pulse_width(45)],  # 舵机 2 设置为 90°
            [3, self.angle_to_pulse_width(90)],  # 舵机 3 设置为 90°
            [4, self.angle_to_pulse_width(45)],  # 舵机 4 设置为 90°
            [5, self.angle_to_pulse_width(90)],  # 舵机 5 设置为 90°
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Reset arm to initial position.")

    def angle_to_pulse_width(self, angle):
        """将角度转换为舵机的脉宽值"""
        # 限制角度范围（假设舵机角度范围为 0° 到 180°）
        angle = max(0, min(180, angle))
        # 角度到脉宽的转换公式
        return int(angle * (2500 - 500) / 180 + 500)