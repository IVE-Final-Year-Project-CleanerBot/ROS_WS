from std_msgs.msg import Int32MultiArray, String
import time

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
            [1, self.angle_to_pulse_width(90)],  
            [2, self.angle_to_pulse_width(135)], 
            [3, self.angle_to_pulse_width(95)], 
            [4, self.angle_to_pulse_width(35)],  
            [5, self.angle_to_pulse_width(135)], 
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Moved arm to pick position.")
        time.sleep(1)  # 等待舵机到达位置
        positions = [
            [5, self.angle_to_pulse_width(180)]  
        ]
        self.board.pwm_servo_set_position(1, positions)
        time.sleep(1)  # 等待夹爪闭合

    def move_to_place_position(self):
        """移动机械臂到放置位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)],  
            [2, self.angle_to_pulse_width(45)], 
            [3, self.angle_to_pulse_width(95)], 
            [4, self.angle_to_pulse_width(35)], 
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Moved arm to place position.")
        time.sleep(1)
        positions = [
            [5, self.angle_to_pulse_width(90)]
        ]
        self.board.pwm_servo_set_position(1, positions)
        time.sleep(1)

    def reset_arm_position(self):
        """重置机械臂到初始位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)], 
            [2, self.angle_to_pulse_width(0)], 
            [3, self.angle_to_pulse_width(45)], 
            [4, self.angle_to_pulse_width(45)], 
            [5, self.angle_to_pulse_width(180)], 
        ]
        self.board.pwm_servo_set_position(1, positions)
        self.node.get_logger().info("Reset arm to initial position.")

    def angle_to_pulse_width(self, angle):
        """将角度转换为舵机的脉宽值"""
        # 限制角度范围（假设舵机角度范围为 0° 到 180°）
        angle = max(0, min(180, angle))
        # 角度到脉宽的转换公式
        return int(angle * (2500 - 500) / 180 + 500)