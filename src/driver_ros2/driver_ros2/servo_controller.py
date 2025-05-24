from std_msgs.msg import Int32MultiArray, String, Bool
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

        # 新增：夹爪状态发布器
        self.gripper_state_pub = self.node.create_publisher(Bool, '/gripper_has_bottle', 10)

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
            [2, self.angle_to_pulse_width(150)], 
            [3, self.angle_to_pulse_width(90)], 
            [4, self.angle_to_pulse_width(45)] 
        ]
        self.board.pwm_servo_set_position(2, positions)
        self.node.get_logger().info("Moved arm to pick position.")
        time.sleep(2)  # 等待舵机到达位置
        self.close_gripper()  # 关闭夹爪以夹取物体
        time.sleep(2)  # 等待夹爪闭合
        self.reset_arm_position()  # 重置机械臂到初始位置
        time.sleep(2)  # 等待舵机到达位置
        self.gripper_state_pub.publish(Bool(data=True))
        self.node.get_logger().info("发布夹爪状态: True")

    def move_to_place_position(self):
        """移动机械臂到放置位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)],  
            [2, self.angle_to_pulse_width(90)], 
            [3, self.angle_to_pulse_width(135)]
        ]
        self.board.pwm_servo_set_position(2, positions)
        self.node.get_logger().info("Moved arm to place position.")
        time.sleep(2)
        self.open_gripper()  # 打开夹爪以释放物体
        time.sleep(2)
        self.reset_arm_position()  # 重置机械臂到初始位置
        time.sleep(2)  # 等待舵机到达位置
        self.gripper_state_pub.publish(Bool(data=False))
        self.node.get_logger().info("发布夹爪状态: False")

    def reset_arm_position(self):
        """重置机械臂到初始位置"""
        positions = [
            [1, self.angle_to_pulse_width(90)], 
            [2, self.angle_to_pulse_width(0)], 
            [3, self.angle_to_pulse_width(45)], 
            [4, self.angle_to_pulse_width(90)], 
        ]
        self.board.pwm_servo_set_position(2, positions)
        self.node.get_logger().info("Reset arm to initial position.")
        self.gripper_state_pub.publish(Bool(data=False))
        self.node.get_logger().info("发布夹爪状态: False")

    def close_gripper(self):
        """关闭夹爪以夹取物体"""
        gripper_position = [
            [4, self.angle_to_pulse_width(90)]  # 舵机 5 设置为 45°（夹爪闭合）
        ]
        self.board.pwm_servo_set_position(2, gripper_position)
        self.node.get_logger().info("Gripper closed to pick up the object.")

    def open_gripper(self):
        """打开夹爪以释放物体"""
        gripper_position = [
            [4, self.angle_to_pulse_width(0)]  # 舵机 5 设置为 90°（夹爪打开）
        ]
        self.board.pwm_servo_set_position(2, gripper_position)
        self.node.get_logger().info("Gripper opened to release the object.")

    def angle_to_pulse_width(self, angle):
        """将角度转换为舵机的脉宽值"""
        # 限制角度范围（假设舵机角度范围为 0° 到 180°）
        angle = max(0, min(180, angle))
        # 角度到脉宽的转换公式
        return int(angle * (2500 - 500) / 180 + 500)