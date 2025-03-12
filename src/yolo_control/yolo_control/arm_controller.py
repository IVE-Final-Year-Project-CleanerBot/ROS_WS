import time
import driver_ros2.ros_robot_controller_sdk as rrc

class ArmController:
    def __init__(self):
        self.board = rrc.Board()
        self.board.enable_reception(True)  # 启用接收模式

    def angle_to_pulse(self, angle):
        # 将角度转换为脉宽
        return 11.1 * angle + 500

    def pulse_to_angle(self, pulse_width):
        # 将脉宽转换为角度
        return (pulse_width - 500) / 11.1

    def move_to_position(self, positions, duration=1):
        # 将角度转换为脉宽
        pulse_positions = [[channel, self.angle_to_pulse(angle)] for channel, angle in positions]
        self.board.pwm_servo_set_position(duration, pulse_positions)

    def pick_up(self):
        # 示例：机械臂拾取动作
        # 你需要根据实际情况调整这些位置和时间
        self.move_to_position([[1, 90], [2, 90], [3, 90], [4, 90], [5, 90], [6, 0]], duration=1)
        time.sleep(1)
        self.move_to_position([[1, 45], [2, 45], [3, 45], [4, 45], [5, 45], [6, 0]], duration=1)
        time.sleep(1)
        self.move_to_position([[1, 90], [2, 90], [3, 90], [4, 90], [5, 90], [6, 0]], duration=1)
        time.sleep(1)

    def release(self):
        # 示例：机械臂释放动作
        # 你需要根据实际情况调整这些位置和时间
        self.move_to_position([[1, 120], [2, 120], [3, 120], [4, 120], [5, 120], [6, 120]], duration=1)
        time.sleep(1)