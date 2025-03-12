import time
import driver_ros2.ros_robot_controller_sdk as rrc

class ArmController:
    def __init__(self):
        self.board = rrc.Board()
        self.board.enable_reception(True)  # 启用接收模式

    def move_to_position(self, positions, duration=1):
        self.board.pwm_servo_set_position(duration, positions)

    def pick_up(self):
        # 示例：机械臂拾取动作
        # 你需要根据实际情况调整这些位置和时间
        self.move_to_position([[1, 1500], [2, 1500], [3, 1500], [4, 1500], [5, 1500], [6, 1500]], duration=1)
        time.sleep(1)
        self.move_to_position([[1, 1000], [2, 1000], [3, 1000], [4, 1000], [5, 1000], [6, 1000]], duration=1)
        time.sleep(1)
        self.move_to_position([[1, 1500], [2, 1500], [3, 1500], [4, 1500], [5, 1500], [6, 1500]], duration=1)
        time.sleep(1)

    def release(self):
        # 示例：机械臂释放动作
        # 你需要根据实际情况调整这些位置和时间
        self.move_to_position([[1, 2000], [2, 2000], [3, 2000], [4, 2000], [5, 2000], [6, 2000]], duration=1)
        time.sleep(1)