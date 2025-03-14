import driver_ros2.ros_robot_controller_sdk as rrc

class MotorController:
    def __init__(self):
        self.board = rrc.Board()
        self.board.enable_reception(True)  # 启用接收模式

    def set_wheel_speeds(self, v1, v2, v3, v4):
        self.board.set_motor_duty([[1, v1], [2, v2], [3, v3], [4, v4]])

    def stop_all_motors(self):
        self.set_wheel_speeds(0, 0, 0, 0)

    def get_motor_commands(self, Id):
        SPEED = 13
        switcher = {
            'forward': [SPEED, SPEED, -SPEED, -SPEED],
            'backward': [-SPEED, -SPEED, SPEED, SPEED],
            'right': [-SPEED, SPEED, -SPEED, SPEED],
            'left': [SPEED, -SPEED, SPEED, -SPEED],
            'forwardLeft': [SPEED, 0, 0, -SPEED],
            'forwardRight': [0, SPEED, -SPEED, 0],
            'backwardLeft': [-SPEED, 0, 0, SPEED],
            'backwardRight': [0, -SPEED, SPEED, 0],
            'rotateLeft': [-SPEED, -SPEED, -SPEED, -SPEED],
            'rotateRight': [SPEED, SPEED, SPEED, SPEED],
            'stop': [0, 0, 0, 0]
        }
        return switcher.get(Id, [0, 0, 0, 0])