import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ros_robot_controller_sdk as rrc
import signal
import numpy as np

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# 定义速度变量
SPEED = 15

# 定义每个方向的阈值
THRESHOLD_FRONT = 0.5
THRESHOLD_FRONT_LEFT = 0.5
THRESHOLD_FRONT_RIGHT = 0.5

THRESHOLD_LEFT = 0.3
THRESHOLD_RIGHT = 0.3

THRESHOLD_BACK = 0.2
THRESHOLD_BACK_LEFT = 0.3
THRESHOLD_BACK_RIGHT = 0.3

class MecanumWheelController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.board = rrc.Board()
        self.board.enable_reception(True)  # 启用接收模式

    def laser_callback(self, msg):
        # 获取激光雷达数据
        ranges = np.array(msg.ranges)

        # 将无穷大值替换为最大测量距离
        ranges[ranges == float('inf')] = msg.range_max

        # 计算每个测量点的角度
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # 确定8个方向的测量值
        back_indices = np.where((angles >= -np.pi/8) & (angles <= np.pi/8))
        back_right_indices = np.where((angles > np.pi/8) & (angles <= 3*np.pi/8))
        right_indices = np.where((angles > 3*np.pi/8) & (angles <= 5*np.pi/8))
        front_right_indices = np.where((angles > 5*np.pi/8) & (angles <= 7*np.pi/8))
        front_indices = np.where((angles > 7*np.pi/8) | (angles <= -7*np.pi/8))
        front_left_indices = np.where((angles > -7*np.pi/8) & (angles <= -5*np.pi/8))
        left_indices = np.where((angles > -5*np.pi/8) & (angles <= -3*np.pi/8))
        back_left_indices = np.where((angles > -3*np.pi/8) & (angles <= -np.pi/8))

        front = np.min(ranges[front_indices])  # 前方距离（-22.5度到22.5度）
        front_left = np.min(ranges[front_left_indices])  # 前左方距离（22.5度到67.5度）
        left = np.min(ranges[left_indices])  # 左方距离（67.5度到112.5度）
        back_left = np.min(ranges[back_left_indices])  # 后左方距离（112.5度到157.5度）
        back = np.min(ranges[back_indices])  # 后方距离（157.5度到-157.5度）
        back_right = np.min(ranges[back_right_indices])  # 后右方距离（-157.5度到-112.5度）
        right = np.min(ranges[right_indices])  # 右方距离（-112.5度到-67.5度）
        front_right = np.min(ranges[front_right_indices])  # 前右方距离（-67.5度到-22.5度）
        print(f'Front: {front}, Front Left: {front_left}, Left: {left}, Back Left: {back_left}, Back: {back}, Back Right: {back_right}, Right: {right}, Front Right: {front_right}')

        # 控制马达
        if front > THRESHOLD_FRONT:
            self.set_wheel_speeds(*self.get_motor_commands('forward'))
        elif front_left > THRESHOLD_FRONT_LEFT:
            self.set_wheel_speeds(*self.get_motor_commands('rotateLeft'))
        elif front_right > THRESHOLD_FRONT_RIGHT:
            self.set_wheel_speeds(*self.get_motor_commands('rotateRight'))
        # elif left > THRESHOLD_LEFT:
        #     self.set_wheel_speeds(*self.get_motor_commands('left'))
        # elif right > THRESHOLD_RIGHT:
        #     self.set_wheel_speeds(*self.get_motor_commands('right'))
        else:
            self.set_wheel_speeds(*self.get_motor_commands('rotateRight'))

        
    def get_motor_commands(self, Id):
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

    def set_wheel_speeds(self, v1, v2, v3, v4):
        self.board.set_motor_duty([[1, v1], [2, v2], [3, v3], [4, v4]])

    def stop_all_motors(self):
        self.board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

def main(args=None):
    global node
    rclpy.init(args=args)
    node = MecanumWheelController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\nStopping...')
        node.stop_all_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()