import numpy as np
from sensor_msgs.msg import LaserScan
from .motor_controller import MotorController

class ObstacleAvoidance:
    def __init__(self, motor_controller: MotorController):
        self.motor_controller = motor_controller
        self.laser_data = None

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg

    def avoid_obstacles(self):
        if self.laser_data is None:
            return

        ranges = np.array(self.laser_data.ranges)
        ranges[ranges == float('inf')] = self.laser_data.range_max
        angles = np.arange(self.laser_data.angle_min, self.laser_data.angle_max, self.laser_data.angle_increment)

        front_indices = np.where((angles > -np.pi/8) & (angles < np.pi/8))
        left_indices = np.where((angles > np.pi/8) & (angles < 3*np.pi/8))
        right_indices = np.where((angles > -3*np.pi/8) & (angles < -np.pi/8))

        front = np.min(ranges[front_indices])
        left = np.min(ranges[left_indices])
        right = np.min(ranges[right_indices])

        THRESHOLD_FRONT = 0.5
        THRESHOLD_SIDE = 0.3

        if front < THRESHOLD_FRONT:
            if left < THRESHOLD_SIDE:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateRight'))
            elif right < THRESHOLD_SIDE:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateLeft'))
            else:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('backward'))
        else:
            if left < THRESHOLD_SIDE:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('right'))
            elif right < THRESHOLD_SIDE:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('left'))
            else:
                self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('forward'))