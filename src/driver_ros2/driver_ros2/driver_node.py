import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Twist
from driver_ros2.ros_robot_controller_sdk import Board


class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')

        # 初始化扩展板通信
        self.board = Board()
        self.board.enable_reception()

        # 舵机控制订阅
        self.servo_subscription = self.create_subscription(
            Int32MultiArray,
            '/cmd_pwm_servo',
            self.pwm_servo_callback,
            10
        )

        # 电机控制订阅
        self.motor_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 电池状态发布
        self.battery_publisher = self.create_publisher(Float32, '/battery_voltage', 10)
        self.battery_timer = self.create_timer(1.0, self.publish_battery_voltage)

        # 电机控制参数
        self.wheel_base = 0.65  # 轮距（单位：米）
        self.max_speed = 100.0  # 最大电机速度（单位：占空比，范围 -100 到 100）
        self.timeout = 0.5  # 超时时间（单位：秒）
        self.is_stopped = False  # 标记电机是否已停止
        self.last_cmd_time = self.get_clock().now()
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("DriverNode has been started.")

    def pwm_servo_callback(self, msg):
        """处理舵机控制指令"""
        positions = []
        for i in range(0, len(msg.data), 2):
            servo_id = msg.data[i]
            angle = msg.data[i + 1]

            # 将角度转换为脉宽
            pulse_width = int(11.1 * angle + 500)

            # 限制脉宽范围在 500 到 2500 之间
            pulse_width = max(500, min(2500, pulse_width))

            positions.append([servo_id, pulse_width])

        # 设置舵机位置
        self.board.pwm_servo_set_position(1, positions)
        self.get_logger().info(f"Set PWM servo positions: {positions}")

    def cmd_vel_callback(self, msg):
        """处理电机控制指令"""
        self.last_cmd_time = self.get_clock().now()

        if self.is_stopped:
            self.is_stopped = False
            self.get_logger().info("Command received. Resuming motors.")

        # 提取线速度和角速度
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 计算左右轮组的速度
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # 将速度转换为占空比（范围 -100 到 100）
        left_duty = max(min(left_speed * self.max_speed, 100.0), -100.0)
        right_duty = max(min(right_speed * self.max_speed, 100.0), -100.0)

        # 设置 4 个轮子的速度
        motor_speeds = [
            [1, left_duty],   # 左前轮
            [2, right_duty],  # 右前轮
            [3, left_duty],   # 左后轮
            [4, right_duty],  # 右后轮
        ]
        self.board.set_motor_duty(motor_speeds)
        self.get_logger().info(f"Set motor speeds: {motor_speeds}")

    def check_timeout(self):
        """检查电机控制指令是否超时"""
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout:
            if not self.is_stopped:
                motor_speeds = [
                    [1, 0.0],  # 左前轮
                    [2, 0.0],  # 左后轮
                    [3, 0.0],  # 右前轮
                    [4, 0.0],  # 右后轮
                ]
                self.board.set_motor_duty(motor_speeds)
                self.get_logger().info("No command received. Stopping motors.")
                self.is_stopped = True

    def publish_battery_voltage(self):
        """发布电池电压"""
        voltage = self.board.get_battery()
        if voltage is not None:
            msg = Float32()
            msg.data = voltage / 1000.0
            self.battery_publisher.publish(msg)
            self.get_logger().info(f"Battery voltage: {msg.data} V")


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # 确保 rclpy 仍然处于运行状态
            rclpy.shutdown()


if __name__ == '__main__':
    main()