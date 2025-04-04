import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from driver_ros2.ros_robot_controller_sdk import Board

class TwistMotorControlNode(Node):
    def __init__(self):
        super().__init__('twist_motor_control_node')
        self.board = Board()  # 初始化扩展板通信
        self.board.enable_reception()  # 启用串口接收

        # 机器人参数
        self.wheel_base = 0.65 # 轮距（单位：米）
        self.max_speed = 100.0  # 最大电机速度（单位：占空比，范围 -100 到 100）
        self.timeout = 0.5  # 超时时间（单位：秒）

        # 状态标志
        self.is_stopped = False  # 用于标记是否已经停止

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 定时器，用于检测速度指令超时
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("TwistMotorControlNode has been started.")

    def cmd_vel_callback(self, msg):
        # 更新最后接收到指令的时间
        self.last_cmd_time = self.get_clock().now()

        # 如果之前处于停止状态，恢复标志并打印日志
        if self.is_stopped:
            self.is_stopped = False
            self.get_logger().info("Command received. Resuming motors.")

        # 提取线速度和角速度
        linear_x = msg.linear.x  # 线速度
        angular_z = msg.angular.z  # 角速度

        # 计算左右轮组的速度
        # 如果线速度为负（后退），忽略角速度的影响
        # if linear_x < 0:
        #     left_speed = linear_x
        #     right_speed = linear_x
        # else:
        #     # 正常计算左右轮速度
        #     left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        #     right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # 正常计算左右轮速度
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # 将速度转换为占空比（范围 -100 到 100）
        left_duty = max(min(left_speed * self.max_speed, 100.0), -100.0)
        right_duty = max(min(right_speed * self.max_speed, 100.0), -100.0)

        # 设置 4 个轮子的速度
        motor_speeds = [
            [1, left_duty],   # 左前轮
            [2, right_duty],   # 右前轮
            [3, left_duty],  # 左后轮
            [4, right_duty],  # 右后轮
        ]
        self.board.set_motor_duty(motor_speeds)

        # 打印日志
        self.get_logger().info(f"Set motor speeds: {motor_speeds}")

    def check_timeout(self):
        # 检查是否超时
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout:
            # 如果已经停止，则不重复打印日志
            if not self.is_stopped:
                # 超时，停止电机
                motor_speeds = [
                    [1, 0.0],  # 左前轮
                    [2, 0.0],  # 左后轮
                    [3, 0.0],  # 右前轮
                    [4, 0.0],  # 右后轮
                ]
                self.board.set_motor_duty(motor_speeds)
                self.get_logger().info("No command received. Stopping motors.")
                self.is_stopped = True  # 标记为已停止

def main(args=None):
    rclpy.init(args=args)
    node = TwistMotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()