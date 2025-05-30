from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class MotorController:
    def __init__(self, node, board):
        self.node = node
        self.board = board

        # 电机控制参数
        self.wheel_base = 0.65  # 轮距（单位：米）
        self.max_speed = 100.0  # 最大电机速度（单位：占空比，范围 -100 到 100）
        self.angular_scale = 0.8  # 缩放比例（0.5 表示减半）
        self.timeout = 0.5  # 超时时间（单位：秒）
        self.is_stopped = False
        self.last_cmd_time = self.node.get_clock().now()

        # IMU数据缓存
        self.last_imu_msg = None

        # 订阅电机控制指令
        self.motor_subscription = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 订阅IMU数据
        self.imu_subscription = self.node.create_subscription(
            Imu,
            '/imu/data',  # 或 '/imu/data_raw'，根据你的实际话题名
            self.imu_callback,
            10
        )

        # 定时检查超时
        self.timeout_timer = self.node.create_timer(0.1, self.check_timeout)

    def imu_callback(self, msg):
        """处理IMU数据"""
        self.last_imu_msg = msg
        # 你可以在这里处理IMU数据，比如打印或滤波
        # print(f"IMU angular_velocity.z: {msg.angular_velocity.z}, linear_acceleration.x: {msg.linear_acceleration.x}")

    def cmd_vel_callback(self, msg):
        """处理电机控制指令"""
        self.last_cmd_time = self.node.get_clock().now()

        if self.is_stopped:
            self.is_stopped = False
            self.node.get_logger().info("Command received. Resuming motors.")

        # 提取线速度和角速度
        linear_x = msg.linear.x
        angular_z = msg.angular.z * self.angular_scale

        # === 用IMU数据做补偿/限制（示例） ===
        if self.last_imu_msg is not None:
            # 例如：限制加速度，防止突然加速
            acc_x = self.last_imu_msg.linear_acceleration.x
            max_acc = 2.0  # m/s^2
            if abs(acc_x) > max_acc:
                linear_x = 0.0
                self.node.get_logger().warn("加速度过大，已强制停止！")

            # 例如：根据陀螺仪角速度做角速度补偿
            gyro_z = self.last_imu_msg.angular_velocity.z
            angular_z += gyro_z  # 简单补偿，可根据实际需求调整

        # 计算左右轮组的速度
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # 将速度转换为占空比（范围 -100 到 100）
        left_duty = max(min(left_speed * self.max_speed, 100.0), -100.0)
        right_duty = max(min(right_speed * self.max_speed, 100.0), -100.0)

        # 设置电机速度
        motor_speeds = [
            [1, left_duty],   # 左前轮
            [2, right_duty],  # 右前轮
            [3, left_duty],   # 左后轮
            [4, right_duty],  # 右后轮
        ]
        self.board.set_motor_duty(motor_speeds)
        self.node.get_logger().info(f"Set motor speeds: {motor_speeds}")

    def check_timeout(self):
        """检查电机控制指令是否超时"""
        now = self.node.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout:
            if not self.is_stopped:
                motor_speeds = [
                    [1, 0.0],  # 左前轮
                    [2, 0.0],  # 左后轮
                    [3, 0.0],  # 右前轮
                    [4, 0.0],  # 右后轮
                ]
                self.board.set_motor_duty(motor_speeds)
                self.node.get_logger().info("No command received. Stopping motors.")
                self.is_stopped = True