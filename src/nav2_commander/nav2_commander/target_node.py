from geometry_msgs.msg import PoseStamped
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
import rclpy
from rclpy.node import Node


class SinglePointNavigation(Node):
    def __init__(self):
        super().__init__('single_point_navigation')  # 初始化节点，名称为 'single_point_navigation'
        self.navigator = BasicNavigator()

        # TF2 Buffer 和 TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布是否到达目标点的状态
        self.target_reached_publisher = self.create_publisher(Bool, '/target_reached', 10)

        # 订阅瓶子检测状态
        self.create_subscription(Bool, '/bottle_detected', self.bottle_detected_callback, 10)

        # 订阅是否回到起点的状态
        self.create_subscription(Bool, '/start_reached', self.start_reached_callback, 10)

        # 订阅瓶子放置完成状态
        self.create_subscription(Bool, '/bottle_place_done', self.bottle_place_done_callback, 10)

        # 标志变量
        self.bottle_detected = False
        self.start_reached = False
        self.bottle_place_done = False

    def get_current_pose(self):
        """通过 TF2 获取机器人当前位置"""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1)
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"Failed to get current pose: {str(e)}")
            return None

    def bottle_detected_callback(self, msg):
        """处理瓶子检测状态"""
        self.bottle_detected = msg.data
        if self.bottle_detected:
            self.get_logger().info("Bottle detected! Setting target to current position.")
            self.set_target_to_current_position()  # 将目标点设置为当前位置

    def start_reached_callback(self, msg):
        """处理是否回到起点的状态"""
        self.start_reached = msg.data
        if self.start_reached and not self.bottle_detected and self.bottle_place_done:
            self.get_logger().info("Start position reached, no bottle detected, and bottle placed! Publishing new target point.")
            self.reset_flags()
            self.navigate_to_target()

    def bottle_place_done_callback(self, msg):
        """处理瓶子放置完成状态"""
        self.bottle_place_done = msg.data
        if self.bottle_place_done:
            self.get_logger().info("Bottle place completed! Ready to publish new target point.")
            self.reset_flags()
            self.navigate_to_target()

    def reset_flags(self):
        """重置标志变量"""
        self.bottle_detected = False
        self.start_reached = False
        self.bottle_place_done = False
        self.get_logger().info("All flags have been reset for the next task.")

    def set_target_to_current_position(self):
        """将目标点设置为当前位置"""
        current_pose = self.get_current_pose()  # 获取当前位姿
        if current_pose is not None:
            self.get_logger().info(f"Setting target to current position: x={current_pose.pose.position.x}, y={current_pose.pose.position.y}")
            self.navigator.goToPose(current_pose)  # 设置目标点为当前位置
        else:
            self.get_logger().warn("Failed to get current position!")

    def navigate_to_target(self):
        """发布目标点"""
        # 定义目标点
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = -0.4  # 目标点的 x 坐标
        target_pose.pose.position.y = 0.0  # 目标点的 y 坐标
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        # 更新时间戳
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # 开始导航到目标点
        self.get_logger().info("Navigating to target point...")
        self.navigator.goToPose(target_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SinglePointNavigation()
    try:
        while rclpy.ok():  # 循环运行，直到程序被终止
            # 发布目标点
            node.navigate_to_target()

            # 持续检查导航任务状态
            while not node.navigator.isTaskComplete():
                rclpy.spin_once(node, timeout_sec=0.1)  # 处理回调
                if node.bottle_detected:  # 如果检测到瓶子，记录日志
                    node.get_logger().info("Bottle detected! Setting target to current position.")

            # 检查任务结果
            result = node.navigator.getResult()
            if result == TaskResult.SUCCEEDED and not node.bottle_detected:
                node.get_logger().info("Navigation succeeded and no bottle detected!")
                node.target_reached_publisher.publish(Bool(data=True))  # 发布到达目标点的状态
            elif result == TaskResult.CANCELED:
                node.get_logger().warn("Navigation was canceled!")
            elif result == TaskResult.FAILED:
                node.get_logger().error("Navigation failed!")
            else:
                node.get_logger().error("Unknown result status!")

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()