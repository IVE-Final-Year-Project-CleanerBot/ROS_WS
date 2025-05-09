from geometry_msgs.msg import PoseStamped
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node


class ReturnToStartNode(Node):
    def __init__(self):
        super().__init__('return_to_start_node')  # 初始化节点，名称为 'return_to_start_node'
        self.navigator = BasicNavigator()

        # 订阅目标完成状态或瓶子拾取完成状态
        self.create_subscription(Bool, '/target_reached', self.target_reached_callback, 10)
        self.create_subscription(Bool, '/bottle_pickup_done', self.bottle_pickup_done_callback, 10)

        # 发布是否到达起点的状态
        self.start_reached_publisher = self.create_publisher(Bool, '/start_reached', 10)

        # 标志变量，记录是否需要返回初始位置
        self.return_to_start = False

    def target_reached_callback(self, msg):
        """处理目标点到达状态"""
        if msg.data:  # 如果目标点到达
            self.get_logger().info("Target reached! Preparing to return to start.")
            self.return_to_start = True

    def bottle_pickup_done_callback(self, msg):
        """处理瓶子拾取完成状态"""
        if msg.data:  # 如果瓶子拾取完成
            self.get_logger().info("Bottle pickup completed! Preparing to return to start.")
            self.return_to_start = True

    def navigate_to_start(self):
        """导航回初始位置"""
        # 定义初始位置
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = 0.0  # 初始位置的 x 坐标
        start_pose.pose.position.y = 0.0  # 初始位置的 y 坐标
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 3.14

        # 更新时间戳
        start_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # 开始导航到初始位置
        self.get_logger().info("Navigating back to start position...")
        self.navigator.goToPose(start_pose)

        # 等待任务完成
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} seconds")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Successfully returned to start position!")
            self.start_reached_publisher.publish(Bool(data=True))
        else:
            self.get_logger().warn("Failed to return to start position. Retrying...")
            self.navigate_to_start()  # 添加重试机制

            # 停止生命周期
            self.navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToStartNode()
    try:
        while rclpy.ok():  # 循环运行，直到程序被终止
            rclpy.spin_once(node, timeout_sec=0.1)  # 处理回调

            # 如果需要返回初始位置
            if node.return_to_start:
                node.navigate_to_start()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()