from geometry_msgs.msg import PoseStamped
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_run')  # 初始化节点，名称为 'navigation_run'
        self.navigator = BasicNavigator()

    def run(self):
        # 定义起始位置
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # 定义正方形的 4 个目标点（包含起点）
        waypoints = [
            [0.0, 0.0, 0.0, 3.14],  # 起点
            [0.67, -0.31, 0.000210057, 1.57],  # 目标点 1
            [0.0, 0.0, 0.0, 0.0],  # 返回起点
        ]

        # 等待导航系统激活
        self.navigator.waitUntilNav2Active()

        # 循环遍历目标点
        while rclpy.ok():
            for waypoint in waypoints:
                # 设置目标位姿
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = waypoint[0]
                goal_pose.pose.position.y = waypoint[1]
                goal_pose.pose.orientation.z = waypoint[2]
                goal_pose.pose.orientation.w = waypoint[3]

                # 开始导航
                self.get_logger().info(f"Navigating to goal: {waypoint[0]}, {waypoint[1]}, {waypoint[2]}, {waypoint[3]}")
                self.navigator.goToPose(goal_pose)

                # 等待任务完成
                # while not self.navigator.isTaskComplete():
                #     feedback = self.navigator.getFeedback()
                #     if feedback:
                #         self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} seconds")

                # 检查任务结果
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Navigation succeeded!")
                elif result == TaskResult.CANCELED:
                    self.get_logger().info("Navigation canceled!")
                    break
                elif result == TaskResult.FAILED:
                    self.get_logger().info("Navigation failed!")
                    break

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()