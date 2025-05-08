#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_run')  # 初始化节点，名称为 'navigation_run'
        self.navigator = BasicNavigator()

    def run(self):
        # # 设置初始位姿
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(initial_pose)

        # 等待导航系统激活
        self.navigator.waitUntilNav2Active()

        # 设置目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.33917
        goal_pose.pose.position.y = 0.113125
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 3.14

        # 开始导航
        self.navigator.goToPose(goal_pose)

        # 等待任务完成
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} seconds")

        # 检查任务结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Navigation canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Navigation failed!")

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()