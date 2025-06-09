from geometry_msgs.msg import PoseStamped
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node

class ReturnToStartNode(Node):
    def __init__(self):
        super().__init__('return_to_start_node')
        self.navigator = BasicNavigator()
        self.start_reached_publisher = self.create_publisher(Bool, '/start_reached', 10)
        self.create_subscription(Bool, '/target_reached', self.target_reached_callback, 10)
        self.create_subscription(Bool, '/gripper_has_bottle', self.gripper_callback, 10)
        self.start_published = False

        self.start_reached_publisher.publish(Bool(data=True))

    def target_reached_callback(self, msg):
        if msg.data and not self.start_published:
            self.get_logger().info("Target reached or gripper has bottle, publishing start point.")
            self.navigate_to_start()
            self.start_published = True

    def gripper_callback(self, msg):
        if msg.data and not self.start_published:
            self.get_logger().info("Gripper has bottle, publishing start point.")
            self.navigate_to_start()
            self.start_published = True

    def navigate_to_start(self):
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 1.0  # 合法四元数
        start_pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Navigating back to start position...")
        self.navigator.goToPose(start_pose)

    def reset(self):
        self.start_published = False

def main(args=None):
    rclpy.init(args=args)
    node = ReturnToStartNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.start_published and node.navigator.isTaskComplete():
                result = node.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    node.get_logger().info("Returned to start position!")
                    node.start_reached_publisher.publish(Bool(data=True))
                    node.reset()
                elif result == TaskResult.CANCELED or result == TaskResult.FAILED:
                    node.reset()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()