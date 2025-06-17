from geometry_msgs.msg import PoseStamped, Point
from nav2_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class SinglePointNavigation(Node):
    def __init__(self):
        super().__init__('single_point_navigation')
        self.navigator = BasicNavigator()
        self.target_reached_publisher = self.create_publisher(Bool, '/target_reached', 10)
        self.create_subscription(Bool, '/start_reached', self.start_reached_callback, 10)
        self.create_subscription(Bool, '/gripper_has_bottle', self.gripper_state_callback, 10)
        self.target_published = False
        self.gripper_has_bottle = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def gripper_state_callback(self, msg):
        self.gripper_has_bottle = msg.data

    def start_reached_callback(self, msg):
        # 只有到达起点且夹爪无瓶子时才允许新一轮
        if msg.data and not self.target_published and not self.gripper_has_bottle:
            self.navigate_to_target()
            self.target_published = True

    def navigate_to_target(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 3.14
        target_pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Navigating to target point...")
        self.navigator.goToPose(target_pose)

    def reset(self):
        self.target_published = False

def main(args=None):
    rclpy.init(args=args)
    node = SinglePointNavigation()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.target_published and node.navigator.isTaskComplete():
                result = node.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    node.target_reached_publisher.publish(Bool(data=True))
                    node.reset()
                elif result == TaskResult.CANCELED or result == TaskResult.FAILED:
                    node.reset()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()