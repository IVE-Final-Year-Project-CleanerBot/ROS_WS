import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_commander.robot_navigator import BasicNavigator, TaskResult

class BottleTargetNode(Node):
    def __init__(self):
        super().__init__('bottle_target_node')
        self.navigator = BasicNavigator()
        self.goal_sub = self.create_subscription(
            PoseStamped, '/bottle_nav_goal', self.goal_callback, 10)
        self.bottle_at_target_pub = self.create_publisher(Bool, '/bottle_at_target', 10)
        self.current_task = None

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"收到新的瓶子目标点: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        # 启动导航
        self.current_task = self.navigator.goToPose(msg)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_task is not None:
                if self.navigator.isTaskComplete(self.current_task):
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info("已到达瓶子面前，发布 /bottle_at_target=True")
                        self.bottle_at_target_pub.publish(Bool(data=True))
                    else:
                        self.get_logger().warn("导航失败，发布 /bottle_at_target=False")
                        self.bottle_at_target_pub.publish(Bool(data=False))
                    self.current_task = None

def main(args=None):
    rclpy.init(args=args)
    node = BottleTargetNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()