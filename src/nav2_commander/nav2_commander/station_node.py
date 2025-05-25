from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class StationTargetNode(Node):
    def __init__(self):
        super().__init__('station_target_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/station_nav_goal', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 3.14
        self.goal_pub.publish(goal)
        self.get_logger().info("已发布station目标点")

def main(args=None):
    rclpy.init(args=args)
    node = StationTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()