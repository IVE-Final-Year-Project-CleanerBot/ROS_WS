from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class BottleTargetYoloNode(Node):
    def __init__(self):
        super().__init__('bottle_target_yolo_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bottle_at_target_pub = self.create_publisher(Bool, '/bottle_at_target', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.create_subscription(Bool, '/gripper_has_bottle', self.gripper_state_callback, 10)
        self.create_subscription(Point, '/bottle_position', self.bottle_position_callback, 10)

        self.x_tolerance = 10
        self.y_threshold_factor = 1.5 / 3
        self.linear_speed = 0.13
        self.fixed_angular_speed = 0.35
        self.bottle_reached = False
        self.image_width = 320
        self.image_height = 240

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def gripper_state_callback(self, msg):
        # 夹爪放下后允许再次响应
        if not msg.data:
            self.bottle_reached = False

    def bottle_position_callback(self, msg: Point):
        # --- 查询TF并发布当前位置 ---
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            self.current_pose_pub.publish(pose)
            self.get_logger().info(
                f"已发布当前位置: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
            )
        except Exception as e:
            self.get_logger().warn(f"TF查找失败: {e}")

        if self.bottle_reached:
            return

        bbox_center_x = msg.x
        bbox_center_y = msg.y
        y_threshold = self.image_height * self.y_threshold_factor
        offset_x = bbox_center_x - (self.image_width / 2)

        twist = Twist()
        if abs(offset_x) > self.x_tolerance:
            twist.angular.z = self.fixed_angular_speed if offset_x < 0 else -self.fixed_angular_speed
            twist.linear.x = 0.0
        elif bbox_center_y < y_threshold:
            twist.angular.z = 0.0
            twist.linear.x = self.linear_speed
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist)
            if not self.bottle_reached:
                self.bottle_at_target_pub.publish(Bool(data=True))
                self.bottle_reached = True
            return

        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BottleTargetYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        rclpy.shutdown()

if __name__ == '__main__':
    main()