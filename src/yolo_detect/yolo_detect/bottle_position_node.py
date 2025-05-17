import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import numpy as np

class Bottle3DPositionNode(Node):
    def __init__(self):
        super().__init__('bottle_3d_position_node')
        self.bridge = CvBridge()
        self.last_depth = None
        self.last_depth_header = None

        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Point, '/bottle_position', self.bottle_callback, 10)
        self.position_pub = self.create_publisher(PointStamped, '/bottle_3d_position', 10)

    def depth_callback(self, msg):
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.last_depth_header = msg.header

    def bottle_callback(self, msg):
        if self.last_depth is None:
            self.get_logger().warn("No depth image received yet.")
            return

        x = int(msg.x)
        y = int(msg.y)
        if y >= self.last_depth.shape[0] or x >= self.last_depth.shape[1]:
            self.get_logger().warn("Bottle position out of depth image bounds.")
            return

        z = float(self.last_depth[y, x])

        # 只发布像素坐标和深度
        point = PointStamped()
        point.header = self.last_depth_header
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = z
        self.position_pub.publish(point)
        self.get_logger().info(f"Pixel: ({x}, {y}), Depth: {z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Bottle3DPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()