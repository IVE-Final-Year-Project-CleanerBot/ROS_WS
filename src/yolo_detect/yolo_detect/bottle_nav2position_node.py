import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

class BottleNav2PositionNode(Node):
    def __init__(self):
        super().__init__('bottle_nav2position_node')
        self.bridge = CvBridge()
        self.last_depth = None
        self.last_depth_header = None

        # 相机内参（请替换为实际标定值）
        self.fx = 600.0  # 焦距x
        self.fy = 600.0  # 焦距y
        self.cx = 320.0  # 主点x
        self.cy = 240.0  # 主点y

        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Point, '/bottle_position', self.bottle_callback, 10)

    def depth_callback(self, msg):
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.last_depth_header = msg.header

    def midas_to_distance(self, z):
        # ====== 关键：归一化深度转实际距离（米）======
        # 这里用简单线性拟合，实际请用你的标定数据
        # 假设 z=200时0.5米，z=50时2米
        # 距离 = a / (z + b)
        a = 100.0
        b = 0.0
        if z == 0:
            return 0.0
        distance = a / (z + b)
        return distance

    def bottle_callback(self, msg):
        if self.last_depth is None:
            self.get_logger().warn("No depth image received yet.")
            return

        u = int(msg.x)
        v = int(msg.y)
        if v >= self.last_depth.shape[0] or u >= self.last_depth.shape[1]:
            self.get_logger().warn("Bottle position out of depth image bounds.")
            return

        z = float(self.last_depth[v, u])  # MiDaS归一化深度

        # ====== 归一化深度转实际距离 ======
        real_z = self.midas_to_distance(z)  # 实际距离，单位米

        # 相机坐标系下的三维坐标(X, Y, Z)
        X = (u - self.cx) * real_z / self.fx
        # Y = (v - self.cy) * real_z / self.fy  # 通常地面机器人不需要
        Z = real_z

        # nav2坐标系
        robot_X = Z      # 前进距离（米）
        robot_Y = X      # 左右距离（米）
        robot_Z = 0.0    # 地面导航，高度为0

        self.get_logger().info(
            f"Nav2目标点: X={robot_X:.2f}m (前进), Y={robot_Y:.2f}m (左右), Z={robot_Z:.2f}m (高度，通常为0)"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BottleNav2PositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()