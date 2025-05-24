import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf_transformations

class BottleNav2PositionNode(Node):
    def __init__(self):
        super().__init__('bottle_nav2position_node')
        self.bridge = CvBridge()
        self.last_depth = None
        self.last_depth_header = None

        # 相机内参（请替换为实际标定值）
        self.fx = 411.0  # 焦距x
        self.fy = 411.0  # 焦距y
        self.cx = 165.2  # 主点x
        self.cy = 124.5  # 主点y

        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Point, '/bottle_position', self.bottle_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/bottle_nav_goal', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # 1. 先将目标点从camera_link变换到base_link
        # 假设相机和base_link重合，否则需要加静态变换
        point_in_base = np.array([Z, X, 0.0, 1.0])  # [前, 左, 上, 1]

        # 2. 获取base_link在map下的pose
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            # 四元数转欧拉角
            quat = [rot.x, rot.y, rot.z, rot.w]
            trans_mat = tf_transformations.quaternion_matrix(quat)
            trans_mat[0:3, 3] = [trans.x, trans.y, trans.z]

            # 3. 目标点加到机器人当前位置
            target_in_map = np.dot(trans_mat, point_in_base)

            # 4. 发布PoseStamped
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = target_in_map[0]
            goal.pose.position.y = target_in_map[1]
            goal.pose.position.z = 0.0
            goal.pose.orientation = rot  # 朝向和机器人一致

            # self.goal_pub.publish(goal)
            self.get_logger().info(
                f"发布全局目标点: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
        except Exception as e:
            self.get_logger().warn(f"TF查找失败: {e}")

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