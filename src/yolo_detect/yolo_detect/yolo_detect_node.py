from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
import os
import math
import sys
import cv2

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        # 初始化 TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化导航目标发布器
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 初始化其他组件
        package_dir = get_package_share_directory("yolo_detect")
        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")
        self.model = YOLO(model_file)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PointStamped, '/detected_objects', 10)

        self.get_logger().info("YoloDetectNode has been started.")

    def publish_goal_to_map(self, detected_point):
        try:
            # 获取从 camera_link 到 map 的变换
            transform = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            
            # 转换目标点到 map 坐标系
            goal_in_map = do_transform_point(detected_point, transform)

            # 创建导航目标消息
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_in_map.point.x
            goal_pose.pose.position.y = goal_in_map.point.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0  # 默认朝向

            # 发布导航目标
            self.nav_goal_publisher.publish(goal_pose)
            self.get_logger().info(f"Published navigation goal: {goal_pose.pose.position}")
        except Exception as e:
            self.get_logger().error(f"Failed to transform point: {e}")

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width, _ = frame.shape  # 获取图像分辨率

        # 运行 YOLO 检测
        results = self.model(frame)

        # 遍历检测结果并发布目标位置
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]
                class_id = int(box.cls[0])
                label = f"{self.model.names[class_id]} {confidence:.2f}"

                # 绘制检测框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绿色边框
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # 标签文字

                # 如果检测到的是塑料瓶，发布其位置
                if self.model.names[class_id] == "PET Bottle":
                    # 计算检测框的中心点
                    bbox_center_x = (x1 + x2) / 2
                    bbox_center_y = (y1 + y2) / 2
                    bbox_height = y2 - y1  # 检测框的高度

                    # 已知参数（需实际测量校准）
                    REAL_HEIGHT = 22.0  # 塑料瓶实际高度（厘米）
                    FOCAL_LENGTH = 720  # 摄像头焦距（像素），通过标定获得

                    # 估算深度信息
                    distance = (REAL_HEIGHT * FOCAL_LENGTH) / bbox_height

                    # 创建 PointStamped 消息
                    detected_point = PointStamped()
                    detected_point.header.frame_id = "camera_link"
                    detected_point.point.x = bbox_center_x - (image_width / 2)  # 转换为相对于图像中心的坐标
                    detected_point.point.y = bbox_center_y - (image_height / 2)
                    detected_point.point.z = distance  # 深度信息

                    # 转换并发布导航目标
                    self.publish_goal_to_map(detected_point)
                    self.get_logger().info(f"Estimated distance: {distance:.2f} cm")

        # 显示检测结果
        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()