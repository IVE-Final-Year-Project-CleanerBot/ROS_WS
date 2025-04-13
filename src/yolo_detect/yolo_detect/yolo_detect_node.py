from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node  # 确保导入 Node
import os
import math
import cv2

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        # 初始化 TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化电机控制发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

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

        self.get_logger().info("YoloDetectNode has been started.")

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width, _ = frame.shape  # 获取图像分辨率

        # 运行 YOLO 检测
        results = self.model(frame)

        # 遍历检测结果并驱动机器人移动
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

                # 如果检测到的是塑料瓶，驱动机器人移动到瓶子面前
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

                    # 控制机器人移动
                    self.drive_to_target(bbox_center_x, image_width, distance)

                    # 控制机械臂拾取瓶子
                    self.pick_up_bottle()

        # 显示检测结果
        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def drive_to_target(self, bbox_center_x, image_width, distance):
        """根据目标位置控制机器人移动"""
        twist = Twist()

        # 计算水平偏移量
        offset_x = bbox_center_x - (image_width / 2)

        # 根据偏移量调整机器人角速度
        twist.angular.z = -0.002 * offset_x  # 调整旋转速度，负号表示方向

        # 根据距离调整机器人线速度
        if distance > 30:  # 距离大于 30 cm 时向前移动
            twist.linear.x = 0.1
        else:
            twist.linear.x = 0.0  # 停止移动

        # 发布速度指令
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Driving to target: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

    def pick_up_bottle(self):
        """控制机械臂拾取瓶子"""
        # 发布拾取指令
        self.arm_command_publisher.publish(String(data="move_to_pick"))
        self.get_logger().info("Sent pick-up command to the arm.")

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