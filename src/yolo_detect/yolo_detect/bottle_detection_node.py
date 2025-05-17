from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import os
import cv2


class BottleDetectionNode(Node):
    def __init__(self):
        super().__init__('bottle_detection_node')

        # 初始化检测结果发布器
        self.bottle_detected_publisher = self.create_publisher(Bool, '/bottle_detected', 10)
        self.bottle_position_publisher = self.create_publisher(Point, '/bottle_position', 10)

        # 初始化 YOLO 模型
        package_dir = get_package_share_directory("yolo_detect")
        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")
        self.model = YOLO(model_file, verbose=False)  # 加载 YOLO 模型

        # 初始化图像处理
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        self.get_logger().info("BottleDetectionNode has been started.")

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width, _ = frame.shape  # 获取图像分辨率

        # 运行 YOLO 检测
        results = self.model(frame)

        # 标志变量，用于判断是否检测到 "PET Bottle"
        detected_bottle = False
        bottle_position = Point()

        # 遍历检测结果
        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                if self.model.names[class_id] == "PET Bottle":
                    detected_bottle = True

                    # 计算检测框的中心点
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    bottle_position.x = (x1 + x2) / 2
                    bottle_position.y = (y1 + y2) / 2
                    bottle_position.z = 0.0  # 假设 z 轴为 0

                    # 在图像上绘制检测框和标签
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绘制绿色矩形框
                    cv2.putText(frame, "PET Bottle", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    break

        # 发布检测结果和瓶子位置
        self.bottle_detected_publisher.publish(Bool(data=detected_bottle))
        if detected_bottle:
            self.bottle_position_publisher.publish(bottle_position)

        # 显示检测结果
        # cv2.imshow("YOLO Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()