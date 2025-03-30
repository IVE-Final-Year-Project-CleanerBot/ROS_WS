from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        package_dir = get_package_share_directory("yolo_detect")

        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 订阅摄像头话题
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO(model_file)  # 替换为您的 YOLO 模型路径

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 运行 YOLO 检测
        results = self.model(frame)

        # 在帧上绘制检测结果
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]
                class_id = int(box.cls[0])
                label = f"{self.model.names[class_id]} {confidence:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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