#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import time

class BottleDetector(Node):
    def __init__(self):
        super().__init__('bottle_detector')
        self.publisher = self.create_publisher(Float32MultiArray, '/bottle_detection_data', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        package_dir = get_package_share_directory("yolo_detect")
        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")
        self.model = YOLO(model_file, verbose=False)  # 加载 YOLO 模型
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        
        detection_msg = Float32MultiArray()
        for result in results:
            for box, cls_id in zip(result.boxes, result.boxes.cls):
                label = result.names[int(cls_id)]  # 获取标签名称
                if label == "PET Bottle":  # 只处理 "PET Bottle"
                    # 获取检测框的坐标
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detection_msg.data = [x1, y1, x2, y2]
                    self.publisher.publish(detection_msg)
                    return
        
        # 如果没有检测到 "PET Bottle"，发布空消息
        detection_msg.data = []
        self.publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = BottleDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # 确保只调用一次 shutdown
            detector.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()