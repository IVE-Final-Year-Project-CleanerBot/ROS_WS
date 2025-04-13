#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class BottleDetector(Node):
    def __init__(self):
        super().__init__('bottle_detector')
        self.publisher = self.create_publisher(String, '/bottle_detection_status', 10)
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
        
        detection_msg = String()
        for result in results:
            if 'bottle' in result.names.values():
                detection_msg.data = "detected"
                self.publisher.publish(detection_msg)
                return
        
        detection_msg.data = "clear"
        self.publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = BottleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()