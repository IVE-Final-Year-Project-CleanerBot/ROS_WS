from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import os
import cv2

class BottleDetectionNode(Node):
    def __init__(self):
        super().__init__('bottle_detection_node')

        self.bottle_position_publisher = self.create_publisher(Point, '/bottle_position', 10)
        self.gripper_has_bottle = False
        self.sent_once = False  # 只发一次

        package_dir = get_package_share_directory("yolo_detect")
        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")
        self.model = YOLO(model_file, verbose=False)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            qos_profile
        )
        self.create_subscription(Bool, '/gripper_has_bottle', self.gripper_state_callback, 10)

        self.get_logger().info("BottleDetectionNode has been started.")

    def gripper_state_callback(self, msg):
        self.gripper_has_bottle = msg.data
        # 夹爪有瓶子时允许下次放下后重新检测并发送
        if self.gripper_has_bottle:
            self.sent_once = False

    def listener_callback(self, msg):
        # 只有在夹爪没有瓶子且还没发过时才检测并发送
        if self.gripper_has_bottle or self.sent_once:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                if self.model.names[class_id] == "PET Bottle":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    bottle_position = Point()
                    bottle_position.x = (x1 + x2) / 2
                    bottle_position.y = (y1 + y2) / 2
                    bottle_position.z = 0.0

                    # 显示检测框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, "PET Bottle", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.bottle_position_publisher.publish(bottle_position)
                    self.get_logger().info(f"已发布瓶子位置: ({bottle_position.x:.1f}, {bottle_position.y:.1f})")
                    # self.sent_once = True  # 只发一次
                    break
            else:
                continue
            break

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