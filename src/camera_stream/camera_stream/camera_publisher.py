import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.0167, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # 打开 USB 摄像头
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置分辨率宽度
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置分辨率高度
        self.cap.set(cv2.CAP_PROP_FPS, 60)  # 设置帧率为 60FPS
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 将 OpenCV 图像转换为 ROS 图像消息
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Failed to capture image")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # 确保 rclpy 仍然处于运行状态
            rclpy.shutdown()

if __name__ == '__main__':
    main()