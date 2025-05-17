import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class MiDaSNode(Node):
    def __init__(self):
        super().__init__('midas_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/camera/depth', 10)
        self.bridge = CvBridge()

        # 加载MiDaS模型
        self.model_type = "MiDaS_small"  # 可选：DPT_Large, DPT_Hybrid, MiDaS_small
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.midas.to(self.device)
        self.midas.eval()
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        if self.model_type in ["DPT_Large", "DPT_Hybrid"]:
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform

        self.get_logger().info("MiDaS model loaded.")

    def image_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
        depth = prediction.cpu().numpy()

        # 归一化到0-255并转为8位灰度图
        depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_uint8 = depth_norm.astype(np.uint8)

        # OpenCV -> ROS Image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding="mono8")
        depth_msg.header = msg.header
        self.publisher_.publish(depth_msg)

        # 可选：显示深度图
        cv2.imshow("Depth", depth_uint8)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MiDaSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()