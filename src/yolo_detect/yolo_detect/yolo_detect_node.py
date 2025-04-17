from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import os
import cv2
import torch
from torchvision.transforms import Compose, Resize, ToTensor, Normalize
import numpy as np


class YoloDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')

        # 初始化 YOLO 模型
        package_dir = get_package_share_directory("yolo_detect")
        model_file = os.path.join(package_dir, "config", "model", "yolo11.pt")
        self.model = YOLO(model_file, verbose=False)  # 加载 YOLO 模型

        # 初始化 MiDaS 深度估计模型
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_type = "MiDaS_small"  # 可选: "DPT_Large", "DPT_Hybrid", "MiDaS_small"
        self.depth_model = torch.hub.load("isl-org/MiDaS", model_type)
        self.depth_model.to(self.device)
        self.depth_model.eval()

        # 定义 MiDaS 输入图像的预处理
        midas_transforms = torch.hub.load("isl-org/MiDaS", "transforms")
        if model_type in ["DPT_Large", "DPT_Hybrid"]:
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform

        # 初始化图像处理
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        self.get_logger().info("YoloDepthNode has been started.")

    def estimate_depth(self, image):
        """使用 MiDaS 模型估计深度"""
        try:
            input_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            input_tensor = self.transform(input_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                depth_map = self.depth_model(input_tensor)
                depth_map = torch.nn.functional.interpolate(
                    depth_map.unsqueeze(1),
                    size=image.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze().cpu().numpy()

            # 归一化深度图以便可视化
            depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            return depth_map, depth_map_normalized
        except Exception as e:
            self.get_logger().error(f"Depth estimation failed: {e}")
            return None, None

    def calculate_3d_position(self, bbox_center_x, bbox_center_y, depth, image_width, image_height):
        """计算物体在相机坐标系下的 3D 坐标"""
        fx, fy = 800, 800  # 假设的焦距（像素）
        cx, cy = image_width / 2, image_height / 2  # 主点（图像中心）

        X = (bbox_center_x - cx) * depth / fx
        Y = (bbox_center_y - cy) * depth / fy
        Z = depth
        return X, Y, Z

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS image to OpenCV format: {e}")
            return

        image_height, image_width, _ = frame.shape  # 获取图像分辨率

        # 运行 YOLO 检测
        results = self.model(frame)

        # 估算深度
        depth_map, depth_map_normalized = self.estimate_depth(frame)
        if depth_map is None:
            return

        # 遍历检测结果并输出物体位置
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

                # 如果检测到的是塑料瓶
                if self.model.names[class_id] == "PET Bottle":
                    # 计算检测框的中心点
                    bbox_center_x = (x1 + x2) / 2
                    bbox_center_y = (y1 + y2) / 2

                    # 检查中心点是否在深度图范围内
                    if 0 <= int(bbox_center_y) < depth_map.shape[0] and 0 <= int(bbox_center_x) < depth_map.shape[1]:
                        depth = depth_map[int(bbox_center_y), int(bbox_center_x)]
                        self.get_logger().info(f"Center X: {bbox_center_x}, Center Y: {bbox_center_y}, Depth: {depth:.2f}")

                        # 计算 3D 坐标
                        X, Y, Z = self.calculate_3d_position(bbox_center_x, bbox_center_y, depth, image_width, image_height)
                        self.get_logger().info(f"Object Position: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")
                    else:
                        self.get_logger().warning("Bounding box center is out of depth map range.")

        # 显示检测结果和深度图
        cv2.imshow("YOLO Detection", frame)
        cv2.imshow("Depth Map", depth_map_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭 OpenCV 窗口
        cv2.destroyAllWindows()

        # 关闭 ROS2 节点
        rclpy.shutdown()


if __name__ == '__main__':
    main()