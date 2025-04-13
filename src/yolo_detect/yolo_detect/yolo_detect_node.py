from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import os
import cv2


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        # 初始化电机控制发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

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

        # 可调整参数
        self.x_tolerance = 50  # 中心点 x 的容忍范围（像素）
        self.y_threshold_factor = 2 / 3  # 中心点 y 的阈值比例（图像高度的 2/3）
        self.linear_speed = 0.1  # 线速度
        self.angular_speed_factor = -0.002  # 角速度调整因子

        self.get_logger().info("YoloDetectNode has been started.")

    def drive_to_target(self, bbox_center_x, image_width, bbox_center_y, image_height):
        """根据目标位置控制机器人移动"""
        twist = Twist()

        # 计算 y 阈值
        y_threshold = image_height * self.y_threshold_factor

        if bbox_center_y < y_threshold:  # 如果中心点 y 小于阈值，向前移动
            twist.linear.x = self.linear_speed

            # 只有当 y 小于阈值时才调整角速度
            offset_x = bbox_center_x - (image_width / 2)
            twist.angular.z = self.angular_speed_factor * offset_x  # 调整旋转速度
        else:
            twist.linear.x = 0.0  # 停止移动
            twist.angular.z = 0.0  # 停止旋转

        # 发布速度指令
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Driving to target: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width, _ = frame.shape  # 获取图像分辨率

        # 运行 YOLO 检测
        results = self.model(frame)

        # 标志变量，用于判断是否检测到 "PET Bottle"
        detected_bottle = False

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

                # 如果检测到的是塑料瓶
                if self.model.names[class_id] == "PET Bottle":
                    detected_bottle = True  # 标记为检测到瓶子

                    # 计算检测框的中心点
                    bbox_center_x = (x1 + x2) / 2
                    bbox_center_y = (y1 + y2) / 2

                    # 输出中心点的 y 值
                    self.get_logger().info(f"Center Y: {bbox_center_y}")

                    # 判断中心点是否在镜头中间
                    if abs(bbox_center_x - image_width / 2) <= self.x_tolerance and bbox_center_y >= image_height * self.y_threshold_factor:
                        self.get_logger().info("Bottle is in position, picking up...")
                        self.pick_up_bottle()
                    else:
                        # 控制机器人移动到瓶子面前
                        self.drive_to_target(bbox_center_x, image_width, bbox_center_y, image_height)

        # 如果没有检测到 "PET Bottle"，停止机器人并重置机械臂
        if not detected_bottle:
            self.stop_robot_and_reset_arm()

        # 显示检测结果
        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def stop_robot_and_reset_arm(self):
        """停止机器人并重置机械臂"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        self.arm_command_publisher.publish(String(data="reset"))

    def pick_up_bottle(self):
        """控制机械臂拾取瓶子"""
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
        # 停止机器人移动
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_publisher.publish(twist)
        node.get_logger().info("Robot stopped.")

        # 重置机械臂
        node.arm_command_publisher.publish(String(data="reset"))
        node.get_logger().info("Arm reset command sent.")

        # 关闭 OpenCV 窗口
        cv2.destroyAllWindows()

        # 关闭 ROS2 节点
        rclpy.shutdown()


if __name__ == '__main__':
    main()