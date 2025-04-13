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
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from action_msgs.srv import GoalResponse


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        # 初始化电机控制发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化机械臂控制发布器
        self.arm_command_publisher = self.create_publisher(String, '/arm_command', 10)

        # 初始化 Nav2 Action 客户端
        self.nav_cancel_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

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
        self.y_threshold_factor = 1.5 / 3  # 中心点 y 的阈值比例（图像高度的 1/2）
        self.linear_speed = 0.1  # 线速度
        self.fixed_angular_speed = 0.45  # 固定角速度
        self.angular_speed_factor = -0.005  # 动态角速度调整因子

        self.get_logger().info("YoloDetectNode has been started.")

    def detection_callback(self, request, response):
        # 根据当前检测状态返回结果
        response.success = self.current_detection
        response.message = "Bottle detected" if self.current_detection else "No bottle detected"
        return response

    def drive_to_target(self, bbox_center_x, image_width, bbox_center_y, image_height):
        """根据目标位置控制机器人移动"""
        twist = Twist()

        # 计算 y 阈值
        y_threshold = image_height * self.y_threshold_factor

        # 计算 x 偏移量
        offset_x = bbox_center_x - (image_width / 2)

        # 阶段 1：水平对齐
        if abs(offset_x) > self.x_tolerance:
            twist.angular.z = self.fixed_angular_speed if offset_x < 0 else -self.fixed_angular_speed  # 固定角速度
            twist.linear.x = 0.0  # 停止向前移动
            self.get_logger().info(f"Aligning X... Angular.z={twist.angular.z}")
        # 阶段 2：向前移动
        elif bbox_center_y < y_threshold:
            twist.angular.z = 0.0  # 停止旋转
            twist.linear.x = self.linear_speed  # 向前移动
            self.get_logger().info("Moving forward...")
        # 阶段 3：执行机械臂操作
        else:
            twist.angular.z = 0.0  # 停止旋转
            twist.linear.x = 0.0  # 停止移动
            self.cmd_vel_publisher.publish(twist)  # 确保机器人停止
            self.get_logger().info("Bottle is in position, picking up...")
            self.pick_up_bottle()
            return  # 结束函数，避免重复发布速度指令

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

                    # 取消 Nav2 的导航目标
                    self.cancel_nav_goal()

                    # 计算检测框的中心点
                    bbox_center_x = (x1 + x2) / 2
                    bbox_center_y = (y1 + y2) / 2

                    # 输出中心点的 x 和 y 值
                    self.get_logger().info(f"Center X: {bbox_center_x}, Center Y: {bbox_center_y}")

                    # 判断中心点是否在镜头中间
                    if abs(bbox_center_x - image_width / 2) <= self.x_tolerance and bbox_center_y >= image_height * self.y_threshold_factor:
                        self.get_logger().info("Bottle is in position, picking up...")
                        # 停止机器人移动
                        self.stop_robot()

                        # 执行机械臂拾取动作
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

    def cancel_nav_goal(self):
        """取消 Nav2 的导航目标"""
        self.get_logger().info("Cancelling Nav2 goal...")

        # 检查 Action 客户端是否可用
        if not self.nav_cancel_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        # 获取当前所有目标句柄
        futures = self.nav_cancel_client._action_client.get_pending_goal_async()
        if not futures:
            self.get_logger().warn("No active goals to cancel.")
            return

        # 遍历并取消所有目标
        for future in futures:
            goal_handle = future.result()
            if goal_handle.status == GoalStatus.STATUS_ACCEPTED or goal_handle.status == GoalStatus.STATUS_EXECUTING:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                if cancel_future.result().return_code == GoalResponse.ERROR_NONE:
                    self.get_logger().info("Successfully cancelled goal.")
                else:
                    self.get_logger().error("Failed to cancel goal.")
            else:
                self.get_logger().warn("Goal is not in a cancellable state.")

    def stop_robot(self):
        """停止机器人移动"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped.")

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