#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smach import StateMachine, State
from smach_ros import IntrospectionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class WanderState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['bottle_detected'])
        self.node = node
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_cancel_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')  # Nav2 目标取消客户端
        self.bottle_detected = False

        # 订阅瓶子检测状态
        self.detection_sub = self.node.create_subscription(
            Float32MultiArray,
            '/bottle_detection_data',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        # 如果检测到瓶子，保存检测数据
        if len(msg.data) == 4:  # 确保接收到有效的检测框数据
            self.bottle_detected = True
            self.bbox_data = msg.data

    def execute(self, userdata):
        self.bottle_detected = False
        self.node.get_logger().info("Wandering...")

        # 让 Nav2 控制机器人移动
        while rclpy.ok():
            if self.bottle_detected:
                # 暂停导航
                self.node.get_logger().info("Bottle detected, pausing navigation...")
                self.pause_navigation()
                return 'bottle_detected'

            # 使用 rclpy.spin_once 代替 time.sleep
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def pause_navigation(self):
        """暂停导航"""
        # 取消当前导航目标
        self.node.get_logger().info("Cancelling Nav2 goal...")
        if not self.nav_cancel_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Nav2 action server not available!")
            return

        # 发送取消请求
        cancel_future = self.nav_cancel_client.cancel_all_goals()
        rclpy.spin_until_future_complete(self.node, cancel_future)
        if cancel_future.result():
            self.node.get_logger().info("Nav2 goal cancelled successfully.")
        else:
            self.node.get_logger().error("Failed to cancel Nav2 goal.")

        # 停止机器人
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

class PickupState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['done'])
        self.node = node
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_command_publisher = self.node.create_publisher(String, '/arm_command', 10)

        # 可调整参数
        self.x_tolerance = 50  # 中心点 x 的容忍范围（像素）
        self.y_threshold_factor = 1.5 / 3  # 中心点 y 的阈值比例（图像高度的 1/2）
        self.linear_speed = 0.1  # 线速度
        self.fixed_angular_speed = 0.45  # 固定角速度

    def execute(self, userdata):
        self.node.get_logger().info("Aligning with bottle and picking up...")

        # 使用检测框数据对齐和靠近瓶子
        while rclpy.ok():
            # 从检测节点获取检测框数据
            bbox_data = self.bbox_data
            if bbox_data is None:
                self.node.get_logger().warn("No bounding box data received.")
                return 'done'

            x1, y1, x2, y2 = bbox_data
            bbox_center_x = (x1 + x2) / 2
            bbox_center_y = (y1 + y2) / 2
            bbox_height = y2 - y1
            image_width = 640
            image_height = 480

            # 计算偏移量
            offset_x = bbox_center_x - (image_width / 2)
            y_threshold = image_height * self.y_threshold_factor

            twist = Twist()

            # 阶段 1：水平对齐
            if abs(offset_x) > self.x_tolerance:
                twist.angular.z = self.fixed_angular_speed if offset_x < 0 else -self.fixed_angular_speed
                twist.linear.x = 0.0
                self.node.get_logger().info(f"Aligning X... Angular.z={twist.angular.z}")
            # 阶段 2：向前移动
            elif bbox_height < y_threshold:
                twist.angular.z = 0.0
                twist.linear.x = self.linear_speed
                self.node.get_logger().info("Moving forward...")
            # 阶段 3：执行拾取
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist)
                self.node.get_logger().info("Bottle is in position, picking up...")
                self.arm_command_publisher.publish(String(data="move_to_pick"))
                rclpy.spin_once(self.node, timeout_sec=2.0)  # 等待拾取完成
                self.resume_navigation()  # 恢复导航
                return 'done'

            # 发布速度指令
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def resume_navigation(self):
        """恢复导航"""
        self.node.get_logger().info("Resuming navigation...")

def main(args=None):
    rclpy.init(args=args)
    node = Node('navigation_smach')

    # 初始化状态机
    sm = StateMachine(outcomes=['finished'])

    with sm:
        StateMachine.add('WANDER', 
                        WanderState(node),
                        transitions={'bottle_detected': 'PICKUP'})
        
        StateMachine.add('PICKUP',
                        PickupState(node),
                        transitions={'done': 'WANDER'})

    # 创建状态机调试器
    sis = IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    # 执行状态机
    outcome = sm.execute()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()