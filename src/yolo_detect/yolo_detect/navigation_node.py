import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import SetBool

class BottleNavigationNode(Node):
    def __init__(self):
        super().__init__('bottle_navigation_node')

        # 订阅 YOLO 检测结果
        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.yolo_callback,
            10
        )

        # 创建导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.is_navigating = True  # 假设导航已经开始

        # 创建服务客户端，用于通知 YOLO 检测节点执行拾取
        self.pickup_client = self.create_client(SetBool, '/detection_status')

    def yolo_callback(self, msg):
        """处理 YOLO 检测结果"""
        if 'PET Bottle' in msg.data and self.is_navigating:
            self.get_logger().info("Bottle detected! Pausing navigation...")
            self.pause_navigation()
            self.execute_pickup()

    def pause_navigation(self):
        """暂停导航"""
        self.nav_client.cancel_all_goals()
        self.is_navigating = False
        self.get_logger().info("Navigation paused.")

    def execute_pickup(self):
        """通知 YOLO 检测节点执行拾取"""
        self.get_logger().info("Notifying YOLO node to pick up the bottle...")
        req = SetBool.Request()
        req.data = True
        self.pickup_client.wait_for_service()
        future = self.pickup_client.call_async(req)
        future.add_done_callback(self.pickup_done)

    def pickup_done(self, future):
        """拾取完成回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Pickup complete. Resuming navigation...")
                self.resume_navigation()
            else:
                self.get_logger().error("Pickup failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def resume_navigation(self):
        """继续导航"""
        self.is_navigating = True
        self.get_logger().info("Resuming navigation...")

def main(args=None):
    rclpy.init(args=args)
    node = BottleNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()