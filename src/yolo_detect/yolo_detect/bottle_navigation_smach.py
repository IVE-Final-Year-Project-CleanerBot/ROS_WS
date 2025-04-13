#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smach import StateMachine, State
from smach_ros2 import RosStateMachine
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# 定义全局变量存储目标点
current_goal = None

class NavigationState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['bottle_detected', 'goal_reached'])
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.bottle_detected = False

        # 订阅瓶子检测服务
        self.detection_sub = self.node.create_subscription(
            String,
            '/bottle_detection_status',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        if msg.data == "detected":
            self.bottle_detected = True

    def execute(self, userdata):
        global current_goal
        self.bottle_detected = False
        
        # 发送导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = current_goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        # 轮询检测状态
        while rclpy.ok():
            if self.bottle_detected:
                goal_handle.cancel_goal_async()
                return 'bottle_detected'
            
            if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
                return 'goal_reached'
            
            self.node.get_clock().sleep_for(0.1)

class PickupState(State):
    def __init__(self, node):
        State.__init__(self, outcomes=['done'])
        self.node = node
        self.arm_client = ActionClient(node, ControlArm, 'control_arm')

    def execute(self, userdata):
        # 执行抓取动作
        goal_msg = ControlArm.Goal()
        goal_msg.action = "pick"
        self.arm_client.wait_for_server()
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        return 'done'

def main(args=None):
    rclpy.init(args=args)
    node = Node('navigation_smach')

    # 初始化状态机
    sm = StateMachine(outcomes=['finished'])
    sm.userdata.goal = current_goal  # 从全局变量获取目标

    with sm:
        StateMachine.add('NAVIGATE', 
                        NavigationState(node),
                        transitions={'bottle_detected': 'PICKUP',
                                    'goal_reached': 'finished'})
        
        StateMachine.add('PICKUP',
                        PickupState(node),
                        transitions={'done': 'NAVIGATE'})

    # 创建ROS2状态机执行器
    sis = RosStateMachine(node, sm)
    sis.execute()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()