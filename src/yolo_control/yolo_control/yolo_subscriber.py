import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from flask import Flask, request
import json
import time
from .motor_controller import MotorController  # 导入 MotorController
from .arm_controller import ArmController  # 导入 ArmController
from .obstacle_avoidance import ObstacleAvoidance  # 导入 ObstacleAvoidance

app = Flask(__name__)

class YOLOSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.publisher_ = self.create_publisher(String, 'yolo_results', 10)
        self.motor_controller = MotorController()  # 初始化 MotorController
        self.arm_controller = ArmController()  # 初始化 ArmController
        self.obstacle_avoidance = ObstacleAvoidance(self.motor_controller)  # 初始化 ObstacleAvoidance
        self.target_detected = False
        self.target_position = None
        self.focal_length = 800  # 假设的焦距，需要根据实际情况校准
        self.real_height = 0.2  # 目标的实际高度（米），需要根据实际情况校准
        self.frame_width = 1280  # 假设图像宽度为640
        self.center_threshold = 50  # 中心阈值
        self.stop_distance = 30  # 停止距离（厘米）
        self.pickup_executed = False  # 新增状态标志，防止重复下发控制命令

        # 订阅激光雷达数据
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.obstacle_avoidance.laser_callback,
            10
        )

        # 定时器，每秒检查一次是否需要避障
        self.timer = self.create_timer(1.0, self.check_and_avoid_obstacles)

    def publish_results(self, results):
        if not results:
            print("No results to publish")
            return
        msg = String()
        msg.data = json.dumps(results)  # 确保发送的是 JSON 字符串
        self.publisher_.publish(msg)
        self.process_yolo_results(results)

    def process_yolo_results(self, results):
        # 解析检测结果并设置目标位置
        if results:
            self.target_detected = True
            self.target_position = results[0]
        else:
            self.target_detected = False
            self.target_position = None
            self.pickup_executed = False  # 目标丢失时重置标志

    def control_robot(self):
        if self.target_detected:
            x1, y1, x2, y2 = self.target_position['boxes'][0]
            x_center = x1 + (x2 - x1) / 2
            frame_center = self.frame_width / 2
            box_height = self.target_position['boxes'][0][3] - self.target_position['boxes'][0][1]
            distance = (self.real_height * self.focal_length) / box_height * 100  # 使用焦距和实际高度计算距离，并转换为厘米
            print(f"x_center: {x_center}, frame_center: {frame_center}, box_height: {box_height}, distance: {distance}")

            if distance < self.stop_distance:  # 距离小于30厘米
                if not self.pickup_executed:
                    self.pickup_executed = True  # 标记已执行
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('forward'))
                    time.sleep(1)
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('stop'))
                    self.arm_controller.pick_up()
            else:
                self.pickup_executed = False
                if x_center < frame_center - self.center_threshold:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateLeft'))
                elif x_center > frame_center + self.center_threshold:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateRight'))
                else:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('forward'))
        else:
            self.obstacle_avoidance.avoid_obstacles()

    def check_and_avoid_obstacles(self):
        if not self.target_detected:
            print("No target detected, avoiding obstacles")
            self.obstacle_avoidance.avoid_obstacles()

    def shutdown(self):
        self.motor_controller.stop_all_motors()
        self.arm_controller.move_to_initial_position()

yolo_subscriber = None

@app.route('/yolo_results', methods=['POST'])
def yolo_results():
    data = request.json
    results = data['results']
    yolo_subscriber.publish_results(results)
    yolo_subscriber.control_robot()
    return "Results received", 200

def main(args=None):
    global yolo_subscriber
    rclpy.init(args=args)
    yolo_subscriber = YOLOSubscriber()

    try:
        app.run(host='0.0.0.0', port=5200)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        yolo_subscriber.shutdown()
        yolo_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()