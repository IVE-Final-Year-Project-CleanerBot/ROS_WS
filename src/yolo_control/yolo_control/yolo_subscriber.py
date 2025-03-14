import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import driver_ros2.ros_robot_controller_sdk as rrc
from flask import Flask, request
import json
from .motor_controller import MotorController  # 导入 MotorController
from .arm_controller import ArmController  # 导入 ArmController

app = Flask(__name__)

class YOLOSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.publisher_ = self.create_publisher(String, 'yolo_results', 10)
        self.motor_controller = MotorController()  # 初始化 MotorController
        self.arm_controller = ArmController()  # 初始化 ArmController
        self.target_detected = False
        self.target_position = None
        self.focal_length = 800  
        self.real_height = 0.2  
        self.frame_width = 1280  
        self.center_threshold = 50  
        self.stop_distance = 20  
        # 新增状态标志，防止重复下发控制命令
        self.pickup_executed = False

    def publish_results(self, results):
        if not results:
            print("No results to publish")
            return
        msg = String()
        msg.data = json.dumps(results)
        self.publisher_.publish(msg)
        self.process_yolo_results(results)

    def process_yolo_results(self, results):
        # 如果有检测结果，保存第一个检测到的对象作为目标
        if results:
            self.target_detected = True
            self.target_position = results[0]
        else:
            self.target_detected = False
            self.target_position = None
            # 目标丢失时重置标志
            self.pickup_executed = False

    def control_robot(self):
        if self.target_detected:
            x1, y1, x2, y2 = self.target_position['boxes'][0]
            x_center = x1 + (x2 - x1) / 2
            frame_center = self.frame_width / 2
            box_height = y2 - y1
            # 计算距离（单位：厘米）
            distance = (self.real_height * self.focal_length) / box_height * 100  
            print(f"x_center: {x_center}, frame_center: {frame_center}, box_height: {box_height}, distance: {distance}")

            # 当目标距离小于停止距离时只执行一次拾取动作
            if distance < self.stop_distance:
                if not self.pickup_executed:
                    self.pickup_executed = True  # 标记已执行
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('stop'))
                    self.arm_controller.pick_up()
            else:
                # 如果距离恢复到大于阈值，则重置标志，允许再次检测和控制
                self.pickup_executed = False
                if x_center < frame_center - self.center_threshold:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateLeft'))
                elif x_center > frame_center + self.center_threshold:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('rotateRight'))
                else:
                    self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('forward'))
        else:
            self.motor_controller.set_wheel_speeds(*self.motor_controller.get_motor_commands('stop'))

    def shutdown(self):
        # 停止所有马达并将机械臂移动到初始位置
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