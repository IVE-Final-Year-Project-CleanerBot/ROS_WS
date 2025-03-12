import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import driver_ros2.ros_robot_controller_sdk as rrc
from flask import Flask, request
import threading
import json

app = Flask(__name__)

class YOLOSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.publisher_ = self.create_publisher(String, 'yolo_results', 10)
        self.board = rrc.Board()
        self.board.enable_reception(True)  # 启用接收模式
        self.target_detected = False
        self.target_position = None
        self.focal_length = 800  # 假设的焦距，需要根据实际情况校准
        self.real_height = 0.2  # 目标的实际高度（米），需要根据实际情况校准
        self.frame_width = 1280  # 假设图像宽度为640
        self.center_threshold = 20  # 中心阈值
        self.stop_distance = 30  # 停止距离（厘米）

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

    def control_robot(self):
        if self.target_detected:
            x_center = self.target_position['boxes'][0][0] + (self.target_position['boxes'][0][2] - self.target_position['boxes'][0][0]) / 2
            frame_center = self.frame_width / 2
            box_height = self.target_position['boxes'][0][3] - self.target_position['boxes'][0][1]
            distance = (self.real_height * self.focal_length) / box_height * 100  # 使用焦距和实际高度计算距离，并转换为厘米
            # 打印调试信息
            print(f"x_center: {x_center}, frame_center: {frame_center}, box_height: {box_height}, distance: {distance}")

            if distance < self.stop_distance:  # 距离小于50厘米
                self.set_wheel_speeds(*self.get_motor_commands('stop'))  # 停止前进
            elif x_center < frame_center - self.center_threshold:
                self.set_wheel_speeds(*self.get_motor_commands('rotateLeft'))  # 向左转
            elif x_center > frame_center + self.center_threshold:
                self.set_wheel_speeds(*self.get_motor_commands('rotateRight'))  # 向右转
            else:
                self.set_wheel_speeds(*self.get_motor_commands('forward'))  # 向前移动
        else:
            self.set_wheel_speeds(*self.get_motor_commands('stop'))  # 停止前进

    def get_motor_commands(self, Id):
        SPEED = 15
        switcher = {
            'forward': [SPEED, SPEED, -SPEED, -SPEED],
            'backward': [-SPEED, -SPEED, SPEED, SPEED],
            'right': [-SPEED, SPEED, -SPEED, SPEED],
            'left': [SPEED, -SPEED, SPEED, -SPEED],
            'forwardLeft': [SPEED, 0, 0, -SPEED],
            'forwardRight': [0, SPEED, -SPEED, 0],
            'backwardLeft': [-SPEED, 0, 0, SPEED],
            'backwardRight': [0, -SPEED, SPEED, 0],
            'rotateLeft': [-SPEED, -SPEED, -SPEED, -SPEED],
            'rotateRight': [SPEED, SPEED, SPEED, SPEED],
            'stop': [0, 0, 0, 0]
        }
        return switcher.get(Id, [0, 0, 0, 0])

    def set_wheel_speeds(self, v1, v2, v3, v4):
        self.board.set_motor_duty([[1, v1], [2, v2], [3, v3], [4, v4]])

    def stop_all_motors(self):
        self.board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

yolo_subscriber = None

@app.route('/yolo_results', methods=['POST'])
def yolo_results():
    data = request.json
    results = data['results']
    yolo_subscriber.publish_results(results)
    yolo_subscriber.control_robot()
    return "Results received", 200

def flask_thread():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global yolo_subscriber
    rclpy.init(args=args)
    yolo_subscriber = YOLOSubscriber()

    flask_thread_instance = threading.Thread(target=flask_thread)
    flask_thread_instance.start()

    rclpy.spin(yolo_subscriber)

    yolo_subscriber.stop_all_motors()
    yolo_subscriber.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()