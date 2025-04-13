import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import multiprocessing as mp
import numpy as np
from multiprocessing import Queue, Event

class CameraCapture(mp.Process):
    def __init__(self, output_queue, stop_event):
        super().__init__()
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.frame_size = (640, 480)
        self.fps = 60

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if ret:
                # 使用numpy数组提高传输效率
                self.output_queue.put(frame.copy())
            else:
                print("Capture process: Failed to read frame")

        cap.release()
        print("Capture process exited")

class ImagePublisher(Node):
    def __init__(self, input_queue):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.input_queue = input_queue
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        if not self.input_queue.empty():
            frame = self.input_queue.get()
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Publish error: {str(e)}")

def publisher_process(input_queue, stop_event):
    rclpy.init()
    node = ImagePublisher(input_queue)
    
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.001)
    
    node.destroy_node()
    rclpy.shutdown()
    print("Publisher process exited")

if __name__ == '__main__':
    # 进程间通信队列（设置最大缓存3帧）
    frame_queue = Queue(maxsize=3)
    stop_event = Event()

    # 启动采集进程
    capture_proc = CameraCapture(frame_queue, stop_event)
    capture_proc.start()

    # 启动发布进程
    publisher_proc = mp.Process(
        target=publisher_process,
        args=(frame_queue, stop_event)
    )
    publisher_proc.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_event.set()
        
        # 清空队列保证快速退出
        while not frame_queue.empty():
            frame_queue.get()

        capture_proc.join(timeout=1.0)
        publisher_proc.join(timeout=1.0)

    print("Main process exited")