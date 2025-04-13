import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from multiprocessing import Process, Queue

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # 创建进程间队列
        self.queue = Queue()

        # 启动摄像头捕获进程
        self.process = Process(target=self.capture_frames, args=(self.queue,))
        self.process.start()

        # 创建定时器，用于从队列中获取图像并发布
        self.timer = self.create_timer(0.0167, self.publish_frames)

    def capture_frames(self, queue):
        cap = cv2.VideoCapture(0)  # 打开 USB 摄像头
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置分辨率宽度
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置分辨率高度
        cap.set(cv2.CAP_PROP_FPS, 60)  # 设置帧率为 60FPS

        if not cap.isOpened():
            print("Unable to open camera")
            return

        while True:
            ret, frame = cap.read()
            if ret:
                if not queue.full():
                    queue.put(frame)  # 将帧放入队列
            else:
                print("Failed to capture image")

        cap.release()

    def publish_frames(self):
        if not self.queue.empty():
            frame = self.queue.get()  # 从队列中获取帧
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

    def destroy_node(self):
        self.process.terminate()  # 停止子进程
        self.process.join()  # 等待子进程结束
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # 确保 rclpy 仍然处于运行状态
            rclpy.shutdown()

if __name__ == '__main__':
    main()