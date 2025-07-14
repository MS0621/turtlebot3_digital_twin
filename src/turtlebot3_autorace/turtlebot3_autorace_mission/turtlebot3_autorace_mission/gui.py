import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout , QPushButton , QProgressBar
from PyQt5.QtGui import QPixmap, QImage
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
import cv2
import numpy as np
import os
from ultralytics import YOLO
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
class CameraNode(Node):
    def __init__(self, update_callback = None , line_callback = None , cmd_vel_callback = None , yolo_callback = None):
        super().__init__('camera_qt_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/detect/image_traffic_light/compressed',
            self.listener_callback,
            10)
        self.yolo_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.yolo_listener_callback,
            10)
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_listener_callback,
            10)
        self.line_subscription = self.create_subscription(
            CompressedImage,
            '/detect/image_lane/compressed',
            self.lines_listener_callback,
            10)
        self.bridge = CvBridge()
        self.update_callback = update_callback
        self.cmd_vel_callback = cmd_vel_callback
        self.line_callback = line_callback
        self.yolo_callback = yolo_callback
        self.pub_stop = self.create_publisher(Float64, '/control/max_vel',1)
        self.model = YOLO("yolov8n.pt")
    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.update_callback(cv_image)
        except Exception as e:
            print("sadads")
    def cmd_vel_listener_callback(self, msg):
        self.cmd_vel_callback(msg)
    def lines_listener_callback(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.line_callback(cv_image)
    def yolo_listener_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        annotated_frame = results[0].plot()
        self.yolo_callback(annotated_frame)
    def publish_stop(self, stop):
        msg = Float64()
        msg.data = stop
        self.pub_stop.publish(msg)
        self.get_logger().info(f'Stop 신호 퍼블리시: {stop}')
class CameraWidget(QWidget):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.setGeometry(100, 100, 1500, 800)
        self.setWindowTitle("Camera Viewer")
        self.image_label = QLabel(self)
        self.image_label.move(20,80)
        self.lane_image_label = QLabel(self)
        self.lane_image_label.move(400,0)
        self.lane_image_label.resize(400, 400)
        self.yolo_image_label = QLabel(self)
        self.yolo_image_label.setStyleSheet('background-color: black;')
        self.yolo_image_label.move(1000, 100)
        self.yolo_image_label.resize(400, 400)
        self.stop_button = QPushButton("Stop", self)
        self.stop_button.resize(80, 30)
        self.stop_button.move(20, 350)
        self.stop_button.clicked.connect(lambda:self.stop_robot(True))
        self.start_button = QPushButton("Start", self)
        self.start_button.move(120, 350)
        self.start_button.resize(80, 30)
        self.start_button.clicked.connect(lambda:self.stop_robot(False))
        self.vel_progress = QProgressBar(self)
        self.vel_progress.setGeometry(20, 500, 900, 50)
        self.vel_progress.setMinimum(0)
        self.vel_progress.setMaximum(100)
        self.ang_progress = QProgressBar(self)
        self.ang_progress.setGeometry(20, 600, 900, 50)
        self.ang_progress.setMinimum(-100)
        self.ang_progress.setMaximum(100)
        self.frame_count = 0
        self.yolo_frame = None
        self.main_frame = None
        self.lane_frame = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_gui)
        self.timer.start(66)
    def update_image(self, cv_image):
        self.main_frame = cv_image
    def update_yolo_image(self, cv_image):
        self.yolo_frame = cv_image
    def update_line_image(self, cv_image):
        self.lane_frame  = cv_image
    def update_cmd_vel(self, msg):
        x = msg.linear.x
        z = msg.angular.z
        if x and z:
            vel_progress_value = int(min(max(x * 1000, 0), 100))
            ang_progress_value = int(np.clip(z * 5000, -100, 100))
            self.vel_progress.setValue(vel_progress_value)
            self.ang_progress.setValue(ang_progress_value)
    def refresh_gui(self):
        if self.yolo_frame is not None:
            height, width, channel = self.yolo_frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(self.yolo_frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            if self.yolo_image_label.pixmap() != pixmap:
                self.yolo_image_label.setPixmap(pixmap)
                self.yolo_image_label.resize(width, height)
        if self.main_frame is not None:
            height, width, channel = self.main_frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(self.main_frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            if self.image_label.pixmap() != pixmap:
                self.image_label.setPixmap(pixmap)
                self.image_label.resize(width, height)
        if self.lane_frame is not None:
            height, width, channel = self.lane_frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(self.lane_frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(400, 400, aspectRatioMode=1)
            if self.lane_image_label.pixmap() != scaled_pixmap:
                self.lane_image_label.setPixmap(scaled_pixmap)
    def stop_robot(self,stop):
        if stop:
            self.node.publish_stop(0.0)
        else:
            self.node.publish_stop(0.1)
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = CameraNode(None)
    camera_widget = CameraWidget(node)
    node.update_callback = camera_widget.update_image
    node.cmd_vel_callback = camera_widget.update_cmd_vel
    node.line_callback = camera_widget.update_line_image
    node.yolo_callback = camera_widget.update_yolo_image
    camera_widget.show()
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    import threading
    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()