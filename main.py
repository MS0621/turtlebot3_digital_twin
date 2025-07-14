import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout , QPushButton , QProgressBar , QDial
from PyQt5.QtGui import QPixmap, QImage
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
import cv2
import numpy as np
import os
from ultralytics import YOLO
from frame_thread import FrameThread
from RosSpinThread import RosSpinThread
import logging
import time
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
logging.getLogger("ultralytics").setLevel(logging.ERROR)
class CameraNode(Node):
    def __init__(self, update_callback = None , line_callback = None , cmd_vel_callback = None , yolo_callback = None , state_callback = None,
                 parking_callback = None):
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
        
        self.status_subscription = self.create_subscription(
            String, '/traffic_light_status', self.status_listener, 10)
        
        self.bridge = CvBridge()
        self.update_callback = update_callback
        self.cmd_vel_callback = cmd_vel_callback
        self.line_callback = line_callback
        self.yolo_callback = yolo_callback
        self.update_state_callback = state_callback
        self.parking_callback = parking_callback
        self.pub_stop = self.create_publisher(Float64, '/control/max_vel',1)
        self.park = self.create_publisher(Bool, '/detect/parking_sign_detected', 1)
        self.start = self.create_publisher(Bool, '/start', 1)
        self.model = YOLO("best.pt")
        self.yolo_class_names = self.model.names
        self.row_duplicate_state = False
        self.x = 0.0
        self.z = 0.0
        self.yolo_detect_state = ''
        self.stop_count = 0
        self.parking_count = 0 

    def status_listener(self, msg):
        status = msg.data
        if self.stop_state:
            if status == "RED":
                self.publish_stop(0.0)
            elif status == "YELLOW":
                self.publish_stop(0.05)
            elif status == "GREEN":
                self.publish_stop(0.1)

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            x1, y1 = 80, 150
            x2, y2 = 250, 250
            cropped_image = cv_image[y1:y2, x1:x2]
            self.stop_state = self.is_green_present(cropped_image)
            self.children_state = self.is_red_present(cropped_image)
            self.row_velocity(self.children_state)
            self.update_callback(cv_image)
            if self.children_state:
                self.update_state_callback(1)
            elif self.stop_state:
                self.update_state_callback(2)
            else:
                self.update_state_callback(0)
        except Exception as e:
            print("listener_callback error:", e) 

    def is_green_present(self,image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        return np.any(green_mask > 0)
    
    def is_red_present(self,image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2
        return np.any(red_mask > 0)
    
    def row_velocity(self,state):
        if state and self.x != 0.03:
            self.publish_stop(0.03)
        else:
            if self.x > 0.02 and self.x < 0.03: 
                self.publish_stop(0.1)

    def cmd_vel_listener_callback(self, msg):
        self.x = msg.linear.x
        self.z = msg.angular.z
        self.cmd_vel_callback(msg)

    def lines_listener_callback(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.line_callback(cv_image)

    def yolo_listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        annotated_frame = cv_image.copy()
        # --- yolo_listener_callback 메소드에 수정된 부분 ---6/12일 표지판
        current_frame_has_valid_sign = False # 현재 프레임에서 유효한 표지판이 감지되었는지 여부
        # 감지된 모든 객체에 대해 반복 (수정: 신뢰도 0.7 이상만)
        for r in results:
            for box in r.boxes:
                confidence = float(box.conf[0]) # 탐지 신뢰도
                class_id = int(box.cls[0]) # 탐지 클래스 ID
                # 최소 신뢰도(예: 0.7)를 넘는 탐지 결과만 
                if confidence > 0.9:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # bounding box 좌표
                    label = self.yolo_class_names.get(class_id, "UNKNOWN")
                    label_text = f"{label} {confidence:.2f}"
                    color = (0, 255, 0)  # 초록색 박스
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated_frame, label_text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    detected_sign_name_raw = self.yolo_class_names.get(class_id, None)
                    if detected_sign_name_raw is not None:
                        # ##수정##: YOLO 클래스 이름은 소문자이므로, perform_action에 대문자로 전달하려면 변환
                        detected_sign_name_upper = detected_sign_name_raw.upper()
                        # 감지된 표지판에 따라 로봇 동작 수행
                        self.perform_action(detected_sign_name_upper)
                        current_frame_has_valid_sign = True # 유효한 표지판이 감지되었음을 표시
                        # 하나의 표지판만 감지되면 바로 처리하고 루프 종료 (선택 사항: 여러 표지판 처리 시 이 break 제거)
                        break
            if current_frame_has_valid_sign:
                break # 이미 유효한 표지판을 찾고 동작을 수행했으므로 외부 루프도 종료
        self.yolo_callback(annotated_frame)

        
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # results = self.model(cv_image)
        # allowed_labels = ['traffic light'] 
        # filtered_boxes = []
        # for result in results:
        #     names = result.names
        #     boxes = result.boxes

        #     for i in range(len(boxes.cls)):
        #         class_id = int(boxes.cls[i].item())
        #         label = names[class_id]
        #         conf = float(boxes.conf[i].item())

        #         if label in allowed_labels and conf >= 0.5:
        #             filtered_boxes.append({
        #                 'xyxy': boxes.xyxy[i],
        #                 'conf': conf,
        #                 'cls': class_id,
        #             })

        # annotated_frame = cv_image.copy()
        # for box in filtered_boxes:
        #     x1, y1, x2, y2 = map(int, box['xyxy'])
        #     conf = float(box['conf'])
        #     class_id = int(box['cls'])
        #     label = f"{names[class_id]} {conf:.2f}"
        #     cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0,255,0), 2)
        #     cv2.putText(annotated_frame, label, (x1, y1 - 10),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        #     self.yolo_callback(annotated_frame)
    def perform_action(self, sign_name):
        """
        감지된 표지판 이름에 따라 특정 로봇 동작을 수행합니다.
        """
        if sign_name == 'STOP':
            self.stop_count += 1
            if self.stop_count % 100 == 0:
                self.publish_stop(0.01)
                time.sleep(3.0)
                self.publish_stop(0.05)
            self.yolo_detect_state = 'STOP'
        elif sign_name == 'PARKING':
            self.parking_count += 1
            if self.parking_count % 10 == 0:
                self.parking_callback(True)
        elif sign_name == 'LEFT':
            pass
        
    def publish_stop(self, stop):
        msg = Float64()
        msg.data = stop
        self.pub_stop.publish(msg)
        self.get_logger().info(f'Stop 신호 퍼블리시: {stop}')

    def parking(self):
        msg = Bool()
        msg.data = True
        self.park.publish(msg)

    def starts(self):
        msg = Bool()
        msg.data = True
        self.start.publish(msg)

class CameraWidget(QWidget):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.setGeometry(100, 100, 1300, 750)
        self.setWindowTitle("Camera Viewer")
        self.setStyleSheet("font-size: 20px;")
        self.create_label('신호등 감지', 20, 50, 200, 30)
        self.image_label = self.create_label('',20,80,400,400)
        self.create_label('차선 감지', 400, 50, 200, 30)
        self.lane_image_label = self.create_label('',400,50,400,300)
        self.create_label('YOLO 감지', 850, 50, 200, 30)
        self.yolo_image_label = self.create_label('',850,80,300,250,'background-color: black;')
        self.stop_button = self.create_button("Stop", 20, 350, 80, 30, lambda:self.stop_robot(True))
        self.start_button = self.create_button("Start", 120, 350, 80, 30, lambda:self.stop_robot(False))
        self.up_vel_button = self.create_button("^", 220, 500, 120, 30, lambda:self.vel_robot(True))
        self.down_vel_button = self.create_button("^", 220, 530, 120, 30, lambda:self.vel_robot(False))
        self.vel_label = self.create_label('속도: 0  ',550, 350, 200, 50)
        self.vel_progress = QProgressBar(self)
        self.vel_progress.setGeometry(20, 400, 700, 50)
        self.vel_progress.setMinimum(0)
        self.vel_progress.setMaximum(100)
        self.create_label('표지판 감지', 950, 350, 100, 50)
        self.state_label = self.create_label('',950, 400, 300, 300,'background-color: yellow;')
        self.state_label.setScaledContents(True)
        self.create_label('방향', 800, 350, 100, 50)
        self.ang_dial = QDial(self)
        self.ang_dial.setFixedSize(200,200)
        self.ang_dial.move(730, 400)
        self.ang_dial.setMinimum(-100)
        self.ang_dial.setMaximum(100)
        self.frame_count = 0
        self.yolo_frame = None
        self.main_frame = None
        self.lane_frame = None
        self.update_thread = FrameThread(self)
        self.update_thread.update_main.connect(self.display_main_image)
        self.update_thread.update_yolo.connect(self.display_yolo_image)
        self.update_thread.update_lane.connect(self.display_lane_image)
        self.update_thread.start()
        self.current_state = 0
        self.parking_button = self.create_button("주차", 220, 350 , 80, 30, lambda:self.node.parking())
        self.start_button = self.create_button("시작", 320, 350 , 80, 30, lambda:self.node.starts())
        self.parking_button.setEnabled(False)

    def create_label(self,text,x,y,width,height,style=None):
        label = QLabel(text, self)
        label.setGeometry(x, y, width, height)
        label.setStyleSheet(style)
        return label
    
    def create_button(self, text, x, y, width, height,connect):
        button = QPushButton(text, self)
        button.setGeometry(x, y, width, height)
        button.clicked.connect(connect)
        return button

    def display_main_image(self, frame):
        try:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)
            self.image_label.resize(width, height)
        except Exception as e:
            print(f"Error displaying main image: {e}")

    def display_yolo_image(self, frame):
        try:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            self.yolo_image_label.setPixmap(pixmap)
            self.yolo_image_label.resize(width, height)
        except Exception as e:
            print(f"Error displaying YOLO image: {e}")

    def display_lane_image(self, frame):
        try:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(400, 350, aspectRatioMode=1)
            self.lane_image_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Error displaying lane image: {e}")

    def update_image(self, cv_image):
        self.main_frame = cv_image

    def update_yolo_image(self, cv_image):
        self.yolo_frame = cv_image

    def update_line_image(self, cv_image):
        self.lane_frame  = cv_image

    def update_cmd_vel(self, msg):
        try:
            self.x = msg.linear.x
            self.z = msg.angular.z
            if self.x is not None and self.x > 0:
                vel_progress_value = int(np.clip((self.x / 0.05) * 100, 0, 100))  
                self.vel_progress.setValue(vel_progress_value)
                self.vel_label.setText(f'속도: {self.x:.2f}')
            else:
                self.vel_progress.setValue(0)                   
            if self.z is not None:
                dial_value = int(np.clip(-self.z * 100, -100, 100))  
                self.ang_dial.setValue(dial_value)  
        except Exception as e:
            print(f"Error updating cmd_vel: {e}")
            pass

    def update_state(self,state):
        if state == 1 and self.current_state != 1:
            pixmap = QPixmap("child.png")
            self.state_label.setPixmap(pixmap)
            self.current_state = 1
        elif state == 2 and self.current_state != 2:
            pixmap = QPixmap("crosswalk.png")
            self.state_label.setPixmap(pixmap)
            self.current_state = 2
        elif state == 0 and self.current_state != 0:
            self.current_state = 0
            self.state_label.clear()

    def update_parking_state(self,state):
        if state:
            self.parking_button.setEnabled(True)

    def stop_robot(self,stop):
        if stop:
            self.node.publish_stop(0.0)
        else:
            self.node.publish_stop(0.05)

    def vel_robot(self,state):
        if state == True:
            x = self.x + 0.01
        else:
            x = self.x - 0.01
        self.node.publish_stop(x)

    def closeEvent(self, event):
        self.update_thread.stop()
        super().closeEvent(event)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = CameraNode(None)
    camera_widget = CameraWidget(node)
    node.update_callback = camera_widget.update_image
    node.cmd_vel_callback = camera_widget.update_cmd_vel
    node.line_callback = camera_widget.update_line_image
    node.yolo_callback = camera_widget.update_yolo_image
    node.update_state_callback = camera_widget.update_state
    node.parking_callback = camera_widget.update_parking_state
    spin_thread = RosSpinThread(node)
    spin_thread.start()

    # 종료 시 처리
    def on_close():
        print("Shutting down ROS2...")
        spin_thread.stop()
        spin_thread.wait()
        node.destroy_node()
        rclpy.shutdown()
    
    app.aboutToQuit.connect(on_close)
    
    camera_widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
