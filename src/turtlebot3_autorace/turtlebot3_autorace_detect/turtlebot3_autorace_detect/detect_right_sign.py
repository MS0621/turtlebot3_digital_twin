from enum import Enum
import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
# ##수정됨##: get_package_share_directory 함수를 사용하기 위해 이 줄을 추가합니다.
from ament_index_python.packages import get_package_share_directory 
# ##수정됨##: 로봇 제어를 위해 Twist 메시지 임포트
from geometry_msgs.msg import Twist, Vector3 # Vector3도 Twist에 필요할 수 있으므로 추가
# ##수정됨##: time.sleep 사용을 위해 time 모듈 임포트
import time


class Detect_traffic_right(Node): # 클래스 이름은 유지

    def __init__(self):
        super().__init__('detect_traffic_right')

        self.sub_image_type = 'raw'  # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            # ##수정됨##: 구독 토픽 및 콜백 함수 이름을 'cbFindTrafficRight'로 통일
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed', # 예시: /camera/image_raw/compressed
                self.cbFindTrafficRight, # 콜백 함수 이름 일치
                10
            )
        elif self.sub_image_type == 'raw':
            # ##수정됨##: 구독 토픽 및 콜백 함수 이름을 'cbFindTrafficRight'로 통일
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input', # 예시: /camera/image_raw
                self.cbFindTrafficRight, # 콜백 함수 이름 일치
                10
            )

        # ##수정됨##: 발행 토픽 이름 및 Enum 정의 변경
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_right', 10) # 감지된 오른쪽 표지판 정보
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/traffic_right/compressed', 10 # 처리된 이미지 출력 토픽
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output/traffic_right', 10 # 처리된 이미지 출력 토픽
            )

        self.cvBridge = CvBridge()
        # ##수정됨##: TrafficSign Enum에 'right'만 정의
        self.TrafficSign = Enum('TrafficSign', 'right')
        self.counter = 1

        self.fnPreproc()

        # ##수정됨##: 로봇 움직임 제어를 위한 /cmd_vel 퍼블리셔 추가
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10) 
        
        self.get_logger().info('DetectTrafficRight Node Initialized') # 초기화 메시지 변경

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        # ##수정됨##: 이미지 파일 경로를 ROS 2의 share 디렉토리에서 찾도록 수정
        # 'turtlebot3_autorace_detect' 패키지의 share 디렉토리 경로를 가져옵니다.
        package_share_directory = get_package_share_directory('turtlebot3_autorace_detect')
        # 이미지 파일이 share 디렉토리의 'image' 폴더 안에 있다고 가정합니다.
        image_dir_path = os.path.join(package_share_directory, 'image') # ##수정됨##: 변수 이름 변경 (dir_path와의 혼동 방지)

        self.img_traffic_right = cv2.imread(os.path.join(image_dir_path, 'right.png'), 0) # ##수정됨##: 경로 수정

        # ##수정됨##: 이미지 로드 실패 시 오류 처리 (기존과 동일)
        if self.img_traffic_right is None:
            self.get_logger().error(f"Error: right.png not found at {os.path.join(image_dir_path, 'right.png')}. Please ensure the file exists in the package's 'image' directory.")
            rclpy.shutdown()
            return

        self.kp_traffic_right, self.des_traffic_right = self.sift.detectAndCompute(self.img_traffic_right, None)

        FLANN_INDEX_KDTREE = 0
        index_params = {
            'algorithm': FLANN_INDEX_KDTREE,
            'trees': 5
        }

        search_params = {
            'checks': 50
        }

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        # ##수정됨##: arr1과 arr2의 형태가 다를 때 MSE 계산 불가 처리
        if arr1.shape != arr2.shape:
            self.get_logger().warn("fnCalcMSE: Input arrays have different shapes. Returning large error.")
            return float('inf') # 무한대 값 반환하여 매칭 실패로 간주
        
        num_all = arr1.shape[0] * arr1.shape[1]  # cv_image_input and 2 should have same shape
        err = total_sum / num_all
        return err

    # ##수정됨##: 콜백 함수 이름을 'cbFindTrafficRight'로 변경하고 로직 수정
    def cbFindTrafficRight(self, image_msg): # 콜백 함수 이름 일치
        # drop the frame to 1/5 (6fps) because of the processing speed.
        # This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            # converting compressed image to opencv image
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # ##수정됨##: 이미지 처리 전에 입력 이미지가 유효한지 확인
        if cv_image_input is None or cv_image_input.size == 0:
            self.get_logger().warn("Received empty or invalid image, skipping processing.")
            return

        MIN_MATCH_COUNT = 7
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)

        # ##수정됨##: 디스크립터가 없거나 특징점이 부족하면 매칭 불가능
        if des1 is None or len(kp1) < MIN_MATCH_COUNT:
            self.publish_processed_image(cv_image_input) # 원본 이미지 발행
            # ##수정됨##: 표지판 미감지 시 기본 동작 수행 (이전에 `perform_default_action`을 제거했으므로, 원하는 기본 동작 로직을 여기에 직접 넣을 수 있습니다.)
            self.perform_action("NONE") # "NONE"은 감지된 표지판이 없음을 나타내는 가상의 이름
            return
        
        # ##수정됨##: 오른쪽 표지판 매칭 시도
        found_traffic_right = False
        matches_traffic_right = self.flann.knnMatch(des1, self.des_traffic_right, k=2)
        # ##수정됨##: des_traffic_right이 비어있지 않고 매칭 결과가 있을 때만 처리
        if self.des_traffic_right is not None and len(self.des_traffic_right) > 0 and matches_traffic_right:
            good_traffic_right = []
            for m, n in matches_traffic_right: # ##수정됨##: 'matches_parking' 오타 수정
                if m.distance < 0.7*n.distance:
                    good_traffic_right.append(m)

            if len(good_traffic_right) > MIN_MATCH_COUNT:
                src_pts_traffic_right = np.float32([kp1[m.queryIdx].pt for m in good_traffic_right]).reshape(-1, 1, 2)
                dst_pts_traffic_right = np.float32([
                    self.kp_traffic_right[m.trainIdx].pt for m in good_traffic_right
                ]).reshape(-1, 1, 2)

                M_traffic_right, mask_traffic_right = cv2.findHomography(
                    src_pts_traffic_right, dst_pts_traffic_right, cv2.RANSAC, 5.0
                )
                # ##수정됨##: findHomography가 None을 반환할 수 있으므로 체크
                if M_traffic_right is not None:
                    # ##수정됨##: matchesMask_traffic_light 오타 수정
                    matchesMask_traffic_right = mask_traffic_right.ravel().tolist() 

                    mse_traffic_right = self.fnCalcMSE(src_pts_traffic_right, dst_pts_traffic_right)
                    if mse_traffic_right < MIN_MSE_DECISION:
                        msg_sign = UInt8()
                        msg_sign.data = self.TrafficSign.right.value 
                        self.pub_traffic_sign.publish(msg_sign)
                        self.get_logger().info('Detected: traffic_right') 

                        final_traffic_right = cv2.drawMatches(
                            cv_image_input, kp1, self.img_traffic_right, self.kp_traffic_right,
                            good_traffic_right, None, matchColor=(0, 255, 0), singlePointColor=None, 
                            matchesMask=matchesMask_traffic_right, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                        )
                        self.publish_processed_image(final_traffic_right)
                        found_traffic_right = True

                        # ##수정됨##: 'right' 표지판 감지 시 특정 동작 수행
                        self.perform_action("RIGHT_SIGN_DETECTED") # 특정 액션 이름 전달
                else:
                    self.get_logger().warn("findHomography for traffic_right returned None (not enough inliers).")

        # ##수정됨##: 아무것도 감지되지 않았을 경우 원본 이미지 발행 및 기본 동작 수행
        if not found_traffic_right:
            self.publish_processed_image(cv_image_input)
            self.perform_action("NONE") # "NONE"은 감지된 표지판이 없음을 나타내는 가상의 이름
            
    def publish_processed_image(self, cv_image):
        """Helper function to publish processed images."""
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(
                    cv_image, 'jpg'
                )
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(
                    cv_image, 'bgr8'
                )
            )

    def publish_cmd_vel(self, linear_x, angular_z):
        """
        /cmd_vel 토픽에 Twist 메시지를 발행하여 로봇의 선속도와 각속도를 제어합니다.
        """
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist_msg)
        # self.get_logger().info(f'Publishing /cmd_vel: Linear X={linear_x}, Angular Z={angular_z}')

    def perform_action(self, action_type):
        """
        감지된 표지판 종류 또는 상태에 따라 특정 로봇 동작을 수행합니다.
        """
        self.get_logger().info(f"DEBUG: perform_action called with action_type: '{action_type}'")

        if action_type == "RIGHT_SIGN_DETECTED":
            self.get_logger().info("Executing RIGHT TURN action based on sign detection!")
            self.publish_cmd_vel(0.1, -0.5) # 전진하며 오른쪽으로 회전 (음수 각속도)
            # 특정 시간 동안 회전 후 멈추거나 직진으로 돌아가려면 time.sleep 또는 rclpy.timer 사용
            # time.sleep(2.0) # 2초간 회전 (이 함수는 노드를 블로킹하니 주의)
            # self.publish_cmd_vel(0.1, 0.0) # 회전 후 다시 직진
        elif action_type == "NONE":
            self.get_logger().info("No specific sign detected. Robot will maintain its current state.")
            # self.publish_cmd_vel(0.1, 0.0) # 이 줄은 제거되었습니다.
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}. Robot will maintain current state.")


# ##수정됨##: 메인 함수 정의
def main(args=None):
    rclpy.init(args=args)
    node = Detect_traffic_right() # 노드 객체 생성 시 클래스 이름 일치
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()