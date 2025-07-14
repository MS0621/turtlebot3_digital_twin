#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import time


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        self.create_timer(0.1, self.control_loop)  # 0.1초마다 루프 실행


        # 주차모드 진입 여부 확인 
        self.pub_parking_mode = self.create_publisher(Bool, '/parking_mode', 10)

        # PD control related variables
        self.last_error = 0
        self.MAX_VEL = 0.2

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()

        # 주차 모드 변수들
        self.parking_mode = False
        self.sub_parking = self.create_subscription(Bool, '/detect/parking_sign_detected', self.callback_parking_mode, 1)
        self.entered_parking_zone = False
        self.parking_start_time = None

        # --- 주차 회전 로직용 추가 변수 ---
        self.yellow_line_visible = True  # 노란선 상태
        self.parking_state = "follow_yellow"
        self.rotate_start_time = None
        self.lost_yellow_time = None
        self.backward_start_time = None
        self.backward_state = None  # "wait", "drive", "done"

        # 주차 후 다시 시동할때 변수 
        self.start_triggered = False
        self.start_action_state = None
        self.start_action_start_time = None


        self.create_subscription(
            Bool,
            '/yellow_line_visible',
            self.callback_yellow_visible,
            10
        )

        self.sub_start = self.create_subscription(
            Bool,
            '/start',
            self.callback_start,
            1
        )



    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.

        If avoidance mode is enabled, lane following control is ignored.
        """
        if self.avoid_active:
            return
    
        twist = Twist()

        Kp = 0.0025
        Kd = 0.007

        # 주차 모드 전용 로직
        # if self.parking_mode:
        #     if self.parking_state == "follow_yellow":
        #         if self.yellow_line_visible:
        #             twist.linear.x = 0.08
        #             twist.angular.z = 0.0
        #         else:
        #             twist.linear.x = 0.0
        #             twist.angular.z = 0.0
        #             self.rotate_start_time = time.time()
        #             self.parking_state = "stop_and_turn"
        
        if self.parking_mode:
            if self.parking_state == "follow_yellow":
                if self.yellow_line_visible:
                    center = desired_center.data
                    error = center - 500
                    angular_z = Kp * error + Kd * (error - self.last_error)
                    self.last_error = error

                    twist.linear.x = 0.05   
                    twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
                    self.lost_yellow_time = None  # 다시 노란선 보이면 초기화

                else:
                    # 노란선이 사라졌을 때 타이머 시작
                    if self.lost_yellow_time is None:
                        self.lost_yellow_time = time.time()
                        self.get_logger().info('[주차모드] 노란선 끊김 → 3초간 전진 시작')

                    elapsed = time.time() - self.lost_yellow_time
                    if elapsed < 5.6:
                        twist.linear.x = 0.04
                        twist.angular.z = 0.0
                        self.get_logger().info(f'[주차모드] 노란선 끊김 이후 전진 중... {elapsed:.2f}초 경과')
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.rotate_start_time = time.time()
                        self.parking_state = "stop_and_turn"
                        self.get_logger().info('[주차모드] 3초 경과 → 회전 준비')



            elif self.parking_state == "stop_and_turn":
                elapsed = time.time() - self.rotate_start_time
                if elapsed < 2.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.785
                    self.get_logger().info(f'[주차모드] 회전 중... {elapsed:.2f}초 경과')
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.parking_state = "done"
                    self.get_logger().info('[주차모드] 회전 완료, 정지 상태 진입')



            elif self.parking_state == "done":
                if self.backward_state is None:
                    self.backward_state = "wait"
                    self.backward_start_time = time.time()
                    self.get_logger().info('[주차모드] 정지 → 1초 대기 시작')

                elapsed = time.time() - self.backward_start_time

                if self.backward_state == "wait":
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if elapsed > 1.0:
                        self.backward_state = "drive"
                        self.backward_start_time = time.time()
                        self.get_logger().info('[주차모드] 후진 시작')

                elif self.backward_state == "drive":
                    if elapsed < 3.05:
                        twist.linear.x = -0.05  # 음수는 후진
                        twist.angular.z = 0.0
                        self.get_logger().info(f'[주차모드] 후진 중... {elapsed:.2f}초')
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.backward_state = "done"
                        self.get_logger().info('[주차모드] 후진 완료')
        


            self.pub_cmd_vel.publish(twist)
       
        else:
            if not self.start_triggered:
                center = desired_center.data
                error = center - 500

                Kp = 0.0025
                Kd = 0.007

                angular_z = Kp * error + Kd * (error - self.last_error)
                self.last_error = error

                twist = Twist()
                # Linear velocity: adjust speed based on error (maximum 0.05 limit)
                twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
                twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
                self.pub_cmd_vel.publish(twist)
                self.get_logger().info(f"[주차모드 상태] parking_mode={self.parking_mode}, parking_state={self.parking_state}, yellow_visible={self.yellow_line_visible}")


    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    # 주차 모드 콜백
    # def callback_parking_mode(self, msg):
    #     if msg.data:
    #         self.get_logger().info('Parking sign detected! Entering parking mode.')
    #         self.parking_mode = True
    #         self.entered_parking_zone = False
    def callback_parking_mode(self, msg):
        if msg.data and not self.parking_mode:
            self.get_logger().info('Parking sign detected! Entering parking mode.')
            self.parking_mode = True
            self.parking_state = "follow_yellow"
            self.lost_yellow_time = None
            self.rotate_start_time = None

            parking_msg = Bool()
            parking_msg.data = True
            self.pub_parking_mode.publish(parking_msg)


    # 주차 동작 콜백
    def callback_yellow_visible(self, msg):
        # 회전 시작 이후에는 감지 무시 (이미 상태 전이됨)
        if self.parking_mode and self.parking_state == "follow_yellow":
            self.yellow_line_visible = msg.data

    def callback_start(self, msg):
        if msg.data and not self.start_triggered and self.backward_state == "done":
            self.get_logger().info('[START] 시퀀스 시작됨')
            self.start_triggered = True
            self.start_action_state = 'forward1'
            self.start_action_start_time = time.time()

            self.parking_mode = False
            self.parking_state = None
            self.backward_state = None

    # 차선 검출 없이 start 로직 실행
    def control_loop(self):
        if self.start_triggered:
            twist = Twist()
            elapsed = time.time() - self.start_action_start_time

            if self.start_action_state == 'forward1':
                if elapsed < 4.5:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
                    self.get_logger().info('[START] 전진 중...')
                else:
                    self.start_action_state = 'turn'
                    self.start_action_start_time = time.time()
                    self.get_logger().info('[START] 좌회전 시작')

            elif self.start_action_state == 'turn':
                if elapsed < 2.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.86
                    self.get_logger().info('[START] 회전 중...')
                else:
                    self.start_action_state = 'forward2'
                    self.start_action_start_time = time.time()
                    self.get_logger().info('[START] 최종 전진 시작')

            elif self.start_action_state == 'forward2':
                if elapsed < 13.8:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.001
                    self.get_logger().info('[START] 전진2 중...')
                elif elapsed >= 13.8 and elapsed < 20.0:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
                elif elapsed >= 20.0 and elapsed < 22.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.785
                elif elapsed >= 22.0 and elapsed < 27.0:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
                    self.get_logger().info('[START] 정지 중...')
                else:
                    self.get_logger().info('[START] 완료 → 차선 인식 복귀')
                    self.start_triggered = False
                    self.start_action_state = None
                    self.start_action_start_time = None
                    self.parking_mode = False
                    parking_msg = Bool()
                    parking_msg.data = False
                    self.pub_parking_mode.publish(parking_msg)
                    return  # 종료 시 publish하지 않음

            self.pub_cmd_vel.publish(twist)




    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()