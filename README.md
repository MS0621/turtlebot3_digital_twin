

### Gazebo Turtlebot3 Simulation
#### 터틀봇3 사용하기전 시뮬레이션상에서 다양한 기능 구현

---
### Turtlebot3 홈페이지 : https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
---
## 시뮬레이션에 사용한 Gazebo 환경
<img width="1000" height="700" alt="Image" src="https://github.com/user-attachments/assets/b80a1bb7-9c92-4904-b78f-2ef06506364c" />

## 진행한 코드와 시나리오 (라인 트레이싱을 통해 실행한 시뮬레이션)
1. SIFT로 표지판 인식 및 표지판에 따른 동작 구현
2. YOLO로 표지판 인식 및 표지판에 따른 동작 구현
3. 신호등 불빛에 따른 동작 구현
4. 자동 주차 시스템​
5. 특정 구간 진입시 동작 구현(어린이 보호구역)​
6. 통합 GUI 구현

### 시나리오 세부 설명

#### 1. SIFT로 표지판 인식 및 표지판에 따른 동작 구현
[1.webm](https://github.com/user-attachments/assets/cb30dcd7-e50f-4ee8-9e81-9011e446c42a)

control_lane.py /detect_sign.launch.py /main.py /detect_right_sign.py  내용 확인
#### 2. YOLO로 표지판 인식 및 표지판에 따른 동작 구현
main.py/ train(학습데이터) 내용 확인
#### 3. 신호등 불빛에 따른 동작 구현
detect_traffic_light.py/ detect_traffic_light.launch.py/ main.py 내용 확인
#### 4. 자동 주차 시스템​
control_lane.py 내용 확인
#### 5. 특정 구간 진입시 동작 구현(어린이 보호구역)​
main.py 내용 확인
#### 6. 통합 GUI 구현
main.py 내용 확인

#### 세부적인 내용 ppt 파일 내용 확인

[▶️ PPT 파일 보기](https://github.com/MS0621/turtlebot3_digital_twin/raw/main/B-2_%ED%98%91%EB%8F%993.pptx)


