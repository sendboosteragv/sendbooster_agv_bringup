# 실제 로봇 테스트 가이드

## 1. 패키지 설치

```bash
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-topic-tools \
  ros-humble-teleop-twist-keyboard
```

## 2. 시리얼 장치 식별

4개 USB 장치를 모두 연결한 후:

```bash
# 하나씩 꽂으면서 어떤 포트에 배정되는지 확인
ls /dev/ttyUSB*

# 각 장치의 고유 정보 확인 (udev 규칙용)
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
udevadm info -a /dev/ttyUSB1 | grep -E "idVendor|idProduct|serial"
udevadm info -a /dev/ttyUSB2 | grep -E "idVendor|idProduct|serial"
udevadm info -a /dev/ttyUSB3 | grep -E "idVendor|idProduct|serial"
```

## 3. udev 규칙 설정

확인한 serial 값으로 포트를 고정:

```bash
sudo nano /etc/udev/rules.d/99-sendbooster-agv.rules
```

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="AAAA", SYMLINK+="motor_driver", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="BBBB", SYMLINK+="ahrs_imu", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="CCCC", SYMLINK+="rplidar_front", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", ATTRS{serial}=="DDDD", SYMLINK+="rplidar_back", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/motor_driver /dev/ahrs_imu /dev/rplidar_front /dev/rplidar_back
```

### 동일 제품(동일 USB 속성값)인 경우

LiDAR 2개가 같은 제품이면 `idVendor`, `idProduct`, `serial`이 모두 동일하여 위 방법으로 구분이 안 됩니다.
이 경우 **물리적 USB 포트 경로(KERNELS)**로 구분합니다.

#### USB 포트 경로 확인

전방/후방 LiDAR를 각각 꽂은 상태에서:

```bash
# 전방 LiDAR 포트 경로 확인
udevadm info -a /dev/ttyUSB0 | grep "KERNELS"

# 출력 예시:
#   KERNELS=="1-1.2"        ← 물리적 USB 포트 경로 (이 값 사용)
#   KERNELS=="1-1"
#   KERNELS=="usb1"
```

후방 LiDAR도 동일하게 확인하면 경로가 다릅니다:

```bash
udevadm info -a /dev/ttyUSB1 | grep "KERNELS"

# 출력 예시:
#   KERNELS=="1-1.3"        ← 다른 포트 경로
```

#### udev 규칙 (USB 포트 경로 기반)

```
# Front LiDAR - USB 포트 1-1.2에 꽂을 것
SUBSYSTEM=="tty", KERNELS=="1-1.2", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar_front", MODE="0666"

# Back LiDAR - USB 포트 1-1.3에 꽂을 것
SUBSYSTEM=="tty", KERNELS=="1-1.3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar_back", MODE="0666"
```

> **주의:** 이 방법은 **항상 같은 물리적 USB 포트에 꽂아야** 동작합니다.
> 포트를 바꾸면 규칙이 안 맞으므로, 로봇에 라벨링해서 "전방 LiDAR = 이 포트"로 표시해두세요.
> USB 허브를 사용하는 경우 허브의 각 포트 번호로 구분됩니다.

## 4. launch 파일 포트 수정

udev 규칙 설정 후 bringup.launch.py의 AHRS 포트를 맞춰야 합니다:

```python
# 현재: 'port': '/dev/ttyUSB0'
# 변경: 'port': '/dev/ahrs_imu'
```

## 5. dialout 권한

```bash
sudo usermod -aG dialout $USER
# 반드시 로그아웃 후 재로그인
```

## 6. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 7. 단계별 테스트

**순서가 중요합니다. 반드시 하나씩 확인하고 다음으로 넘어가세요.**

### 7-1. 모터 드라이버

```bash
ros2 run sendbooster_agv_bringup motor_driver_node --ros-args \
  -p serial_port:=/dev/motor_driver -p baudrate:=19200

# 다른 터미널
ros2 topic echo /odom --once          # 데이터 나오는지
ros2 topic hz /odom                    # 50Hz 나오는지
```

- [ ] /odom 토픽 발행 확인
- [ ] 50Hz 주파수 확인

### 7-2. AHRS IMU

```bash
ros2 run stella_ahrs stella_ahrs_node --ros-args \
  -p port:=/dev/ahrs_imu -p baud_rate:=115200

# 다른 터미널
ros2 topic echo /imu/data --once       # 데이터 나오는지
ros2 topic hz /imu/data                # 주파수 확인
```

- [ ] /imu/data 토픽 발행 확인
- [ ] 평평한 바닥에서 시작했는지

### 7-3. LiDAR (전방)

```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/rplidar_front -p serial_baudrate:=115200 -p frame_id:=base_scan

# 다른 터미널
ros2 topic echo /scan --once
ros2 topic hz /scan                    # 약 10Hz
```

- [ ] /scan 토픽 발행 확인
- [ ] 약 10Hz 주파수 확인

### 7-4. LiDAR (후방, 2개일 때)

```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/rplidar_back -p serial_baudrate:=115200 -p frame_id:=base_scan_back \
  --remap scan:=scan2

# 다른 터미널
ros2 topic echo /scan2 --once
```

- [ ] /scan2 토픽 발행 확인

### 7-5. 통합 bringup

개별 노드 모두 확인 후:

```bash
ros2 launch sendbooster_agv_bringup bringup.launch.py

# 확인
ros2 topic list                        # 모든 토픽 존재하는지
ros2 topic hz /odometry/filtered       # EKF 출력 50Hz
ros2 topic hz /scan_merged             # 합성 스캔
ros2 run tf2_tools view_frames         # TF 트리 확인 (frames.pdf)
```

- [ ] /odometry/filtered 50Hz 발행
- [ ] /scan_merged 발행
- [ ] TF 트리 정상 (odom → base_link → imu_link, base_scan, base_scan_back)

### 7-6. 텔레옵 주행

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- [ ] `i` 누르면 **전진**하는지 (반대면 모터 배선 문제)
- [ ] `j` 누르면 **좌회전**하는지
- [ ] 1m 전진 → Rviz 오도메트리도 약 1m 이동
- [ ] 360도 제자리 회전 → Rviz 오도메트리도 한 바퀴

### 7-7. Cartographer SLAM (지도 생성)

```bash
# 터미널 1
ros2 launch sendbooster_agv_bringup bringup.launch.py

# 터미널 2
ros2 launch sendbooster_agv_bringup cartographer.launch.py

# 터미널 3 - 텔레옵으로 천천히 돌아다니며 지도 생성
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 지도 저장
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

- [ ] Rviz에서 지도가 그려지는지
- [ ] 지도 저장 완료 (my_map.pgm + my_map.yaml)

### 7-8. Nav2 네비게이션

```bash
# 터미널 1
ros2 launch sendbooster_agv_bringup bringup.launch.py

# 터미널 2
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  params_file:=$(ros2 pkg prefix sendbooster_agv_bringup)/share/sendbooster_agv_bringup/config/nav2_params.yaml \
  map:=~/maps/my_map.yaml
```

- [ ] Rviz에서 2D Pose Estimate로 초기 위치 설정
- [ ] Nav2 Goal로 목표점 지정 시 자율 주행 동작

---

## 오도메트리 캘리브레이션

주행 테스트에서 오차가 큰 경우 launch 파일에서 조정:

| 파라미터 | 기본값 | 조정 상황 |
|---------|--------|----------|
| `wheel_radius` | 0.0965 | 직진 거리가 실제보다 길거나 짧을 때 (마모 시 줄어듦) |
| `wheel_separation` | 0.37 | 회전 각도가 실제보다 크거나 작을 때 |
| `encoder_resolution` | 6535 | 직진 거리 비례 오차가 일정할 때 |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `Permission denied: /dev/ttyUSB*` | dialout 그룹 미가입 | `sudo usermod -aG dialout $USER` + 재로그인 |
| EKF가 /odometry/filtered 발행 안 함 | /odom 또는 /imu/data 중 하나가 없음 | 두 토픽 모두 발행 중인지 확인 |
| scan_merged가 안 나옴 | /scan 또는 /scan2가 없음 | LiDAR 토픽 확인, 1 LiDAR면 `num_lidars:=1` |
| 전진/회전 방향 반대 | 모터 배선 또는 파라미터 | launch 파일에서 wheel_separation 부호 확인 |
| 오도메트리 드리프트 큼 | wheel_radius/separation 부정확 | 실측 후 launch 파일 파라미터 수정 |
| Cartographer가 맵을 못 만듦 | IMU 또는 LiDAR TF 미스매치 | `ros2 run tf2_tools view_frames`로 TF 확인 |
| rplidar 모터가 안 돌아감 | 전원 부족 또는 포트 오류 | USB 허브 전원 확인, 포트 재연결 |
| Nav2가 경로를 못 찾음 | 초기 위치 미설정 또는 맵 불일치 | Rviz에서 2D Pose Estimate 재설정 |
