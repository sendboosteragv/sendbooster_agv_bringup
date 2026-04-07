# Sendbooster AGV Bringup

ROS2 Humble 기반 Sendbooster AGV 실제 로봇 통합 런치 패키지 (Jetson Orin Nano)

## 개요

이 패키지는 Sendbooster AGV의 모든 하드웨어를 한 번에 실행하는 통합 bringup 패키지입니다.
모터 드라이버(MDROBOT MD400T), AHRS IMU, EKF 퓨전, 듀얼 LiDAR(전방 RPLIDAR A2M8, 후방 A1M8), Cartographer SLAM, Nav2 네비게이션을 지원합니다.

## 시스템 아키텍처

```
/cmd_vel ──→ [motor_driver_node] ──→ /odom (엔코더 기반)
                                        │
                                        ├──→ [EKF] ──→ /odometry/filtered + TF(odom→base_footprint)
                                        │       ↑
[stella_ahrs_node] ──→ /imu/data ───────┘  angular_velocity_z 보정

[rplidar_front] → /scan_raw_front ─┐
                                    ├─→ [scan_processor] ──→ /scan_merged → [AMCL + Nav2]
[rplidar_back]  → /scan_raw_back  ─┘    (C++ 통합 노드)
```

### 핵심 포인트
- **EKF (robot_localization)**: 휠 엔코더 오도메트리(pose x,y,yaw) + IMU angular_velocity_z 퓨전
- **scan_processor**: Python 필터 2개 + C++ merger를 하나의 C++ 노드로 통합 (RAM ~100MB 절감)
- **Cyclone DDS localhost**: 모든 노드가 같은 보드에서 실행 → 멀티캐스트 비활성화
- **SLAM 시**: 전면 LiDAR만 구독 (`/scan`) — 뒤에서 조종하는 사람이 맵에 안 잡힘
- **네비게이션 시**: 머지 LiDAR 구독 (`/scan_merged`) — 360도 장애물 감지

## TF 트리

```
map → odom (AMCL / Cartographer)
  odom → base_footprint (EKF: robot_localization)
    base_footprint → base_link (URDF fixed joint)
      base_link → imu_link
      base_link → base_scan (front LiDAR)
      base_link → base_scan_back (back LiDAR)
      base_link → wheel_left_link
      base_link → wheel_right_link
```

## 의존 레포지토리

| 레포지토리 | 설명 | 링크 |
|-----------|------|------|
| **sendbooster_agv_bringup** | 실제 로봇 통합 런치 (이 패키지) | [GitHub](https://github.com/SeongminJaden/sendbooster_agv_bringup) |
| **2th_NtrexAHRS_lib_ROS_Sendbooster** | AHRS IMU 드라이버 (stella_ahrs) | [GitHub](https://github.com/SeongminJaden/2th_NtrexAHRS_lib_ROS_Sendbooster) |
| **sendbooster_agv_simulation** | 시뮬레이션 + URDF 모델 | [GitHub](https://github.com/SeongminJaden/sendbooster_agv_simulation) |
| **serial-ros2** | ROS2 시리얼 통신 라이브러리 | [GitHub](https://github.com/RoverRobotics/serial-ros2) |

## 패키지 구조

```
sendbooster_agv_bringup/
├── config/
│   ├── ekf.yaml                    # EKF 설정 (robot_localization)
│   ├── nav2_params.yaml            # Nav2 네비게이션 파라미터
│   ├── cyclonedds.xml              # DDS localhost 설정 (Jetson 최적화)
│   ├── motor_params.yaml           # 모터 드라이버 파라미터
│   ├── cartographer_base.lua       # Cartographer 공통 설정
│   ├── cartographer_1lidar.lua     # 1 LiDAR용
│   └── cartographer_2lidar.lua     # 2 LiDAR용
├── launch/
│   ├── bringup.launch.py           # 통합 런치 (모터+IMU+EKF+LiDAR+ScanProcessor)
│   ├── navigation2.launch.py       # Nav2 네비게이션
│   ├── cartographer.launch.py      # Cartographer SLAM (Jetson)
│   └── rviz_view.launch.py         # RViz 뷰어 (PC)
├── src/
│   ├── motor_driver.cpp            # RS485 시리얼 통신
│   ├── motor_driver_node.cpp       # 모터 드라이버 ROS2 노드
│   ├── scan_processor.cpp          # LiDAR 필터 + 머지 통합 C++ 노드
│   └── laser_scan_merger.cpp       # (legacy) 듀얼 LiDAR 스캔 합성
├── scripts/
│   ├── scan_angle_filter.py        # (legacy) LiDAR 각도 필터
│   ├── odom_imu_fuser_real.py      # (legacy) Odom+IMU 퓨전 노드
│   └── odom_path_publisher.py      # Raw/Filtered 경로 비교 시각화
├── urdf/
│   └── sendbooster_agv.urdf        # 로봇 URDF
└── rviz/
    └── slam_view.rviz              # SLAM용 RViz 설정
```

## 새 컴퓨터 설치 가이드

### 1. 시스템 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-foxglove-bridge \
  ros-humble-teleop-twist-keyboard

# PC에서 모니터링용 (선택)
sudo snap install foxglove-studio       # Foxglove 데스크탑 앱
sudo apt install ros-humble-rviz2       # rviz2
```

### 2. 워크스페이스 구성

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/RoverRobotics/serial-ros2.git
git clone https://github.com/SeongminJaden/sendbooster_agv_bringup.git
git clone -b ver_2.0 https://github.com/SeongminJaden/2th_NtrexAHRS_lib_ROS_Sendbooster.git
```

### 3. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sendbooster_agv_bringup stella_ahrs
source install/setup.bash
```

### 4. 환경 설정

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 사전 점검

### 시리얼 권한

```bash
sudo usermod -aG dialout $USER
# 재로그인 필요
```

### udev 규칙 (포트 고정)

```bash
sudo nano /etc/udev/rules.d/99-sendbooster-agv.rules
```

```
# Motor Driver - serial 값은 본인 장치에 맞게 변경
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_SERIAL", SYMLINK+="motor_driver", MODE="0666"

# AHRS IMU
SUBSYSTEM=="tty", ATTRS{idVendor}=="YOUR_VID", ATTRS{idProduct}=="YOUR_PID", SYMLINK+="imu", MODE="0666"

# Front LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_SERIAL", SYMLINK+="rplidar_front", MODE="0666"

# Back LiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="YOUR_SERIAL", SYMLINK+="rplidar_back", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/motor_driver /dev/imu /dev/rplidar_front /dev/rplidar_back
```

## 실행 방법

### 센서만 실행 (동작 확인용)

```bash
# Jetson에서
source ~/ros2_ws/install/setup.bash
ros2 launch sendbooster_agv_bringup bringup.launch.py
```

→ 모터, IMU, LiDAR×2, scan_processor, EKF, Foxglove bridge 실행

### SLAM (지도 생성)

```bash
# 터미널 1: 브링업
ros2 launch sendbooster_agv_bringup bringup.launch.py

# 터미널 2: Cartographer SLAM
ros2 launch sendbooster_agv_bringup cartographer.launch.py

# 터미널 3: 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

지도 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/sendbooster_agv_bringup/map/my_map
```

### Nav2 네비게이션 (자율 주행)

```bash
# 한 줄로 실행 (센서 + Nav2 + AMCL + Foxglove 전부)
ros2 launch sendbooster_agv_bringup bringup.launch.py \
  nav:=true localization:=amcl \
  map:=/home/sendbooster_agv/ros2_ws/src/sendbooster_agv_bringup/map/map.yaml
```

### 원격 모니터링 (PC에서)

**Foxglove (추천 — Jetson 부하 없음):**
```bash
# PC에서 Foxglove 데스크탑 앱 실행
foxglove-studio
# Open connection → ws://[Jetson IP]:8765
```

또는 브라우저: `http://app.foxglove.dev` → `ws://[Jetson IP]:8765`

**rviz2 (DDS 직접 연결 — 같은 네트워크 필요, Jetson에 부하 발생):**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> ⚠️ rviz2는 DDS로 LiDAR 데이터를 가져오므로 Jetson에 네트워크 부하 발생. Foxglove 사용 권장.

### Jetson IP 확인

```bash
ip addr show wlP1p1s0 | grep inet    # WiFi 핫스팟
ip addr show l4tbr0 | grep inet       # USB 연결 (항상 192.168.55.1)
```

### Goal 전송

```bash
# CLI
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}" --once

# rviz2: 상단 "2D Goal Pose" 버튼 → 맵 클릭+드래그
# Foxglove: Publish 패널 → /goal_pose에 JSON 입력 → Publish
```

## DDS 설정

Jetson은 **DDS localhost 전용**으로 설정되어 있어 외부로 토픽 데이터가 나가지 않습니다.
WiFi 핫스팟으로 rviz2 연결 시 발생하던 **네트워크 폭풍(CPU 과부하) 문제를 방지**합니다.

- 외부 모니터링은 **Foxglove bridge (WebSocket, port 8765)** 가 담당
- IP가 바뀌어도 **DDS 설정 변경 불필요** — Foxglove 접속 IP만 변경
- rviz2 사용 시에만 DDS를 임시로 네트워크 모드로 변경 필요:
  ```bash
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
  ```

## 시뮬레이션

```bash
# PC에서 all-in-one (Gazebo + Nav2)
ros2 launch sendbooster_agv_bringup simulation.launch.py localization:=amcl

# 분산 실행 (PC: Gazebo, Jetson: Nav2) — 같은 네트워크, 같은 RMW 필요
# PC:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=30
ros2 launch sendbooster_agv_bringup simulation.launch.py gazebo:=true nav:=false

# Jetson:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=30
ros2 launch sendbooster_agv_bringup simulation.launch.py gazebo:=false nav:=true localization:=amcl
```

---

## Nav2 네비게이션 파라미터 튜닝 가이드

### RViz에서 반드시 표시할 토픽

| Display | 토픽 | 용도 |
|---------|------|------|
| LaserScan | `/scan_merged` | 360도 스캔 데이터 확인 |
| Map - Costmap | `/local_costmap/costmap` | 장애물 감지 확인 |
| Map - Costmap | `/global_costmap/costmap` | 글로벌 장애물 확인 |
| Path | `/plan` | 글로벌 경로 |
| Path | `/local_plan` | DWB 선택 궤적 |
| PoseArray | `/particle_cloud` | AMCL 파티클 (위치 수렴 확인) |

### 튜닝 순서

```
1단계: 위치 확인
  └─ RViz에서 /particle_cloud 확인 → 파티클이 로봇 주변에 모이는지

2단계: 장애물 감지
  └─ RViz에서 local/global costmap 확인 → 장애물이 제대로 표시되는지

3단계: 로컬 플래너 동작
  └─ 저속으로 시작 (max_vel_x: 0.2) → 정상 동작 확인 후 점진적 증가

4단계: DWB Critics 비율 조정
  └─ 장애물 회피 vs 경로 추종 비율 미세 조정

5단계: 속도 올리기
  └─ max_vel_x 0.2 → 0.3 → 0.5 점진적으로
```

---

### 1. AMCL (위치 추정) — `nav2_params.yaml > amcl`

로봇이 지도에서 어디 있는지 추정합니다.

| 파라미터 | 현재값 | 영향 | 튜닝 방법 |
|---|---|---|---|
| `alpha1`~`alpha4` | 0.1 | 오도메트리 노이즈 모델. 클수록 "odom 못 믿겠다" → 파티클 넓게 퍼짐 | 파티클이 너무 퍼지면 줄이고, 수렴 안 하면 높임 |
| `max_particles` | 3000 | 많을수록 정확하지만 CPU 증가 | 파티클이 잘 수렴하면 줄여도 됨 |
| `laser_likelihood_max_dist` | 5.0m | 스캔 매칭 시 장애물 탐색 거리 | 좁은 공간: 2.0, 넓은 공간: 5.0 |
| `sigma_hit` | 0.1 | 스캔 매칭 허용 오차. 작을수록 엄격 | 맵 정확: 0.1~0.15, 맵 부정확: 0.3 |
| `update_min_d` | 0.05m | 이만큼 이동해야 AMCL 업데이트 | 작을수록 자주 업데이트 (정확도 증가, CPU 증가) |
| `update_min_a` | 0.05rad | 이만큼 회전해야 AMCL 업데이트 | 회전 시 위치 잘 못잡으면 줄임 |

**확인법**: `/particle_cloud` 파티클이 로봇 주변에 모여있으면 OK, 넓게 퍼지면 alpha를 줄이거나 sigma_hit을 줄임

---

### 2. EKF (오도메트리 퓨전) — `ekf.yaml`

odom 프레임의 정확도를 결정합니다.

| 파라미터 | 현재값 | 영향 | 튜닝 방법 |
|---|---|---|---|
| `process_noise x,y` | 0.05 | 위치 예측 노이즈. 클수록 측정값(엔코더) 더 신뢰 | odom이 부드러우면 줄이고, 급변하면 높임 |
| `process_noise yaw` | 0.05 | 방향 예측 노이즈 | 회전 시 떨림 있으면 줄임 |
| `process_noise vx` | 0.1 | 속도 예측 노이즈 | 가감속 응답 느리면 높임 |
| `process_noise vyaw` | 0.1 | 회전속도 예측 노이즈 | IMU와 odom 불일치 시 조정 |
| `odom0_pose_rejection_threshold` | 5.0 | 이상치 거부 임계값. 작을수록 엄격 | 순간이동 남아있으면 3.0으로 줄임 |

**확인법**: `ros2 topic echo /odometry/filtered` 로봇 정지 시 값이 안정적이어야 함

---

### 3. DWB 로컬 플래너 — `nav2_params.yaml > controller_server > FollowPath`

실시간 장애물 회피 + 경로 추종을 담당합니다.

#### 속도/가속도 (가장 직접적)

| 파라미터 | 현재값 | 영향 |
|---|---|---|
| `max_vel_x` | 0.5 m/s | 최대 직진 속도 |
| `max_vel_theta` | 0.5 rad/s | 최대 회전 속도 |
| `acc_lim_x` | 0.5 m/s2 | 직진 가속 한계. 높으면 빠르게 출발, 낮으면 부드럽게 |
| `acc_lim_theta` | 0.5 rad/s2 | 회전 가속 한계 |
| `sim_time` | 1.7s | 궤적 시뮬레이션 시간. 길수록 먼 미래 고려 (장애물 회피 향상, CPU 증가) |
| `vx_samples` | 15 | 직진 속도 샘플 수 |
| `vtheta_samples` | 30 | 회전 속도 샘플 수 |

#### Critics (행동 우선순위 — 가장 중요한 튜닝 포인트)

각 Critic의 scale 비율이 로봇의 주행 행동을 결정합니다.

| Critic | 현재 scale | 역할 | 높이면 | 낮추면 |
|---|---|---|---|---|
| `BaseObstacle` | **50.0** | 장애물 회피 | 장애물에서 더 멀리 우회 | 장애물 가까이 지나감 |
| `PathDist` | 32.0 | 글로벌 경로 추종 | 경로에 딱 붙어 이동 | 경로에서 벗어나도 허용 |
| `GoalDist` | 24.0 | 목표점 향해 이동 | 목표에 직진 | 경로 추종 우선 |
| `PathAlign` | 32.0 | 경로 방향 정렬 | 경로 방향으로 정렬 | 방향 자유 |
| `GoalAlign` | 24.0 | 목표 방향 정렬 | 도착 시 목표 방향 맞춤 | 방향 무관 |
| `RotateToGoal` | 32.0 | 도착 시 제자리 회전 | 정확한 최종 방향 | 대략적 방향 |

**튜닝 핵심** — `BaseObstacle`와 `PathDist`의 비율:
- 장애물에 부딪힘 → `BaseObstacle` 올림 (50 → 60~80)
- 경로에서 너무 벗어남 → `PathDist` 올림
- 좁은 통로 못 지나감 → `BaseObstacle` 내림

---

### 4. 코스트맵 — `nav2_params.yaml > local_costmap / global_costmap`

어디가 갈 수 있는 곳이고 어디가 위험한 곳인지 결정합니다.

| 파라미터 | 현재값 | 영향 | 튜닝 |
|---|---|---|---|
| `inflation_radius` | 0.50m | 장애물 주변 "위험 영역" 크기 | footprint 외곽 + 여유. 장애물에 너무 가까이 가면 올림 |
| `cost_scaling_factor` | 5.0 | 비용 감소 기울기. 클수록 inflation 안에서 비용이 빠르게 줄어듦 | 장애물 가까이 감: 줄임 (1.5~2.0). 좁은 곳 못감: 높임 (3.0~5.0) |
| `footprint` | `[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]` | 로봇 실제 크기 (0.6x0.6m) | 실제 크기보다 크면 좁은 곳 못감, 작으면 충돌 |
| `resolution` | local 0.05m, global 0.1m | 셀 크기 | 작을수록 정밀 (CPU 증가) |
| `update_frequency` | local 5Hz, global 0.5Hz | 코스트맵 갱신 빈도 | 동적 장애물 반응 느리면 올림 |
| `obstacle_max_range` | 1.5m | 장애물 감지 최대 거리 | 먼 장애물 놓치면 올림 |

**확인법**: RViz에서 costmap 표시 → 장애물 주변에 inflation 영역이 보이는지, 로봇 앞 장애물이 즉시 반영되는지

---

### 5. 글로벌 플래너 — `nav2_params.yaml > planner_server`

| 파라미터 | 현재값 | 영향 |
|---|---|---|
| `use_astar` | true | A* 알고리즘 (Dijkstra보다 빠름) |
| `tolerance` | 0.5m | 목표 근처 이 거리 안에 도달하면 성공 |

---

### 6. Velocity Smoother — `nav2_params.yaml > velocity_smoother`

| 파라미터 | 현재값 | 영향 |
|---|---|---|
| `max_velocity` | [0.5, 0.0, 0.5] | [vx, vy, vtheta] 최종 속도 제한 |
| `max_accel` | [0.5, 0.0, 0.5] | 가속 제한. 낮을수록 부드럽게 출발 |
| `max_decel` | [-0.5, 0.0, -0.5] | 감속 제한. 절대값 작을수록 부드럽게 정지 |

---

### 증상별 빠른 진단

| 증상 | 확인할 곳 | 조치 |
|---|---|---|
| 위치를 못잡음 (파티클 퍼짐) | AMCL alpha1~4, sigma_hit | alpha 줄이기, sigma_hit 줄이기 |
| 순간이동 | EKF rejection threshold, AMCL transform_tolerance | rejection 줄이기, transform_tolerance 0.5 유지 |
| 장애물 충돌 | costmap에 장애물 표시 여부 | obstacle_max_range 확인, BaseObstacle.scale 올리기 |
| 좁은 곳 못 지나감 | inflation_radius, footprint | inflation_radius 줄이기 (>0.3m), cost_scaling_factor 올리기 |
| 경로에서 벗어남 | DWB PathDist.scale | PathDist.scale 올리기 |
| 장애물 앞에서 멈춤 | DWB sim_time, vx_samples | sim_time 줄이기, samples 늘리기 |
| 회전이 느림/불안정 | max_vel_theta, acc_lim_theta | 값 조정 |
| 목표 도착 못함 | goal_checker xy_goal_tolerance | tolerance 늘리기 |

---

## 오도메트리 캘리브레이션

| 파라미터 | 기본값 | 설명 | 조정 방법 |
|---------|--------|------|----------|
| `wheel_radius` | 0.0965 | 바퀴 반지름 (m) | 직진 거리 오차 시 조정 |
| `wheel_separation` | 0.37 | 좌우 바퀴 간 거리 (m) | 회전 각도 오차 시 조정 |
| `encoder_resolution` | 6535 | 엔코더 해상도 (PPR) | 직진 거리 오차 시 조정 |

## IMU 캘리브레이션

```bash
# 캘리브레이션 모드 진입
ros2 service call /imu/calibration std_srvs/srv/Trigger

# 로봇째로 천천히 제자리에서 360도 이상 회전

# 캘리브레이션 완료
ros2 service call /imu/calibration std_srvs/srv/Trigger

# Euler 리셋 (현재 자세를 0도로 설정)
ros2 service call /imu/euler_reset std_srvs/srv/Trigger
```

## 로봇 스펙

| 항목 | 값 |
|------|-----|
| 플랫폼 | Jetson Orin Nano (6코어 A78AE, 8GB RAM) |
| 구동 방식 | 차동 구동 (Differential Drive) |
| 모터 드라이버 | MDROBOT MD400T |
| 기어비 | 1:10 |
| 바퀴 지름 | 193mm |
| 바퀴 간격 | 370mm |
| 로봇 크기 | 600mm x 600mm |
| LiDAR | RPLIDAR A2M8 (전방) + A1M8 (후방) |
| IMU | NTRexLAB MW-AHRSv1 |
| 통신 | RS485 19200bps (모터), UART 115200bps (IMU, LiDAR) |

## ROS2 토픽

### Published Topics

| Topic | Type | 발행 노드 | 설명 |
|-------|------|----------|------|
| `/odom` | Odometry | motor_driver_node | 엔코더 기반 휠 오도메트리 |
| `/imu/data` | Imu | stella_ahrs_node | IMU 데이터 |
| `/odometry/filtered` | Odometry | ekf_filter_node | EKF 퓨전 오도메트리 |
| `/scan_merged` | LaserScan | scan_processor | 360도 합성 스캔 |
| `/joint_states` | JointState | motor_driver_node | 바퀴 조인트 상태 |
| `/diagnostics` | DiagnosticArray | motor_driver_node | 모터 진단 정보 |

### Subscribed Topics

| Topic | Type | 구독 노드 | 설명 |
|-------|------|----------|------|
| `/cmd_vel` | Twist | motor_driver_node | 속도 명령 |

### Services

| Service | Type | 노드 | 설명 |
|---------|------|------|------|
| `~/emergency_stop` | Trigger | motor_driver_node | 긴급 정지 |
| `~/reset_position` | Trigger | motor_driver_node | 오도메트리 리셋 |
| `~/reset_alarm` | Trigger | motor_driver_node | 모터 알람 리셋 |
| `~/set_torque` | SetBool | motor_driver_node | 토크 ON/OFF |
| `/imu/calibration` | Trigger | stella_ahrs_node | 자기장 캘리브레이션 |
| `/imu/euler_reset` | Trigger | stella_ahrs_node | Euler 각도 리셋 |

## 센서 통신 구조 및 주파수

### 데이터 흐름

```
[Motor Driver HW] ──RS485 19200baud──→ [motor_driver_node]
  단일 타이머 10Hz: read → publish(/odom) → send

[MW-AHRSv1 IMU] ──UART 115200baud──→ [stella_ahrs_node]
  시리얼 읽기: 200Hz (내부), 발행: 50Hz → /imu/data
  EKF에 angular_velocity_z만 퓨전 (gyro z)

[RPLIDAR Front] ──UART 115200baud──→ [rplidar_front]
  Sensitivity 모드: ~7-8Hz → /scan_raw_front ─┐
[RPLIDAR Back]  ──UART 115200baud──→ [rplidar_back]   ├→ [scan_processor] → /scan_merged
  Express 모드: ~10-15Hz → /scan_raw_back    ─┘

/odom (10Hz) + /imu/data (50Hz) → [EKF 10Hz] → /odometry/filtered + TF(odom→base_footprint)

/scan_merged (~8Hz) → [AMCL 3000 particles] → TF(map→odom)
/scan_merged (~8Hz) → [costmap] → [RPP 10Hz] → /cmd_vel → [motor_driver_node]

[foxglove_bridge] → ws://0.0.0.0:8765 → PC 브라우저/앱에서 모니터링
```

### 주파수 적정성

| 센서 | 발행 Hz | 소비 노드 | 소비 Hz | queue | 상태 |
|------|---------|----------|---------|-------|------|
| Motor → /odom | 10 | EKF | 10 | 10 | OK (19200 baud 안정 한계) |
| IMU → /imu/data | 50 | EKF | 10 | 10 | OK (angular_vel_z만 사용) |
| LiDAR → /scan_merged | 7-10 | local_costmap | 5 | - | OK |
| LiDAR → /scan_merged | 7-10 | AMCL | ~5 | - | OK |
| EKF → TF(odom→base) | 10 | Nav2 전체 | - | - | OK (transform_tolerance 0.5s) |

### IMU 데이터 사용 근거

MW-AHRSv1은 가속도, 자이로, 지자기, orientation 등 다양한 데이터를 제공하지만, **EKF에는 angular_velocity_z (gyro z) 하나만 퓨전**합니다.

| IMU 데이터 | 사용 여부 | 이유 |
|-----------|----------|------|
| **angular_velocity_z** | **사용** | 엔코더의 yaw drift를 직접 보정. 차동구동 로봇에서 가장 효과적 |
| orientation yaw | 미사용 | 자북 기준(-177° 등) → odom 프레임(0°)과 충돌. 실내 모터/금속 자기장 간섭 심각 |
| orientation yaw (differential) | 미사용 | yaw 변화량 = gyro z 적분과 동일, 중복 정보 |
| linear_acceleration_x | 미사용 | 0.5m/s 저속 AGV에서 가속도 값이 작고 노이즈 큼. 적분 시 drift 심각. 엔코더가 훨씬 정확 |
| angular_velocity_x,y | 미사용 | 평지 주행 + 2D 모드 → roll/pitch 회전 없음 |
| orientation roll,pitch | 미사용 | `two_d_mode: true` → EKF가 무시 |

**EKF 퓨전 전략**:
```
엔코더: x, y, yaw (위치 정확, 방향은 시간 지나면 drift)
IMU:    angular_velocity_z (방향 변화율 정확, 위치 정보 없음)
─────────────────────────────────────────────────────────
EKF:    엔코더의 위치 + IMU의 방향 보정 → 상호 보완
```

추가 IMU 데이터를 넣으면 노이즈만 증가하고, 실내 자기장 간섭으로 EKF 발산 위험이 있습니다.

---

## 테스트 절차 (순차적)

브링업 후 네비게이션까지 순차적으로 테스트합니다. **각 단계가 통과해야 다음 단계로 진행하세요.**

### 1단계: 시리얼 포트 확인

```bash
# udev 심볼릭 링크 확인
ls -la /dev/motor_driver /dev/imu /dev/rplidar_front /dev/rplidar_back

# 권한 확인 (dialout 그룹)
groups $USER | grep dialout
```

통과 기준: 4개 포트 모두 존재, dialout 그룹 포함

---

### 2단계: 모터 드라이버 단독 테스트

```bash
# 모터 드라이버 노드만 실행
ros2 run sendbooster_agv_bringup motor_driver_node --ros-args \
  -p serial_port:=/dev/motor_driver -p baudrate:=19200 \
  -p control_rate:=10.0 \
  -p use_encoder_odom:=true -p publish_tf:=true
```

```bash
# 다른 터미널에서 확인
ros2 topic hz /odom                    # 목표: ~10Hz
ros2 topic echo /odom --once           # pose, twist 값 확인
ros2 topic echo /diagnostics --once    # 에러 없는지 확인

# 토크 ON/OFF 테스트
ros2 service call /motor_driver_node/set_torque std_srvs/srv/SetBool "{data: true}"

# 저속 cmd_vel 테스트 (0.1 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# 엔코더 변화 확인
ros2 topic echo /odom --field pose.pose.position.x

# 정지
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

통과 기준:
- `/odom` ~50Hz 발행
- cmd_vel 시 position.x 증가
- diagnostics 에러 없음

---

### 3단계: IMU 단독 테스트

```bash
ros2 run stella_ahrs stella_ahrs_node --ros-args \
  -p port:=/dev/imu -p baud_rate:=115200
```

```bash
ros2 topic hz /imu/data               # 목표: ~50Hz
ros2 topic echo /imu/data --once       # orientation, angular_velocity 값 확인

# 로봇을 손으로 회전시키며 확인
ros2 topic echo /imu/data --field angular_velocity.z
# 시계방향: 음수, 반시계방향: 양수 (또는 반대 — 일관성만 확인)
```

통과 기준:
- `/imu/data` ~50Hz 발행
- 회전 시 angular_velocity.z 변화

---

### 4단계: LiDAR 단독 테스트

```bash
# 전면 LiDAR
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/rplidar_back -p serial_baudrate:=115200 \
  -p frame_id:=base_scan -p scan_mode:=Sensitivity \
  --remap scan:=scan_raw_front
```

```bash
ros2 topic hz /scan_raw_front          # 목표: ~7-10Hz
ros2 topic echo /scan_raw_front --field ranges --once | head -5
```

후면 LiDAR도 동일하게 테스트 (`/dev/rplidar_front`, `scan_raw_back`).

통과 기준:
- 각 LiDAR ~7-15Hz 발행
- ranges에 유한한 값(벽/장애물 거리) 존재

---

### 5단계: 브링업 전체 실행 + EKF 확인

```bash
ros2 launch sendbooster_agv_bringup bringup.launch.py num_lidars:=2
```

```bash
# 토픽 주파수 확인
ros2 topic hz /odom                    # ~50Hz
ros2 topic hz /imu/data                # ~50Hz
ros2 topic hz /scan_merged             # ~7-10Hz
ros2 topic hz /odometry/filtered       # ~30Hz

# TF 확인
ros2 run tf2_ros tf2_echo odom base_footprint
# Translation, Rotation 값이 나와야 함 (에러 아님)

# EKF 동작 확인: 로봇 정지 상태에서
ros2 topic echo /odometry/filtered --field pose.pose.position
# x, y가 안정적이어야 함 (drift 거의 없음)
```

통과 기준:
- 4개 토픽 모두 정상 주파수
- `odom → base_footprint` TF 존재
- 정지 시 EKF 출력 안정

---

### 6단계: 오도메트리 직진 캘리브레이션

```bash
# 바닥에 2m 표시
ros2 service call /motor_driver_node/reset_position std_srvs/srv/Trigger

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10
# 2m 지점에서 정지

ros2 topic echo /odometry/filtered --field pose.pose.position.x --once
```

- **목표**: position.x ≈ 2.0 (오차 ±5% 이내)
- **오차 크면**: `wheel_radius` 조정 (실측 > odom → radius 줄이기)

---

### 7단계: 오도메트리 회전 캘리브레이션

```bash
ros2 service call /motor_driver_node/reset_position std_srvs/srv/Trigger

# 제자리 회전 (360도)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}" -r 10
# 로봇이 정확히 1바퀴 돌면 정지

ros2 topic echo /odometry/filtered --field pose.pose.orientation --once
# yaw ≈ 0 (또는 ±2π) 이면 OK
```

- **오차 크면**: `wheel_separation` 조정 (실제 > odom → separation 늘리기)

---

### 8단계: scan_merged 확인 (PC에서 RViz)

```bash
# PC에서 RViz 실행
ros2 launch sendbooster_agv_bringup rviz_view.launch.py
```

RViz에서 확인:
- LaserScan `/scan_merged` 추가 → 360도 스캔 표시되는지
- 로봇 앞/뒤/옆 장애물이 모두 보이는지
- 로봇 몸체 부분은 빈 영역 (angle filter 동작 확인)

---

### 9단계: SLAM 테스트

```bash
# Jetson
ros2 launch sendbooster_agv_bringup cartographer.launch.py

# PC에서 RViz로 맵 생성 과정 확인
# teleop으로 천천히 주행하며 맵 빌드
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

확인:
- 맵이 깨끗하게 빌드되는지 (이중선, 떨림 없음)
- 긴 복도 왕복 후 루프 클로저 동작하는지

맵 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map
```

---

### 10단계: Nav2 기본 동작

```bash
# Jetson
ros2 launch sendbooster_agv_bringup navigation2.launch.py map:=~/maps/test_map.yaml
```

PC RViz에서:
1. **2D Pose Estimate** → 로봇 초기 위치 설정
2. `/particle_cloud` 확인 → 파티클이 로봇 주변에 수렴하는지
3. **2D Goal Pose** → 가까운 목표 (2-3m) 설정
4. 로봇이 경로 따라 이동하는지 확인

통과 기준:
- AMCL 파티클 수렴
- 가까운 목표까지 자율 주행 성공

---

### 11단계: 장애물 회피 테스트

1. Nav2 주행 중 경로 위에 장애물 배치
2. RViz에서 `local_costmap`에 장애물 표시되는지 확인
3. 로봇이 장애물을 우회하여 목표에 도달하는지 확인

문제 시:
- 장애물 안 보임 → `/scan_merged` 토픽, costmap의 `obstacle_max_range` 확인
- 장애물에 부딪힘 → `BaseObstacle.scale` 올리기 (50 → 60~80)
- 좁은 곳 못 지나감 → `inflation_radius` 줄이기 (0.6 → 0.5)

---

### 12단계: 장거리 네비게이션

1. 10m 이상 떨어진 목표 설정
2. 중간에 방향 전환이 있는 경로 확인
3. 도착 후 위치 정확도 확인 (RViz에서 파티클 수렴 유지)

문제 시:
- 중간에 멈춤 → `progress_checker` 의 `movement_time_allowance` 확인
- 경로 이탈 → `PathDist.scale` 올리기
- 위치 틀어짐 → AMCL `alpha1~4` 조정

---

### 트러블슈팅 체크리스트

| 증상 | 확인 명령어 | 원인/해결 |
|------|-----------|----------|
| 토픽 안 나옴 | `ros2 topic list` | 노드 실행 확인, 포트 연결 확인 |
| 주파수 낮음 | `ros2 topic hz <topic>` | 시리얼 포트 에러, CPU 과부하 확인 |
| TF 에러 | `ros2 run tf2_ros tf2_monitor` | 누락된 TF 프레임 확인 |
| EKF 발산 | `ros2 topic echo /odometry/filtered` | odom/imu covariance, rejection threshold 조정 |
| AMCL 발산 | RViz `/particle_cloud` | 초기 위치 재설정, alpha 값 조정 |
| 모터 에러 | `ros2 topic echo /diagnostics` | 과전류/과전압/과열 확인 |

---

## 참고 자료

- [MDROBOT 공식 홈페이지](https://www.mdrobot.co.kr)
- [RPLIDAR A2M8 Datasheet](https://www.slamtec.com/en/Lidar/A2)
- [Nav2 Documentation](https://docs.nav2.org/)
- [Nav2 Tuning Guide](https://docs.nav2.org/tuning/)
- [robot_localization Wiki](http://docs.ros.org/en/noetic/api/robot_localization/html/)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)

## 라이선스

Apache-2.0
