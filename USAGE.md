# Sendbooster AGV 실행 가이드

## 네트워크 구성
- Jetson Orin Nano (로봇): `172.20.10.14`
- PC (원격 제어): 같은 네트워크에 연결

## 사전 준비 (최초 1회)

### Jetson
```bash
cd ~/ros2_ws/src/sendbooster_agv_bringup
git pull origin dev-reverse

cd ~/ros2_ws
colcon build --packages-select sendbooster_agv_bringup
source install/setup.bash
```

### PC
```bash
cd ~/ros2_ws/src/sendbooster_agv_simulation/agv_web_control/frontend
npm install
```

## 실행 순서

### 1. Jetson — 터미널 1: Bringup (센서 + 모터 + EKF)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sendbooster_agv_bringup bringup.launch.py
```
LiDAR 1개만 사용 시 (기본값). 2개 사용 시 `num_lidars:=2` 추가.

### 2. Jetson — 터미널 2: Navigation (Nav2)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sendbooster_agv_bringup navigation2.launch.py
```
다른 맵 사용 시: `map:=/path/to/map.yaml`

### 3. Jetson — 터미널 3: 백엔드 (웹 API 서버)
```bash
cd ~/ros2_ws/src/sendbooster_agv_bringup
./start_backend.sh
```
최초 실행 시 venv 자동 생성. 포트: `8000`

### 4. PC — 프론트엔드 (웹 UI)
```bash
cd ~/ros2_ws/src/sendbooster_agv_simulation/agv_web_control/frontend
npm run dev
```
브라우저에서 `http://localhost:3000` 접속.

`vite.config.ts`의 IP가 젯슨 IP(`172.20.10.14`)와 일치하는지 확인.

### (선택) PC — RViz2
```bash
export ROS_DOMAIN_ID=0
rviz2
```
Add → By topic → `/map`, `/scan`, `/global_costmap/costmap` 등 추가.

## 종료 순서

역순으로 `Ctrl+C`:
1. PC: 프론트엔드 / RViz2
2. Jetson 터미널 3: 백엔드
3. Jetson 터미널 2: Navigation
4. Jetson 터미널 1: Bringup

## 네트워크 끊김 시

- PC와 젯슨이 끊겨도 **네비게이션은 계속 진행**됨
- goal / waypoint 모두 젯슨에서 독립 실행
- 재연결 후 RViz2나 웹 UI에서 상태 다시 확인 가능

## Keepout Zone (출입금지 구역) 편집

1. `map/keepout.pgm`을 GIMP에서 열기
2. 출입금지 구역을 검정색(값 0)으로 칠하기
3. `Shift+Ctrl+E`로 내보내기 (PGM 형식 유지)
4. 빌드 후 재실행
