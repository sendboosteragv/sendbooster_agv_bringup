#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/seongmin/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=31
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/config/fastdds_udp.xml

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/recordings/nav_${TIMESTAMP}"
LOG_FILE="/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/test_logs/test_${TIMESTAMP}.log"

echo "=== Starting bag recording: $BAG_DIR ==="
ros2 bag record -o "$BAG_DIR" \
  /odom /odometry/filtered /scan_merged /cmd_vel /tf /tf_static \
  --use-sim-time --max-bag-duration 600 &
BAG_PID=$!
sleep 3

echo "=== Starting waypoint test ==="
ros2 run sendbooster_agv_bringup nav2_waypoint_test.py \
  --ros-args -p use_sim_time:=true -p preset:=warehouse 2>&1 | tee "$LOG_FILE"

echo "=== Stopping bag ==="
kill $BAG_PID 2>/dev/null
wait $BAG_PID 2>/dev/null || true
sleep 2

echo "=== Results ==="
echo "Bag: $(du -sh "$BAG_DIR" 2>/dev/null)"
echo "Log: $LOG_FILE"
