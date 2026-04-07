#!/bin/bash
# Auto-tune Nav2 parameters via Docker simulation
# Usage: ./auto_tune_test.sh [test_name]
#
# Starts gazebo (if not running), restarts nav container with current params,
# runs waypoint test, logs results.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
DOCKER_DIR="$PKG_DIR/docker"
LOG_DIR="$PKG_DIR/test_logs"
DOMAIN_ID="${ROS_DOMAIN_ID:-31}"

TEST_NAME="${1:-test_$(date +%Y%m%d_%H%M%S)}"
LOG_FILE="$LOG_DIR/${TEST_NAME}.log"

mkdir -p "$LOG_DIR"

echo "=== Auto Tune Test: $TEST_NAME ===" | tee "$LOG_FILE"
echo "Time: $(date)" | tee -a "$LOG_FILE"
echo "ROS_DOMAIN_ID: $DOMAIN_ID" | tee -a "$LOG_FILE"

# Log current nav2 params
echo "" | tee -a "$LOG_FILE"
echo "--- nav2_params_odom.yaml (key params) ---" | tee -a "$LOG_FILE"
grep -E '(controller_frequency|xy_goal_tolerance|yaw_goal_tolerance|max_vel_x|max_vel_theta|min_vel_x|sim_time|vx_samples|vtheta_samples|RotateToGoal|inflation_radius|slowing_factor|lookahead_time|required_movement|time_allowance)' \
    "$PKG_DIR/config/nav2_params_odom.yaml" | tee -a "$LOG_FILE"
echo "---" | tee -a "$LOG_FILE"

# Rebuild workspace
echo "" | tee -a "$LOG_FILE"
echo "[1/4] Building workspace..." | tee -a "$LOG_FILE"
cd /home/seongmin/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sendbooster_agv_bringup --symlink-install 2>&1 | tail -3 | tee -a "$LOG_FILE"

# Check if gazebo is running
if docker ps --format '{{.Names}}' | grep -q sendbooster-gazebo; then
    echo "[2/4] Gazebo already running, restarting nav only..." | tee -a "$LOG_FILE"
    cd "$DOCKER_DIR"
    ROS_DOMAIN_ID=$DOMAIN_ID docker compose stop nav 2>&1 | tee -a "$LOG_FILE"
    ROS_DOMAIN_ID=$DOMAIN_ID docker compose rm -f nav 2>&1 | tee -a "$LOG_FILE"
    ROS_DOMAIN_ID=$DOMAIN_ID docker compose up -d nav 2>&1 | tee -a "$LOG_FILE"
else
    echo "[2/4] Starting gazebo + nav..." | tee -a "$LOG_FILE"
    cd "$DOCKER_DIR"
    ROS_DOMAIN_ID=$DOMAIN_ID docker compose up -d gazebo nav 2>&1 | tee -a "$LOG_FILE"
fi

# Wait for Nav2 to be ready
echo "[3/4] Waiting for Nav2 lifecycle..." | tee -a "$LOG_FILE"
MAX_WAIT=120
WAITED=0
while [ $WAITED -lt $MAX_WAIT ]; do
    # Check if navigate_to_pose action is available
    if ROS_DOMAIN_ID=$DOMAIN_ID ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
        echo "  Nav2 action server found after ${WAITED}s" | tee -a "$LOG_FILE"
        # Give lifecycle manager a few more seconds to fully activate
        sleep 5
        break
    fi
    sleep 2
    WAITED=$((WAITED + 2))
    if [ $((WAITED % 20)) -eq 0 ]; then
        echo "  Still waiting... (${WAITED}s)" | tee -a "$LOG_FILE"
    fi
done

if [ $WAITED -ge $MAX_WAIT ]; then
    echo "ERROR: Nav2 did not start after ${MAX_WAIT}s" | tee -a "$LOG_FILE"
    echo "Nav container logs:" | tee -a "$LOG_FILE"
    docker logs sendbooster-nav --tail 50 2>&1 | tee -a "$LOG_FILE"
    exit 1
fi

# Run waypoint test
echo "[4/4] Running waypoint test..." | tee -a "$LOG_FILE"
cd /home/seongmin/ros2_ws
source install/setup.bash

ROS_DOMAIN_ID=$DOMAIN_ID timeout 600 ros2 run sendbooster_agv_bringup nav2_waypoint_test.py \
    --ros-args -p use_sim_time:=true -p preset:=warehouse 2>&1 | tee -a "$LOG_FILE"

TEST_EXIT=$?

# CPU stats
echo "" | tee -a "$LOG_FILE"
echo "--- Docker Stats ---" | tee -a "$LOG_FILE"
docker stats sendbooster-nav --no-stream 2>&1 | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "Test exit code: $TEST_EXIT" | tee -a "$LOG_FILE"
echo "Log saved: $LOG_FILE" | tee -a "$LOG_FILE"
echo "=== Done ===" | tee -a "$LOG_FILE"
