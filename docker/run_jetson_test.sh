#!/bin/bash
#
# Jetson Orin Nano Performance Test
#
# Tests Nav2 under CPU/memory constraints matching Jetson Orin Nano.
# Supports normal (warehouse) and stress (factory maze) modes.
#
# Usage:
#   cd ~/ros2_ws/src/sendbooster_agv_bringup/docker
#   ./run_jetson_test.sh                    # Normal test, CPU 3.0/2.0/1.5
#   ./run_jetson_test.sh --stress           # Stress test (factory maze + delays)
#   ./run_jetson_test.sh --stress 2.0       # Stress test, CPU 2.0 only
#   ./run_jetson_test.sh 2.0               # Normal test, CPU 2.0 only
#
# Prerequisites:
#   1. colcon build && docker compose build
#   2. xhost +local:docker

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/test_logs"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Parse args
STRESS_MODE=false
CPU_LIMITS=()
for arg in "$@"; do
    if [ "$arg" = "--stress" ]; then
        STRESS_MODE=true
    else
        CPU_LIMITS+=("$arg")
    fi
done

# Default CPU limits
if [ ${#CPU_LIMITS[@]} -eq 0 ]; then
    CPU_LIMITS=(3.0 2.0 1.5)
fi

# Select launch file and waypoint preset
if [ "$STRESS_MODE" = true ]; then
    LAUNCH_FILE="stress_test.launch.py"
    LAUNCH_GAZEBO="stress_test.launch.py"
    WAYPOINT_PRESET="factory"
    MAP_FILE="/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/map/warehouse_sim.yaml"
    MODE_LABEL="STRESS"
    GAZEBO_WAIT=25
    NAV_WAIT=30
else
    LAUNCH_FILE="simulation.launch.py"
    LAUNCH_GAZEBO="simulation.launch.py"
    WAYPOINT_PRESET="warehouse"
    MAP_FILE="/home/seongmin/ros2_ws/src/sendbooster_agv_bringup/map/warehouse_sim.yaml"
    MODE_LABEL="NORMAL"
    GAZEBO_WAIT=15
    NAV_WAIT=25
fi

mkdir -p "$LOG_DIR"

echo "=============================================="
echo "  Jetson Orin Nano Nav2 Performance Test"
echo "  Mode: ${MODE_LABEL}"
echo "  CPU limits: ${CPU_LIMITS[*]}"
echo "  ROS_DOMAIN_ID: ${DOMAIN_ID}"
echo "=============================================="

# Ensure Docker image exists
if ! docker images sendbooster-sim --format "{{.Repository}}" | grep -q sendbooster-sim; then
    echo "Building Docker image..."
    cd "$SCRIPT_DIR"
    docker compose build
fi

cleanup() {
    echo ""
    echo "Cleaning up containers..."
    docker stop sendbooster-gazebo-test sendbooster-nav-test 2>/dev/null || true
    docker rm sendbooster-gazebo-test sendbooster-nav-test 2>/dev/null || true
}
trap cleanup EXIT

for CPU in "${CPU_LIMITS[@]}"; do
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    LOG_FILE="${LOG_DIR}/${MODE_LABEL,,}_cpu${CPU}_${TIMESTAMP}.log"
    STATS_FILE="${LOG_DIR}/${MODE_LABEL,,}_stats_cpu${CPU}_${TIMESTAMP}.csv"

    echo ""
    echo "=============================================="
    echo "  [${MODE_LABEL}] CPU limit: ${CPU} cores"
    echo "=============================================="

    # Cleanup previous
    cleanup 2>/dev/null || true
    sleep 2

    # Start Gazebo
    echo "[1/4] Starting Gazebo (${MODE_LABEL})..."
    docker run -d \
        --name sendbooster-gazebo-test \
        --network=host --ipc=host \
        -e DISPLAY="${DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
        -e GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${HOME}/ros2_ws:${HOME}/ros2_ws:ro" \
        sendbooster-sim \
        bash -c "source /home/seongmin/ros2_ws/install/setup.bash && \
                 ros2 launch sendbooster_agv_bringup ${LAUNCH_GAZEBO} \
                 gazebo:=true nav:=false headless:=true"

    echo "  Waiting ${GAZEBO_WAIT}s..."
    sleep "$GAZEBO_WAIT"

    if ! docker ps --format "{{.Names}}" | grep -q sendbooster-gazebo-test; then
        echo "  ERROR: Gazebo failed to start"
        docker logs sendbooster-gazebo-test 2>&1 | tail -10
        continue
    fi

    # Start Nav with CPU limit
    echo "[2/4] Starting Nav2 (CPU: ${CPU}, RAM: 7GB)..."
    docker run -d \
        --name sendbooster-nav-test \
        --network=host --ipc=host \
        --cpus="${CPU}" --memory=7g --memory-swap=7g \
        -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
        -v "${HOME}/ros2_ws:${HOME}/ros2_ws:ro" \
        sendbooster-sim \
        bash -c "source /home/seongmin/ros2_ws/install/setup.bash && \
                 ros2 launch sendbooster_agv_bringup ${LAUNCH_FILE} \
                 gazebo:=false nav:=true map:=${MAP_FILE}"

    echo "  Waiting ${NAV_WAIT}s..."
    sleep "$NAV_WAIT"

    if ! docker ps --format "{{.Names}}" | grep -q sendbooster-nav-test; then
        echo "  ERROR: Nav container failed"
        docker logs sendbooster-nav-test 2>&1 | tail -20
        continue
    fi

    # Start resource monitoring
    echo "[3/4] Starting resource monitor..."
    (
        echo "timestamp,cpu_percent,mem_usage,mem_percent"
        while docker ps --format "{{.Names}}" | grep -q sendbooster-nav-test; do
            STAT=$(docker stats sendbooster-nav-test --format \
                "{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}}" --no-stream 2>/dev/null)
            if [ -n "$STAT" ]; then
                echo "$(date +%H:%M:%S),$STAT"
            fi
            sleep 5
        done
    ) > "$STATS_FILE" &
    MONITOR_PID=$!

    # Run waypoint test
    echo "[4/4] Running waypoint test (preset: ${WAYPOINT_PRESET})..."
    docker exec sendbooster-nav-test bash -c \
        "source /home/seongmin/ros2_ws/install/setup.bash && \
         ros2 run sendbooster_agv_bringup nav2_waypoint_test.py \
         --ros-args -p use_sim_time:=true -p preset:=${WAYPOINT_PRESET}" \
        2>&1 | tee "$LOG_FILE"

    # Stop monitoring
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true

    # Resource summary
    if [ -f "$STATS_FILE" ] && [ "$(wc -l < "$STATS_FILE")" -gt 1 ]; then
        echo ""
        echo "--- Resource Usage (CPU: ${CPU}) ---"
        echo "Samples: $(( $(wc -l < "$STATS_FILE") - 1 ))"
        tail -5 "$STATS_FILE" | column -t -s ','
    fi

    cleanup 2>/dev/null || true
    sleep 3
done

echo ""
echo "=============================================="
echo "  All tests complete!"
echo "  Logs: $LOG_DIR"
echo "=============================================="
echo ""
echo "Quick analysis:"
echo "  grep -E '(Result|Total time)' ${LOG_DIR}/${MODE_LABEL,,}_cpu*.log"
