#!/bin/bash
# record_navigation.sh — Record Nav2 simulation with ROS2 bag + RViz2 screen capture
#
# Usage:
#   ./record_navigation.sh [--no-rviz] [--no-video] [--duration SECONDS]
#
# Prerequisites:
#   - Gazebo + Nav containers running
#   - xhost +local: (for RViz X11 access)
#   - ffmpeg installed (for screen recording)
#   - FASTRTPS_DEFAULT_PROFILES_FILE set if using FastDDS

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
RECORDING_DIR="${PKG_DIR}/recordings"
RVIZ_CONFIG="${PKG_DIR}/rviz/nav_sim.rviz"
FASTDDS_XML="${PKG_DIR}/config/fastdds_udp.xml"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${RECORDING_DIR}/nav_bag_${TIMESTAMP}"
VIDEO_FILE="${RECORDING_DIR}/nav_video_${TIMESTAMP}.mp4"

# Defaults
LAUNCH_RVIZ=true
RECORD_VIDEO=true
DURATION=0  # 0 = unlimited

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-rviz)   LAUNCH_RVIZ=false; shift ;;
        --no-video)  RECORD_VIDEO=false; shift ;;
        --duration)  DURATION="$2"; shift 2 ;;
        *)           echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# Ensure recordings directory exists
mkdir -p "${RECORDING_DIR}"

# Set FastDDS profile if not already set
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]] && [[ -f "${FASTDDS_XML}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_XML}"
    echo "[INFO] Set FASTRTPS_DEFAULT_PROFILES_FILE=${FASTDDS_XML}"
fi

# Set ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# Source ROS2
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
    source "${HOME}/ros2_ws/install/setup.bash"
fi

# Array to track background PIDs for cleanup
PIDS=()

cleanup() {
    echo ""
    echo "[INFO] Stopping all recording processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
            # Wait briefly for graceful shutdown
            for i in {1..10}; do
                kill -0 "$pid" 2>/dev/null || break
                sleep 0.5
            done
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    echo "[INFO] Recording stopped."
    echo "[INFO] Bag saved to: ${BAG_DIR}"
    if [[ "${RECORD_VIDEO}" == "true" ]] && [[ -f "${VIDEO_FILE}" ]]; then
        echo "[INFO] Video saved to: ${VIDEO_FILE}"
    fi
    echo "[INFO] Done."
}
trap cleanup EXIT INT TERM

# ── 1. Start ROS2 bag recording ──
echo "[INFO] Starting ros2 bag record → ${BAG_DIR}"
TOPICS=(
    /odom
    /odometry/filtered
    /amcl_pose
    /scan_merged
    /cmd_vel
    /tf
    /tf_static
    /map
    /global_costmap/costmap
    /local_costmap/costmap
    /plan
    /local_plan
    /particlecloud
)

ros2 bag record -o "${BAG_DIR}" "${TOPICS[@]}" --use-sim-time &
PIDS+=($!)
echo "[INFO] Bag recorder PID: ${PIDS[-1]}"

# ── 2. Launch RViz2 ──
if [[ "${LAUNCH_RVIZ}" == "true" ]]; then
    echo "[INFO] Launching RViz2 with config: ${RVIZ_CONFIG}"
    ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
    PIDS+=($RVIZ_PID)
    echo "[INFO] RViz2 PID: ${RVIZ_PID}"

    # Wait for RViz window to appear
    echo "[INFO] Waiting for RViz2 window..."
    for i in {1..30}; do
        if xdotool search --name "RViz2" >/dev/null 2>&1; then
            echo "[INFO] RViz2 window detected."
            break
        fi
        sleep 1
    done
fi

# ── 3. Screen recording with ffmpeg ──
if [[ "${RECORD_VIDEO}" == "true" ]] && [[ "${LAUNCH_RVIZ}" == "true" ]]; then
    # Try to find RViz window for targeted recording
    RVIZ_WID=""
    for i in {1..10}; do
        RVIZ_WID=$(xdotool search --name "RViz2" 2>/dev/null | head -1) || true
        if [[ -n "${RVIZ_WID}" ]]; then
            break
        fi
        sleep 1
    done

    if [[ -n "${RVIZ_WID}" ]]; then
        # Get window geometry
        eval "$(xdotool getwindowgeometry --shell "${RVIZ_WID}")"
        echo "[INFO] Recording RViz2 window at ${X},${Y} ${WIDTH}x${HEIGHT}"
        ffmpeg -y -f x11grab -framerate 15 \
            -video_size "${WIDTH}x${HEIGHT}" \
            -i "${DISPLAY}+${X},${Y}" \
            -c:v libx264 -preset ultrafast -crf 23 \
            -pix_fmt yuv420p \
            "${VIDEO_FILE}" &
    else
        # Fallback: record full screen
        echo "[INFO] Could not find RViz window, recording full screen"
        SCREEN_RES=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")
        ffmpeg -y -f x11grab -framerate 15 \
            -video_size "${SCREEN_RES}" \
            -i "${DISPLAY}+0,0" \
            -c:v libx264 -preset ultrafast -crf 23 \
            -pix_fmt yuv420p \
            "${VIDEO_FILE}" &
    fi
    PIDS+=($!)
    echo "[INFO] Video recorder PID: ${PIDS[-1]}"
fi

echo ""
echo "========================================"
echo "  Recording in progress"
echo "  Bag: ${BAG_DIR}"
if [[ "${RECORD_VIDEO}" == "true" ]]; then
    echo "  Video: ${VIDEO_FILE}"
fi
echo "  Press Ctrl+C to stop recording"
echo "========================================"
echo ""

# Wait for duration or Ctrl+C
if [[ "${DURATION}" -gt 0 ]]; then
    echo "[INFO] Recording for ${DURATION} seconds..."
    sleep "${DURATION}"
else
    # Wait forever until interrupted
    wait "${PIDS[0]}" 2>/dev/null || true
fi
