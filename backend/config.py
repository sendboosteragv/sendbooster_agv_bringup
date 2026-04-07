"""
AGV Web Control - Configuration
"""
import os
from pathlib import Path

# Base directories
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
DATA_DIR = PROJECT_ROOT / "data"
MAPS_DIR = DATA_DIR / "maps"

# Create directories if they don't exist
DATA_DIR.mkdir(exist_ok=True)
MAPS_DIR.mkdir(exist_ok=True)

# Database
DATABASE_URL = f"sqlite:///{DATA_DIR}/agv_control.db"

# Server
HOST = os.getenv("AGV_HOST", "0.0.0.0")
PORT = int(os.getenv("AGV_PORT", 8000))

# ROS2
ROS2_DOMAIN_ID = int(os.getenv("ROS_DOMAIN_ID", 0))

# Robot configuration (single robot for now, expandable to fleet)
DEFAULT_ROBOT_ID = "sendbooster_agv"
ROBOT_NAMESPACE = ""  # Empty for single robot, "/robot1" etc for fleet

# ROS2 Topics
TOPICS = {
    "odom": "/odom",
    "cmd_vel": "/cmd_vel",
    "scan": "/scan_merged",
    "amcl_pose": "/amcl_pose",
    "initial_pose": "/initialpose",
    "goal_pose": "/goal_pose",
    "nav_path": "/plan",
    "nav_status": "/navigate_to_pose/_action/status",
}

# ROS2 Actions
ACTIONS = {
    "navigate_to_pose": "/navigate_to_pose",
    "follow_waypoints": "/follow_waypoints",
}

# WebSocket
WS_BROADCAST_INTERVAL = 0.1  # 100ms = 10Hz update rate
