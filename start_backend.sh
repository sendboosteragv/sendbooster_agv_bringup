#!/bin/bash
# AGV Web Control - Backend Startup Script

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Change to backend directory
cd "$(dirname "$0")/backend"

# Create virtual environment if not exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    source venv/bin/activate
fi

# Start server
echo "Starting AGV Web Control backend..."
python3 main.py
