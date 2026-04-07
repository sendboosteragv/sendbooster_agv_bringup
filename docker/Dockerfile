# Dockerfile for Sendbooster AGV Simulation
# Provides ROS2 Humble + Gazebo + Nav2 runtime environment
#
# Usage:
#   docker build -t sendbooster-sim ./docker
#   See docker-compose.yml for running with Jetson Orin Nano constraints

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install Nav2, robot_localization, and Gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-bringup \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-mppi-controller \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    python3-pip \
    && pip3 install --no-cache-dir psutil \
    && rm -rf /var/lib/apt/lists/*

# Setup entrypoint
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
if [ -f /ros2_ws/install/setup.bash ]; then\n\
  source /ros2_ws/install/setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
