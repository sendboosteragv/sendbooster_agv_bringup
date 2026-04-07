"""
Robot State - Holds current robot state from ROS2 topics
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from datetime import datetime


@dataclass
class RobotState:
    """Current state of the robot"""
    # Position (from AMCL or odom)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    # Velocity (from odom)
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0

    # Navigation status
    nav_status: str = "idle"  # idle, navigating, succeeded, failed, cancelled
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_yaw: Optional[float] = None

    # Planned path
    path: List[Tuple[float, float]] = field(default_factory=list)

    # Sensor status
    lidar_ok: bool = False
    imu_ok: bool = False

    # Simulated battery (for demo purposes)
    battery_percent: float = 100.0

    # Timestamps
    last_pose_update: Optional[datetime] = None
    last_odom_update: Optional[datetime] = None

    def to_dict(self):
        """Convert to dictionary for JSON serialization"""
        return {
            "position": {
                "x": round(self.x, 3),
                "y": round(self.y, 3),
                "yaw": round(self.yaw, 3),
                "yaw_degrees": round(math.degrees(self.yaw), 1),
            },
            "velocity": {
                "linear": round(self.linear_velocity, 3),
                "angular": round(self.angular_velocity, 3),
            },
            "navigation": {
                "status": self.nav_status,
                "goal": {
                    "x": round(self.goal_x, 3) if self.goal_x is not None else None,
                    "y": round(self.goal_y, 3) if self.goal_y is not None else None,
                    "yaw": round(self.goal_yaw, 3) if self.goal_yaw is not None else None,
                },
            },
            "path": [[round(p[0], 3), round(p[1], 3)] for p in self.path],
            "sensors": {
                "lidar": self.lidar_ok,
                "imu": self.imu_ok,
            },
            "battery": round(self.battery_percent, 1),
            "timestamps": {
                "pose": self.last_pose_update.isoformat() if self.last_pose_update else None,
                "odom": self.last_odom_update.isoformat() if self.last_odom_update else None,
            },
        }

    def update_from_pose(self, x: float, y: float, yaw: float):
        """Update position from AMCL pose"""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.last_pose_update = datetime.now()

    def update_from_odom(self, linear: float, angular: float):
        """Update velocity from odom"""
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.last_odom_update = datetime.now()

    def set_goal(self, x: float, y: float, yaw: float):
        """Set navigation goal"""
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw
        self.nav_status = "navigating"
        self.path = []  # Clear previous path when setting new goal

    def clear_goal(self):
        """Clear navigation goal"""
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.path = []

    def set_path(self, path: List[Tuple[float, float]]):
        """Set planned path"""
        self.path = path
