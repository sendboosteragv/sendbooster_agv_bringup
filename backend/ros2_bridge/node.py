"""
ROS2 Bridge Node - Connects FastAPI to ROS2
"""
import math
import threading
from typing import Optional, Callable, List
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus

from ros2_bridge.robot_state import RobotState


class ROS2Bridge(Node):
    """ROS2 Bridge node for web control"""

    def __init__(self, state_callback: Optional[Callable] = None):
        super().__init__("agv_web_control_bridge")

        self.robot_state = RobotState()
        self.state_callback = state_callback
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._waypoints_goal_future = None
        self._waypoints_goal_handle = None

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            reliable_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            sensor_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan_merged",
            self._scan_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            sensor_qos
        )

        self.path_sub = self.create_subscription(
            Path,
            "/plan",
            self._path_callback,
            reliable_qos
        )

        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            reliable_qos
        )

        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            reliable_qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            reliable_qos
        )

        # Action clients
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            "/follow_waypoints"
        )

        self.get_logger().info("ROS2 Bridge initialized")

    def _quaternion_to_yaw(self, q) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_to_quaternion(self, yaw: float):
        """Convert yaw to quaternion (z, w components)"""
        return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose updates"""
        pos = msg.pose.pose.position
        yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_state.update_from_pose(pos.x, pos.y, yaw)

        if self.state_callback:
            self.state_callback("pose", self.robot_state)

    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        self.robot_state.update_from_odom(linear, angular)

        # If no AMCL pose yet, use odom for position
        if self.robot_state.last_pose_update is None:
            pos = msg.pose.pose.position
            yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
            self.robot_state.update_from_pose(pos.x, pos.y, yaw)

    def _scan_callback(self, msg: LaserScan):
        """Handle laser scan - just track sensor health"""
        self.robot_state.lidar_ok = True

    def _imu_callback(self, msg: Imu):
        """Handle IMU - just track sensor health"""
        self.robot_state.imu_ok = True

    def _path_callback(self, msg: Path):
        """Handle path updates from Nav2"""
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.robot_state.set_path(path)

        if self.state_callback:
            self.state_callback("path", self.robot_state)

    def set_initial_pose(self, x: float, y: float, yaw: float):
        """Set initial pose for AMCL"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        qz, qw = self._yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Set covariance (small values = high confidence)
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.07  # yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Initial pose set: ({x}, {y}, {yaw})")

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        """Send navigation goal using action"""
        try:
            # Cancel previous goal if exists
            if self._nav_goal_handle is not None:
                self.get_logger().info("Cancelling previous goal...")
                try:
                    self._nav_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f"Failed to cancel previous goal: {e}")
                self._nav_goal_handle = None

            if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("NavigateToPose action server not available")
                return False

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y

            qz, qw = self._yaw_to_quaternion(yaw)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            self.robot_state.set_goal(x, y, yaw)

            self._nav_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self._nav_feedback_callback
            )
            self._nav_goal_future.add_done_callback(self._nav_goal_response_callback)

            self.get_logger().info(f"Goal sent: ({x}, {y}, {yaw})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            return False

    def send_waypoints(self, waypoints: List[dict]) -> bool:
        """Send multiple waypoints using FollowWaypoints action"""
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("FollowWaypoints action server not available")
            return False

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = []

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp["x"]
            pose.pose.position.y = wp["y"]

            yaw = wp.get("yaw", 0.0)
            qz, qw = self._yaw_to_quaternion(yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            goal_msg.poses.append(pose)

        if waypoints:
            last = waypoints[-1]
            self.robot_state.set_goal(last["x"], last["y"], last.get("yaw", 0.0))

        self._waypoints_goal_handle = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self._waypoints_feedback_callback
        )
        self._waypoints_goal_handle.add_done_callback(self._waypoints_goal_response_callback)

        self.get_logger().info(f"Waypoints sent: {len(waypoints)} points")
        return True

    def cancel_goal(self):
        """Cancel current navigation"""
        try:
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
                self._nav_goal_handle = None
            if self._waypoints_goal_handle is not None:
                self._waypoints_goal_handle.cancel_goal_async()
                self._waypoints_goal_handle = None

            self.robot_state.nav_status = "cancelled"
            self.robot_state.clear_goal()
            self.get_logger().info("Navigation cancelled")
        except Exception as e:
            self.get_logger().error(f"Failed to cancel goal: {e}")

    def send_cmd_vel(self, linear: float, angular: float):
        """Send velocity command (manual control)"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def _nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass  # Can be used to track progress

    def _nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.robot_state.nav_status = "rejected"
                self.get_logger().warn("Goal rejected")
                self._nav_goal_handle = None
                return

            # Store the actual goal handle for cancellation
            self._nav_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(self._nav_result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")
            self._nav_goal_handle = None

    def _nav_result_callback(self, future):
        """Handle navigation result"""
        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.robot_state.nav_status = "succeeded"
                self.get_logger().info("Goal reached!")
            elif status == GoalStatus.STATUS_CANCELED:
                self.robot_state.nav_status = "cancelled"
                self.get_logger().info("Goal cancelled")
            else:
                self.robot_state.nav_status = "failed"
                self.get_logger().warn(f"Goal failed with status: {status}")

            self._nav_goal_handle = None
            self.robot_state.clear_goal()

            if self.state_callback:
                self.state_callback("nav_result", self.robot_state)
        except Exception as e:
            self.get_logger().error(f"Nav result error: {e}")
            self._nav_goal_handle = None

    def _waypoints_feedback_callback(self, feedback_msg):
        """Handle waypoints feedback"""
        current_wp = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f"Waypoint {current_wp} in progress")

    def _waypoints_goal_response_callback(self, future):
        """Handle waypoints goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.robot_state.nav_status = "rejected"
                self._waypoints_goal_handle = None
                return

            self._waypoints_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(self._waypoints_result_callback)
        except Exception as e:
            self.get_logger().error(f"Waypoints goal response error: {e}")
            self._waypoints_goal_handle = None

    def _waypoints_result_callback(self, future):
        """Handle waypoints result"""
        try:
            result = future.result()
            missed = result.result.missed_waypoints

            if len(missed) == 0:
                self.robot_state.nav_status = "succeeded"
                self.get_logger().info("All waypoints reached!")
            else:
                self.robot_state.nav_status = "partial"
                self.get_logger().warn(f"Missed waypoints: {missed}")

            self._waypoints_goal_handle = None
            self.robot_state.clear_goal()

            if self.state_callback:
                self.state_callback("nav_result", self.robot_state)
        except Exception as e:
            self.get_logger().error(f"Waypoints result error: {e}")
            self._waypoints_goal_handle = None

    def get_state(self) -> dict:
        """Get current robot state as dictionary"""
        return self.robot_state.to_dict()


class ROS2BridgeThread:
    """Wrapper to run ROS2Bridge in a separate thread"""

    def __init__(self, state_callback: Optional[Callable] = None):
        self.state_callback = state_callback
        self.node: Optional[ROS2Bridge] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Start ROS2 bridge in background thread"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """Run ROS2 node in thread"""
        rclpy.init()
        self.node = ROS2Bridge(state_callback=self.state_callback)

        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        """Stop ROS2 bridge"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_state(self) -> dict:
        """Get current robot state"""
        if self.node:
            return self.node.get_state()
        return {}

    def set_initial_pose(self, x: float, y: float, yaw: float):
        """Set initial pose"""
        if self.node:
            self.node.set_initial_pose(x, y, yaw)

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        """Send navigation goal"""
        if self.node:
            return self.node.send_goal(x, y, yaw)
        return False

    def send_waypoints(self, waypoints: List[dict]) -> bool:
        """Send waypoints"""
        if self.node:
            return self.node.send_waypoints(waypoints)
        return False

    def cancel_goal(self):
        """Cancel navigation"""
        if self.node:
            self.node.cancel_goal()

    def send_cmd_vel(self, linear: float, angular: float):
        """Send velocity command"""
        if self.node:
            self.node.send_cmd_vel(linear, angular)
