#!/usr/bin/env python3
"""
Nav2 Goal Metrics Collector for MPPI Auto-Tuning

Sends navigation goals and collects metrics:
- Success rate, time to goal
- Path smoothness (angular velocity variance from /cmd_vel)
- CPU usage during navigation

Outputs a single JSON line to stdout for machine parsing.

Usage:
  ros2 run sendbooster_agv_bringup nav_goal_metrics_collector.py \
    --ros-args -p use_sim_time:=true -p quick:=false
"""

import json
import math
import sys
import time
import threading

import psutil
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

# Waypoints from real map (my_map)
FULL_WAYPOINTS = [
    (0.0, -1.0, -90.0),
    (0.0, -2.0, -90.0),
    (0.0, -1.0, 90.0),
    (1.0, 0.0, 90.0),
    (2.0, 1.0, 90.0),
    (2.0, 3.0, 90.0),
    (0.0, 4.0, 135.0),
    (-2.0, 4.0, 135.0),
    (-3.0, 7.0, 0.0),
    (0.0, 7.0, 0.0),
    (2.0, 3.0, -90.0),
    (2.0, 1.0, -90.0),
    (0.0, 0.0, 0.0),
]

QUICK_WAYPOINTS = [
    (0.0, -2.0, -90.0),
    (1.0, 0.0, 90.0),
    (2.0, 3.0, 90.0),
]

GOAL_TOLERANCE = 0.7
GOAL_TIMEOUT = 120.0
STALL_TIMEOUT = 30.0


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('nav_goal_metrics_collector')
        self.declare_parameter('quick', False)
        self.declare_parameter('goal_tolerance', GOAL_TOLERANCE)
        self.declare_parameter('goal_timeout', GOAL_TIMEOUT)

        quick = self.get_parameter('quick').value
        self._tolerance = self.get_parameter('goal_tolerance').value
        self._timeout = self.get_parameter('goal_timeout').value
        self._waypoints = QUICK_WAYPOINTS if quick else FULL_WAYPOINTS

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Position tracking
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._pos_lock = threading.Lock()

        # Path smoothness: collect angular velocities from cmd_vel
        self._angular_vels = []
        self._angular_lock = threading.Lock()

        # CPU monitoring
        self._cpu_samples = []
        self._cpu_monitoring = False
        self._cpu_thread = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

    def _odom_cb(self, msg):
        with self._pos_lock:
            self._cur_x = msg.pose.pose.position.x
            self._cur_y = msg.pose.pose.position.y

    def _cmd_vel_cb(self, msg):
        with self._angular_lock:
            self._angular_vels.append(msg.angular.z)

    def _dist_to(self, gx, gy):
        with self._pos_lock:
            dx = self._cur_x - gx
            dy = self._cur_y - gy
        return math.sqrt(dx * dx + dy * dy)

    def _start_cpu_monitor(self):
        self._cpu_samples = []
        self._cpu_monitoring = True
        self._cpu_thread = threading.Thread(target=self._cpu_monitor_loop, daemon=True)
        self._cpu_thread.start()

    def _stop_cpu_monitor(self):
        self._cpu_monitoring = False
        if self._cpu_thread:
            self._cpu_thread.join(timeout=3)

    def _find_nav_pids(self):
        """Find PIDs of Nav2-related processes (AMCL, EKF, Nav2 nodes)."""
        nav_names = [
            'amcl', 'ekf_node', 'controller_server', 'planner_server',
            'behavior_server', 'bt_navigator', 'map_server',
            'costmap_filter_info_server', 'scan_processor',
        ]
        pids = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info.get('cmdline') or [])
                for name in nav_names:
                    if name in cmdline and 'gzserver' not in cmdline:
                        pids.append(proc.info['pid'])
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return pids

    def _cpu_monitor_loop(self):
        """Monitor CPU usage of Nav2 processes only (excludes Gazebo)."""
        num_cores = psutil.cpu_count()
        # Prime psutil
        nav_procs = []
        for pid in self._find_nav_pids():
            try:
                p = psutil.Process(pid)
                p.cpu_percent(interval=None)
                nav_procs.append(p)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        while self._cpu_monitoring:
            time.sleep(1.0)
            total_cpu = 0.0
            alive_procs = []
            for p in nav_procs:
                try:
                    total_cpu += p.cpu_percent(interval=None)
                    alive_procs.append(p)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            nav_procs = alive_procs

            # Refresh process list periodically
            if len(nav_procs) < 3:
                for pid in self._find_nav_pids():
                    if pid not in [p.pid for p in nav_procs]:
                        try:
                            p = psutil.Process(pid)
                            p.cpu_percent(interval=None)
                            nav_procs.append(p)
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass

            # Normalize to percentage of total system (like top shows)
            nav_cpu_pct = total_cpu / num_cores
            self._cpu_samples.append(nav_cpu_pct)

    def _compute_smoothness(self):
        with self._angular_lock:
            vels = list(self._angular_vels)
        if len(vels) < 2:
            return 1.0
        mean = sum(vels) / len(vels)
        variance = sum((v - mean) ** 2 for v in vels) / len(vels)
        # Normalize: lower variance = smoother. Map to 0-1 range.
        return 1.0 / (1.0 + variance)

    def run(self):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=120.0):
            self._output_error('Action server not available after 120s')
            return

        # Wait for first odom
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)

        time.sleep(3.0)  # AMCL convergence

        self._start_cpu_monitor()

        results = []
        n = len(self._waypoints)
        self.get_logger().info(f'Running {n} waypoint goals...')

        for i, (gx, gy, yaw) in enumerate(self._waypoints, 1):
            with self._angular_lock:
                self._angular_vels.clear()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._make_goal(gx, gy, yaw)

            send_future = self._client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                results.append({
                    'wp': i, 'status': 'REJECTED', 'time': 0.0,
                    'smoothness': 0.0, 'final_dist': self._dist_to(gx, gy),
                })
                continue

            t_start = time.monotonic()
            prev_dist = self._dist_to(gx, gy)
            last_progress = time.monotonic()
            reached = False

            while (time.monotonic() - t_start) < self._timeout:
                rclpy.spin_once(self, timeout_sec=0.5)
                cur_dist = self._dist_to(gx, gy)

                if cur_dist < self._tolerance:
                    reached = True
                    break

                if abs(cur_dist - prev_dist) > 0.05:
                    last_progress = time.monotonic()
                    prev_dist = cur_dist

                if (time.monotonic() - last_progress) > STALL_TIMEOUT:
                    break

            elapsed = time.monotonic() - t_start
            smoothness = self._compute_smoothness()

            status = 'SUCCESS' if reached else 'TIMEOUT'
            if not reached:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass

            self.get_logger().info(
                f'  [{i}/{n}] ({gx:.1f},{gy:.1f}) -> {status} {elapsed:.1f}s')

            results.append({
                'wp': i, 'status': status, 'time': elapsed,
                'smoothness': smoothness, 'final_dist': self._dist_to(gx, gy),
            })

        self._stop_cpu_monitor()

        # Compile metrics
        succeeded = [r for r in results if r['status'] == 'SUCCESS']
        total = len(results)
        success_count = len(succeeded)
        success_rate = success_count / total if total > 0 else 0.0
        avg_time = (sum(r['time'] for r in succeeded) / success_count
                    if success_count > 0 else self._timeout)
        avg_smoothness = (sum(r['smoothness'] for r in succeeded) / success_count
                          if success_count > 0 else 0.0)
        cpu_avg = sum(self._cpu_samples) / len(self._cpu_samples) if self._cpu_samples else 0.0
        cpu_max = max(self._cpu_samples) if self._cpu_samples else 0.0

        output = {
            'goals_total': total,
            'goals_succeeded': success_count,
            'success_rate': round(success_rate, 4),
            'avg_time_to_goal': round(avg_time, 2),
            'avg_path_smoothness': round(avg_smoothness, 4),
            'cpu_avg_percent': round(cpu_avg, 2),
            'cpu_max_percent': round(cpu_max, 2),
            'per_goal': results,
        }

        # Output JSON to stdout for machine parsing
        print(f'METRICS_JSON:{json.dumps(output)}')

    def _make_goal(self, x, y, yaw_deg):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _output_error(self, msg):
        output = {
            'goals_total': 0, 'goals_succeeded': 0, 'success_rate': 0.0,
            'avg_time_to_goal': 999.0, 'avg_path_smoothness': 0.0,
            'cpu_avg_percent': 0.0, 'cpu_max_percent': 0.0,
            'error': msg, 'per_goal': [],
        }
        print(f'METRICS_JSON:{json.dumps(output)}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
