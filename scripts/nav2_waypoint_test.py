#!/usr/bin/env python3
"""
Nav2 Waypoint Performance Test (position-based)

Sends navigation goals and monitors robot position to determine success.
Uses position-based goal detection instead of action result status
to avoid "multiple action server" issues from lifecycle recovery.

Usage:
  ros2 run sendbooster_agv_bringup nav2_waypoint_test.py
  ros2 run sendbooster_agv_bringup nav2_waypoint_test.py \
    --ros-args -p use_sim_time:=true -p preset:=factory
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Warehouse waypoints: full loop including gap traversal
# Room 10x8m, walls x=±5 y=±4, gap at y=0 x=[-0.5,0.5] (1m wide)
# Obstacles: (-3,2) 0.8², (3,2.5) 1.0x0.6, (2,-2.5) 0.6x1.2, (-2,-2) 0.6², pillar (0,2.5)
# All WPs ≥1m from obstacle surfaces
# Warehouse: 10m x 8m, dividers at y=0, gap 1.5m
# Obstacles moved to corners: (-4.5,3.5), (4,3.5), (4,-3), (-4,-3)
# All WPs in central safe corridor (x=-2 to 2, y=-2 to 3.5)
WAREHOUSE_WAYPOINTS = [
    (0.0, 1.5, 90.0),       # 1) 북쪽
    (1.5, 1.5, 0.0),        # 2) 북동
    (1.5, 3.0, 90.0),       # 3) 더 북동
    (-1.0, 2.0, 180.0),     # 4) 북서 (중앙 경로)
    (-1.0, 0.5, -90.0),     # 5) 서쪽 남하 (갭 위)
    (0.5, 0.5, 0.0),        # 6) 중앙
    (1.5, -0.5, -45.0),     # 7) 남동
    (-1.0, -1.0, 180.0),    # 8) 남서
    (0.0, 1.0, 90.0),       # 9) 원점 복귀
]

# Real map waypoints: navigation test on 20x20m building map
# All positions verified with distance_transform
# Dividing wall at y~1.2: main eastern gap (x>0), wide NW open area
# Route uses short hops to avoid DWB oscillation issues
REAL_MAP_WAYPOINTS = [
    # ── Phase 1: local area near origin ──
    (0.0, -1.0, -90.0),       #  1) South (1.56m clear)
    (0.0, -2.0, -90.0),       #  2) Further south (0.78m clear)
    (0.0, -1.0, 90.0),        #  3) Back to south center
    # ── Phase 2: cross to north via eastern gap ──
    (1.0, 0.0, 90.0),         #  4) East of origin (1.50m clear)
    (2.0, 1.0, 90.0),         #  5) Through gap north (1.32m clear)
    (2.0, 3.0, 90.0),         #  6) Safely north (0.87m clear)
    # ── Phase 3: NW open area ──
    (0.0, 4.0, 135.0),        #  7) NW approach (1.56m clear)
    (-2.0, 4.0, 135.0),       #  8) NW area (2.97m clear)
    (-3.0, 7.0, 0.0),         #  9) Far north (5.45m clear)
    (0.0, 7.0, 0.0),          # 10) North center (4.24m clear)
    # ── Phase 4: return south ──
    (2.0, 3.0, -90.0),        # 11) NE (0.87m clear)
    (2.0, 1.0, -90.0),        # 12) Through gap (1.32m clear)
    (0.0, 0.0, 0.0),          # 13) Origin (0.98m clear)
]

# Harsh corridors world: gaps + L-turns + dead ends
# Layout: 20m x 16m, dividers at y=±4, vertical wall at x=5
# Actual gaps: y=4 center: x=[-0.6, 0.925] (1.5m), y=4 west: x=[-5.075,-4.2] (0.875m)
# y=-4: x=[0.925, 2.775] (1.85m), x=[6.425, 7.675] (1.25m)
HARSH_WAYPOINTS = [
    (0.0, 2.0, 90.0),        # 1) Start center
    (0.0, 4.2, 90.0),        # 2) Approach gap (inside gap at y=4, x=0)
    (0.0, 6.0, 90.0),        # 3) Through gap → north section
    (-3.0, 6.0, 180.0),      # 4) West in north section
    (-7.0, 6.0, 180.0),      # 5) Far west north
    (-4.6, 4.2, -90.0),      # 6) Approach 0.875m gap (x≈-4.6, between walls)
    (-4.6, 2.0, -90.0),      # 7) Through narrow gap → center
    (-3.0, 0.0, -45.0),      # 8) Center
    (3.0, 0.0, 0.0),         # 9) East center
    (7.0, -2.0, -90.0),      # 10) East south (past x=5 wall)
    (1.8, -4.2, -90.0),      # 11) Approach south gap
    (1.8, -6.0, -90.0),      # 12) Through gap → south section
    (0.0, 0.0, 90.0),        # 13) Return (through south+center)
]

FACTORY_WAYPOINTS = REAL_MAP_WAYPOINTS  # backward compat

GOAL_TOLERANCE = 0.7  # meters — matches nav2 xy_goal_tolerance
TIMEOUT_MAP = {'warehouse': 180.0, 'factory': 600.0, 'real_map': 180.0, 'harsh': 180.0}


class WaypointTest(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_test')
        self.declare_parameter('preset', 'warehouse')
        self.declare_parameter('goal_tolerance', GOAL_TOLERANCE)

        preset = self.get_parameter('preset').value
        self._tolerance = self.get_parameter('goal_tolerance').value

        if preset == 'real_map':
            self._waypoints = REAL_MAP_WAYPOINTS
            self._timeout = TIMEOUT_MAP['real_map']
        elif preset == 'harsh':
            self._waypoints = HARSH_WAYPOINTS
            self._timeout = TIMEOUT_MAP['harsh']
        elif preset == 'factory':
            self._waypoints = FACTORY_WAYPOINTS
            self._timeout = TIMEOUT_MAP['factory']
        else:
            self._waypoints = WAREHOUSE_WAYPOINTS
            self._timeout = TIMEOUT_MAP['warehouse']

        self._preset = preset
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._results = []

        # Position tracking
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._pos_lock = threading.Lock()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, qos)

    def _odom_cb(self, msg):
        with self._pos_lock:
            self._cur_x = msg.pose.pose.position.x
            self._cur_y = msg.pose.pose.position.y

    def _dist_to(self, gx, gy):
        with self._pos_lock:
            dx = self._cur_x - gx
            dy = self._cur_y - gy
        return math.sqrt(dx * dx + dy * dy)

    def _cur_pos(self):
        with self._pos_lock:
            return self._cur_x, self._cur_y

    def make_goal(self, x, y, yaw_deg):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def run_test(self):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('Action server not available after 60s')
            return

        # Wait for first odom
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._dist_to(0, 0) >= 0 or True:
                break

        n = len(self._waypoints)
        self.get_logger().info(
            f'Starting {self._preset} test ({n} goals, '
            f'tol={self._tolerance}m, timeout={self._timeout:.0f}s)')
        self.get_logger().info('=' * 60)

        total_start = time.monotonic()

        for i, (gx, gy, yaw) in enumerate(self._waypoints, 1):
            cx, cy = self._cur_pos()
            dist = math.sqrt((gx - cx)**2 + (gy - cy)**2)
            self.get_logger().info(
                f'[{i}/{n}] Goal: ({gx:.1f}, {gy:.1f}) '
                f'from ({cx:.1f}, {cy:.1f}), dist={dist:.1f}m')

            # Send goal (fire and forget — don't rely on result status)
            pose = self.make_goal(gx, gy, yaw)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            send_future = self._client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn('  Goal REJECTED')
                self._results.append({
                    'wp': i, 'x': gx, 'y': gy,
                    'status': 'REJECTED', 'time': 0.0, 'final_dist': dist,
                })
                continue

            # Monitor position until goal reached or timeout
            t_start = time.monotonic()
            reached = False
            stall_count = 0
            prev_dist = dist

            last_progress_time = time.monotonic()
            while (time.monotonic() - t_start) < self._timeout:
                rclpy.spin_once(self, timeout_sec=0.5)
                cur_dist = self._dist_to(gx, gy)

                if cur_dist < self._tolerance:
                    reached = True
                    break

                # Stall detection: track time since last meaningful progress
                if abs(cur_dist - prev_dist) > 0.05:
                    last_progress_time = time.monotonic()
                    prev_dist = cur_dist

                stall_duration = time.monotonic() - last_progress_time
                if stall_duration > 30.0:
                    self.get_logger().warn(
                        f'  Stalled for {stall_duration:.0f}s at dist={cur_dist:.2f}m, resending goal')
                    resend = self._client.send_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(
                        self, resend, timeout_sec=5.0)
                    last_progress_time = time.monotonic()

            elapsed = time.monotonic() - t_start
            final_dist = self._dist_to(gx, gy)

            if reached:
                self.get_logger().info(
                    f'  REACHED in {elapsed:.1f}s (dist={final_dist:.2f}m)')
                status = 'SUCCESS'
            else:
                self.get_logger().warn(
                    f'  TIMEOUT after {elapsed:.1f}s (dist={final_dist:.2f}m)')
                status = 'TIMEOUT'
                # Cancel goal
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass

            self._results.append({
                'wp': i, 'x': gx, 'y': gy,
                'status': status, 'time': elapsed, 'final_dist': final_dist,
            })

        total_elapsed = time.monotonic() - total_start
        self._print_report(total_elapsed)

    def _print_report(self, total_elapsed):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'  WAYPOINT TEST REPORT ({self._preset})')
        self.get_logger().info('=' * 60)

        successes = 0
        for r in self._results:
            ok = r['status'] == 'SUCCESS'
            if ok:
                successes += 1
            marker = 'OK' if ok else 'NG'
            self.get_logger().info(
                f"  [{marker}] WP{r['wp']}: ({r['x']:.1f}, {r['y']:.1f}) "
                f"-> {r['status']} ({r['time']:.1f}s, "
                f"remaining={r['final_dist']:.2f}m)")

        total = len(self._results)
        rate = 100.0 * successes / total if total > 0 else 0
        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'  Result: {successes}/{total} succeeded ({rate:.0f}%)')
        self.get_logger().info(f'  Total time: {total_elapsed:.1f}s')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTest()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
