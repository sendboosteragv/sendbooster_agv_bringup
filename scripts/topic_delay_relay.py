#!/usr/bin/env python3
"""
Topic Delay Relay — Simulates hardware communication delays

Subscribes to sensor topics, buffers messages, and republishes
after a configurable delay. Simulates:
  - Serial communication delay (motor driver RS485 ~20-50ms)
  - IMU polling delay (~5-15ms jitter)
  - LiDAR scan buffering delay (~30-100ms)

Topics:
  /odom           → /odom_delayed        (default 30ms delay)
  /imu/data       → /imu/data_delayed    (default 15ms delay)
  /scan_raw_front → /scan_raw_front_delayed (default 50ms delay)
  /scan_raw_back  → /scan_raw_back_delayed  (default 50ms delay)

Usage:
  ros2 run sendbooster_agv_bringup topic_delay_relay.py
  ros2 run sendbooster_agv_bringup topic_delay_relay.py \\
    --ros-args -p odom_delay_ms:=50 -p scan_delay_ms:=100
"""

import random
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


class DelayedRelay:
    """Buffers messages and releases them after a delay."""

    def __init__(self, delay_sec, jitter_sec, publisher):
        self.delay_sec = delay_sec
        self.jitter_sec = jitter_sec
        self.publisher = publisher
        self._queue = deque()
        self._lock = threading.Lock()

    def enqueue(self, msg):
        delay = self.delay_sec + random.uniform(-self.jitter_sec, self.jitter_sec)
        delay = max(0.0, delay)
        release_time = time.monotonic() + delay
        with self._lock:
            self._queue.append((release_time, msg))

    def flush(self):
        now = time.monotonic()
        with self._lock:
            while self._queue and self._queue[0][0] <= now:
                _, msg = self._queue.popleft()
                self.publisher.publish(msg)


class TopicDelayRelay(Node):
    def __init__(self):
        super().__init__('topic_delay_relay')

        # Delay parameters (milliseconds)
        self.declare_parameter('odom_delay_ms', 30.0)
        self.declare_parameter('odom_jitter_ms', 10.0)
        self.declare_parameter('imu_delay_ms', 15.0)
        self.declare_parameter('imu_jitter_ms', 5.0)
        self.declare_parameter('scan_delay_ms', 50.0)
        self.declare_parameter('scan_jitter_ms', 20.0)
        self.declare_parameter('enabled', True)

        odom_delay = self.get_parameter('odom_delay_ms').value / 1000.0
        odom_jitter = self.get_parameter('odom_jitter_ms').value / 1000.0
        imu_delay = self.get_parameter('imu_delay_ms').value / 1000.0
        imu_jitter = self.get_parameter('imu_jitter_ms').value / 1000.0
        scan_delay = self.get_parameter('scan_delay_ms').value / 1000.0
        scan_jitter = self.get_parameter('scan_jitter_ms').value / 1000.0

        # Publishers (delayed output)
        odom_pub = self.create_publisher(Odometry, 'odom_delayed', 10)
        imu_pub = self.create_publisher(Imu, 'imu/data_delayed', 10)
        scan_front_pub = self.create_publisher(
            LaserScan, 'scan_raw_front_delayed', 10)
        scan_back_pub = self.create_publisher(
            LaserScan, 'scan_raw_back_delayed', 10)

        # Delay buffers
        self._odom_relay = DelayedRelay(odom_delay, odom_jitter, odom_pub)
        self._imu_relay = DelayedRelay(imu_delay, imu_jitter, imu_pub)
        self._scan_front_relay = DelayedRelay(
            scan_delay, scan_jitter, scan_front_pub)
        self._scan_back_relay = DelayedRelay(
            scan_delay, scan_jitter, scan_back_pub)

        # Subscribers (raw input from Gazebo)
        self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(
            Imu, 'imu/data', self._imu_cb, 10)
        self.create_subscription(
            LaserScan, 'scan_raw_front', self._scan_front_cb, 10)
        self.create_subscription(
            LaserScan, 'scan_raw_back', self._scan_back_cb, 10)

        # Flush timer (1ms resolution)
        self.create_timer(0.001, self._flush_all)

        self.get_logger().info(
            f'TopicDelayRelay: odom={odom_delay*1000:.0f}ms±{odom_jitter*1000:.0f}, '
            f'imu={imu_delay*1000:.0f}ms±{imu_jitter*1000:.0f}, '
            f'scan={scan_delay*1000:.0f}ms±{scan_jitter*1000:.0f}')

    def _odom_cb(self, msg):
        self._odom_relay.enqueue(msg)

    def _imu_cb(self, msg):
        self._imu_relay.enqueue(msg)

    def _scan_front_cb(self, msg):
        self._scan_front_relay.enqueue(msg)

    def _scan_back_cb(self, msg):
        self._scan_back_relay.enqueue(msg)

    def _flush_all(self):
        self._odom_relay.flush()
        self._imu_relay.flush()
        self._scan_front_relay.flush()
        self._scan_back_relay.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TopicDelayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
