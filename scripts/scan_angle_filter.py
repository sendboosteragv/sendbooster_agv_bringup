#!/usr/bin/env python3
"""
Scan angle filter: limits LaserScan to a specified angle range.

Normal mode (invert=false): keeps rays within [angle_min, angle_max].
Invert mode (invert=true):  discards rays within [angle_min, angle_max],
                             sets them to range_max (no obstacle).
                             Use this for axis-reversed LiDARs to remove
                             the robot-body-facing half.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanAngleFilter(Node):
    def __init__(self):
        super().__init__('scan_angle_filter')

        self.declare_parameter('angle_min', -math.pi / 2)
        self.declare_parameter('angle_max', math.pi / 2)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('invert', False)
        self.declare_parameter('max_scan_age', 1.0)

        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.invert = self.get_parameter('invert').value
        self.max_scan_age = self.get_parameter('max_scan_age').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self.sub = self.create_subscription(LaserScan, input_topic, self.callback, sub_qos)

        mode = 'INVERT' if self.invert else 'KEEP'
        self.get_logger().info(
            f'[{mode}] Filtering {input_topic} -> {output_topic} '
            f'[{math.degrees(self.angle_min):.0f}°, {math.degrees(self.angle_max):.0f}°]'
        )

    def callback(self, msg: LaserScan):
        # Discard scans older than 1 second to prevent TF lookup failures in costmap
        scan_time = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - scan_time).nanoseconds / 1e9
        if age > self.max_scan_age:
            return

        angle_min_idx = max(0, int((self.angle_min - msg.angle_min) / msg.angle_increment))
        angle_max_idx = min(len(msg.ranges), int((self.angle_max - msg.angle_min) / msg.angle_increment) + 1)

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        if self.invert:
            # Keep full scan structure, blank out the robot-body-facing zone
            ranges = list(msg.ranges)
            for i in range(angle_min_idx, angle_max_idx):
                ranges[i] = float('inf')
            filtered.angle_min = msg.angle_min
            filtered.angle_max = msg.angle_max
            filtered.ranges = ranges
            filtered.intensities = list(msg.intensities) if msg.intensities else []
        else:
            if angle_min_idx >= angle_max_idx:
                return
            filtered.angle_min = msg.angle_min + angle_min_idx * msg.angle_increment
            filtered.angle_max = msg.angle_min + (angle_max_idx - 1) * msg.angle_increment
            filtered.ranges = list(msg.ranges[angle_min_idx:angle_max_idx])
            filtered.intensities = list(msg.intensities[angle_min_idx:angle_max_idx]) if msg.intensities else []

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanAngleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
