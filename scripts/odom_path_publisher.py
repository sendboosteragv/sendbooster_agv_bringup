#!/usr/bin/python3
"""
/odom (raw wheel odom) 과 /odometry/filtered (IMU 보정 odom) 을
각각 nav_msgs/Path 로 변환하여 RViz2 에서 비교 시각화.

Published:
  /odom_path       (nav_msgs/Path) - 원시 휠 오도메트리 경로 (빨간색)
  /filtered_path   (nav_msgs/Path) - IMU 보정 후 경로 (초록색)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')

        self.declare_parameter('max_path_length', 5000)
        self.declare_parameter('min_dist', 0.01)
        max_len = self.get_parameter('max_path_length').value
        self.min_dist = self.get_parameter('min_dist').value

        self._raw_path = Path()
        self._raw_path.header.frame_id = 'odom'
        self._filtered_path = Path()
        self._filtered_path.header.frame_id = 'odom'

        self._max_len = max_len

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, '/odom', self._raw_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._filtered_cb, qos)

        self._raw_pub = self.create_publisher(Path, '/odom_path', 10)
        self._filtered_pub = self.create_publisher(Path, '/filtered_path', 10)

    def _should_append(self, path: Path, pose: PoseStamped) -> bool:
        if not path.poses:
            return True
        prev = path.poses[-1].pose.position
        cur = pose.pose.position
        dist = ((cur.x - prev.x) ** 2 + (cur.y - prev.y) ** 2) ** 0.5
        if dist > 1.0:
            path.poses[-1] = pose
            return False
        return dist >= self.min_dist

    def _append(self, path: Path, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        if not self._should_append(path, pose):
            return
        path.poses.append(pose)
        if len(path.poses) > self._max_len:
            path.poses = path.poses[-self._max_len:]

    def _raw_cb(self, msg: Odometry) -> None:
        self._append(self._raw_path, msg)
        self._raw_path.header.stamp = msg.header.stamp
        self._raw_pub.publish(self._raw_path)

    def _filtered_cb(self, msg: Odometry) -> None:
        self._append(self._filtered_path, msg)
        self._filtered_path.header.stamp = msg.header.stamp
        self._filtered_pub.publish(self._filtered_path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
