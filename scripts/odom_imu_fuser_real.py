#!/usr/bin/env python3
"""
Odom + IMU fuser for real robot (EKF 대체).

motor_driver_node의 /odom에서 직선 속도(vx)를 가져오고,
IMU의 절대 yaw로 heading을 보정하여 /odometry/filtered를 발행.

IMU yaw는 자북 기준이므로 첫 메시지의 yaw를 offset으로 저장하여
odom 프레임(시작 시 0°)에 맞춤.

Input:
  /odom (nav_msgs/Odometry) - motor_driver의 휠 오도메트리
  /imu/data (sensor_msgs/Imu) - AHRS IMU

Output:
  /odometry/filtered (nav_msgs/Odometry) - IMU yaw 보정된 odom
  TF: odom → base_footprint
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomImuFuserReal(Node):
    def __init__(self):
        super().__init__('odom_imu_fuser')

        # State
        self.x = 0.0
        self.y = 0.0
        self.imu_yaw = None
        self.imu_yaw_offset = None  # 첫 IMU yaw (자북→odom 보정)
        self.imu_gyro_z = 0.0

        self.last_stamp = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher
        self.pub_filtered = self.create_publisher(Odometry, 'odometry/filtered', 50)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_cb, 50)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, 50)

        self.get_logger().info('Odom+IMU fuser (real robot) started')

    def imu_cb(self, msg):
        q = msg.orientation
        raw_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # 첫 IMU 메시지의 yaw를 offset으로 저장
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = raw_yaw
            self.get_logger().info(
                f'IMU yaw offset set: {math.degrees(raw_yaw):.1f} deg'
            )

        # Initial offset already compensates for IMU mounting direction
        self.imu_yaw = raw_yaw - self.imu_yaw_offset
        self.imu_gyro_z = msg.angular_velocity.z

    def odom_cb(self, msg):
        if self.imu_yaw is None:
            return

        vx = msg.twist.twist.linear.x

        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9

        if self.last_stamp is None:
            self.last_stamp = now
            return

        dt = now - self.last_stamp
        if dt <= 0.0 or dt > 1.0:
            self.last_stamp = now
            return
        self.last_stamp = now

        # IMU yaw를 heading으로 사용하여 위치 적분
        self.x += vx * math.cos(self.imu_yaw) * dt
        self.y += vx * math.sin(self.imu_yaw) * dt

        # Publish odometry
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y

        half_yaw = self.imu_yaw / 2.0
        out.pose.pose.orientation.z = math.sin(half_yaw)
        out.pose.pose.orientation.w = math.cos(half_yaw)

        out.twist.twist.linear.x = vx
        out.twist.twist.angular.z = self.imu_gyro_z

        self.pub_filtered.publish(out)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuFuserReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
