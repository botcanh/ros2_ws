#!/usr/bin/env python3
"""
Differential-drive odometry node.

Subscribes:
  /encoder_ticks  (std_msgs/Int32MultiArray)  [left_ticks, right_ticks]

Publishes:
  /odom  (nav_msgs/Odometry)

Broadcasts TF:
  odom → base_link
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── Kinematics — fill in your actual robot values ─────────────────────────
TICKS_PER_REV  = 1440.0   # TODO: encoder CPR × gear ratio (same as ESP32)
WHEEL_RADIUS_M = 0.05    # TODO: metres
WHEEL_BASE_M   = 0.21     # TODO: metres, left-to-right wheel distance

# ── Frame IDs ─────────────────────────────────────────────────────────────
ODOM_FRAME      = 'odom'
BASE_LINK_FRAME = 'base_link'


def yaw_to_quat(yaw: float):
    """Convert a yaw angle (rad) to a (x, y, z, w) quaternion tuple."""
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Robot pose state
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        # Previous encoder tick counts
        self._prev_left  = None
        self._prev_right = None
        self._prev_time  = None

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Publisher
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Subscriber
        self.create_subscription(
            Int32MultiArray, 'encoder_ticks', self._encoder_cb, 10)

        self.get_logger().info('Odometry node started.')

    def _encoder_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return

        left_ticks  = msg.data[0]
        right_ticks = msg.data[1]
        now         = self.get_clock().now()

        # First message — just initialise reference counts
        if self._prev_left is None:
            self._prev_left  = left_ticks
            self._prev_right = right_ticks
            self._prev_time  = now
            return

        # ── Delta ticks → metres ───────────────────────────────────────────
        delta_left_m  = (left_ticks  - self._prev_left)  / TICKS_PER_REV * 2.0 * math.pi * WHEEL_RADIUS_M
        delta_right_m = (right_ticks - self._prev_right) / TICKS_PER_REV * 2.0 * math.pi * WHEEL_RADIUS_M

        self._prev_left  = left_ticks
        self._prev_right = right_ticks

        # ── Differential drive kinematics ─────────────────────────────────
        delta_dist  = (delta_left_m + delta_right_m) / 2.0
        delta_theta = (delta_right_m - delta_left_m) / WHEEL_BASE_M

        mid_theta = self._theta + delta_theta / 2.0
        self._x     += delta_dist * math.cos(mid_theta)
        self._y     += delta_dist * math.sin(mid_theta)
        self._theta += delta_theta

        # ── Velocity estimate ─────────────────────────────────────────────
        dt_s = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        vx = delta_dist  / dt_s if dt_s > 0 else 0.0
        wz = delta_theta / dt_s if dt_s > 0 else 0.0

        # ── Publish Odometry ──────────────────────────────────────────────
        qx, qy, qz, qw = yaw_to_quat(self._theta)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id  = BASE_LINK_FRAME

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz

        self._odom_pub.publish(odom)

        # ── Broadcast TF: odom → base_link ───────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id  = BASE_LINK_FRAME

        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.x    = qx
        tf.transform.rotation.y    = qy
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw

        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
