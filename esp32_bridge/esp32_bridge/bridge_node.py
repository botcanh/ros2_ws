#!/usr/bin/env python3
"""
Serial bridge between ROS2 and ESP32.

Serial protocol (115200 baud):
  ROS2 → ESP32:  "C:<left_rpm>:<right_rpm>\n"   e.g. C:45.0:-20.5
  ESP32 → ROS2:  "E:<left_ticks>:<right_ticks>\n" e.g. E:1234:-56

Subscriptions:
  /cmd_vel  (geometry_msgs/Twist)

Publications:
  /encoder_ticks  (std_msgs/Int32MultiArray)  [left_ticks, right_ticks]
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not found — run: pip install pyserial")


# ── Differential-drive kinematics ─────────────────────────────────────────
# TODO: replace with your actual robot measurements
WHEEL_RADIUS_M = 0.05   # metres, radius of each wheel
WHEEL_BASE_M   = 0.20   # metres, distance between left and right wheels
MAX_RPM        = 100.0  # absolute RPM limit sent to ESP32

# ── Serial port ───────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 115200

# ── Control loop ─────────────────────────────────────────────────────────
LOOP_HZ          = 20.0          # command send rate
CMD_VEL_TIMEOUT  = 0.5           # seconds before zeroing velocity


def twist_to_rpm(linear_x: float, angular_z: float):
    """Convert Twist cmd_vel to (left_rpm, right_rpm) using differential drive."""
    v = linear_x
    w = angular_z

    v_left  = v - w * WHEEL_BASE_M / 2.0
    v_right = v + w * WHEEL_BASE_M / 2.0

    rpm_left  = (v_left  / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    rpm_right = (v_right / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0

    rpm_left  = max(-MAX_RPM, min(MAX_RPM, rpm_left))
    rpm_right = max(-MAX_RPM, min(MAX_RPM, rpm_right))

    return rpm_left, rpm_right


class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Serial port
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial open: {SERIAL_PORT} @ {BAUD_RATE}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise SystemExit(1)

        # State
        self._target_rpm_left  = 0.0
        self._target_rpm_right = 0.0
        self._last_cmd_time    = self.get_clock().now()
        self._lock             = threading.Lock()

        # ROS2 interfaces
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self.enc_pub = self.create_publisher(
            Int32MultiArray, 'encoder_ticks', 10)

        # Timer: send commands at LOOP_HZ
        period = 1.0 / LOOP_HZ
        self.timer = self.create_timer(period, self._send_command)

        # Background thread: read encoder feedback
        self._read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

    # ── cmd_vel subscriber ────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        l, r = twist_to_rpm(msg.linear.x, msg.angular.z)
        with self._lock:
            self._target_rpm_left  = l
            self._target_rpm_right = r
            self._last_cmd_time    = self.get_clock().now()

    # ── Send timer ────────────────────────────────────────────────────────
    def _send_command(self):
        now     = self.get_clock().now()
        with self._lock:
            elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
            if elapsed > CMD_VEL_TIMEOUT:
                l, r = 0.0, 0.0
            else:
                l = self._target_rpm_left
                r = self._target_rpm_right

        cmd = f'C:{l:.1f}:{r:.1f}\n'
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Serial read thread ────────────────────────────────────────────────
    def _serial_read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors='ignore').strip()
                self.get_logger().info(f'[RAW] {line}')
                if not line.startswith('E:'):
                    continue

                parts = line[2:].split(':')
                if len(parts) != 2:
                    continue

                left_ticks  = int(parts[0])
                right_ticks = int(parts[1])

                msg = Int32MultiArray()
                msg.data = [left_ticks, right_ticks]
                self.enc_pub.publish(msg)

            except (serial.SerialException, ValueError) as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.1)

    # ── Cleanup ───────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.ser.is_open:
            # Stop motors before closing
            try:
                self.ser.write(b'C:0.0:0.0\n')
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
