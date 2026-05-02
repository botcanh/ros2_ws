#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialSender(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Serial init
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info('✅ Serial connected to /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to serial: {e}')
            exit(1)

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        setpoint = int(linear_x * 10)  # Scale
        try:
            self.ser.write(f"{setpoint}\n".encode())
            self.get_logger().info(f"📤 Sent setpoint: {setpoint}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to write to serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
