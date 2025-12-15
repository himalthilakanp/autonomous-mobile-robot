#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            this_callback := self.cmd_vel_callback,
            10
        )

        # Try opening all likely ESP32 ports
        self.arduino = None
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']

        for port in possible_ports:
            try:
                self.arduino = serial.Serial(port, 115200, timeout=0.05)
                self.get_logger().info(f"✅ Connected to ESP32 on {port}")
                break
            except:
                continue

        if self.arduino is None:
            self.get_logger().error("❌ No ESP32 found on /dev/ttyUSB* or /dev/ttyACM*")
            raise SystemExit

        self.get_logger().info("Serial Bridge Started")

    def cmd_vel_callback(self, msg):
        # Convert Twist to single-character motor command
        cmd = 'S'

        if msg.linear.x > 0.01:
            cmd = 'F'   # Forward
        elif msg.linear.x < -0.01:
            cmd = 'B'   # Backward
        elif msg.angular.z > 0.01:
            cmd = 'L'   # Left turn
        elif msg.angular.z < -0.01:
            cmd = 'R'   # Right turn
        else:
            cmd = 'S'   # Stop

        try:
            self.arduino.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
