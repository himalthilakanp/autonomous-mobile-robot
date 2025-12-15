#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055
import math


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)

        # Initialize I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

        # ROS2 timer for 20 Hz publishing
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"  # important for Cartographer

        # Read Euler angles
        heading, roll, pitch = self.sensor.euler
        if heading is None or roll is None or pitch is None:
            self.get_logger().warn("IMU data not ready")
            return

        # Convert to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        heading = math.radians(heading)

        # Compute quaternion
        cy = math.cos(heading * 0.5)
        sy = math.sin(heading * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quat = [
            cr * cp * cy + sr * sp * sy,  # w
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy   # z
        ]

        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]

        # Linear acceleration
        if self.sensor.acceleration:  # m/s²
            msg.linear_acceleration.x = self.sensor.acceleration[0]
            msg.linear_acceleration.y = self.sensor.acceleration[1]
            msg.linear_acceleration.z = self.sensor.acceleration[2]

        # Angular velocity
        if self.sensor.gyro:  # deg/s -> rad/s
            msg.angular_velocity.x = math.radians(self.sensor.gyro[0])
            msg.angular_velocity.y = math.radians(self.sensor.gyro[1])
            msg.angular_velocity.z = math.radians(self.sensor.gyro[2])

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
