#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select, time

class KeyboardMotorControl(Node):
    def __init__(self):
        super().__init__('keyboard_motor_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.15
        self.turn = 0.4
        self.last_key_time = time.time()

        self.get_logger().info("Keyboard Teleop Ready (Arrow Keys). Press CTRL+C to stop.")

    def get_key_nonblocking(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                key = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return key
        return None

    def run(self):
        twist = Twist()

        while rclpy.ok():
            key = self.get_key_nonblocking()

            if key:
                self.last_key_time = time.time()

                if key == '\x1b[A':     # UP
                    twist.linear.x = self.speed
                    twist.angular.z = 0.0
                elif key == '\x1b[B':   # DOWN
                    twist.linear.x = -self.speed
                    twist.angular.z = 0.0
                elif key == '\x1b[C':   # RIGHT
                    twist.linear.x = 0.0
                    twist.angular.z = -self.turn
                elif key == '\x1b[D':   # LEFT
                    twist.linear.x = 0.0
                    twist.angular.z = self.turn
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)

            # Auto send STOP if no input for 100ms
            if time.time() - self.last_key_time > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMotorControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
