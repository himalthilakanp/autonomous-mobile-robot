import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("test_node")
    node.get_logger().info("my_nav2 package is working!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
