#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class LidarNode(Node):

    def __init__(self):
        super().__init__("Lidar")
        self.get_logger().info("Initialised CHARMIE LIDAR Node")

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
