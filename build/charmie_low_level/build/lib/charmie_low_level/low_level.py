#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class LowLevelNode(Node):

    def __init__(self):
        super().__init__("Low_Level")
        self.get_logger().info("Initialised CHARMIE Low Level Node")


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelNode()
    rclpy.spin(node)
    rclpy.shutdown()
