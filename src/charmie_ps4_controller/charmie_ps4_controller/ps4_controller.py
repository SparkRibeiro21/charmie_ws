#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ControllerNode(Node):

    def __init__(self):
        super().__init__("PS4_Controller")
        self.get_logger().info("Initialised CHARMIE PS4 Controller Node")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
