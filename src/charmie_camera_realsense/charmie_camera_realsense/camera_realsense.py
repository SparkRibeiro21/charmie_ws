#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CameraNode(Node):

    def __init__(self):
        super().__init__("Camera_RealSense")
        self.get_logger().info("Initialised CHARMIE RealSense Node")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()
