#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class AudioNode(Node):

    def __init__(self):
        super().__init__("Audio")
        self.get_logger().info("Initialised CHARMIE Audio Node")

def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    rclpy.shutdown()
