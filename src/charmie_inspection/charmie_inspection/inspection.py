#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class Charmie_inspection(Node):

    def __init__(self):
        super().__init__("Charmie_inspection")
        self.get_logger().info("Initialised CHARMIE Inspection Node")
        
  

def main(args=None):
    rclpy.init(args=args)
    node = Charmie_inspection()
    rclpy.spin(node)
    rclpy.shutdown()
