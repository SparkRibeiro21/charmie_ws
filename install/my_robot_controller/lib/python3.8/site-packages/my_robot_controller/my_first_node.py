#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

class TRNode(Node):

    def __init__(self):
        super().__init__("first_node")
        # self.get_logger().info("Hello from ROS2")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello World " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)

    
    # insert code here
    node = TRNode()

    rclpy.spin(node)


    rclpy.shutdown()

if __name__ == '__main__':
    main()