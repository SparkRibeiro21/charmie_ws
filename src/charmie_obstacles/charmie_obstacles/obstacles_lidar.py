#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
from charmie_interfaces.msg import ObstacleInfo, Obstacles

class RobotClass:

    def __init__(self):
        print("Executing Main Code")


class RobotNode(Node):

    def __init__(self):
        super().__init__("Robot")
        self.get_logger().info("Initialised CHARMIE Robot Node")

        # Create Code Class Instance
        robot = RobotClass() 

        # Create PUBs/SUBs
        self.start_button_publisher = self.create_publisher(Obstacles, "get_start_button", 10)
        self.flag_start_button_subscriber = self.create_subscription(Bool, "flag_start_button", self.flag_start_button_callback , 10)

        # Create Timers
        self.create_timer(1, self.timer_callback)
        
    def flag_start_button_callback(self, flag:Bool):
        pass

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
