#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from charmie_interfaces.msg import TarNavSDNL

class NavigationSDNLClass:

    def __init__(self):
        print("Executing Main Code")


class NavSDNLNode(Node):

    def __init__(self):
        super().__init__("NavigationSDNL")
        self.get_logger().info("Initialised CHARMIE NavigationSDNL Node")

        # Create Code Class Instance
        self.nav = NavigationSDNLClass() 

        # Create PUBs/SUBs
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        # Create Timers
        self.create_timer(1, self.timer_callback)
        
    def target_pos_callback(self, flag:TarNavSDNL):
        pass

    def timer_callback(self):
        aux = TarNavSDNL()
        print(aux)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NavSDNLNode()
    rclpy.spin(node)
    rclpy.shutdown()
