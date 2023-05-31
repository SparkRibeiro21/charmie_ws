#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Bool
import time

flag_get_neck_position = False 

class TRNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Node")
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback , 10)
        self.flag_neck_position_publisher = self.create_publisher(Bool, "flag_neck_pos", 10)

        self.counter = 1 # starts at 1 to avoid initial 
        self.create_timer(5, self.timer_callback)

    def get_neck_position_callback(self, pos: Pose2D):
        print("Received: pan =", int(pos.x), " tilt = ", int(pos.y))

    def timer_callback(self):
        cmd = Pose2D()
        flag = Bool()
        global flag_get_neck_position

        if self.counter == 0:
            cmd.x = 180.0
            cmd.y = 180.0 
            flag_get_neck_position = False
        if self.counter == 1:
            cmd.x = 270.0
            cmd.y = 180.0 
        if self.counter == 2:
            cmd.x = 180.0
            cmd.y = 180.0 
            flag_get_neck_position = True
        if self.counter == 3:
            cmd.x = 90.0
            cmd.y = 180.0 
        if self.counter == 4:
            cmd.x = 180.0
            cmd.y = 180.0 
            flag_get_neck_position = False
        if self.counter == 5:
            cmd.x = 180.0
            cmd.y = 120.0 
        if self.counter == 6:
            cmd.x = 180.0
            cmd.y = 180.0 
            flag_get_neck_position = True
        if self.counter == 7:
            cmd.x = 180.0
            cmd.y = 235.0 
            self.counter = -1

        self.neck_position_publisher.publish(cmd)
        flag.data = flag_get_neck_position
        self.flag_neck_position_publisher.publish(flag)
        print("DATA SENT ", self.counter)
        self.counter+=1

def main(args=None):
    rclpy.init(args=args)
    node = TRNode()

    cmd = Pose2D()
    cmd.x = 180.0
    cmd.y = 180.0
    node.neck_position_publisher.publish(cmd)
    print("INITIAL STATE")

    rclpy.spin(node)
    rclpy.shutdown()
