#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Bool, Int16
# import time

class TRNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Node")
        
        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback , 10)
        self.flag_neck_position_publisher = self.create_publisher(Bool, "flag_neck_pos", 10)
        
        # Low Level Topics
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback ,10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        
        self.counter = 1 # starts at 1 to avoid initial 
        self.create_timer(4, self.timer_callback)

        self.flag_get_neck_position = False 
        self.flag_get_start_button = False 

    def get_neck_position_callback(self, pos: Pose2D):
        print("Received Neck Position: pan =", int(pos.x), " tilt = ", int(pos.y))

    def get_start_button_callback(self, state: Bool):
        print("Received Start Button: ", state.data)

    def timer_callback(self):
        cmd = Pose2D()
        flag_neck = Bool()
        flag_start_button = Bool()
        mode = Int16()

        if self.counter == 0:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = False
            self.flag_get_start_button = False
            mode.data = 1
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 1:
            cmd.x = 270.0
            cmd.y = 180.0 
        if self.counter == 2:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = True
            self.flag_get_start_button = True
            mode.data = 2
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 3:
            cmd.x = 90.0
            cmd.y = 180.0 
        if self.counter == 4:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = False
            self.flag_get_start_button = False
            mode.data = 3
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 5:
            cmd.x = 180.0
            cmd.y = 120.0 
        if self.counter == 6:
            cmd.x = 180.0
            cmd.y = 180.0 
            self.flag_get_neck_position = True
            self.flag_get_start_button = True
            mode.data = 4
            self.rgb_mode_publisher.publish(mode)
        if self.counter == 7:
            cmd.x = 180.0
            cmd.y = 235.0 
            self.counter = -1

        self.neck_position_publisher.publish(cmd)

        flag_neck.data = self.flag_get_neck_position
        self.flag_neck_position_publisher.publish(flag_neck)

        flag_start_button.data = self.flag_get_start_button
        self.flag_start_button_publisher.publish(flag_start_button)


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
