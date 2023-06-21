#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("Receptionist")
        self.get_logger().info("Initiliased Receptionist Node")
        self.counter = 0

        self.start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.start_button_callback, 10)
        
        self.rgb_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        
        
        # self.counter_publisher = self.create_publisher(Int16, "talker", 10)
        # self.counter_subscriber = self.create_subscription(Int16, "talker", self.counter_callback, 10)

        self.create_timer(5.0, self.timer_callback)

        flag = Bool()
        flag.data = False
        self.start_button_publisher.publish(flag)

    def start_button_callback(self, state: Bool):
        print("O estado do start button é:", state.data)

    def timer_callback(self):
        var = Int16()
        neck = Pose2D()
        var.data= self.counter
        self.rgb_publisher.publish(var)
        
        if self.counter == 0:
            neck.x = 180.0
            neck.y = 180.0 
        if self.counter == 1:
            neck.x = 270.0
            neck.y = 180.0 
        if self.counter == 2:
            neck.x = 90.0
            neck.y = 180.0 
        if self.counter == 3:
            neck.x = 180.0
            neck.y = 120.0 

        self.neck_position_publisher.publish(neck)
        self.counter+=1
        if self.counter > 3:
            self.counter = 0
        
        
        
        # pass
        # num = Int16()
        # var = Int16()
        # num.data = 100
        # var.data= self.counter
        # self.counter_publisher.publish(var)
        # print("Publiquei:", var.data)
        # self.counter+=1

    # def counter_callback(self, num: Int16):
    #     print("Recebi:", num.data)



def main(args=None):
    rclpy.init(args=args)
    node = ReceptionistNode()
    rclpy.spin(node)
    rclpy.shutdown()