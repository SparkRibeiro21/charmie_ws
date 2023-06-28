#!/usr/bin/env python3
import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.node import Node

class DoorNode(Node):

    def __init__(self):
        super().__init__("DoorStart")
        self.get_logger().info("Initiliased Door Start Node")

        #LIDAR SUB
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.start_door_subscriber = self.create_subscription(Bool, 'start_door', self.start_door_callback, 10) 
        self.done_start_door_publisher = self.create_publisher(Bool, 'done_start_door', 10) 

        self.is_door_opened = False
        self.flag_detection = False
        
        self.create_timer(0.1, self.timer_callback)

    def lidar_callback(self, scan: LaserScan):
        
        middle_index = len(scan.ranges) // 2  # Índice do sensor do meio
        middle_values = scan.ranges[middle_index-10:middle_index+10]  # Leitura do sensor do meio
        middle_range = np.mean(middle_values)
        """ print("v1: ", scan.ranges[middle_index-1])
        print("v2: ", scan.ranges[middle_index])
        print("v3: ", scan.ranges[middle_index+1])"""
        print("media: ", middle_range)

        if middle_range <= 1:
            self.get_logger().info('Door state: Closed')
            #print('Door_state: ', door_state)
            self.is_door_opened = False
        else:
            self.get_logger().info('Door state: Open')
            #print('Door_state: ', door_state)
            self.is_door_opened = True
        
    def start_door_callback(self, state: Bool):
        print("Recebi Start Door")
        self.flag_detection = True

    def timer_callback(self):
        if self.flag_detection:
            if self.is_door_opened:
                door_state = Bool()
                door_state.data = self.is_door_opened
                self.done_start_door_publisher.publish(door_state)
                self.is_door_opened = False
                self.flag_detection = False
                print("Enviei Start Door")

def main(args=None):
    rclpy.init(args=args)
    node = DoorNode()
    rclpy.spin(node)
    rclpy.shutdown()

