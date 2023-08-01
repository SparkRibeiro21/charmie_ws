#!/usr/bin/env python3
import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int16
from rclpy.node import Node
from charmie_interfaces.msg import Obstacles, ObstacleInfo

import math

class DoorNode(Node):

    def __init__(self):
        super().__init__("DoorStart")
        self.get_logger().info("Initiliased Door Start Node")


        self.obstacles_subscriber = self.create_subscription(Obstacles, 'obs_lidar', self.obstacles_callback, 10)
        self.start_door_subscriber = self.create_subscription(Bool, 'start_door', self.start_door_callback, 10) 
        self.done_start_door_publisher = self.create_publisher(Bool, 'done_start_door', 10) 
        
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        self.is_door_opened = False
        self.flag_detection = False
        self.obstacles = Obstacles()
        self.max_angle_for_door = math.radians(18) # degrees
        self.max_dist_for_door = 1.5 # meters

        self.color = Int16()

        
        self.create_timer(0.1, self.timer_callback)

        self.door_start_diagnostic_publisher = self.create_publisher(Bool, "door_start_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.door_start_diagnostic_publisher.publish(flag_diagn)


    def obstacles_callback(self, obs:Obstacles):
        self.obstacles = obs


    def calculate_obstacles_lidar(self):
        print("###")
        ctr = 0
        for obs in self.obstacles.obstacles:
            # if the robot detects any obstacle inside the max_angle with a dist under max_dist it considers the door is closed
            # the max distance was introduced since in some cases, there may be a sofa, 3 meters away in that direction...
            if -self.max_angle_for_door < obs.alfa < self.max_angle_for_door and obs.dist < self.max_dist_for_door:
                ctr += 1
                print(obs.alfa, obs.dist)
        if ctr == 0:
            print("OPEN")
            self.is_door_opened = True
        else:
            print("CLOSED")
            self.is_door_opened = False


    def start_door_callback(self, state: Bool):
        print("Recebi Start Door")
        self.flag_detection = True
        
        
        self.color.data = 21
        self.rgb_mode_publisher.publish(self.color)


    def timer_callback(self):
        if self.flag_detection:
            self.calculate_obstacles_lidar()
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