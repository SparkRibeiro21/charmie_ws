#!/usr/bin/env python3
import rclpy
from example_interfaces.msg import Bool
from rclpy.node import Node
from charmie_interfaces.msg import Obstacles

import math

class DoorNode(Node):

    def __init__(self):
        super().__init__("DoorStart")
        self.get_logger().info("Initiliased Door Start Node") 

        self.declare_parameter("max_door_angle"     , 18) # degrees
        self.declare_parameter("max_door_distance"  , 1.0) # meters

        # Obstacles 
        self.obstacles_subscriber = self.create_subscription(Obstacles, 'obs_lidar', self.obstacles_callback, 10)
        
        # Door Start
        self.start_door_subscriber = self.create_subscription(Bool, 'flag_door_start', self.start_door_callback, 10) 
        self.done_start_door_publisher = self.create_publisher(Bool, 'get_door_start', 10) 
        
        self.is_door_opened = False
        self.flag_detection = False
        self.obstacles = Obstacles()

        # max angle considered to be a door
        self.MAX_DOOR_ANGLE = math.radians(self.get_parameter("max_door_angle").value)
        # max distance to be considered a door 
        self.MAX_DOOR_DISTANCE = self.get_parameter("max_door_distance").value

        self.create_timer(0.1, self.timer_callback)

        self.door_start_diagnostic_publisher = self.create_publisher(Bool, "door_start_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.door_start_diagnostic_publisher.publish(flag_diagn)


    def obstacles_callback(self, obs:Obstacles):
        self.obstacles = obs

    def calculate_obstacles_lidar(self):
        # print("###")
        ctr = 0
        for obs in self.obstacles.obstacles:
            # if the robot detects any obstacle inside the max_angle with a dist under max_dist it considers the door is closed
            # the max distance was introduced since in some cases, there may be a sofa, 3 meters away in that direction...
            if -self.MAX_DOOR_ANGLE < obs.alfa < self.MAX_DOOR_ANGLE and obs.dist < self.MAX_DOOR_DISTANCE:
                ctr += 1
                print(math.degrees(obs.alfa), obs.dist)
        if ctr == 0:
            print("DOOR OPEN")
            self.is_door_opened = True
        else:
            print("DOOR CLOSED")
            self.is_door_opened = False

    def start_door_callback(self, state: Bool):
        print("Received Door Start:", state.data)
        self.flag_detection = state.data

    def timer_callback(self):
        if self.flag_detection:
            print(".")
            self.calculate_obstacles_lidar()
            if self.is_door_opened:
                door_state = Bool()
                door_state.data = self.is_door_opened
                self.done_start_door_publisher.publish(door_state)
                # self.is_door_opened = False
                # self.flag_detection = False
                print("Sent Door Start")


def main(args=None):
    rclpy.init(args=args)
    node = DoorNode()
    rclpy.spin(node)
    rclpy.shutdown()
