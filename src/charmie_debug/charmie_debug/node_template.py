#!/usr/bin/env python3

# initial instructions:
# this is an example of the layout code for a node in our workspace
# It is being used Face as an example, so when changing the code for a specific task Ctrl+F face
# and replace everything fot the name of your node 

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech
from charmie_interfaces.srv import SpeechCommand

from geometry_msgs.msg import Point
from xarm_msgs.msg import RobotMsg

from math import cos, sin
import math
import numpy as np

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class Face():
    def __init__(self):
        # initialised face class
        # all non ros2 functions that have to do with the face must be inside this class
        print("New Face Class Initialised")


class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        # Initialise face class
        self.face = Face()

        ### Topics (Publisher and Subscribers) ###   
        # Low Level   
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)    
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)

        self.arm_values_callback_subscriber = self.create_subscription(RobotMsg, "xarm/robot_states", self.get_arm_values_callback, 10)
        
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # while not self.speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.flag_navigation_done = False

        self.end_effector = Point()
        self.base_arm = Point()
        self.goal_point = Point()
        self.base_arm_offset = Point()

    # Subscriber to get the start button status     
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

    def get_arm_values_callback(self, Robot_msg: RobotMsg):
        self.current_pose = Robot_msg.pose
        #print(self.current_pose )
        self.current_position = np.array([[self.current_pose[0]],
                                         [self.current_pose[1]],
                                         [self.current_pose[2]]])
        #print(self.current_position)
        self.roll = self.current_pose[3]
        self.pitch = self.current_pose[4]
        self.yaw = self.current_pose[5]
        self.calculate_transformations()

    def deg_to_rad(self, deg):
        rad = deg * math.pi / 180
        return rad

    def calculate_transformations(self):
        self.rotation_matrix = np.array([[cos(self.deg_to_rad(self.yaw)) * cos(self.deg_to_rad(self.roll)), -cos(self.deg_to_rad(self.yaw)) * sin(self.deg_to_rad(self.roll)), sin(self.deg_to_rad(self.yaw))],
                                        [cos(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.roll)) + cos(self.deg_to_rad(self.roll)) * sin(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.yaw)),
                                        cos(self.deg_to_rad(self.pitch)) * cos(self.deg_to_rad(self.roll)) - sin(self.deg_to_rad(self.roll)) * sin(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.yaw)),
                                        -cos(self.deg_to_rad(self.yaw)) * sin(self.deg_to_rad(self.pitch))],
                                        [sin(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.roll)) - cos(self.deg_to_rad(self.roll)) * cos(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.yaw)),
                                        cos(self.deg_to_rad(self.roll)) * sin(self.deg_to_rad(self.pitch)) + sin(self.deg_to_rad(self.roll)) * cos(self.deg_to_rad(self.pitch)) * sin(self.deg_to_rad(self.yaw)),
                                        cos(self.deg_to_rad(self.yaw)) * cos(self.deg_to_rad(self.pitch))]])
        #print(self.rotation_matrix)
        #self.goal_point.x = 0.0
        #self.goal_point.y = 0.85
        #self.goal_point.z = 0.8
        self.goal_point = np.array([[0.0],
                                    [0.85],
                                    [0.8]])
        #self.base_arm_offset.x = -70.0
        #self.base_arm_offset.y = -60.0
        #self.base_arm_offset.z = 1100.0
        self.base_arm_offset = np.array([[-70.0],
                                         [-60.0],
                                         [1100.0]])
        self.base_arm = self.base_arm_offset
        #print(self.base_arm)

        #print(np.dot(self.rotation_matrix, self.base_arm))
        T_ab = self.goal_point
        print("Objeto - base robot", T_ab)
        T_bc = self.base_arm_offset
        print("base robot - base braço",T_bc)
        T_cd = np.dot(self.rotation_matrix, self.base_arm) + self.current_position
        print("base braço - end effector", T_cd)


        print('\n\n\n')

        print("Shape of T_ab:", T_ab.shape)
        print("Shape of T_bc:", T_bc.shape)
        print("Shape of T_cd:", T_cd.shape)

        T_ad = np.dot(np.dot(T_ab, T_bc), T_cd)
        #T_ad = T_bc * T_cd

        print('Transformação final - ',T_ad)


        



def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
