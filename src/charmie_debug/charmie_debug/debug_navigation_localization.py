#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Bool, Float32, Int16, String 
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL
from charmie_interfaces.srv import TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects
from sensor_msgs.msg import Image

import cv2 
import threading
import time
from cv_bridge import CvBridge

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Navigation and Localisation Node")

        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        # Variables
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.rgb_success = True
        self.rgb_message = ""

        self.nav_tar_sdnl = TarNavSDNL()

        self.flag_navigation_reached = False

    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: TestNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: TestNode):
        self.node = node
    

    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message

    # def track_object(self, object, wait_for_end_of=True):
    # 
    #     self.node.call_neck_track_object_server(object=object, wait_for_end_of=wait_for_end_of)
    #     
    #     if wait_for_end_of:
    #        while not self.node.waited_for_end_of_track_object:
    #         pass
    #     self.node.waited_for_end_of_track_object = False
    # 
    #     return self.node.track_object_success, self.node.track_object_message   

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7
        
        # VARS ...
        self.state = Waiting_for_start_button


        while True:

            if self.state == Waiting_for_start_button:
                # your code here ...


                self.node.nav_tar_sdnl.move_target_coordinates.x = 0.0
                self.node.nav_tar_sdnl.move_target_coordinates.y = 2.0
                self.node.nav_tar_sdnl.rotate_target_coordinates.x = 0.0
                self.node.nav_tar_sdnl.rotate_target_coordinates.y = 4.0
                self.node.nav_tar_sdnl.flag_not_obs = False
                self.node.nav_tar_sdnl.follow_me = False

                self.node.target_pos_publisher.publish(self.node.nav_tar_sdnl)


                while not self.node.flag_navigation_reached:
                    pass

                
                print("REACHED TARGET POSITION")
                                

                while True:
                    pass

                # next state
                # self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass