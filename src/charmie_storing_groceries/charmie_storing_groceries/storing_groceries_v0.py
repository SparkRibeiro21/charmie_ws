#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from charmie_interfaces.srv import SpeechCommand, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, ArmTrigger, ActivateYoloObjects, NavTrigger, SetFace
from charmie_interfaces.msg import Yolov8Objects, DetectedObject, TarNavSDNL, ListOfDetectedObject, Obstacles, ArmController
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import json

from pathlib import Path
from datetime import datetime
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('agg')
from collections import defaultdict, Counter

import os

import time

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

""" object_name_mapping = {
    'Sponge': 'Sponge', 'Cleanser': 'Cleanser', 'Dishwasher Tab': 'Dishwasher Tab', 'Bag': 'Bag', 'Red Wine': 'Red Wine', 'Juice Pack': 'Juice Pack', 'Cola': 'Cola', 'Tropical Juice': 'Tropical Juice',
    'Milk': 'Milk', 'Iced Tea': 'Iced Tea', 'Orange Juice': 'Orange Juice', 'Seven Up': 'Seven Up', 'Water': 'Water', 'Tuna': 'Tuna', 'Tomato Soup': 'Tomato Soup',
    'Spam': 'Spam', 'Mustard': 'Mustard', 'Strawberry Jello': 'Strawberry Jello', 'Chocolate Jello': 'Chocolate Jello', 'Coffee Grounds': 'Coffee Grounds', 'Sugar': 'Sugar',
    'Pear': 'Pear', 'Plum': 'Plum', 'Peach': 'Peach', 'Lemon': 'Lemon', 'Orange': 'Orange', 'Strawberry': 'Strawberry', 'Banana': 'Banana', 'Apple': 'Apple', 'Tennis Ball': 'Tennis Ball', 
    'Soccer Ball': 'Soccer Ball', 'Rubiks Cube': 'Rubiks Cube', 'Dice': 'Dice', 'Baseball': 'Baseball', 'Pringles': 'Pringles', 'Cornflakes': 'Cornflakes', 'Cheezit': 'Cheezit',
    'Spoon': 'Spoon', 'Plate': 'Plate', 'Cup': 'Cup', 'Fork': 'Fork', 'Bowl': 'Bowl', 'Knife': 'Knife'
}

object_class_mapping = {
    'Cleaning Supplies': 'Cleaning Supplies', 'Drinks': 'Drinks', 'Foods': 'Foods', 'Fruits': 'Fruits', 'Toys': 'Toys', 'Snacks': 'Snacks', 'Dishes': 'Dishes'
} """

object_position_mapping = {
    ('First', 'Right'): 'First_shelf_rs',
    ('First', 'Left'): 'First_shelf_ls',
    ('Second', 'Right'): 'Second_shelf_rs',
    ('Second', 'Left'): 'Second_shelf_ls',
    ('Third', 'Right'): 'Third_shelf_rs',
    ('Third', 'Left'): 'Third_shelf_ls',
    ('Fourth', 'Right'): 'Fourth_shelf_rs',
    ('Fourth', 'Left'): 'Fourth_shelf_ls',
    ('Fifth', 'Right'): 'Fifth_shelf_rs',
    ('Fifth', 'Left'): 'Fifth_shelf_ls',
    ('Sixth', 'Right'): 'Sixth_shelf_rs',
    ('Sixth', 'Left'): 'Sixth_shelf_ls'
}
class StoringGroceriesNode(Node):

    def __init__(self):
        super().__init__("StoringGroceries")
        self.get_logger().info("Initialised CHARMIE StoringGroceries Node")

        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        self.home = str(Path.home())
        self.midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+self.midpath_configuration_files+'/'

        ### Topics (Publisher and Subscribers) ###  
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        # Neck
        # self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        self.neck_to_coords_publisher = self.create_publisher(Pose2D, "neck_to_coords", 10)

        # Arm 
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        
        #DEBUG
        self.detected_objects_publisher = self.create_publisher(DetectedObject, "objects_detected_sg", 10)
        self.detected_objects_subscriber = self.create_subscription(DetectedObject, "objects_detected_sg", self.detected_objects_callback, 10)

        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        
        # Search for person and object 
        self.search_for_object_detections_publisher = self.create_publisher(ListOfDetectedObject, "search_for_object_detections", 10)
        # Obstacles
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obstacles_callback, 10)
        
        ### Services (Clients) ###
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # Objects detected
        # self.objects_filtered_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered', self.get_objects_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered_hand', self.object_detected_filtered_hand_callback, 10)
        self.doors_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "doors_detected_filtered", self.doors_detected_filtered_callback, 10)
        self.doors_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'doors_detected_filtered_hand', self.doors_detected_filtered_hand_callback, 10)
        self.shoes_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "shoes_detected_filtered", self.shoes_detected_filtered_callback, 10)
        self.shoes_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'shoes_detected_filtered_hand', self.shoes_detected_filtered_hand_callback, 10)
        
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")

        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")

        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
        # Navigation
        # while not self.nav_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        # # Speakers
        # while not self.speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Speech Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        # # Face
        # while not self.face_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Face Command...")
        # # Arm (CHARMIE)
        """ while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...") """
        
        try:
            with open(self.complete_path_configuration_files + 'objects_lar.json', encoding='utf-8') as json_file:
                self.objects_file = json.load(json_file)
                # print(self.objects_file)
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")

        # Variables
        self.start_button_state = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_face = False
        
        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.face_success = True
        self.face_message = ""
        self.navigation_success = True
        self.navigation_message = ""
        self.flag_navigation_reached = False

        self.get_neck_position = [1.0, 1.0]
        self.objects_classNames_dict = {}

        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        
        self.objects_classNames_dict = {item["name"]: item["class"] for item in self.objects_file}
        self.detected_objects = Yolov8Objects()
        self.obstacles = Obstacles()
        self.filtered_objects_storing_groceries = []
        self.flag_storing_groceries_received = False
        self.br = CvBridge()
        #print(self.objects_classNames_dict)
        
    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

    def object_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_objects_hand = det_object

    def doors_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_doors = det_object

    def doors_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_doors_hand = det_object

    def shoes_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_shoes = det_object

    def shoes_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_shoes_hand = det_object
    
    def detected_objects_callback(self, det_object: DetectedObject):
        self.filtered_objects_storing_groceries.append(det_object)
        self.flag_storing_groceries_received = True
        
    # def get_objects_callback(self, objects: Yolov8Objects):
    #     #print(objects.objects)
    #    self.nr_objects = objects.num_objects
    #     self.objects = objects.objects
    #     self.image = objects.image_rgb

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
    
    ### OBSTACLES
    def obstacles_callback(self, obs: Obstacles):
        self.obstacles = obs
        
    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5):
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.activate_objects_hand = activate_objects_hand
        request.activate_shoes_hand = activate_shoes_hand
        request.activate_doors_hand = activate_doors_hand
        request.minimum_objects_confidence = minimum_objects_confidence
        request.minimum_shoes_confidence = minimum_shoes_confidence
        request.minimum_doors_confidence = minimum_doors_confidence

        self.activate_yolo_objects_client.call_async(request)

    #### FACE SERVER FUNCTIONS #####
    def call_face_command_server(self, command="", custom="", wait_for_end_of=True):
        request = SetFace.Request()
        request.command = command
        request.custom = custom
        
        future = self.face_command_client.call_async(request)
        
        if wait_for_end_of:
            future.add_done_callback(self.callback_call_face_command)
        else:
            self.face_success = True
            self.face_message = "Wait for answer not needed"
    
    def callback_call_face_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.face_success = response.success
            self.face_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_face = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    #### SPEECH SERVER FUNCTIONS #####
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True, show_in_face=False):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.show_in_face = show_in_face
    
        future = self.speech_command_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"
   
    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    

    #### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, position=[0, 0], wait_for_end_of=True):
        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])
        
        future = self.set_neck_position_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.neck_success = response.success
            self.neck_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### SET NECK COORDINATES SERVER FUNCTIONS #####
    def call_neck_coordinates_server(self, x, y, z, tilt, flag, wait_for_end_of=True):
        request = SetNeckCoordinates.Request()
        request.coords.x = float(x)
        request.coords.y = float(y)
        request.coords.z = float(z)
        request.is_tilt = flag
        request.tilt = float(tilt)
        
        future = self.set_neck_coordinates_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_coords_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_coords_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.neck_success = response.success
            self.neck_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_coords = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### GET NECK POSITION SERVER FUNCTIONS #####
    def call_get_neck_position_server(self):
        request = GetNeckPosition.Request()
        
        future = self.get_neck_position_client.call_async(request)
        # print("Sent Command")

        # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
        future.add_done_callback(self.callback_call_get_neck_command)
    
    def callback_call_get_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info("Received Neck Position: (%s" %(str(response.pan) + ", " + str(response.tilt)+")"))
            self.get_neck_position[0] = response.pan
            self.get_neck_position[1] = response.tilt
            # time.sleep(3)
            self.waited_for_end_of_get_neck = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))     

    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        self.arm_ready = True
        self.waited_for_end_of_arm = True
        self.arm_success = flag.data
        if flag.data:
            self.arm_message = "Arm successfully moved"
        else:
            self.arm_message = "Wrong Movement Received"

        self.get_logger().info("Received Arm Finished")


def main(args=None):
    rclpy.init(args=args)
    node = StoringGroceriesNode()
    th_main = threading.Thread(target=thread_main_storing_groceries, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_storing_groceries(node: StoringGroceriesNode):
    main = StoringGroceriesMain(node)
    main.main()

class StoringGroceriesMain():

    def __init__(self, node: StoringGroceriesNode):
        self.node = node
        
        # Task Related Variables
        self.Waiting_for_task_start = 0
        # self.Open_cabinet_door = 1
        self.Approach_cabinet_first_time = 1
        self.Approach_tables_first_time = 2
        self.Picking_first_object = 3
        self.Picking_second_object = 4
        self.Picking_third_object = 5
        self.Picking_fourth_object = 6
        self.Picking_fifth_object = 7
        self.Placing_first_object = 8     
        self.Placing_second_object = 9
        self.Placing_third_object = 10
        self.Placing_fourth_object = 11
        self.Placing_fifth_object = 12        
        self.Final_State = 13

        self.object_counter = 0

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_back = [180,0]
        self.look_navigation = [0, -30]
        self.look_judge = [-45, 0]
        self.look_table_objects = [90, -40]
        self.look_tray = [0, -60]
        self.look_cabinet_top = [-45, 45]
        self.look_cabinet_center = [0, -30]
        self.look_cabinet_bottom = [-45, -45]

        self.shelf_1_height = 0.1 # 0.15 # 0.14 # 0.15
        self.shelf_2_height = 0.5  # 0.60 # 0.55 # 0.60 
        self.shelf_3_height = 0.9  # 1.10 # 0.97 # 1.10 
        self.shelf_4_height = 1.33  # 1.39
        self.shelf_5_height = 1.8
        self.shelf_6_height = 2.5

        self.shelf_length = 0.70
        self.left_limit_shelf = -0.7 # -0.38
        self.right_limit_shelf = 0.7 # 0.38
        self.center_shelf = 0.0

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # to debug just a part of the task you can just change the initial state, example:
        # self.state = self.Approach_kitchen_table
        self.state = self.Waiting_for_task_start

        self.nr_objects_detected_previous = 0
        self.nr_max_objects_detected = 0
        self.image_most_obj_detected = Image()
        self.image_most_priority = Image()
        self.image_objects_detected = Image()
        self.prev_time = 0.0
        self.new_time = 0.0

        self.names_counter = 0
        self.objects_names_list = [""]
        self.object_details = {}
        self.object_position = {}
        self.priority_dict = {}
        self.nr_objects_accessed = 0

        self.classes_detected_wardrobe = []
        self.detect_object_total = [DetectedObject(), DetectedObject(), DetectedObject(), DetectedObject(), DetectedObject()]

        self.wait_time_to_put_objects_in_hand = 1
        
    ##### SETS #####

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message

    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message
    
    def wait_for_door_start(self):
        
        # max angle considered to be a door (degrees)
        MAX_DOOR_ANGLE = math.radians(15.0)
        # max distance to be considered a door (meters)
        MAX_DOOR_DISTANCE = 1.0 
        
        door_open = False

        self.set_rgb(WHITE+ALTERNATE_QUARTERS)

        while not door_open:
            ctr = 0
            for obs in self.node.obstacles.obstacles:
                # if the robot detects any obstacle inside the max_angle with a dist under max_dist it considers the door is closed
                # the max distance was introduced since in some cases, there may be a sofa, 3 meters away in that direction...
                if -MAX_DOOR_ANGLE < obs.alfa < MAX_DOOR_ANGLE and obs.dist < MAX_DOOR_DISTANCE:
                    ctr += 1
                    # print(math.degrees(obs.alfa), obs.dist)
            
            if ctr == 0:
                door_open = True
                print("DOOR OPEN")
            else:
                door_open = False
                print("DOOR CLOSED", ctr)
        
        self.set_rgb(GREEN+ALTERNATE_QUARTERS)
    
    def set_neck_coords(self, position=[], ang=0.0, wait_for_end_of=True):

        #  x, y, z, tilt, flag, wait_for_end_of=True):
        self.node.get_logger().info("LENGTH %d"%len(position))

        if len(position) == 2:
            self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=0.0, tilt=ang, flag=True, wait_for_end_of=wait_for_end_of)
        elif len(position) == 3:
            print("You tried neck to coordintes using (x,y,z) please switch to (x,y,theta)")
            pass
            # The following line is correct, however since the functionality is not implemented yet, should not be called
            # self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=position[2], tilt=0.0, flag=False, wait_for_end_of=wait_for_end_of)
        else:
            print("Something went wrong")


        # self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        # if wait_for_end_of:
        #   while not self.node.waited_for_end_of_neck_coords:
        #     pass
        # self.node.waited_for_end_of_neck_coords = False

        return self.node.neck_success, self.node.neck_message
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message
    
    def wait_for_start_button(self):

        self.node.start_button_state = False

        t = Bool()
        t.data = True
        self.node.flag_start_button_publisher.publish(t)

        while not self.node.start_button_state:
            pass

        t.data = False 
        self.node.flag_start_button_publisher.publish(t)
    
    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        self.node.call_face_command_server(command=command, custom=custom, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_face:
                pass
        self.node.waited_for_end_of_face = False

        return self.node.face_success, self.node.face_message

    ##### GETS #####
    def get_neck(self, wait_for_end_of=True):
    
        self.node.call_get_neck_position_server()
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False

        return self.node.get_neck_position[0], self.node.get_neck_position[1] 

    def set_arm(self, command="", pose=[], adjust_position=0.0, wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False
        
        temp = ArmController()
        temp.command = command
        temp.adjust_position = adjust_position
        temp.pose = pose
        self.node.arm_command_publisher.publish(temp)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_arm:
                pass
            self.node.waited_for_end_of_arm = False
        else:
            self.node.arm_success = True
            self.node.arm_message = "Wait for answer not needed"

        # self.node.get_logger().info("Set Arm Response: %s" %(str(self.arm_success) + " - " + str(self.arm_message)))
        return self.node.arm_success, self.node.arm_message

    
    def set_navigation(self, movement="", target=[0.0, 0.0], max_speed=15.0, absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, wait_for_end_of=True):

        if movement.lower() != "move" and movement.lower() != "rotate" and movement.lower() != "orientate" and movement.lower() != "adjust" and movement.lower() != "adjust_obstacle" and movement.lower() != "adjust_angle" :   
            self.node.get_logger().error("WRONG MOVEMENT NAME: PLEASE USE: MOVE, ROTATE OR ORIENTATE.")

            self.navigation_success = False
            self.navigation_message = "Wrong Movement Name"

        else:
            
            navigation = TarNavSDNL()

            # Pose2D target_coordinates
            # string move_or_rotate
            # float32 orientation_absolute
            # bool flag_not_obs
            # float32 reached_radius
            # bool avoid_people
            # float32 adjust_distance
            # float32 adjust_direction
            # float32 adjust_min_dist
            # float32 max_speed

            if adjust_direction < 0:
                adjust_direction += 360

            navigation.target_coordinates.x = float(target[0])
            navigation.target_coordinates.y = float(target[1])
            navigation.move_or_rotate = movement
            navigation.orientation_absolute = float(absolute_angle)
            navigation.flag_not_obs = flag_not_obs
            navigation.reached_radius = float(reached_radius)
            navigation.avoid_people = False
            navigation.adjust_distance = float(adjust_distance)
            navigation.adjust_direction = float(adjust_direction)
            navigation.adjust_min_dist = float(adjust_min_dist)
            navigation.max_speed = float(max_speed)

            self.node.flag_navigation_reached = False
            
            self.node.target_pos_publisher.publish(navigation)

            if wait_for_end_of:
                while not self.node.flag_navigation_reached:
                    pass
                self.node.flag_navigation_reached = False

            self.navigation_success = True
            self.navigation_message = "Arrived at selected location"

        return self.node.navigation_success, self.node.navigation_message    

    
    def set_initial_position(self, initial_position):

        task_initialpose = PoseWithCovarianceStamped()

        task_initialpose.header.frame_id = "map"
        task_initialpose.header.stamp = self.node.get_clock().now().to_msg()

        task_initialpose.pose.pose.position.x = initial_position[1]
        task_initialpose.pose.pose.position.y = -initial_position[0]
        task_initialpose.pose.pose.position.z = 0.0

        # quaternion = self.get_quaternion_from_euler(0,0,math.radians(initial_position[2]))

        # Convert an Euler angle to a quaternion.
        # Input
        #     :param roll: The roll (rotation around x-axis) angle in radians.
        #     :param pitch: The pitch (rotation around y-axis) angle in radians.
        #     :param yaw: The yaw (rotation around z-axis) angle in radians.
        # 
        # Output
        #     :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format

        roll = 0.0
        pitch = 0.0
        yaw = math.radians(initial_position[2])

        task_initialpose.pose.pose.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        task_initialpose.pose.pose.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        task_initialpose.pose.pose.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        task_initialpose.pose.pose.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        self.node.initialpose_publisher.publish(task_initialpose)

    def search_for_objects(self, tetas, delta_t=3.0, list_of_objects = [], list_of_objects_detected_as = [], use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False):

        final_objects = []
        mandatory_object_detected_flags = [False for _ in list_of_objects]
        # print(mandatory_object_detected_flags)
        DETECTED_ALL_LIST_OF_OBJECTS = False
        MIN_DIST_DIFFERENT_FRAMES = 0.3 # maximum distance for the robot to assume it is the same objects
        MIN_DIST_SAME_FRAME = 0.2

        merged_lists = []
        for obj, detected_as in zip(list_of_objects, list_of_objects_detected_as):
            merged_lists.append([obj] + detected_as)
        merged_lists = [[item.replace(" ","_").lower() for item in sublist] for sublist in merged_lists]
        # print(merged_lists)
        # for merged_list in merged_lists:
        #     print(merged_list)
        
        while not DETECTED_ALL_LIST_OF_OBJECTS:

            total_objects_detected = []
            objects_detected = []
            shoes_detected = []
            doors_detected = []
            objects_ctr = 0

            self.activate_yolo_objects(activate_objects=detect_objects, activate_shoes=detect_shoes, activate_doors=detect_doors,
                                        activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False,
                                        minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5)
            self.set_speech(filename="generic/search_objects", wait_for_end_of=False)
            self.set_rgb(WHITE+ALTERNATE_QUARTERS)
            time.sleep(0.5)

            ### MOVES NECK AND SAVES DETECTED OBJECTS ###
            for t in tetas:
                self.set_rgb(RED+SET_COLOUR)
                self.set_neck(position=t, wait_for_end_of=True)
                time.sleep(1.0) # 0.5
                self.set_rgb(WHITE+SET_COLOUR)

                start_time = time.time()
                while (time.time() - start_time) < delta_t:      

                    if detect_objects: 
                        local_detected_objects = self.node.detected_objects
                        for temp_objects in local_detected_objects.objects:
                            
                            is_already_in_list = False
                            object_already_in_list = DetectedObject()
                            for object in objects_detected:

                                # filters by same index
                                if temp_objects.index == object.index and temp_objects.object_name == object.object_name:
                                    is_already_in_list = True
                                    object_already_in_list = object

                                # second filter: sometimes yolo loses the IDS and creates different IDS for same objects, this filters the duplicates
                                if temp_objects.object_name == object.object_name: # and 
                                    dist = math.dist((temp_objects.position_absolute.x, temp_objects.position_absolute.y, temp_objects.position_absolute.z), (object.position_absolute.x, object.position_absolute.y, object.position_absolute.z))
                                    if dist < MIN_DIST_SAME_FRAME:
                                        is_already_in_list = True
                                        object_already_in_list = object

                            if is_already_in_list:
                                objects_detected.remove(object_already_in_list)
                            else:
                            # elif temp_objects.index > 0: # debug
                                # print("added_first_time", temp_objects.index, temp_objects.position_absolute.x, temp_objects.position_absolute.y)
                                self.set_rgb(GREEN+SET_COLOUR)
                            
                            # if temp_objects.index > 0:
                            objects_detected.append(temp_objects)
                            objects_ctr+=1

                        
                    if detect_shoes: 
                        local_detected_objects = self.node.detected_shoes
                        for temp_objects in local_detected_objects.objects:
                            
                            is_already_in_list = False
                            object_already_in_list = DetectedObject()
                            for object in shoes_detected:

                                # filters by same index
                                if temp_objects.index == object.index and temp_objects.object_name == object.object_name:
                                    is_already_in_list = True
                                    object_already_in_list = object

                                # second filter: sometimes yolo loses the IDS and creates different IDS for same objects, this filters the duplicates
                                if temp_objects.object_name == object.object_name and temp_objects.index != object.index: 
                                    dist = math.dist((temp_objects.position_absolute.x, temp_objects.position_absolute.y, temp_objects.position_absolute.z), (object.position_absolute.x, object.position_absolute.y, object.position_absolute.z))
                                    if dist < MIN_DIST_SAME_FRAME:
                                        is_already_in_list = True
                                        object_already_in_list = object

                            if is_already_in_list:
                                shoes_detected.remove(object_already_in_list)
                            else:
                            # elif temp_objects.index > 0: # debug
                                # print("added_first_time", temp_objects.index, temp_objects.position_absolute.x, temp_objects.position_absolute.y)
                                self.set_rgb(GREEN+SET_COLOUR)
                            
                            # if temp_objects.index > 0:
                            shoes_detected.append(temp_objects)
                            objects_ctr+=1

                        
                    if detect_doors: 
                        local_detected_objects = self.node.detected_doors
                        for temp_objects in local_detected_objects.objects:
                            
                            is_already_in_list = False
                            object_already_in_list = DetectedObject()
                            for object in doors_detected:

                                # filters by same index
                                if temp_objects.index == object.index and temp_objects.object_name == object.object_name:
                                    is_already_in_list = True
                                    object_already_in_list = object

                                # second filter: sometimes yolo loses the IDS and creates different IDS for same objects, this filters the duplicates
                                if temp_objects.object_name == object.object_name and temp_objects.index != object.index: 
                                    dist = math.dist((temp_objects.position_absolute.x, temp_objects.position_absolute.y, temp_objects.position_absolute.z), (object.position_absolute.x, object.position_absolute.y, object.position_absolute.z))
                                    if dist < MIN_DIST_SAME_FRAME:
                                        is_already_in_list = True
                                        object_already_in_list = object

                            if is_already_in_list:
                                doors_detected.remove(object_already_in_list)
                            else:
                            # elif temp_objects.index > 0: # debug
                                # print("added_first_time", temp_objects.index, temp_objects.position_absolute.x, temp_objects.position_absolute.y)
                                self.set_rgb(GREEN+SET_COLOUR)
                            
                            # if temp_objects.index > 0:
                            doors_detected.append(temp_objects)
                            objects_ctr+=1


                # DEBUG
                # print("objects in this neck pos:")
                # for object in objects_detected:
                #     print(object.index, object.position_absolute.x, object.position_absolute.y)
            
                total_objects_detected.append(objects_detected.copy() + shoes_detected.copy() + doors_detected.copy())
                # print("Total number of objects detected:", len(objects_detected), objects_ctr)
                objects_detected.clear()   
                shoes_detected.clear()
                doors_detected.clear()

                if list_of_objects: #only does this if there are items in the list of mandatory detection objects
                    
                    mandatory_ctr = 0
                    # for m_object in list_of_objects:
                    for m_object in merged_lists:
                        is_in_mandatory_list = False
                        
                        for frame in range(len(total_objects_detected)):
                            for object in range(len(total_objects_detected[frame])):
                                
                                # compares to local detected frame
                                # if total_objects_detected[frame][object].object_name.lower() == m_object.lower():
                                if total_objects_detected[frame][object].object_name.replace(" ","_").lower() in m_object:
                                    is_in_mandatory_list = True
                                    # print(m_object, total_objects_detected[frame][object].object_name, total_objects_detected[frame][object].index, is_in_mandatory_list)
                    
                                # compares to overall final detected objects
                                for final_obj in final_objects:
                                    # if final_obj.object_name.lower() == m_object.lower():
                                    if final_obj.object_name.replace(" ","_").lower() in m_object:
                                        is_in_mandatory_list = True
                                        # print(m_object, final_obj.object_name, final_obj.index, is_in_mandatory_list)

                        if is_in_mandatory_list:
                            mandatory_ctr += 1
                        # print(m_object, is_in_mandatory_list)

                    if mandatory_ctr == len(list_of_objects): # if all objects are already in the detected list 
                        break


            self.activate_yolo_objects(activate_objects=False, activate_shoes=False, activate_doors=False,
                                        activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False,
                                        minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5)
            

            # DEBUG
            # print("TOTAL objects in this neck pos:")
            # for frame in total_objects_detected:
            #     for object in frame:    
            #         print(object.index, object.object_name, "\t", round(object.position_absolute.x, 2), round(object.position_absolute.y, 2), round(object.position_absolute.z, 2))
            #     print("-")

            ### DETECTS ALL THE OBJECTS SHOW IN EVERY FRAME ###
            
            filtered_objects = []

            for frame in range(len(total_objects_detected)):

                to_append = []
                to_remove = []

                if not len(filtered_objects):
                    # print("NO OBJECTS", frame)
                    for object in range(len(total_objects_detected[frame])):
                        to_append.append(total_objects_detected[frame][object])
                else:
                    # print("YES OBJECTS", frame)

                    for object in range(len(total_objects_detected[frame])):
                        same_object_ctr = 0

                        for filtered in range(len(filtered_objects)):

                            if total_objects_detected[frame][object].object_name == filtered_objects[filtered].object_name: 

                                # dist_xy = math.dist((total_objects_detected[frame][object].position_absolute.x, total_objects_detected[frame][object].position_absolute.y), (filtered_objects[filtered].position_absolute.x, filtered_objects[filtered].position_absolute.y))
                                dist = math.dist((total_objects_detected[frame][object].position_absolute.x, total_objects_detected[frame][object].position_absolute.y, total_objects_detected[frame][object].position_absolute.z), (filtered_objects[filtered].position_absolute.x, filtered_objects[filtered].position_absolute.y, filtered_objects[filtered].position_absolute.z))
                                # print("new:", total_objects_detected[frame][object].index, total_objects_detected[frame][object].object_name, ", old:", filtered_objects[filtered].index, filtered_objects[filtered].object_name, ", dist:", round(dist,3)) # , dist_xy) 
                                
                                if dist < MIN_DIST_DIFFERENT_FRAMES:
                                    same_object_ctr+=1
                                    same_object_old = filtered_objects[filtered]
                                    same_object_new = total_objects_detected[frame][object]
                                    # print("SAME OBJECT")                        
                        
                        if same_object_ctr > 0:

                            image_center = (1280/2, 720/2)
                            same_object_old_distance_center = math.dist(image_center, (same_object_old.box_center_x, same_object_old.box_center_y))
                            same_object_new_distance_center = math.dist(image_center, (same_object_new.box_center_x, same_object_new.box_center_y))
                            
                            # print("OLD (pixel):", same_object_old.index, same_object_old.object_name, ", dist_2_center:", round(same_object_old_distance_center,2))
                            # print("NEW (pixel):", same_object_new.index, same_object_new.object_name, ", dist_2_center:", round(same_object_new_distance_center,2))

                            if same_object_new_distance_center < same_object_old_distance_center: # object from newer frame is more centered with camera center
                                to_remove.append(same_object_old)
                                to_append.append(same_object_new)
                            else: # object from older frame is more centered with camera center
                                pass # that object is already in the filtered list so we do not have to do anything, this is here just for explanation purposes 

                        else:
                            to_append.append(total_objects_detected[frame][object])

                for o in to_remove:
                    if o in filtered_objects:
                        # print("REMOVED: ", o.index, o.object_name)
                        filtered_objects.remove(o)
                    else:
                        pass
                        # print("TRIED TO REMOVE TWICE THE SAME OBJECT")
                to_remove.clear()  

                for o in to_append:
                    # print("ADDED: ", o.index, o.object_name)
                    filtered_objects.append(o)
                to_append.clear()

            print("FILTERED:")
            for o in filtered_objects:
                print(o.index, o.object_name, "\t", round(o.position_absolute.x, 2), round(o.position_absolute.y, 2), round(o.position_absolute.z, 2))


            if list_of_objects: #only does this if there are items in the list of mandatory detection objects
                
                for l_object in range(len(list_of_objects)):
                    for object in filtered_objects:

                        # if not final_objects: # if final_objects is empty

                        # if object.object_name.lower() == list_of_objects[l_object].lower() and not mandatory_object_detected_flags[l_object]:
                        if object.object_name.replace(" ","_").lower() in merged_lists[l_object] and not mandatory_object_detected_flags[l_object]:
                            final_objects.append(object)
                            # mandatory_object_detected_flags.append(True)
                            mandatory_object_detected_flags[l_object] = True
                            # break
                            # else:
                            #     mandatory_object_detected_flags.append(False)

                        # else:
                        #     pass
                
                # print(list_of_objects)
                # print(mandatory_object_detected_flags)
                
                if not all(mandatory_object_detected_flags):
                    # Speech: "There seems to be a problem with detecting the objects. Can you please slightly move and rotate the following objects?"
                    self.set_speech(filename="generic/problem_detecting_change_object", wait_for_end_of=True) 
                    for obj in range(len(list_of_objects)):
                        if not mandatory_object_detected_flags[obj]:
                            # Speech: (Name of object)
                            self.set_speech(filename="objects_names/"+list_of_objects[obj].replace(" ","_").lower(), wait_for_end_of=True)
                else:
                    DETECTED_ALL_LIST_OF_OBJECTS = True
                    # forces the change of objects name for possible detected_as_object 
                    # (i.e. might detect cleanser as milk, but we need it as milk for the DEM show in face)
                    for o in range(len(list_of_objects)): 
                        final_objects[o].object_name = list_of_objects[o]

            else:
                final_objects = filtered_objects
                DETECTED_ALL_LIST_OF_OBJECTS = True

        self.set_neck(position=[0, 0], wait_for_end_of=False)
        self.set_rgb(BLUE+HALF_ROTATE)

        # Debug Speak
        # self.set_speech(filename="generic/found_following_items")
        # for obj in final_objects:
        #     self.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=True)

        sfo_pub = ListOfDetectedObject()
        # print("FILTERED:")
        for o in final_objects:
            sfo_pub.objects.append(o)
        #     print(o.object_name)
        self.node.search_for_object_detections_publisher.publish(sfo_pub)
            
        return final_objects   

    def detected_object_to_face_path(self, object, send_to_face, bb_color=(0,0,255)):

        thresh_h = 220
        thresh_v = 220

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        cf = self.node.br.imgmsg_to_cv2(object.image_rgb_frame, "bgr8")
        
        # checks whether the text has to start inside the bounding box or can start outside (image boundaries)
        start_point = (object.box_top_left_x, object.box_top_left_y)
        end_point = (object.box_top_left_x+object.box_width, object.box_top_left_y+object.box_height)
        cv2.rectangle(cf, start_point, end_point, bb_color , 4) 
        # cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
        
        if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
        else:
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
            
        text_size, _ = cv2.getTextSize(f"{object.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        text_w, text_h = text_size
        cv2.rectangle(cf, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), bb_color, -1)
        cv2.putText(cf, f"{object.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    
        object_image = cf[max(object.box_top_left_y-thresh_v,0):min(object.box_top_left_y+object.box_height+thresh_v,720), max(object.box_top_left_x-thresh_h,0):min(object.box_top_left_x+object.box_width+thresh_h,1280)]
        # cv2.imshow("Search for Person", object_image)
        # cv2.waitKey(100)
        
        face_path = current_datetime + str(object.index) + str(object.object_name)
        
        cv2.imwrite(self.node.complete_path_custom_face + face_path + ".jpg", object_image) 
        time.sleep(0.5)
        
        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path
    
    # def analysis_cabinet(self):
    #     nr_classes_detected = 0
    #     i = 0
    #     objects = []
    #     self.object_position = {}
    #     if hasattr(self.node, 'image') and self.node.image:
    #         if hasattr(self.node, 'objects') and self.node.objects:
    #             objects_stored = self.node.objects
    #             self.nr_objects_detected = self.node.nr_objects


    #             self.current_image = self.node.image
    #             bridge = CvBridge()
    #             # Convert ROS Image to OpenCV image
    #             cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
    #             self.current_image_2= cv_image

    #             for detected_objects in objects_stored:
    #                 print(detected_objects.object_name, detected_objects.object_class)
    #                 if detected_objects.object_name in objects:
    #                     pass
    #                 else:
    #                     objects.append(detected_objects) 

    #             print(objects)

    #             print('Will iterate for: ', self.nr_objects_detected)
    #             while i < self.nr_objects_detected:                    
    #                 detected_object = objects_stored[i]
    #                 object_name = detected_object.object_name
    #                 object_class = detected_object.object_class
    #                 object_height = detected_object.position_relative.z
    #                 object_distance = detected_object.position_relative.y
    #                 object_confidence = detected_object.confidence
    #                 object_x_position = detected_object.position_relative.x
    #                 box_top_left_x = detected_object.box_top_left_x
    #                 box_top_left_y = detected_object.box_top_left_y
    #                 box_width = detected_object.box_width
    #                 box_height = detected_object.box_height
    #                 position = ' '
    #                 #print(f"Object: {object_name}, Height: {object_height}, Confidence: {object_confidence}")
    #                 if object_name in self.object_details:
    #                     pass
    #                 else:
    #                     if object_distance > 3.0:
    #                         print(object_name, '- too far')
    #                         print(object_height)
                            
    #                     elif self.shelf_1_height < object_height < self.shelf_2_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'First shelf '
    #                         print(object_name, 'is in the first shelf ')
    #                         # print(object_x_position)

    #                     elif self.shelf_2_height < object_height < self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'Second shelf '
    #                         print(object_name, 'is in the second shelf ')
    #                         # print(object_x_position)

    #                     elif self.shelf_4_height > object_height > self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'Third shelf '
    #                         print(object_name, 'is in the third shelf ')
    #                         # print(object_x_position)
                        
    #                     elif self.shelf_5_height > object_height > self.shelf_4_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'Fourth shelf '
    #                         print(object_name, 'is in the fourth shelf ')
    #                         # print(object_x_position)

    #                     elif self.shelf_6_height > object_height > self.shelf_5_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'Fifth shelf '
    #                         print(object_name, 'is in the fifth shelf ')
    #                         # print(object_x_position)

    #                     elif object_height > self.shelf_6_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
    #                         position = 'Sixth shelf '
    #                         print(object_name, 'is in the sixth shelf ')
    #                         # print(object_x_position)

    #                     else:
    #                         print(object_name, '- none of the shelfs')
    #                         print(object_height)
                            
    #                     """ if  self.center_shelf <= object_x_position <= self.right_limit_shelf :
    #                         position += 'Right side '
                            
    #                     elif self.left_limit_shelf <= object_x_position < self.center_shelf :
    #                         position += 'Left side '
                            
    #                     else:
    #                         position += 'Outside shelf ' """
                            
    #                     if detected_object.object_class in self.object_position:
    #                         self.object_position[detected_object.object_class].append(position)
    #                     else:
    #                         self.object_position[detected_object.object_class] = [position]

    #                     #self.object_position[object_class] = position


    #                     print('object ', object_name, ' and confidence ', object_confidence)

    #                 i += 1

    #             # Código para dizer 'tal classe está em tal prateleira'

    #             print('objects position:', self.object_position)
    #             object_x_values = {}

    #             # Dictionary to store filtered values
    #             filtered_objects_position = {}

    #             # Loop through the original dictionary
    #             for key, values in self.object_position.items():
    #                 # Use set to remove duplicates and then convert back to list
    #                 unique_values = list(set(values))
    #                 # If there's only one unique value, use that
    #                 if len(unique_values) == 1:
    #                     filtered_objects_position[key] = unique_values[0]

    #             print("Filtered objects position:", filtered_objects_position)

    #             # Iterate through the objects and store object_x values for each class
    #             for obj in objects:
    #                 obj_class = obj.object_class
    #                 object_x = obj.position_relative.x
    #                 if obj_class not in object_x_values:
    #                     object_x_values[obj_class] = []
    #                 object_x_values[obj_class].append(object_x)

    #             # Print object_x values for each class
    #             for obj_class, x_values in object_x_values.items():
    #                 #print(f"{obj_class} object_x values:")
    #                 i = 0
    #                 average_x_values = 0
    #                 for x in x_values:
    #                     i +=1
    #                     print(f"  - {x}")
    #                     average_x_values += x
    #                 average_x_values = average_x_values / i
    #                 #print(f"average: {average_x_values}")
                
    #             # Organize objects by position
    #             objects_by_position = {}
    #             for obj_class, position in filtered_objects_position.items():
    #                 if position not in objects_by_position:
    #                     objects_by_position[position] = []
    #                 objects_by_position[position].append(obj_class)
                
    #             for position, obj_classes in objects_by_position.items():
    #                 print(f"\nObjects in {position}:")
    #                 for obj_class in obj_classes:
    #                     if obj_class in object_x_values:
    #                         x_values = object_x_values[obj_class]
    #                         print(f"{obj_class} object_x values:")
    #                         i = 0
    #                         average_x_values = 0
    #                         for x in x_values:
    #                             i +=1
    #                             print(f"  - {x}")
    #                             average_x_values += x
    #                         average_x_values = average_x_values / i
    #                         print(f"  Average: {average_x_values}")
    #                     else:
    #                         print(f"No object_x values found for class {obj_class}")
                    

    #             # Dictionary to store average x values for each class and shelf
    #             average_values_by_shelf = {}

    #             # Iterate through the objects and store object_x values for each class
    #             for obj in objects:
    #                 obj_class = obj.object_class
    #                 object_x = obj.position_relative.x
    #                 position = filtered_objects_position.get(obj_class)
    #                 if position:
    #                     if position not in average_values_by_shelf:
    #                         average_values_by_shelf[position] = {}
    #                     if obj_class not in average_values_by_shelf[position]:
    #                         average_values_by_shelf[position][obj_class] = []
    #                     average_values_by_shelf[position][obj_class].append(object_x)
    #                     print('average values by shelf', average_values_by_shelf)

    

    #             # Initialize variables to store reference x values
    #             left_reference_x = None
    #             right_reference_x = None
    #             left_reference_x = -0.5
    #             right_reference_x = 0.05

    #             # Initialize variable for single class average x
    #             single_class_average_x = None

    #             for shelf, class_values in average_values_by_shelf.items():
    #                 average_values_by_shelf[shelf] = dict(sorted(class_values.items(), key=lambda item: len(item[1]), reverse=True))

    #             # Sort the shelves based on the total number of classes they contain
    #             sorted_shelves = dict(sorted(average_values_by_shelf.items(), key=lambda item: len(item[1]), reverse=True))

    #             print('sorted shelves ', sorted_shelves)

    #             average_values_by_shelf = sorted_shelves

    #             positions_in_cabinet = {}

    #             # Iterate through the average values for each shelf
    #             for shelf, class_values in average_values_by_shelf.items():
    #                 print(f"\nShelf: {shelf}")

    #                 # Get class names and average x values
    #                 class_names = list(class_values.keys())
    #                 average_x_values = [sum(x_values) / len(x_values) for x_values in class_values.values()]

    #                 left_reference_x = -0.5
    #                 right_reference_x = 0.05

    #                 if len(average_x_values) == 2:
    #                     # If two classes and reference x values are not yet established, determine left or right side and store reference x values
    #                     if left_reference_x is None and right_reference_x is None:
    #                         if average_x_values[0] > average_x_values[1]:
    #                             print(f'{class_names[1]} - left side')
    #                             print(f'{class_names[0]} - right side')
    #                             left_reference_x = average_x_values[1]
    #                             right_reference_x = average_x_values[0]
    #                             positions_in_cabinet[class_names[1]] = f"{shelf} Left side"
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Right side"
    #                         else:
    #                             print(f'{class_names[1]} - right side')
    #                             print(f'{class_names[0]} - left side')
    #                             left_reference_x = average_x_values[0]
    #                             right_reference_x = average_x_values[1]
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Left side"
    #                             positions_in_cabinet[class_names[1]] = f"{shelf} Right side"
    #                     else:
    #                         if average_x_values[0] > average_x_values[1]:
    #                             print(f'{class_names[1]} - left side')
    #                             print(f'{class_names[0]} - right side')
    #                             positions_in_cabinet[class_names[1]] = f"{shelf} Left side"
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Right side"
    #                         else:
    #                             print(f'{class_names[0]} - right side')
    #                             print(f'{class_names[1]} - left side')
    #                             positions_in_cabinet[class_names[1]] = f"{shelf} Left side"
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Right side"
    #                 elif len(average_x_values) == 1:
    #                     # If only one class, compare with reference x values
    #                     if left_reference_x is not None and right_reference_x is not None:
    #                         single_class_average_x = average_x_values[0]
    #                         if abs(single_class_average_x - left_reference_x ) < self.shelf_length / 3:
    #                             print(f"{class_names[0]} - left side")
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Left side"
    #                         elif abs(single_class_average_x - right_reference_x ) < self.shelf_length / 3:
    #                             print(f"{class_names[0]} - right side")
    #                             positions_in_cabinet[class_names[0]] = f"{shelf} Right side"
    #                     else:
    #                         print("Not enough data to determine position")
    #                 else:
    #                     # If three classes, print a message indicating the situation is strange
    #                     print("Strange situation: Three classes present")
                    

    #             print(positions_in_cabinet)
    #             self.object_position = positions_in_cabinet
    
    #             keywords = []

    #             self.classes_detected_wardrobe.clear()

    #             class_name_array = []
    #             nr_classes_detected = 0

    #             for position in positions_in_cabinet.values():
    #                 keywords = position.split()  # Split each position string into words and extend the keywords list

    #                 # Initialize filenames
    #                 class_filename = None
    #                 location_filename = None

    #                 # Iterate over object_position_mapping
    #                 for condition, object_location in object_position_mapping.items():
    #                     # Check if all keywords in the condition are in the current position
    #                     if all(keyword in keywords for keyword in condition):
    #                         # Get the class name associated with the current position
    #                         class_name = [class_name for class_name, pos in positions_in_cabinet.items() if pos == position][0]
    #                         if class_name not in class_name_array:
    #                             class_name_array.append(class_name)
    #                         print('Class name:', class_name)
    #                         print('All class names', class_name_array)
    #                         self.classes_detected_wardrobe.append(class_name)
    #                         print(self.classes_detected_wardrobe)
    #                         nr_classes_detected = len(class_name_array)
    #                         print('Nr classes detected: ', nr_classes_detected)
    #                         location_filename = f"storing_groceries/{object_location}"
    #                         class_filename = f"objects_classes/{class_name}"
    #                         break

    #     return nr_classes_detected


  
    def analysis_cabinet(self, filtered_objects):

        # min_object = min(filtered_objects, key=lambda x: x[1])
        # max_object = max(filtered_objects, key=lambda x: x[1])
        min_object = min(filtered_objects, key=lambda obj: obj.position_relative.x)
        max_object = max(filtered_objects, key=lambda obj: obj.position_relative.x)

        print('x_min:', min_object.position_relative.x)
        print('x_max:', max_object.position_relative.x)
        dif = abs(max_object.position_relative.x - min_object.position_relative.x)
        print('dif', dif)
        print(round(dif/2))
        half = min_object.position_relative.x + dif/2

        print('half:', half)

        print("Object with the minimum value in the 2nd argument:", min_object.object_name, min_object.position_relative.x)
        print("Object with the maximum value in the 2nd argument:", max_object.object_name, max_object.position_relative.x)

        


        # Initialize dictionaries to store objects and class counts for each shelf and side
        self.shelf_side_objects = defaultdict(list)
        self.shelf_side_class_counts = defaultdict(Counter)
        self.shelf_side_common_class = {}
        attributed_classes = set()

        # Analyze and categorize objects
        for obj in filtered_objects:
            side = 'left' if obj.position_relative.x < half else 'right'
            if self.shelf_1_height < obj.position_relative.z < self.shelf_2_height:
                shelf = 1
            elif self.shelf_2_height < obj.position_relative.z < self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                shelf = 2
            elif self.shelf_3_height < obj.position_relative.z < self.shelf_4_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                shelf = 3
            elif self.shelf_4_height < obj.position_relative.z < self.shelf_5_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                shelf = 4
            else:
                continue  # Only considering the first shelf for now, can be expanded as needed
            
            shelf_side_key = (shelf, side)
            self.shelf_side_objects[shelf_side_key].append(obj)
            self.shelf_side_class_counts[shelf_side_key][obj.object_class] += 1

        # Determine the most common class for each shelf and side
        self.shelf_side_common_class = {}
        self.tied_shelves = defaultdict(list)

        # Loop through each shelf-side combination
        for key, class_count in self.shelf_side_class_counts.items():
            # Find the most common class(es) and handle ties
            most_common_classes = class_count.most_common()
            highest_count = most_common_classes[0][1]

            # Filter classes that have the highest count
            tied_classes = [cls for cls, count in most_common_classes if count == highest_count]

            if len(tied_classes) == 1:
                # No tie, only one class has the highest count
                most_common_class = tied_classes[0]
                self.shelf_side_common_class[key] = most_common_class
                attributed_classes.add(most_common_class)
            else:
                # Tie exists, mark as "tied" and store tied classes
                # self.shelf_side_common_class[key] = "tied"
                self.tied_shelves[key] = tied_classes


        # Second Pass: Resolve ties
        for key, tied_classes in self.tied_shelves.items():
            unassigned_class = None
            for cls in tied_classes:
                if cls not in attributed_classes:
                    unassigned_class = cls
                    break

            if unassigned_class:
                # Assign the unassigned class
                self.shelf_side_common_class[key] = unassigned_class
                attributed_classes.add(unassigned_class)
            else:
                # Keep the tie if all classes are already attributed
                self.shelf_side_common_class[key] = "tied"

        # Print the results
        print("Common classes per shelf-side combination:")
        print(self.shelf_side_common_class)
        print("\nTied classes per shelf-side combination:")
        print(dict(self.tied_shelves))
        print('\n')
        # for key, class_count in self.shelf_side_class_counts.items():
        #     most_common_class, count = class_count.most_common(1)[0]
        #     self.shelf_side_common_class[key] = most_common_class



        # Output results
        for key, common_class in self.shelf_side_common_class.items():
            shelf, side = key
            print(f"Shelf {shelf}, Side {side} - Most Common Class: {common_class}")
            print(f"Objects cabinet: {[obj.object_name for obj in self.shelf_side_objects[key]]}")


        # for obj in filtered_objects:
        #     if obj.position_relative.x < half:
        #         self.objects_left_side.append(obj)

        #         if self.shelf_1_height < obj.position_relative.z < self.shelf_2_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_first_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the first shelf in the left side')
        #             # print(object_x_position)

        #         elif self.shelf_2_height < obj.position_relative.z < self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_second_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the second shelf in the left side')
        #             # print(object_x_position)

        #         elif self.shelf_4_height > obj.position_relative.z > self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_third_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the third shelf in the left side')
        #             # print(object_x_position)
                
        #         elif self.shelf_5_height > obj.position_relative.z > self.shelf_4_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_fourth_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the fourth shelf in the left side')
        #             # print(object_x_position)

        #         elif self.shelf_6_height > obj.position_relative.z > self.shelf_5_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_fifth_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the fifth shelf in the left side')
        #             # print(object_x_position)

        #         elif obj.position_relative.z > self.shelf_6_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_sixth_shelf_ls.append(obj)
        #             print(obj.object_name, 'is in the sixth shelf in the left side')
        #             # print(object_x_position)

        #     else:
        #         self.objects_right_side.append(obj)
        #         print(obj.object_name, 'is in the right side')
                
        #         if self.shelf_1_height < obj.position_relative.z < self.shelf_2_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_first_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the first shelf is in the right side')
        #             # print(object_x_position)

        #         elif self.shelf_2_height < obj.position_relative.z < self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_second_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the second shelf is in the right side')
        #             # print(object_x_position)

        #         elif self.shelf_4_height > obj.position_relative.z > self.shelf_3_height: #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_third_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the third shelf is in the right side')
        #             # print(object_x_position)
                
        #         elif self.shelf_5_height > obj.position_relative.z > self.shelf_4_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_fourth_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the fourth shelf is in the right side')
        #             # print(object_x_position)

        #         elif self.shelf_6_height > obj.position_relative.z > self.shelf_5_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_fifth_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the fifth shelf is in the right side')
        #             # print(object_x_position)

        #         elif obj.position_relative.z > self.shelf_6_height:  #and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
        #             self.objects_sixth_shelf_rs.append(obj)
        #             print(obj.object_name, 'is in the sixth shelf is in the right side')
        #             # print(object_x_position)      

            
    def load_image_one_object(self, obj_name, obj):
        # Construct the filename for the image
        image_name = f"image_{obj_name}.jpg"

        # Specify the directory where the images are stored
        output_dir = "images_with_rectangles"

        # Construct the file path for the image
        image_path = os.path.join(output_dir, image_name)

        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        # self.custom_face_filename = current_datetime

        current_frame_draw = self.image_objects_detected.copy()

        # Check if the image exists
        # if os.path.exists(image_path):
        #     # Load the image
        #     image = cv2.imread(image_path)
        #     current_frame_draw = image.copy()

            # Check if the image was loaded successfully
        if current_frame_draw is not None:
            # Display or process the loaded image as needed
            """ cv2.imshow("Original image ", current_frame_draw)
            cv2.waitKey(0) """

            thresh_h = 220
            thresh_v = 220

            x_min = 1280
            x_max = 0
            y_min = 720
            y_max = 0
        
            
            if obj.box_top_left_x < x_min:
                x_min = obj.box_top_left_x
            if obj.box_top_left_x+obj.box_width > x_max:
                x_max = obj.box_top_left_x+obj.box_width

            if obj.box_top_left_y < y_min:
                y_min = obj.box_top_left_y
            if obj.box_top_left_y+obj.box_height > y_max:
                y_max = obj.box_top_left_y+obj.box_height
            
            
            start_point = (obj.box_top_left_x, obj.box_top_left_y)
            end_point = (obj.box_top_left_x+obj.box_width, obj.box_top_left_y+obj.box_height)
            cv2.rectangle(current_frame_draw, start_point, end_point, (255,0,0) , 4) 
            
            if obj.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                start_point_text = (obj.box_top_left_x-2, obj.box_top_left_y+25)
            else:
                start_point_text = (obj.box_top_left_x-2, obj.box_top_left_y-22)
                
            text_size, _ = cv2.getTextSize(f"{obj.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            text_w, text_h = text_size
            cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,0,0), -1)
            cv2.putText(current_frame_draw, f"{obj.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

            current_frame_draw = current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]

            cv2.imwrite(self.node.complete_path_custom_face + current_datetime + obj_name + ".jpg", current_frame_draw)
            
            self.set_face(custom=current_datetime + obj_name)
            """ cv2.imshow("New Image", current_frame_draw)
            cv2.waitKey(0)
            cv2.destroyAllWindows() """
        else:
            print("Error: Unable to load the image.")
    # else:
    #     print("Error: Image not found.")

    def analysis_table(self, table_objects):
        objects_choosed = []
        for obj in table_objects:
            if obj.object_class in self.high_priority_class:
                print('Object table high priority:', obj.object_name)
                objects_choosed.append(obj)
                if len(objects_choosed) == 5:
                    break
            if len(objects_choosed) == 5:
                break
            
        if len(objects_choosed) < 5:
            for obj in table_objects:
                if obj.object_class in self.medium_priority_class:
                    print('Object table medium priority:', obj.object_name)
                    objects_choosed.append(obj)
                    if len(objects_choosed) == 5:
                        break
                if len(objects_choosed) == 5:
                    break
        
        if len(objects_choosed) < 5:
            for obj in table_objects:
                print('Object table low priority:', obj.object_name)
                objects_choosed.append(obj)
                if len(objects_choosed) == 5:
                    break
           
        
        # if len(objects_choosed) < 5:
        #     for obj in table_objects:
        #         print('Object table:', obj.object_name)
        #         if obj.object_class not in self.medium_priority_class and obj.object_class not in self.high_priority_class:
        #             objects_choosed.append(obj)
        #             if len(objects_choosed) == 5:
        #                 break
        #         if len(objects_choosed) == 5:
        #             break
           
        return objects_choosed

    # def detect_table_objects(self):
    #     i = 0
    #     nr_objects_high_priority_detected = 0
    #     self.detected_object = []

    #     for name, class_name in self.node.objects_classNames_dict.items():
    #         if class_name in self.classes_detected_wardrobe:
    #             self.priority_dict[class_name] = 'High'
    #             print(class_name + ' High')
    #         else:
    #             self.priority_dict[class_name] = 'Low'
    #             print(class_name + ' Low')

    #     # if hasattr(self.node, 'image') and self.node.image:
    #     #     if hasattr(self.node, 'objects') and self.node.objects:

    #     five_objects_detected = False
    #     detect_object = []
        
    #     list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]
    #     while not five_objects_detected:
            
    #         self.activate_yolo_objects(activate_objects=True)
    #         finished_detection = False

    #         for pos in list_of_neck_position_search:

    #             print(pos)
    #             new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
    #             #new_neck_pos = [ pos[0],  pos[1]]
    #             print('Neck: ', new_neck_pos)
    #             self.set_neck(position=new_neck_pos, wait_for_end_of=True)
    #             self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
    #             time.sleep(1)

    #             # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=5.0, with_hand=False)    

    #             self.objects_stored = self.node.objects
    #             self.nr_objects_detected = self.node.nr_objects
    #             self.current_image = self.node.image
    #             bridge = CvBridge()
    #             # Convert ROS Image to OpenCV image
    #             cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
    #             self.image_objects_detected = cv_image
    #             current_frame_draw = self.image_objects_detected
    #             print('Will iterate for: ', self.nr_objects_detected)
    #             nr_objects_high_priority_detected = 0
    #             i = 0
    #             for detected_objects in self.objects_stored:
    #                 print(detected_objects.object_name, detected_objects.object_class)
    #                 if detected_objects.object_name in detect_object:
    #                     pass
    #                 else:
    #                     detect_object.append(self.objects_stored) 
    #                     if self.priority_dict[detected_objects.object_class] == 'High':
    #                         nr_objects_high_priority_detected += 1
    #                         print('Nr objects high: ', nr_objects_high_priority_detected)

    #                     print('Object ' + detected_objects.object_name + ' from class ' + detected_objects.object_class + ' has ' + self.priority_dict[detected_objects.object_class] + 'priority')

    #                 i += 1

    #             print(i)
    #             if nr_objects_high_priority_detected >= 5:
    #                 print(self.objects_stored)
    #                 five_objects_detected = True
    #                 self.set_rgb(command=GREEN+BLINK_LONG)
    #                 break
    #             self.set_rgb(command=RED+BLINK_LONG)

    #     return nr_objects_high_priority_detected

    def choose_place_object_wardrobe(self, counter): 
        object_ = self.selected_objects[counter]
        obj_class = object_.object_class
        keywords = []
        print('choose place object wardrobe ', object_.object_name)
        # print(obj_class)
        # print(self.selected_objects)
        # print(self.object_position.keys())
        print(self.object_position)
        if obj_class in self.object_position.keys():
            # print(self.object_position.items())
            position = self.object_position[obj_class]
            print(position)
            # self.set_speech(filename='storing_groceries/Place_the_object_sg', wait_for_end_of=True)
            keywords = position.split() # Split each position string into words and extend the keywords list

            location_filename = None
            class_filename = None
            
            print('keywords ', keywords)
            print('object mapping ', object_position_mapping)
            for condition, object_location in object_position_mapping.items():
                if all(keyword in keywords for keyword in condition):
                    # If conditions are met, relate the class name to the position
                    location_filename = f"storing_groceries/{object_location}"
                    class_filename = f"objects_classes/_{obj_class}"
                    print(location_filename)
                    print(class_filename)
                    # print(object_location)
                    self.set_speech(filename=location_filename, wait_for_end_of=True)
                    self.set_speech(filename='generic/Near', wait_for_end_of=True)
                    self.set_speech(filename=class_filename, wait_for_end_of=True)
        else:
            # eventualmente terei de colocar aqui algo caso o objeto escolhido não estivesse na prateleira
            pass
        self.object_counter += 1

    def choose_priority(self, objects):
        # Este nível fica para a versão 1. Para a versão 0 faço ver o que está na prateleira, guardar essas classes e ficam essas como high
        
        # print(self.shelf_side_objects.values())
        print(self.shelf_side_objects.keys())

        print('\n')


        # Check if object is in self.shelf_side_objects with keys having shelf 2 or 3
        for obj in objects:
            found = any(
                obj in obj_list
                for key, obj_list in self.shelf_side_objects.items()
                if key[0] in (2, 3, 4)  # Only consider shelves 2, 3 and 4
            )
            if found:
                print(f"{obj.object_name} is in self.shelf_side_objects for shelf 2, 3 or 4")
                if obj.object_class not in self.high_priority_class:
                    self.high_priority_class.append(obj.object_class)
                    # print(obj.object_class + ' High')
            else:
                print(f"{obj.object_name} is NOT in self.shelf_side_objects for shelf 2, 3 or 4")
                if obj.object_class not in self.medium_priority_class:
                    self.medium_priority_class.append(obj.object_class)


        """ i = 0
        if hasattr(self.node, 'image') and self.node.image:
            if hasattr(self.node, 'objects') and self.node.objects:
                objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects
                while i < len(self.classes_detected_wardrobe) and i < self.nr_objects_detected:                   
                    detected_object = objects_stored[i]
                    object_name = detected_object.object_name
                    object_class = detected_object.object_class
                    object_height = detected_object.position_relative.z
                    object_confidence = detected_object.confidence
                    object_x_position = detected_object.position_relative.x
                    box_top_left_x = detected_object.box_top_left_x
                    box_top_left_y = detected_object.box_top_left_y
                    box_width = detected_object.box_width
                    box_height = detected_object.box_height
                    #print(f"Object: {object_name}, Height: {object_height}, Confidence: {object_confidence}")

                    if object_class in self.classes_detected_wardrobe:
                        object_priority = 'High'
                        
                    else:
                        object_priority = 'Low'

                    print(object_class + ' ' + object_priority)

                    self.object_details[object_name] = {'confidence': object_confidence, 'object_height': object_height,
                                                        'object_class': object_class,'x_position': object_x_position, 'box_top_left_x': box_top_left_x,
                                                        'box_top_left_y': box_top_left_y, 'box_width': box_width, 'box_height': box_height, 'priority': object_priority}

                    i += 1 """
        
        # for name, class_name in self.node.objects_classNames_dict.items():
        #     if class_name in self.classes_detected_wardrobe:
        #         self.priority_dict[class_name] = 'High'
        #         print(class_name + ' High')
        #     else:
        #         self.priority_dict[class_name] = 'Low'
        #         print(class_name + ' Low')



        
      
    def select_voice_audio(self, object):
        print('dentro')
        
            # category = self.node.objects_classNames_dict[name]
        name = object.object_name.lower().replace(" ", "_")
        print(name)
        filename = f"objects_names/{name}"
        self.set_speech(filename=filename, wait_for_end_of=True)
        print(f"Playing audio file: {filename}")

    def select_five_objects(self, objects_stored):
        
        self.priority_dict = {}

        # Iterate through detected objects to determine priority for each class_name
        for detected_object in objects_stored:
            class_name = detected_object.object_class
            if class_name in self.classes_detected_wardrobe:
                self.priority_dict[class_name] = 'High'
                print(class_name + ' High')
            else:
                self.priority_dict[class_name] = 'Low'
                print(class_name + ' Low')

        # Sort objects by confidence in descending order
        sorted_objects = sorted(objects_stored, key=lambda x: x.confidence, reverse=True)

        # Filter objects with confidence higher than 0.5
        filtered_objects = [obj for obj in sorted_objects if obj.confidence > 0.5]

        # Initialize selected objects list
        self.selected_objects = []

        # Select objects with higher confidence and higher priority
        for obj in filtered_objects:
            if len(self.selected_objects) == 5:
                break
            if self.priority_dict[obj.object_class] == 'High':
                self.selected_objects.append((obj))

        # If there are not enough high priority objects, select from remaining objects
        if len(self.selected_objects) < 5:
            remaining_count = 5 - len(self.selected_objects)
            remaining_objects = [obj for obj in filtered_objects if self.priority_dict[obj.object_class] != 'High']
            self.selected_objects.extend([(obj) for obj in remaining_objects[:remaining_count]])

        print('Selected: ', self.selected_objects)

        # self.selected_objects[self.nr_objects_accessed]

        output_dir = "images_with_rectangles"
        os.makedirs(output_dir, exist_ok=True)  
                
        # Iterate over each object in self.selected_objects
        # for detected_object in self.selected_objects:
            # Copy the original image to draw rectangles on
        images_table = self.image_objects_detected.copy()

        """ # Get the attributes of the detected object
        box_top_left_x = detected_object.box_top_left_x
        box_top_left_y = detected_object.box_top_left_y
        box_width = detected_object.box_width
        box_height = detected_object.box_height

        # Calculate end point of the rectangle
        end_point = (box_top_left_x + box_width, box_top_left_y + box_height) """

        # Draw rectangle on the copy of the original image
        # cv2.rectangle(image_with_rectangles, (box_top_left_x, box_top_left_y), end_point, (0, 255, 0), 2)
        # cv2.putText(image_with_rectangles, f"{detected_object.object_name}", (box_top_left_x, box_top_left_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)


        # cv2.imshow('A', image_with_rectangles)
        # cv2.waitKey(0)

        # Construct the filename for the image
        image_name = f"image_{detected_object.object_name}.jpg"

            # Save the image with rectangles
        cv2.imwrite(os.path.join(output_dir, image_name), images_table)

        # print(f"Saved image with rectangle for object {detected_object.object_name} as {image_name}")
        
    def plot_histograms(self, data):
        height, width = 780, 1280
        image = np.zeros((height, width), dtype=np.uint8)
        image2 = np.zeros((height, width), dtype=np.uint8)

        # Center of the image
        center_x, center_y = width // 2, height // 2

        # Real-world dimensions (in meters)
        min_width_meters = -3.0
        max_width_meters = 3.0
        min_height_meters = -2.5
        max_height_meters = 2.5
        min_height_meters_y = 5.0
        max_height_meters_y = 0.0

        coordinates = data

        # Function to convert real-world coordinates to image coordinates
        
        # Lists to store y-coordinates for the histogram
        y_coordinates_front_view = []
        y_coordinates_top_view = []
            

        # Plot points on the image
        for obj in coordinates:
            img_x = int((obj.position_relative.x - min_width_meters) / (max_width_meters - min_width_meters) * width)
            # Scale y from (-max_height, max_height) to (0, img_height)
            img_y = height - int((obj.position_relative.y - min_height_meters_y) / (max_height_meters_y - min_height_meters_y) * height)
    
            img_z = height - int((obj.position_relative.z - min_height_meters) / (max_height_meters - min_height_meters) * height)
            
            img_x = center_x + (img_x - center_x)
            img_y = center_y - (img_y - center_y)
            img_z = center_y + (img_z - center_y)
            cv2.circle(image, (img_x, img_z), 5, (255, 255, 255), -1)  # Draw white points
            cv2.putText(image, obj.object_name, (img_x + 5, img_z - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_coordinates_front_view.append(img_x)
            
            cv2.circle(image2, (img_x, img_y), 5, (255, 255, 255), -1)  # Draw white points
            cv2.putText(image2, obj.object_name, (img_x + 5, img_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_coordinates_top_view.append(img_y)

 
        # Plot histogram for front view (x-coordinates)
        """ ####
        1280 pixeis -> 600 centimetros
        600 / 120 = 5 cm
        5 cm -> 10,7 pixeis

        logo uso 18 vizinhos para perceber se a deteção do armário está top (porque o armário mede 90cm)
        #### """
        bins_front = 120
        plt.figure(figsize=(10, 6))
        plt.hist(y_coordinates_front_view, bins=bins_front, range=(0, width), color='blue', alpha=0.7)
        plt.title("Histogram of X-coordinates in Front View")
        plt.xlabel("Pixel X-coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.savefig('histogram_front_view.png')
        plt.close()

        # Calculate the histogram and find peaks for front view
        hist_front, bins_front = np.histogram(y_coordinates_front_view, bins=bins_front, range=(0, width))
        threshold = np.max(hist_front) * 0.8
        peak_indices_front = np.where(hist_front >= threshold)[0]

        # Define the range of neighbors to consider
        neighbor_ranges = [(2, 17), (17, 2), (9, 9)]

        # Store neighbor counts for front view peaks
        neighbor_counts_front = {}

        # Initialize variables to track the best peak and its associated neighbor range
        best_peak_index_front = None
        highest_count_across_peaks_front = 0
        best_neighbor_range_front = None
        left_bound_front = None
        right_bound_front = None
        left_bound_final_front = None
        right_bound_final_front = None

        for index in peak_indices_front:
            best_range_front = None
            highest_sum_front = 0
            neighbor_counts = []

            for left, right in neighbor_ranges:
                start_index_front = max(0, index - left)
                end_index_front = min(len(hist_front) - 1, index + right)
                total_frequency = np.sum(hist_front[start_index_front:end_index_front + 1])

                # Calculate number of objects in this range
                left_bound_front_current = bins_front[start_index_front]
                right_bound_front_current = bins_front[end_index_front + 1]
                objects_in_range_front = np.sum((y_coordinates_front_view >= left_bound_front_current) & (y_coordinates_front_view < right_bound_front_current))

                neighbor_counts.append(objects_in_range_front)

                if total_frequency > highest_sum_front:
                    highest_sum_front = total_frequency
                    best_range_front = (start_index_front, end_index_front)
                    left_bound_front = left_bound_front_current
                    right_bound_front = right_bound_front_current

            # Store neighbor counts for this peak index
            neighbor_counts_front[index] = neighbor_counts

            # Find the highest count across all neighbor ranges for this peak index
            max_count_for_peak_front = max(neighbor_counts)
            
            # Check if this peak index has a higher count than previously found peaks
            if max_count_for_peak_front > highest_count_across_peaks_front:
                highest_count_across_peaks_front = max_count_for_peak_front
                best_peak_index_front = index
                
                # Determine which neighbor range had the highest count for this peak index
                best_neighbor_range_index_front = np.argmax(neighbor_counts)
                best_neighbor_range_front = neighbor_ranges[best_neighbor_range_index_front]
                left_bound_final_front = left_bound_front
                right_bound_final_front = right_bound_front

        # Print the best peak and its highest count and selected neighbor range
        print('--------------------')
        print(f"Best peak in front view histogram: {bins_front[best_peak_index_front]}")
        print(f"Highest count: {highest_count_across_peaks_front}")
        print(f"Selected neighbor range: {best_neighbor_range_front}")
        print(f"Left bound of selected range: {left_bound_final_front}")
        print(f"Right bound of selected range: {right_bound_final_front}")
        print("Neighbor counts for front view peaks:")
        for index, counts in neighbor_counts_front.items():
            print(f"Peak at bin {bins_front[index]}: {counts}")

        """ ####
        720 pixeis -> 500 centimetros
        500 / 50 = 5 cm
        5 cm -> 7,2 pixeis

        logo uso 6 vizinhos para perceber se a deteção do armário está top (porque o armário mede 30cm em profundidade)
        #### """
        
        # Plot histogram for top view (z-coordinates)
        bins_top = 144
        plt.figure(figsize=(10, 6))
        plt.hist(y_coordinates_top_view, bins=bins_top, range=(0, height), color='green', alpha=0.7)
        plt.title("Histogram of Z-coordinates in Top View")
        plt.xlabel("Pixel Z-coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.savefig('histogram_top_view.png')
        plt.close()

        # Calculate the histogram
        hist_top, bins_top = np.histogram(y_coordinates_top_view, bins=bins_top, range=(0, height))

        # Determine the threshold for peak detection
        threshold = np.max(hist_top) * 0.8
        peak_indices_top = np.where(hist_top >= threshold)[0]

        # Define the range of neighbors to consider
        neighbor_ranges = [(2, 8), (8, 2), (5, 5)]

        # Initialize variables to track best peak and its properties
        best_peak_index_top = None
        highest_neighbor_count_top = 0
        selected_neighbor_range_top = None
        neighbor_counts_top = {}
        left_bound_top = None
        right_bound_top = None
        left_bound_final = 0
        right_bound_final = 0

        # Loop through each peak index identified in the histogram
        for index in peak_indices_top:
            best_range_top = None
            highest_sum_top = 0
            neighbor_counts = []

            # Evaluate different neighbor ranges for this peak index
            for left, right in neighbor_ranges:
                start_index_top = max(0, index - left)
                end_index_top = min(len(hist_top) - 1, index + right)
                
                # Calculate total frequency in the current range
                total_frequency = np.sum(hist_top[start_index_top:end_index_top + 1])

                # Calculate number of objects in this range
                left_bound_top_current = bins_top[start_index_top]
                right_bound_top_current = bins_top[end_index_top + 1]
                objects_in_range_top = np.sum((y_coordinates_top_view >= left_bound_top_current) & (y_coordinates_top_view < right_bound_top_current))

                neighbor_counts.append(objects_in_range_top)

                # Track the highest total frequency and its associated range
                if total_frequency > highest_sum_top:
                    highest_sum_top = total_frequency
                    best_range_top = (start_index_top, end_index_top)
                    left_bound_top = left_bound_top_current  # Update left bound
                    right_bound_top = right_bound_top_current  # Update right bound

            # Store neighbor counts for this peak
            neighbor_counts_top[index] = neighbor_counts

            # Check for the highest count within neighbor counts
            max_count = max(neighbor_counts)
            if max_count > highest_neighbor_count_top:
                highest_neighbor_count_top = max_count
                best_peak_index_top = index
                selected_neighbor_range_top = neighbor_ranges[np.argmax(neighbor_counts)]
                left_bound_final = left_bound_top
                right_bound_final = right_bound_top


        # Print the results
        print('--------------------')
        print(f"Best peak in top view histogram: {bins_top[best_peak_index_top]}")
        print(f"Highest count: {highest_neighbor_count_top}")
        print(f"Selected neighbor range: {selected_neighbor_range_top}")
        print(f"Left bound of selected range: {left_bound_final}")
        print(f"Right bound of selected range: {right_bound_final}")
        print("Neighbor counts for top view peaks:")
        for index, counts in neighbor_counts_top.items():
            print(f"Peak at bin {bins_top[index]}: {counts}")


        # Filter coordinates to keep only those within the common range
        filtered_coordinates = []
        for obj in coordinates:
            img_x = int((obj.position_relative.x - min_width_meters) / (max_width_meters - min_width_meters) * width)
            img_y = height - int((obj.position_relative.y - min_height_meters_y) / (max_height_meters_y - min_height_meters_y) * height)
            img_z = height - int((obj.position_relative.z - min_height_meters) / (max_height_meters - min_height_meters) * height)

            img_x = center_x + (img_x - center_x)
            img_y = center_y - (img_y - center_y)
            img_z = center_y + (img_z - center_y)
            # print('name:', obj.object_name)
            # print('- :', img_x, img_y, img_z)
            # print('- :', left_bound_front, right_bound_front, left_bound_top, right_bound_top)

            if left_bound_final_front <= img_x <= right_bound_final_front and left_bound_final <= img_y <= right_bound_final:
                # print('- :', obj.object_name)
                filtered_coordinates.append(obj)

        # Plotting the peak regions in histograms
        plt.figure(figsize=(10, 6))
        plt.hist(y_coordinates_front_view, bins=bins_front, range=(0, width), color='blue', alpha=0.7)
        plt.axvline(left_bound_final_front, color='red', linestyle='--', linewidth=2, label='Left Bound')
        plt.axvline(right_bound_final_front, color='green', linestyle='--', linewidth=2, label='Right Bound')
        plt.title("Histogram of X-coordinates in Front View with Peak Region")
        plt.xlabel("Pixel X-coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.legend()
        plt.savefig('histogram_front_view_peak.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.hist(y_coordinates_top_view, bins=bins_top, range=(0, height), color='green', alpha=0.7)
        plt.axvline(left_bound_final , color='red', linestyle='--', linewidth=2, label='Left Bound')
        plt.axvline(right_bound_final, color='green', linestyle='--', linewidth=2, label='Right Bound')
        plt.title("Histogram of Z-coordinates in Top View with Peak Region")
        plt.xlabel("Pixel Z-coordinate")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.legend()
        plt.savefig('histogram_top_view_peak.png')
        plt.close()

        time.sleep(1)
        # Load and display the saved histogram images using OpenCV
        hist_front_view_peak = cv2.imread('histogram_front_view_peak.png')
        hist_top_view_peak = cv2.imread('histogram_top_view_peak.png')

        cv2.imshow("Top view of cabinet", image2)
        cv2.waitKey(100)

        cv2.imshow("Histogram of Z-coordinates in Top View with Peak Region", hist_top_view_peak)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        cv2.imshow("Front view of cabinet", image)
        cv2.waitKey(100)

        cv2.imshow("Histogram of X-coordinates in Front View with Peak Region", hist_front_view_peak)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print(f"Front view: {left_bound_front} to {right_bound_front}")
        print(f"Top view: {left_bound_top} to {right_bound_top}")

        # Return the filtered coordinates
        return filtered_coordinates

    def choose_place_arm(self, shelf, side):
        use_arm = False
        
        height_arm = 0.0
        if side == 'left':
            print('left side')
            if shelf == 1:
                print('first shelf')
                use_arm = False
            elif shelf == 2:
                print('second shelf')
                height_arm = self.shelf_2_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_second_shelf_left_side",adjust_position=a[1], wait_for_end_of=True)
                use_arm = True
            elif shelf == 3:
                print('third shelf')
                height_arm = self.shelf_3_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_third_shelf_left_side", adjust_position=a[1], wait_for_end_of=True)
                use_arm = True
            elif shelf == 4:
                print('fourth shelf')
                height_arm = self.shelf_4_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_fourth_shelf_left_side", adjust_position=a[1], wait_for_end_of=True)
                use_arm = True

        elif side == 'right':
            print('right side')
            if shelf == 1:
                print('first shelf') 
                use_arm = False
            elif shelf == 2:
                print('second shelf')
                height_arm = self.shelf_2_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_second_shelf_right_side", adjust_position=a[1], wait_for_end_of=True)
                use_arm = True
            elif shelf == 3:
                print('third shelf')
                height_arm = self.shelf_3_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_third_shelf_right_side", adjust_position=a[1], wait_for_end_of=True)
                use_arm = True
            elif shelf == 4:
                print('fourth shelf')
                height_arm = self.shelf_4_height + 0.2
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_arm(command="place_cabinet_fourth_shelf_right_side", adjust_position=a[1], wait_for_end_of=True)
                use_arm = True

        return use_arm

    def activate_yolo_objects(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5, wait_for_end_of=True):
        
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, activate_objects_hand=activate_objects_hand, activate_shoes_hand=activate_shoes_hand, activate_doors_hand=activate_doors_hand, minimum_objects_confidence=minimum_objects_confidence, minimum_shoes_confidence=minimum_shoes_confidence, minimum_doors_confidence=minimum_doors_confidence)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def Rot(self, eixo, angulo):
        ang_rad = angulo*math.pi/180.0
        c = math.cos(ang_rad)
        s = math.sin(ang_rad)
        M = np.identity(4)
        if (eixo == 'x' or eixo == 'X'):
            M[1][1] = M[2][2] = c
            M[1][2] = -s
            M[2][1] = s
        elif (eixo == 'y' or eixo == 'Y'):
            M[0][0] = M[2][2] = c
            M[0][2] = s
            M[2][0] = -s
        elif (eixo == 'z' or eixo == 'Z'):
            M[0][0] = M[1][1] = c
            M[0][1] = -s
            M[1][0] = s
        return M
        
    def Trans(self, tx, ty, tz):
        M = np.identity(4)
        M[0][3] = tx
        M[1][3] = ty
        M[2][3] = tz
        return M

    def transform(self, obj):
        # C representa o ponto no espaço para onde eu quero transformar a base do braço do robô
        # A matriz de transformação desde a  base do braço até ao centro do Robot pode ser representada por:
        # T = Rot(z, 180) * Rot (x, -90) * Trans (3, -6, -110)
        # a2 representa a translação desde a base do braço até ao centro do robô  (em cm)
        # a1 representa a rotação sobre o eixo coordenadas x em -90º para alinhar os eixos coordenados
        # a0 representa a rotação sobre o eixo coordenadas z em 180º para alinhar o eixo dos x 
        # c representa o ponto (x,y,z) em relação ao centro do braço
        
        

        ### nos numeros que chegam: x representa a frente do robô. y positivo vai para a esquerda do robô. z vai para cima no robô
        ### nos numeros que saem: x vai para trás do robô. y vai para baixo no robô. z vai para a direita do robô
        
        
        ### PARECE-ME QUE X E Z ESTÃO TROCADOS NO RESULTADO QUE TENHO EM RELAºÃO AO BRAÇO
        print('\n\n')
    
        c = np.dot(np.identity(4), [0, 0, 0, 1])
        # c = np.dot(np.identity(4), [90.0, -30.0, 105.0, 1])
        ### ESTAS TRANSFORMAÇÕES SEGUINTES SÃO NECESSÁRIAS PORQUE O ROBOT TEM EIXO COORDENADAS COM Y PARA A FRENTE E X PARA A DIREITA E AS TRANSFORMAÇÕES DA CAMARA SÃO FEITAS COM X PARA A FRENTE Y PARA A ESQUERDA
        new_x = 0.0
        new_y = -0.0
        new_z = obj * 1000
        c = np.dot(np.identity(4), [new_x, new_y, new_z, 1])
        print(f'Posição em relação ao solo:[{new_x:.2f}, {new_y:.2f}, {new_z:.2f}]')
        a2 = self.Trans(30.0, -60.0, -1100.0)
        a1 = self.Rot('x', -90.0)
        a0 = self.Rot('z', 180.0)
        T = np.dot(a0, a1)
        T = np.dot(T, a2)
        
        #print('T', T)
        
        AA = np.dot(T, c)
        
        print('Ponto em relação ao braço:', AA)


        # aux = AA[0]
        # AA[0] = AA[2]
        # AA[2] = aux

        # AA[0] = AA[0] * 10
        # AA[1] = AA[1] * 10
        # AA[2] = AA[2] * 10
        # my_formatted_list = [ '%.2f' % elem for elem in AA ]
        ### VALOR DO Z ESTÀ INVERSO AO QUE EU DEVO PASSAR PARA O BRAÇO EM AA !!!
        
        # print('Ponto em relação ao braço:', AA)
        # print('y = ', AA[1]*10)

        print('\n\n')

        return AA

    def main(self):

        print("IN NEW MAIN")
        # time.sleep(1)

        # Navigation Coordinates
        """## LAR
        self.front_of_door = [0.0, 1.5] 
        self.outside_kitchen_door = [-0.7, 5.5]
        self.inside_kitchen_door = [-0.7, 7.5] """

        self.front_of_door = [0.3, 2.5]
        self.almost_kitchen = [1.7, 5.3]
        self.inside_kitchen = [1.7, 7.0]
        self.cabinet = [-1.5, 7.5]
       
        while True:

            if self.state == self.Waiting_for_task_start:

                time.sleep(1)
                #print('State 0 = Initial')

                #self.set_face("charmie_face")
                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")

                time.sleep(1)

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_arm(command="open_gripper", wait_for_end_of=True)

                data = []
                real_data = []
                # tetas = [[-120, -10], [-60, -10], [0, -10], [60, -10], [120, -10]]
                tetas = [[0, -45], [0, -30], [0, -15], [0, 0]]
                objects_found = self.search_for_objects(tetas=tetas, delta_t=3.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False)

                for o in objects_found:
                    
                    print(o.index, o.object_name, "\t", 
                        round(o.position_absolute.x, 2), round(o.position_absolute.y, 2), 
                        round(o.position_absolute.z, 2)) # round(o.box_center_x), round(o.box_center_y)
                    name = o.object_name
                    x = round(o.position_absolute.x, 2)
                    y = round(o.position_absolute.y, 2)
                    z = round(o.position_absolute.z, 2)
                    obj_class = o.object_class
                    # print('-- \n ', o.position_relative.z, '\n')
                    data.append((name, x, y, z, obj_class))
                    real_data.append(o)
                
                print('Nr de objetos: ', len(data))

                # print(self.node.filtered_objects_storing_groceries)
                # print('waiting...')
                # while self.node.flag_storing_groceries_received == False:
                #     pass
                # time.sleep(2)
                
                # for o in self.node.filtered_objects_storing_groceries:
                    
                #     print(o.index, o.object_name, "\t", 
                #         round(o.position_absolute.x, 2), round(o.position_absolute.y, 2), 
                #         round(o.position_absolute.z, 2)) # round(o.box_center_x), round(o.box_center_y)
                #     name = o.object_name
                #     x = round(o.position_absolute.x, 2)
                #     y = round(o.position_absolute.y, 2)
                #     z = round(o.position_absolute.z, 2)
                #     data.append((name, x, y, z))

                # self.node.flag_storing_groceries_received = False
                # self.node.filtered_objects_storing_groceries.clear()
                # print('---', data)

                filtered_objects = self.plot_histograms(real_data)

                coord_y = 0.0
                coord_z = 0.0
                for obj in filtered_objects:
                    coord_y += obj.position_relative.y
                    coord_z += obj.position_relative.z
                    print(obj.position_relative.y, obj.position_relative.z)

                print('average depth: ', coord_y/len(filtered_objects))
                print('average height: ', coord_z/len(filtered_objects))
                distance_y_to_navigate = coord_y/len(filtered_objects) - 0.9
                print('I must navigate for: ', distance_y_to_navigate)                

                # for obj in filtered_objects:
                #     print('Filtered objects:', obj.object_name)
                print('nr de objetos filtrados: ', len(filtered_objects))

                # self.objects_left_side = []
                # self.objects_right_side = []
                # self.objects_first_shelf_rs = []
                # self.objects_first_shelf_ls = []
                # self.objects_second_shelf_rs = []
                # self.objects_second_shelf_ls = []
                # self.objects_third_shelf_rs = []
                # self.objects_third_shelf_ls = []
                # self.objects_fourth_shelf_rs = []
                # self.objects_fourth_shelf_ls = []
                # self.objects_fifth_shelf_rs = []
                # self.objects_fifth_shelf_ls = []
                # self.objects_sixth_shelf_rs = []
                # self.objects_sixth_shelf_ls = []
                self.high_priority_class = []
                self.medium_priority_class = []


                self.analysis_cabinet(filtered_objects)

                self.choose_priority(filtered_objects)

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_y_to_navigate, adjust_direction=0.0, wait_for_end_of=True)
                #self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.15, adjust_direction=90.0, wait_for_end_of=True)

                tetas = [[120, -30], [120, -15]]
                objects_found_table = self.search_for_objects(tetas=tetas, delta_t=3.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False)

                print('Objects found on table: ')
                for obj in objects_found_table:
                    print(obj.object_name)

                print('---------')

                table_objects = self.analysis_table(objects_found_table)
                print('Objects filtered from table: ')
                for obj in table_objects:
                    print(obj.object_name)


                # print('High class:')
                # for obj in self.high_priority_class:
                #     print(obj)
                
                # print('Medium class:')
                # for obj in self.medium_priority_class:
                #     print(obj)


                print('---', self.shelf_side_common_class.items())

                self.set_neck(position=self.look_back, wait_for_end_of=False)

                # self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)
                # time.sleep(10)
                # self.set_arm(command="close_gripper", wait_for_end_of=True)

                

                # self.set_arm(command="arm_front_robot", wait_for_end_of=True)

                print('Choosed objects in table: ')
                for obj in table_objects:
                    print(obj.object_name, obj.object_class)

                    # Output results
                    for key, common_class in self.shelf_side_common_class.items():
                        shelf, side = key
                        print('Inside loop: ', shelf, side, common_class)
                        # print(f"Shelf {shelf}, Side {side} - Most Common Class: {common_class}")
                        # print(f"Objects: {[obj.object_name for obj in self.shelf_side_objects[key]]}")
                        if obj.object_class == common_class:
                            path_to_image = self.detected_object_to_face_path(object=obj, send_to_face=False)
                            print(path_to_image)
                            cf = self.node.br.imgmsg_to_cv2(obj.image_rgb_frame, "bgr8")

                            cv2.rectangle(cf, (obj.box_top_left_x, obj.box_top_left_y), (obj.box_top_left_x + obj.box_width, obj.box_top_left_y + obj.box_height), (255,0,0), 2)


                            cv2.imshow('table object', cf)
                            cv2.waitKey(0) 
                            print(f"{obj.object_name} goes to {shelf} shelf, {side} side")
                            self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)
                            time.sleep(10)
                            self.set_arm(command="close_gripper", wait_for_end_of=True)
                            self.set_arm(command="arm_front_robot", wait_for_end_of=True)
                            self.set_arm(command="arm_front_robot_linear", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_LONG)
                            use_arm = self.choose_place_arm(shelf, side)
                            if use_arm == True:
                                self.set_rgb(command=GREEN+BLINK_LONG)
                                self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.3, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_arm(command="open_gripper", wait_for_end_of=True)
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.3, adjust_direction=180.0, wait_for_end_of=True)
                                self.set_arm(command="arm_front_robot_linear", wait_for_end_of=True)
                                self.set_arm(command="arm_front_robot", wait_for_end_of=True)
                                
                                
                            else:
                                print(f'{obj.object_name} goes to the {shelf} shelf, on the {side} side')
                                self.set_rgb(command=RED+BLINK_LONG)
                            

                        # else:
                        #     print('resolver caso sem prioridade')

                    
                """ 
                ### TO DO:
                - COLOCAR ORIENTATE A FUNCIONAR DIREITINHO PARA FICAR A 0.0 GRAUS
                - COLOCAR SPEECH PICK AND PLACE
                - COLOCAR IMAGEM NA CARA
                - TRATAR DE ABRIR A PORTA
                - FAZER ADJUST EM X E Y COM ARMÁRIO
                - RETIRAR POSIÇÃO INTERMÉDIA
                - FAZER DICIONÁRIO DE ALTURAS PARA CADA OBJETO
                - AO FECHAR MÃO, SE NÃO TIVER NADA AVANÇO PARA O SEGUINTE
                """
                while True:
                    pass
                
                self.set_speech(filename="storing_groceries/sg_ready_start", wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True) # must change to door open

                self.set_rgb(command=MAGENTA+SET_COLOUR)

                self.wait_for_start_button()

                self.set_rgb(command=BLUE+SET_COLOUR)

                ###### WAITS FOR START BUTTON / DOOR OPEN
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
                         
                self.wait_for_door_start()

                time.sleep(3.5)
                # next state
                # self.state = self.Approach_tables_first_time

                # self.state = 0 
                self.state = self.Approach_cabinet_first_time

            elif self.state == self.Approach_cabinet_first_time:
                #print('State 5 = Approaching cabinet for the first time')

                # self.set_speech(filename="storing_groceries/sg_collected_objects_1st_round", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                self.set_speech(filename="generic/moving_cabinet", wait_for_end_of=True)

                ###### MOVEMENT TO THE CABINET

                # self.set_navigation(movement="move", target=self.front_of_door, flag_not_obs=True, wait_for_end_of=True)
                # # self.set_navigation(movement="rotate", target=self.almost_kitchen, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.almost_kitchen, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="rotate", target=self.inside_kitchen, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.inside_kitchen, flag_not_obs=False, wait_for_end_of=True)
                # self.set_navigation(movement="rotate", target=self.cabinet, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.cabinet, flag_not_obs=False, wait_for_end_of=True)
                # self.set_navigation(movement="orientate", absolute_angle= 85.0, flag_not_obs = True, wait_for_end_of=True)

                


                self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)
               

                nr_classes_detected = 0
                list_of_neck_position_search = [[0, 0], [0, 10], [0, 15], [0, 20], [0, 25], [0, 30]]
                position_index = 0

                self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)


                while nr_classes_detected < 2:
                    
                    










                    print('\n \n \n \n')

                    pos_offset = list_of_neck_position_search[position_index]
                    new_neck_pos = [self.look_cabinet_center[0] + pos_offset[0], self.look_cabinet_center[1] + pos_offset[1]]
                    print('pescoço: ', new_neck_pos)
                    
                    # Set the neck position
                    self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                    print('Neck: ', new_neck_pos)
                    time.sleep(3)
                    if position_index == 0:
                        self.set_speech(filename="storing_groceries/sg_analysing_object_cabinet", wait_for_end_of=True)

                    self.current_image = self.node.image
                    bridge = CvBridge()
                    # Convert ROS Image to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                    self.image_most_obj_detected= cv_image

                    nr_classes_detected = self.analysis_cabinet()

                    self.current_image = self.node.image
                    bridge = CvBridge()
                    cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                    current_frame_draw = cv_image.copy()

                    """ cv2.imshow('A', current_frame_draw)
                    cv2.waitKey(0) """
                    
                    if nr_classes_detected is None:
                        nr_classes_detected = 0

                    if nr_classes_detected < 2:
                        self.set_rgb(command=RED+BLINK_LONG)
                        nr_classes_detected = 0

                    # Move to the next position
                    position_index = (position_index + 1) % len(list_of_neck_position_search)
                    print(position_index)

                    # Adicionar ajuste de pescoço ou então depois ajustar dentro do próprio analysis cabinet
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_priority()
                
                self.set_speech(filename="storing_groceries/sg_finished_analise_cabinet", wait_for_end_of=True) 

                # self.set_arm(command="arm_up_a_bit", wait_for_end_of=True)

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                # next state
                self.state = self.Approach_tables_first_time

            elif self.state == self.Approach_tables_first_time:
                #print('State 1 = Approaching table for the first time')

                self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                # self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                ###### MOVEMENT TO THE KITCHEN COUNTER

                # self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)

                self.set_neck(position=self.look_table_objects)

                nr_objects_detected_high_priority = 0
                i = 0

                while nr_objects_detected_high_priority < 5:
                    self.current_image = self.node.image
                    bridge = CvBridge()
                    # Convert ROS Image to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                    self.image_objects_detected = cv_image

                    # nr_objects_detected_high_priority = self.analysis_table()
                    nr_objects_detected_high_priority = self.detect_table_objects()


                self.select_five_objects(self.objects_stored)
                # self.set_speech(filename="storing_groceries/sg_detected", wait_for_end_of=True) 
                
                # self.select_five_objects()

                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)
                # next state
                self.state = self.Picking_first_object
                
            elif self.state == self.Picking_first_object:
                #print('State 2 = Picking first object from table')

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)

                object_= self.selected_objects[self.object_counter]
                obj_name = object_.object_name
                print(obj_name)
                obj_name_lower = obj_name.lower().replace(" ", "_")
                print(obj_name_lower)

                object_help_pick = 'help_pick_' + obj_name_lower
                # self.set_face(str(object_help_pick))
                print(object_help_pick)

                # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.load_image_one_object(obj_name, object_)

                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True) 

                time.sleep(2)

                self.set_speech(filename="storing_groceries/place_tray", wait_for_end_of=True)
                
                # self.create_image_five_objects_same_time(self.selected_objects)
                self.activate_yolo_objects(activate_objects=False)
                               
                # time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                """ object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                        self.set_rgb(command=RED+BLINK_LONG)
                
                        self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=True)
                    
                    else:
                        self.set_rgb(command=GREEN+BLINK_LONG)
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False) """
                # self.set_face("charmie_face")

                self.object_counter += 1

                time.sleep(2)
                                            
                # next state
                self.state = self.Picking_second_object
                
            

            elif self.state == self.Picking_second_object:
                
                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)

                object_= self.selected_objects[self.object_counter]
                obj_name = object_.object_name
                print(obj_name)
                obj_name_lower = obj_name.lower().replace(" ", "_")
                print(obj_name_lower)

                object_help_pick = 'help_pick_' + obj_name_lower
                # self.set_face(str(object_help_pick))
                print(object_help_pick)

                # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.load_image_one_object(obj_name, object_)

                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True) 

                time.sleep(2)

                self.set_speech(filename="storing_groceries/place_tray", wait_for_end_of=True)
                
                # self.create_image_five_objects_same_time(self.selected_objects)
                self.activate_yolo_objects(activate_objects=False)
                               
                # time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                """ object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                        self.set_rgb(command=RED+BLINK_LONG)
                
                        self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=True)
                    
                    else:
                        self.set_rgb(command=GREEN+BLINK_LONG)
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False) """
                # self.set_face("charmie_face")

                self.object_counter += 1

                time.sleep(2)

                self.state = self.Picking_third_object
                
            elif self.state == self.Picking_third_object:
                
                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)

                object_= self.selected_objects[self.object_counter]
                obj_name = object_.object_name
                print(obj_name)
                obj_name_lower = obj_name.lower().replace(" ", "_")
                print(obj_name_lower)

                object_help_pick = 'help_pick_' + obj_name_lower
                # self.set_face(str(object_help_pick))
                print(object_help_pick)

                # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.load_image_one_object(obj_name, object_)

                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True) 

                time.sleep(2)

                self.set_speech(filename="storing_groceries/place_tray", wait_for_end_of=True)
                
                # self.create_image_five_objects_same_time(self.selected_objects)
                self.activate_yolo_objects(activate_objects=False)
                               
                # time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                """ object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                        self.set_rgb(command=RED+BLINK_LONG)
                
                        self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=True)
                    
                    else:
                        self.set_rgb(command=GREEN+BLINK_LONG)
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False) """
                # self.set_face("charmie_face")

                self.object_counter += 1

                time.sleep(2)

                self.state = self.Picking_fourth_object

            elif self.state == self.Picking_fourth_object:
                
                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)

                object_= self.selected_objects[self.object_counter]
                obj_name = object_.object_name
                print(obj_name)
                obj_name_lower = obj_name.lower().replace(" ", "_")
                print(obj_name_lower)

                object_help_pick = 'help_pick_' + obj_name_lower
                # self.set_face(str(object_help_pick))
                print(object_help_pick)

                # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.load_image_one_object(obj_name, object_)

                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True) 

                time.sleep(2)

                self.set_speech(filename="storing_groceries/place_tray", wait_for_end_of=True)
                
                # self.create_image_five_objects_same_time(self.selected_objects)
                self.activate_yolo_objects(activate_objects=False)
                               
                # time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                """ object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                        self.set_rgb(command=RED+BLINK_LONG)
                
                        self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=True)
                    
                    else:
                        self.set_rgb(command=GREEN+BLINK_LONG)
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False) """
                # self.set_face("charmie_face")

                self.object_counter += 1

                time.sleep(2)

                self.state = self.Picking_fifth_object

            elif self.state == self.Picking_fifth_object:
                
                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)

                object_= self.selected_objects[self.object_counter]
                obj_name = object_.object_name
                print(obj_name)
                obj_name_lower = obj_name.lower().replace(" ", "_")
                print(obj_name_lower)

                object_help_pick = 'help_pick_' + obj_name_lower
                # self.set_face(str(object_help_pick))
                print(object_help_pick)

                # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.load_image_one_object(obj_name, object_)

                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True) 

                time.sleep(2)

                self.set_speech(filename="storing_groceries/place_tray", wait_for_end_of=True)
                
                # self.create_image_five_objects_same_time(self.selected_objects)
                self.activate_yolo_objects(activate_objects=False)
                               
                # time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                """ object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                        self.set_rgb(command=RED+BLINK_LONG)
                
                        self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=True)
                    
                    else:
                        self.set_rgb(command=GREEN+BLINK_LONG)
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False) """
                self.set_face("charmie_face")

                self.object_counter += 1

                time.sleep(2)

                self.state = self.Placing_first_object

            elif self.state == self.Placing_first_object:
                self.object_counter = 0
                # self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                # self.set_speech(filename="generic/moving_cabinet", wait_for_end_of=True)
                time.sleep(1)
                # MOVIMENTAR
                # self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                time.sleep(3.5)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Placing_second_object

            elif self.state == self.Placing_second_object:
                
                time.sleep(3)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                time.sleep(3.5)

                self.state = self.Placing_third_object

           

            elif self.state == self.Placing_third_object:
 
                time.sleep(3)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                time.sleep(3.5)

                self.state = self.Placing_fourth_object

            
            elif self.state == self.Placing_fourth_object:
         
                time.sleep(3)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                time.sleep(3.5)

                self.state = self.Placing_fifth_object

            

            elif self.state == self.Placing_fifth_object:
                
                time.sleep(3)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                time.sleep(3.5)

                self.state = self.Final_State
            
            elif self.state == self.Final_State:
                #print('State 15 = Finished task')
                # self.node.speech_str.command = "I have finished my storing groceries task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                self.set_rgb(command=RAINBOW_ROT)
                
                # self.set_neck(position=self.look_judge) # , wait_for_end_of=True)

                self.set_speech(filename="storing_groceries/sg_finished", wait_for_end_of=True)

                while True:
                    pass

            else:
                # self.get_caracteristics_image()
                # start_time = time.time()

                
                
                """ print(' ---------------------- ')

                # while time.time() - start_time < 5.0:
                self.analysis_cabinet()
                self.choose_priority() 


                #print(self.object_details)

                print(' ---------------------- ')

                #print(self.object_position)

                time.sleep(5)
                self.set_speech(filename="storing_groceries/sg_detected", wait_for_end_of=True)
                self.select_five_objects() #Called with Choose_priority routine. Without it I just comment it

                if self.image_most_obj_detected is not None:
                    cv2.imshow('Image with all objects detected', self.image_most_obj_detected)
                    cv2.waitKey(1)
                else:
                    print("Error: self.image_most_obj_detected is None")

                self.object_details.clear()
                self.object_details = {} """


                pass