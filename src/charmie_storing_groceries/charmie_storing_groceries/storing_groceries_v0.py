#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from example_interfaces.msg import Bool, String, Int16, Float32
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from charmie_interfaces.srv import SpeechCommand, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, ArmTrigger, ActivateYoloObjects, NavTrigger, SetFace, ActivateObstacles
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

        # Camera
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_hand_image_callback, 10)
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_callback, 10)
        
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
        self.arm_pose_subscriber = self.create_subscription(ArmController, 'arm_current_pose', self.get_arm_current_pose_callback, 10)

        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")

        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")

        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        # # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        # Obstacles
        while not self.activate_obstacles_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Activate Obstacles Command...")
        # Face
        # while not self.face_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Face Command...")
        # Arm (CHARMIE)
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
        self.first_depth_image_received = False
        self.first_depth_image_hand_received = False
        self.new_image_hand_flag = False

        self.get_neck_position = [1.0, 1.0]
        self.objects_classNames_dict = {}

        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        
        self.objects_classNames_dict = {item["name"]: item["class"] for item in self.objects_file}
        self.detected_objects = Yolov8Objects()
        self.detected_doors = Yolov8Objects()
        self.detected_shoes = Yolov8Objects()
        self.detected_objects_hand = Yolov8Objects()
        self.detected_doors_hand = Yolov8Objects()
        self.detected_shoes_hand = Yolov8Objects()
        self.obstacles = Obstacles()
        self.filtered_objects_storing_groceries = []
        self.flag_storing_groceries_received = False
        self.br = CvBridge()
        self.activate_obstacles_success = True
        self.activate_obstacles_message = ""
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

    def get_aligned_depth_image_callback(self, img: Image):

        self.depth_img = img
        self.first_depth_image_received = True

    def get_aligned_depth_hand_image_callback(self, img: Image):
        self.depth_img_hand = img
        self.first_depth_image_hand_received = True
        self.new_image_hand_flag = True
        # print("Received Depth Image")

    def get_color_image_callback(self, img: Image):

        self.rgb_img = img
    
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

    ### ACTIVATE OBSTACLES SERVER FUNCTIONS ###
    def call_activate_obstacles_server(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False):
        request = ActivateObstacles.Request()
        request.activate_lidar_up = obstacles_lidar_up
        request.activate_lidar_bottom = obstacles_lidar_bottom
        request.activate_camera_head = obstacles_camera_head

        self.activate_obstacles_client.call_async(request)

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

    def get_arm_current_pose_callback(self, arm_pose: ArmController):
        self.arm_current_pose = arm_pose.pose

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
        
        self.ATTEMPTS_AT_RECEIVING = 2

        self.object_counter = 0

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_back = [180,0]
        self.look_navigation = [0, -30]
        self.look_judge = [90, 0]
        self.look_table_objects = [160, -20]
        self.look_table_objects_front = [0, -40]
        self.look_tray = [0, -60]
        self.look_cabinet_top = [-45, 45]
        self.look_cabinet_center = [0, -30]
        self.look_cabinet_bottom = [-45, -45]

        self.shelf_1_height = 0.05 # 0.15 # 0.14 # 0.15
        self.shelf_2_height = 0.37  # 0.60 # 0.55 # 0.60 
        self.shelf_3_height = 0.68  # 1.10 # 0.97 # 1.10 
        self.shelf_4_height = 1.00  # 1.39
        self.shelf_5_height = 1.35
        self.shelf_6_height = 1.64

        self.shelf_1_height_to_place = 0.12 
        self.shelf_2_height_to_place = 0.45 
        self.shelf_3_height_to_place = 0.75 + 0.05
        self.shelf_4_height_to_place = 1.08 + 0.05
        self.shelf_5_height_to_place = 1.43 + 0.05
        self.shelf_6_height_to_place = 1.72

        self.wardrobe_width = 0.8
        self.door_width = 0.4
        self.wardrobe_depth = 0.27
        self.robot_radius = 0.28

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        self.kitchen_counter = [-0.4, 5.5]
        self.kitchen_table = [-1.5, 6.8]

        # to debug just a part of the task you can just change the initial state, example:

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
        MAX_DOOR_ANGLE = math.radians(10.0)
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
    
    def set_face(self, command="", custom="", wait_for_end_of=False):
        
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
        if not list_of_objects_detected_as:
            list_of_objects_detected_as = [None] * len(list_of_objects)
            
        mandatory_object_detected_flags = [False for _ in list_of_objects]
        # print(mandatory_object_detected_flags)
        DETECTED_ALL_LIST_OF_OBJECTS = False
        MIN_DIST_DIFFERENT_FRAMES = 0.3 # maximum distance for the robot to assume it is the same objects
        MIN_DIST_SAME_FRAME = 0.2

        merged_lists = []
        for obj, detected_as in zip(list_of_objects, list_of_objects_detected_as):
            if detected_as != None:
                merged_lists.append([obj] + detected_as)
            else:
                merged_lists.append([obj])
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
                    for o in final_objects:
                        for m in range(len(merged_lists)):
                            if o.object_name.replace(" ","_").lower() in merged_lists[m]:
                                o.object_name = list_of_objects[m]

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
  
    def analysis_cabinet(self, filtered_objects):
        # Step 1: Categorize Objects
        self.shelf_side_objects = defaultdict(list)
        self.shelf_side_class_counts = defaultdict(Counter)
        self.shelf_side_common_class = {}
        
        for obj in filtered_objects:
            side = 'left' if obj.position_relative.x < 0 else 'right'
            if self.shelf_1_height < obj.position_relative.z < self.shelf_2_height:
                shelf = 1
            elif self.shelf_2_height < obj.position_relative.z < self.shelf_3_height:
                shelf = 2
            elif self.shelf_3_height < obj.position_relative.z < self.shelf_4_height:
                shelf = 3
            elif self.shelf_4_height < obj.position_relative.z < self.shelf_5_height:
                shelf = 4
            else:
                continue  # Only considering the first four shelves for now
            
            shelf_side_key = (shelf, side)
            self.shelf_side_objects[shelf_side_key].append(obj)
            self.shelf_side_class_counts[shelf_side_key][obj.object_class] += 1
        
        # Step 2: Determine Most Common Class
        potential_assignments = defaultdict(list)
        for shelf_side, class_counter in self.shelf_side_class_counts.items():
            most_common_classes = class_counter.most_common()
            if len(most_common_classes) == 1:
                potential_assignments[most_common_classes[0][0]].append((most_common_classes[0][1], shelf_side))
            else:
                max_count = most_common_classes[0][1]
                tied_classes = [cls for cls, count in most_common_classes if count == max_count]
                if len(tied_classes) == 1:
                    potential_assignments[tied_classes[0]].append((max_count, shelf_side))
                # Tie within the shelf-side, discard this shelf-side for final assignment

        # Step 3: Assign Classes Uniquely
        final_class_assignment = {}
        for obj_class, occurrences in potential_assignments.items():
            if len(occurrences) == 1:
                final_class_assignment[occurrences[0][1]] = obj_class
            else:
                occurrences.sort(reverse=True)  # Sort by count (descending)
                selected_shelf_side = occurrences[0][1]
                final_class_assignment[selected_shelf_side] = obj_class
                # Discard other occurrences
        
        # Step 4: Print and Store Results
        print("Final class assignments per shelf-side combination:")
        for shelf_side, obj_class in final_class_assignment.items():
            shelf, side = shelf_side
            print(f"Shelf {shelf}, Side {side} - Most Common Class: {obj_class}")

        for key, objects in self.shelf_side_objects.items():
            shelf, side = key
            print(f"Shelf {shelf}, Side {side}")
            print(f"Objects cabinet: {[obj.object_name for obj in objects]}")

        self.shelf_side_common_class = final_class_assignment

        # Example usage
        # Assuming `self.shelf_1_height` to `self.shelf_5_height` are defined and `filtered_objects` is provided
        # analysis_cabinet(self, filtered_objects)
        
    def load_image_one_object(self, obj_name, obj):
        # Construct the filename for the image
        image_name = f"image_{obj_name}.jpg"

        # Specify the directory where the images are stored
        output_dir = "images_with_rectangles"

        # Construct the file path for the image
        image_path = os.path.join(output_dir, image_name)

        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        # self.custom_face_filename = current_datetime

        self.current_image = obj.image_rgb_frame

        bridge = CvBridge()
        # Convert ROS Image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
        image_objects_detected = cv_image

        current_frame_draw = image_objects_detected.copy()

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
                if obj.object_class in self.low_priority_class:
                    print('Object table low priority:', obj.object_name)
                    objects_choosed.append(obj)
                    if len(objects_choosed) == 5:
                        break
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

    def choose_priority(self):
        # Este nível fica para a versão 1. Para a versão 0 faço ver o que está na prateleira, guardar essas classes e ficam essas como high

        try:
            with open(self.node.complete_path_configuration_files + 'objects_lar.json', encoding='utf-8') as json_file:
                objects = json.load(json_file)
                # print(self.objects_file)
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")

    
        # Extract unique classes from the JSON data
        total_classes = {obj['class'] for obj in objects}

        high_priority_class_set = set()
        medium_priority_class_set = set()

        # Define the shelves that should be considered "high"
        high_shelves = {4, 5}

        for key, common_class in self.shelf_side_common_class.items():
            shelf, side = key
            # print(f"Shelf {shelf}, Side {side} - Most Common Class: {common_class}")
            # print(f"Objects cabinet: {[obj.object_name for obj in self.shelf_side_objects[key]]}")
            
            # Check if the current shelf is in the high_shelves set
            if shelf in high_shelves:
                # print("high")
                high_priority_class_set.add(common_class)
            else:
                # print("low")
                medium_priority_class_set.add(common_class)

        # Convert sets to lists if needed
        self.high_priority_class = list(high_priority_class_set)
        self.medium_priority_class = list(medium_priority_class_set)

        # Determine low priority classes
        low_priority_class_set = total_classes - high_priority_class_set - medium_priority_class_set

        self.low_priority_class = list(low_priority_class_set)


        for low_priority_class in self.low_priority_class:
            # Use unique keys for low-priority classes to avoid overwriting
            shelf_side_key = (0, f'none-{low_priority_class}')
            self.shelf_side_common_class[shelf_side_key] = low_priority_class

        # Print the results for verification
        print("High priority classes:", self.high_priority_class)
        print("Medium priority classes:", self.medium_priority_class)
        print("Low priority classes:", self.low_priority_class)
        print("Updated shelf_side_common_class:", self.shelf_side_common_class)

        for key, common_class in self.shelf_side_common_class.items():
            shelf, side = key
            print(key, common_class)
            
   
    def select_voice_audio(self, object):
        print('dentro')
        
            # category = self.node.objects_classNames_dict[name]
        name = object.object_name.lower().replace(" ", "_")
        print(name)
        filename = f"objects_names/{name}"
        self.set_speech(filename=filename, wait_for_end_of=False)
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
            # cv2.circle(image, (img_x, img_z), 5, (255, 255, 255), -1)  # Draw white points
            # cv2.putText(image, obj.object_name, (img_x + 5, img_z - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_coordinates_front_view.append(img_x)
            
            # cv2.circle(image2, (img_x, img_y), 5, (255, 255, 255), -1)  # Draw white points
            # cv2.putText(image2, obj.object_name, (img_x + 5, img_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_coordinates_top_view.append(img_y)

 
        # Plot histogram for front view (x-coordinates)
        """ ####
        1280 pixeis -> 600 centimetros
        600 / 120 = 5 cm
        5 cm -> 10,7 pixeis

        logo uso 18 vizinhos para perceber se a deteção do armário está top (porque o armário mede 90cm)
        #### """
        bins_front = 120
        # plt.figure(figsize=(10, 6))
        # plt.hist(y_coordinates_front_view, bins=bins_front, range=(0, width), color='blue', alpha=0.7)
        # plt.title("Histogram of X-coordinates in Front View")
        # plt.xlabel("Pixel X-coordinate")
        # plt.ylabel("Frequency")
        # plt.grid(True)
        # plt.savefig('histogram_front_view.png')
        # plt.close()

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

        # # Print the best peak and its highest count and selected neighbor range
        # print('--------------------')
        # print(f"Best peak in front view histogram: {bins_front[best_peak_index_front]}")
        # print(f"Highest count: {highest_count_across_peaks_front}")
        # print(f"Selected neighbor range: {best_neighbor_range_front}")
        # print(f"Left bound of selected range: {left_bound_final_front}")
        # print(f"Right bound of selected range: {right_bound_final_front}")
        # print("Neighbor counts for front view peaks:")
        # for index, counts in neighbor_counts_front.items():
        #     print(f"Peak at bin {bins_front[index]}: {counts}")

        """ ####
        720 pixeis -> 500 centimetros
        500 / 50 = 5 cm
        5 cm -> 7,2 pixeis

        logo uso 6 vizinhos para perceber se a deteção do armário está top (porque o armário mede 30cm em profundidade)
        #### """
        
        # # Plot histogram for top view (z-coordinates)
        bins_top = 144
        # plt.figure(figsize=(10, 6))
        # plt.hist(y_coordinates_top_view, bins=bins_top, range=(0, height), color='green', alpha=0.7)
        # plt.title("Histogram of Z-coordinates in Top View")
        # plt.xlabel("Pixel Z-coordinate")
        # plt.ylabel("Frequency")
        # plt.grid(True)
        # plt.savefig('histogram_top_view.png')
        # plt.close()

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


        # # Print the results
        # print('--------------------')
        # print(f"Best peak in top view histogram: {bins_top[best_peak_index_top]}")
        # print(f"Highest count: {highest_neighbor_count_top}")
        # print(f"Selected neighbor range: {selected_neighbor_range_top}")
        # print(f"Left bound of selected range: {left_bound_final}")
        # print(f"Right bound of selected range: {right_bound_final}")
        # print("Neighbor counts for top view peaks:")
        # for index, counts in neighbor_counts_top.items():
        #     print(f"Peak at bin {bins_top[index]}: {counts}")


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

        # # Plotting the peak regions in histograms
        # plt.figure(figsize=(10, 6))
        # plt.hist(y_coordinates_front_view, bins=bins_front, range=(0, width), color='blue', alpha=0.7)
        # plt.axvline(left_bound_final_front, color='red', linestyle='--', linewidth=2, label='Left Bound')
        # plt.axvline(right_bound_final_front, color='green', linestyle='--', linewidth=2, label='Right Bound')
        # plt.title("Histogram of X-coordinates in Front View with Peak Region")
        # plt.xlabel("Pixel X-coordinate")
        # plt.ylabel("Frequency")
        # plt.grid(True)
        # plt.legend()
        # plt.savefig('histogram_front_view_peak.png')
        # plt.close()

        # plt.figure(figsize=(10, 6))
        # plt.hist(y_coordinates_top_view, bins=bins_top, range=(0, height), color='green', alpha=0.7)
        # plt.axvline(left_bound_final , color='red', linestyle='--', linewidth=2, label='Left Bound')
        # plt.axvline(right_bound_final, color='green', linestyle='--', linewidth=2, label='Right Bound')
        # plt.title("Histogram of Z-coordinates in Top View with Peak Region")
        # plt.xlabel("Pixel Z-coordinate")
        # plt.ylabel("Frequency")
        # plt.grid(True)
        # plt.legend()
        # plt.savefig('histogram_top_view_peak.png')
        # plt.close()

        # time.sleep(1)
        # # Load and display the saved histogram images using OpenCV
        # hist_front_view_peak = cv2.imread('histogram_front_view_peak.png')
        # hist_top_view_peak = cv2.imread('histogram_top_view_peak.png')

        # cv2.imshow("Top view of cabinet", image2)
        # cv2.waitKey(0)

        # cv2.imwrite('top_view_cabinet' + ".jpg", image2) 
        # time.sleep(0.5)

        # cv2.imshow("Histogram of Z-coordinates in Top View with Peak Region", hist_top_view_peak)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # cv2.imshow("Front view of cabinet", image)
        # cv2.waitKey(0)

        # cv2.imwrite('Front_view_of_cabinet' + ".jpg", image) 
        # time.sleep(0.5)

        # cv2.imshow("Histogram of X-coordinates in Front View with Peak Region", hist_front_view_peak)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # print(f"Front view: {left_bound_front} to {right_bound_front}")
        # print(f"Top view: {left_bound_top} to {right_bound_top}")

        # Return the filtered coordinates
        return filtered_coordinates

    def choose_place_arm(self, shelf, side, obj):
        adjust_dist = 0.0
        object_height = float(self.heights_dict.get(obj.object_name))
        class_filename = f"objects_classes/_{obj.object_class}"
        print(class_filename)
        print(object_height)
        height_arm = 0.0
        if side == 'left':
            print('left side')
            if shelf == 1:
                print('first shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_first_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + object_height + 0.1
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_third_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + object_height + 0.1
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_third_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 2:
                print('second shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_second_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_second_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_second_shelf_left_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 3:
                print('third shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_third_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_third_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_third_shelf_left_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 4:
                print('fourth shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_fourth_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_left_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

            elif shelf == 5:
                print('fifth shelf')
                adjust_dist = 0.5
                self.set_speech(filename="storing_groceries/Place_fifth_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                height_arm = self.shelf_5_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fifth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_5_height + 0.1 + object_height
                # height_arm = self.shelf_5_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fifth_shelf_left_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

            elif shelf == 6:
                print('fifth shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_sixth_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

        elif side == 'right':
            print('right side')
            if shelf == 1:
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_first_shelf_rs", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 2:
                print('second shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_second_shelf_rs", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 3:
                print('third shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_third_shelf_rs", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
            elif shelf == 4:
                print('fourth shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_fourth_shelf_rs", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_right_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

            elif shelf == 5:
                print('fifth shelf')
                adjust_dist = 0.5
                self.set_speech(filename="storing_groceries/Place_fifth_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                height_arm = self.shelf_5_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fifth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_5_height + 0.1 + object_height
                # height_arm = self.shelf_5_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fifth_shelf_right_side",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

            elif shelf == 6:
                print('fifth shelf')
                adjust_dist = 0.35
                self.set_speech(filename="storing_groceries/Place_sixth_shelf_ls", wait_for_end_of=False)
                self.set_speech(filename='generic/Near', wait_for_end_of=False)
                self.set_speech(filename=class_filename, wait_for_end_of=False)
                self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                height_arm = self.shelf_4_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fourth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                height_arm = self.shelf_4_height + 0.1 + object_height
                # height_arm = self.shelf_4_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fourth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

        else:
            if shelf == 0:
                print('No shelf')
                adjust_dist = 0.5
                self.set_speech(filename="storing_groceries/object_not_belong_cabinet", wait_for_end_of=False)
                height_arm = self.shelf_5_height + 0.1 + object_height
                a = self.transform(height_arm)
                print(a[0], a[1], a[2])
                self.set_rgb(command=WHITE+ROTATE)
                self.set_arm(command="place_cabinet_fifth_shelf_centre", adjust_position=a[1], wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist, adjust_direction=0.0, wait_for_end_of=True)
                self.shelf_5_height + 0.1 + object_height
                # height_arm = self.shelf_5_height_to_place + object_height + 0.05
                a = self.transform(height_arm)
                self.set_arm(command="place_cabinet_fifth_shelf_centre",adjust_position=a[1], wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)



        print('Altura final = ', height_arm)
        return adjust_dist

    def pick_and_place_objects(self, table_objects):
        object_grabbed = False
        obj_0 = table_objects[0]

        obj_name_lower = obj_0.object_name.lower().replace(" ", "_")
        print(obj_name_lower)

        object_help_pick = 'help_pick_' + obj_name_lower
        self.set_face(str(object_help_pick))
        print(object_help_pick)

        self.set_arm(command="ask_for_object_routine", wait_for_end_of=False)

        self.set_neck(position=self.look_judge, wait_for_end_of=False)

        self.set_rgb(command=BLUE+ROTATE)
        self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
        self.set_rgb(command=GREEN+BLINK_LONG)

        self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=False) 

        self.select_voice_audio(obj_0)

        # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

        self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False) 

        self.load_image_one_object(obj_0.object_name, obj_0)

        time.sleep(5)

        self.set_face(str(object_help_pick))

        self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)
        
        time.sleep(1.5) # waits for person to put object in hand
            
        object_in_gripper = False
        gripper_ctr = 0
        while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
            
            gripper_ctr += 1
            self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

            object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

            if not object_in_gripper:

                self.set_rgb(command=RED+BLINK_LONG)
                if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
            
                self.set_arm(command="open_gripper", wait_for_end_of=False)
           
        if object_in_gripper:
            object_grabbed = True
            self.set_rgb(command=GREEN+BLINK_LONG)
            print('object grabbed')
            
        else:
            self.set_speech(filename="generic/misdetection_move_to_next", wait_for_end_of=False)
            self.set_rgb(command=RED+BLINK_LONG)
            print('object not grabbed')
            object_grabbed = False

        self.set_arm(command="arm_front_robot", wait_for_end_of=True)
        # self.set_arm(command="arm_front_robot_linear", wait_for_end_of=True)
        
        self.set_rgb(command=GREEN+BLINK_LONG)

        for i in range(len(table_objects)):
            print('Object grabbed', object_grabbed)
            if object_grabbed == True:
                if i < len(table_objects) - 1:
                    obj = table_objects[i]
                    next_obj = table_objects[i+1]
                else:
                    obj = table_objects[i]
                    next_obj = obj

                print('---')
                print(obj.object_name, next_obj.object_name)
                print(i)
                print(len(table_objects) - 1)
                print('---')

                # Output results
                for key, common_class in self.shelf_side_common_class.items():
                    shelf, side = key
                    print(key, common_class)
                    # print('Inside loop: ', shelf, side, common_class)
                    # print(f"Shelf {shelf}, Side {side} - Most Common Class: {common_class}")
                    # print(f"Objects: {[obj.object_name for obj in self.shelf_side_objects[key]]}")
                    if obj.object_class == common_class:
                        if i < len(table_objects) - 1:
                            print('i < len')
                            adjust_dist = self.choose_place_arm(shelf, side, obj)
                            print(adjust_dist)
                        
                            self.set_rgb(command=GREEN+BLINK_LONG)
                            # self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                            # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                            time.sleep(2)
                            self.set_arm(command="open_gripper", wait_for_end_of=True)
                            self.set_rgb(command=RED+BLINK_LONG)


                            print('a')
                            
                            obj_name_lower = next_obj.object_name.lower().replace(" ", "_")
                            print(obj_name_lower)

                            object_help_pick = 'help_pick_' + obj_name_lower
                            self.set_face(str(object_help_pick))
                            print(object_help_pick)

                            self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=False) 

                            self.select_voice_audio(next_obj)

                            print('b')

                            # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                            self.load_image_one_object(next_obj.object_name, next_obj)

                            self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False) 

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist + 0.02, adjust_direction=180.0, wait_for_end_of=True)
                            print('c')
                    
                            time.sleep(1)

                            self.set_arm(command="arm_front_robot", wait_for_end_of=True)

                            self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)

                            self.set_face(str(object_help_pick))

                            self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)

                            print('d')

                            time.sleep(1.5) # waits for person to put object in hand

                            self.set_neck(position=self.look_judge, wait_for_end_of=False)

                            self.set_rgb(command=BLUE+ROTATE)
                            self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_LONG)

                            if i == 2:
                                self.set_rgb(command=BLUE+ROTATE)
                                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_LONG)
                
                            # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.11, adjust_direction=-90.0 + 360.0, wait_for_end_of=False)

                            print('3')

                            object_in_gripper = False
                            gripper_ctr = 0
                            object_grabbed = True
                            while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                                
                                gripper_ctr += 1
                                self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                                object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                                if not object_in_gripper:

                                    self.set_rgb(command=RED+BLINK_LONG)
                                    if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                                
                                    self.set_arm(command="open_gripper", wait_for_end_of=False)
                            
                            if object_in_gripper:
                                object_grabbed = True
                                self.set_rgb(command=GREEN+BLINK_LONG)
                                object_grabbed = True
                                
                            else:
                                self.set_speech(filename="generic/misdetection_move_to_next", wait_for_end_of=False)
                                self.set_rgb(command=RED+BLINK_LONG)
                                object_grabbed = False

                            self.set_arm(command="arm_front_robot", wait_for_end_of=True)
                            # self.set_arm(command="arm_front_robot_linear", wait_for_end_of=True)

                            print('front arm')
                            
                            self.set_rgb(command=GREEN+BLINK_LONG)
            
                        else:
                            print('i >= len')
                            adjust_dist = self.choose_place_arm(shelf, side, obj)
                            print(adjust_dist)
                            self.set_rgb(command=GREEN+BLINK_LONG)
                            # self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                            # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                            time.sleep(2)
                            self.set_arm(command="open_gripper", wait_for_end_of=True)
                            self.set_rgb(command=RED+BLINK_LONG)

                            print('a')
                            
                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist + 0.02, adjust_direction=180.0, wait_for_end_of=True)
                            print('c')
                            time.sleep(1)

                            self.set_arm(command="arm_front_robot", wait_for_end_of=True)

                            self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)

                            self.set_arm(command="go_initial_position", wait_for_end_of=True)
                            
                            self.set_rgb(command=GREEN+BLINK_LONG)

            else:
                if i < len(table_objects) - 1:
                    obj = table_objects[i]
                    next_obj = table_objects[i+1]
                else:
                    obj = table_objects[i]
                    next_obj = obj

                if i < len(table_objects) - 1:
                    print('---')
                    print(obj.object_name, next_obj.object_name)
                    print(i)
                    print(len(table_objects) - 1)
                    print('---')
                    
                    
                    print('not grabbed')
                    self.set_rgb(command=RED+BLINK_LONG)
                    self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)
                    
                    self.set_arm(command="open_gripper", wait_for_end_of=True)                
                    obj_name_lower = next_obj.object_name.lower().replace(" ", "_")
                    print(obj_name_lower)

                    object_help_pick = 'help_pick_' + obj_name_lower



                    self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=False) 

                    self.select_voice_audio(next_obj)

                    print('b')

                    # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                    self.load_image_one_object(next_obj.object_name, next_obj)

                    self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False) 
                    
                    time.sleep(3)

                    self.set_face(str(object_help_pick))

                    self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)
                    
                    object_in_gripper = False
                    gripper_ctr = 0
                    while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                        
                        gripper_ctr += 1
                        self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                        object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                        if not object_in_gripper:

                            self.set_rgb(command=RED+BLINK_LONG)
                            if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                                self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                            self.set_arm(command="open_gripper", wait_for_end_of=False)
                    
                    if object_in_gripper:
                        object_grabbed = True
                        self.set_rgb(command=GREEN+BLINK_LONG)
                        
                    else:
                        self.set_speech(filename="generic/misdetection_move_to_next", wait_for_end_of=False)
                        self.set_rgb(command=RED+BLINK_LONG)

                    self.set_arm(command="arm_front_robot", wait_for_end_of=True)
                    # self.set_arm(command="arm_front_robot_linear", wait_for_end_of=True)

                    print('front arm')
                    
                    self.set_rgb(command=GREEN+BLINK_LONG)

                else:
                    print('i >= len')
                    adjust_dist = self.choose_place_arm(shelf, side, obj)
                    print(adjust_dist)
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    # self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                    # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                    time.sleep(2)
                    self.set_arm(command="open_gripper", wait_for_end_of=True)

                    print('a')
                    
                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_dist + 0.02, adjust_direction=180.0, wait_for_end_of=True)
                    print('c')
                    time.sleep(1)

                    self.set_arm(command="arm_front_robot", wait_for_end_of=True)

                    self.set_arm(command="ask_for_object_routine", wait_for_end_of=True)

                    self.set_arm(command="go_initial_position", wait_for_end_of=True)
                    
                    self.set_rgb(command=GREEN+BLINK_LONG)
   
    def check_door_depth_hand(self, half_image_zero_or_near_percentage=0.3, full_image_near_percentage=0.1, near_max_dist=600):

        overall = False
        DEBUG = True

        if self.node.new_image_hand_flag:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img_hand, desired_encoding="passthrough")
            height, width = current_frame_depth_hand.shape
            current_frame_depth_hand_half = current_frame_depth_hand[height//2:height,:]
            current_frame_depth_hand_center = current_frame_depth_hand[height//4:height-height//4, width//3:width-width//3]
            # FOR THE FULL IMAGE

            tot_pixeis = height*width 
            tot_pixeis = (height-height//4 -height//4) * (width-width//3 - width//3)
            mask_zero = (current_frame_depth_hand == 0)
            mask_near = (current_frame_depth_hand > 0) & (current_frame_depth_hand <= near_max_dist)
            mask_zero_center = (current_frame_depth_hand_center == 0)
            mask_near_center = (current_frame_depth_hand_center > 0) & (current_frame_depth_hand_center <= near_max_dist)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_hand > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)
            pixel_count_zeros_center = np.count_nonzero(mask_zero_center)

            # FOR THE BOTTOM HALF OF THE IMAGE

            mask_zero_half = (current_frame_depth_hand_half == 0)
            mask_near_half = (current_frame_depth_hand_half > 0) & (current_frame_depth_hand_half <= near_max_dist)
            mask_near_center = (current_frame_depth_hand_center > 0) & (current_frame_depth_hand_center <= near_max_dist)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_hand_half > near_max_dist) # just for debug
                blank_image_half = np.zeros((height//2,width,3), np.uint8)
                blank_image_half[mask_zero_half] = [255,255,255]
                blank_image_half[mask_near_half] = [255,0,0]
                blank_image_half[mask_remaining_half] = [0,0,255]
                    
            pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
            pixel_count_near_half = np.count_nonzero(mask_near_half)
            pixel_count_near_center = np.count_nonzero(mask_near_center)
            
            if DEBUG:
                cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
                cv2.rectangle(blank_image, (width//3, height//4), (width - width//3, height - height//4), (0, 255, 0), 3)
                cv2.imshow("New Img Distance Inspection", blank_image)
                cv2.waitKey(10)

                # cv2.imwrite('Distance_to_door' + ".jpg", blank_image) 
                # time.sleep(0.5)


                

            half_image_zero_or_near = False
            half_image_zero_or_near_err = 0.0
            
            full_image_near = False
            full_image_near_err = 0.0


            half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
            if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
                half_image_zero_or_near = True
            
            full_image_near_err = pixel_count_near/tot_pixeis
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True

            center_image_near_err = pixel_count_near_center / tot_pixeis
            print(center_image_near_err*100)
            center_image_zeros = pixel_count_zeros_center/tot_pixeis
            #print(center_image_zeros*100)
            if full_image_near_err >= full_image_near_percentage:
                center_image_near = True
            
            if half_image_zero_or_near or full_image_near:
                overall = True

            # just for debug
            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
            # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err

            # Define the coordinates of the rectangle
            x_start = width // 3
            y_start = height // 4
            x_end = width - width // 3
            y_end = height - height // 4

            # Extract the region of interest (ROI) from the depth image
            roi_depth = current_frame_depth_hand[y_start:y_end, x_start:x_end]

            # Calculate the average depth value within the ROI, excluding zero (invalid) values
            valid_depths = roi_depth[roi_depth > 0]  # Filter out invalid depth values
            if valid_depths.size > 0:
                average_depth = np.mean(valid_depths)
            else:
                average_depth = 0  # Handle case where no valid depths are found

            # Print the average depth value
            print(f"Average depth in the rectangle: {average_depth}")

        
            return center_image_near_err, average_depth
        else:
            return -1.0

    def open_cabinet_door(self, objects_found, wanted_object):
        # - Centrar com armário em x
        # - Ir até certo ponto em Y (ver qual)
        
        if wanted_object != '':
            # set_pose_arm = ListOfFloats()
            self.set_rgb(command=WHITE+HALF_ROTATE)
            cabinet_position = wanted_object.position_relative
            object_location = self.transform_object(wanted_object)
            #Value of height I want the arm to go to not touch in shelfs:
            # desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
            # new_height = Float32()
            # new_height.data = desired_height
            
            object_x = object_location[0]
            object_y = object_location[1]
            object_y = object_location[1] + 100.0
            object_y = 350.0
            object_z = object_location[2]

            print('x y e z do objeto em relação ao braço:',object_x, object_y, object_z)  

            print('x y e z do objeto em relação ao robô:',cabinet_position)   

            distance_to_close = abs(object_x)/1000 
            print('Distance I am from door', distance_to_close)

            distance_x_to_center = cabinet_position.x
            distance_y_to_center_original = cabinet_position.y - self.wardrobe_depth - self.door_width - self.robot_radius - self.robot_radius - self.robot_radius
            distance_y_to_center = abs(cabinet_position.y) - self.wardrobe_depth - self.door_width - self.robot_radius - self.robot_radius
            ### ISTO CENTRA QD PORTA FECHADA É A ESQUERDA. -> door_position.x + self.node.wardrobe_width/4 
            ###  SE PORTA FECHADA FOR A DIREITA, TENHO DE TROCAR SINAL PARA -> door_position.x - self.node.wardrobe_width/4 
            if distance_x_to_center < 0.0:
                move_side = 90.0
            else:
                move_side = -90.0 + 360.0

            # distance_x_to_center = abs(abs(distance_x_to_center) - 0.1)
            distance_x_to_center_original = distance_x_to_center
            distance_y_to_center_original = distance_y_to_center_original
            
            # distance_x_to_center = abs(distance_x_to_center)
            # distance_y_to_center = abs(distance_y_to_center)

            # print('distancia lateral:', distance_x_to_center)
            # print('distancia frontal:', distance_y_to_center)

            # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_x_to_center, adjust_direction=move_side, wait_for_end_of=True)
            # self.set_rgb(command=GREEN+BLINK_LONG)
            
            # # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
            # # self.set_rgb(command=BLUE+BLINK_LONG)

            # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_y_to_center, adjust_direction=0.0, wait_for_end_of=True)
            # self.set_rgb(command=GREEN+BLINK_LONG)
            
            print('d_lateral:', distance_x_to_center_original)
            print('d_frontal:', distance_y_to_center_original)
            
            ang_to_bag = -math.degrees(math.atan2(distance_x_to_center, distance_y_to_center))
            dist_to_bag = (math.sqrt(distance_x_to_center**2 + distance_y_to_center**2))
            print(ang_to_bag, dist_to_bag)
            self.set_rgb(command=WHITE+ROTATE)
            self.set_navigation(movement="adjust", adjust_distance=dist_to_bag, adjust_direction=ang_to_bag, wait_for_end_of=True)
            self.set_rgb(command=GREEN+BLINK_LONG)

            time.sleep(2)

            self.set_rgb(command=BLUE+ROTATE)
            self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
            self.set_rgb(command=GREEN+BLINK_LONG)
            
            self.set_rgb(command=BLUE+ROTATE)
            self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
            self.set_rgb(command=GREEN+BLINK_LONG)

            print('inside')
            tetas = [[0, 0]]
            objects_found = self.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=False, detect_shoes=False, detect_doors=True)
            print('pos-search')
            for obj in objects_found:
                if obj.object_name == 'Cabinet':
                    cabinet_found = True
                    wanted_object = obj
                    cabinet_position = wanted_object.position_relative
                    print('Object found')

            # REAJUSTAR COM CABINET
            print('Reajustar cabinet!!!')
            # object_location = self.transform_object(wanted_object)
            print('Object found')
            distance_x_to_center = cabinet_position.x
            distance_y_to_center_original = cabinet_position.y - self.wardrobe_depth - self.door_width - self.robot_radius - self.robot_radius
            print('d_lateral:', distance_x_to_center)
            print('d_frontal:', distance_y_to_center_original)
            
            ang_to_bag = -math.degrees(math.atan2(distance_x_to_center, distance_y_to_center_original))
            dist_to_bag = (math.sqrt(distance_x_to_center**2 + distance_y_to_center_original**2))
            print(ang_to_bag, dist_to_bag)
            self.set_rgb(command=WHITE+ROTATE)
            self.set_navigation(movement="adjust", adjust_distance=dist_to_bag, adjust_direction=ang_to_bag, wait_for_end_of=True)
            self.set_rgb(command=GREEN+BLINK_LONG)

            time.sleep(2)

            self.set_rgb(command=BLUE+ROTATE)
            self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
            self.set_rgb(command=GREEN+BLINK_LONG)


            # print(self.node.arm_current_pose)

            # self.set_arm(command="arm_side_of_washing_machine", wait_for_end_of=True)

            self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
            time.sleep(2)

            left_door = False
            right_door = False


            
            self.set_arm(command="check_right_door", wait_for_end_of=True)
            time.sleep(1)
            self.node.new_image_hand_flag = False
            while self.node.new_image_hand_flag == False:
                pass
            near_percentage = -1.0
            while near_percentage == -1.0:
                near_percentage, avg_door_right = self.check_door_depth_hand()
                

            
            self.set_arm(command="check_left_door", wait_for_end_of=True)
            time.sleep(1)
            self.node.new_image_hand_flag = False
            while self.node.new_image_hand_flag == False:
                pass

            near_percentage = -1.0
            while near_percentage == -1.0:
                near_percentage, avg_door_left = self.check_door_depth_hand()

            if avg_door_left < avg_door_right:
                right_door = True
                print('Porta direita aberta creio eu')
                average_depth_door = avg_door_left

            else:
                left_door = True
                print('Porta esquerda aberta creio eu')
                average_depth_door = avg_door_right

            

            # # Check if the door is near
            # if near_percentage < 0.5:
            #     right_door = True
            #     print('Porta direita aberta creio eu')

            # else:
            #     right_door = False
            #     print('Porta direita fechada creio eu')
            #     average_depth_door = avg_door

            
            # # self.set_arm(command="change_height_front_left_robot", wait_for_end_of=True)
            # self.set_arm(command="check_left_door", wait_for_end_of=True)
            # time.sleep(1)
            # self.node.new_image_hand_flag = False
            # while self.node.new_image_hand_flag == False:
            #     pass

            # near_percentage = -1.0
            # while near_percentage == -1.0:
            #     near_percentage, avg_door_left = self.check_door_depth_hand()

            # if near_percentage < 0.5:
            #     left_door = True
            #     print('Porta esquerda aberta creio eu')
                

            # else:
            #     left_door = False
            #     print('Porta esquerda fechada creio eu')
            #     average_depth_door = avg_door

            cabinet_found = False

            # average_depth_door = (average_depth_door / 1000) - 0.13 + 0.22 #0.13 é a distância da câmera à garra e 0.2 é uma margem para entrar
            # print('Tip of arm distance to cabinet =', average_depth_door)

            self.set_speech(filename="storing_groceries/might_touch_cabinet", wait_for_end_of=False)  
            self.set_rgb(command=WHITE+HALF_ROTATE)          

            if right_door == True:
                average_depth_door = (average_depth_door / 1000) - 0.13 + 0.22 #0.13 é a distância da câmera à garra e 0.2 é uma margem para entrar
                print('Tip of arm distance to cabinet =', average_depth_door)

                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

                print('right door')
                
                self.set_arm(command="check_right_door_inside", wait_for_end_of=True)

                # distance_in_y_to_get_inside_cabinet = average_depth_door - 0.175 # nova posição do braço está 0.175 cm à frente da que viu o armário
                distance_in_y_to_get_inside_cabinet = average_depth_door - 0.230 # nova posição do braço está 0.230 cm à frente da que viu o armário

                print('I will navigate in front for ', distance_in_y_to_get_inside_cabinet, 'meters')

                self.set_rgb(command=BLUE+BLINK_LONG)                

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_in_y_to_get_inside_cabinet, adjust_direction=0.0, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                self.set_arm(command="open_left_door_from_inside", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                # self.set_navigation(movement="adjust_angle", absolute_angle=5.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="adjust_angle", absolute_angle=-165.0, flag_not_obs=True, wait_for_end_of=True)
                time.sleep(1)

                self.set_arm(command="open_left_door_from_inside_2", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.3, adjust_direction=170.0, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(2)

                self.set_arm(command="finish_open_left_door_from_inside", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                self.set_arm(command="go_initial_position", wait_for_end_of=False)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                navigate_backwards = distance_in_y_to_get_inside_cabinet - 0.5
                
                print('backwards', navigate_backwards)
                if navigate_backwards > 0.0:
                    print('backwards', navigate_backwards)
                    # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.1, adjust_direction=180.0, wait_for_end_of=True)
                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=navigate_backwards, adjust_direction=180.0, wait_for_end_of=True)
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    time.sleep(2)
                
                else:
                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=navigate_backwards, adjust_direction=0.0, wait_for_end_of=True)
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    time.sleep(2)

                

            elif left_door == True:
                average_depth_door = (average_depth_door / 1000) - 0.13 + 0.15 #0.13 é a distância da câmera à garra e 0.2 é uma margem para entrar
                print('Tip of arm distance to cabinet =', average_depth_door)

                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                print('left door')
                self.set_arm(command="check_left_door", wait_for_end_of=True)

                distance_in_y_to_get_inside_cabinet = average_depth_door

                print('I will navigate in front for ', distance_in_y_to_get_inside_cabinet, 'meters')

                self.set_rgb(command=BLUE+BLINK_LONG)                

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_in_y_to_get_inside_cabinet, adjust_direction=0.0, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                self.set_arm(command="open_right_door_from_inside", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)
                
                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                
                # self.set_navigation(movement="adjust_angle", absolute_angle=-90.0, flag_not_obs=True, wait_for_end_of=True)
                time.sleep(1)

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.35, adjust_direction=200.0, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(2)

                self.set_arm(command="finish_open_right_door_from_inside", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                self.set_arm(command="go_initial_position", wait_for_end_of=False)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(1)

                navigate_backwards = distance_in_y_to_get_inside_cabinet - 0.35
                if navigate_backwards > 0.0:
                    print(navigate_backwards)
                    # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.1, adjust_direction=180.0, wait_for_end_of=True)
                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=navigate_backwards, adjust_direction=180.0, wait_for_end_of=True)
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    time.sleep(2)
                


            """  response = self.node.pose_planner([object_x, object_y, object_z, self.node.arm_current_pose[3], self.node.arm_current_pose[4], self.node.arm_current_pose[5]])
        
            if response == True:
                print('YES')

                set_pose_arm.pose[:] = array('f')

                # set_pose_arm.pose.clear()

                set_pose_arm.pose.append(object_x)
                set_pose_arm.pose.append(object_y)
                set_pose_arm.pose.append(object_z)
                # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                set_pose_arm.pose.append(self.node.arm_current_pose[3])
                set_pose_arm.pose.append(self.node.arm_current_pose[4])
                set_pose_arm.pose.append(self.node.arm_current_pose[5])
                
                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                self.set_arm(command="move_linear", wait_for_end_of=True)

                time.sleep(3)
                self.set_arm(command="close_gripper", wait_for_end_of=True)

            else:
                print('NO')
                print(response)

            while True:
                pass """

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
    
    def transform_object(self, obj):
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
        new_x = obj.position_relative.y * 1000
        new_y = -obj.position_relative.x * 1000
        new_z = obj.position_relative.z * 1000
        print(obj.object_name)
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
    
    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message

    def main(self):

        print("IN NEW MAIN")
        # time.sleep(1)
        try:
            with open(self.node.complete_path_configuration_files + 'objects_height.json', encoding='utf-8') as json_file:
                self.node.objects_file = json.load(json_file)
                # print(self.objects_file)
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")


        # Print the dictionary
        # print(self.node.objects_file)
        
        # Create a dictionary to store the heights with the object names as keys
        self.heights_dict = {item['name']: item['height'] for item in self.node.objects_file}

        # Print the heights dictionary
        # print(self.heights_dict)
        """
        # Access a specific height using the object's name
        sponge_height = self.heights_dict.get('Sponge')
        print(f"Height of Sponge: {sponge_height}")

        # Example: Iterate and print all object names and their heights
        for name, height in self.heights_dict.items():
            print(f"Object: {name}, Height: {height}") """
    

        # Navigation Coordinates
        """## LAR
        self.front_of_door = [0.0, 1.5] 
        self.outside_kitchen_door = [-0.7, 5.5]
        self.inside_kitchen_door = [-0.7, 7.5] """


        # self.state = self.Approach_kitchen_table
        # self.state = self.Waiting_for_task_start
        self.state = self.Approach_cabinet_first_time
        # self.state = self.Approach_tables_first_time


        self.front_of_door = [0.0, 1.5]
        self.almost_kitchen = [1.7, 5.3]
        self.inside_kitchen = [1.7, 7.0]
        self.cabinet = [-1.5, 7.5]
        self.CABINET_ANGLE = -90.0
        
        self.MAX_SPEED = 40

        self.pre_room_door = [0.45, 3.65]
        self.post_room_door = [0.45, 4.70]
        self.front_of_start_door = [0.0, 1.0]
        self.front_sofa = [0.45, 5.8]
        self.midway_living_room = [-0.9, 8.5]
        self.close_to_garbage_bin = [-2.76, 5.9]
        self.close_to_dishwasher = [-2.26, 8.0]
        self.close_to_table_sb = [-4.26, 8.5]
        self.pre_table = [-4.76, 6.0]
       
        while True:

            if self.state == self.Waiting_for_task_start:

                time.sleep(1)
                print('State 0 = Initial')

                self.set_face("charmie_face")
                self.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")

                time.sleep(1)          

                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)
                
                self.set_speech(filename="storing_groceries/sg_ready_start", wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True) # must change to door open

                self.set_rgb(command=MAGENTA+SET_COLOUR)

                self.wait_for_start_button()

                self.set_rgb(command=BLUE+SET_COLOUR)

                ###### WAITS FOR START BUTTON / DOOR OPEN
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
                         
                self.wait_for_door_start()        

                self.state = self.Approach_cabinet_first_time

            elif self.state == self.Approach_cabinet_first_time:
                
                print('State 1 = Approaching cabinet for the first time')

                ##### MOVEMENT TO THE CABINET

                # self.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(BLUE+ROTATE)
                # # self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.pre_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(MAGENTA+ROTATE)
                # self.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.post_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                # self.set_rgb(BLUE+ROTATE)
                # self.set_navigation(movement="rotate", target=self.front_sofa, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.front_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(MAGENTA+ROTATE)
                # self.set_navigation(movement="rotate", target=self.midway_living_room, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.midway_living_room, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(BLUE+ROTATE)

                # self.set_navigation(movement="rotate", target=self.close_to_dishwasher, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.close_to_dishwasher, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(MAGENTA+ROTATE)

                # self.set_navigation(movement="rotate", target=self.close_to_garbage_bin, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.close_to_garbage_bin, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_rgb(BLUE+ROTATE)

                # self.set_navigation(movement="rotate", target=self.pre_table, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.pre_table, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                # self.set_rgb(MAGENTA+ROTATE)

                self.set_face("charmie_face")
                self.set_initial_position(self.initial_position)
                    
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                self.set_rgb(command=WHITE+ROTATE)
                    
        

                # self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=True)

                # self.set_speech(filename="generic/moving_cabinet", wait_for_end_of=False)
                
                # self.set_navigation(movement="move", target=self.kitchen_table, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=BLUE+ROTATE)                
                self.set_navigation(movement="orientate", absolute_angle= 90.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                

                time.sleep(1)

                self.activate_obstacles(obstacles_lidar_up=False, obstacles_camera_head=False)

                """ self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=90.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG) """
                
                self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)
                
                self.activate_yolo_objects(activate_objects=True)
                
                self.set_neck(position=self.look_table_objects_front)
                
                tetas = [[-120, -30], [-90, -30], [-60, -30]]
                # tetas = [[-30, -30], [0, -30], [30, -30]]
                objects_found_table = []
                while objects_found_table == []:
                    objects_found_table = self.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False)

                print('Objects found on table: ')
                for obj in objects_found_table:
                    print(obj.object_name)

                

                self.set_neck(position=self.look_forward, wait_for_end_of=False)
                
                self.set_rgb(command=BLUE+ROTATE)                
                self.set_navigation(movement="orientate", absolute_angle= 180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(0.5)
                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                time.sleep(0.5)
                
                self.activate_yolo_objects(activate_doors=True, activate_doors_hand = False)
                
                # self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=False)

                cabinet_found = False

                print('pre')

                while cabinet_found == False:
                    print('inside')
                    tetas = [[-20, -10], [20, -10]]
                    objects_found = self.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=False, detect_shoes=False, detect_doors=True)
                    print('pos-search')
                    for obj in objects_found:
                        if obj.object_name == 'Cabinet':
                            cabinet_found = True
                            wanted_object = obj
                            print('Object found')

                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

                self.open_cabinet_door(objects_found, wanted_object)

                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

                data = []
                real_data = []
                tetas = [[0, -40], [0, -20], [0, 0]]
                objects_found = self.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False)
                
                self.set_rgb(command=GREEN+BLINK_LONG)
                
                unique_classes = set()
                
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
                    unique_classes.add(obj_class)
                
                print('Nr de objetos: ', len(data))
                print('Nr de classes: ', len(unique_classes))
                print('Classes: ', unique_classes)
                
                self.set_speech(filename="storing_groceries/sg_finished_analise_cabinet", wait_for_end_of=True) 
                
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
                
                self.high_priority_class = []
                self.medium_priority_class = []
                self.low_priority_class = []


                self.analysis_cabinet(filtered_objects)

                self.choose_priority()

                # next state
                self.state = self.Approach_tables_first_time

            elif self.state == self.Approach_tables_first_time:
                print('State 2 = Approaching table for the first time')

                self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                # self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                # self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)

                # self.set_neck(position=self.look_table_objects)
                
                # tetas = [[160, -20], [140, -20], [120, -20]]
                # objects_found_table = []
                # while objects_found_table == []:
                #     objects_found_table = self.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_doors=False)

                # print('Objects found on table: ')
                # for obj in objects_found_table:
                #     print(obj.object_name)

                print('---------')

                table_objects = self.analysis_table(objects_found_table)
                print('Objects filtered from table: ')
                for obj in table_objects:
                    print(obj.object_name)

                print('---', self.shelf_side_common_class.items())

                # self.set_neck(position=self.look_back, wait_for_end_of=False)
                for key, common_class in self.shelf_side_common_class.items():
                    print(common_class)
                    print(key)


                # self.set_speech(filename="storing_groceries/might_touch_cabinet", wait_for_end_of=False)   
                self.set_rgb(command=BLUE+ROTATE)
                self.set_navigation(movement="adjust_angle", absolute_angle=180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                

                self.pick_and_place_objects(table_objects)

                
                
                self.state = self.Final_State

            elif self.state == self.Final_State:
                print('Terminei')  
                self.set_speech(filename="storing_groceries/sg_finished", wait_for_end_of=False)

                while True:
                    pass
                """ 
                ### TO DO:

                atribuir prateleira livre
                """      