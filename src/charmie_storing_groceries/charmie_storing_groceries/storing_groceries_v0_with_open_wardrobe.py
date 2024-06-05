#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Pose2D
from charmie_interfaces.msg import Yolov8Objects, DetectedObject
from charmie_interfaces.srv import SpeechCommand, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, ArmTrigger, ActivateYoloObjects, SetFace
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import json

from pathlib import Path
from datetime import datetime

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
    ('Fourth', 'Left'): 'Fourth_shelf_ls'
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
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)


        ### Services (Clients) ###
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")

        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # Objects detected
        self.objects_filtered_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered', self.get_objects_callback, 10)

        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        
        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        
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

        self.get_neck_position = [1.0, 1.0]
        self.objects_classNames_dict = {}

        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        
        self.objects_classNames_dict = {item["name"]: item["class"] for item in self.objects_file}
        self.detected_objects = Yolov8Objects()
        #print(self.objects_classNames_dict)

        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        while not self.activate_yolo_objects_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        while not self.get_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        # Face
        while not self.face_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Face Command...")
        # Arm (CHARMIE)
        """ while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...") """
        

    def get_objects_callback(self, objects: Yolov8Objects):
        #print(objects.objects)
        self.nr_objects = objects.num_objects
        self.objects = objects.objects
        self.image = objects.image_rgb

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
    
    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_objects_confidence=0.5):
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.minimum_objects_confidence = minimum_objects_confidence

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
            self.speech_success = response.success
            self.speech_message = response.message
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
            self.speech_success = response.success
            self.speech_message = response.message
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
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [120, -20]
        self.look_tray = [0, -60]
        self.look_cabinet_top = [-45, 45]
        self.look_cabinet_center = [0, -30]
        self.look_cabinet_bottom = [-45, -45]

        self.shelf_1_height = 0.14 # 0.15
        self.shelf_2_height = 0.55 # 0.60 
        self.shelf_3_height = 0.97 # 1.10 
        self.shelf_4_height = 1.39

        door_height = 0.95

        self.left_limit_shelf = -0.7 # -0.38
        self.right_limit_shelf = 0.7 # 0.38
        self.third_shelf_x = (self.right_limit_shelf - self.left_limit_shelf) / 3
        self.center_shelf = 0.0


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
    
    def set_arm(self, command="", wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False
        
        temp = String()
        temp.data = command
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
                
    ##### ANALYSE CABINET #####

    def analysis_cabinet(self):
        nr_classes_detected = 0
        i = 0
        self.object_position = {}
        if hasattr(self.node, 'image') and self.node.image:
            if hasattr(self.node, 'objects') and self.node.objects:
                objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects


                self.current_image = self.node.image
                bridge = CvBridge()
                # Convert ROS Image to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                self.current_image_2= cv_image

                print('Will iterate for: ', self.nr_objects_detected)
                while i < self.nr_objects_detected:                    
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
                    position = ' '
                    #print(f"Object: {object_name}, Height: {object_height}, Confidence: {object_confidence}")
                    if object_name in self.object_details:
                        pass
                    else:  
                        if self.shelf_1_height < object_height < self.shelf_2_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            position = 'First shelf '
                            print(object_name, 'is in the first shelf ')

                        elif self.shelf_2_height < object_height < self.shelf_3_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            position = 'Second shelf '
                            print(object_name, 'is in the second shelf ')

                        elif self.shelf_4_height > object_height > self.shelf_3_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            position = 'Third shelf '
                            print(object_name, 'is in the third shelf ')

                        elif object_height > self.shelf_4_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            position = 'Fourth shelf '
                            print(object_name, 'is in the fourth shelf ')

                        else:
                            print(object_name, '- none of the shelfs')
                            
                        if  self.center_shelf <= object_x_position <= self.right_limit_shelf :
                            position += 'Right side '
                            
                        elif self.left_limit_shelf <= object_x_position < self.center_shelf :
                            position += 'Left side '
                            
                        else:
                            position += 'Outside shelf '
                            
                        if detected_object.object_class in self.object_position:
                            self.object_position[detected_object.object_class].append(position)
                        else:
                            self.object_position[detected_object.object_class] = [position]

                        #self.object_position[object_class] = position


                        print('object ', object_name, ' and confidence ', object_confidence)
                            
                    i += 1

                # Código para dizer 'tal classe está em tal prateleira'

                print('objects position:', self.object_position)
                
                if not self.opening_doors:
                    self.remove_duplicates()
                    merged_object_position = self.merge_objects_wardobe()
                    
                    object_class_name = list(merged_object_position.keys())
                    position_wardrobe = list(merged_object_position.values())
                    
                    print('Positions: ', position_wardrobe)
                    print('classes: ', object_class_name)

                    # print("Positions:", position)
                    # print("Object class names:", object_class_name)
                    
                    keywords = []

                    self.classes_detected_wardrobe.clear()

                    class_name_array = []
                    nr_classes_detected = 0

                    for position in merged_object_position.values():
                        keywords = position.split()  # Split each position string into words and extend the keywords list

                        # Initialize filenames
                        class_filename = None
                        location_filename = None

                        # Iterate over object_position_mapping
                        for condition, object_location in object_position_mapping.items():
                            # Check if all keywords in the condition are in the current position
                            if all(keyword in keywords for keyword in condition):
                                # Get the class name associated with the current position
                                class_name = [class_name for class_name, pos in merged_object_position.items() if pos == position][0]
                                if class_name not in class_name_array:
                                    class_name_array.append(class_name)
                                print('Class name:', class_name)
                                print('All class names', class_name_array)
                                self.classes_detected_wardrobe.append(class_name)
                                print(self.classes_detected_wardrobe)
                                nr_classes_detected = len(class_name_array)
                                print('Nr classes detected: ', nr_classes_detected)
                                location_filename = f"storing_groceries/{object_location}"
                                class_filename = f"objects_classes/{class_name}"
                                break
                    

                    return nr_classes_detected
                else:
                    first_shelf = "First shelf"
                    second_shelf = "Second shelf"
                    third_shelf = "Third shelf"
                    Fourth_shelf = "Fourth shelf"
                    right_side = "Right side"
                    left_side = "Left side"
                    i = 0
                    search = [first_shelf, second_shelf]
                    print(self.object_position.values())
                    print(len(self.object_position.values()))
                    for object_positions in self.object_position.values():
                        for position in object_positions:
                            print(position)
                            for search_string in search:
                                if search_string in position:
                                    if right_side in position:
                                        door_opened = "Right"
                                        self.opening_doors = False
                                        return door_opened
                                    elif left_side in position:
                                        door_opened = "Left"
                                        self.opening_doors = False
                                        return door_opened
                                    else:
                                        pass 
                            
    def remove_duplicates(self):
        values_to_remove = []
        for key in self.object_position:
            # Collect values to remove from self.object_position[key]
            # Get the values associated with the current key
            values_to_check = self.object_position[key]

            # Iterate over the other keys of the object_position dictionary
            for other_key in self.object_position:
                # Skip the current key
                if other_key == key:
                    continue

                # Get the values associated with the other key
                other_values = self.object_position[other_key]

                # Iterate over the values associated with the current key
                for value in values_to_check:
                    # If the value exists in the values of the other key, add it to values_to_remove
                    if value in other_values:
                        values_to_remove.append(value)
                        print('Values to remove ', values_to_remove)
                        self.temp = values_to_remove

            # Remove collected values from self.object_position[key]
            for value in values_to_remove:
                if value in self.object_position[key]:
                    self.object_position[key].remove(value)

        # Remove collected values from self.object_position[other_key]
        # Remove collected values from other keys
        for key in self.object_position:
            print('Values to remove 2', values_to_remove)
            for value in values_to_remove:
                # print('A', self.object_position[key])
                # print('B', value)
                if value in self.object_position[key]:
                    self.object_position[key].remove(value)

        print('New objects position: ', self.object_position)
                           
    def merge_objects_wardobe(self):
        position_wardrobe = []
        object_class_name= []

        # Create a dictionary to store the count of occurrences for each position for each class
        position_counts = {}

        # Iterate over the object_position dictionary
        for class_name, positions in self.object_position.items():
            for position in positions:
                # Increment the count of occurrences for the current position and class
                position_counts[(class_name, position)] = position_counts.get((class_name, position), 0) + 1

        # Create a dictionary to store the merged positions for each class
        merged_object_position = {}

        # Iterate over the position_counts dictionary to find the position with the highest count for each class
        for (class_name, position), count in position_counts.items():
            if class_name in merged_object_position:
                # If the class already exists in the merged_object_position dictionary,
                # check if the count for the current position is higher than the count for the existing position
                if count > position_counts[(class_name, merged_object_position[class_name])]:
                    merged_object_position[class_name] = position
            else:
                merged_object_position[class_name] = position
                
        
        print('Merged objects position ', merged_object_position)
        return merged_object_position
              
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
            cv2.imshow("Original image ", current_frame_draw)
            cv2.waitKey(0)

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
            cv2.rectangle(current_frame_draw, start_point, end_point, (255,255,255) , 4) 
            
            if obj.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                start_point_text = (obj.box_top_left_x-2, obj.box_top_left_y+25)
            else:
                start_point_text = (obj.box_top_left_x-2, obj.box_top_left_y-22)
                
            text_size, _ = cv2.getTextSize(f"{obj.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            text_w, text_h = text_size
            cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
            cv2.putText(current_frame_draw, f"{obj.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)

            current_frame_draw = current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]

            cv2.imwrite(self.node.complete_path_custom_face + current_datetime + obj_name + ".jpg", current_frame_draw)
            
            self.set_face(custom=current_datetime + obj_name)
            cv2.imshow("New Image", current_frame_draw)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("Error: Unable to load the image.")
    # else:
    #     print("Error: Image not found.")

    def detect_table_objects(self):
        i = 0
        nr_objects_high_priority_detected = 0
        self.detected_object = []

        for name, class_name in self.node.objects_classNames_dict.items():
            if class_name in self.classes_detected_wardrobe:
                self.priority_dict[class_name] = 'High'
                print(class_name + ' High')
            else:
                self.priority_dict[class_name] = 'Low'
                print(class_name + ' Low')

        # if hasattr(self.node, 'image') and self.node.image:
        #     if hasattr(self.node, 'objects') and self.node.objects:

        five_objects_detected = False
        detect_object = []
        
        list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]
        while not five_objects_detected:
            
            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False

            for pos in list_of_neck_position_search:

                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                #new_neck_pos = [ pos[0],  pos[1]]
                print('Neck: ', new_neck_pos)
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                time.sleep(1)

                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=5.0, with_hand=False)    

                self.objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects
                self.current_image = self.node.image
                bridge = CvBridge()
                # Convert ROS Image to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                self.image_objects_detected = cv_image
                current_frame_draw = self.image_objects_detected
                print('Will iterate for: ', self.nr_objects_detected)
                nr_objects_high_priority_detected = 0
                i = 0
                for detected_objects in self.objects_stored:
                    print(detected_objects.object_name, detected_objects.object_class)
                    if detected_objects.object_name in detect_object:
                        pass
                    else:
                        detect_object.append(self.objects_stored) 
                        if self.priority_dict[detected_objects.object_class] == 'High':
                            nr_objects_high_priority_detected += 1
                            print('Nr objects high: ', nr_objects_high_priority_detected)

                            """ start_point = (box_top_left_x, box_top_left_y)
                            end_point = (box_top_left_x + box_width, box_top_left_y + box_height)
                            # cv2.rectangle(current_frame_draw, start_point, end_point, (56, 56, 255) , 4) 
                                                            
                             current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{object_name}",
                                (box_top_left_x, box_top_left_y),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (255, 0, 0),
                                1,
                                cv2.LINE_AA
                            ) """


                        print('Object ' + detected_objects.object_name + ' from class ' + detected_objects.object_class + ' has ' + self.priority_dict[detected_objects.object_class] + 'priority')

                    i += 1

                print(i)
                if nr_objects_high_priority_detected >= 5:
                    print(self.objects_stored)
                    five_objects_detected = True
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    break
                else:
                    self.set_rgb(command=RED+BLINK_LONG)

        return nr_objects_high_priority_detected

    def choose_place_object_wardrobe(self, counter): 
        object_ = self.selected_objects[counter]
        obj_class = object_.object_class
        keywords = []
        print(object_.object_name)
        # print(obj_class)
        # print(self.selected_objects)
        # print(self.object_position.keys())
        if obj_class in self.object_position.keys():
            # print(self.object_position.items())
            position = self.object_position[obj_class][0]
            print(position)
            # self.set_speech(filename='storing_groceries/Place_the_object_sg', wait_for_end_of=True)
            keywords = position.split() # Split each position string into words and extend the keywords list

            location_filename = None
            class_filename = None
            # print(keywords)
            for condition, object_location in object_position_mapping.items():
                if all(keyword in keywords for keyword in condition):
                    # If conditions are met, relate the class name to the position
                    location_filename = f"storing_groceries/{object_location}"
                    class_filename = f"objects_classes/_{obj_class}"
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
        
        for name, class_name in self.node.objects_classNames_dict.items():
            if class_name in self.classes_detected_wardrobe:
                self.priority_dict[class_name] = 'High'
                print(class_name + ' High')
            else:
                self.priority_dict[class_name] = 'Low'
                print(class_name + ' Low')
      
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
        # images_table = self.image_objects_detected.copy()

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
        # image_name = f"image_{detected_object.object_name}.jpg"

            # Save the image with rectangles
        # cv2.imwrite(os.path.join(output_dir, image_name), images_table)

        # print(f"Saved image with rectangle for object {detected_object.object_name} as {image_name}")

    def activate_yolo_objects(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_objects_confidence=0.5, wait_for_end_of=True):
        
        # self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, characteristics=characteristics)
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, minimum_objects_confidence=minimum_objects_confidence)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def main(self):

        print("IN NEW MAIN")
        # time.sleep(1)

        while True:

            if self.state == self.Waiting_for_task_start:
                #print('State 0 = Initial')

                #self.set_face("charmie_face")

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=False, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                self.set_rgb(command=MAGENTA+SET_COLOUR)

                # self.wait_for_start_button()

                self.set_rgb(command=BLUE+SET_COLOUR)

                ###### WAITS FOR START BUTTON / DOOR OPEN
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
                         
                # next state
                # self.state = self.Approach_tables_first_time

                # self.state = 0 
                self.state = self.Approach_cabinet_first_time

            elif self.state == self.Approach_cabinet_first_time:
                #print('State 5 = Approaching cabinet for the first time')

                # self.set_speech(filename="storing_groceries/sg_collected_objects_1st_round", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ###### MOVEMENT TO THE CABINET

                self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)
                

                nr_classes_detected = 0
                list_of_neck_position_search = [[0, 0], [0, 15], [0, 30], [0, 45]]
                position_index = 0

                self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)
                
                self.opening_doors = True
                door_opened = None
                
                while door_opened == None:
                    print(door_opened)
                    pos_offset = list_of_neck_position_search[position_index]
                    new_neck_pos = [self.look_cabinet_center[0] + pos_offset[0], self.look_cabinet_center[1] + pos_offset[1]]
                    
                    # Set the neck position
                    self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                    time.sleep(3)
                    door_opened = self.analysis_cabinet()
                    
                    position_index = (position_index + 1) % len(list_of_neck_position_search)
                    
                print(door_opened)
                position_index = 0
                
                # navegar até ponto perto da porta
                # fazer set_arm com Right ou Left
                if door_opened ==  "Right":
                    print(door_opened)
                    # set_arm(open left door)
                    
                elif door_opened ==  "Left":
                    print(door_opened)
                    # set_arm(open left door)
                    
                time.sleep(3)
                    
                # navegar de novo de costas até ao ponto antes

                while nr_classes_detected < 4:
                    

                    print('\n \n \n \n')

                    pos_offset = list_of_neck_position_search[position_index]
                    new_neck_pos = [self.look_cabinet_center[0] + pos_offset[0], self.look_cabinet_center[1] + pos_offset[1]]
                    print('pescoço: ', new_neck_pos)
                    
                    # Set the neck position
                    self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                    print('Neck: ', new_neck_pos)
                    time.sleep(1)
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

                    if nr_classes_detected < 4 :
                        self.set_rgb(command=RED+BLINK_LONG)
                        nr_classes_detected = 0

                    # Move to the next position
                    position_index = (position_index + 1) % len(list_of_neck_position_search)
                    print(position_index)

                    # Adicionar ajuste de pescoço ou então depois ajustar dentro do próprio analysis cabinet
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_priority()
                
                self.set_speech(filename="storing_groceries/sg_finished_analise_cabinet", wait_for_end_of=True) 

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                # next state
                self.state = self.Approach_tables_first_time

            elif self.state == self.Approach_tables_first_time:
                #print('State 1 = Approaching table for the first time')

                self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                ###### MOVEMENT TO THE KITCHEN COUNTER

                self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)

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

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True) 
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                
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
                                            
                # next state
                self.state = self.Picking_second_object
                
            

            elif self.state == self.Picking_second_object:
                
                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True)
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                
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

                self.state = self.Picking_third_object
                
            elif self.state == self.Picking_third_object:
                
                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True)
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                
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

                self.state = self.Picking_fourth_object

            elif self.state == self.Picking_fourth_object:
                
                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True)
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                
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

                self.state = self.Picking_fifth_object

            elif self.state == self.Picking_fifth_object:
                
                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.set_speech(filename="storing_groceries/sg_detected_single_object", wait_for_end_of=True)
                
                self.select_voice_audio(self.selected_objects[self.object_counter])

                
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

                self.state = self.Placing_first_object

            elif self.state == self.Placing_first_object:
                self.object_counter = 0
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_speech(filename="generic/moving_cabinet", wait_for_end_of=True)
                time.sleep(1)
                # MOVIMENTAR
                self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Placing_second_object

            elif self.state == self.Placing_second_object:
                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Placing_third_object

           

            elif self.state == self.Placing_third_object:
 
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Placing_fourth_object

            
            elif self.state == self.Placing_fourth_object:
         
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Placing_fifth_object

            

            elif self.state == self.Placing_fifth_object:
                
                # self.set_arm(command="help_pick_and_place_object", wait_for_end_of=True)
                self.set_speech(filename='storing_groceries/help_place_cabinet', wait_for_end_of=True)
                self.select_voice_audio(self.selected_objects[self.object_counter])
                # self.set_speech(filename='storing_groceries/help_place_object', wait_for_end_of=True)
                # self.set_arm(command="open_gripper", wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                self.choose_place_object_wardrobe(self.object_counter)
                # self.set_arm(command="close_gripper", wait_for_end_of=False)
                # self.set_arm(command="arm_go_rest", wait_for_end_of=False)

                self.state = self.Final_State
            
            elif self.state == self.Final_State:
                #print('State 15 = Finished task')
                # self.node.speech_str.command = "I have finished my storing groceries task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")
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