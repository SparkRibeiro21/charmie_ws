#!/usr/bin/env python3

# initial instructions:
# this is an example of the layout code for a task in our workspace
# It is used a threading system so the ROS2 functionalities are not blocked when executing the state machine
# So we create a thread for the main state machine of the task and another for the ROS2 funcitonalities
# It is being used Serve the Breakfast as an example, so when changing the code for a specific task:
# Ctrl+F "ServeBreakfast" and replace everything for the name of your task 

# THE MAIN GOAL IS FOR YOU TO ONLY CHANGE BELOW THE LINE THAT STATES: "def main(self):"

# The following code already has implemnted with examples the following modes:
# Speakers
# RGB
# Start Button
# Audio
# Face
# Neck
# Yolos
# Arm
# Door Start

# The following modules are still missing:
# Nav - TBD


"""
NEXT I PROVIDE AN EXAMPLE ON HOW THE CODE OF A TASK SHOULD BE MADE:

->  ->  ->  ->  ->  HOW TO CREATE A TASK?

1) IMPORT ALL ROS2 TOPICS/SERVICES NECESSARY AND WAIT_FOR_SERVICE (THIS IS TIAGO JOB, ASK HIM TO HELP OR EXPLAIN)
2) PLAN THE STATES AND SET THE STATES FOR YOUR TASK:

        self.Waiting_for_task_start = 0
        self.Approach_kitchen_counter = 1
        self.Picking_up_spoon = 2
        self.Picking_up_milk = 3
        self.Picking_up_cereal = 4
        self.Picking_up_bowl = 5
        self.Approach_kitchen_table = 6
        self.Placing_bowl = 7
        self.Placing_cereal = 8
        self.Placing_milk = 9
        self.Placing_spoon = 10
        self.Final_State = 11

# 3) CREATE THE STATE STRUCTURE:
        
        if self.state == self.Waiting_for_task_start:
                # your code here ...
                                
                # next state
                self.state = self.Approach_kitchen_counter

            elif self.state == self.Approach_kitchen_counter:
                # your code here ...
                                
                # next state
                self.state = self.Picking_up_spoon

            elif self.state == self.Picking_up_spoon:
                # your code here ...
                                
                # next state
                self.state = self.Picking_up_milk

            (...)

# 4) CREATE THE PSEUDOCODE OF EACH STATE:
            
            elif self.state == self.Picking_up_spoon:
                
                ##### NECK LOOKS AT TABLE

                ##### MOVES ARM TO TOP OF TABLE POSITION

                ##### SPEAK: Searching for objects

                ##### YOLO OBJECTS SEARCH FOR SPOON, FOR BOTH CAMERAS
                
                ##### SPEAK: Found spoon
                
                ##### SPEAK: Check face to see object detected

                ##### SHOW FACE DETECTED OBJECT

                ##### MOVE ARM TO PICK UP OBJECT 

                ##### IF AN ERROR IS DETECTED:

                    ##### SPEAK: There is a problem picking up the object

                    ##### MOVE ARM TO ERROR POSITION 
                
                    ##### NECK LOOK JUDGE

                    ##### SPEAK: Need help, put object on my hand as it is on my face

                    ##### SHOW FACE GRIPPER SPOON 

                    ##### WHILE OBJECT IS NOT IN GRIPPER:

                        ##### SPEAK: Close gripper in 3 2 1 

                        ##### ARM: CLOSE GRIPPER

                        ##### IF OBJECT NOT GRABBED: 

                            ##### SPEAK: There seems to be a problem, please retry.

                            ##### ARM OPEN GRIPPER
                        
                ##### NECK LOOK TRAY
                        
                ##### ARM PLACE OBJECT IN TRAY

                self.state = self.Picking_up_milk

# 5) REPLACE ALL THE SPEAKS IN PSEUDOCODE WITH self.set_speech(...), CREATE FILES IN ros2 run charmie_speakers save_audio

# 6) TEST ALL SENTENCES ALONE TO SEE IF EVERYTHING IS OK

# 5) REPLACE ALL THE FACE IN PSEUDOCODE WITH self.set_face(...)

# 6) TEST ALL FACES WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 7) REPLACE ALL THE START_BUTTON AND RGB IN PSEUDOCODE WITH self.set_rgb(...) and self.wait_for_start_button()

# 8) TEST ALL START_BUTTON AND RGB WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 9) REPLACE ALL THE AUDIO IN PSEUDOCODE WITH self.get_audio(...)

# 10) TEST ALL AUDIO WITH THE PREVIOUS GETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 11) REPLACE ALL THE NECK IN PSEUDOCODE WITH self.set_neck(...) OR ANY OF ALL THE OTHER FORMS TO SET THE NECK

# 12) TEST ALL NECK WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 13) REPLACE ALL THE YOLO DETECTIONS IN PSEUDOCODE WITH self.activate_yolo_pose(...) and self.activate_yolo_objects

# 14) TEST ALL YOLOS WITH THE PREVIOUS ACTIVATES ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 15) REPLACE ALL THE ARM MOVE WITH self.set_arm(...)

# 16) TEST ALL ARM MOVE WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# MISSING NAVIGATION ... (TIAGO)

"""

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, ListOfPoints, Obstacles, ArmController
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, Trigger, SetFace

import cv2 
import threading
import time
from cv_bridge import CvBridge
import math
from datetime import datetime
import numpy as np
from pathlib import Path



# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class SticklerForTheRulesNode(Node):

    def __init__(self):
        super().__init__("SticklerForTheRules")
        self.get_logger().info("Initialised CHARMIE SticklerForTheRules Node")

        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        # self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)

        # Search for Person debug publisher
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)

         # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Obstacles
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obstacles_callback, 10)
        
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        # self.arm_trigger_client = self.create_client(Trigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(Trigger, "nav_trigger")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # Audio
        # while not self.get_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Audio Server...")
        # while not self.calibrate_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Calibrate Audio Server...")
        
        # Audio
        # while not self.get_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Audio Server...")
        # while not self.calibrate_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Calibrate Audio Server...")
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        while not self.neck_track_person_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Face
        while not self.face_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Face Command...")
        # while not self.neck_track_object_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        
        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_arm = False
        self.waited_for_end_of_face = False

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        self.obstacles = Obstacles()
        
        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.calibrate_audio_success = True
        self.calibrate_audio_message = ""
        self.audio_command = ""
        self.face_success = True
        self.face_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        self.track_object_success = True
        self.track_object_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.arm_success = True
        self.arm_message = ""
        self.navigation_success = True
        self.navigation_message = ""
        self.flag_navigation_reached = False


        self.get_neck_position = [1.0, 1.0]


    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()
        
        cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        cv2.waitKey(10)

    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

        current_frame = self.br.imgmsg_to_cv2(self.detected_objects.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()


        # img = [0:720, 0:1280]
        corr_image = False
        thresh_h = 50
        thresh_v = 200

        if self.detected_objects.num_objects > 0:

            x_min = 1280
            x_max = 0
            y_min = 720
            y_max = 0

            for object in self.detected_objects.objects:      
            
                if object.object_class == "Dishes":
                    corr_image = True

                    if object.box_top_left_x < x_min:
                        x_min = object.box_top_left_x
                    if object.box_top_left_x+object.box_width > x_max:
                        x_max = object.box_top_left_x+object.box_width

                    if object.box_top_left_y < y_min:
                        y_min = object.box_top_left_y
                    if object.box_top_left_y+object.box_height > y_max:
                        y_max = object.box_top_left_y+object.box_height

                    start_point = (object.box_top_left_x, object.box_top_left_y)
                    end_point = (object.box_top_left_x+object.box_width, object.box_top_left_y+object.box_height)
                    cv2.rectangle(current_frame_draw, start_point, end_point, (255,255,255) , 4) 

                    cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
                    
            
            for object in self.detected_objects.objects:      
                
                if object.object_class == "Dishes":
                
                    if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
                    else:
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
                        
                    # just to test for the "serve the breakfast" task...
                    aux_name = object.object_name
                    if object.object_name == "Fork" or object.object_name == "Knife":
                        aux_name = "Spoon"
                    elif object.object_name == "Plate" or object.object_name == "Cup":
                        aux_name = "Bowl"

                    text_size, _ = cv2.getTextSize(f"{aux_name}", cv2.FONT_HERSHEY_DUPLEX, 1, 1)
                    text_w, text_h = text_size
                    cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
                    cv2.putText(current_frame_draw, f"{aux_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        if corr_image:
            # current_frame_draw = current_frame_draw[x_min:y_min, x_max,y_max]
            # img = current_frame_draw[y_min:y_max, x_min,x_max]
            cv2.imshow("c", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)])
            cv2.waitKey(10)
            pass
        # cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
        # cv2.waitKey(10)

        # cv2.imwrite("object_detected_test4.jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        # cv2.waitKey(10)

    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        # self.arm_ready = True
        self.waited_for_end_of_arm = True
        self.arm_success = flag.data
        if flag.data:
            self.arm_message = "Arm successfully moved"
        else:
            self.arm_message = "Wrong Movement Received"

        self.get_logger().info("Received Arm Finished")

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

    ### OBSTACLES
    def obstacles_callback(self, obs: Obstacles):
        self.obstacles = obs

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag
    
    ### ACTIVATE YOLO POSE SERVER FUNCTIONS ###
    def call_activate_yolo_pose_server(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False):
        request = ActivateYoloPose.Request()
        request.activate = activate
        request.only_detect_person_legs_visible = only_detect_person_legs_visible
        request.minimum_person_confidence = minimum_person_confidence
        request.minimum_keypoints_to_detect_person = minimum_keypoints_to_detect_person
        request.only_detect_person_arm_raised = only_detect_person_arm_raised
        request.only_detect_person_right_in_front = only_detect_person_right_in_front
        request.characteristics = characteristics

        self.activate_yolo_pose_client.call_async(request)

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


    #### AUDIO SERVER FUNCTIONS #####
    def call_audio_server(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, wait_for_end_of=True):
        request = GetAudio.Request()
        request.yes_or_no = yes_or_no
        request.receptionist = receptionist
        request.gpsr = gpsr
        request.restaurant = restaurant

        future = self.get_audio_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_audio)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.command))
            self.audio_command = response.command
            # self.track_object_success = response.success
            # self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_calibrate_audio_server(self, wait_for_end_of=True):
        request = CalibrateAudio.Request()

        future = self.calibrate_audio_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_calibrate_audio)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_calibrate_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_person_success = response.success
            self.track_person_message = response.message
            # self.track_object_success = response.success
            # self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_calibrate_audio = True
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


    #### NECK SERVER FUNCTIONS #####
    def call_neck_track_person_server(self, person, body_part="Head", wait_for_end_of=True):
        request = TrackPerson.Request()
        request.person = person
        request.body_part = body_part

        future = self.neck_track_person_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_person)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_person(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_person_success = response.success
            self.track_person_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_person = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_neck_track_object_server(self, object, wait_for_end_of=True):
        request = TrackObject.Request()
        request.object = object

        future = self.neck_track_object_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_object)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_object(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_object_success = response.success
            self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_object = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = SticklerForTheRulesNode()
    th_main = threading.Thread(target=ThreadMainSticklerForTheRules, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainSticklerForTheRules(node: SticklerForTheRulesNode):
    main = SticklerForTheRulesMain(node)
    main.main()

class SticklerForTheRulesMain():

    def __init__(self, node: SticklerForTheRulesNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message

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

    def get_audio(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, question="", face_hearing="charmie_face_green", wait_for_end_of=True):

        if yes_or_no or receptionist or gpsr or restaurant:

            # this code continuously asks for new audio info eveytime it gets an error for mishearing
            audio_error_counter = 0
            keywords = "ERROR"
            while keywords=="ERROR":
                
                self.set_speech(filename=question, wait_for_end_of=True)
                self.set_face(face_hearing)
                self.node.call_audio_server(yes_or_no=yes_or_no, receptionist=receptionist, gpsr=gpsr, restaurant=restaurant, wait_for_end_of=wait_for_end_of)
                
                if wait_for_end_of:
                    while not self.node.waited_for_end_of_audio:
                        pass
                self.node.waited_for_end_of_audio = False
                self.set_face("charmie_face")

                keywords = self.node.audio_command  
                
                if keywords=="ERROR":
                    audio_error_counter += 1

                    if audio_error_counter == 2:
                        self.set_speech(filename="generic/please_wait", wait_for_end_of=True)
                        self.calibrate_audio(wait_for_end_of=True)
                        audio_error_counter = 0

                    self.set_speech(filename="generic/not_understand_please_repeat", wait_for_end_of=True)

            return self.node.audio_command  

        else:
            self.node.get_logger().error("ERROR: No audio type selected")
            return "ERROR: No audio type selected" 

    def calibrate_audio(self, wait_for_end_of=True):
            
        self.node.call_calibrate_audio_server(wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_calibrate_audio:
                pass
        self.node.waited_for_end_of_calibrate_audio = False

        return self.node.calibrate_audio_success, self.node.calibrate_audio_message 
    
    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        self.node.call_face_command_server(command=command, custom=custom, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_face:
                pass
        self.node.waited_for_end_of_face = False

        return self.node.face_success, self.node.face_message

    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message
    
    def set_neck_coords(self, position=[], ang=0.0, wait_for_end_of=True):

        if len(position) == 2:
            self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=0.0, tilt=ang, flag=True, wait_for_end_of=wait_for_end_of)
        elif len(position) == 3:
            print("You tried neck to coordintes using (x,y,z) please switch to (x,y,theta)")
            pass
            # The following line is correct, however since the functionality is not implemented yet, should not be called
            # self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=position[2], tilt=0.0, flag=False, wait_for_end_of=wait_for_end_of)
        else:
            print("Something went wrong")
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_coords:
            pass
        self.node.waited_for_end_of_neck_coords = False

        return self.node.neck_success, self.node.neck_message
    
    def get_neck(self, wait_for_end_of=True):
    
        self.node.call_get_neck_position_server()
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False


        return self.node.get_neck_position[0], self.node.get_neck_position[1] 
    
    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def activate_yolo_objects(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5, wait_for_end_of=True):
        
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, activate_objects_hand=activate_objects_hand, activate_shoes_hand=activate_shoes_hand, activate_doors_hand=activate_doors_hand, minimum_objects_confidence=minimum_objects_confidence, minimum_shoes_confidence=minimum_shoes_confidence, minimum_doors_confidence=minimum_doors_confidence)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def track_person(self, person, body_part="Head", wait_for_end_of=True):

        self.node.call_neck_track_person_server(person=person, body_part=body_part, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_person:
            pass
        self.node.waited_for_end_of_track_person = False

        return self.node.track_person_success, self.node.track_person_message
 
    def track_object(self, object, wait_for_end_of=True):

        self.node.call_neck_track_object_server(object=object, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_object:
            pass
        self.node.waited_for_end_of_track_object = False

        return self.node.track_object_success, self.node.track_object_message   

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

    def search_for_person_2(self, tetas):

        self.activate_yolo_pose(activate=True, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=True)                

        self.set_rgb(WHITE+ALTERNATE_QUARTERS)

        time.sleep(3.0)
        
        imshow_detected_people = True

        total_person_detected = []
        person_detected = []
        total_cropped_people = []
        cropped_people = []
        points = []
        croppeds = []

        people_ctr = 0
        delay_ctr = 0

        print("Started")
        for t in tetas:
            self.set_neck(position=[t, -25])
            time.sleep(2)

            for people in self.node.detected_people.persons:
                people_ctr+=1
                print(" - ", people.index, people.position_absolute.x,people.position_absolute.y, people.position_absolute.z)
                print(" - ", people.index, people.position_relative.x,people.position_relative.y, people.position_relative.z)
                aux = (people.position_absolute.x, people.position_absolute.y) 
                person_detected.append(aux)
                points.append(aux)

                if imshow_detected_people:

                    y1 = people.box_top_left_y
                    y2 = people.box_top_left_y + people.box_height

                    x1 = people.box_top_left_x
                    x2 = people.box_top_left_x + people.box_width

                    print(y1, y1, x1,x2)
                    br = CvBridge()
                    current_frame = br.imgmsg_to_cv2(self.node.detected_people.image_rgb, "bgr8")
                    cropped_image = current_frame[y1:y2, x1:x2]
                    cropped_people.append(cropped_image)
                    
                    croppeds.append(cropped_image)

            total_person_detected.append(person_detected.copy())
            total_cropped_people.append(cropped_people.copy())
            print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()          
            cropped_people.clear()


        ### DETECTS ALL THE PEOPLE SHOW IN EVERY FRAME ###
        
        print(total_person_detected)
        print(len(points))
       
        filtered_persons = []
        filtered_persons_cropped = []
        new_filtered_persons_cropped = []
        for frame in range(len(total_person_detected)):
            print(filtered_persons)
            # print(filtered_persons_cropped)

            if not len(filtered_persons):
                for person in range(len(total_person_detected[frame])):
                    filtered_persons.append(total_person_detected[frame][person])
                    filtered_persons_cropped.append(total_cropped_people[frame][person])
                    new_filtered_persons_cropped.append(total_cropped_people[frame][person])
            else:
                for person in range(len(total_person_detected[frame])):
                    same_person_ctr = 0
                    same_person_coords = (0,0)
                    for filtered in range(len(filtered_persons)): #_aux:

                        # print("??? ", total_person_detected[frame][person], filtered_persons[filtered])
                        dist = math.dist(total_person_detected[frame][person], filtered_persons[filtered])
                        # print("person:", person, "filtered:", filtered, "dist:", dist)
                        
                        if dist < 1.0:
                            same_person_ctr+=1
                            same_person_coords = filtered_persons[filtered]
                            same_person_cropped = filtered_persons_cropped[filtered] 

                            # print(same_person_cropped.shape)
                        
                    
                    if same_person_ctr > 0:

                        # just debug
                        for p in filtered_persons_cropped:
                            print(p.shape)

                        print("---", same_person_cropped.shape)

                        
                        # print(same_person_cropped)
                        # print(total_cropped_people[frame][person])
                        # print(len(same_person_cropped), len(total_cropped_people[frame][person]))
                        # print(same_person_cropped.shape[0], same_person_cropped.shape[1])
                        # print(total_cropped_people[frame][person].shape[0], total_cropped_people[frame][person].shape[1])
                        
                        # the same person is the person on the first frame, whereas total_cropped_people[frame][person] is the same person on the second frame
                        # if total_cropped_people[frame][person].shape[1] > same_person_cropped.shape[1]:
                            # filtered_persons_cropped.remove(same_person_cropped)
                            # filtered_persons_cropped.append(total_cropped_people[frame][person])
                            # total_cropped_people[frame][person] = 
                            # pass
                        # for p in filtered_persons_cropped:
                        #     if p.shape == same_person_cropped.shape:
                        #         filtered_persons_cropped.remove(p)

                        #         del my_list[index_to_remove]  # Removes the value at index 2 (i.e., the third element)
                        
                        
                        #         filtered_persons_cropped.remove(p)
                        
                        
                        # filtered_persons_cropped.remove(same_person_cropped)
                        # if total_cropped_people[frame][person].shape[1] > same_person_cropped.shape[1]:
                        #     filtered_persons_cropped.append(total_cropped_people[frame][person])
                        #     new_filtered_persons_cropped.append(total_cropped_people[frame][person])
                        # else:
                        #     filtered_persons_cropped.append(same_person_cropped)
                        #     new_filtered_persons_cropped.append(same_person_cropped)
                            
                        #print(same_person_ctr, same_person_coords, person)
                        filtered_persons.remove(same_person_coords)

                        avg_person = ((total_person_detected[frame][person][0]+same_person_coords[0])/2, (total_person_detected[frame][person][1]+same_person_coords[1])/2)
                        # print(avg_person)
                        filtered_persons.append(avg_person)
                        points.append(avg_person)

                    else:
                    
                        filtered_persons.append(total_person_detected[frame][person])
                        filtered_persons_cropped.append(total_cropped_people[frame][person])
                        new_filtered_persons_cropped.append(total_cropped_people[frame][person])
        
        # same time for all people
        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        self.custom_face_filename = current_datetime
        
        # ctr = 0
        # for c in croppeds:
        #     ctr+=1
        #     cv2.imwrite("Person Detected_"+str(ctr)+".jpg", c)
        ctr = 0
        filenames = []
        for c in new_filtered_persons_cropped:
            ctr+=1
            path = current_datetime + "_person_" + str(ctr) 
            filenames.append(path)
            
            # cv2.imwrite("Person Filtered_"+str(ctr)+".jpg", c)
            cv2.imwrite(self.node.complete_path_custom_face + path + ".jpg", c) 
        
        print("Finished")


        print("---", filtered_persons)
        points_to_send = ListOfPoints()
        # for debug, see all points and the average calculations
        # for p in points:
        for p in filtered_persons:
            aux = Point()
            aux.x = float(p[0])
            aux.y = float(p[1])
            aux.z = 0.0
            points_to_send.coords.append(aux)

        print(filtered_persons)

        # print(points_to_send)
        self.node.search_for_person_publisher.publish(points_to_send)
        
        self.activate_yolo_pose(activate=False)                

        return filtered_persons, filenames
    
    def check_if_charmie_is_being_followed(self):
        self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)
        
        detected_person_temp = Yolov8Pose()
        self.set_rgb(WHITE+HALF_ROTATE)
        time.sleep(2.5)
        detected_person_temp = self.node.detected_people  
        print(detected_person_temp.num_person)
        # print(detected_person_temp.persons)
        not_following = True
        while not_following:
            detected_person_temp = self.node.detected_people 
            if detected_person_temp.num_person == 0:  
                self.set_rgb(YELLOW+HALF_ROTATE)
                self.set_speech(filename="sftr/Come_behind_me", wait_for_end_of=True)
                self.set_rgb(RED+BLINK_QUICK)
                time.sleep(1)
            else:
                self.set_rgb(GREEN+HALF_ROTATE)
                self.set_speech(filename="sftr/Thanks_for_following", wait_for_end_of=True)
                not_following = False
          
    def check_if_charmie_is_being_followed2(self):
        self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)
        
        detected_person_temp = Yolov8Pose()
        self.set_rgb(WHITE+HALF_ROTATE)
        time.sleep(2.5)
        detected_person_temp = self.node.detected_people  
        print(detected_person_temp.num_person)
        # print(detected_person_temp.persons)
        not_following = True
        while not_following:
            detected_person_temp = self.node.detected_people 
            if detected_person_temp.num_person == 0:  
                self.set_rgb(YELLOW+HALF_ROTATE)
                self.set_speech(filename="sftr/Come_behind_me", wait_for_end_of=True)
                self.set_rgb(RED+BLINK_QUICK)
                time.sleep(1)
            else:
                self.set_rgb(GREEN+HALF_ROTATE)
                self.set_speech(filename="sftr/Thanks_for_following_2", wait_for_end_of=True)
                not_following = False

    # main state-machine function
    def main(self):
        
        # States in SticklerForTheRules Task
        self.Waiting_for_start_button = 0
        self.Navigation_forbidden_room = 1
        self.Detect_people_forbidden_room = 2
        self.Speak_forbidden_room = 3
        self.Navigation_out_forbidden_room = 4
        self.Final_State = 5
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_back = [-180, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        
        self.initial_position = [-1.0, 3.0, 180.0]
        self.pre_door_to_office = [-2.0, 3.0, 90.0]
        self.inside_office = [-4.0, 3.0, 90.0]
        self.center_office = [-5.0, 3.5, 180.0]
        self.after_leaving_bedroom = [-5.3, 5.0, 180.0]
        self.pre_door_to_bedroom = [-5.5, 5.0, 0.0]
        self.inside_bedroom = [-5.5, 6.2, 0.0]
        self.nr_times_tracking_fb = 0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_start_button 

        # DEBUG:
        # self.state = self.Detect_people_forbidden_room 
        #self.initial_position = [-4.5, 5.0, 0.0]
        #self.set_initial_position(self.initial_position)
        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In SticklerForTheRules Main...")

        while True:

            if self.state == self.Waiting_for_start_button:
                print('State 0 = Waiting_for_start_button')

                self.set_face("charmie_face")
                self.activate_yolo_pose(activate=False)

                #NECK: LOOKS IN FRONT
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_rgb(MAGENTA+ALTERNATE_QUARTERS)
                
                # SET START POSITION
                self.set_initial_position(self.initial_position)
                
                #START TASK
                self.set_speech(filename="sftr/start_sftr", wait_for_end_of=True)
                #WAITING START BUTTON
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)
                self.wait_for_start_button()
                
                # NAVIGATION POSE
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                # MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                self.state = self.Navigation_forbidden_room


            elif self.state == self.Navigation_forbidden_room:
                print('State 1 = Navigation forbidden room')

                self.set_speech(filename="sftr/go_forbidden_room", wait_for_end_of=False)
                #MOVE TO THE ROOM DOOR
                
                self.set_navigation(movement="rotate", target=self.pre_door_to_office, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.pre_door_to_office, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.inside_office, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_office, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.pre_door_to_bedroom, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.pre_door_to_bedroom, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.inside_bedroom, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_bedroom, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="orientate", absolute_angle = 0.0, flag_not_obs=True, wait_for_end_of=True)

                # next state
                self.state = self.Detect_people_forbidden_room

            elif self.state == self.Detect_people_forbidden_room:
                print('State 2 = Detect people forbidden room')
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.nr_times_tracking_fb += 1
                
                print('nr times tracking = ', self.nr_times_tracking_fb)
                
                self.set_speech(filename="sftr/start_searching", wait_for_end_of=False)
                self.set_rgb(YELLOW+ROTATE)
                #REPLACE: LOOK TO THE ROOM
                            
                tetas = [-30, 0, 30]
                coords_of_people, images_of_people = self.search_for_person_2(tetas)
                print('Coordinates of people with legs detected: ', coords_of_people)
                if coords_of_people == []:
                    detected_person = False
                else:
                    print('\n\n Nr de pessoas detetadas: ', len(coords_of_people))
                    nr_persons_detected_bedroom = len(coords_of_people)
                    detected_person = True
                    self.set_rgb(POLICE)
                    """ for i in images_of_people:
                        self.set_face(custom=i)
                        time.sleep(3) """
                
                for c in coords_of_people:
                    # self.set_neck_coords(position=c, wait_for_end_of=True)
                    print('\n\n\n\ C:' , c)
                    print('x: ', c[0])
                    print('y: ', c[1])
                    
                    if -6.5 < c[0] < -2.5 and 5.6 < c[1] < 11.0:
                        coordinates_guest = c
                        self.set_neck_coords(position=c, ang= -5.0, wait_for_end_of=True)
                        
                        time.sleep(2)
                        self.state = self.Speak_forbidden_room
                        break
                    else:
                        self.state = self.Detect_people_forbidden_room
                    #ONLY FOR DEBUG

        
                # self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                   # self.state = self.Speak_forbidden_room

            elif self.state == self.Speak_forbidden_room:
                print('State 3 = Speak forbidden room')
                
                if detected_person ==True:
                    self.set_speech(filename="sftr/detection_forbidden_room", wait_for_end_of=True)
                    #REPLACE: LOOK TO THE PERSON
                    
                    # index = len(coords_of_people) - nr_persons_detected_bedroom
                    # print('Index da pessoa que eu detetei e estou a olhar: ', index)
                    
                    print('Coordinates of the  guest I am looking at: ', coordinates_guest)

                    # self.set_neck(position=coords_of_people[0], wait_for_end_of=True)
                    self.set_speech(filename="sftr/looking_guest_forbidden_room", wait_for_end_of=True)
                    self.set_speech(filename="sftr/guest_breaking_rule_forbidden_room", wait_for_end_of=True)
                    self.set_speech(filename="sftr/action_forbidden_room", wait_for_end_of=True)
                    self.set_speech(filename="sftr/follow_robot_outside_room", wait_for_end_of=True)
                    self.state = self.Navigation_out_forbidden_room
                    
                else:
                    if self.nr_times_tracking_fb == 3:
                        self.state == self.Final_State
                    else:
                        self.state = self.Detect_people_forbidden_room
                    
                # next state
           
            elif self.state == self.Navigation_out_forbidden_room:
                print('State 4 = Navigation out forbidden room')
                nr_persons_detected_bedroom -= 1
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                #MOVE TO OUT OF THE BEDROOM
                self.set_navigation(movement="rotate", target=self.after_leaving_bedroom, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="orientate", absolute_angle = 180.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_neck(position=self.look_back, wait_for_end_of=True)
                self.set_speech(filename="sftr/follow_me", wait_for_end_of=True)
                time.sleep(2)
                self.set_navigation(movement="move", target=self.after_leaving_bedroom, flag_not_obs=False, wait_for_end_of=True)
                self.check_if_charmie_is_being_followed()
                self.set_navigation(movement="move", target=self.center_office, flag_not_obs=True, wait_for_end_of=True)
                self.check_if_charmie_is_being_followed2()
                self.set_speech(filename="sftr/no_longer_breaking_rule", wait_for_end_of=False)
                self.set_rgb(GREEN+BLINK_LONG)
                        
                if nr_persons_detected_bedroom == 0:
                    print('No more persons detected')
                    self.state = self.Final_State 
                else: 
                    print('Still persons in the room')
                    self.set_navigation(movement="rotate", target=self.pre_door_to_bedroom, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.pre_door_to_bedroom, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="rotate", target=self.inside_bedroom, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.inside_bedroom, flag_not_obs=False, wait_for_end_of=True)
                    self.set_navigation(movement="orientate", absolute_angle = 0.0, flag_not_obs=True, wait_for_end_of=True)

                    self.state = self.Detect_people_forbidden_room
                                
            elif self.state == self.Final_State:
                
                print("Finished task!!!")
                #NECK: LOOK IN FRONT
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_speech(filename="sftr/finish_sftr", wait_for_end_of=True)
                #NECK: LOOK TO THE FLOOR
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_rgb(BLUE+ROTATE)
                # Lock after finishing task
                while True:
                    pass

            else:
                pass
