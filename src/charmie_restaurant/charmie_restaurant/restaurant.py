#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, ListOfPoints, ListOfDetectedObject, ListOfDetectedPerson, Obstacles
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger, SetFace, ActivateObstacles

import cv2 
import threading
import time
from cv_bridge import CvBridge
import math
import numpy as np
from pathlib import Path
from datetime import datetime

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class RestaurantNode(Node):

    def __init__(self):
        super().__init__("Restaurant")
        self.get_logger().info("Initialised CHARMIE Restaurant Node")

        self.DIST_CLIENT = 6.0

        # path to save detected people in search for person
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
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 
         # Search for Person debug publisher
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)


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

        # Search for person and object 
        self.search_for_person_detections_publisher = self.create_publisher(ListOfDetectedPerson, "search_for_person_detections", 10)
        self.search_for_object_detections_publisher = self.create_publisher(ListOfDetectedObject, "search_for_object_detections", 10)
        # Obstacles
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obstacles_callback, 10)
        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")
        

        


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # # Audio
        while not self.get_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Audio Server...")
        # while not self.calibrate_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Calibrate Audio Server...")
        # # Face
        # while not self.face_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Face Command...")

       
        
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        while not self.get_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        while not self.neck_track_person_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        while not self.neck_track_object_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Yolos
        while not self.activate_yolo_pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Obstacles
        while not self.activate_obstacles_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Activate Obstacles Command...")
       
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_face = False

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        self.detected_objects = Yolov8Objects()
        self.detected_doors = Yolov8Objects()
        self.detected_shoes = Yolov8Objects()
        self.detected_objects_hand = Yolov8Objects()
        self.detected_doors_hand = Yolov8Objects()
        self.detected_shoes_hand = Yolov8Objects()
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
        self.track_person_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.navigation_success = True
        self.navigation_message = ""
        self.activate_obstacles_success = True
        self.activate_obstacles_message = ""

    def call_activate_obstacles_server(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False):
        request = ActivateObstacles.Request()
        request.activate_lidar_up = obstacles_lidar_up
        request.activate_lidar_bottom = obstacles_lidar_bottom
        request.activate_camera_head = obstacles_camera_head

        self.activate_obstacles_client.call_async(request)

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    def obstacles_callback(self, obs: Obstacles):
        self.obstacles = obs

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
    node = RestaurantNode()
    th_main = threading.Thread(target=ThreadMainRestaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainRestaurant(node: RestaurantNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: RestaurantNode):
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
    
    def set_face(self, command="", custom="", wait_for_end_of=False):
        
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
    
    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message
    
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

    def search_for_person(self, tetas, delta_t=3.0, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=False):

        self.activate_yolo_pose(activate=True, characteristics=characteristics, only_detect_person_arm_raised=only_detect_person_arm_raised, only_detect_person_legs_visible=only_detect_person_legs_visible) 
        self.set_speech(filename="generic/search_people", wait_for_end_of=False)
        self.set_rgb(WHITE+ALTERNATE_QUARTERS)
        time.sleep(0.5)
        
        total_person_detected = []
        person_detected = []
        people_ctr = 0

        ### MOVES NECK AND SAVES DETECTED PEOPLE ###
        
        for t in tetas:
            self.set_rgb(RED+SET_COLOUR)
            self.set_neck(position=t, wait_for_end_of=True)
            time.sleep(1.0) # 0.5
            self.set_rgb(WHITE+SET_COLOUR)

            start_time = time.time()
            while (time.time() - start_time) < delta_t:        
                local_detected_people = self.node.detected_people
                for temp_people in local_detected_people.persons:
                    
                    is_already_in_list = False
                    person_already_in_list = DetectedPerson()
                    for people in person_detected:

                        if temp_people.index_person == people.index_person:
                            is_already_in_list = True
                            person_already_in_list = people

                    if is_already_in_list:
                        person_detected.remove(person_already_in_list)
                    elif temp_people.index_person > 0: # debug
                        # print("added_first_time", temp_people.index_person, temp_people.position_absolute.x, temp_people.position_absolute.y)
                        self.set_rgb(GREEN+SET_COLOUR)
                    
                    if temp_people.index_person > 0:
                        person_detected.append(temp_people)
                        people_ctr+=1

            # DEBUG
            # print("people in this neck pos:")
            # for people in person_detected:
            #     print(people.index_person, people.position_absolute.x, people.position_absolute.y)
        
            total_person_detected.append(person_detected.copy())
            # print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()          

        self.activate_yolo_pose(activate=False)
        # print(total_person_detected)

        # DEBUG
        # print("TOTAL people in this neck pos:")
        # for frame in total_person_detected:
        #     for people in frame:    
        #         print(people.index_person, people.position_absolute.x, people.position_absolute.y)
        #     print("-")

        ### DETECTS ALL THE PEOPLE SHOW IN EVERY FRAME ###
        
        filtered_persons = []

        for frame in range(len(total_person_detected)):

            to_append = []
            to_remove = []

            if not len(filtered_persons):
                # print("NO PEOPLE", frame)
                for person in range(len(total_person_detected[frame])):
                    to_append.append(total_person_detected[frame][person])
            else:
                # print("YES PEOPLE", frame)

                MIN_DIST = 1.0 # maximum distance for the robot to assume it is the same person

                for person in range(len(total_person_detected[frame])):
                    same_person_ctr = 0

                    for filtered in range(len(filtered_persons)):

                        dist = math.dist((total_person_detected[frame][person].position_absolute.x, total_person_detected[frame][person].position_absolute.y), (filtered_persons[filtered].position_absolute.x, filtered_persons[filtered].position_absolute.y))
                        # print("new:", total_person_detected[frame][person].index_person, "old:", filtered_persons[filtered].index_person, dist)
                        
                        if dist < MIN_DIST:
                            same_person_ctr+=1
                            same_person_old = filtered_persons[filtered]
                            same_person_new = total_person_detected[frame][person]
                            # print("SAME PERSON")                        
                    
                    if same_person_ctr > 0:

                        same_person_old_distance_center = abs(1280/2 - same_person_old.body_center_x) 
                        same_person_new_distance_center = abs(1280/2 - same_person_new.body_center_x) 

                        # print("OLD (pixel):", same_person_old.body_center_x, same_person_old_distance_center)
                        # print("NEW (pixel):", same_person_new.body_center_x, same_person_new_distance_center)

                        if same_person_new_distance_center < same_person_old_distance_center: # person from newer frame is more centered with camera center
                            to_remove.append(same_person_old)
                            to_append.append(same_person_new)
                        else: # person from older frame is more centered with camera center
                            pass # that person is already in the filtered list so we do not have to do anything, this is here just for explanation purposes 

                    else:
                        to_append.append(total_person_detected[frame][person])

            for p in to_remove:
                if p in filtered_persons:
                    # print("REMOVED: ", p.index_person)
                    filtered_persons.remove(p)
                # else:
                    # print("TRIED TO REMOVE TWICE THE SAME PERSON")
            to_remove.clear()  

            for p in to_append:
                # print("ADDED: ", p.index_person)
                filtered_persons.append(p)
            to_append.clear()
            
        self.set_neck(position=[0, 0], wait_for_end_of=False)
        self.set_rgb(BLUE+HALF_ROTATE)

        sfp_pub = ListOfDetectedPerson()
        # print("FILTERED:")
        for p in filtered_persons:
            sfp_pub.persons.append(p)
        #     print(p.index_person)
        self.node.search_for_person_detections_publisher.publish(sfp_pub)

        return filtered_persons

    def detected_person_to_face_path(self, person, send_to_face):

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        
        cf = self.node.br.imgmsg_to_cv2(person.image_rgb_frame, "bgr8")
        just_person_image = cf[person.box_top_left_y:person.box_top_left_y+person.box_height, person.box_top_left_x:person.box_top_left_x+person.box_width]
        # cv2.imshow("Search for Person", just_person_image)
        # cv2.waitKey(100)
        
        face_path = current_datetime + str(person.index_person)
        
        cv2.imwrite(self.node.complete_path_custom_face + face_path + ".jpg", just_person_image) 
        time.sleep(0.5)

        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path


    def search_for_person_2(self, tetas):

        self.activate_yolo_pose(activate=True, characteristics=False, only_detect_person_arm_raised=True)                

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
            self.set_neck(position=[t, -10])
            time.sleep(2)

            for people in self.node.detected_people.persons:
                people_ctr+=1
                print(" - ", people.index_person, people.position_absolute.x,people.position_absolute.y, people.position_absolute.z)
                print(" - ", people.index_person, people.position_relative.x,people.position_relative.y, people.position_relative.z)
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

        return filtered_persons, filenames
    
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

    # main state-machine function
    def main(self):

        print('inside')
        
        # States in Restaurant Task
        self.Waiting_for_task_start = 0
        self.Looking_for_barman = 1
        self.Detecting_waving_customers = 2
        self.Approach_customer1_table = 3
        self.Receiving_order1_listen_and_confirm = 4
        self.Detecting_customer2_again = 5
        self.Approach_customer2_table = 6
        self.Receiving_order2_listen_and_confirm = 7
        self.Collect_orders_from_barman = 8
        self.Delivering_customer_order1 = 9
        self.Delivering_customer_order2 = 10
        self.Final_State = 11
        self.temp_state = 99
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_back = [-180, 0]
        self.look_navigation = [0, -50]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.look_customers = [0, 0]

        # Start Localisation Position
        self.initial_position = [0.0, 0.1, 0.0]

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start
        # self.state = self.Detecting_waving_customers
        self.set_face("charmie_face")


        self.all_orders = []
        self.counter = 0
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.just_one_waving = False

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Restaurant Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")

                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")

                # send speech command to speakers voice, intrucing the robot 
                self.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                
                # sends RGB value for debug
                self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)
                
                ##### SPEAK : I am ready to start the restaurant task. Waiting for start button to be pressed!
                self.set_speech(filename="restaurant/start_restaurant_task", wait_for_end_of=True)

                # waiting for start button
                self.wait_for_start_button()

                print('start button')

                self.set_rgb(command=CYAN+ALTERNATE_QUARTERS)

                # change face, to standard face
                self.set_face("charmie_face")

                self.state = self.Looking_for_barman

            elif self.state == self.Looking_for_barman:
                
                # FRASE "I WILL LOOK FOR THE BARMAN NOW. BARMAN, PLS RAISE YOUR HANDS"
                self.set_speech(filename="restaurant/search_barman", wait_for_end_of=True)

                tetas = [[180, 0], [90, 0], [-90, 0]]
                barman_not_found = True
                self.barman_position = []

                while barman_not_found:
                    people_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True)

                    print("FOUND:", len(people_found)) 
                    for p in people_found:
                        # self.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                        print("ID:", p.index_person)
                        print('Barman position', p.position_relative)
                        self.barman_position.append(p)
                        barman_not_found = False
                    
                    if barman_not_found:
                        self.set_speech(filename="restaurant/search_barman", wait_for_end_of=True)


                barman_final_position = 100000.0
                if len(self.barman_position) > 0:
                    for barman in self.barman_position:
                        dist = math.sqrt(barman.position_relative.x**2 + barman.position_relative.y**2)
                        if dist < barman_final_position:
                            barman_final_position = dist
                            self.barman_position = barman
                            print(barman.position_relative)


                if self.barman_position.position_relative.x < 0:
                    print('barman on my left')
                    
                else:
                    print('barman on my right')


                # self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], ang=-10, wait_for_end_of=True)     
                
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)   
                
                # BARMAN, ARE YOU THE GUY ON MY FACE?
                # self.set_speech(filename="restaurant/confirm_barman", wait_for_end_of=False)
                path = self.detected_person_to_face_path(person=self.barman_position, send_to_face=True)

                # YES OR NO
                ##### AUDIO: Listen "YES" OR "NO"
                ##### "Please say yes or no to confirm the order"
                confirmation = self.get_audio(yes_or_no=True, question="restaurant/confirm_barman", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                print("Finished:", confirmation)




                ##### Verifica a resposta recebida
                if confirmation.lower() == "yes":
                    self.set_rgb(command=GREEN+BLINK_LONG)
                    ##### SPEAK : Hello! Nice to meet you! My name is charmie and I am here to help you serve the customers.
                    self.set_speech(filename="restaurant/barman_meeting", wait_for_end_of=True)

                    ##### SPEAK : I am going to turn around and search for possible customers. See you soon
                    self.set_speech(filename="restaurant/go_search", wait_for_end_of=True)

                    #### NAVIGATION
                    # self.set_navigation(movement="orientate", absolute_angle= 180.0, flag_not_obs = True, wait_for_end_of=True)

                    self.set_neck(position=self.look_forward, wait_for_end_of=True)

                    self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                    
                
                    # next state
                    self.state = self.Detecting_waving_customers
                    # Nice meeting you

                elif confirmation.lower() == "no":
                    self.set_rgb(command=RED+BLINK_LONG)
                    ##### SPEAK: Sorry, TRY AGAIN
                    self.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                    self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                    self.state = self.Looking_for_barman
                    
                else:
                    self.set_rgb(command=YELLOW+BLINK_LONG)
                    self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)

                    ##### ERROR
                    print("ERROR")
                    self.state = self.Looking_for_barman


                print(self.state)

            elif self.state == self.Detecting_waving_customers:
                print("State:", self.state, "- Detecting_waving_customers")
                ##### BACK TO STANDART FACE
                self.set_face("charmie_face")

                ##### SPEAK: Searching for waving customers
                # self.set_speech(filename="restaurant/search_customers", wait_for_end_of=True)

                self.set_rgb(command=ORANGE+HALF_ROTATE)

                self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)


                ##### NECK: Rotate left and right for better view
                # tetas = [-120, -60, 0, 60, 120]
                ##### YOLO POSE SEARCH FOR ONLY_DETECT_PERSON_ARM_RAISED
                # coords_of_people, images_of_people = self.search_for_person_2(tetas)


                tetas = [[-45, 0], [0, 0], [45, 0]]
                customers_not_found = True
                customers_list = []

            
                customers_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True)

                print("FOUND:", len(customers_found)) 
                for p in customers_found:
                    # self.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                    print("ID:", p.index_person)
                    print('Customer position', p.position_relative)
                    customers_list.append(p)
                    
                print('Nr of detected customers waving: ', len(customers_list))

                self.set_neck(position=[0.0, 0.0], wait_for_end_of=True)

                ##### IF AN ERROR IS DETECTED:
                if len(customers_list) == 0:

                    self.set_rgb(command=RED+BLINK_QUICK)

                    ##### SPEAK: No costumers detected
                    self.set_speech(filename="restaurant/no_customers", wait_for_end_of=True)

                    ##### BACK TO initial searching
                    self.state = self.Detecting_waving_customers

                elif len(customers_list) == 1 and self.counter < 3:

                    # AQUI!
                    # ADICIONAR CLIENTES AO ARRAY E ORDENAR POR DISTÂNICAS

                    self.set_rgb(command=YELLOW+BLINK_QUICK)

                    ##### SPEAK: Found waving customers
                    self.set_speech(filename="restaurant/found_customer", wait_for_end_of=True)
                    dist_array = []

                    for p in customers_list:
                        self.set_neck_coords(position=[p.position_relative.x, p.position_relative.y], wait_for_end_of=True)
                        dist = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                        dist_array.append(dist)
                        print('\n \n coordenadas cliente :', p.position_relative.x, p.position_relative.y)
                        print('distance to client: ', dist)
                        
                        time.sleep(2)

                    print('\n\n dist all clients: ', dist_array)

                    self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], wait_for_end_of=True)
                    ##### SPEAK: Check face to see customers detected
                    self.set_speech(filename="restaurant/face_customer", wait_for_end_of=True)

                    path = self.detected_person_to_face_path(person=customers_list[0], send_to_face=True)
                    time.sleep(3)

                    self.state = self.Detecting_waving_customers

                elif len(customers_list) > 1 or self.counter > 2:

                    self.set_rgb(command=GREEN+BLINK_QUICK)

                    ##### SPEAK: Found waving customers
                    self.set_speech(filename="restaurant/found_customer", wait_for_end_of=True)
                    dist_array = []
                    ##### NECK: look waving customers
                    self.set_speech(filename="restaurant/look_detected_waving_customers", wait_for_end_of=False)

                    time.sleep(1)
                        
                    for p in customers_list:
                        self.set_neck_coords(position=[p.position_relative.x, p.position_relative.y], wait_for_end_of=True)
                        dist = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                        dist_array.append(dist)
                        print('\n \n coordenadas cliente :', p.position_relative.x, p.position_relative.y)
                        print('distance to client: ', dist)
                        
                        time.sleep(2)


                    # Step 1: Create a list of tuples (distance, index)
                    distance_with_indices = [(distance, index) for index, distance in enumerate(dist_array)]

                    # Step 2: Sort the list of tuples by distance
                    sorted_distances = sorted(distance_with_indices)

                    # Step 3: Use the sorted indices to reorder the persons list
                    sorted_persons = [customers_list[index] for _, index in sorted_distances]

                    # Print the results
                    print("Sorted Distances with Indices:", sorted_distances)
                    
                    for p in sorted_persons:
                        print("Positions:", p.position_relative, ' + ',p.index_person)


                    self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], wait_for_end_of=True)
                    ##### SPEAK: Check face to see customers detected
                    self.set_speech(filename="restaurant/face_customer", wait_for_end_of=True)

                    i = 0
                    person_count = 2
                    for p in sorted_persons :
                        if i == person_count:
                            break
                        path_customer = self.detected_person_to_face_path(person=sorted_persons[i], send_to_face=True)
                        print(f'sorted person {i} position {sorted_persons[i].position_relative.x}, {sorted_persons[i].position_relative.y}')
                
                
                        time.sleep(3)
                        i += 1
                            
                    
                
                    
                    
                    ##### SHOW FACE DETECTED CUSTOMER 
                    # path_customer_0 = self.detected_person_to_face_path(person=sorted_persons[0], send_to_face=True)
                    
                    # path_customer_1 = self.detected_person_to_face_path(person=sorted_persons[1], send_to_face=True)
                    
                    # print('sorted person 0 position:', sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y)
                    # print('sorted person 1 position:', sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y)
                    
                    # counter = 0 
                    # for i in sorted_persons:
                    #     counter += 1
                    #     self.set_face(custom=i)
                    #     time.sleep(2)
                    #     if counter == 2:
                    #         break

                    

                    ##### NECK: Reset the neck for standart position
                    self.set_neck(position=[0.0, 0.0], wait_for_end_of=True)             
                    # next state
                    self.state = self.Approach_customer1_table
                
                print(self.counter)
                self.counter +=1
                
            elif self.state == self.Approach_customer1_table:
                print("State:", self.state, "- Approach_customer1_table")

                ##### NECK MOVEMENT FORWARD POSITION
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                ##### BACK TO STANDART FACE
                self.set_face("charmie_face")
                
                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                # ATIVAR NAVEGAÇAO POR OBSTCLES

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=True)

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ##### MOVE TO CUSTOMER 1
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                ##### NECK: Look to customer 1
                self.set_neck_coords(position=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], wait_for_end_of=True)
                time.sleep(3)
                ##### NECK MOVEMENT FORWARD POSITION
                # self.set_neck(position=self.look_forward, wait_for_end_of=True)

                ##### SPEAK: Hello
                # self.set_speech(filename="restaurant/hello_customer", wait_for_end_of=True)
                

                self.state = self.Receiving_order1_listen_and_confirm

            elif self.state == self.Receiving_order1_listen_and_confirm:
                print("State:", self.state, "- Receiving_order1_listen_and_confirm")
                order_collected_customer_1 = False
                
                # CASO SEJA NECESSÁRIO ACRESCENTAR SEARCH FOR PERSON:
                
                # tetas = [[0, 0]]
                # customers_not_found = True
                # customers_list = []

            
                # customers_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True)
                # dist = 100000.0

                # print("FOUND:", len(customers_found)) 
                # client = DetectedPerson()
                # for p in customers_found:
                #     dist_client = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                #     if dist_client < dist:
                #         dist = dist_client
                #         client = p
                #         print('\n \n coordenadas cliente :', client.position_relative.x, client.position_relative.y)
                #         print('distance to client: ', dist)
                    
                # print('Nr of detected customers waving: ', len(customers_list))
                
                # self.set_neck_coords(position=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], wait_for_end_of=True)
                
                
                
                
                
                
                while not order_collected_customer_1:

                    self.set_rgb(command=BLUE+ALTERNATE_QUARTERS)

                    # old: self.set_speech(filename="restaurant/green_face", wait_for_end_of=True)
                    self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    ##### AUDIO: Listen the order and repeat for confirmation
                    command = self.get_audio(restaurant=True, question="restaurant/what_is_your_order", face_hearing="charmie_face_green_my_order", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list = command.split(" ")
                    self.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                    for kw in keyword_list:
                        print(kw)
                        self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)

                    ##### AUDIO: Listen "YES" OR "NO"
                    ##### "Please say yes or no to confirm the order"
                    confirmation = self.get_audio(yes_or_no=True, question="restaurant/yes_no_question", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                        self.set_rgb(command=GREEN+BLINK_LONG)

                        self.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                        for kw in keyword_list:
                            print(kw)
                            self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                        ##### SPEAK: Thank you
                        # self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
                        order_collected_customer_1 = True  # Sai do loop se a confirmação for "yes"
                    elif confirmation.lower() == "no":
                        self.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry, TRY AGAIN
                        self.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                    else:
                        self.set_rgb(command=YELLOW+BLINK_LONG)
                        ##### ERROR
                        print("ERROR")

                # next state
                """ self.state = self.Detecting_customer2_again

            elif self.state == self.Detecting_customer2_again:
                print("State:", self.state, "- Detecting_customer2_again")


                ##### SPEAK: Could the second customer please raise their hand again?
                self.set_speech(filename="restaurant/raise_hand_again", wait_for_end_of=True)

                ##### SEARCH FOR PERSON AGAIN!!!!
                tetas = [-120, -60, 0, 60, 120]
                ##### YOLO POSE SEARCH FOR ONLY_DETECT_PERSON_ARM_RAISED
                coords_of_people, images_of_people = self.search_for_person_2(tetas)

                if len(coords_of_people) >= 1:
                    self.x2, self.y2 = coords_of_people[0]

                ##### NECK: Look to customer 2
                self.set_neck(position=[self.x2, self.y2], wait_for_end_of=True) """

                # next state

                print(len(sorted_persons))
                if len(sorted_persons) == 1:
                    self.just_one_waving = True
                    self.state = self.Collect_orders_from_barman
                    
                
                else:
                    self.state = self.Approach_customer2_table

            elif self.state == self.Approach_customer2_table:
                print("State:", self.state, "- Approach_customer2_table")

                ##### NECK MOVEMENT FORWARD POSITION
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                ##### BACK TO STANDART FACE
                self.set_face("charmie_face")
                
                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                ##### MOVE TO CUSTOMER 2

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)

                ##### NECK: Look to customer 2
                self.set_neck_coords(position=[sorted_persons[1].position_absolute.x, sorted_persons[1].position_absolute.y], wait_for_end_of=True)
                ##### NECK MOVEMENT FORWARD POSITION
                time.sleep(3)
                # self.set_neck(position=self.look_forward, wait_for_end_of=True)

                ##### SPEAK: Hello
                # self.set_speech(filename="restaurant/hello_customer", wait_for_end_of=True)
                self.state = self.Receiving_order2_listen_and_confirm

            elif self.state == self.Receiving_order2_listen_and_confirm:
                print("State:", self.state, "- Receiving_order2_listen_and_confirm")
                order_collected_customer_2 = False
                while not order_collected_customer_2:

                    self.set_rgb(command=BLUE+ALTERNATE_QUARTERS)

                    # old: self.set_speech(filename="restaurant/green_face", wait_for_end_of=True)
                    self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    ##### AUDIO: Listen the order and repeat for confirmation
                    command = self.get_audio(restaurant=True, question="restaurant/what_is_your_order", face_hearing="charmie_face_green_my_order", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list = command.split(" ")
                    self.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                    for kw in keyword_list:
                        print(kw)
                        self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)

                    ##### AUDIO: Listen "YES" OR "NO"
                    ##### "Please say yes or no to confirm the order"
                    confirmation = self.get_audio(yes_or_no=True, question="restaurant/yes_no_question", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                        self.set_rgb(command=GREEN+BLINK_LONG)

                        self.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                        for kw in keyword_list:
                            print(kw)
                            self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                        ##### SPEAK: Thank you
                        # self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
                        order_collected_customer_2 = True  # Sai do loop se a confirmação for "yes"
                    elif confirmation.lower() == "no":
                        self.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry, TRY AGAIN
                        self.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                    else:
                        self.set_rgb(command=YELLOW+BLINK_LONG)
                        ##### ERROR
                        print("ERROR")

                # next state
                self.state = self.Collect_orders_from_barman

            elif self.state == self.Collect_orders_from_barman:
                print("State:", self.state, "- Collect_orders_from_barman")
                
                ##### SPEAK: Start moving
                self.set_speech(filename="restaurant/move_barman", wait_for_end_of=True)
                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ##### MOVE TO THE BARMAN TABLE
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=False, wait_for_end_of=True, reached_radius=1.5)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)


                self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], wait_for_end_of=True)

                ##### SPEAK: Say the order to barman
                self.set_speech(filename="restaurant/say_order_to_barman", wait_for_end_of=True)

                ##### SPEAK: Say order elements
                ##### Itera sobre cada elemento do pedido
                for pedido in self.all_orders:
                    for elemento in pedido:
                        # Define o nome do arquivo correspondente ao elemento
                        filename = "objects_names/" + elemento.lower().replace(" ", "_")  # Supondo que os arquivos estejam na pasta "restaurant" e tenham o mesmo nome que os elementos em minúsculas
                        
                        # SPEAK: Diz o elemento do pedido
                        self.set_speech(filename=filename, wait_for_end_of=True)

                ##### SPEAK: Say to the barman put the order in the tray
                self.set_speech(filename="restaurant/tray_order", wait_for_end_of=True)

                time.sleep(20)

                # CONFIRMAR SE O BARMAN JÁ COLOCOU PEDIDO NO TABULEIRO??

                # next state
                self.state = self.Delivering_customer_order1

            elif self.state == self.Delivering_customer_order1:
                print("State:", self.state, "- Delivering_customer_order1")

                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                ##### NAVIGATION TO FIRST CUSTOMER
                ##### MOVE TO CUSTOMER 1

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_neck_coords(position=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], wait_for_end_of=True)
                
                ##### SPEAK: Here are the items, CAN YOU PICK FROM MY TRAY?
                self.set_speech(filename="restaurant/pick_tray", wait_for_end_of=True)

                time.sleep(10)

                ##### SPEAK: Hope you enjoy!
                self.set_speech(filename="restaurant/enjoy_order", wait_for_end_of=True)

                if self.just_one_waving == True:
                    
                    self.state = self.temp_state
                    
                else:               
                    # next state
                    self.state = self.Delivering_customer_order2

            elif self.state == self.Delivering_customer_order2:
                print("State:", self.state, "- Delivering_customer_order2")

                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                ##### NAVIGATION TO SECOND CUSTOMER
                ##### MOVE TO CUSTOMER 2
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[1].position_relative.x, sorted_persons[1].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                
                # self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_neck_coords(position=[sorted_persons[1].position_absolute.x, sorted_persons[1].position_absolute.y], wait_for_end_of=True)
                

                ##### SPEAK: Here are the items, CAN YOU PICK FROM MY TRAY?
                self.set_speech(filename="restaurant/pick_tray", wait_for_end_of=True)
                
                time.sleep(10)
                ##### SPEAK: Hope you enjoy!
                self.set_speech(filename="restaurant/enjoy_order", wait_for_end_of=True)
                                
                # next state
                self.state = self.Final_State
                
            elif self.state == self.temp_state:
                # JUST ONE CLIENT WAVING

                ##### MOVE TO THE BARMAN TABLE
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=False, wait_for_end_of=True, reached_radius=2.0)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True) # Virar de frente para pessoas de novo
                self.set_rgb(command=GREEN+BLINK_LONG)

                tetas = [[-45, 0], [0, 0], [45, 0]]
                customers_not_found = True
                customers_list = []
                
                print(len(customers_list))
                
                while len(customers_list) == 0:
                    customers_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True)

                    print("FOUND:", len(customers_found)) 
                    for p in customers_found:
                        self.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                        print("ID:", p.index_person)
                        print('Customer position', p.position_relative)
                        customers_list.append(p)
                
                    
                    print('Nr of detected customers waving: ', len(customers_list))

                self.set_neck(position=[0.0, 0.0], wait_for_end_of=True)


                ##### SPEAK: Found waving customers
                self.set_speech(filename="restaurant/found_customer", wait_for_end_of=True)
                dist_array = []
                ##### NECK: look waving customers

                for p in customers_list:
                    self.set_neck_coords(position=[p.position_relative.x, p.position_relative.y], wait_for_end_of=True)
                    dist = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                    dist_array.append(dist)
                    print('\n \n coordenadas cliente :', p.position_relative.x, p.position_relative.y)
                    print('distance to client: ', dist)
                    
                    time.sleep(2)


                # Step 1: Create a list of tuples (distance, index)
                distance_with_indices = [(distance, index) for index, distance in enumerate(dist_array)]

                # Step 2: Sort the list of tuples by distance
                sorted_distances = sorted(distance_with_indices)

                # Step 3: Use the sorted indices to reorder the persons list
                sorted_persons = []
                sorted_persons = [customers_list[index] for _, index in sorted_distances]

                # Print the results
                print("Sorted Distances with Indices:", sorted_distances)
                
                for p in sorted_persons:
                    print("Positions:", p.position_relative, ' + ',p.index_person)


                self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], wait_for_end_of=True)
                ##### SPEAK: Check face to see customers detected
                self.set_speech(filename="restaurant/face_customer", wait_for_end_of=True)

                i = 0
                person_count = 2
                for p in sorted_persons :
                    if i == person_count:
                        break
                    path_customer = self.detected_person_to_face_path(person=sorted_persons[i], send_to_face=True)
                    print(f'sorted person {i} position {sorted_persons[i].position_relative.x}, {sorted_persons[i].position_relative.y}')
            
            
                    time.sleep(3)
                    i += 1



                # ACABAR LÓGICA AQUI. TENHO DE GARANTIR QUE VOU PARA O SEGUNDO CLIENTE E CRIAR ESSE ESTADO E DEPOIS ESTADO FINAL

                
                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                ##### NAVIGATION TO SECOND CUSTOMER
                ##### MOVE TO CUSTOMER 2
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], reached_radius=2.2, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                # self.set_neck_coords(position=[sorted_persons[1].position_absolute.x, sorted_persons[1].position_absolute.y], wait_for_end_of=True)
                
                # IR BUSCAR PEDIDO E VOLTAR BARMAN

                ##### NECK MOVEMENT FORWARD POSITION
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                ##### BACK TO STANDART FACE
                self.set_face("charmie_face")
                
                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                # ATIVAR NAVEGAÇAO POR OBSTCLES

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=True)

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ##### MOVE TO CUSTOMER 1
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)
                ##### NECK: Look to customer 1
                self.set_neck_coords(position=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], wait_for_end_of=True)
                time.sleep(3)

                order_collected_customer_1 = False
                
                while not order_collected_customer_1:

                    self.set_rgb(command=BLUE+ALTERNATE_QUARTERS)

                    # old: self.set_speech(filename="restaurant/green_face", wait_for_end_of=True)
                    self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    ##### AUDIO: Listen the order and repeat for confirmation
                    command = self.get_audio(restaurant=True, question="restaurant/what_is_your_order", face_hearing="charmie_face_green_my_order", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list = command.split(" ")
                    self.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                    for kw in keyword_list:
                        print(kw)
                        self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)

                    ##### AUDIO: Listen "YES" OR "NO"
                    ##### "Please say yes or no to confirm the order"
                    confirmation = self.get_audio(yes_or_no=True, question="restaurant/yes_no_question", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    self.all_orders = []

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                        self.set_rgb(command=GREEN+BLINK_LONG)

                        self.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                        for kw in keyword_list:
                            print(kw)
                            self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                        ##### SPEAK: Thank you
                        # self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
                        order_collected_customer_1 = True  # Sai do loop se a confirmação for "yes"
                    elif confirmation.lower() == "no":
                        self.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry, TRY AGAIN
                        self.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                    else:
                        self.set_rgb(command=YELLOW+BLINK_LONG)
                        ##### ERROR
                        print("ERROR")

                ##### SPEAK: Start moving
                self.set_speech(filename="restaurant/move_barman", wait_for_end_of=True)
                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ##### MOVE TO THE BARMAN TABLE
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=False, wait_for_end_of=True, reached_radius=1.5)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_LONG)


                self.set_neck_coords(position=[self.barman_position.position_relative.x, self.barman_position.position_relative.y], wait_for_end_of=True)

                ##### SPEAK: Say the order to barman
                self.set_speech(filename="restaurant/say_order_to_barman", wait_for_end_of=True)

                ##### SPEAK: Say order elements
                ##### Itera sobre cada elemento do pedido
                for pedido in self.all_orders:
                    for elemento in pedido:
                        # Define o nome do arquivo correspondente ao elemento
                        filename = "objects_names/" + elemento.lower().replace(" ", "_")  # Supondo que os arquivos estejam na pasta "restaurant" e tenham o mesmo nome que os elementos em minúsculas
                        
                        # SPEAK: Diz o elemento do pedido
                        self.set_speech(filename=filename, wait_for_end_of=True)

                ##### SPEAK: Say to the barman put the order in the tray
                self.set_speech(filename="restaurant/tray_order", wait_for_end_of=True)

                time.sleep(20)






                ###### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                self.set_rgb(command=BLUE+ROTATE)

                ##### NAVIGATION TO FIRST CUSTOMER
                ##### MOVE TO CUSTOMER 1

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], reached_radius=1.5, flag_not_obs=False, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                self.set_navigation(movement="rotate", target=[sorted_persons[0].position_relative.x, sorted_persons[0].position_relative.y], flag_not_obs=True, wait_for_end_of=True)
                self.set_rgb(command=GREEN+BLINK_QUICK)
                
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_neck_coords(position=[sorted_persons[0].position_absolute.x, sorted_persons[0].position_absolute.y], wait_for_end_of=True)
                
                ##### SPEAK: Here are the items, CAN YOU PICK FROM MY TRAY?
                self.set_speech(filename="restaurant/pick_tray", wait_for_end_of=True)

                time.sleep(10)

                ##### SPEAK: Hope you enjoy!
                self.set_speech(filename="restaurant/enjoy_order", wait_for_end_of=True)

                                
                # next state
                self.state = self.Final_State

            elif self.state == self.Final_State:
                print("State:", self.state, "- Final_State")

                ##### SPEAK: FINISHING RESTAURANT TASK
                
                self.set_speech(filename="restaurant/finish_restaurant", wait_for_end_of=True)

                self.set_rgb(command=RAINBOW_ROT)

                # Lock after finishing task




                while True:
                    pass

            else:
                pass

