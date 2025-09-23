#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Point
from sensor_msgs.msg import Image
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, BoundingBox, BoundingBoxAndPoints, ListOfDetectedPerson, ListOfDetectedObject, Obstacles, ArmController
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, Trigger, SetFace, ActivateObstacles, GetPointCloudBB

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


class ServeBreakfastNode(Node):

    def __init__(self):
        super().__init__("ServeBreakfast")
        self.get_logger().info("Initialised CHARMIE ServeBreakfast Node")

        # path to save detected people in search for person
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        ### Topics (Publisher and Subscribers) ###   
        # Intel Realsense Subscribers
        # self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        # self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_head_image_callback, 10)
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered_hand', self.object_detected_filtered_hand_callback, 10)
        self.doors_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "doors_detected_filtered", self.doors_detected_filtered_callback, 10)
        self.doors_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'doors_detected_filtered_hand', self.doors_detected_filtered_hand_callback, 10)
        self.shoes_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "shoes_detected_filtered", self.shoes_detected_filtered_callback, 10)
        self.shoes_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'shoes_detected_filtered_hand', self.shoes_detected_filtered_hand_callback, 10)
        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        # Search for person and object 
        self.search_for_person_detections_publisher = self.create_publisher(ListOfDetectedPerson, "search_for_person_detections", 10)
        self.search_for_object_detections_publisher = self.create_publisher(ListOfDetectedObject, "search_for_object_detections", 10)
        # Obstacles
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obstacles_callback, 10)
        
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        self.save_speech_command_client = self.create_client(SaveSpeechCommand, "save_speech_command")
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
        # Yolo Pose
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # Yolo Objects
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(Trigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(Trigger, "nav_trigger")
        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloudBB, "get_point_cloud_bb")
        
        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # while not self.save_speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Save Speech Command...")
        # Audio
        # while not self.get_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Audio Server...")
        # while not self.calibrate_audio_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Calibrate Audio Server...")        
        # Face        
        while not self.face_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Face Command...")
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # Yolos
        while not self.activate_yolo_objects_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        # Obstacles
        while not self.activate_obstacles_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Activate Obstacles Command...")
        # Point Cloud
        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")
        
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_save_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_arm = False
        self.waited_for_end_of_face = False
        self.waiting_for_pcloud = False

        self.br = CvBridge()
        self.depth_head_img = Image()
        self.depth_hand_img = Image()
        self.first_depth_head_image_received = False
        self.first_depth_hand_image_received = False
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        self.flag_navigation_reached = False
        self.point_cloud_response = GetPointCloudBB.Response()
        self.obstacles = Obstacles()

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.save_speech_success = True
        self.save_speech_message = ""
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
        self.activate_obstacles_success = True
        self.activate_obstacles_message = ""

        self.get_neck_position = [1.0, 1.0]


    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)

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

    def get_aligned_depth_head_image_callback(self, img: Image):
        self.depth_head_img = img
        self.first_depth_head_image_received = True
        # print("Received Depth Image")

    def get_aligned_depth_hand_image_callback(self, img: Image):
        self.depth_hand_img = img
        self.first_depth_hand_image_received = True
        # print("Received HAND Depth Image")

    ### OBSTACLES
    def obstacles_callback(self, obs: Obstacles):
        self.obstacles = obs

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta

    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
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

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req, camera):
        request = GetPointCloudBB.Request()
        request.data = req
        request.retrieve_bbox = False
        request.camera = camera
    
        future = self.point_cloud_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud)

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_response = future.result()
            self.waiting_for_pcloud = False
            # print("Received Back")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

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


    #### SAVE SPEECH SERVER FUNCTIONS #####
    def call_save_speech_command_server(self, filename="", command="", quick_voice=False, play_command=False, show_in_face=False, wait_for_end_of=True):
        request = SaveSpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.play_command = play_command
        request.show_in_face = show_in_face
    
        future = self.save_speech_command_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_save_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"
    
    def callback_call_save_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.save_speech_success = response.success
            self.save_speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_save_speaking = True
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


def main(args=None):
    rclpy.init(args=args)
    node = ServeBreakfastNode()
    th_main = threading.Thread(target=ThreadMainServeBreakfast, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainServeBreakfast(node: ServeBreakfastNode):
    main = ServeBreakfastMain(node)
    main.main()

class ServeBreakfastMain():

    def __init__(self, node: ServeBreakfastNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message
    
    def save_speech(self, filename="", command="", quick_voice=False, play_command=False, show_in_face=False, wait_for_end_of=True):

        # the commands should be lists, because you can send a list of commands and a list of filenames,
        # making it possible to create multiple temp commands with one instruction
        # But if by mistake someone sends them as a string (beause set_speech is done that way) I correct it  
        file = []
        comm = [] 
        if isinstance(filename, str) and isinstance(command, str):
            file.append(filename)
            comm.append(command)
        elif isinstance(filename, list) and isinstance(command, list):
            file = filename
            comm = command
        
        if len(file) > 0 and len(comm) > 0:

            self.node.call_save_speech_command_server(filename=file, command=comm, quick_voice=quick_voice, play_command=play_command, show_in_face=show_in_face, wait_for_end_of=wait_for_end_of)
            
            if wait_for_end_of:
                while not self.node.waited_for_end_of_save_speaking:
                    pass
            self.node.waited_for_end_of_save_speaking = False

            return self.node.save_speech_success, self.node.save_speech_message

        else:

            self.node.get_logger().error("Could not generate save speech as as filename and command types are incompatible.")
            return False, "Could not generate save speech as as filename and command types are incompatible."

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
        
        wait_for_end_of=False
        
        self.node.call_face_command_server(command=command, custom=custom, wait_for_end_of=wait_for_end_of)
        
        if command != "" and command != "charmie_face" and command != "charmie_face_green_yes_no":
            home = str(Path.home())
            midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_media_faces"
            complete_path = home+'/'+midpath+'/'
            image = cv2.imread(complete_path + command + ".jpg")
            cv2.imshow(command, image) 
            cv2.waitKey(200)
        elif custom != "":
            home = str(Path.home())
            midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
            complete_path = home+'/'+midpath+'/'
            image = cv2.imread(complete_path + custom + ".jpg")
            cv2.imshow(custom, image) 
            cv2.waitKey(200)
        
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

                        if temp_people.index == people.index:
                            is_already_in_list = True
                            person_already_in_list = people

                    if is_already_in_list:
                        person_detected.remove(person_already_in_list)
                    elif temp_people.index > 0: # debug
                        # print("added_first_time", temp_people.index, temp_people.position_absolute.x, temp_people.position_absolute.y)
                        self.set_rgb(GREEN+SET_COLOUR)
                    
                    if temp_people.index > 0:
                        person_detected.append(temp_people)
                        people_ctr+=1

            # DEBUG
            # print("people in this neck pos:")
            # for people in person_detected:
            #     print(people.index, people.position_absolute.x, people.position_absolute.y)
        
            total_person_detected.append(person_detected.copy())
            # print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()          

        self.activate_yolo_pose(activate=False)
        # print(total_person_detected)

        # DEBUG
        # print("TOTAL people in this neck pos:")
        # for frame in total_person_detected:
        #     for people in frame:    
        #         print(people.index, people.position_absolute.x, people.position_absolute.y)
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
                        # print("new:", total_person_detected[frame][person].index, "old:", filtered_persons[filtered].index, dist)
                        
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
                    # print("REMOVED: ", p.index)
                    filtered_persons.remove(p)
                # else:
                    # print("TRIED TO REMOVE TWICE THE SAME PERSON")
            to_remove.clear()  

            for p in to_append:
                # print("ADDED: ", p.index)
                filtered_persons.append(p)
            to_append.clear()
            
        self.set_neck(position=[0, 0], wait_for_end_of=False)
        self.set_rgb(BLUE+HALF_ROTATE)

        sfp_pub = ListOfDetectedPerson()
        # print("FILTERED:")
        for p in filtered_persons:
            sfp_pub.persons.append(p)
        #     print(p.index)
        self.node.search_for_person_detections_publisher.publish(sfp_pub)

        return filtered_persons

    def detected_person_to_face_path(self, person, send_to_face):

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        
        cf = self.node.br.imgmsg_to_cv2(person.image_rgb_frame, "bgr8")
        just_person_image = cf[person.box_top_left_y:person.box_top_left_y+person.box_height, person.box_top_left_x:person.box_top_left_x+person.box_width]
        # cv2.imshow("Search for Person", just_person_image)
        # cv2.waitKey(100)
        
        face_path = current_datetime + str(person.index)
        
        cv2.imwrite(self.node.complete_path_custom_face + face_path + ".jpg", just_person_image) 
        time.sleep(0.5)

        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path

    def search_for_objects(self, tetas, delta_t=3.0, list_of_objects = [], list_of_objects_detected_as = [], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False):

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

            self.activate_yolo_objects(activate_objects=detect_objects, activate_shoes=detect_shoes, activate_doors=detect_furniture,
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

                        
                    if detect_furniture: 
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
                print(o.index, o.object_name, o.index, "\t", round(o.position_absolute.x, 2), round(o.position_absolute.y, 2), round(o.position_absolute.z, 2))


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

    def get_point_cloud(self, bb=BoundingBox(), wait_for_end_of=True):

        requested_objects = []
            
        # bb = BoundingBox()
        # bb.box_top_left_x = 0
        # bb.box_top_left_y = 0
        # bb.box_width = 1280
        # bb.box_height = 720

        get_pc = BoundingBoxAndPoints()
        get_pc.bbox = bb

        requested_objects.append(get_pc)

        self.node.waiting_for_pcloud = True
        self.node.call_point_cloud_server(requested_objects, "hand")

        if wait_for_end_of:
            while self.node.waiting_for_pcloud:
                pass

        return self.node.point_cloud_response.coords[0]

    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message
    

    def main(self):
        
        # Task Related Variables
        self.Waiting_for_task_start = 0
        self.Approach_milk_location = 1
        self.Detect_and_pick_milk = 2
        self.Approach_cornflakes_location = 3
        self.Detect_and_pick_cornflakes = 4
        self.Approach_dishes_location = 5
        self.Detect_and_pick_dishes = 6
        self.Approach_kitchen_table = 7
        self.Placing_bowl = 8
        self.Placing_milk = 9
        self.Placing_cornflakes = 10
        self.Placing_spoon = 11
        self.Final_State = 12
    
        # Configurables
        self.ATTEMPTS_AT_RECEIVING = 2
        self.SHOW_OBJECT_DETECTED_WAIT_TIME = 3.0
        self.MAX_SPEED = 30
        self.TABLE_APPROACH_OBSTACLES = 0.35
        self.COUNTER_APPROACH_OBSTACLES = 0.25
        self.GET_MILK = False
        self.GET_CORNFLAKES = True
        self.GET_DISHES = True
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]] # , [-45-10, -45-5], [-45+10, -45-5]]

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # Navigation Positions
        self.front_of_door = [0.0, 1.5] 
        self.cofee_table = [-0.4, 3.5]
        self.side_table = [-0.4, 4.5]
        self.kitchen_counter = [-0.4, 5.5]
        self.kitchen_table = [-2.0, 6.8]
        
        ### ROBOCUP24
        self.MAX_SPEED = 40

        self.pre_room_door = [0.45, 3.55]
        self.post_room_door = [0.45, 4.70]
        self.front_of_start_door = [0.0, 1.0]
        self.front_sofa = [0.45, 5.8]
        self.midway_living_room = [-0.9, 8.5]
        self.close_to_garbage_bin = [-2.76, 5.9]
        self.close_to_dishwasher = [-2.26, 8.0]
        self.close_to_table_sb = [-4.26, 8.8]
        self.pre_table = [-4.76, 6.0]

        self.state = self.Waiting_for_task_start


        self.node.get_logger().info("IN SERVE THE BREAKFAST MAIN")

        while True:

            if self.state == self.Waiting_for_task_start:

                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")
                print("GET_MILK:", self.GET_MILK, "GET_CORNFLAKES:", self.GET_CORNFLAKES, "GET_DISHES:", self.GET_DISHES)

                time.sleep(1)
        
                self.activate_yolo_objects(activate_objects=False)

                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)

                self.set_face("charmie_face")

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)

                self.wait_for_start_button()
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)

                self.wait_for_door_start()

                self.state = self.Approach_cornflakes_location


            elif self.state == self.Approach_milk_location:
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    
                self.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    
                if self.GET_MILK:
                    self.set_speech(filename="generic/moving_drinks_location", wait_for_end_of=False)

                    self.set_navigation(movement="rotate", target=self.cofee_table, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.cofee_table, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    self.set_speech(filename="generic/arrived_drinks_location", wait_for_end_of=True)
                
                self.state = self.Detect_and_pick_milk


            elif self.state == self.Detect_and_pick_milk:

                if self.GET_MILK:
                    object_in_gripper = False
                    
                    while not object_in_gripper:

                        objects_found = self.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        
                        self.detected_object_to_face_path(object=objects_found[0], send_to_face=True, bb_color=(0,255,0))

                        self.set_neck(position=self.look_judge, wait_for_end_of=False)

                        self.set_speech(filename="serve_breakfast/found_the_milk", wait_for_end_of=False)  
                        
                        self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  

                        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)

                        time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)

                        self.set_arm(command="open_gripper", wait_for_end_of=False)

                        self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                        self.set_face("help_pick_milk") 
                        
                        object_in_gripper = False
                        gripper_ctr = 0
                        while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                            
                            gripper_ctr += 1
                            
                            self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                            object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                            
                            if not object_in_gripper:
                        
                                if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:

                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                                
                                self.set_arm(command="open_gripper", wait_for_end_of=False)

                        if not object_in_gripper and gripper_ctr >= self.ATTEMPTS_AT_RECEIVING:

                            self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                            self.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)
                                
                    self.set_face("charmie_face")

                    self.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                    
                    self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                self.state = self.Approach_cornflakes_location


            elif self.state == self.Approach_cornflakes_location:

                if self.GET_CORNFLAKES:
                    
                    self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.set_speech(filename="generic/sb_moving_kitchen_counter", wait_for_end_of=False)

                    
                    self.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_rgb(BLUE+ROTATE)
                    # self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.pre_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_rgb(MAGENTA+ROTATE)
                    self.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.post_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                    self.set_rgb(BLUE+ROTATE)
                    self.set_navigation(movement="rotate", target=self.front_sofa, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.front_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_rgb(MAGENTA+ROTATE)
                    self.set_navigation(movement="rotate", target=self.midway_living_room, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.midway_living_room, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_rgb(BLUE+ROTATE)

                    self.set_navigation(movement="rotate", target=self.close_to_table_sb, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.close_to_table_sb, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.set_rgb(BLUE+ROTATE)
                    
                    # debug
                    # self.set_initial_position([0.0, 0.0, 90.0])

                    self.set_speech(filename="generic/sb_arrived_kitchen_counter", wait_for_end_of=False)
                    self.set_navigation(movement="orientate", absolute_angle= 45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    
                self.state = self.Detect_and_pick_cornflakes


            elif self.state == self.Detect_and_pick_cornflakes:

                if self.GET_CORNFLAKES:
                    object_in_gripper = False
                    
                    while not object_in_gripper:

                        objects_found = self.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Cornflakes"], list_of_objects_detected_as=[["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                    
                        self.detected_object_to_face_path(object=objects_found[0], send_to_face=True, bb_color=(0,255,0))

                        self.set_neck(position=self.look_judge, wait_for_end_of=False)

                        self.set_speech(filename="serve_breakfast/found_the_cornflakes", wait_for_end_of=False)  
                        
                        self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  

                        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)

                        time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)

                        self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                        self.set_face("help_pick_cornflakes") 

                        object_in_gripper = False
                        gripper_ctr = 0
                        while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                            
                            gripper_ctr += 1
                            
                            self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                            object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                            
                            if not object_in_gripper:
                        
                                if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:

                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                                
                                self.set_arm(command="open_gripper", wait_for_end_of=False)

                        if not object_in_gripper and gripper_ctr >= self.ATTEMPTS_AT_RECEIVING:

                            self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                            self.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)
                                
                    self.set_face("charmie_face")
                        
                    self.set_arm(command="collect_cornflakes_to_tray_alternative_robocup_cornflakes", wait_for_end_of=True)
                    
                    self.set_arm(command="ask_for_objects_to_initial_position_alternative_robocup_cornflakes", wait_for_end_of=True)

                self.state = self.Detect_and_pick_dishes


            elif self.state == self.Approach_dishes_location:

                if self.GET_DISHES:
                    self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                    self.set_speech(filename="generic/moving_dishes_location", wait_for_end_of=False)
                    
                    self.set_navigation(movement="rotate", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                    self.set_navigation(movement="move", target=self.kitchen_counter, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    
                    self.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    self.set_speech(filename="generic/arrived_dishes_location", wait_for_end_of=True)
                
                self.state = self.Detect_and_pick_dishes


            elif self.state == self.Detect_and_pick_dishes:

                if self.GET_DISHES:
                    object_in_gripper = False
                    correct_object_bowl = DetectedObject()
                    correct_object_spoon = DetectedObject()
                    while not object_in_gripper:

                        objects_found = self.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Spoon", "Bowl"], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        # objects_found = self.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Spoon", "Bowl"], list_of_objects_detected_as=[["Fork", "Knife"],["Plate"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                    
                        for of in objects_found:
                            print(of.object_name.lower(), of.index)
                            if of.object_name.lower() == "bowl":
                                correct_object_bowl = of
                            elif of.object_name.lower() == "spoon":
                                correct_object_spoon = of

                        print("correct_bowl:", correct_object_bowl.object_name, correct_object_bowl.index)
                        print("correct_spoon:", correct_object_spoon.object_name, correct_object_spoon.index)

                        # BOWL
                        self.detected_object_to_face_path(object=correct_object_bowl, send_to_face=True, bb_color=(0,255,0))

                        self.set_neck(position=self.look_judge, wait_for_end_of=False)

                        self.set_speech(filename="serve_breakfast/found_the_bowl", wait_for_end_of=False)  
                        
                        self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  

                        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)

                        time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)

                        self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                        self.set_face("help_pick_bowl") 
                        
                        object_in_gripper = False
                        gripper_ctr = 0
                        while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                            
                            gripper_ctr += 1
                            
                            self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                            object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                            
                            if not object_in_gripper:
                        
                                if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:

                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                                
                                self.set_arm(command="open_gripper", wait_for_end_of=False)

                        if not object_in_gripper and gripper_ctr >= self.ATTEMPTS_AT_RECEIVING:

                            self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                            self.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)
                                
                    self.set_arm(command="ask_for_objects_to_initial_position_alternative_robocup_cornflakes", wait_for_end_of=False)

                    # SPOON
                    self.detected_object_to_face_path(object=correct_object_spoon, send_to_face=True, bb_color=(0,255,0))

                    self.set_neck(position=self.look_judge, wait_for_end_of=False)

                    self.set_speech(filename="serve_breakfast/found_the_spoon", wait_for_end_of=False)  
                    
                    self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  

                    time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)

                    self.set_face(command="place_spoon_in_tray")

                    self.set_speech(filename="serve_breakfast/place_object_in_funilocopo", wait_for_end_of=True)

                    time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)

                    self.set_face("charmie_face")

                self.state = self.Approach_kitchen_table
                
                # debug
                # self.state = self.Placing_bowl


            elif self.state == self.Approach_kitchen_table:

                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="serve_breakfast/sb_moving_kitchen_table", wait_for_end_of=False)

                self.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                self.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=self.COUNTER_APPROACH_OBSTACLES, wait_for_end_of=True)
                    
                # self.set_navigation(movement="rotate", target=self.kitchen_table, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.kitchen_table, max_speed=20.0, reached_radius=1.0, flag_not_obs=False, wait_for_end_of=True)

                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)
                                
                self.set_navigation(movement="orientate", absolute_angle= 180.0, flag_not_obs=True, wait_for_end_of=True)

                # self.set_navigation(movement="adjust_angle", absolute_angle= 0.0, flag_not_obs=True, wait_for_end_of=True)
                time.sleep(3)
                self.set_speech(filename="serve_breakfast/remove_decorations_table", wait_for_end_of=False)

                self.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=self.TABLE_APPROACH_OBSTACLES, wait_for_end_of=True)
                
                self.set_navigation(movement="orientate", absolute_angle= 225.0, flag_not_obs=True, wait_for_end_of=True)
                
                self.set_speech(filename="serve_breakfast/sb_arrived_kitchen_table", wait_for_end_of=True)
                
                self.set_neck(position=self.look_table_objects, wait_for_end_of=False)
                
                self.state = self.Placing_bowl


            elif self.state == self.Placing_bowl:

                if self.GET_DISHES:
                    self.set_arm(command="place_bowl_table", wait_for_end_of=True)
                    self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                self.state = self.Placing_cornflakes 
            

            elif self.state == self.Placing_cornflakes:

                if self.GET_CORNFLAKES:
                    ##### ARM POUR IN BOWL
                    self.set_arm(command="pour_cereals_bowl_alternative_robocup_cornflakes", wait_for_end_of=True)
                    self.set_speech(filename="serve_breakfast/cornflakes_poured", wait_for_end_of=False)
                    
                    ##### ARM PLACE OBJECT
                    self.set_arm(command="place_cereal_table_alternative_robocup_cornflakes", wait_for_end_of=True)
                    self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)
                
                self.state = self.Placing_milk

           
            elif self.state == self.Placing_milk:

                if self.GET_MILK:
                    ##### ARM POUR IN BOWL
                    self.set_arm(command="pour_milk_bowl", wait_for_end_of=True)
                    self.set_speech(filename="serve_breakfast/milk_poured", wait_for_end_of=False)

                    ##### ARM PLACE OBJECT
                    self.set_arm(command="place_milk_table", wait_for_end_of=True)
                    self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                self.state = self.Placing_spoon


            elif self.state == self.Placing_spoon:

                if self.GET_DISHES:
                    self.set_arm(command="place_spoon_table", wait_for_end_of=True)
                    self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                self.state = self.Final_State 


            elif self.state == self.Final_State:
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=False)
                self.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=False)

                while True:
                    pass

            else:
                pass
            