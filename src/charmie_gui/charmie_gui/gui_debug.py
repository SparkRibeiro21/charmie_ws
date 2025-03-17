#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from example_interfaces.msg import Bool, String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Point
from sensor_msgs.msg import Image, LaserScan
from xarm_msgs.srv import MoveCartesian
from nav2_msgs.action import NavigateToPose
from charmie_interfaces.msg import NeckPosition, ListOfPoints, TarNavSDNL, ListOfDetectedObject, ListOfDetectedPerson, PS4Controller, DetectedPerson, DetectedObject, \
    TrackingMask
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, \
    TrackPerson, ActivateYoloPose, ActivateYoloObjects, Trigger, SetFace, ActivateObstacles, GetPointCloudBB, SetAcceleration, NodesUsed, GetVCCs, GetLLMGPSR, \
    GetLLMDemo, ActivateTracking
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import math
import threading
from pathlib import Path
import json
import os
import time
from datetime import datetime

import pygame_widgets
import pygame
from pygame_widgets.toggle import Toggle
from pygame_widgets.button import Button
from pygame_widgets.textbox import TextBox

class DebugVisualNode(Node):

    def __init__(self):
        super().__init__("Robot")
        self.get_logger().info("Initialised CHARMIE Debug Visual Node")

        ### Topics ###

        # Audio: Help in Debug
        self.audio_interpreted_subscriber = self.create_subscription(String, "audio_interpreted", self.audio_interpreted_callback, 10)
        self.audio_final_subscriber = self.create_subscription(String, "audio_final", self.audio_final_callback, 10)

        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)
        
        # get neck position
        self.get_neck_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)
        
        # lidar
        self.lidar_subscriber = self.create_subscription(LaserScan, "scan", self.lidar_callback , 10)
        self.lidar_bottom_subscriber = self.create_subscription(LaserScan, "scan_bottom", self.lidar_bottom_callback , 10)

        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_pose_callback, 10)

        # Navigation
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  

        # search for person and object 
        self.search_for_person_subscriber = self.create_subscription(ListOfDetectedPerson, "search_for_person_detections", self.search_for_person_detections_callback, 10)
        self.search_for_object_subscriber = self.create_subscription(ListOfDetectedObject, "search_for_object_detections", self.search_for_object_detections_callback, 10)
        
        # IMU
        self.get_orientation_subscriber = self.create_subscription(Float32, "get_orientation", self.get_orientation_callback, 10)
       
        # Camera Obstacles
        self.temp_camera_obstacles_subscriber = self.create_subscription(ListOfPoints, "camera_head_obstacles", self.get_camera_obstacles_callback, 10)

        # Obstacles
        self.final_obstacles_subscriber = self.create_subscription(ListOfPoints, "final_obstacles", self.get_final_obstacles_callback, 10)

        # PS4 Controller
        self.controller_subscriber = self.create_subscription(PS4Controller, "controller_state", self.ps4_controller_callback, 10)

        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)

        # Yolo Objects
        self.objects_filtered_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered', self.object_detected_filtered_callback, 10)
        self.objects_filtered_hand_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered_hand', self.object_detected_filtered_hand_callback, 10)

        # Tracking (SAM2)
        self.tracking_mask_subscriber = self.create_subscription(TrackingMask, 'tracking_mask', self.tracking_mask_callback, 10)
        
        ### Services (Clients) ###
		# Arm (Ufactory)
        self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
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
        # Low level
        self.set_acceleration_ramp_client = self.create_client(SetAcceleration, "set_acceleration_ramp")
        self.get_vccs_client = self.create_client(GetVCCs, "get_vccs")
        # LLM
        self.llm_demonstration_client = self.create_client(GetLLMDemo, "llm_demonstration")
        self.llm_gpsr_client = self.create_client(GetLLMGPSR, "llm_gpsr")
        # Tracking (SAM2)
        self.activate_tracking_client = self.create_client(ActivateTracking, "activate_tracking")

        ### Actions (Clients) ###
        self.nav2_client_ = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.nodes_used_server = self.create_service(NodesUsed, "nodes_used_gui", self.nodes_used_callback)

        self.create_timer(1.0, self.check_yolos_timer)
        self.create_timer(0.4, self.check_tracking_timer)
        self.is_yolo_pose_comm = False
        self.is_yolo_obj_head_comm = False
        self.is_yolo_obj_hand_comm = False

        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.activate_obstacles_success = True
        self.activate_obstacles_message = ""

        self.battery_voltage = 0.0
        self.emergency_stop = False
        self.waited_for_end_of_get_vccs = False
        self.battery_timer_ctr = 0
        self.previous_is_low_level_on = False
        self.battery_timer()
        self.create_timer(1.0, self.battery_timer)
        
        self.head_rgb = Image()
        self.hand_rgb = Image()
        self.head_depth = Image()
        self.hand_depth = Image()
        self.new_head_rgb = False
        self.new_hand_rgb = False
        self.new_head_depth = False
        self.new_hand_depth = False

        self.detected_people = ListOfDetectedPerson()
        self.detected_objects = ListOfDetectedObject()
        self.detected_objects_hand = ListOfDetectedObject()
        self.new_detected_people = False
        self.new_detected_objects = False
        self.new_detected_objects_hand = False

        self.robot_pose = Pose2D()

        self.lidar_time = 0.0
        self.lidar_bottom_time = 0.0
        self.odometry_time = 0.0
        self.ps4_controller_time = 0.0
        self.localisation_time = 0.0
        self.amcl_time = 0.0

        self.head_camera_time = 0.0
        self.head_depth_camera_time = 0.0
        self.hand_camera_time = 0.0
        self.hand_depth_camera_time = 0.0

        self.last_head_camera_time = 0.0
        self.last_head_depth_camera_time = 0.0
        self.last_hand_camera_time = 0.0
        self.last_hand_depth_camera_time = 0.0

        self.head_rgb_fps = 0.0
        self.head_depth_fps = 0.0
        self.hand_rgb_fps = 0.0
        self.hand_depth_fps = 0.0

        self.head_yp_time = 0.0
        self.head_yo_time = 0.0
        self.hand_yo_time = 0.0
        self.track_time = 0.0

        self.last_head_yp_time = 0.0
        self.last_head_yo_time = 0.0
        self.last_hand_yo_time = 0.0
        self.last_track_time = 0.0

        self.head_yp_fps = 0.0
        self.head_yo_fps = 0.0
        self.hand_yo_fps = 0.0
        self.track_fps = 0.0

        self.all_pos_x_val = []
        self.all_pos_y_val = []

        self.nodes_used = NodesUsed.Request()
        self.scan = LaserScan()
        self.search_for_person = ListOfDetectedPerson()
        self.search_for_object = ListOfDetectedObject()
        self.new_search_for_person = False
        self.new_search_for_object = False
        self.navigation = TarNavSDNL()
        self.is_navigating = False
        self.tracking_mask = TrackingMask()
        self.new_tracking_mask_msg = False
        self.is_tracking_comm = False

        self.neck_pan = 0.0
        self.neck_tilt = 0.0

        self.lidar_obstacle_points = []
        self.lidar_bottom_obstacle_points = []
        self.camera_obstacle_points = []
        self.final_obstacle_points = []

        self.robot_radius = 0.560/2 # meters
        self.lidar_radius = 0.050/2 # meters
        self.lidar_to_robot_center = 0.255 # meters

        self.NORTE = -45.0
        self.imu_orientation_norm_rad = 0.0


    def check_yolos_timer(self):
        
        if self.new_detected_people:
            self.new_detected_people = False
            self.is_yolo_pose_comm = True
        else:
            self.is_yolo_pose_comm = False

        if self.new_detected_objects: # or self.new_detected_shoes or self.new_detected_doors:
            self.new_detected_objects = False
            self.is_yolo_obj_head_comm = True
        else:
            self.is_yolo_obj_head_comm = False
    
        if self.new_detected_objects_hand: # or self.new_detected_shoes_hand or self.new_detected_doors_hand:
            self.new_detected_objects_hand = False
            self.is_yolo_obj_hand_comm = True
        else:
            self.is_yolo_obj_hand_comm = False

    def check_tracking_timer(self):

        if self.new_tracking_mask_msg:
            self.new_tracking_mask_msg = False
            self.is_tracking_comm = True
        else:
            self.is_tracking_comm = False

    def battery_timer(self):
        
        self.battery_timer_ctr += 1
        check = False
        current_is_low_level_on = self.set_acceleration_ramp_client.wait_for_service(0.0)
        
        if current_is_low_level_on and not self.previous_is_low_level_on:
            check = True
            self.battery_timer_ctr = 0
        elif self.battery_timer_ctr >= 60.0 and current_is_low_level_on:
            check = True
            self.battery_timer_ctr = 0

        if check:
            print("CHECKING BATTERY VOLTAGE")
            request=GetVCCs.Request()
            self.call_vccs_command_server(request=request)

        self.previous_is_low_level_on = current_is_low_level_on

    def nodes_used_callback(self, request, response): # this only exists to have a service where we can: "while not self.arm_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # 
        # bool charmie_arm
        # bool charmie_audio
        # bool charmie_face
        # bool charmie_head_camera
        # bool charmie_hand_camera
        # bool charmie_lidar
        # bool charmie_localisation
        # bool charmie_low_level
        # bool charmie_navigation
        # bool charmie_nav2
        # bool charmie_neck
        # bool charmie_obstacles
        # bool charmie_odometry
        # bool charmie_point_cloud
        # bool charmie_ps4_controller
        # bool charmie_speakers
        # bool charmie_tracking
        # bool charmie_yolo_objects
        # bool charmie_yolo_pose
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages

        print("TASK NODES USED RECEIVED")
        self.nodes_used = request
        
        
        response.success = True
        response.message = "Nodes Used Sucessfully Received"
        return response

    def audio_interpreted_callback(self, str: String):
        print("Audio Interpreted:", str.data)

    def audio_final_callback(self, str: String):
        print("Audio Final:", str.data)

    def get_color_image_hand_callback(self, img: Image):
        self.hand_rgb = img
        self.new_hand_rgb = True

        self.last_hand_camera_time = self.hand_camera_time
        self.hand_camera_time = time.time()
        self.hand_rgb_fps = round(1/(self.hand_camera_time-self.last_hand_camera_time), 1)

    def get_color_image_head_callback(self, img: Image):
        self.head_rgb = img
        self.new_head_rgb = True

        self.last_head_camera_time = self.head_camera_time
        self.head_camera_time = time.time()
        self.head_rgb_fps = round(1/(self.head_camera_time-self.last_head_camera_time), 1)

    def get_aligned_depth_image_hand_callback(self, img: Image):
        self.hand_depth = img
        self.new_hand_depth = True

        self.last_hand_depth_camera_time = self.hand_depth_camera_time
        self.hand_depth_camera_time = time.time()
        self.hand_depth_fps = round(1/(self.hand_depth_camera_time-self.last_hand_depth_camera_time), 1)


    def get_aligned_depth_image_head_callback(self, img: Image):
        self.head_depth = img
        self.new_head_depth = True
        
        self.last_head_depth_camera_time = self.head_depth_camera_time
        self.head_depth_camera_time = time.time()
        self.head_depth_fps = round(1/(self.head_depth_camera_time-self.last_head_depth_camera_time), 1)


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

    
    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people
        self.new_detected_people = True
    
        self.last_head_yp_time = self.head_yp_time
        self.head_yp_time = time.time()
        self.head_yp_fps = round(1/(self.head_yp_time-self.last_head_yp_time), 1)

    def object_detected_filtered_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects = det_object
        self.new_detected_objects = True

        self.last_head_yo_time = self.head_yo_time
        self.head_yo_time = time.time()
        self.head_yo_fps = round(1/(self.head_yo_time-self.last_head_yo_time), 1)

    def object_detected_filtered_hand_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects_hand = det_object
        self.new_detected_objects_hand = True

        self.last_hand_yo_time = self.hand_yo_time
        self.hand_yo_time = time.time()
        self.hand_yo_fps = round(1/(self.hand_yo_time-self.last_hand_yo_time), 1)

    def tracking_mask_callback(self, mask: TrackingMask):
        self.tracking_mask = mask
        self.new_tracking_mask_msg = True
    
        self.last_track_time = self.track_time
        self.track_time = time.time()
        self.track_fps = round(1/(self.track_time-self.last_track_time), 1)

    def ps4_controller_callback(self, controller: PS4Controller):
        self.ps4_controller_time = time.time()

    def get_orientation_callback(self, orientation: Float32):
        imu_orientation_norm = orientation.data - self.NORTE
        if imu_orientation_norm > 180.0:
            imu_orientation_norm -= 360.0
        if imu_orientation_norm < -180.0:
            imu_orientation_norm += 360.0

        self.imu_orientation_norm_rad = math.radians(imu_orientation_norm)
        self.robot_pose.theta = -self.imu_orientation_norm_rad
        
    def get_camera_obstacles_callback(self, points: ListOfPoints):
        self.camera_obstacle_points = points.coords
        # print("Received Points")
        # print
        
    def get_final_obstacles_callback(self, points: ListOfPoints):
        self.final_obstacle_points = points.coords
        # print("Received Points")
        # print(self.final_obstacle_points)

    def lidar_callback(self, scan: LaserScan):
        self.scan = scan
        # print(scan)

        self.lidar_time = time.time()

        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.lidar_obstacle_points.clear()

        # calculates list of lidar obstacle points
        for i in range(len(scan.ranges)):
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if value > self.min_dist_error: # and value < self.max_dist_error:

                obs_y = -value * math.cos(key + self.robot_pose.theta + math.pi/2)
                obs_x = value * math.sin(key + self.robot_pose.theta + math.pi/2)

                adj_x = (self.robot_radius - self.lidar_radius + 0.035)*math.cos(self.robot_pose.theta + math.pi/2)
                adj_y = (self.robot_radius - self.lidar_radius + 0.035)*math.sin(self.robot_pose.theta + math.pi/2)

                target = Point()
                target.x = self.robot_pose.x + obs_x + adj_x
                target.y = self.robot_pose.y + obs_y + adj_y
                target.z = 0.35 # lidar height on the robot

                self.lidar_obstacle_points.append(target)

    def lidar_bottom_callback(self, scan: LaserScan):
        self.scan = scan
        # print(scan)

        self.lidar_bottom_time = time.time()

        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.lidar_bottom_obstacle_points.clear()

        # calculates list of lidar obstacle points
        for i in range(len(scan.ranges)):
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if value > self.min_dist_error: # and value < self.max_dist_error:

                obs_y = -value * math.cos(key + self.robot_pose.theta + math.pi/2)
                obs_x = value * math.sin(key + self.robot_pose.theta + math.pi/2)

                adj_x = (self.robot_radius - self.lidar_radius - 0.015)*math.cos(self.robot_pose.theta + math.pi/2)
                adj_y = (self.robot_radius - self.lidar_radius - 0.015)*math.sin(self.robot_pose.theta + math.pi/2)

                target = Point()
                target.x = self.robot_pose.x + obs_x + adj_x
                target.y = self.robot_pose.y + obs_y + adj_y
                target.z = 0.35 # lidar height on the robot

                self.lidar_bottom_obstacle_points.append(target)

    def target_pos_callback(self, nav: TarNavSDNL):
        self.navigation = nav
        self.is_navigating = True
        print(nav)

    def flag_navigation_reached_callback(self, flag: Bool):
        self.is_navigating = False

    def get_neck_position_callback(self, pose: NeckPosition):
        # print("Received new neck position. PAN = ", pose.pan, " TILT = ", pose.tilt)
        self.neck_pan = -math.radians(- pose.pan)
        self.neck_tilt = -math.radians(- pose.tilt)

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_pose = pose
        self.localisation_time = time.time()
        
        self.all_pos_x_val.append(self.robot_pose.x)
        self.all_pos_y_val.append(self.robot_pose.y)
        
        # self.odometry_time = time.time()
        # self.robot_pose.theta = pose.theta
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        # self.amcl_pose = msg
        self.amcl_time = time.time()

    def search_for_person_detections_callback(self, points: ListOfDetectedPerson):
        self.search_for_person = points
        self.new_search_for_person = True
        
    def search_for_object_detections_callback(self, points: ListOfDetectedObject):
        self.search_for_object = points
        self.new_search_for_object = True

    def call_vccs_command_server(self, request=GetVCCs.Request()):
    
        future = self.get_vccs_client.call_async(request)
        future.add_done_callback(self.callback_call_vccs_command)
        
    def callback_call_vccs_command(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info("Battery_Voltage: "+str(response.battery_voltage) + ", Emergency_Button: " + str(response.emergency_stop))
            self.battery_voltage = response.battery_voltage
            self.emergency_stop = response.emergency_stop
            self.waited_for_end_of_get_vccs = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))  
    
class CheckNodesMain():

    def __init__(self, node: DebugVisualNode):
        self.node = node

        self.CHECK_ARM_UFACTORY_NODE = False
        self.CHECK_ARM_NODE = False
        self.CHECK_AUDIO_NODE = False
        self.CHECK_FACE_NODE = False
        self.CHECK_HEAD_CAMERA_NODE = False
        self.CHECK_HAND_CAMERA_NODE = False
        self.CHECK_LIDAR_NODE = False
        self.CHECK_LLM_NODE = False
        self.CHECK_LOCALISATION_NODE = False
        self.CHECK_LOW_LEVEL_NODE = False
        self.CHECK_NAVIGATION_NODE = False
        self.CHECK_NAV2_NODE = False
        self.CHECK_NECK_NODE = False
        self.CHECK_OBSTACLES_NODE = False
        self.CHECK_ODOMETRY_NODE = False
        self.CHECK_POINT_CLOUD_NODE = False
        self.CHECK_PS4_CONTROLLER_NODE = False
        self.CHECK_SPEAKERS_NODE = False
        self.CHECK_TRACKING_NODE = False
        self.CHECK_YOLO_OBJECTS_NODE = False
        self.CHECK_YOLO_POSE_NODE = False

        self.WAIT_TIME_CHECK_NODE = 0.0
        self.MIN_TIMEOUT_FOR_CHECK_NODE = 1.0

    def main(self):

        while True:

            current_time = time.time()

            # ARM_UFACTORY
            if not self.node.set_position_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.get_logger().warn("Waiting for Arm (uFactory) ...")
                self.CHECK_ARM_UFACTORY_NODE = False
            else:
                self.CHECK_ARM_UFACTORY_NODE = True

            # ARM_CHARMIE
            if not self.node.arm_trigger_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Arm (CHARMIE) ...")
                self.CHECK_ARM_NODE = False
            else:
                self.CHECK_ARM_NODE = True

            # AUDIO
            if not self.node.get_audio_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Audio ...")
                self.CHECK_AUDIO_NODE = False
            else:
                self.CHECK_AUDIO_NODE = True

            # FACE
            if not self.node.face_command_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Face ...")
                self.CHECK_FACE_NODE = False
            else:
                self.CHECK_FACE_NODE = True

            # HEAD CAMERA
            if current_time - self.node.head_camera_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Head Camera ...")
                self.CHECK_HEAD_CAMERA_NODE = False
            else:
                self.CHECK_HEAD_CAMERA_NODE = True

            # HAND CAMERA
            if current_time - self.node.hand_camera_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Hand Camera ...")
                self.CHECK_HAND_CAMERA_NODE = False
            else:
                self.CHECK_HAND_CAMERA_NODE = True

            ########## MISSING CHECK FOR BOTTOM LIDAR, IF NEEDED ##########
            # LIDAR
            if current_time - self.node.lidar_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_LIDAR_NODE = False
            else:
                self.CHECK_LIDAR_NODE = True

            # LLM
            if not self.node.llm_demonstration_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Face ...")
                self.CHECK_LLM_NODE = False
            else:
                self.CHECK_LLM_NODE = True

            # LOCALISATION
            if current_time - self.node.localisation_time > self.MIN_TIMEOUT_FOR_CHECK_NODE: # or current_time - self.node.amcl_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
            #     # self.node.get_logger().warn("Waiting for Topic Localisation ...")
                self.CHECK_LOCALISATION_NODE = False
            else:
                self.CHECK_LOCALISATION_NODE = True

            # LOW LEVEL
            if not self.node.set_acceleration_ramp_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Navigation ...")
                self.CHECK_LOW_LEVEL_NODE = False
                self.node.battery_voltage = 0.0
            else:
                self.CHECK_LOW_LEVEL_NODE = True

            # NAVIGATION
            if not self.node.nav_trigger_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Navigation ...")
                self.CHECK_NAVIGATION_NODE = False
            else:
                self.CHECK_NAVIGATION_NODE = True

            # NAV2
            if not self.node.nav2_client_.server_is_ready():
                # self.node.get_logger().warn("Waiting for Server Navigation ...")
                self.CHECK_NAV2_NODE = False
            else:
                self.CHECK_NAV2_NODE = True

            # NECK
            if not self.node.set_neck_position_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Neck ...")
                self.CHECK_NECK_NODE = False
            else:
                self.CHECK_NECK_NODE = True
            
            # OBSTACLES
            if not self.node.activate_obstacles_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Obstacles ...")
                self.CHECK_OBSTACLES_NODE = False
            else:
                self.CHECK_OBSTACLES_NODE = True

            # ODOMETRY
            if current_time - self.node.odometry_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Odometry ...")
                self.CHECK_ODOMETRY_NODE = False
            else:
                self.CHECK_ODOMETRY_NODE = True

            # POINT CLOUD
            if not self.node.point_cloud_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Speech ...")
                self.CHECK_POINT_CLOUD_NODE = False
            else:
                self.CHECK_POINT_CLOUD_NODE = True

            # PS4 CONTROLLER
            if current_time - self.node.ps4_controller_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic PS4 Controller ...")
                self.CHECK_PS4_CONTROLLER_NODE = False
            else:
                self.CHECK_PS4_CONTROLLER_NODE = True

            # SPEAKERS
            if not self.node.speech_command_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Speech ...")
                self.CHECK_SPEAKERS_NODE = False
            else:
                self.CHECK_SPEAKERS_NODE = True

            # TRACKING (SAM2)
            if not self.node.activate_tracking_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Tracking ...")
                self.CHECK_TRACKING_NODE = False
            else:
                self.CHECK_TRACKING_NODE = True

            # YOLO OBJECTS
            if not self.node.activate_yolo_objects_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Yolo Objects ...")
                self.CHECK_YOLO_OBJECTS_NODE = False
            else:
                self.CHECK_YOLO_OBJECTS_NODE = True
            
            # YOLO POSE
            if not self.node.activate_yolo_pose_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Yolo Pose ...")
                self.CHECK_YOLO_POSE_NODE = False
            else:
                self.CHECK_YOLO_POSE_NODE = True


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualNode()
    check_nodes = CheckNodesMain(node)
    th_nodes = threading.Thread(target=thread_check_nodes, args=(node,check_nodes,), daemon=True)
    th_main = threading.Thread(target=thread_main_debug_visual, args=(node,check_nodes,), daemon=True)
    th_main.start()
    th_nodes.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_debug_visual(node: DebugVisualNode, check_nodes: CheckNodesMain):
    main = DebugVisualMain(node, check_nodes)
    main.main()

def thread_check_nodes(node: DebugVisualNode, check_nodes: CheckNodesMain):
    check_nodes.main()

class DebugVisualMain():

    def __init__(self, node: DebugVisualNode, check_nodes: CheckNodesMain):
        
        self.node = node
        self.check_nodes = check_nodes

        self.RED     = (255,  0,  0)
        self.GREEN   = (  0,255,  0)
        self.BLUE    = ( 50, 50,255)
        self.BLUE_L  = (  0,128,255)
        self.WHITE   = (255,255,255)
        self.GREY    = (128,128,128)
        self.BLACK   = (  0,  0,  0)
        self.ORANGE  = (255,153, 51)
        self.MAGENTA = (255, 51,255)
        self.YELLOW  = (255,255,  0)
        self.PURPLE  = (132, 56,255)
        self.CYAN    = (  0,255,255)

        self.WIDTH, self.HEIGHT = 1387, 752

        self.BB_WIDTH = 3

        self.cam_width_ = 640
        self.cam_height_ = 360
        self.camera_resize_ratio = 1.0
        self.cams_initial_height = 10
        self.cams_initial_width = 165

        self.map_init_width = int(self.cams_initial_width+self.cam_width_+self.cams_initial_height)
        self.map_init_height = 260

        self.MAP_SIDE = int(self.HEIGHT - 260 - 12)
        self.MAP_SCALE = 1.40
        self.MAP_ADJUST_X = 0.8
        self.MAP_ADJUST_Y = -3.0

        self.MAP_ZOOM_INC = 0.2
        self.MAP_SHIFT_INC = 1.0

        self.first_pos_h = 19.8 # 19.1
        
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        logo_midpath = "/charmie_ws/src/configuration_files/docs/logos/"
        configuration_files_midpath = "/charmie_ws/src/configuration_files/"
        self.save_recordings_midpath = "/charmie_ws/src/charmie_gui/charmie_gui/saved_gui_recordings/"

        self.br = CvBridge()

        pygame.init()

        self.FPS = 20

        os.environ['SDL_VIDEO_CENTERED'] = '1'
        info = pygame.display.Info()
        screen_width, screen_height = info.current_w, info.current_h
        print(screen_width, screen_height)
        self.WIN = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)

        # self.text_font = pygame.font.SysFont("Arial", 30)
        self.text_font_t = pygame.font.SysFont(None, 28)
        self.text_font   = pygame.font.SysFont(None, 24)
        self.text_map_font   = pygame.font.SysFont(None, 20)

        self.button_size = 30

        self.button_zoom_in = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(5.5*self.button_size*self.camera_resize_ratio), self.map_init_height-(self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='Z+', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_zoom_in_function())

        self.button_zoom_out = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(4.5*self.button_size*self.camera_resize_ratio), self.map_init_height-(self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='Z-', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_zoom_out_function())
        
        self.button_shift_up = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(2*self.button_size*self.camera_resize_ratio), self.map_init_height-(2*self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='U', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_shift_up_function())
        
        self.button_shift_down = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(2*self.button_size*self.camera_resize_ratio), self.map_init_height-(self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='D', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_shift_down_function())
        
        self.button_shift_left = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(3*self.button_size*self.camera_resize_ratio), self.map_init_height-(self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='L', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_shift_left_function())
        
        self.button_shift_right = Button(self.WIN, self.map_init_width+self.MAP_SIDE-(1*self.button_size*self.camera_resize_ratio), self.map_init_height-(self.button_size*self.camera_resize_ratio), self.button_size*self.camera_resize_ratio, self.button_size*self.camera_resize_ratio,
                                     text='R', fontSize=16, textColour=self.WHITE, inactiveColour=(200, 50, 0), hoverColour=(150, 0, 0), pressedColour=(255, 75, 0), radius=5,
                                     onClick=lambda: self.button_shift_right_function())
        
        # self.textbox = TextBox(self.WIN, 500, 500, 800, 80, fontSize=50,
        #           borderColour=(255, 0, 0), textColour=(0, 200, 0),
        #           onSubmit=self.output, radius=10, borderThickness=5)        
        # def output(self):
        # Get text in the textbox
        # print(self.textbox.getText())

        icon = pygame.image.load(self.home+logo_midpath+"logo_light_cropped_squared.png")
        pygame.display.set_icon(icon)
        pygame.display.set_caption("CHARMIE Debug Node")

        # Open all configuration files
        try:
            with open(self.home + configuration_files_midpath + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)

            with open(self.home + configuration_files_midpath + 'furniture.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)

        except:
            print("Could NOT import data from json configuration files. (objects, rooms and furniture)")

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.current_datetime = str(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
        
        self.init_pos_w_rect_check_nodes = 15
        self.init_pos_h_rect_check_nodes = 40
        self.deviation_pos_h_rect_check_nodes = 24
        self.square_size_rect_check_nodes = 10

        self.ARM_UFACTORY_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*0, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_ARM_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*1, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_AUDIO_NODE_RECT            = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*2, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_FACE_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*3, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.HEAD_CAMERA_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*4, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.HAND_CAMERA_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*5, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LIDAR_NODE_RECT            = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*6, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LLM_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*7, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LOCALISATION_NODE_RECT     = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*8, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LOW_LEVEL_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*9, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        # the two navs have the same node rect for now, only nav2 will be used for now
        self.CHARMIE_NAVIGATION_NODE_RECT       = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*10, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_NAV2_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*10, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_NECK_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*11, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_OBSTACLES_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*12, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_ODOMETRY_NODE_RECT         = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*13, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_POINT_CLOUD_NODE_RECT      = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*14, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_PS4_CONTROLLER_NODE_RECT   = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*15, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_SPEAKERS_NODE_RECT         = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*16, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_TRACKING_NODE_RECT         = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*17, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_YOLO_OBJECTS_NODE_RECT     = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*18, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_YOLO_POSE_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*19, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)

        toggle_h_init = 21.0
        toggle_h_diff = 2.25
        self.toggle_record =         Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(toggle_h_init+0*toggle_h_diff)), 40, 16)
        self.toggle_pause_cams =     Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(toggle_h_init+1*toggle_h_diff)), 40, 16)
        self.toggle_head_rgb_depth = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(toggle_h_init+2*toggle_h_diff)), 40, 16)
        self.toggle_hand_rgb_depth = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(toggle_h_init+3*toggle_h_diff)), 40, 16)

        self.toggle_activate_objects_head =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+50, 40, 16)
        self.toggle_activate_furniture_head = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+90,  self.cams_initial_height+50, 40, 16)
        self.toggle_activate_shoes_head =     Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+192, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_objects_hand =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+298, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_furniture_hand = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+390, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_shoes_hand =     Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+492, self.cams_initial_height+50, 40, 16)

        self.toggle_pose_activate =       Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_waving =         Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+93,  self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_front_close =    Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+182, self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_legs_visible =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+302, self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_characteristcs = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+427, self.cams_initial_height+80+50, 40, 16)
        
        self.toggle_obstacles_lidar_top =    Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+160+50, 40, 16)
        self.toggle_obstacles_lidar_bottom = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+100,  self.cams_initial_height+160+50, 40, 16)
        self.toggle_obstacles_head_camera =  Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+233, self.cams_initial_height+160+50, 40, 16)

        self.last_toggle_record = False

        self.last_toggle_activate_objects_head =   False
        self.last_toggle_activate_furniture_head = False
        self.last_toggle_activate_shoes_head =     False
        self.last_toggle_activate_objects_hand =   False
        self.last_toggle_activate_furniture_hand = False
        self.last_toggle_activate_shoes_hand =     False

        self.last_toggle_pose_activate =       False
        self.last_toggle_pose_waving =         False
        self.last_toggle_pose_front_close =    False
        self.last_toggle_pose_legs_visible =   False
        self.last_toggle_pose_characteristcs = False
        
        self.last_toggle_obstacles_lidar_top =    False
        self.last_toggle_obstacles_lidar_bottom = False
        self.last_toggle_obstacles_head_camera =  False

        self.curr_head_rgb = Image()
        self.last_head_rgb = Image()
        self.curr_head_depth = Image()
        self.last_head_depth = Image()
        self.curr_hand_rgb = Image()
        self.last_hand_rgb = Image()
        self.curr_hand_depth = Image()
        self.last_hand_depth = Image()

        self.curr_detected_people = ListOfDetectedPerson()
        self.last_detected_people = ListOfDetectedPerson()
    
        self.curr_detected_objects = ListOfDetectedObject()
        self.last_detected_objects = ListOfDetectedObject()
        self.curr_detected_objects_hand = ListOfDetectedObject()
        self.last_detected_objects_hand = ListOfDetectedObject()

        self.curr_tracking = TrackingMask()
        self.last_tracking = TrackingMask()

        self.show_navigation_locations = False

        # robot info
        self.robot_radius = self.node.robot_radius

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
    
    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message

    def button_zoom_in_function(self):
        print(self.MAP_SCALE - self.MAP_ZOOM_INC)
        if self.MAP_SCALE - self.MAP_ZOOM_INC > 0.1:
            self.MAP_SCALE -= self.MAP_ZOOM_INC

    def button_zoom_out_function(self):
        self.MAP_SCALE += self.MAP_ZOOM_INC

    def button_shift_up_function(self):
        self.MAP_ADJUST_Y -= self.MAP_SHIFT_INC
    
    def button_shift_down_function(self):
        self.MAP_ADJUST_Y += self.MAP_SHIFT_INC

    def button_shift_left_function(self):
        self.MAP_ADJUST_X -= self.MAP_SHIFT_INC

    def button_shift_right_function(self):
        self.MAP_ADJUST_X += self.MAP_SHIFT_INC
    
    def draw_text(self, text, font, text_col, x, y):
        img = font.render(text, True, text_col)
        self.WIN.blit(img, (x, y))

    def draw_transparent_rect(self, x, y, width, height, color, alpha):
        temp_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        temp_surface.fill((*color, alpha))
        self.WIN.blit(temp_surface, (x, y))

    def draw_nodes_check(self):

        self.draw_text("Check Nodes:", self.text_font_t, self.WHITE, 10, 10)

        # ARM_UFACTORY
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_arm, self.check_nodes.CHECK_ARM_UFACTORY_NODE)
        self.draw_text("Arm uFactory", self.text_font, tc, self.ARM_UFACTORY_NODE_RECT.x+2*self.ARM_UFACTORY_NODE_RECT.width, self.ARM_UFACTORY_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.ARM_UFACTORY_NODE_RECT)
            
        # ARM_CHARMIE
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_arm, self.check_nodes.CHECK_ARM_NODE)
        self.draw_text("Arm", self.text_font, tc, self.CHARMIE_ARM_NODE_RECT.x+2*self.CHARMIE_ARM_NODE_RECT.width, self.CHARMIE_ARM_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_ARM_NODE_RECT)

        # AUDIO
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_audio, self.check_nodes.CHECK_AUDIO_NODE)
        self.draw_text("Audio", self.text_font, tc, self.CHARMIE_AUDIO_NODE_RECT.x+2*self.CHARMIE_AUDIO_NODE_RECT.width, self.CHARMIE_AUDIO_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_AUDIO_NODE_RECT)

        # FACE
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_face, self.check_nodes.CHECK_FACE_NODE)
        self.draw_text("Face", self.text_font, tc, self.CHARMIE_FACE_NODE_RECT.x+2*self.CHARMIE_FACE_NODE_RECT.width, self.CHARMIE_FACE_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_FACE_NODE_RECT)

        # HEAD CAMERA
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_head_camera, self.check_nodes.CHECK_HEAD_CAMERA_NODE)
        self.draw_text("Head Camera", self.text_font, tc, self.HEAD_CAMERA_NODE_RECT.x+2*self.HEAD_CAMERA_NODE_RECT.width, self.HEAD_CAMERA_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.HEAD_CAMERA_NODE_RECT)
        
        # HAND CAMERA
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_hand_camera, self.check_nodes.CHECK_HAND_CAMERA_NODE)
        self.draw_text("Hand Camera", self.text_font, tc, self.HAND_CAMERA_NODE_RECT.x+2*self.HAND_CAMERA_NODE_RECT.width, self.HAND_CAMERA_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.HAND_CAMERA_NODE_RECT)
        
        # LIDAR
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_lidar, self.check_nodes.CHECK_LIDAR_NODE)
        self.draw_text("Lidar", self.text_font, tc, self.CHARMIE_LIDAR_NODE_RECT.x+2*self.CHARMIE_LIDAR_NODE_RECT.width, self.CHARMIE_LIDAR_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_LIDAR_NODE_RECT)
        
        # LLM
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_llm, self.check_nodes.CHECK_LLM_NODE)
        self.draw_text("LLM", self.text_font, tc, self.CHARMIE_LLM_NODE_RECT.x+2*self.CHARMIE_LLM_NODE_RECT.width, self.CHARMIE_LLM_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_LLM_NODE_RECT)

        # LOCALISATION
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_localisation, self.check_nodes.CHECK_LOCALISATION_NODE)
        self.draw_text("Localisation", self.text_font, tc, self.CHARMIE_LOCALISATION_NODE_RECT.x+2*self.CHARMIE_LOCALISATION_NODE_RECT.width, self.CHARMIE_LOCALISATION_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_LOCALISATION_NODE_RECT)

        # LOW LEVEL
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_low_level, self.check_nodes.CHECK_LOW_LEVEL_NODE)
        self.draw_text("Low Level", self.text_font, tc, self.CHARMIE_LOW_LEVEL_NODE_RECT.x+2*self.CHARMIE_LOW_LEVEL_NODE_RECT.width, self.CHARMIE_LOW_LEVEL_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_LOW_LEVEL_NODE_RECT)
        
        # NAVIGATION
        # tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_navigation, self.check_nodes.CHECK_NAVIGATION_NODE)
        # self.draw_text("Navigation", self.text_font, tc, self.CHARMIE_NAVIGATION_NODE_RECT.x+2*self.CHARMIE_NAVIGATION_NODE_RECT.width, self.CHARMIE_NAVIGATION_NODE_RECT.y-2)
        # pygame.draw.rect(self.WIN, rc, self.CHARMIE_NAVIGATION_NODE_RECT)
        
        # NAV2
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_nav2, self.check_nodes.CHECK_NAV2_NODE)
        self.draw_text("Nav2", self.text_font, tc, self.CHARMIE_NAV2_NODE_RECT.x+2*self.CHARMIE_NAV2_NODE_RECT.width, self.CHARMIE_NAV2_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_NAV2_NODE_RECT)

        # NECK
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_neck, self.check_nodes.CHECK_NECK_NODE)
        self.draw_text("Neck", self.text_font, tc, self.CHARMIE_NECK_NODE_RECT.x+2*self.CHARMIE_NECK_NODE_RECT.width, self.CHARMIE_NECK_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_NECK_NODE_RECT)

        # OBSTACLES
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_obstacles, self.check_nodes.CHECK_OBSTACLES_NODE)
        self.draw_text("Obstacles", self.text_font, tc, self.CHARMIE_OBSTACLES_NODE_RECT.x+2*self.CHARMIE_OBSTACLES_NODE_RECT.width, self.CHARMIE_OBSTACLES_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_OBSTACLES_NODE_RECT)

        # ODOMETRY
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_odometry, self.check_nodes.CHECK_ODOMETRY_NODE)
        self.draw_text("Odometry", self.text_font, tc, self.CHARMIE_ODOMETRY_NODE_RECT.x+2*self.CHARMIE_ODOMETRY_NODE_RECT.width, self.CHARMIE_ODOMETRY_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_ODOMETRY_NODE_RECT)

        # POINT CLOUD
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_point_cloud, self.check_nodes.CHECK_POINT_CLOUD_NODE)
        self.draw_text("Point Cloud", self.text_font, tc, self.CHARMIE_POINT_CLOUD_NODE_RECT.x+2*self.CHARMIE_POINT_CLOUD_NODE_RECT.width, self.CHARMIE_POINT_CLOUD_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_POINT_CLOUD_NODE_RECT)

        # PS4 CONTROLLER
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_ps4_controller, self.check_nodes.CHECK_PS4_CONTROLLER_NODE)
        self.draw_text("PS4 Controller", self.text_font, tc, self.CHARMIE_PS4_CONTROLLER_NODE_RECT.x+2*self.CHARMIE_PS4_CONTROLLER_NODE_RECT.width, self.CHARMIE_PS4_CONTROLLER_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_PS4_CONTROLLER_NODE_RECT)

        # SPEAKERS
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_speakers, self.check_nodes.CHECK_SPEAKERS_NODE)
        self.draw_text("Speakers", self.text_font, tc, self.CHARMIE_SPEAKERS_NODE_RECT.x+2*self.CHARMIE_SPEAKERS_NODE_RECT.width, self.CHARMIE_SPEAKERS_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_SPEAKERS_NODE_RECT)

        # TRACKING
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_tracking, self.check_nodes.CHECK_TRACKING_NODE)
        self.draw_text("Tracking SAM2", self.text_font, tc, self.CHARMIE_TRACKING_NODE_RECT.x+2*self.CHARMIE_TRACKING_NODE_RECT.width, self.CHARMIE_TRACKING_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_TRACKING_NODE_RECT)

        # YOLO OBJECTS
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_yolo_objects, self.check_nodes.CHECK_YOLO_OBJECTS_NODE)
        self.draw_text("YOLO Objects", self.text_font, tc, self.CHARMIE_YOLO_OBJECTS_NODE_RECT.x+2*self.CHARMIE_YOLO_OBJECTS_NODE_RECT.width, self.CHARMIE_YOLO_OBJECTS_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_YOLO_OBJECTS_NODE_RECT)

        # YOLO POSE
        tc, rc = self.get_check_nodes_rectangle_and_text_color(self.node.nodes_used.charmie_yolo_pose, self.check_nodes.CHECK_YOLO_POSE_NODE)
        self.draw_text("YOLO Pose", self.text_font, tc, self.CHARMIE_YOLO_POSE_NODE_RECT.x+2*self.CHARMIE_YOLO_POSE_NODE_RECT.width, self.CHARMIE_YOLO_POSE_NODE_RECT.y-2)
        pygame.draw.rect(self.WIN, rc, self.CHARMIE_YOLO_POSE_NODE_RECT)

    def get_check_nodes_rectangle_and_text_color(self, text_condition, rectangle_condition):
        
        rectangle_color_checked = self.GREEN
        rectangle_color_not_checked = self.RED
        text_color_used = self.BLUE_L
        text_color_not_used = self.WHITE

        if text_condition:
            text_color = text_color_used
        else:
            text_color = text_color_not_used
        
        if rectangle_condition:
            rectangle_color = rectangle_color_checked
        else:
            rectangle_color = rectangle_color_not_checked
        
        return text_color, rectangle_color


    def draw_cameras(self):

        self.curr_head_rgb = self.node.head_rgb
        self.curr_head_depth = self.node.head_depth
        self.curr_hand_rgb = self.node.hand_rgb
        self.curr_hand_depth = self.node.hand_depth

        if self.toggle_pause_cams.getValue():
            used_img_head_rgb = self.last_head_rgb
            used_img_head_depth = self.last_head_depth
            used_img_hand_rgb = self.last_hand_rgb
            used_img_hand_depth = self.last_hand_depth

        else:
            used_img_head_rgb = self.curr_head_rgb 
            used_img_head_depth = self.curr_head_depth 
            used_img_hand_rgb = self.curr_hand_rgb
            used_img_hand_depth = self.curr_hand_depth

        self.last_head_rgb = used_img_head_rgb 
        self.last_head_depth = used_img_head_depth 
        self.last_hand_rgb = used_img_hand_rgb
        self.last_hand_depth = used_img_hand_depth


        if not self.toggle_head_rgb_depth.getValue():

            if self.node.new_head_rgb:
                
                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_head_rgb, "bgr8")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD RGB): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_, 3), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cams_initial_height, 80, 10*self.cams_initial_height, self.BLACK, 85)
                self.draw_text("RGB: "+str(self.node.head_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cams_initial_height)
                self.draw_text("Dep: "+str(self.node.head_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, 3*self.cams_initial_height)
                self.draw_text("Y_O: "+str(self.node.head_yo_fps), self.text_font, self.WHITE, self.cams_initial_width, 5*self.cams_initial_height)
                self.draw_text("Y_P: "+str(self.node.head_yp_fps), self.text_font, self.WHITE, self.cams_initial_width, 7*self.cams_initial_height)
                self.draw_text("Track: "+str(self.node.track_fps), self.text_font, self.WHITE, self.cams_initial_width, 9*self.cams_initial_height)

            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cams_initial_height+(self.cam_height_//2))
        else:
            if self.node.new_head_depth:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_head_depth, "passthrough")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD Depth): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                
                """
                # Get the minimum and maximum values in the depth image
                min_val, max_val, _, _ = cv2.minMaxLoc(opencv_image)

                # Avoid zero values (invalid measurements) in the depth image
                if min_val == 0:
                    min_val = np.min(opencv_image[opencv_image > 0])
                if max_val == 0:
                    max_val = np.max(opencv_image[opencv_image > 0])

                print(min_val, max_val)
                """

                min_val = 0
                max_val = 6000

                # Normalize the depth image to fall between 0 and 1
                # depth_normalized = cv2.normalize(opencv_image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                # Normalize the depth image to fall between 0 and 1
                depth_normalized = (opencv_image - min_val) / (max_val - min_val)
                depth_normalized = np.clip(depth_normalized, 0, 1)
                
                # Convert the normalized depth image to an 8-bit image (0-255)
                depth_8bit = (depth_normalized * 255).astype(np.uint8)

                # Apply a colormap to the 8-bit depth image
                opencv_image = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cams_initial_height, 80, 10*self.cams_initial_height, self.BLACK, 85)
                self.draw_text("RGB: "+str(self.node.head_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cams_initial_height)
                self.draw_text("Dep: "+str(self.node.head_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, 3*self.cams_initial_height)
                self.draw_text("Y_O: "+str(self.node.head_yo_fps), self.text_font, self.WHITE, self.cams_initial_width, 5*self.cams_initial_height)
                self.draw_text("Y_P: "+str(self.node.head_yp_fps), self.text_font, self.WHITE, self.cams_initial_width, 7*self.cams_initial_height)
                self.draw_text("Track: "+str(self.node.track_fps), self.text_font, self.WHITE, self.cams_initial_width, 9*self.cams_initial_height)

            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cams_initial_height+(self.cam_height_//2))


        if not self.toggle_hand_rgb_depth.getValue():

            if self.node.new_hand_rgb:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_hand_rgb, "bgr8")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HAND RGB): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_, 3), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cam_height_+2*self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, 80, 6*self.cams_initial_height, self.BLACK, 85)
                self.draw_text("RGB: "+str(self.node.hand_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+2*self.cams_initial_height)
                self.draw_text("Dep: "+str(self.node.hand_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+3*self.cams_initial_height+self.cams_initial_height)
                self.draw_text("Y_O: "+str(self.node.hand_yo_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+5*self.cams_initial_height+self.cams_initial_height)
                
            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cam_height_+2*self.cams_initial_height+(self.cam_height_//2))

        else:

            if self.node.new_hand_depth:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_hand_depth, "passthrough")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD Depth): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                
                """
                # Get the minimum and maximum values in the depth image
                min_val, max_val, _, _ = cv2.minMaxLoc(opencv_image)

                # Avoid zero values (invalid measurements) in the depth image
                if min_val == 0:
                    min_val = np.min(opencv_image[opencv_image > 0])
                if max_val == 0:
                    max_val = np.max(opencv_image[opencv_image > 0])

                print(min_val, max_val)
                """

                min_val = 0
                max_val = 3000

                # Normalize the depth image to fall between 0 and 1
                # depth_normalized = cv2.normalize(opencv_image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                # Normalize the depth image to fall between 0 and 1
                depth_normalized = (opencv_image - min_val) / (max_val - min_val)
                depth_normalized = np.clip(depth_normalized, 0, 1)
                
                # Convert the normalized depth image to an 8-bit image (0-255)
                depth_8bit = (depth_normalized * 255).astype(np.uint8)

                # Apply a colormap to the 8-bit depth image
                opencv_image = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cam_height_+2*self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, 80, 6*self.cams_initial_height, self.BLACK, 85)
                self.draw_text("RGB: "+str(self.node.hand_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+2*self.cams_initial_height)
                self.draw_text("Dep: "+str(self.node.hand_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+3*self.cams_initial_height+self.cams_initial_height)
                self.draw_text("Y_O: "+str(self.node.hand_yo_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+5*self.cams_initial_height+self.cams_initial_height)
                
            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cam_height_+2*self.cams_initial_height+(self.cam_height_//2))

        self.draw_text("Record Data:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*self.first_pos_h)
        self.draw_text("Pause Cams:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(self.first_pos_h+2.5-0.2))
        self.draw_text("Depth Head:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(self.first_pos_h+5.0-0.4))
        self.draw_text("Depth Hand:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(self.first_pos_h+7.5-0.6))

    def draw_activates(self):


        self.draw_text("Activate YOLO Objects: (Head/Hand)", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, self.cams_initial_height)
        self.draw_text("Activate YOLO Pose:", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 80+self.cams_initial_height)
        self.draw_text("Activate Obstacles:", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 160+self.cams_initial_height)
        
        self.draw_text("Objects:      Furniture:      Shoes:      /      Objects:      Furniture:      Shoes:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 25+self.cams_initial_height)
        self.draw_text("Activate:      Waving:      Front Close:      Legs Visible:      Characteristics:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 80+25+self.cams_initial_height)
        self.draw_text("Lidar Top:      Lidar Bottom:      Head Camera:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 160+25+self.cams_initial_height)
        

        # this is done to make sure that the same value used for checking pressed is the same used to attribute to last toggle
        # YOLO OBJECTS ACTIVATE
        toggle_activate_objects_head   = self.toggle_activate_objects_head.getValue()
        toggle_activate_furniture_head = self.toggle_activate_furniture_head.getValue()
        toggle_activate_shoes_head     = self.toggle_activate_shoes_head.getValue()
        toggle_activate_objects_hand   = self.toggle_activate_objects_hand.getValue()
        toggle_activate_furniture_hand = self.toggle_activate_furniture_hand.getValue()
        toggle_activate_shoes_hand     = self.toggle_activate_shoes_hand.getValue()

        # YOLO POSE ACTIVATE
        toggle_pose_activate        = self.toggle_pose_activate.getValue()
        toggle_pose_waving          = self.toggle_pose_waving.getValue()
        toggle_pose_front_close     = self.toggle_pose_front_close.getValue()
        toggle_pose_legs_visible    = self.toggle_pose_legs_visible.getValue()
        toggle_pose_characteristcs  = self.toggle_pose_characteristcs.getValue()

        # OBSTACLES ACTIVATE
        toggle_obstacles_lidar_top      = self.toggle_obstacles_lidar_top.getValue()
        toggle_obstacles_lidar_bottom   = self.toggle_obstacles_lidar_bottom.getValue()
        toggle_obstacles_head_camera    = self.toggle_obstacles_head_camera.getValue()

        if toggle_activate_objects_head     != self.last_toggle_activate_objects_head or \
            toggle_activate_furniture_head  != self.last_toggle_activate_furniture_head or \
            toggle_activate_shoes_head      != self.last_toggle_activate_shoes_head or \
            toggle_activate_objects_hand    != self.last_toggle_activate_objects_hand or \
            toggle_activate_furniture_hand  != self.last_toggle_activate_furniture_hand or \
            toggle_activate_shoes_hand      != self.last_toggle_activate_shoes_hand:
            
            print("YOLO OBJECTS - CHANGED STATUS.")

            self.activate_yolo_objects(activate_objects=toggle_activate_objects_head, activate_shoes=toggle_activate_shoes_head, \
                                       activate_doors=toggle_activate_furniture_head, activate_objects_hand=toggle_activate_objects_hand, \
                                       activate_shoes_hand=toggle_activate_shoes_hand, activate_doors_hand=toggle_activate_furniture_hand)


        if toggle_pose_activate         != self.last_toggle_pose_activate or \
            toggle_pose_waving          != self.last_toggle_pose_waving or \
            toggle_pose_front_close     != self.last_toggle_pose_front_close or \
            toggle_pose_legs_visible    != self.last_toggle_pose_legs_visible or \
            toggle_pose_characteristcs  != self.last_toggle_pose_characteristcs:
            
            print("YOLO POSE - CHANGED STATUS.")

            self.activate_yolo_pose(activate=toggle_pose_activate, only_detect_person_legs_visible=toggle_pose_legs_visible, \
                                    only_detect_person_right_in_front=toggle_pose_front_close, only_detect_person_arm_raised=toggle_pose_waving, \
                                    characteristics=toggle_pose_characteristcs)
        

        if toggle_obstacles_lidar_top     != self.last_toggle_obstacles_lidar_top or \
            toggle_obstacles_lidar_bottom != self.last_toggle_obstacles_lidar_bottom or \
            toggle_obstacles_head_camera  != self.last_toggle_obstacles_head_camera:
            
            print("OBSTACLES - CHANGED STATUS.")

            self.activate_obstacles(obstacles_lidar_up=toggle_obstacles_lidar_top, obstacles_lidar_bottom=toggle_obstacles_lidar_bottom, \
                                    obstacles_camera_head=toggle_obstacles_head_camera)


        self.last_toggle_activate_objects_head =   toggle_activate_objects_head 
        self.last_toggle_activate_furniture_head = toggle_activate_furniture_head
        self.last_toggle_activate_shoes_head =     toggle_activate_shoes_head
        self.last_toggle_activate_objects_hand =   toggle_activate_objects_hand
        self.last_toggle_activate_furniture_hand = toggle_activate_furniture_hand
        self.last_toggle_activate_shoes_hand =     toggle_activate_shoes_hand

        self.last_toggle_pose_activate =       toggle_pose_activate
        self.last_toggle_pose_waving =         toggle_pose_waving
        self.last_toggle_pose_front_close =    toggle_pose_front_close
        self.last_toggle_pose_legs_visible =   toggle_pose_legs_visible
        self.last_toggle_pose_characteristcs = toggle_pose_characteristcs
        
        self.last_toggle_obstacles_lidar_top =    toggle_obstacles_lidar_top
        self.last_toggle_obstacles_lidar_bottom = toggle_obstacles_lidar_bottom
        self.last_toggle_obstacles_head_camera =  toggle_obstacles_head_camera

    def draw_pose_detections(self):

        MIN_DRAW_CONF = 0.5
        CIRCLE_RADIUS = 4
        MIN_KP_LINE_WIDTH = 3

        self.curr_detected_people = self.node.detected_people
        if self.toggle_pause_cams.getValue():
            used_detected_people = self.last_detected_people
        else:
            used_detected_people = self.curr_detected_people 
        self.last_detected_people = used_detected_people 

        if self.node.is_yolo_pose_comm:
           
            if len(used_detected_people.persons) > 0:
                # print("DETECTED PEOPLE:")
                pass

            for p in used_detected_people.persons:

                # room_and_furn_str = str(p.room_location + " (" + p.furniture_location + ")")
                # relative_coords_str = str("("+str(round(p.position_relative.x,2))+", "+str(round(p.position_relative.y,2))+", "+str(round(p.position_relative.z,2))+")")
                # print("id:", p.index, "|", str(int(round(p.confidence,2)*100)) + "%", "|", room_and_furn_str.ljust(22), "|", relative_coords_str.ljust(22), "|", "wave:", p.arm_raised, "|", "point:", p.pointing_at)

                PERSON_BB = pygame.Rect(int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio), int(p.box_width/2*self.camera_resize_ratio), int(p.box_height/2*self.camera_resize_ratio))
                pygame.draw.rect(self.WIN, self.RED, PERSON_BB, width=self.BB_WIDTH)

                if int(p.box_top_left_y) < 30: # depending on the height of the box, so it is either inside or outside
                    self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio), int(p.box_width/2*self.camera_resize_ratio), 30/2, self.RED, 85)
                    self.draw_text("id:"+str(p.index)+" "+str(int(round(p.confidence,2)*100))+"%", self.text_font_t, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio))
                else:
                    self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio-30/2), int(p.box_width/2*self.camera_resize_ratio), 30/2, self.RED, 85)
                    self.draw_text("id:"+str(p.index)+" "+str(int(round(p.confidence,2)*100))+"%", self.text_font_t, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio-30/2))
                
                self.draw_line_between_two_keypoints(p.kp_nose_conf, p.kp_nose_x, p.kp_nose_y, p.kp_eye_left_conf, p.kp_eye_left_x, p.kp_eye_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_nose_conf, p.kp_nose_x, p.kp_nose_y, p.kp_eye_right_conf, p.kp_eye_right_x, p.kp_eye_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_eye_left_conf, p.kp_eye_left_x, p.kp_eye_left_y, p.kp_ear_left_conf, p.kp_ear_left_x, p.kp_ear_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_eye_right_conf, p.kp_eye_right_x, p.kp_eye_right_y, p.kp_ear_right_conf, p.kp_ear_right_x, p.kp_ear_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_ear_left_conf, p.kp_ear_left_x, p.kp_ear_left_y, p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_ear_right_conf, p.kp_ear_right_x, p.kp_ear_right_y, p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                
                self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_elbow_left_conf, p.kp_elbow_left_x, p.kp_elbow_left_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_elbow_right_conf, p.kp_elbow_right_x, p.kp_elbow_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_elbow_left_conf, p.kp_elbow_left_x, p.kp_elbow_left_y, p.kp_wrist_left_conf, p.kp_wrist_left_x, p.kp_wrist_left_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_elbow_right_conf, p.kp_elbow_right_x, p.kp_elbow_right_y, p.kp_wrist_right_conf, p.kp_wrist_right_x, p.kp_wrist_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                
                self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                # self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                # self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                
                self.draw_line_between_two_keypoints(p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, p.kp_knee_left_conf, p.kp_knee_left_x, p.kp_knee_left_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, p.kp_knee_right_conf, p.kp_knee_right_x, p.kp_knee_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_knee_left_conf, p.kp_knee_left_x, p.kp_knee_left_y, p.kp_ankle_left_conf, p.kp_ankle_left_x, p.kp_ankle_left_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                self.draw_line_between_two_keypoints(p.kp_knee_right_conf, p.kp_knee_right_x, p.kp_knee_right_y, p.kp_ankle_right_conf, p.kp_ankle_right_x, p.kp_ankle_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                
                self.draw_circle_keypoint(p.kp_nose_conf,           p.kp_nose_x,            p.kp_nose_y,            self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_eye_left_conf,       p.kp_eye_left_x,        p.kp_eye_left_y,        self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_eye_right_conf,      p.kp_eye_right_x,       p.kp_eye_right_y,       self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_ear_left_conf,       p.kp_ear_left_x,        p.kp_ear_left_y,        self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_ear_right_conf,      p.kp_ear_right_x,       p.kp_ear_right_y,       self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
                
                self.draw_circle_keypoint(p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_elbow_left_conf,     p.kp_elbow_left_x,      p.kp_elbow_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_elbow_right_conf,    p.kp_elbow_right_x,     p.kp_elbow_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_wrist_left_conf,     p.kp_wrist_left_x,      p.kp_wrist_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_wrist_right_conf,    p.kp_wrist_right_x,     p.kp_wrist_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                
                self.draw_circle_keypoint(p.kp_hip_left_conf,       p.kp_hip_left_x,        p.kp_hip_left_y,        self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_hip_right_conf,      p.kp_hip_right_x,       p.kp_hip_right_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_knee_left_conf,      p.kp_knee_left_x,       p.kp_knee_left_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_knee_right_conf,     p.kp_knee_right_x,      p.kp_knee_right_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_ankle_left_conf,     p.kp_ankle_left_x,      p.kp_ankle_left_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                self.draw_circle_keypoint(p.kp_ankle_right_conf,    p.kp_ankle_right_x,     p.kp_ankle_right_y,     self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                
                self.check_face_for_characteristics(p, MIN_DRAW_CONF)

    def draw_circle_keypoint(self, conf, x, y, color, min_draw_conf, circle_radius):
        if conf > min_draw_conf:
            pygame.draw.circle(self.WIN, color, (self.cams_initial_width+(x/2)*self.camera_resize_ratio, self.cams_initial_height+(y/2)*self.camera_resize_ratio), radius=circle_radius, width=0)
                
    def draw_line_between_two_keypoints(self, conf1, x1, y1, conf2, x2, y2, color, min_draw_conf, min_kp_line_width):
        if conf1 > min_draw_conf and conf2 > min_draw_conf:  
            pygame.draw.line(self.WIN, color, (self.cams_initial_width+(x1/2)*self.camera_resize_ratio, self.cams_initial_height+(y1/2)*self.camera_resize_ratio), (self.cams_initial_width+(x2/2)*self.camera_resize_ratio, self.cams_initial_height+(y2/2)*self.camera_resize_ratio), min_kp_line_width)
    
    def check_face_for_characteristics(self, p: DetectedPerson, min_draw_conf):

        if p.gender != "None" or p.age_estimate != "None" or p.ethnicity != "None":

            if p.kp_shoulder_right_conf > min_draw_conf and p.kp_shoulder_left_conf > min_draw_conf and \
                p.kp_eye_right_conf > min_draw_conf and p.kp_eye_left_conf > min_draw_conf and p.kp_nose_conf > min_draw_conf:
            
                y1 = p.box_top_left_y
                y2 = max(p.kp_shoulder_right_y, p.kp_shoulder_left_y)
                y_height = y2-y1

                x1 = min(p.kp_shoulder_right_x, p.kp_shoulder_left_x, p.kp_nose_x, p.kp_eye_right_x, p.kp_eye_left_x)
                x2 = max(p.kp_shoulder_right_x, p.kp_shoulder_left_x, p.kp_nose_x, p.kp_eye_right_x, p.kp_eye_left_x)
                x_width = x2-x1
            
                self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+30/2), int(p.box_width/2*self.camera_resize_ratio), 6*(30/2), self.RED, 85)
                self.draw_text(str(p.gender), self.text_font, self.BLACK,          int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+1*(30/2)))
                self.draw_text(str(p.ethnicity), self.text_font, self.BLACK,       int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+2*(30/2)))
                self.draw_text(str(p.age_estimate), self.text_font, self.BLACK,    int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+3*(30/2)))
                self.draw_text(str(round(p.height,2)), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+4*(30/2)))
                self.draw_text(str(p.shirt_color), self.text_font, self.BLACK,     int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+5*(30/2)))
                self.draw_text(str(p.pants_color), self.text_font, self.BLACK,     int(self.cams_initial_width+(p.box_top_left_x/2)*self.camera_resize_ratio), int(self.cams_initial_height+(p.box_top_left_y/2)*self.camera_resize_ratio+6*(30/2)))
            
    def draw_object_detections(self):

        self.curr_detected_objects = self.node.detected_objects
        self.curr_detected_objects_hand = self.node.detected_objects_hand

        if self.toggle_pause_cams.getValue():
            used_detected_objects = self.last_detected_objects
            used_detected_objects_hand = self.last_detected_objects_hand
        else:
            used_detected_objects = self.curr_detected_objects 
            used_detected_objects_hand = self.curr_detected_objects_hand

        self.last_detected_objects = used_detected_objects 
        self.last_detected_objects_hand = used_detected_objects_hand

        if self.node.is_yolo_obj_head_comm:
            if len(used_detected_objects.objects) > 0: # or len(used_detected_shoes.objects) > 0 or len(used_detected_furniture.objects) > 0:
                # print("DETECTED OBJECTS HEAD:")
                pass
            self.draw_object_bounding_boxes(used_detected_objects, "head")

        if self.node.is_yolo_obj_hand_comm:
            if len(used_detected_objects_hand.objects) > 0: # or len(used_detected_shoes_hand.objects) > 0 or len(used_detected_furniture_hand.objects) > 0:
                # print("DETECTED OBJECTS HAND:")
                pass
            self.draw_object_bounding_boxes(used_detected_objects_hand, "hand")

    def draw_object_bounding_boxes(self, objects, head_or_hand):

        if head_or_hand.lower() == "head":
            window_cam_height = self.cams_initial_height
        else: # "hand"
            window_cam_height = self.cam_height_+2*self.cams_initial_height

        if len(objects.objects) > 0:
            # print("DETECTED OBJECTS ("+head_or_hand.lower()+"):")
            pass
        
        for o in objects.objects:
           
            if not o.mask.point: # if object does not have a mask, we show bounding box
                # name_and_cat_str = str(o.object_name + " (" + o.object_class + ")")
                # room_and_furn_str = str(o.room_location + " (" + o.furniture_location + ")")
                # relative_coords_str = str("("+str(round(o.position_relative.x,2))+", "+str(round(o.position_relative.y,2))+", "+str(round(o.position_relative.z,2))+")")
                # print("id:", o.index, "|", str(int(round(o.confidence,2)*100)) + "%", "|", name_and_cat_str.ljust(22) ,"|", room_and_furn_str.ljust(22), "|", relative_coords_str)
                bb_color = self.object_class_to_bb_color(o.object_class)
                OBJECT_BB = pygame.Rect(int(self.cams_initial_width+(o.box_top_left_x/2)*self.camera_resize_ratio), int(window_cam_height+(o.box_top_left_y/2)*self.camera_resize_ratio), int(o.box_width/2*self.camera_resize_ratio), int(o.box_height/2*self.camera_resize_ratio))
                pygame.draw.rect(self.WIN, bb_color, OBJECT_BB, width=self.BB_WIDTH)
            
            else: # if object has mask, we should segmentation mask

                temp_mask = []
                for p in o.mask.point: # converts received mask into local coordinates and numpy array
                    p_list = []
                    p_list.append(int(self.cams_initial_width+(p.x/2)*self.camera_resize_ratio))
                    p_list.append(int(window_cam_height+(p.y/2)*self.camera_resize_ratio))
                    temp_mask.append(p_list)

                np_mask = np.array(temp_mask)
                bb_color = self.object_class_to_bb_color(o.object_class)
                pygame.draw.polygon(self.WIN, bb_color, np_mask, self.BB_WIDTH) # outside line (darker)
                self.draw_polygon_alpha(self.WIN, bb_color+(128,), np_mask) # inside fill with transparecny

        # this is separated into two for loops so that no bounding box overlaps with the name of the object, making the name unreadable 
        for o in objects.objects:

            # text = str(o.object_name)
            text = str(o.object_name)+" ("+str(o.index)+")"
            text_width, text_height = self.text_font_t.size(text)
            bb_color = self.object_class_to_bb_color(o.object_class)

            if int(o.box_top_left_y) < 30: # depending on the height of the box, so it is either inside or outside
                self.draw_transparent_rect(int(self.cams_initial_width+(o.box_top_left_x/2)*self.camera_resize_ratio), int(window_cam_height+(o.box_top_left_y/2)*self.camera_resize_ratio), text_width, text_height, bb_color, 255)
                self.draw_text(text, self.text_font_t, self.BLACK, int(self.cams_initial_width+(o.box_top_left_x/2)*self.camera_resize_ratio), int(window_cam_height+(o.box_top_left_y/2)*self.camera_resize_ratio))
            else:
                self.draw_transparent_rect(int(self.cams_initial_width+(o.box_top_left_x/2)*self.camera_resize_ratio), int(window_cam_height+(o.box_top_left_y/2)*self.camera_resize_ratio-30/2), text_width, text_height, bb_color, 255)
                self.draw_text(text, self.text_font_t, self.BLACK, int(self.cams_initial_width+(o.box_top_left_x/2)*self.camera_resize_ratio), int(window_cam_height+(o.box_top_left_y/2)*self.camera_resize_ratio-30/2))

    def draw_polygon_alpha(self, surface, color, points):
        lx, ly = zip(*points)
        min_x, min_y, max_x, max_y = min(lx), min(ly), max(lx), max(ly)
        target_rect = pygame.Rect(min_x, min_y, max_x-min_x, max_y-min_y)
        shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
        pygame.draw.polygon(shape_surf, color, [(x-min_x, y-min_y) for x,y in points])
        surface.blit(shape_surf, target_rect)

    def object_class_to_bb_color(self, object_class):

        if object_class == "Cleaning Supplies":
            bb_color = self.YELLOW
        elif object_class == "Drinks":
            bb_color = self.PURPLE
        elif object_class == "Foods":
            bb_color = self.BLUE_L
        elif object_class == "Fruits":
            bb_color = self.ORANGE
        elif object_class == "Toys":
            bb_color = self.BLUE
        elif object_class == "Snacks":
            bb_color = self.MAGENTA
        elif object_class == "Dishes":
            bb_color = self.GREY
        elif object_class == "Footwear":
            bb_color = self.RED
        elif object_class == "Furniture":
            bb_color = self.GREEN
        else:
            bb_color = self.BLACK

        return bb_color

    def draw_tracking(self):

        self.curr_tracking = self.node.tracking_mask
        if self.toggle_pause_cams.getValue():
            used_tracking = self.last_tracking
        else:
            used_tracking = self.curr_tracking 
        self.last_tracking = used_tracking

        window_cam_height = self.cams_initial_height
        
        if self.node.is_tracking_comm:
           
            for used_point in used_tracking.mask.masks:
                
                temp_mask = []
                for p in used_point.point: # converts received mask into local coordinates and numpy array
                    p_list = []
                    p_list.append(int(self.cams_initial_width+(p.x/2)*self.camera_resize_ratio))
                    p_list.append(int(window_cam_height+(p.y/2)*self.camera_resize_ratio))
                    temp_mask.append(p_list)
                
                np_mask = np.array(temp_mask)
                # print(len(np_mask))

                if len(np_mask) > 2:
                    bb_color = self.WHITE
                    pygame.draw.polygon(self.WIN, bb_color, np_mask, self.BB_WIDTH) # outside line (darker)
                    self.draw_polygon_alpha(self.WIN, bb_color+(128,), np_mask) # inside fill with transparecny

            """ # old method that used a binary mask rather than a detection mask           
            binary_mask = self.br.imgmsg_to_cv2(used_tracking.binary_mask, desired_encoding='mono8')
            binary_mask = cv2.resize(binary_mask, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
            # Convert the image to RGB (OpenCV loads as BGR by default)

            binary_mask = np.transpose(binary_mask)

            mask_surface = pygame.surfarray.make_surface(np.stack([binary_mask] * 3, axis=-1))  # Convert grayscale to RGB
            mask_surface.set_colorkey((0, 0, 0))  # Make black pixels transparent

            # Apply a color to white pixels
            mask_color = (0, 255, 0, 128)  # Green
            mask_surface.fill(mask_color, special_flags=pygame.BLEND_RGB_MULT)
            
            self.WIN.blit(mask_surface, (self.cams_initial_width, self.cams_initial_height))
            """

            self.draw_circle_keypoint(1.0, used_tracking.centroid.x, used_tracking.centroid.y, self.BLACK, 0.0, 9)
            self.draw_circle_keypoint(1.0, used_tracking.centroid.x, used_tracking.centroid.y, self.WHITE, 0.0, 5)
        
    def check_record_data(self):
        
        if self.toggle_record.getValue() and not self.last_toggle_record:
            print("STARTED RECORDING")
            self.current_datetime = str(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))       
            self.video = cv2.VideoWriter(self.home+self.save_recordings_midpath+self.current_datetime+".avi", self.fourcc, self.FPS, (self.WIDTH, self.HEIGHT))
            self.WIN = pygame.display.set_mode((self.WIDTH, self.HEIGHT), 0)

        if not self.toggle_record.getValue() and self.last_toggle_record:
            print("STOPPED RECORDING")
            self.video.release()
            self.WIN = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)

        if self.toggle_record.getValue():
            # transform the pixels to the format used by open-cv
            pixels = cv2.rotate(pygame.surfarray.pixels3d(self.WIN), cv2.ROTATE_90_CLOCKWISE)
            pixels = cv2.flip(pixels, 1)
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)

            # write the frame
            self.video.write(pixels)

        self.last_toggle_record = self.toggle_record.getValue()

    def adjust_window_size(self):

        # default values are: 
        # self.WIDTH, self.HEIGHT = 1387, 752
        # self.cam_width_ = 640
        # self.cam_height_ = 360

        # Get current window size
        self.WIDTH, self.HEIGHT = self.WIN.get_size()
        # print(self.WIDTH, self.HEIGHT)

        custom_height = self.HEIGHT - 752
        self.cam_height_ = int(360 + custom_height/2)
        cam_height_ratio = self.cam_height_/360
        self.cam_width_ = int(640*cam_height_ratio)
        self.camera_resize_ratio = cam_height_ratio
        # print(self.cam_width_, self.cam_height_, self.camera_resize_ratio )
        
        # adjust activate toggle positions
        self.toggle_activate_objects_head.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height)
        self.toggle_activate_furniture_head.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+90)
        self.toggle_activate_shoes_head.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+192)
        self.toggle_activate_objects_hand.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+298)
        self.toggle_activate_furniture_hand.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+390)
        self.toggle_activate_shoes_hand.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+492)

        self.toggle_pose_activate.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height)
        self.toggle_pose_waving.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+93)
        self.toggle_pose_front_close.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+182)
        self.toggle_pose_legs_visible.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+302)
        self.toggle_pose_characteristcs.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+427)
        
        self.toggle_obstacles_lidar_top.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height)
        self.toggle_obstacles_lidar_bottom.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+100)
        self.toggle_obstacles_head_camera.setX(self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+233)

        self.button_zoom_in.setX(self.map_init_width+self.MAP_SIDE-(5.5*self.button_size*self.camera_resize_ratio))
        self.button_zoom_out.setX(self.map_init_width+self.MAP_SIDE-(4.5*self.button_size*self.camera_resize_ratio))
        self.button_shift_up.setX(self.map_init_width+self.MAP_SIDE-(2*self.button_size*self.camera_resize_ratio))
        self.button_shift_down.setX(self.map_init_width+self.MAP_SIDE-(2*self.button_size*self.camera_resize_ratio))
        self.button_shift_left.setX(self.map_init_width+self.MAP_SIDE-(3*self.button_size*self.camera_resize_ratio))
        self.button_shift_right.setX(self.map_init_width+self.MAP_SIDE-(1*self.button_size*self.camera_resize_ratio))

        self.button_zoom_in.setY(self.map_init_height-self.button_size*self.camera_resize_ratio)
        self.button_zoom_out.setY(self.map_init_height-self.button_size*self.camera_resize_ratio)
        self.button_shift_up.setY(self.map_init_height-2*self.button_size*self.camera_resize_ratio)
        self.button_shift_down.setY(self.map_init_height-self.button_size*self.camera_resize_ratio)
        self.button_shift_left.setY(self.map_init_height-self.button_size*self.camera_resize_ratio)
        self.button_shift_right.setY(self.map_init_height-self.button_size*self.camera_resize_ratio)

        self.button_zoom_in.setHeight(self.button_size*self.camera_resize_ratio)
        self.button_zoom_out.setHeight(self.button_size*self.camera_resize_ratio)
        self.button_shift_up.setHeight(self.button_size*self.camera_resize_ratio)
        self.button_shift_down.setHeight(self.button_size*self.camera_resize_ratio)
        self.button_shift_left.setHeight(self.button_size*self.camera_resize_ratio)
        self.button_shift_right.setHeight(self.button_size*self.camera_resize_ratio)

        self.button_zoom_in.setWidth(self.button_size*self.camera_resize_ratio)
        self.button_zoom_out.setWidth(self.button_size*self.camera_resize_ratio)
        self.button_shift_up.setWidth(self.button_size*self.camera_resize_ratio)
        self.button_shift_down.setWidth(self.button_size*self.camera_resize_ratio)
        self.button_shift_left.setWidth(self.button_size*self.camera_resize_ratio)
        self.button_shift_right.setWidth(self.button_size*self.camera_resize_ratio)
        
    def draw_map(self):
        
        self.MAP_SIDE = int(self.HEIGHT - 260 - 12)
        # print(self.HEIGHT, self.MAP_SIDE)

        self.xc = self.MAP_SIDE
        self.yc = self.MAP_SIDE
        self.xx_shift = int(self.MAP_SIDE/2 + self.MAP_ADJUST_X*self.MAP_SIDE/20) # (self.MAP_SIDE*self.MAP_ADJUST_X))
        self.yy_shift = int(self.MAP_SIDE/2 + self.MAP_ADJUST_Y*self.MAP_SIDE/20) # (self.MAP_SIDE*self.MAP_ADJUST_Y))
        self.xc_adj = self.xc - self.xx_shift
        self.yc_adj = self.yc - self.yy_shift

        self.map_init_width = int(self.cams_initial_width+self.cam_width_+self.cams_initial_height)
        self.map_init_height = 260

        # visual configs:
        neck_visual_lines_length = 1.0
        detected_person_radius = 0.2
        detected_object_radius = 0.08

        MAP_BB = pygame.Rect(self.map_init_width, self.map_init_height, self.MAP_SIDE, self.MAP_SIDE)
        pygame.draw.rect(self.WIN, self.WHITE, MAP_BB, width=self.BB_WIDTH)

        ### DRAWS REFERENCE 1 METER LINES ###
        for i in range(20):
            # 1 meter lines horizontal and vertical
            if i == 0:
                color = self.RED
            else:
                color = self.GREY
            
            pygame.draw.line(self.WIN, color, (int(self.map_init_width+self.xc_adj+(self.MAP_SIDE*(i/(10*self.MAP_SCALE)))), self.map_init_height), (int(self.map_init_width+self.xc_adj+(self.MAP_SIDE*(i/(10*self.MAP_SCALE)))), self.map_init_height+self.MAP_SIDE-1), 1)
            pygame.draw.line(self.WIN, color, (int(self.map_init_width+self.xc_adj-(self.MAP_SIDE*(i/(10*self.MAP_SCALE)))), self.map_init_height), (int(self.map_init_width+self.xc_adj-(self.MAP_SIDE*(i/(10*self.MAP_SCALE)))), self.map_init_height+self.MAP_SIDE-1), 1)
            pygame.draw.line(self.WIN, color, (self.map_init_width, int(self.map_init_height+self.yc_adj+(self.MAP_SIDE*(i/(10*self.MAP_SCALE))))), (self.map_init_width+self.MAP_SIDE-1, int(self.map_init_height+self.yc_adj+(self.MAP_SIDE*(i/(10*self.MAP_SCALE))))), 1)
            pygame.draw.line(self.WIN, color, (self.map_init_width, int(self.map_init_height+self.yc_adj-(self.MAP_SIDE*(i/(10*self.MAP_SCALE))))), (self.map_init_width+self.MAP_SIDE-1, int(self.map_init_height+self.yc_adj-(self.MAP_SIDE*(i/(10*self.MAP_SCALE))))), 1)
    
        ### DRAWS THE HOUSE FURNITURE ###
        for furniture in self.house_furniture:
            temp_rect = pygame.Rect(self.coords_to_map(furniture['top_left_coords'][0], furniture['top_left_coords'][1])[0], \
                                    self.coords_to_map(furniture['top_left_coords'][0], furniture['top_left_coords'][1])[1], \
                                    abs(self.coords_to_map(furniture['top_left_coords'][0], furniture['top_left_coords'][1])[0] - self.coords_to_map(furniture['bot_right_coords'][0], furniture['bot_right_coords'][1])[0]), \
                                    abs(self.coords_to_map(furniture['top_left_coords'][0], furniture['top_left_coords'][1])[1] - self.coords_to_map(furniture['bot_right_coords'][0], furniture['bot_right_coords'][1])[1]))
            pygame.draw.rect(self.WIN, self.GREY, temp_rect, width=0)
    
        ### DRAWS THE HOUSE WALLS ###
        for room in self.house_rooms:
            temp_rect = pygame.Rect(self.coords_to_map(room['top_left_coords'][0], room['top_left_coords'][1])[0], \
                                    self.coords_to_map(room['top_left_coords'][0], room['top_left_coords'][1])[1], \
                                    abs(self.coords_to_map(room['top_left_coords'][0], room['top_left_coords'][1])[0] - self.coords_to_map(room['bot_right_coords'][0], room['bot_right_coords'][1])[0]), \
                                    abs(self.coords_to_map(room['top_left_coords'][0], room['top_left_coords'][1])[1] - self.coords_to_map(room['bot_right_coords'][0], room['bot_right_coords'][1])[1]))
            pygame.draw.rect(self.WIN, self.WHITE, temp_rect, width=3)
        
        ### DRAWS NAVIGATION LOCATIONS ###
        if self.show_navigation_locations:
            for furniture in self.house_furniture:

                furniture_center_map_coords = self.coords_to_map((furniture['top_left_coords'][0] + furniture['bot_right_coords'][0])/2, (furniture['top_left_coords'][1] + furniture['bot_right_coords'][1])/2)
                pygame.draw.circle(self.WIN, self.ORANGE, furniture_center_map_coords, radius=self.size_to_map(self.robot_radius/2), width=0)
                self.draw_text(str(furniture['name']), self.text_map_font, self.ORANGE, furniture_center_map_coords[0]-(3*len(str(furniture['name']))), furniture_center_map_coords[1]+13)

                furniture_nav_map_coords = self.coords_to_map(furniture['nav_coords'][0], furniture['nav_coords'][1])
                pygame.draw.circle(self.WIN, self.GREEN, furniture_nav_map_coords, radius=self.size_to_map(self.robot_radius/2), width=0)
                self.draw_text(str(furniture['name']), self.text_map_font, self.GREEN, furniture_nav_map_coords[0]-(3*len(str(furniture['name']))), furniture_nav_map_coords[1]+13)

            for rooms in self.house_rooms:

                rooms_nav_map_coords = self.coords_to_map(rooms['nav_coords'][0], rooms['nav_coords'][1])
                pygame.draw.circle(self.WIN, self.BLUE_L, rooms_nav_map_coords, radius=self.size_to_map(self.robot_radius/2), width=0)
                self.draw_text(str(rooms['name']), self.text_map_font, self.BLUE_L, rooms_nav_map_coords[0]-(3*len(str(rooms['name']))), rooms_nav_map_coords[1]+13)


        ### DRAW ROBOT
        pygame.draw.circle(self.WIN, self.BLUE_L, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), radius=self.size_to_map(self.robot_radius), width=0)
        
        front_of_robot_point = (self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[0]-(self.size_to_map(self.robot_radius)*math.cos(-self.node.robot_pose.theta + math.pi/2)), \
                                self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[1]-(self.size_to_map(self.robot_radius)*math.sin(-self.node.robot_pose.theta + math.pi/2)))
        left_of_robot_point =  (self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[0]-(self.size_to_map(self.robot_radius)*math.cos(-self.node.robot_pose.theta)), \
                                self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[1]-(self.size_to_map(self.robot_radius)*math.sin(-self.node.robot_pose.theta)))
        right_of_robot_point = (self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[0]+(self.size_to_map(self.robot_radius)*math.cos(-self.node.robot_pose.theta)), \
                                self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[1]+(self.size_to_map(self.robot_radius)*math.sin(-self.node.robot_pose.theta)))

        pygame.draw.line(self.WIN, self.BLUE, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), front_of_robot_point, int((self.MAP_SIDE*(0.05/10.0*(1/self.MAP_SCALE)))))
        pygame.draw.line(self.WIN, self.BLUE, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), left_of_robot_point,  int((self.MAP_SIDE*(0.05/10.0*(1/self.MAP_SCALE)))))
        pygame.draw.line(self.WIN, self.BLUE, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), right_of_robot_point, int((self.MAP_SIDE*(0.05/10.0*(1/self.MAP_SCALE)))))
            

        ### DRAW ROBOT PAST LOCATIONS (MOVEMENT)
        for i in range(len(self.node.all_pos_x_val)):
            pygame.draw.circle(self.WIN, self.YELLOW, self.coords_to_map(self.node.all_pos_x_val[i], self.node.all_pos_y_val[i]), radius=1, width=0)


        ### NECK DIRECTION, CAMERA FOV
        left_vision_range_limit_point =  (self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[0]-(self.size_to_map(neck_visual_lines_length)*math.cos(-self.node.robot_pose.theta - self.node.neck_pan + math.pi/2 - math.pi/4)), \
                                          self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[1]-(self.size_to_map(neck_visual_lines_length)*math.sin(-self.node.robot_pose.theta - self.node.neck_pan + math.pi/2 - math.pi/4)))
        right_vision_range_limit_point = (self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[0]-(self.size_to_map(neck_visual_lines_length)*math.cos(-self.node.robot_pose.theta - self.node.neck_pan + math.pi/2 + math.pi/4)), \
                                          self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y)[1]-(self.size_to_map(neck_visual_lines_length)*math.sin(-self.node.robot_pose.theta - self.node.neck_pan + math.pi/2 + math.pi/4)))

        pygame.draw.line(self.WIN, self.BLUE, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), left_vision_range_limit_point, int((self.MAP_SIDE*(0.05/10.0*(1/self.MAP_SCALE)))))
        pygame.draw.line(self.WIN, self.BLUE, self.coords_to_map(self.node.robot_pose.x, self.node.robot_pose.y), right_vision_range_limit_point, int((self.MAP_SIDE*(0.05/10.0*(1/self.MAP_SCALE)))))


        ### NAVIGATION TARGETS
        if self.node.is_navigating:
            if self.node.navigation.move_or_rotate == "move" or self.node.navigation.move_or_rotate == "rotate":
                pygame.draw.circle(self.WIN, self.GREEN, self.coords_to_map(self.node.navigation.target_coordinates.x, self.node.navigation.target_coordinates.y), radius=self.size_to_map(self.robot_radius/2), width=0)
                pygame.draw.circle(self.WIN, self.GREEN, self.coords_to_map(self.node.navigation.target_coordinates.x, self.node.navigation.target_coordinates.y), radius=self.size_to_map(self.node.navigation.reached_radius), width=1)


        ### OBSTACLES POINTS (LIDAR, Depth Head Camera and Final Obstacles Fusion)
        for points in self.node.lidar_obstacle_points:
            # pygame.draw.circle(self.WIN, self.RED, self.coords_to_map(self.node.robot_pose.x+points.x, self.node.robot_pose.y+points.y), radius=1, width=0)
            pygame.draw.circle(self.WIN, self.RED, self.coords_to_map(points.x, points.y), radius=1, width=0)

        for points in self.node.lidar_bottom_obstacle_points:
            # pygame.draw.circle(self.WIN, self.RED, self.coords_to_map(self.node.robot_pose.x+points.x, self.node.robot_pose.y+points.y), radius=1, width=0)
            pygame.draw.circle(self.WIN, self.MAGENTA, self.coords_to_map(points.x, points.y), radius=1, width=0)

        for points in self.node.camera_obstacle_points:
            pygame.draw.circle(self.WIN, self.BLUE, self.coords_to_map(points.x, points.y), radius=2, width=0)

        for points in self.node.final_obstacle_points:

            # calculate the absolute position according to the robot localisation
            dist_obj = math.sqrt(points.x**2 + points.y**2)

            angle_obj = math.atan2(points.x, points.y)
            theta_aux = math.pi/2 - (angle_obj - self.node.robot_pose.theta)

            target = Point()
            target.x = dist_obj * math.cos(theta_aux) + self.node.robot_pose.x
            target.y = dist_obj * math.sin(theta_aux) + self.node.robot_pose.y
            target.z = points.z

            pygame.draw.circle(self.WIN, self.ORANGE, self.coords_to_map(target.x, target.y), radius=2, width=0)
            

        ### PERSON DETECTED
        for person in self.node.detected_people.persons:
            pygame.draw.circle(self.WIN, self.CYAN, self.coords_to_map(person.position_absolute.x, person.position_absolute.y), radius=self.size_to_map(detected_person_radius), width=0)

        ### SEARCH FOR PERSON
        if self.node.new_search_for_person:
            print("DETECTED SEARCH FOR PERSON:")

        for person in self.node.search_for_person.persons:
            pygame.draw.circle(self.WIN, self.MAGENTA, self.coords_to_map(person.position_absolute.x, person.position_absolute.y), radius=self.size_to_map(detected_person_radius), width=0)

            if self.node.new_search_for_person:            
                room_and_furn_str = str(person.room_location + " (" + person.furniture_location + ")")
                relative_coords_str = str("("+str(round(person.position_relative.x,2))+", "+str(round(person.position_relative.y,2))+", "+str(round(person.position_relative.z,2))+")")
                print("id:", person.index, "|", str(int(round(person.confidence,2)*100)) + "%", "|", room_and_furn_str.ljust(22), "|", relative_coords_str.ljust(22), "|", "wave:", person.arm_raised, "|", "point:", person.pointing_at)

        if self.node.new_search_for_person:
            self.node.new_search_for_person = False
        
        ### OBJECT_DETECTED
        for object in self.node.detected_objects.objects:
            temp_rect = pygame.Rect(self.coords_to_map(object.position_absolute.x, object.position_absolute.y)[0]-self.size_to_map(detected_object_radius), \
                                    self.coords_to_map(object.position_absolute.x, object.position_absolute.y)[1]-self.size_to_map(detected_object_radius), \
                                    2*self.size_to_map(detected_object_radius), 2*self.size_to_map(detected_object_radius))
            pygame.draw.rect(self.WIN, self.CYAN, temp_rect, width=0)

        ### SEARCH FOR OBJECT
        if self.node.new_search_for_object:
            print("DETECTED SEARCH FOR OBJECT:")

        for object in self.node.search_for_object.objects:
            temp_rect = pygame.Rect(self.coords_to_map(object.position_absolute.x, object.position_absolute.y)[0]-self.size_to_map(detected_object_radius), \
                                    self.coords_to_map(object.position_absolute.x, object.position_absolute.y)[1]-self.size_to_map(detected_object_radius), \
                                    2*self.size_to_map(detected_object_radius), 2*self.size_to_map(detected_object_radius))
            pygame.draw.rect(self.WIN, self.MAGENTA, temp_rect, width=0)

            if self.node.new_search_for_object:
                name_and_cat_str = str(object.object_name + " (" + object.object_class + ")")
                room_and_furn_str = str(object.room_location + " (" + object.furniture_location + ")")
                relative_coords_str = str("("+str(round(object.position_relative.x,2))+", "+str(round(object.position_relative.y,2))+", "+str(round(object.position_relative.z,2))+")")
                print("id:", object.index, "|", str(int(round(object.confidence,2)*100)) + "%", "|", name_and_cat_str.ljust(22) ,"|", room_and_furn_str.ljust(22), "|", relative_coords_str)
         
        if self.node.new_search_for_object:
            self.node.new_search_for_object = False    

        ### TRACKING
        pygame.draw.circle(self.WIN, self.WHITE, self.coords_to_map(self.node.tracking_mask.position_absolute.x, self.node.tracking_mask.position_absolute.y), radius=self.size_to_map(detected_person_radius), width=0)

        ### FINAL DRAWINGS (for clearing remaining of image without checking every drawing (just draw and then clear everything outside the the map slot))
        self.WIDTH, self.HEIGHT = self.WIN.get_size()
        
        CLEAR_SCREEN_POST_MAP_DRAWINGS = pygame.Rect(0, 0, self.WIDTH, self.map_init_height)
        pygame.draw.rect(self.WIN, self.BLACK, CLEAR_SCREEN_POST_MAP_DRAWINGS, width=0)
        CLEAR_SCREEN_POST_MAP_DRAWINGS = pygame.Rect(0, 0, self.map_init_width, self.HEIGHT)
        pygame.draw.rect(self.WIN, self.BLACK, CLEAR_SCREEN_POST_MAP_DRAWINGS, width=0)

        CLEAR_SCREEN_POST_MAP_DRAWINGS = pygame.Rect(0, self.map_init_height+self.MAP_SIDE, self.WIDTH, self.HEIGHT-(self.map_init_height+self.MAP_SIDE))
        pygame.draw.rect(self.WIN, self.BLACK, CLEAR_SCREEN_POST_MAP_DRAWINGS, width=0)
        CLEAR_SCREEN_POST_MAP_DRAWINGS = pygame.Rect(self.map_init_width+self.MAP_SIDE, 0, self.WIDTH-(self.map_init_width+self.MAP_SIDE), self.HEIGHT)
        pygame.draw.rect(self.WIN, self.BLACK, CLEAR_SCREEN_POST_MAP_DRAWINGS, width=0)
        
        pygame.draw.rect(self.WIN, self.WHITE, MAP_BB, width=self.BB_WIDTH)

    def draw_battery(self):

        battery_colour = self.WHITE

        if self.node.battery_voltage > 37.0:
            battery_colour = self.GREEN
        elif self.node.battery_voltage > 35.0:
            battery_colour = self.YELLOW
        elif self.node.battery_voltage > 10.0:
            battery_colour = self.RED

        self.draw_text("Battery: "+str(self.node.battery_voltage)+"V", self.text_font_t, battery_colour, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(self.first_pos_h+10.0-0.9))

    def coords_to_map(self, xx, yy):
        return (self.map_init_width+self.xc_adj+self.MAP_SIDE*(-yy/(10*self.MAP_SCALE)), self.map_init_height+self.yc_adj-self.MAP_SIDE*(xx/(10*self.MAP_SCALE)))

    def size_to_map(self, size): # convert physical real size into map size (meters)
        return self.MAP_SIDE*((size)/10.0*(1/self.MAP_SCALE))

    def main(self):

        clock = pygame.time.Clock()
        run = True
        while run:
            clock.tick(self.FPS)
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    run = False
                    pygame.quit()
                    print("OVER")
                    os._exit(0)  # Force exit of the entire application

                # Check if a key is pressed
                if event.type == pygame.KEYDOWN:
                    # if event.key == pygame.K_SPACE:
                    #     print("Space key pressed!")
                    if event.key == pygame.K_LEFT:
                        # print("Left arrow key pressed!")
                        self.button_shift_left_function()
                    if event.key == pygame.K_RIGHT:
                        # print("Right arrow key pressed!")
                        self.button_shift_right_function()
                    if event.key == pygame.K_UP:
                        # print("Up arrow key pressed!")
                        self.button_shift_up_function()
                    if event.key == pygame.K_DOWN:
                        # print("Down arrow key pressed!")
                        self.button_shift_down_function()

                    if event.key == pygame.K_PLUS:
                        # print("PLUS key pressed!")
                        self.button_zoom_in_function()
                    if event.key == pygame.K_MINUS:
                        # print("MINUS key pressed!")
                        self.button_zoom_out_function()

                    if event.key == pygame.K_m:
                        self.show_navigation_locations = not self.show_navigation_locations

                    if event.key == pygame.K_w:
                        self.node.robot_pose.x+=0.1
                    if event.key == pygame.K_s:
                        self.node.robot_pose.x-=0.1
                    if event.key == pygame.K_a:
                        self.node.robot_pose.y+=0.1
                    if event.key == pygame.K_d:
                        self.node.robot_pose.y-=0.1
                    if event.key == pygame.K_q:
                        self.node.robot_pose.theta+=math.radians(15)
                    if event.key == pygame.K_e:
                        self.node.robot_pose.theta-=math.radians(15)

                    if event.key == pygame.K_c:
                        self.node.all_pos_x_val.clear()
                        self.node.all_pos_y_val.clear()

            self.WIN.fill((0, 0, 0))
            self.draw_map()
            self.adjust_window_size()  
            self.draw_nodes_check()
            self.draw_battery()
            self.draw_cameras()
            self.draw_activates()
            self.draw_pose_detections()
            self.draw_object_detections()
            self.draw_tracking()
            
            pygame_widgets.update(events)
            pygame.display.update()

            self.check_record_data()
