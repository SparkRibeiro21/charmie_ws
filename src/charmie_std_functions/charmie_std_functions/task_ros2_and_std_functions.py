# import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, Float32, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Vector3, Point, PoseStamped, Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav2_msgs.srv import ClearEntireCostmap
from realsense2_camera_msgs.msg import RGBD
from charmie_interfaces.msg import DetectedPerson, DetectedObject, TarNavSDNL, BoundingBox, ListOfDetectedPerson, ListOfDetectedObject, \
    ArmController, GamepadController, ListOfStrings, ListOfPoints, TrackingMask, ButtonsLowLevel, VCCsLowLevel, TorsoPosition, \
    TaskStatesInfo, RadarData
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, \
    SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, Trigger, SetFace, SetFloat, \
    NodesUsed, ContinuousGetAudio, SetRGB, SetTorso, ActivateBool, GetLLMGPSR, GetLLMDemo, GetLLMConfirmCommand, TrackContinuous, \
    ActivateTracking, SetPoseWithCovarianceStamped, SetInt, GetFaceTouchscreenMenu, SetFaceTouchscreenMenu, GetSoundClassification, \
    GetSoundClassificationContinuous, GetMinRadarDistance, ActivateYoloWorld
from charmie_interfaces.action import AdjustNavigationAngle, AdjustNavigationOmnidirectional, AdjustNavigationObstacles

from charmie_point_cloud.point_cloud_class import PointCloud

import os
import cv2 
import time
from cv_bridge import CvBridge
import math
import numpy as np
from pathlib import Path
from datetime import datetime
import random
import json
import face_recognition
from skimage.metrics import structural_similarity as ssim

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class ROS2TaskNode(Node):

    def __init__(self, ros2_modules):
        super().__init__("ROS2TaskCHARMIE")
        self.get_logger().info("Initialised CHARMIE ROS2Task Node")

        ### ROS2 Parameters ###
        # DEMO mode for task_demonstrations allows operator to control the robot task selection with the controller  
        self.declare_parameter("DEMO", False) 
        self.DEMO_OPTION = self.get_parameter("DEMO").value
        
        self.ros2_modules = ros2_modules

        # path to save detected people in search for person
        self.home = str(Path.home())
        custom_face_midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        configuration_files_midpath = "/charmie_ws/src/configuration_files/"
        self.complete_path_custom_face = self.home+'/'+custom_face_midpath+'/'

        # Open all configuration files
        try:
            with open(self.home + configuration_files_midpath + 'objects.json', encoding='utf-8') as json_file:
                self.objects_file = json.load(json_file)
            # print(self.objects_file)
            with open(self.home + configuration_files_midpath + 'objects_classes.json', encoding='utf-8') as json_file:
                self.objects_classes_file = json.load(json_file)
            # print(self.objects_classes_file)
            with open(self.home + configuration_files_midpath + 'rooms.json', encoding='utf-8') as json_file:
                self.rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.home + configuration_files_midpath + 'furniture.json', encoding='utf-8') as json_file:
                self.furniture = json.load(json_file)
            # print(self.house_furniture)
            with open(self.home + configuration_files_midpath + 'names.json', encoding='utf-8') as json_file:
                self.names = json.load(json_file)
            # print(self.names)
            self.get_logger().info("Successfully imported data from json configuration files.")
        except:
            self.get_logger().error("Could NOT import data from json configuration files.")

        ### Class ###
        self.point_cloud = PointCloud()

        ### TOPICS ###
        # Intel Realsense Subscribers (RGBD) Head and Hand Cameras
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        # Orbbec Camera (Base)
        self.color_image_base_subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.get_color_image_base_callback, 10)
        self.aligned_depth_image_base_subscriber = self.create_subscription(Image, "/camera/depth/image_raw", self.get_depth_base_image_callback, 10)
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.objects_filtered_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered', self.object_detected_filtered_callback, 10)
        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.target_pos_check_answer_subscriber = self.create_subscription(Bool, "target_pos_check_answer", self.target_pos_check_answer_callback, 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_pos_reached", 10) # used only for gamepad controller
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_pose_callback, 10)
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        self.robot_gripper_localisation_subscriber = self.create_subscription(Point, "robot_gripper_localisation", self.robot_gripper_localisation_callback, 10)
        self.robot_base_gripper_localisation_subscriber = self.create_subscription(Point, "robot_base_gripper_localisation", self.robot_base_gripper_localisation_callback, 10)
        # Search for person and object 
        self.search_for_person_detections_publisher = self.create_publisher(ListOfDetectedPerson, "search_for_person_detections", 10)
        self.search_for_object_detections_publisher = self.create_publisher(ListOfDetectedObject, "search_for_object_detections", 10)
        # Gamepad Controller
        self.gamepad_controller_subscriber = self.create_subscription(GamepadController, "gamepad_controller", self.gamepad_controller_callback, 10)
        # Low level
        self.torso_movement_publisher = self.create_publisher(Pose2D, "torso_move" , 10) # used only for gamepad controller
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10) # used only for gamepad controller
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.buttons_low_level_subscriber = self.create_subscription(ButtonsLowLevel, "buttons_low_level", self.buttons_low_level_callback, 10)
        self.vccs_low_level_subscriber = self.create_subscription(VCCsLowLevel, "vccs_low_level", self.vccs_low_level_callback, 10)
        self.torso_low_level_subscriber = self.create_subscription(TorsoPosition, "torso_position", self.torso_low_level_callback, 10)
        self.orientation_low_level_subscriber = self.create_subscription(Float32, "orientation_low_level", self.orientation_callback, 10)
        # Neck
        self.continuous_tracking_position_publisher = self.create_publisher(Point, "continuous_tracking_position", 10)
        # Tracking
        self.tracking_mask_subscriber = self.create_subscription(TrackingMask, 'tracking_mask', self.tracking_mask_callback, 10)
        # Task States Info
        self.task_states_info_publisher = self.create_publisher(TaskStatesInfo, "task_states_info", 10)
        self.task_states_info_subscriber = self.create_subscription(TaskStatesInfo, "task_states_info", self.demo_task_states_info_callback, 10)
        self.task_state_selectable_publisher = self.create_publisher(Int16, "task_state_selectable", 10)
        # Odom
        self.odom_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.odom_wheels_subscriber = self.create_subscription(Odometry, "/wheel_encoders", self.odom_wheels_callback, 10)
        # Radar
        self.radar_data_subscriber = self.create_subscription(RadarData, "radar/data", self.radar_data_callback, 10)
        

        ### Services (Clients) ###
        # Arm
        self.set_height_furniture_for_arm_manual_movement_client = self.create_client(SetFloat, "set_table_height")
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        self.save_speech_command_client = self.create_client(SaveSpeechCommand, "save_speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.continuous_get_audio_client = self.create_client(ContinuousGetAudio, "continuous_audio")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Sound Classification
        self.get_sound_classification_client = self.create_client(GetSoundClassification, "get_sound_classification")
        self.continuous_get_sound_classification_client = self.create_client(GetSoundClassificationContinuous, "get_sound_classification_continuous")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        self.face_set_touchscreen_menu_client = self.create_client(SetFaceTouchscreenMenu, "set_face_touchscreen_menu")
        self.server_face_get_touchscreen_menu = self.create_service(GetFaceTouchscreenMenu, "get_face_touchscreen_menu", self.callback_face_get_touchscreen_menu) 
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        self.neck_continuous_tracking_client = self.create_client(TrackContinuous, "set_continuous_tracking")
        # Yolo Pose
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # Yolo Objects
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Yolo World
        self.activate_yolo_world_client = self.create_client(ActivateYoloWorld, "activate_yolo_world")
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(Trigger, "arm_trigger")
        # Radar
        self.get_minimum_radar_distance_client = self.create_client(GetMinRadarDistance, "get_min_radar_distance")
        # NAV2
        self.clear_entire_local_costmap_client  = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.clear_entire_global_costmap_client = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        # Navigation
        self.clear_nav2_costmaps_client = self.create_client(Trigger, "clear_nav_costmaps")
        # Low level
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode")
        self.set_torso_position_client = self.create_client(SetTorso, "set_torso_position")
        self.internal_set_initial_position_define_north_client = self.create_client(SetPoseWithCovarianceStamped, "internal_initial_pose_for_north")
        self.activate_motors_client = self.create_client(ActivateBool, "activate_motors")
        # GUI
        self.nodes_used_client = self.create_client(NodesUsed, "nodes_used_gui")
        # LLM
        self.llm_demonstration_client = self.create_client(GetLLMDemo, "llm_demonstration")
        self.llm_confirm_command_client = self.create_client(GetLLMConfirmCommand, "llm_confirm_command")
        self.llm_gpsr_client = self.create_client(GetLLMGPSR, "llm_gpsr")
        # Tracking (SAM2)
        self.activate_tracking_client = self.create_client(ActivateTracking, "activate_tracking")
        # Task State Demo 
        self.set_task_state_demo_client = self.create_client(SetInt, "task_state_demo")
        self.get_task_state_demo_server = self.create_service(SetInt, "task_state_demo", self.callback_get_task_state_demo) 

        ### Actions (Clients) ###
        # From NAV2
        self.nav2_client_ = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav2_client_follow_waypoints_ = ActionClient(self, FollowWaypoints, "follow_waypoints")
        # From CHARMIE Navigation
        self.charmie_nav2_client_ = ActionClient(self, NavigateToPose, "charmie_navigate_to_pose")
        self.charmie_nav2_follow_waypoints_client_ = ActionClient(self, FollowWaypoints, "charmie_navigate_follow_waypoints")
        self.charmie_nav2_safety_client = ActionClient(self, NavigateToPose, "charmie_navigate_to_pose_safety")

        self.adjust_navigation_angle_client = ActionClient(self, AdjustNavigationAngle, "adjust_navigation_angle")
        self.adjust_navigation_omni_client = ActionClient(self, AdjustNavigationOmnidirectional, "adjust_navigation_omni")
        self.adjust_navigation_obstacle_client = ActionClient(self, AdjustNavigationObstacles, "adjust_navigation_obstacle")
    

        self.send_node_used_to_gui()

        """
            "charmie_arm":                  True,
            "charmie_audio":                True,
            "charmie_face":                 True,
        "charmie_head_camera":              True,
        "charmie_hand_camera":              True,
        "charmie_base_camera":              False,
        "charmie_gamepad":                  False,
        "charmie_lidar":                    True,
        "charmie_lidar_bottom":             False,
        "charmie_lidar_livox":              False,
            "charmie_llm":                  False,
        "charmie_localisation":             False,
            "charmie_low_level":            True,
            "charmie_navigation":           True,
            "charmie_nav2":                 False,
            "charmie_neck":                 True,
        "charmie_radar":                    True,
            "charmie_sound_classification": False,
            "charmie_speakers":             True,
            "charmie_tracking":             False,
            "charmie_yolo_objects":         True,
            "charmie_yolo_pose":            False,
            "charmie_yolo_world":           False,
        """

        # waits until all modules are correctly turned ON
        if self.ros2_modules["charmie_arm"]:
            while not self.arm_trigger_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Arm Trigger Command...")

        if self.ros2_modules["charmie_audio"]:
            while not self.get_audio_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Audio Server...")
            while not self.calibrate_audio_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Calibrate Audio Server...")

        if self.ros2_modules["charmie_face"]:
            while not self.face_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Face Command...")

        if self.ros2_modules["charmie_llm"]:
            while not self.llm_demonstration_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Demo Server LLM ...")
            while not self.llm_confirm_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Confirm Command Server LLM ...")
            while not self.llm_gpsr_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for GPSR Server LLM ...")

        if self.ros2_modules["charmie_low_level"]:
            while not self.set_rgb_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Low Level ...")

        if self.ros2_modules["charmie_navigation"]:
            while not self.adjust_navigation_angle_client.server_is_ready():
                self.get_logger().warn("Waiting for Action Server Adjust Angle Command...")
                time.sleep(1.0)

        if self.ros2_modules["charmie_nav2"]:
            while not self.nav2_client_.server_is_ready():
                self.get_logger().warn("Waiting for Server Nav2 Trigger Command...")
                time.sleep(1.0)

        if self.ros2_modules["charmie_neck"]:
            while not self.set_neck_position_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Set Neck Position Command...")
            while not self.get_neck_position_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Get Neck Position Command...")
            while not self.set_neck_coordinates_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
            while not self.neck_track_person_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
            while not self.neck_track_object_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        
        if self.ros2_modules["charmie_radar"]:
            while not self.get_minimum_radar_distance_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Get Minimum Radar Distance Command...")

        if self.ros2_modules["charmie_sound_classification"]:
            while not self.get_sound_classification_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Sound Classification Command...")
            
        if self.ros2_modules["charmie_speakers"]:
            while not self.speech_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Speech Command...")
            while not self.save_speech_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Save Speech Command...")

        if self.ros2_modules["charmie_tracking"]:
            while not self.activate_tracking_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Activate Tracking Command...")

        if self.ros2_modules["charmie_yolo_objects"]:
            while not self.activate_yolo_objects_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")

        if self.ros2_modules["charmie_yolo_pose"]:
            while not self.activate_yolo_pose_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")

        if self.ros2_modules["charmie_yolo_world"]:
            while not self.activate_yolo_world_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Yolo World Activate Command...")
        
        self.create_timer(0.5, self.task_states_info_publisher_timer)

        # Task Variables
        self.task_name = ""
        self.task_states = {}
        self.swapped_task_states = {}
        self.current_task_state_id = 0

        #Task Demo Setting
        self.received_new_demo_task_state = False
        self.new_demo_task_state = 0

        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_continuous_audio = False
        self.waited_for_end_of_sound_classification = False
        self.waited_for_end_of_continuous_sound_classification = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_save_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_continuous_tracking = False
        self.waited_for_end_of_arm = False
        self.waited_for_end_of_face = False
        self.waited_for_end_of_face_touchscreen_menu = False
        self.waited_for_end_of_set_torso_position = False
        self.waited_for_end_of_llm_demonstration = False
        self.waited_for_end_of_llm_confirm_command = False
        self.waited_for_end_of_llm_gpsr = False
        self.waited_for_end_of_get_minimum_radar_distance = False

        self.br = CvBridge()
        self.rgb_head_img = Image()
        self.rgb_hand_img = Image()
        self.rgb_base_img = Image()
        self.depth_head_img = Image()
        self.depth_hand_img = Image()
        self.depth_base_img = Image()
        self.first_rgb_head_image_received = False
        self.first_rgb_hand_image_received = False
        self.first_rgb_base_image_received = False
        self.first_depth_head_image_received = False
        self.first_depth_hand_image_received = False
        self.first_depth_base_image_received = False
        self.detected_people = ListOfDetectedPerson()
        self.detected_objects = ListOfDetectedObject()
        self.flag_navigation_reached = False
        self.flag_target_pos_check_answer = False
        self.gamepad_controller_state = GamepadController()
        self.previous_gamepad_controller_state = GamepadController()
        self.current_gamepad_controller_state = GamepadController()
        self.new_object_frame_for_tracking = False
        self.new_person_frame_for_tracking = False
        self.tracking_mask = TrackingMask()
        self.new_tracking_mask_msg = False
        self.amcl_pose = PoseWithCovarianceStamped()
        self.new_amcl_pose_msg = False
        self.selected_list_options_touchscreen_menu = []
        self.get_minimum_radar_distance_value = GetMinRadarDistance.Response()

        # robot localization
        self.robot_pose = Pose2D()
        self.gripper_point = Point()
        self.base_gripper_point = Point()

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.save_speech_success = True
        self.save_speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.torso_success = True
        self.torso_message = ""
        self.audio_success = True
        self.audio_message = ""
        self.continuous_audio_success = True
        self.continuous_audio_message = ""
        self.calibrate_audio_success = True
        self.calibrate_audio_message = ""
        self.sound_classification_success = True
        self.sound_classification_message = ""
        self.continuous_sound_classification_success = True
        self.continuous_sound_classification_message = ""
        self.face_success = True
        self.face_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        self.track_object_success = True
        self.track_object_message = ""
        self.continuous_tracking_success = True
        self.continuous_tracking_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.activate_yolo_world_success = True
        self.activate_yolo_world_message = ""
        self.arm_success = True
        self.arm_message = ""
        self.navigation_success = True
        self.navigation_message = ""
        self.activate_motors_success = True
        self.activate_motors_message = ""
        self.activate_tracking_success = True
        self.activate_tracking_message = ""
        self.set_height_furniture_for_arm_manual_movement_client_success = True
        self.set_height_furniture_for_arm_manual_movement_client_message = ""

        self.audio_command = ""
        self.received_continuous_audio = False
        self.continuous_audio_detected_keyword = ""

        self.sound_classification_labels = []
        self.sound_classification_scores = []
        self.received_continuous_sound_classification = False
        self.continuous_sound_classification_detected_label = ""
        self.continuous_sound_classification_detected_score = 0.0

        self.face_recognition_encoding = []
        self.face_recognition_names = []
        
        self.CAM_IMAGE_WIDTH = 848
        self.CAM_IMAGE_HEIGHT = 480

        self.get_neck_position = [1.0, 1.0]
        self.torso_position = TorsoPosition()
        self.vccs = VCCsLowLevel()
        self.buttons_low_level = ButtonsLowLevel()
        self.orientation_yaw = 0.0
        self.new_gamepad_controller_msg = False
        self.llm_demonstration_response = ""
        self.llm_confirm_command_response = ""
        self.llm_gpsr_response = ListOfStrings()
        self.received_demo_tsi = TaskStatesInfo()
        self.radar = RadarData()
        self.cmd_vel = Twist()
        self.is_radar_initialized = False

        self.goal_handle_ = None
        self.nav2_goal_accepted = None
        self.nav2_feedback = NavigateToPose.Feedback()
        self.nav2_status = GoalStatus.STATUS_UNKNOWN

        self.goal_follow_waypoints_handle_ = None
        self.nav2_follow_waypoints_goal_accepted = None
        self.nav2_follow_waypoints_feedback = NavigateToPose.Feedback()
        self.nav2_follow_waypoints_status = GoalStatus.STATUS_UNKNOWN

        self.goal_safety_handle_ = None
        self.nav2_safety_goal_accepted = None
        self.nav2_safety_feedback = NavigateToPose.Feedback()
        self.nav2_safety_status = GoalStatus.STATUS_UNKNOWN

        self.adjust_angle_navigation_handle_ = None
        self.adjust_angle_navigation_accepted = None
        self.adjust_angle_navigation_feedback = AdjustNavigationAngle.Feedback()
        self.adjust_angle_navigation_status = GoalStatus.STATUS_UNKNOWN

        self.adjust_omni_navigation_handle_ = None
        self.adjust_omni_navigation_accepted = None
        self.adjust_omni_navigation_feedback = AdjustNavigationOmnidirectional.Feedback()
        self.adjust_omni_navigation_status = GoalStatus.STATUS_UNKNOWN

        self.adjust_obstacle_navigation_handle_ = None
        self.adjust_obstacle_navigation_accepted = None
        self.adjust_obstacle_navigation_feedback = AdjustNavigationObstacles.Feedback()
        self.adjust_obstacle_navigation_status = GoalStatus.STATUS_UNKNOWN

        self.current_odom_pose = None
        self.current_odom_wheels_pose = None

    def task_states_info_publisher_timer(self):

        if self.task_name != "":
            tsi = TaskStatesInfo()
            tsi.task_name = self.task_name
            tsi.current_task_state_id = self.current_task_state_id
            tsi.list_of_states = list(self.task_states.keys())
            tsi.list_of_states_ids = list(self.task_states.values())
            self.task_states_info_publisher.publish(tsi)
    
    def demo_task_states_info_callback(self, task_states_info: TaskStatesInfo):
        self.received_demo_tsi = task_states_info

    def send_node_used_to_gui(self):

        nodes_used = NodesUsed.Request()

        nodes_used.charmie_arm                  = self.ros2_modules["charmie_arm"]
        nodes_used.charmie_audio                = self.ros2_modules["charmie_audio"]
        nodes_used.charmie_face                 = self.ros2_modules["charmie_face"]
        nodes_used.charmie_head_camera          = self.ros2_modules["charmie_head_camera"]
        nodes_used.charmie_hand_camera          = self.ros2_modules["charmie_hand_camera"]
        nodes_used.charmie_base_camera          = self.ros2_modules["charmie_base_camera"]
        nodes_used.charmie_gamepad              = self.ros2_modules["charmie_gamepad"]
        nodes_used.charmie_lidar                = self.ros2_modules["charmie_lidar"]
        nodes_used.charmie_lidar_bottom         = self.ros2_modules["charmie_lidar_bottom"]
        nodes_used.charmie_lidar_livox          = self.ros2_modules["charmie_lidar_livox"]
        nodes_used.charmie_localisation         = self.ros2_modules["charmie_localisation"]
        nodes_used.charmie_low_level            = self.ros2_modules["charmie_low_level"]
        nodes_used.charmie_llm                  = self.ros2_modules["charmie_llm"]
        nodes_used.charmie_navigation           = self.ros2_modules["charmie_navigation"]
        nodes_used.charmie_nav2                 = self.ros2_modules["charmie_nav2"]
        nodes_used.charmie_neck                 = self.ros2_modules["charmie_neck"]
        nodes_used.charmie_radar                = self.ros2_modules["charmie_radar"]
        nodes_used.charmie_sound_classification = self.ros2_modules["charmie_sound_classification"]
        nodes_used.charmie_speakers             = self.ros2_modules["charmie_speakers"]
        nodes_used.charmie_tracking             = self.ros2_modules["charmie_tracking"]
        nodes_used.charmie_yolo_objects         = self.ros2_modules["charmie_yolo_objects"]
        nodes_used.charmie_yolo_pose            = self.ros2_modules["charmie_yolo_pose"]
        nodes_used.charmie_yolo_world           = self.ros2_modules["charmie_yolo_world"]

        self.nodes_used_client.call_async(nodes_used)

    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people
        self.new_person_frame_for_tracking = True

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)

    def object_detected_filtered_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects = det_object
        self.new_object_frame_for_tracking = True

    def get_rgbd_head_callback(self, rgbd: RGBD):
        self.rgb_head_img = rgbd.rgb
        self.first_rgb_head_image_received = True
        self.depth_head_img = rgbd.depth
        self.first_depth_head_image_received = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        self.rgb_hand_img = rgbd.rgb
        self.first_rgb_hand_image_received = True
        self.depth_hand_img = rgbd.depth
        self.first_depth_hand_image_received = True
        # print("HAND:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_color_image_base_callback(self, img: Image):
        self.rgb_base_img = img
        self.first_rgb_base_image_received = True

    def get_depth_base_image_callback(self, img: Image):
        self.depth_base_img = img
        self.first_depth_base_image_received = True

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_pose = pose

    def robot_gripper_localisation_callback(self, point: Point):
        self.gripper_point = point

    def robot_base_gripper_localisation_callback(self, point: Point):
        self.base_gripper_point = point

    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        self.waited_for_end_of_arm = True
        self.arm_success = flag.data
        if flag.data:
            self.arm_message = "Arm successfully moved"
        else:
            self.arm_message = "Wrong Movement Received"

        self.get_logger().info("Received Arm Finished")

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    def target_pos_check_answer_callback(self, flag: Bool):
        self.flag_target_pos_check_answer = flag.data
        # print("RECEIVED NAVIGATION CONFIRMATION")

    ### ODOMETRY ###
    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose

    def odom_wheels_callback(self, msg):
        self.current_odom_wheels_pose = msg.pose
    
    ### Gamepad Controller ###
    def gamepad_controller_callback(self, controller: GamepadController):
        self.gamepad_controller_state = controller
        self.new_gamepad_controller_msg = True

    def tracking_mask_callback(self, mask: TrackingMask):
        self.tracking_mask = mask
        self.new_tracking_mask_msg = True

    ### Low Level ###
    def buttons_low_level_callback(self, buttons: ButtonsLowLevel):
        self.buttons_low_level = buttons

    def vccs_low_level_callback(self, vccs: VCCsLowLevel):
        self.vccs = vccs

    def torso_low_level_callback(self, torso: TorsoPosition):
        self.torso_position = torso
        # print("Received Torso Position:", torso) 

    def orientation_callback(self, orientation: Float32):
        self.orientation_yaw = orientation.data

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def radar_data_callback(self, radar: RadarData):
        self.radar = radar
        self.is_radar_initialized = True

        # DEBUG PRINTS
        # nos     = self.radar.number_of_sectors
        # sar     = self.radar.sector_ang_range
        # sectors = self.radar.sectors
        # print(f"Radar Data: Number of Sectors: {nos}, Sector Angle Range (deg): {round(math.degrees(sar),1)}")
        # i = 0
        # for s in sectors:
        #     sa = s.start_angle
        #     ea = s.end_angle
        #     md = s.min_distance
        #     p  = s.point
        #     hp = s.has_point
        #     print(f"Sector {i}: Start Angle: {round(math.degrees(sa),1)}, End Angle: {round(math.degrees(ea),1)}, Min Distance: {round(md,2)}, Has Point: {hp}, Point: ({round(p.x,2)}, {round(p.y,2)}, {round(p.z,2)})")
        #     i += 1    

    ### SERVICES ###

    ### ACTIVATE YOLO POSE SERVER FUNCTIONS ###
    def call_activate_yolo_pose_server(self, request=ActivateYoloPose.Request()):

        self.activate_yolo_pose_client.call_async(request)


    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, request=ActivateYoloObjects.Request()):

        self.activate_yolo_objects_client.call_async(request)


    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_world_server(self, request=ActivateYoloWorld.Request()):

        self.activate_yolo_world_client.call_async(request)


    ### ACTIVATE TRACKING SERVER FUNCTIONS ###
    def call_activate_tracking_server(self, request=ActivateTracking.Request()):

        self.activate_tracking_client.call_async(request)


    ### SET TABLE HEIGHT FOR MANUAL ARM MOVMENTS ###
    def call_set_height_furniture_for_arm_manual_movement_server(self, request=SetFloat.Request()):

        self.set_height_furniture_for_arm_manual_movement_client.call_async(request)


    #### FACE SERVER FUNCTIONS #####
    def call_face_command_server(self, request=SetFace.Request(), wait_for_end_of=True):
        
        future = self.face_command_client.call_async(request)
        
        if wait_for_end_of:
            future.add_done_callback(self.callback_call_face_command)
        else:
            self.face_success = True
            self.face_message = "Wait for answer not needed"
    
    def callback_call_face_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.face_success = response.success
            self.face_message = response.message
            self.waited_for_end_of_face = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_face_set_touchscreen_menu_server(self, request=SetFaceTouchscreenMenu.Request()):
        self.face_set_touchscreen_menu_client.call_async(request)

    def callback_face_get_touchscreen_menu(self, request, response):
        self.selected_list_options_touchscreen_menu  = request.command
        self.waited_for_end_of_face_touchscreen_menu = True
        # print(self.selected_list_options_touchscreen_menu)
        return response
    
    #### SPEECH SERVER FUNCTIONS #####
    def call_speech_command_server(self, request=SpeechCommand.Request(), wait_for_end_of=True):
    
        future = self.speech_command_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"

    def callback_call_speech_command(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### SAVE SPEECH SERVER FUNCTIONS #####
    def call_save_speech_command_server(self, request = SaveSpeechCommand.Request(), wait_for_end_of=True):
    
        future = self.save_speech_command_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_save_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"
    
    def callback_call_save_speech_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.save_speech_success = response.success
            self.save_speech_message = response.message
            self.waited_for_end_of_save_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    #### AUDIO SERVER FUNCTIONS #####
    def call_audio_server(self, request=GetAudio.Request(), wait_for_end_of=True):

        future = self.get_audio_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_audio)
        else:
            self.audio_success = True
            self.audio_message = "Wait for answer not needed"
    
    def callback_call_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.command))
            self.audio_success = True
            self.audio_message = "Finished audio command"
            self.audio_command = response.command
            self.waited_for_end_of_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    
    def call_continuous_audio_server(self, request=ContinuousGetAudio.Request(), wait_for_end_of=True):

        future = self.continuous_get_audio_client.call_async(request)

        future.add_done_callback(self.callback_call_continuous_audio)

    def callback_call_continuous_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message) + " - " + str(response.detected_keyword))
            self.continuous_audio_success = response.success
            self.continuous_audio_message = response.message
            self.continuous_audio_detected_keyword = response.detected_keyword
            self.waited_for_end_of_continuous_audio = True
            self.received_continuous_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_calibrate_audio_server(self, request=CalibrateAudio.Request(), wait_for_end_of=True):

        future = self.calibrate_audio_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_calibrate_audio)
        else:
            self.calibrate_audio_success = True
            self.calibrate_audio_message = "Wait for answer not needed"
    
    def callback_call_calibrate_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.calibrate_audio_success = response.success
            self.calibrate_audio_message = response.message
            self.waited_for_end_of_calibrate_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    #### SOUND CLASSIFICATION SERVER FUNCTIONS #####
    def call_sound_classification_server(self, request=GetSoundClassification.Request(), wait_for_end_of=True):

        future = self.get_sound_classification_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_sound_classification)
        else:
            self.sound_classification_success = True
            self.sound_classification_message = "Wait for answer not needed"
    
    def callback_call_sound_classification(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.labels)+" "+str(response.scores))
            self.sound_classification_success = True
            self.sound_classification_message = "Finished sound classification command"
            self.sound_classification_labels = response.labels
            self.sound_classification_scores = response.scores
            self.waited_for_end_of_sound_classification = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    def call_continuous_sound_classification_server(self, request=GetSoundClassificationContinuous.Request(), wait_for_end_of=True):

        future = self.continuous_get_sound_classification_client.call_async(request)

        future.add_done_callback(self.callback_call_continuous_sound_classification)

    def callback_call_continuous_sound_classification(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message) + " - " + str(response.detected_label) + " - " + str(response.detected_score))
            self.continuous_sound_classification_success = response.success
            self.continuous_sound_classification_message = response.message
            self.continuous_sound_classification_detected_label = response.detected_label
            self.continuous_sound_classification_detected_score = response.detected_score
            self.waited_for_end_of_continuous_sound_classification = True
            self.received_continuous_sound_classification = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    #### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, request = SetNeckPosition.Request(), wait_for_end_of=True):
        
        future = self.set_neck_position_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.neck_success = response.success
            self.neck_message = response.message
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### SET NECK COORDINATES SERVER FUNCTIONS #####
    def call_neck_coordinates_server(self, request = SetNeckCoordinates.Request(), wait_for_end_of=True):
        
        future = self.set_neck_coordinates_client.call_async(request)

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_set_neck_coords_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_coords_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.neck_success = response.success
            self.neck_message = response.message
            self.waited_for_end_of_neck_coords = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### GET NECK POSITION SERVER FUNCTIONS #####
    def call_get_neck_position_server(self, request=GetNeckPosition.Request()):
        
        future = self.get_neck_position_client.call_async(request)

        future.add_done_callback(self.callback_call_get_neck_command)
    
    def callback_call_get_neck_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info("Received Neck Position: (%s" %(str(response.pan) + ", " + str(response.tilt)+")"))
            self.get_neck_position[0] = response.pan
            self.get_neck_position[1] = response.tilt
            self.waited_for_end_of_get_neck = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### NECK SERVER FUNCTIONS #####
    def call_neck_track_person_server(self, request = TrackPerson.Request(), wait_for_end_of=True):

        future = self.neck_track_person_client.call_async(request)
        
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
            self.waited_for_end_of_track_person = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_neck_track_object_server(self, request=TrackObject.Request(), wait_for_end_of=True):

        future = self.neck_track_object_client.call_async(request)
        
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
            self.waited_for_end_of_track_object = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_neck_continuous_tracking_server(self, request=TrackContinuous.Request(), wait_for_end_of=True):

        future = self.neck_continuous_tracking_client.call_async(request)
        
        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_continuous_tracking)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_continuous_tracking(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.continuous_tracking_success = response.success
            self.continuous_tracking_message = response.message
            self.waited_for_end_of_continuous_tracking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    #### LOW LEVEL SERVER FUNCTIONS #####
    def call_rgb_command_server(self, request=SetRGB.Request(), wait_for_end_of=True):
        
        self.set_rgb_client.call_async(request)
        
        self.rgb_success = True
        self.rgb_message = "Value Sucessfully Sent"


    def call_set_torso_position_server(self, request=SetTorso.Request()):
    
        future = self.set_torso_position_client.call_async(request)
        future.add_done_callback(self.callback_call_set_torso_position)
        
    def callback_call_set_torso_position(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.torso_success = response.success
            self.torso_message = response.message
            # self.waited_for_end_of_set_torso_position = True # the wait for end of is done in the std_functions
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))  

    def call_activate_motors_server(self, request=ActivateBool.Request()):
    
        future = self.activate_motors_client.call_async(request)
        future.add_done_callback(self.callback_call_activate_motors)

    def call_internal_set_initial_position_define_north_command_server(self, request=SetPoseWithCovarianceStamped.Request(), wait_for_end_of=True):

        self.internal_set_initial_position_define_north_client.call_async(request)
        
        
    def callback_call_activate_motors(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.activate_motors_success = response.success
            self.activate_motors_message = response.message
            # self.waited_for_end_of_set_torso_position = True # the wait for end of is done in the std_functions
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))  

    def call_llm_demonstration_server(self, request=GetLLMDemo.Request(), wait_for_end_of=True):

        future = self.llm_demonstration_client.call_async(request)
        future.add_done_callback(self.callback_call_llm_demonstration)
        
    def callback_call_llm_demonstration(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.llm_demonstration_response = response.answer
            self.get_logger().info("Received LLM Demo Answer:"+str(self.llm_demonstration_response))
            self.waited_for_end_of_llm_demonstration = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_llm_confirm_command_server(self, request=GetLLMConfirmCommand.Request(), wait_for_end_of=True):

        future = self.llm_confirm_command_client.call_async(request)
        future.add_done_callback(self.callback_call_llm_confirm_command)
        
    def callback_call_llm_confirm_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.llm_confirm_command_response = response.answer
            self.get_logger().info("Received LLM Confirm Command Answer:"+str(self.llm_confirm_command_response))
            self.waited_for_end_of_llm_confirm_command = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_llm_gpsr_server(self, request=GetLLMGPSR.Request(), wait_for_end_of=True):

        future = self.llm_gpsr_client.call_async(request)
        future.add_done_callback(self.callback_call_llm_gpsr)
        
    def callback_call_llm_gpsr(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.llm_gpsr_response = response.answer
            self.get_logger().info("Received LLM GPSR Answer:"+str(self.llm_gpsr_response))
            # for cmd in self.llm_gpsr_response.strings:
            #     self.get_logger().info(str(cmd))
            self.waited_for_end_of_llm_gpsr = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg
        self.new_amcl_pose_msg = True

    def call_set_task_state_demo_server(self, request=SetInt.Request()):
        self.set_task_state_demo_client.call_async(request)
        print("Sent Demo Task State Request")

    def callback_get_task_state_demo(self, request, response):
        # print(request)

        # Type of service received:
        # int32 data   # generic int value sent  
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        print("Received Demo Task State Request")

        self.received_new_demo_task_state = True
        self.new_demo_task_state = request.data

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Set task state: " + str(self.received_demo_tsi.list_of_states[self.new_demo_task_state])
        print(response.message)

        return response
    
    def call_get_minimum_radar_distance_server(self, request=GetMinRadarDistance.Request()):
        
        future = self.get_minimum_radar_distance_client.call_async(request)
        future.add_done_callback(self.callback_call_get_minimum_radar_distance_command)
        
    def callback_call_get_minimum_radar_distance_command(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_minimum_radar_distance_value = response
            self.get_logger().info("Received Min Radar Dist: %s" %(str(response.min_radar_distance_to_robot_edge)))
            self.waited_for_end_of_get_minimum_radar_distance = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    def call_clear_nav2_costmaps(self, request=Trigger.Request(), wait_for_end_of=True):
        
        self.clear_nav2_costmaps_client.call_async(request)

    ### Nav2 Action Client ###
    # ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

    def nav2_client_goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.nav2_client_goal_result_callback)
            self.nav2_goal_accepted = True
        else:
            self.nav2_goal_accepted = False
            self.goal_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def nav2_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav2_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.nav2_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.nav2_status = status
            self.get_logger().info(f"Result: Unknown result code {status}")
        
        # When goal is finished, clear the handle
        self.goal_handle_ = None
    
    def nav2_client_cancel_goal(self):
        if self.goal_handle_ is None:
            self.get_logger().warn("No active NavigateToPose goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to Nav2...")
        self.goal_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.goal_handle_ = None

    def nav2_client_goal_feedback_callback(self, feedback_msg):
        self.nav2_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        # current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        # current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        # no_recoveries = str(feedback.number_of_recoveries)
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")


    # Navigate through poses
    def nav2_follow_waypoints_client_goal_response_callback(self, future):
        self.goal_follow_waypoints_handle_:ClientGoalHandle = future.result()
        if self.goal_follow_waypoints_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_follow_waypoints_handle_.get_result_async().add_done_callback(self.nav2_follow_waypoints_client_goal_result_callback)
            self.nav2_follow_waypoints_goal_accepted = True
        else:
            self.nav2_follow_waypoints_goal_accepted = False
            self.goal_follow_waypoints_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def nav2_follow_waypoints_client_cancel_goal(self):
        if self.goal_follow_waypoints_handle_ is None:
            self.get_logger().warn("No active NavigateToPoseFollowWaypoints goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to Nav2...")
        self.goal_follow_waypoints_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.goal_follow_waypoints_handle_ = None

    def nav2_follow_waypoints_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_follow_waypoints_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav2_follow_waypoints_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.nav2_follow_waypoints_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.nav2_follow_waypoints_status = status
            self.get_logger().info(f"Result: Unknown result code {status}")
            
        # When goal is finished, clear the handle
        self.goal_follow_waypoints_handle_ = None

    def nav2_follow_waypoints_client_goal_feedback_callback(self, feedback_msg):
        self.nav2_follow_waypoints_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        # current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        # current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        # no_recoveries = str(feedback.number_of_recoveries)
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")
    

    # Adjust Angle Navigation Action Client
    def adjust_angle_navigation_client_goal_response_callback(self, future):
        self.adjust_angle_navigation_handle_:ClientGoalHandle = future.result()
        if self.adjust_angle_navigation_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.adjust_angle_navigation_handle_.get_result_async().add_done_callback(self.adjust_angle_navigation_client_goal_result_callback)
            self.adjust_angle_navigation_accepted = True
        else:
            self.adjust_angle_navigation_accepted = False
            self.adjust_angle_navigation_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def adjust_angle_navigation_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.adjust_angle_navigation_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.adjust_angle_navigation_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.adjust_angle_navigation_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.adjust_angle_navigation_status = status
            self.get_logger().warn(f"Finished with status={status}")
            
        # When goal is finished, clear the handle
        self.adjust_angle_navigation_handle_ = None
        
    def adjust_angle_navigation_client_cancel_goal(self):
        if self.adjust_angle_navigation_handle_ is None:
            self.get_logger().warn("No active Adjust Angle Navigation goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to adjust_angle_navigation...")
        self.adjust_angle_navigation_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.adjust_angle_navigation_handle_ = None
        
    def adjust_angle_navigation_client_goal_feedback_callback(self, feedback_msg):
        self.adjust_angle_navigation_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")

    def nav2_safety_client_goal_response_callback(self, future):
        self.goal_safety_handle_:ClientGoalHandle = future.result()
        if self.goal_safety_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_safety_handle_.get_result_async().add_done_callback(self.nav2_safety_client_goal_result_callback)
            self.nav2_safety_goal_accepted = True
        else:
            self.nav2_safety_goal_accepted = False
            self.goal_safety_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def nav2_safety_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_safety_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav2_safety_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.nav2_safety_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.nav2_safety_status = status
            self.get_logger().info(f"Result: Unknown result code {status}")
        
        # When goal is finished, clear the handle
        self.goal_safety_handle_ = None
    
    def nav2_safety_client_cancel_goal(self):
        if self.goal_safety_handle_ is None:
            self.get_logger().warn("No active NavigateToPose goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to Nav2...")
        self.goal_safety_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.goal_safety_handle_ = None

    def nav2_safety_client_goal_feedback_callback(self, feedback_msg):
        self.nav2_safety_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        # current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        # current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        # no_recoveries = str(feedback.number_of_recoveries)
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")


    # Adjust omni Navigation Action Client
    def adjust_omni_navigation_client_goal_response_callback(self, future):
        self.adjust_omni_navigation_handle_:ClientGoalHandle = future.result()
        if self.adjust_omni_navigation_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.adjust_omni_navigation_handle_.get_result_async().add_done_callback(self.adjust_omni_navigation_client_goal_result_callback)
            self.adjust_omni_navigation_accepted = True
        else:
            self.adjust_omni_navigation_accepted = False
            self.adjust_omni_navigation_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def adjust_omni_navigation_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.adjust_omni_navigation_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.adjust_omni_navigation_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.adjust_omni_navigation_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.adjust_omni_navigation_status = status
            self.get_logger().warn(f"Finished with status={status}")
            
        # When goal is finished, clear the handle
        self.adjust_omni_navigation_handle_ = None
            
    def adjust_omni_navigation_client_cancel_goal(self):
        if self.adjust_omni_navigation_handle_ is None:
            self.get_logger().warn("No active Adjust omni Navigation goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to adjust_omni_navigation...")
        self.adjust_omni_navigation_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.adjust_omni_navigation_handle_ = None
        
    def adjust_omni_navigation_client_goal_feedback_callback(self, feedback_msg):
        self.adjust_omni_navigation_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")


    # Adjust obstacle Navigation Action Client
    def adjust_obstacle_navigation_client_goal_response_callback(self, future):
        self.adjust_obstacle_navigation_handle_:ClientGoalHandle = future.result()
        if self.adjust_obstacle_navigation_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.adjust_obstacle_navigation_handle_.get_result_async().add_done_callback(self.adjust_obstacle_navigation_client_goal_result_callback)
            self.adjust_obstacle_navigation_accepted = True
        else:
            self.adjust_obstacle_navigation_accepted = False
            self.adjust_obstacle_navigation_handle_ = None
            self.get_logger().warn("Goal rejected.")

    def adjust_obstacle_navigation_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.adjust_obstacle_navigation_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.adjust_obstacle_navigation_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.adjust_obstacle_navigation_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        else:
            self.adjust_obstacle_navigation_status = status
            self.get_logger().warn(f"Finished with status={status}")
            
        # When goal is finished, clear the handle
        self.adjust_obstacle_navigation_handle_ = None
            
    def adjust_obstacle_navigation_client_cancel_goal(self):
        if self.adjust_obstacle_navigation_handle_ is None:
            self.get_logger().warn("No active Adjust obstacle Navigation goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to adjust_obstacle_navigation...")
        self.adjust_obstacle_navigation_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.adjust_obstacle_navigation_handle_ = None
        
    def adjust_obstacle_navigation_client_goal_feedback_callback(self, feedback_msg):
        self.adjust_obstacle_navigation_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")


#############################################################################################################################
#
#   Robot Standard Functions Class
#
#############################################################################################################################



class RobotStdFunctions():

    def __init__(self, node: ROS2TaskNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

        self.AXIS_L3_XX = 0
        self.AXIS_L3_YY = 1
        self.AXIS_L3_ANGLE = 2
        self.AXIS_L3_DIST = 3
        self.AXIS_R3_XX = 4
        self.AXIS_R3_YY = 5
        self.AXIS_R3_ANGLE = 6
        self.AXIS_R3_DIST = 7
        self.AXIS_L2 = 8
        self.AXIS_R2 = 9

        self.BUTTON_CROSS = 0
        self.BUTTON_CIRCLE = 1
        self.BUTTON_SQUARE = 2
        self.BUTTON_TRIANGLE = 3
        self.BUTTON_L1 = 4
        self.BUTTON_R1 = 5
        self.BUTTON_L2 = 6
        self.BUTTON_R2 = 7
        self.BUTTON_L3 = 8
        self.BUTTON_R3 = 9
        self.BUTTON_DPAD_UP = 10
        self.BUTTON_DPAD_DOWN = 11
        self.BUTTON_DPAD_LEFT = 12
        self.BUTTON_DPAD_RIGHT = 13
        self.BUTTON_SHARE = 14
        self.BUTTON_OPTIONS = 15
        self.BUTTON_LOGO = 16
        self.BUTTON_PAD = 17

        self.ON = 0
        self.RISING = 1
        self.OFF = 2
        self.FALLING = 3

        self.ROBOT_RADIUS = 0.28 # in meters

    def get_demo_mode(self):
        return self.node.DEMO_OPTION

    def set_task_name_and_states(self, task_name="", task_states={}):
        
        if task_name == "" or task_states == {}:
            self.node.get_logger().error("Task name or task states cannot be empty... Please set task name and states.")
            while True:
                pass
        
        self.node.task_name = task_name
        self.node.task_states = task_states
        self.node.swapped_task_states = {value: key for key, value in self.node.task_states.items()}

        print("\nTask Name: " + self.node.task_name)
        print("Task States:")
        for key, value in self.node.task_states.items():
            print(key)
        print()

    def set_current_task_state_id(self, current_state=None):
        
        if current_state == None:
            self.node.get_logger().error("Current task state cannot be empty... Please set current task state.")
            while True:
                pass

        if current_state == -1: # for demo mode
            print("\n>>> Current Task State: DEMO MODE  <<<\n")
        else:
            self.node.current_task_state_id = current_state # only updates current task if it is not DEMO_MODE
            print("\n>>> Current Task State: " + str(self.node.swapped_task_states[self.node.current_task_state_id]) + " <<<\n")

    def set_task_state_selection(self, task_state_selection):
        data = Int16()
        data.data = task_state_selection
        self.node.task_state_selectable_publisher.publish(data)

    def get_received_new_demo_task_state(self):
        temp = self.node.received_new_demo_task_state
        if self.node.received_new_demo_task_state: # clears variable if true, and sends the value pre clearing
            self.node.received_new_demo_task_state = False
        return temp

    def get_new_demo_task_state(self):
        return self.node.new_demo_task_state

    def set_task_state_demo(self, new_demo_state=0):
        request = SetInt.Request()
        request.data = int(new_demo_state)
        self.node.call_set_task_state_demo_server(request=request)

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, long_pause_show_in_face=False, breakable_play=False, break_play=False, wait_for_end_of=True):

        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.show_in_face = show_in_face
        request.long_pause_show_in_face = long_pause_show_in_face
        request.breakable_play = breakable_play
        request.break_play = break_play

        self.node.call_speech_command_server(request=request, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message
    
    def save_speech(self, filename="", command="", quick_voice=False, play_command=False, show_in_face=False, long_pause_show_in_face=False, wait_for_end_of=True):

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

            request = SaveSpeechCommand.Request()
            request.filename = file
            request.command = comm
            request.quick_voice = quick_voice
            request.play_command = play_command
            request.show_in_face = show_in_face
            request.long_pause_show_in_face = long_pause_show_in_face

            self.node.call_save_speech_command_server(request=request, wait_for_end_of=wait_for_end_of)
            
            if wait_for_end_of:
                while not self.node.waited_for_end_of_save_speaking:
                    pass
            self.node.waited_for_end_of_save_speaking = False

            return self.node.save_speech_success, self.node.save_speech_message

        else:

            self.node.get_logger().error("Could not generate save speech as as filename and command types are incompatible.")
            return False, "Could not generate save speech as as filename and command types are incompatible."

    def set_rgb(self, command=0, wait_for_end_of=True):

        request = SetRGB.Request()
        request.colour = int(command)

        self.node.call_rgb_command_server(request=request, wait_for_end_of=wait_for_end_of)

        return self.node.rgb_success, self.node.rgb_message

    def get_vccs(self, wait_for_end_of=True):

        return self.node.vccs.battery_voltage, self.node.vccs.emergency_stop
    
    def get_low_level_buttons(self, wait_for_end_of=True):

        return self.node.buttons_low_level.start_button, self.node.buttons_low_level.debug_button1, self.node.buttons_low_level.debug_button2, self.node.buttons_low_level.debug_button3
        
    def wait_for_start_button(self):
        
        self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)
        self.set_rgb(WHITE+ALTERNATE_QUARTERS)
        
        print("Waiting for Start Button...")
        while not self.node.buttons_low_level.start_button:
            time.sleep(0.05)
        print("Start Button Pressed")
        self.set_rgb(GREEN+ALTERNATE_QUARTERS)

    def get_orientation_yaw(self, wait_for_end_of=True):

        return self.node.orientation_yaw
        
    def wait_for_door_opening(self):
        
        success = False
        message = ""

        MAX_DOOR_ANGLE = 25.0 # max angle considered to be a door (degrees)
        MAX_DOOR_DISTANCE = 1.5 # max distance to be considered a door (meters) from robot edge
        door_open = False

        self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=False)
        self.set_rgb(WHITE+ALTERNATE_QUARTERS)

        if self.node.is_radar_initialized:

            while not door_open:
                radar = self.node.radar
                obstacle_ctr = 0

                print("USED SECTORS:")
                for s in radar.sectors:
                    if -math.radians(MAX_DOOR_ANGLE/2) <= s.start_angle <= math.radians(MAX_DOOR_ANGLE/2) and \
                       -math.radians(MAX_DOOR_ANGLE/2) <= s.end_angle   <= math.radians(MAX_DOOR_ANGLE/2):
                        # used_sectors.append(s)
                        print(f"Start Angle: {round(math.degrees(s.start_angle),1)}, End Angle: {round(math.degrees(s.end_angle),1)}, Min Distance: {round(s.min_distance,2)}, Has Point: {s.has_point}")
            
                        if s.has_point:
                            if s.min_distance - self.ROBOT_RADIUS < MAX_DOOR_DISTANCE:
                                obstacle_ctr += 1

                if obstacle_ctr == 0:
                    self.set_rgb(GREEN+ALTERNATE_QUARTERS)
                    door_open = True
                    print("DOOR OPEN")
                    success = True
                    message = "Door opened successfully detected"
                    return success, message
                else:
                    door_open = False
                    print("DOOR CLOSED", obstacle_ctr)

        else:
            success = False
            message = "Radar not initialized or wrong distance parameter"
            return success, message

    def enter_house_after_door_opening(self, distance=0.5, speed=0.3):

        position_threshold = 0.5 # meters

        self.set_speech(filename="generic/entering_house", wait_for_end_of=False)

        # Clear costmaps before sending a new goal
        # Helps clearing cluttered costmaps that may cause navigation problems
        self.clear_navigation_costmaps()
            
        self.adjust_omnidirectional_position(dx=distance+position_threshold, dy=0.0, safety=False, max_speed=speed, tolerance=position_threshold, kp=3.0, enter_house_special_case=True, use_wheel_odometry=False)
        
        # Clear costmaps before sending a new goal
        # Helps clearing cluttered costmaps that may cause navigation problems
        self.clear_navigation_costmaps()

        self.set_initial_position([distance+position_threshold/2, 0.0, 0.0]) #  set initial position to be inside the house, the position_threshold/2 is because the robot is already moving

    def set_torso_position(self, legs=0.0, torso=0.0, wait_for_end_of=True):
        # legs from 0.0 (minimum height) to 0.14 (maximum height)
        # torso from 8 degrees (vertical torso) to 64 (horizontal torso)
        
        success = False
        message = ""

        MAX_ERROR_LEGS_READING = 0.003
        MAX_ERROR_TORSO_READING = 3

        MIN_POSSIBLE_LEGS =  0.00 - (MAX_ERROR_LEGS_READING/2)
        MAX_POSSIBLE_LEGS =  0.14 + (MAX_ERROR_LEGS_READING/2)
        MIN_POSSIBLE_TORSO =  8.0 - (MAX_ERROR_TORSO_READING/2)
        MAX_POSSIBLE_TORSO = 64.0 + (MAX_ERROR_TORSO_READING/2)

        legs_valid = False
        if MIN_POSSIBLE_LEGS <= legs <= MAX_POSSIBLE_LEGS:
            legs_valid = True

        torso_valid = False
        if MIN_POSSIBLE_TORSO <= torso <= MAX_POSSIBLE_TORSO:
            torso_valid = True
        
        if legs_valid and torso_valid:
        
            request = SetTorso.Request()
            request.legs = float(legs)
            request.torso = float(torso)

            self.node.call_set_torso_position_server(request=request)

            if wait_for_end_of:
                # must check the position until it has arrived 
                while not self.node.waited_for_end_of_set_torso_position:
                    
                    l, t = self.get_torso_position()
                    error_l = abs(request.legs - l)
                    error_t = abs(request.torso - t)
                    # print(l, t, error_l, error_t)

                    if error_l <= MAX_ERROR_LEGS_READING and error_t <= MAX_ERROR_TORSO_READING:
                        self.node.waited_for_end_of_set_torso_position = True
                    else:
                        time.sleep(0.25)
            
                # time.sleep(0.5) # the max error may make robot think it has reached before it has acutally reached

            self.node.waited_for_end_of_set_torso_position = False

            success = self.node.torso_success
            message = self.node.torso_message
        
        else:
            if legs_valid and not torso_valid:
                message = "Invalid Torso Value, please select a value from " + str(round(MIN_POSSIBLE_TORSO,2)) + " to " +  str(round(MAX_POSSIBLE_TORSO,2)) + "!"
            if not legs_valid and torso_valid:
                message = "Invalid Legs Value, please select a value from " + str(round(MIN_POSSIBLE_LEGS,2)) + " to " +  str(round(MAX_POSSIBLE_LEGS,2)) + "!"
            if not legs_valid and not torso_valid:
                message = "Invalid Legs AND Torso Value. For legs please select a value from " + str(round(MIN_POSSIBLE_LEGS,2)) + " to " +  str(round(MAX_POSSIBLE_LEGS,2)) + "!" + \
                        " And for torso, please select a value from " + str(round(MIN_POSSIBLE_TORSO,2)) + " to " +  str(round(MIN_POSSIBLE_TORSO,2)) + "!"

            self.node.get_logger().error(message)

        return success, message
    
    def get_torso_position(self,  wait_for_end_of=True):

        return self.node.torso_position.legs_position, self.node.torso_position.torso_position
    
    def get_audio(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, question="", max_attempts=0, face_hearing="charmie_face_green", wait_for_end_of=True):

        if yes_or_no or receptionist or gpsr or restaurant:

            request = GetAudio.Request()
            request.yes_or_no = yes_or_no
            request.receptionist = receptionist
            request.gpsr = gpsr
            request.restaurant = restaurant

            # this code continuously asks for new audio info eveytime it gets an error for mishearing
            audio_error_counter = 0
            total_audio_counter = 0
            keywords = "ERROR"
            while keywords=="ERROR":
                
                self.set_speech(filename=question, wait_for_end_of=True)
                self.set_face(face_hearing)
                self.node.call_audio_server(request=request, wait_for_end_of=wait_for_end_of)
                
                if wait_for_end_of:
                    while not self.node.waited_for_end_of_audio:
                        pass
                self.node.waited_for_end_of_audio = False
                self.set_face("charmie_face")

                keywords = self.node.audio_command  

                if keywords=="ERROR":
                    total_audio_counter += 1

                    if max_attempts > 0: # this way if is set to 0, it is intended to infinetely hear until a correct keyword is heard
                        print("MAX HEAR ATTEMPTS: "+str(total_audio_counter)+"/"+str(max_attempts))
                        if total_audio_counter >= max_attempts: # exceeds the maximum number of predefined hearing opportunities
                            return "ERR_MAX"                            

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

    def get_continuous_audio(self, keywords=[], max_number_attempts=3, speak_pre_hearing=True, speak_post_hearing=True, face_hearing="charmie_face_green", wait_for_end_of=True):

        request = ContinuousGetAudio.Request()
        request.keywords = keywords
        request.max_number_attempts = max_number_attempts

        self.node.continuous_audio_detected_keyword = ""
           
        if speak_pre_hearing:
            self.set_speech(filename="audio/audio_continuous_start_"+str(random.randint(1, 5)), wait_for_end_of=True)
            for k in keywords:
                self.set_speech(command=k, wait_for_end_of=True)
        self.set_face(face_hearing)
            
        self.node.call_continuous_audio_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_continuous_audio:
                pass
            self.set_face("charmie_face")
            if speak_post_hearing:
                if self.node.continuous_audio_success:
                    self.set_speech(filename="audio/audio_continuous_stop", wait_for_end_of=True)
                    self.set_speech(command=self.node.continuous_audio_detected_keyword, wait_for_end_of=True)
                else: # timeout or error
                    self.set_speech(filename="audio/audio_continuous_timeout", wait_for_end_of=True)
        self.node.waited_for_end_of_continuous_audio = False

        return  self.node.continuous_audio_success, \
                self.node.continuous_audio_message, \
                self.node.continuous_audio_detected_keyword
    
    def is_get_continuous_audio_done(self, speak_post_hearing=True):

        if self.node.received_continuous_audio:
            self.node.received_continuous_audio = False
            self.set_face("charmie_face")
            if speak_post_hearing:
                if self.node.continuous_audio_success:
                    self.set_speech(filename="audio/audio_continuous_stop", wait_for_end_of=True)
                    self.set_speech(command=self.node.continuous_audio_detected_keyword, wait_for_end_of=True)
                else: # timeout or error
                    self.set_speech(filename="audio/audio_continuous_timeout", wait_for_end_of=True)
            return  True, \
                    self.node.continuous_audio_success, \
                    self.node.continuous_audio_message, \
                    self.node.continuous_audio_detected_keyword
        
        return False, False, "", ""
    
    def calibrate_audio(self, wait_for_end_of=True):

        request = CalibrateAudio.Request()
            
        self.node.call_calibrate_audio_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_calibrate_audio:
                pass
        self.node.waited_for_end_of_calibrate_audio = False

        return self.node.calibrate_audio_success, self.node.calibrate_audio_message 
    
    def get_sound_classification(self, question="", duration=0.0, score_threshold=-1.0, face_hearing="charmie_face_green", wait_for_end_of=True):

        request = GetSoundClassification.Request()
        request.duration = float(duration)
        request.score_threshold = float(score_threshold) # from 0 to 1, if by deafult -1 is used, sound_classification server default value is used

        self.set_speech(filename=question, wait_for_end_of=True)
        self.set_face(face_hearing)
        self.node.call_sound_classification_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_sound_classification:
                pass
        self.node.waited_for_end_of_sound_classification = False
        self.set_face("charmie_face")

        return self.node.sound_classification_labels, self.node.sound_classification_scores 
    
    def get_continuous_sound_classification(self, break_sounds=[], timeout=0.0, score_threshold=-1.0, speak_pre_hearing=True, speak_post_hearing=True, face_hearing="charmie_face_green", wait_for_end_of=True):
        
        request = GetSoundClassificationContinuous.Request()
        request.break_sounds = break_sounds
        request.timeout = float(timeout)
        request.score_threshold = float(score_threshold) # from 0 to 1, if by deafult -1 is used, sound_classification server default value is used

        self.node.continuous_sound_classification_detected_label = ""
        self.node.continuous_sound_classification_detected_score = 0.0
        
        if speak_pre_hearing:
            self.set_speech(filename="sound_classification/sound_classification_start_"+str(random.randint(1, 6)), wait_for_end_of=True)
        self.set_face(face_hearing)
        
        self.node.call_continuous_sound_classification_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_continuous_sound_classification:
                pass
            self.set_face("charmie_face")
            if speak_post_hearing:
                if self.node.continuous_sound_classification_success:
                    self.set_speech(filename="sound_classification/sound_classification_continuous_stop", wait_for_end_of=True)
                    self.set_speech(command=self.node.continuous_sound_classification_detected_label, wait_for_end_of=True)
                else: # timeout or error
                    self.set_speech(filename="sound_classification/sound_classification_continuous_timeout", wait_for_end_of=True)
        self.node.waited_for_end_of_continuous_sound_classification = False

        return  self.node.continuous_sound_classification_success, \
                self.node.continuous_sound_classification_message, \
                self.node.continuous_sound_classification_detected_label, \
                self.node.continuous_sound_classification_detected_score
        
    def is_get_continuous_sound_classification_done(self, speak_post_hearing=True):

        if self.node.received_continuous_sound_classification:
            self.node.received_continuous_sound_classification = False
            self.set_face("charmie_face")
            if speak_post_hearing:
                if self.node.continuous_sound_classification_success:
                    self.set_speech(filename="sound_classification/sound_classification_continuous_stop", wait_for_end_of=True)
                    self.set_speech(command=self.node.continuous_sound_classification_detected_label, wait_for_end_of=True)
                else: # timeout or error
                    self.set_speech(filename="sound_classification/sound_classification_continuous_timeout", wait_for_end_of=True)
            return  True, \
                    self.node.continuous_sound_classification_success, \
                    self.node.continuous_sound_classification_message, \
                    self.node.continuous_sound_classification_detected_label, \
                    self.node.continuous_sound_classification_detected_score
        
        return False, False, "", "", 0.0
    
    def wait_for_doorbell(self, timeout=20.0, score_threshold=0.1):

        # List of possible doorbell sounds
        doorbell_break_sounds=["alarm",
                               "alarm clock",
                               "beep, bleep",
                               "bell",
                               "bicycle bell",
                               "buzzer",
                               "chime",
                               "clock",
                               "ding-dong",
                               "doorbell",
                               "glockenspiel",
                               "mains hum",
                               "mallet percussion",
                               "marimba, xylophone",
                               "ringtone",
                               "telephone",
                               "telephone bell ringing",
                               "tubular bells",
                               "tuning fork",
                               "vibraphone",
                              ]
        
        # Speak specific for doorbell before starting listening for doorbell 
        self.set_speech(filename="sound_classification/doorbell_sound_classification_start_"+str(random.randint(1, 6)), wait_for_end_of=True)

        # Call sound classification continuous(with list of possible doorbell break sounds)
        success, message, label, score = self.get_continuous_sound_classification(break_sounds=doorbell_break_sounds, timeout=timeout, score_threshold=score_threshold, speak_pre_hearing=False, speak_post_hearing=False, wait_for_end_of=True)

        # Speak specific for doorbell after starting listening for doorbell if success:
        if success:
            self.set_speech(filename="sound_classification/doorbell_sound_classification_continuous_stop", wait_for_end_of=True)
        else: # timeout or error
            self.set_speech(filename="sound_classification/doorbell_sound_classification_continuous_timeout", wait_for_end_of=True)

        return success, message, label, score

    def set_face(self, command="", custom="", camera="", show_detections=False, wait_for_end_of=False):
        
        request = SetFace.Request()
        request.command = command
        request.custom = custom
        request.camera = camera
        request.show_detections = show_detections
        
        self.node.call_face_command_server(request=request, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_face:
                pass
        self.node.waited_for_end_of_face = False

        return self.node.face_success, self.node.face_message
    
    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])

        self.node.call_neck_position_server(request=request, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message
    
    def set_neck_coords(self, position=[0.0, 0.0, 0.0], wait_for_end_of=True):

        if len(position) == 3:

            request = SetNeckCoordinates.Request()
            request.coords.x = float(position[0])
            request.coords.y = float(position[1])
            request.coords.z = float(position[2])
            
            self.node.call_neck_coordinates_server(request=request, wait_for_end_of=wait_for_end_of)
        
            if wait_for_end_of:
                while not self.node.waited_for_end_of_neck_coords:
                    pass
            self.node.waited_for_end_of_neck_coords = False

        else:
            print("Something went wrong. Please check if position used is (x, y, z) and not (x, y)")

        return self.node.neck_success, self.node.neck_message
    
    def get_neck(self, wait_for_end_of=True):
    
        request=GetNeckPosition.Request()

        self.node.call_get_neck_position_server(request=request)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False


        return self.node.get_neck_position[0], self.node.get_neck_position[1] 
    
    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        request = ActivateYoloPose.Request()
        request.activate = activate
        request.only_detect_person_legs_visible = only_detect_person_legs_visible
        request.minimum_person_confidence = float(minimum_person_confidence)
        request.minimum_keypoints_to_detect_person = int(minimum_keypoints_to_detect_person)
        request.only_detect_person_right_in_front = only_detect_person_right_in_front
        request.only_detect_person_arm_raised = only_detect_person_arm_raised
        request.characteristics = characteristics

        self.node.call_activate_yolo_pose_server(request=request)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def activate_yolo_objects(self, activate_objects=False, activate_furniture=False, activate_objects_hand=False, activate_furniture_hand=False, activate_objects_base=False, activate_furniture_base=False, minimum_objects_confidence=0.5, minimum_furniture_confidence=0.5, wait_for_end_of=True):
        
        request = ActivateYoloObjects.Request()
        request.activate_objects             = activate_objects
        request.activate_furniture           = activate_furniture
        request.activate_objects_hand        = activate_objects_hand
        request.activate_furniture_hand      = activate_furniture_hand
        request.activate_objects_base        = activate_objects_base
        request.activate_furniture_base      = activate_furniture_base
        request.minimum_objects_confidence   = float(minimum_objects_confidence)
        request.minimum_furniture_confidence = float(minimum_furniture_confidence)

        self.node.call_activate_yolo_objects_server(request=request)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def activate_yolo_world(self, activate_prompt_free_head=False, activate_tv_prompt_head=False, activate_prompt_free_hand=False, activate_tv_prompt_hand=False, activate_prompt_free_base=False, activate_tv_prompt_base=False, minimum_prompt_free_confidence=0.5, minimum_tv_prompt_confidence=0.5, text_prompts=[]):
        request = ActivateYoloWorld.Request()
        request.activate_prompt_free_head       = activate_prompt_free_head
        request.activate_tv_prompt_head         = activate_tv_prompt_head
        request.activate_prompt_free_hand       = activate_prompt_free_hand
        request.activate_tv_prompt_hand         = activate_tv_prompt_hand
        request.activate_prompt_free_base       = activate_prompt_free_base
        request.activate_tv_prompt_base         = activate_tv_prompt_base
        request.minimum_prompt_free_confidence  = float(minimum_prompt_free_confidence)
        request.minimum_tv_prompt_confidence    = float(minimum_tv_prompt_confidence)
        request.text_prompts = text_prompts

        self.node.call_activate_yolo_world_server(request=request)

        self.node.activate_yolo_world_success = True
        self.node.activate_yolo_world_message = "Activated with selected parameters"

        return self.node.activate_yolo_world_success, self.node.activate_yolo_world_message

    def activate_motors(self, activate=True, wait_for_end_of=True):
        
        request = ActivateBool.Request()
        request.activate = activate

        self.node.call_activate_motors_server(request=request)

        self.node.activate_motors_success = True
        self.node.activate_motors_message = "Activated with selected parameters"

        return self.node.activate_motors_success, self.node.activate_motors_message

    def activate_tracking(self, activate=False, points=ListOfPoints(), bbox=BoundingBox(), wait_for_end_of=True):
        
        request = ActivateTracking.Request()
        request.activate = activate
        request.points = points
        request.bounding_box = bbox

        self.node.call_activate_tracking_server(request=request)

        self.node.activate_tracking_success = True
        self.node.activate_tracking_message = "Activated with selected parameters"

        return self.node.activate_tracking_success, self.node.activate_tracking_message

    def track_person(self, person=DetectedPerson(), body_part="Head", wait_for_end_of=True):

        request = TrackPerson.Request()
        request.person = person
        request.body_part = body_part

        self.node.call_neck_track_person_server(request=request, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_person:
            pass
        self.node.waited_for_end_of_track_person = False

        return self.node.track_person_success, self.node.track_person_message
 
    def track_object(self, object=DetectedObject(), wait_for_end_of=True):

        request = TrackObject.Request()
        request.object = object

        self.node.call_neck_track_object_server(request=request, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_object:
            pass
        self.node.waited_for_end_of_track_object = False

        return self.node.track_object_success, self.node.track_object_message   

    def set_arm(self, command="", linear_motion_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_tool_line_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_motion_values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_for_end_of=True):        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False

        ### convert the lists to floats (prevents ROS2 errors)
        linear_motion_pose = [float(x) for x in linear_motion_pose]
        move_tool_line_pose = [float(x) for x in move_tool_line_pose]
        joint_motion_values = [float(x) for x in joint_motion_values]

        # converts values that should be radians but we are using blockly standard so we expect values as degrees, which are then internally converted to radians 
        linear_motion_pose[3] = math.radians(linear_motion_pose[3])
        linear_motion_pose[4] = math.radians(linear_motion_pose[4])
        linear_motion_pose[5] = math.radians(linear_motion_pose[5])
       
        move_tool_line_pose[3] = math.radians(move_tool_line_pose[3])
        move_tool_line_pose[4] = math.radians(move_tool_line_pose[4])
        move_tool_line_pose[5] = math.radians(move_tool_line_pose[5])
    
        temp = ArmController()
        temp.command = command
        temp.linear_motion_pose  = linear_motion_pose
        temp.move_tool_line_pose = move_tool_line_pose
        temp.joint_motion_values = joint_motion_values
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
    
    def set_height_furniture_for_arm_manual_movements(self, height=0.0, wait_for_end_of=True):        

        request = SetFloat.Request()
        request.data = float(height)

        self.node.call_set_height_furniture_for_arm_manual_movement_server(request=request)

        self.node.set_height_furniture_for_arm_manual_movement_client_success = True
        self.node.set_height_furniture_for_arm_manual_movement_client_message = "Height set for arm manual movements"

        return self.node.set_height_furniture_for_arm_manual_movement_client_success, self.node.set_height_furniture_for_arm_manual_movement_client_message
    
    def set_navigation(self, movement="", target=[0.0, 0.0], max_speed=15.0, absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, avoid_people=False, wait_for_end_of=True):

        if movement.lower() != "move" and movement.lower() != "rotate" and movement.lower() != "orientate" and movement.lower() != "adjust" and movement.lower() != "adjust_obstacle" and movement.lower() != "adjust_angle" :   
            self.node.get_logger().error("WRONG MOVEMENT NAME: PLEASE USE: MOVE, ROTATE OR ORIENTATE.")

            self.node.navigation_success = False
            self.node.navigation_message = "Wrong Movement Name"

        else:

            self.node.flag_target_pos_check_answer = False
            while not self.node.flag_target_pos_check_answer:
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
                navigation.avoid_people = avoid_people
                navigation.adjust_distance = float(adjust_distance)
                navigation.adjust_direction = float(adjust_direction)
                navigation.adjust_min_dist = float(adjust_min_dist)
                navigation.max_speed = float(max_speed)

                self.node.flag_navigation_reached = False                
                self.node.target_pos_publisher.publish(navigation)
                time.sleep(0.1)

            if wait_for_end_of:
                while not self.node.flag_navigation_reached:
                    pass
                self.node.flag_navigation_reached = False

            self.node.navigation_success = True
            self.node.navigation_message = "Arrived at selected location"

        return self.node.navigation_success, self.node.navigation_message   

    def set_initial_position(self, initial_position, clear_costmaps=True):

        if initial_position is not None:

            initial_pose_correctly_received_by_nav2 = False 
            xy_min_error_for_initial_pose = 0.05 # 5 cm
            yaw_min_error_for_initial_pose = math.radians(5.0) # 5 degrees

            self.node.new_amcl_pose_msg = False
            while not initial_pose_correctly_received_by_nav2:
                print("Attempting Sending Initial Pose to Nav2...")
                initial_pose = PoseWithCovarianceStamped()

                initial_pose.header.frame_id = "map"
                initial_pose.header.stamp = self.node.get_clock().now().to_msg()
                initial_pose.pose.pose.position.x = float(initial_position[0])
                initial_pose.pose.pose.position.y = float(initial_position[1])
                initial_pose.pose.pose.position.z = float(0.0)
                q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(initial_position[2]))
                initial_pose.pose.pose.orientation.x = q_x
                initial_pose.pose.pose.orientation.y = q_y
                initial_pose.pose.pose.orientation.z = q_z
                initial_pose.pose.pose.orientation.w = q_w
                self.node.initialpose_publisher.publish(initial_pose)

                # Sometimes the initial_pose is not received by nav2, so it is necessary to check if it was received correctly
                # We do this by using /amcl_pose topic, which is the topic that receives the robot's pose from the amcl node
                # if /amcl_pose is not updated with the initial_pose coordinates means that nav2 did not receive the initial_pose
                # so we need to resend it.

                time.sleep(0.1)
                amcl_yaw = self.get_yaw_from_quaternion(self.node.amcl_pose.pose.pose.orientation.x, self.node.amcl_pose.pose.pose.orientation.y, self.node.amcl_pose.pose.pose.orientation.z, self.node.amcl_pose.pose.pose.orientation.w)
                initial_position_yaw_rad = math.radians(initial_position[2])
                
                # print(amcl_yaw, initial_position_yaw_rad)

                # Angle conversions because they may be the same angle but 2*math.pi apart
                while amcl_yaw < 0:
                    amcl_yaw += 2*math.pi
                while amcl_yaw >= 2*math.pi:
                    amcl_yaw -= 2*math.pi

                while initial_position_yaw_rad < 0:
                    initial_position_yaw_rad += 2*math.pi
                while initial_position_yaw_rad >= 2*math.pi:
                    initial_position_yaw_rad -= 2*math.pi

                # print(amcl_yaw, initial_position_yaw_rad)

                if self.node.new_amcl_pose_msg and \
                    initial_position[0]+xy_min_error_for_initial_pose > self.node.amcl_pose.pose.pose.position.x > initial_position[0]-xy_min_error_for_initial_pose and \
                    initial_position[1]+xy_min_error_for_initial_pose > self.node.amcl_pose.pose.pose.position.y > initial_position[1]-xy_min_error_for_initial_pose and \
                    initial_position_yaw_rad+yaw_min_error_for_initial_pose > amcl_yaw > initial_position_yaw_rad-yaw_min_error_for_initial_pose:

                    initial_pose_correctly_received_by_nav2 = True
                    print("Success Sending Initial Pose to Nav2!")

                else:
                    self.node.new_amcl_pose_msg = False
                    initial_pose_correctly_received_by_nav2 = False

            # Internal definition of NORTH for standardization of orientation from low_level  
            request = SetPoseWithCovarianceStamped.Request()
            request.pose = initial_pose
            self.node.call_internal_set_initial_position_define_north_command_server(request=request)

        else:

            print(" --- ERROR WITH RECEIVED INITIAL POSITION --- ")

    def move_to_position(self, move_coords, print_feedback=True, feedback_freq=1.0, wait_for_end_of=True):

        # Create a goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(move_coords[0])
        goal_msg.pose.pose.position.y = float(move_coords[1])
        goal_msg.pose.pose.position.z = float(0.0)
        q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(move_coords[2]))        
        goal_msg.pose.pose.orientation.x = q_x
        goal_msg.pose.pose.orientation.y = q_y
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w

        self.node.get_logger().info("Waiting for CHARMIE Nav2 server...")
        self.node.charmie_nav2_client_.wait_for_server()
        self.node.get_logger().info("CHARMIE Nav2 server is ON...")

        self.node.goal_handle_ = None
        self.node.nav2_goal_accepted = None
        self.node.nav2_status = GoalStatus.STATUS_UNKNOWN
        self.node.nav2_feedback = NavigateToPose.Feedback()

        # Send the goal
        # self.node.get_logger().info("Sending goal...")
        self.node.charmie_nav2_client_.send_goal_async(goal_msg, feedback_callback=self.node.nav2_client_goal_feedback_callback).add_done_callback(self.node.nav2_client_goal_response_callback)
        self.node.get_logger().info("CHARMIE Nav2 Goal Sent")

        while self.node.nav2_goal_accepted is None:
            time.sleep(0.05)
        
        success = self.node.nav2_goal_accepted
        message = ""

        if wait_for_end_of:

            feedback_freq = 1.0
            feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
            feedback_start_time = time.time()

            while self.node.nav2_status == GoalStatus.STATUS_UNKNOWN:

                if print_feedback:

                    if time.time() - feedback_start_time > feedback_timer_period:
                        feedback = self.node.nav2_feedback
                        current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
                        current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
                        current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
                        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                        estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
                        no_recoveries = str(feedback.number_of_recoveries)
                        distance_remaining = str(round(feedback.distance_remaining, 2))
                        print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
                        # self.get_logger().info(f"Feedback: {feedback}")
                        feedback_start_time = time.time()
                        
            if self.node.nav2_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("CHARMIE NAV2 RESULT: SUCCEEDED.")
                success = True
                message = "Successfully moved to position"
            # elif self.node.nav2_status == GoalStatus.STATUS_ABORTED:
            #     self.set_rgb(RED+BACK_AND_FORTH_8)
            #     self.node.get_logger().info("CHARMIE NAV2 RESULT: ABORTED.")
            #     success = False
            #     message = "Canceled moved to position"
            elif self.node.nav2_status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("CHARMIEN NAV2 RESULT: CANCELED.")
                success = False
                message = "Canceled moved to position"

            return success, message
        
        else:
            success = True
            message = "Sent Command to CHARMIE Nav2, not waiting for end of"
            return success, message

    def move_to_position_cancel(self):
        if self.node.goal_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.nav2_client_cancel_goal()

    def move_to_position_is_done(self):
        if self.node.nav2_status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False
        
    def move_to_position_follow_waypoints(self, move_coords = [], print_feedback=True, feedback_freq=1.0, wait_for_end_of=True):

        if move_coords:

            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = []

            for x, y, yaw in move_coords: 
                
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = float(0.0)
                q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(yaw)) # math.radians(initial_position[2]))        
                pose.pose.orientation.x = q_x
                pose.pose.orientation.y = q_y
                pose.pose.orientation.z = q_z
                pose.pose.orientation.w = q_w
                
                goal_msg.poses.append(pose)

            self.node.get_logger().info("Waiting for CHARMIE Nav2 Follow Waypoints server...")
            self.node.charmie_nav2_follow_waypoints_client_.wait_for_server()
            self.node.get_logger().info("CHARMIE Nav2 Follow Waypoints server is ON...")

            self.node.goal_follow_waypoints_handle_ = None
            self.node.nav2_follow_waypoints_goal_accepted = None
            self.node.nav2_follow_waypoints_status = GoalStatus.STATUS_UNKNOWN
            self.node.nav2_follow_waypoints_feedback = NavigateToPose.Feedback()

            # Send the goal
            # self.node.get_logger().info("Sending goal...")
            self.node.charmie_nav2_follow_waypoints_client_.send_goal_async(goal_msg, feedback_callback=self.node.nav2_follow_waypoints_client_goal_feedback_callback).add_done_callback(self.node.nav2_follow_waypoints_client_goal_response_callback)
            self.node.get_logger().info("CHARMIE Nav2 Follow Waypoints Goal Sent")

            while self.node.nav2_follow_waypoints_goal_accepted is None:
                time.sleep(0.05)
            
            success = self.node.nav2_follow_waypoints_goal_accepted
            message = ""

            if wait_for_end_of:

                feedback_freq = 1.0
                feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
                feedback_start_time = time.time()

                while self.node.nav2_follow_waypoints_status == GoalStatus.STATUS_UNKNOWN:

                    if print_feedback:

                        if time.time() - feedback_start_time > feedback_timer_period:
                            feedback = self.node.nav2_follow_waypoints_feedback
                            current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
                            current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
                            current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
                            navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                            estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
                            no_recoveries = str(feedback.number_of_recoveries)
                            distance_remaining = str(round(feedback.distance_remaining, 2))
                            print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
                            # self.get_logger().info(f"Feedback: {feedback}")
                            feedback_start_time = time.time()
                        
                if self.node.nav2_follow_waypoints_status == GoalStatus.STATUS_SUCCEEDED:
                    self.node.get_logger().info("CHARMIE NAV2 FOLLOW WAYPOINTS RESULT: SUCCEEDED.")
                    success = True
                    message = "Successfully moved to position"
                # elif self.node.nav2_follow_waypoints_status == GoalStatus.STATUS_ABORTED:
                #     self.set_rgb(RED+BACK_AND_FORTH_8)
                #     self.node.get_logger().info("CHARMIE NAV2 RESULT: ABORTED.")
                #     success = False
                #     message = "Canceled moved to position"
                elif self.node.nav2_follow_waypoints_status == GoalStatus.STATUS_CANCELED:
                    self.node.get_logger().info("CHARMIEN NAV2 FOLLOW WAYPOINTS RESULT: CANCELED.")
                    success = False
                    message = "Canceled moved to position"

                return success, message
            
            else:
                success = True
                message = "Sent Command to CHARMIE Nav2 Follow Waypoints, not waiting for end of"
                return success, message

    def move_to_position_follow_waypoints_cancel(self):
        if self.node.goal_follow_waypoints_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.nav2_follow_waypoints_client_cancel_goal()

    def move_to_position_follow_waypoints_is_done(self):
        if self.node.nav2_follow_waypoints_status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False
        
    def add_rotation_to_pick_position(self, move_coords):
        move_coords_copy = move_coords.copy()
        move_coords_copy[2]+=45.0
        return move_coords_copy

    def clear_navigation_costmaps(self):
        self.node.call_clear_nav2_costmaps()

    def move_to_position_with_safety_navigation(self, move_coords, print_feedback=True, feedback_freq=1.0, wait_for_end_of=True):

        self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)
        self.set_face(camera="head", show_detections=True)

        # Create a goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(move_coords[0])
        goal_msg.pose.pose.position.y = float(move_coords[1])
        goal_msg.pose.pose.position.z = float(0.0)
        q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(move_coords[2]))        
        goal_msg.pose.pose.orientation.x = q_x
        goal_msg.pose.pose.orientation.y = q_y
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w

        self.node.get_logger().info("Waiting for CHARMIE Nav2 Safety server...")
        self.node.charmie_nav2_safety_client.wait_for_server()
        self.node.get_logger().info("CHARMIE Nav2 Safety server is ON...")

        self.node.goal_safety_handle_ = None
        self.node.nav2_safety_goal_accepted = None
        self.node.nav2_safety_feedback = NavigateToPose.Feedback()
        self.node.nav2_safety_status = GoalStatus.STATUS_UNKNOWN 

        # Send the goal
        # self.node.get_logger().info("Sending goal...")
        self.node.charmie_nav2_safety_client.send_goal_async(goal_msg, feedback_callback=self.node.nav2_safety_client_goal_feedback_callback).add_done_callback(self.node.nav2_safety_client_goal_response_callback)
        self.node.get_logger().info("CHARMIE Nav2 Safety Goal Sent")

        while self.node.nav2_safety_goal_accepted is None:
            time.sleep(0.05)
        
        success = self.node.nav2_safety_goal_accepted
        message = ""

        if wait_for_end_of:

            feedback_freq = 1.0
            feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
            feedback_start_time = time.time()

            while self.node.nav2_safety_status == GoalStatus.STATUS_UNKNOWN:

                if print_feedback:

                    if time.time() - feedback_start_time > feedback_timer_period:
                        feedback = self.node.nav2_safety_feedback
                        current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
                        current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
                        current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
                        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                        estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
                        no_recoveries = str(feedback.number_of_recoveries)
                        distance_remaining = str(round(feedback.distance_remaining, 2))
                        print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
                        # self.get_logger().info(f"Feedback: {feedback}")
                        feedback_start_time = time.time()
            
            if self.node.nav2_safety_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("CHARMIE NAV2 SAFETY RESULT: SUCCEEDED.")
                success = True
                message = "Successfully moved to position"
            # elif self.node.nav2_safety_status == GoalStatus.STATUS_ABORTED:
            #     self.set_rgb(RED+BACK_AND_FORTH_8)
            #     self.node.get_logger().info("CHARMIE SAFETY NAV2 RESULT: ABORTED.")
            #     success = False
            #     message = "Canceled moved to position"
            elif self.node.nav2_safety_status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("CHARMIEN NAV2 SAFETY RESULT: CANCELED.")
                success = False
                message = "Canceled moved to position"

            self.set_face("charmie_face")
            self.activate_yolo_pose(activate=False)

            return success, message
        
        else:
            success = True
            message = "Sent Command to CHARMIE Nav2 Safety, not waiting for end of"
            return success, message
    
    def move_to_position_with_safety_navigation_cancel(self):
        if self.node.goal_safety_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.nav2_safety_client_cancel_goal()
        self.set_face("charmie_face")
        self.activate_yolo_pose(activate=False)

    def move_to_position_with_safety_navigation_is_done(self):
        if self.node.nav2_safety_status == GoalStatus.STATUS_SUCCEEDED:
            self.set_face("charmie_face")
            self.activate_yolo_pose(activate=False)
            return True
        else:
            return False        

    ### THIS FUNCTION IS HERE BUT IS NOT USED, NOW IS IN CHARMIE_NAVIGATION ###
    ### HOWEVER, DUE TO THE THREADING SYSTEM, THE DEBUG MODE DOES NOT WORK IN CHARMIE_NAVIGATION ###
    ### SO WE LEAVE THIS HERE JUST FOR DEBUGGING PURPOSES - USE DEBUG_INSPECTION_PERSON_DETECTION ###
    def safety_navigation_check_depth_head_camera(self, half_image_zero_or_near_percentage=0.6, full_image_near_percentage=0.3, near_max_dist=0.8):

        DEBUG = True

        overall = False
        depth_head_image_received, current_frame_depth_head = self.get_head_depth_image()
        
        if depth_head_image_received:
            
            height, width = current_frame_depth_head.shape
            current_frame_depth_head_half = current_frame_depth_head[height//2:height,:]
            
            # FOR THE FULL IMAGE
            tot_pixeis = height*width 
            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head > 0) & (current_frame_depth_head <= near_max_dist*1000)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_head > near_max_dist*1000) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            # FOR THE BOTTOM HALF OF THE IMAGE
            mask_zero_half = (current_frame_depth_head_half == 0)
            mask_near_half = (current_frame_depth_head_half > 0) & (current_frame_depth_head_half <= near_max_dist*1000)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_head_half > near_max_dist*1000) # just for debug
                blank_image_half = np.zeros((height//2,width,3), np.uint8)
                blank_image_half[mask_zero_half] = [255,255,255]
                blank_image_half[mask_near_half] = [255,0,0]
                blank_image_half[mask_remaining_half] = [0,0,255]
                    
            pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
            pixel_count_near_half = np.count_nonzero(mask_near_half)
            
            if DEBUG:
                cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
                cv2.imshow("New Img Distance Inspection", blank_image)
                cv2.waitKey(10)

            half_image_zero_or_near = False
            half_image_zero_or_near_err = 0.0
            
            full_image_near = False
            full_image_near_err = 0.0


            half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
            if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
                half_image_zero_or_near = True
                # print("BOTTOM HALF DEPTH IMAGE STOP!")
            
            full_image_near_err = pixel_count_near/tot_pixeis # zeros not used here as they can be data very far away and not just too close to the camera
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True
                # print("FULL DEPTH IMAGE STOP!")
            
            if half_image_zero_or_near or full_image_near:
                overall = True

            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)

        return overall
    
    def adjust_omnidirectional_position(self, dx, dy, ang_obstacle_check=45, safety=True, max_speed=0.05, tolerance=0.01, kp=1.5, enter_house_special_case=False, use_wheel_odometry=False, timeout=0.0, print_feedback=True, wait_for_end_of=True):

        # Create a goal
        adjust_msg = AdjustNavigationOmnidirectional.Goal()
        adjust_msg.dx =                         float(dx)
        adjust_msg.dy =                         float(dy)
        adjust_msg.ang_obstacle_check =         float(ang_obstacle_check)
        adjust_msg.safety =                     bool(safety)
        adjust_msg.max_speed =                  float(max_speed)
        adjust_msg.tolerance =                  float(tolerance)
        adjust_msg.kp =                         float(kp)
        adjust_msg.enter_house_special_case =   bool(enter_house_special_case)
        adjust_msg.use_wheel_odometry =         bool(use_wheel_odometry)
        adjust_msg.timeout =                    float(timeout)

        self.node.get_logger().info("Waiting for adjust navigation server...")
        self.node.adjust_navigation_omni_client.wait_for_server()
        self.node.get_logger().info("Adjust Navigation server is ON...")

        self.node.adjust_omni_navigation_handle_ = None
        self.node.adjust_omni_navigation_accepted = None
        self.node.adjust_omni_navigation_status = GoalStatus.STATUS_UNKNOWN
        self.node.adjust_omni_navigation_feedback = None

        # Send the goal
        # self.node.get_logger().info("Sending goal...")
        self.node.adjust_navigation_omni_client.send_goal_async(adjust_msg, feedback_callback=self.node.adjust_omni_navigation_client_goal_feedback_callback).add_done_callback(self.node.adjust_omni_navigation_client_goal_response_callback)
        self.node.get_logger().info("Adjust Navigation Goal Sent")

        while self.node.adjust_omni_navigation_accepted is None:
            time.sleep(0.05)
        
        success = self.node.adjust_omni_navigation_accepted
        message = ""

        if wait_for_end_of:

            feedback_freq = 1.0
            feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
            feedback_start_time = time.time()

            while self.node.adjust_omni_navigation_status == GoalStatus.STATUS_UNKNOWN:

                if print_feedback:

                    if time.time() - feedback_start_time > feedback_timer_period:
                        feedback = self.node.adjust_omni_navigation_feedback
                        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                        distance_remaining = str(round(feedback.distance_remaining, 2))
                        print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
                        # self.node.get_logger().info(f"Feedback: {feedback}")
                        feedback_start_time = time.time()
            
            if self.node.adjust_omni_navigation_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("ADJUST OMNI RESULT: SUCCEEDED.")
                success = True
                message = "Successfully moved to position"
            elif self.node.adjust_omni_navigation_status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().info("ADJUST OMNI RESULT: ABORTED.")
                success = False
                message = "Canceled moved to position"
            elif self.node.adjust_omni_navigation_status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("ADJUST OMNI RESULT: CANCELED.")
                success = False
                message = "Canceled moved to position"

            return success, message
        
        else:
            success = True
            message = "Sent Command to Adjust Omni, not waiting for end of"
            return success, message
    
    def adjust_omnidirectional_position_cancel(self):
        if self.node.adjust_omni_navigation_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.adjust_omni_navigation_client_cancel_goal()

    def adjust_omnidirectional_position_is_done(self):
        if self.node.adjust_omni_navigation_status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False

    def adjust_obstacles(self, distance=0.0, direction=0.0, ang_obstacle_check=45, max_speed=0.05, tolerance=0.01, kp=1.5, timeout=0.0, print_feedback=True, wait_for_end_of=True):

        # Create a goal
        adjust_msg = AdjustNavigationObstacles.Goal()
        adjust_msg.distance =           float(distance)
        adjust_msg.direction =          float(direction)
        adjust_msg.ang_obstacle_check = float(ang_obstacle_check)
        adjust_msg.max_speed =          float(max_speed)
        adjust_msg.tolerance =          float(tolerance)
        adjust_msg.kp =                 float(kp)
        adjust_msg.timeout =            float(timeout)

        self.node.get_logger().info("Waiting for adjust navigation server...")
        self.node.adjust_navigation_obstacle_client.wait_for_server()
        self.node.get_logger().info("Adjust Navigation server is ON...")

        self.node.adjust_obstacle_navigation_handle_ = None
        self.node.adjust_obstacle_navigation_accepted = None
        self.node.adjust_obstacle_navigation_status = GoalStatus.STATUS_UNKNOWN
        self.node.adjust_obstacle_navigation_feedback = None

        # Send the goal
        # self.node.get_logger().info("Sending goal...")
        self.node.adjust_navigation_obstacle_client.send_goal_async(adjust_msg, feedback_callback=self.node.adjust_obstacle_navigation_client_goal_feedback_callback).add_done_callback(self.node.adjust_obstacle_navigation_client_goal_response_callback)
        self.node.get_logger().info("Adjust Navigation Goal Sent")

        while self.node.adjust_obstacle_navigation_accepted is None:
            time.sleep(0.05)
        
        success = self.node.adjust_obstacle_navigation_accepted
        message = ""

        if wait_for_end_of:

            feedback_freq = 1.0
            feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
            feedback_start_time = time.time()

            while self.node.adjust_obstacle_navigation_status == GoalStatus.STATUS_UNKNOWN:

                if print_feedback:

                    if time.time() - feedback_start_time > feedback_timer_period:
                        feedback = self.node.adjust_obstacle_navigation_feedback
                        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                        distance_remaining = str(round(feedback.distance_remaining, 2))
                        print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
                        # self.node.get_logger().info(f"Feedback: {feedback}")
                        feedback_start_time = time.time()
            
            if self.node.adjust_obstacle_navigation_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("ADJUST OBSTACLE RESULT: SUCCEEDED.")
                success = True
                message = "Successfully moved to position"
            elif self.node.adjust_obstacle_navigation_status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().info("ADJUST OBSTACLE RESULT: ABORTED.")
                success = False
                message = "Canceled moved to position"
            elif self.node.adjust_obstacle_navigation_status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("ADJUST OBSTACLE RESULT: CANCELED.")
                success = False
                message = "Canceled moved to position"

            return success, message
        
        else:
            success = True
            message = "Sent Command to Adjust Obstacle, not waiting for end of"
            return success, message
            
    def adjust_obstacle_cancel(self):
        if self.node.adjust_obstacle_navigation_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.adjust_obstacle_navigation_client_cancel_goal()

    def adjust_obstacle_is_done(self):
        if self.node.adjust_obstacle_navigation_status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False

    def adjust_angle(self, angle=0.0, max_angular_speed=0.25, tolerance=1.0, kp=1.3, use_wheel_odometry=False, timeout=0.0, print_feedback=True, wait_for_end_of=True):

        # Create a goal
        adjust_msg = AdjustNavigationAngle.Goal()
        adjust_msg.angle =              float(angle)
        adjust_msg.max_angular_speed =  float(max_angular_speed)
        adjust_msg.tolerance =          float(tolerance)
        adjust_msg.kp =                 float(kp)
        adjust_msg.use_wheel_odometry = bool(use_wheel_odometry)
        adjust_msg.timeout =            float(timeout)

        self.node.get_logger().info("Waiting for adjust navigation server...")
        self.node.adjust_navigation_angle_client.wait_for_server()
        self.node.get_logger().info("Adjust Navigation server is ON...")

        self.node.adjust_angle_navigation_handle_ = None
        self.node.adjust_angle_navigation_accepted = None
        self.node.adjust_angle_navigation_status = GoalStatus.STATUS_UNKNOWN
        self.node.adjust_angle_navigation_feedback = None

        # Send the goal
        # self.node.get_logger().info("Sending goal...")
        self.node.adjust_navigation_angle_client.send_goal_async(adjust_msg, feedback_callback=self.node.adjust_angle_navigation_client_goal_feedback_callback).add_done_callback(self.node.adjust_angle_navigation_client_goal_response_callback)
        self.node.get_logger().info("Adjust Navigation Goal Sent")

        while self.node.adjust_angle_navigation_accepted is None:
            time.sleep(0.05)
        
        success = self.node.adjust_angle_navigation_accepted
        message = ""

        if wait_for_end_of:

            feedback_freq = 1.0
            feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
            feedback_start_time = time.time()

            while self.node.adjust_angle_navigation_status == GoalStatus.STATUS_UNKNOWN:

                if print_feedback:

                    if time.time() - feedback_start_time > feedback_timer_period:
                        feedback = self.node.adjust_angle_navigation_feedback
                        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                        distance_remaining = str(round(feedback.distance_remaining, 2))
                        print("Nav Time: " + navigation_time + " Distance Left:" + distance_remaining)
                        # self.node.get_logger().info(f"Feedback: {feedback}")
                        feedback_start_time = time.time()
            
            if self.node.adjust_angle_navigation_status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("ADJUST ANGLE RESULT: SUCCEEDED.")
                success = True
                message = "Successfully moved to position"
            elif self.node.adjust_angle_navigation_status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().info("ADJUST ANGLE RESULT: ABORTED.")
                success = False
                message = "Canceled moved to position"
            elif self.node.adjust_angle_navigation_status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("ADJUST ANGLE RESULT: CANCELED.")
                success = False
                message = "Canceled moved to position"

            return success, message
        
        else:
            success = True
            message = "Sent Command to Adjust Angle, not waiting for end of"
            return success, message
    
    def adjust_angle_cancel(self):
        if self.node.adjust_angle_navigation_handle_ is not None:
            self.set_rgb(RED+BACK_AND_FORTH_8)
        self.node.adjust_angle_navigation_client_cancel_goal()

    def adjust_angle_is_done(self):
        if self.node.adjust_angle_navigation_status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False

    def get_minimum_radar_distance(self, direction=0.0, ang_obstacle_check=45):

        # success = False
        # message = ""
        # min_radar_distance_to_robot_edge = None

        request = GetMinRadarDistance.Request()
        request.direction = float(direction)
        request.ang_obstacle_check = float(ang_obstacle_check)
        self.node.call_get_minimum_radar_distance_server(request=request)

        while not self.node.waited_for_end_of_get_minimum_radar_distance:
            pass
        self.node.waited_for_end_of_get_minimum_radar_distance = False

        success = self.node.get_minimum_radar_distance_value.success
        message = self.node.get_minimum_radar_distance_value.message
        min_radar_distance_to_robot_edge = self.node.get_minimum_radar_distance_value.min_radar_distance_to_robot_edge
        print(success, message, min_radar_distance_to_robot_edge)
        
        return success, message, min_radar_distance_to_robot_edge
    
    def search_for_person(self, tetas, time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, break_if_detect=False, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=False, only_detect_person_right_in_front=False):

        self.activate_yolo_pose(activate=True, characteristics=characteristics, only_detect_person_arm_raised=only_detect_person_arm_raised, only_detect_person_legs_visible=only_detect_person_legs_visible, only_detect_person_right_in_front=only_detect_person_right_in_front) 
        self.set_speech(filename="generic/search_people", wait_for_end_of=False)
        # self.set_rgb(WHITE+ALTERNATE_QUARTERS)
        self.set_face(camera="head", show_detections=True)

        total_person_detected = []
        person_detected = []
        people_ctr = 0

        ### MOVES NECK AND SAVES DETECTED PEOPLE ###
        
        for t in tetas:
            self.set_rgb(RED+SET_COLOUR)
            self.set_neck(position=t, wait_for_end_of=True)
            time.sleep(time_wait_neck_move_pre_each_frame)
            self.node.detected_people.persons = [] # clears detected_objects after receiving them to make sure the objects from previous frames are not considered again
            self.set_rgb(WHITE+SET_COLOUR)

            start_time = time.time()
            while (time.time() - start_time) < time_in_each_frame:        
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

                if break_if_detect and people_ctr > 0:
                    break

            # DEBUG
            # print("people in this neck pos:")
            # for people in person_detected:
            #     print(people.index, people.position_absolute.x, people.position_absolute.y)
        
            total_person_detected.append(person_detected.copy())
            # print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()

            if break_if_detect and people_ctr > 0:
                time.sleep(0.5)
                break

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

                        same_person_old_distance_center = abs(self.node.CAM_IMAGE_WIDTH/2 - same_person_old.body_center_x) 
                        same_person_new_distance_center = abs(self.node.CAM_IMAGE_WIDTH/2 - same_person_new.body_center_x) 

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
        
        self.set_face("charmie_face")    
        self.set_neck(position=[0, 0], wait_for_end_of=False)
        self.set_rgb(YELLOW+HALF_ROTATE)

        sfp_pub = ListOfDetectedPerson()
        # print("FILTERED:")
        for p in filtered_persons:
            sfp_pub.persons.append(p)
        #     print(p.index)
        self.node.search_for_person_detections_publisher.publish(sfp_pub)

        return filtered_persons

    def detected_person_to_face_path(self, person=DetectedPerson(), just_face=False, send_to_face=False):

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        
        cf = self.node.br.imgmsg_to_cv2(person.image_rgb_frame, "bgr8")
        img_path = ""

        if not just_face:
            just_person_image = cf[person.box_top_left_y:person.box_top_left_y+person.box_height, person.box_top_left_x:person.box_top_left_x+person.box_width]
            img_path = current_datetime + str(person.index)
            cv2.imwrite(self.node.complete_path_custom_face + img_path + ".jpg", just_person_image) 
            time.sleep(0.1)
        else:
            if person.is_box_head:
                just_person_image = cf[person.box_head_top_left_y:person.box_head_top_left_y+person.box_head_height, person.box_head_top_left_x:person.box_head_top_left_x+person.box_head_width]
                img_path = current_datetime + str(person.index) + "face"
                cv2.imwrite(self.node.complete_path_custom_face + img_path + ".jpg", just_person_image) 
                time.sleep(0.1)
        
        # print(img_path)
        
        # cv2.imshow("Search for Person", just_person_image)
        # cv2.waitKey(100)

        if send_to_face:
            self.set_face(custom=img_path)
        
        return img_path

    def search_for_objects(self, tetas, time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, list_of_objects = [], list_of_objects_detected_as = [], use_arm=False, detect_objects=False, detect_furniture=False, detect_objects_hand=False, detect_furniture_hand=False, detect_objects_base=False, detect_furniture_base=False):

        final_objects = []
        if not list_of_objects_detected_as:
            list_of_objects_detected_as = [None] * len(list_of_objects)
            
        mandatory_object_detected_flags = [False for _ in list_of_objects]
        # print(mandatory_object_detected_flags)
        DETECTED_ALL_LIST_OF_OBJECTS = False
        MIN_DIST_DIFFERENT_FRAMES = 0.3 # maximum distance for the robot to assume it is the same objects
        MIN_DIST_SAME_FRAME = 0.2
        is_break_list_of_objects = False

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
            objects_ctr = 0

            self.activate_yolo_objects(activate_objects=detect_objects,             activate_furniture=detect_furniture,
                                       activate_objects_hand=detect_objects_hand,   activate_furniture_hand=detect_furniture_hand,
                                       activate_objects_base=detect_objects_base,   activate_furniture_base=detect_furniture_base,
                                       minimum_objects_confidence=0.5,              minimum_furniture_confidence=0.5)
            self.set_speech(filename="generic/search_objects", wait_for_end_of=False)
            
            if detect_objects or detect_furniture:        
                self.set_face(camera="head", show_detections=True)
            elif detect_objects_hand or detect_furniture_hand:
                self.set_face(camera="hand", show_detections=True)
            elif detect_objects_base or detect_furniture_base:
                self.set_face(camera="base", show_detections=True)
                
            # self.set_rgb(WHITE+ALTERNATE_QUARTERS)
            
            ### MOVES NECK AND SAVES DETECTED OBJECTS ###
            for t in tetas:
                rgb_found_list_of_objects = False
                self.set_rgb(RED+SET_COLOUR)
                self.set_neck(position=t, wait_for_end_of=True)
                time.sleep(time_wait_neck_move_pre_each_frame)
                self.node.detected_objects.objects = [] # clears detected_objects after receiving them to make sure the objects from previous frames are not considered again
                self.set_rgb(WHITE+SET_COLOUR)

                start_time = time.time()
                while (time.time() - start_time) < time_in_each_frame:      

                    # if detect_objects: 
                    local_detected_objects = self.node.detected_objects
                    for temp_objects in local_detected_objects.objects:
                        
                        is_already_in_list = False
                        object_already_in_list = DetectedObject()
                        for object in objects_detected:

                            # filters by same index
                            if temp_objects.index == object.index and temp_objects.object_name == object.object_name and temp_objects.camera == object.camera:
                                is_already_in_list = True
                                object_already_in_list = object

                            # second filter: sometimes yolo loses the IDS and creates different IDS for same objects, this filters the duplicates
                            if temp_objects.object_name == object.object_name and temp_objects.camera == object.camera:
                                if temp_objects.position_absolute.x != 0 and temp_objects.position_absolute.y != 0 and temp_objects.position_absolute.z != 0:
                                    dist = math.dist((temp_objects.position_absolute.x, temp_objects.position_absolute.y, temp_objects.position_absolute.z), (object.position_absolute.x, object.position_absolute.y, object.position_absolute.z))
                                    if dist < MIN_DIST_SAME_FRAME:
                                        is_already_in_list = True
                                        object_already_in_list = object

                        if is_already_in_list:
                            objects_detected.remove(object_already_in_list)
                        # else:
                        elif temp_objects.index > 0: # debug
                            # print("added_first_time", temp_objects.index, temp_objects.position_absolute.x, temp_objects.position_absolute.y)
                            if list_of_objects: 
                                if not rgb_found_list_of_objects:
                                    for m_object in merged_lists:
                                        if temp_objects.object_name.replace(" ","_").lower() in m_object:
                                            rgb_found_list_of_objects = True
                                if rgb_found_list_of_objects:
                                    self.set_rgb(GREEN+SET_COLOUR)
                            else:
                                self.set_rgb(GREEN+SET_COLOUR)
                        
                        if temp_objects.index > 0:
                            objects_detected.append(temp_objects)
                            objects_ctr+=1


                    if list_of_objects: #only does this if there are items in the list of mandatory detection objects
                        
                        mandatory_ctr = 0
                        # for m_object in list_of_objects:
                        for m_object in merged_lists:
                            is_in_mandatory_list = False
                            
                            # checks for previou
                            for frame in range(len(total_objects_detected)):
                                for object in range(len(total_objects_detected[frame])):
                                    
                                    # compares to local detected frame
                                    # if total_objects_detected[frame][object].object_name.lower() == m_object.lower():
                                    if total_objects_detected[frame][object].object_name.replace(" ","_").lower() in m_object:
                                        is_in_mandatory_list = True
                                        # print(m_object, total_objects_detected[frame][object].object_name, total_objects_detected[frame][object].index, is_in_mandatory_list)
                        
                                    # compares to overall final detected objects (multiple observations of multiple tetas)
                                    for final_obj in final_objects:
                                        # if final_obj.object_name.lower() == m_object.lower():
                                        if final_obj.object_name.replace(" ","_").lower() in m_object:
                                            is_in_mandatory_list = True
                                            # print(m_object, final_obj.object_name, final_obj.index, is_in_mandatory_list)

                            # checks for current teta (added to stop doing the full time_in_each_frame if all objects have already been detected)
                            for curr_teta_obj in objects_detected:

                                if curr_teta_obj.object_name.replace(" ","_").lower() in m_object:
                                    is_in_mandatory_list = True


                            if is_in_mandatory_list:
                                mandatory_ctr += 1
                            # print(m_object, is_in_mandatory_list)

                        if mandatory_ctr == len(list_of_objects): # if all objects are already in the detected list 
                            is_break_list_of_objects = True
                            break

                # DEBUG
                # print("objects in this neck pos:")
                # for object in objects_detected:
                #     print(object.index, object.position_absolute.x, object.position_absolute.y)
            
                total_objects_detected.append(objects_detected.copy())
                # print("Total number of objects detected:", len(objects_detected), objects_ctr)
                objects_detected.clear()   
                
                if is_break_list_of_objects: 
                    break

            self.activate_yolo_objects(activate_objects=False, activate_furniture=False,
                                       activate_objects_hand=False, activate_furniture_hand=False,
                                       activate_objects_base=False, activate_furniture_base=False,
                                       minimum_objects_confidence=0.5, minimum_furniture_confidence=0.5)
            
            # DEBUG
            # print("TOTAL objects in this neck pos:")
            # for frame in total_objects_detected:
            #     for object in frame:
            #         conf = f"{object.confidence * 100:.0f}%"
            #         x_ = f"{object.position_absolute.x:4.2f}"
            #         y_ = f"{object.position_absolute.y:5.2f}"
            #         z_ = f"{object.position_absolute.z:5.2f}"
            #         print(f"{'ID:'+str(object.index):<7} {object.object_name:<17} {conf:<3} {object.camera} ({x_}, {y_}, {z_})")
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

                            if total_objects_detected[frame][object].object_name == filtered_objects[filtered].object_name and \
                                total_objects_detected[frame][object].camera == filtered_objects[filtered].camera and \
                                total_objects_detected[frame][object].index == filtered_objects[filtered].index :
                                        same_object_ctr+=1
                                        same_object_old = filtered_objects[filtered]
                                        same_object_new = total_objects_detected[frame][object]

                            if total_objects_detected[frame][object].object_name == filtered_objects[filtered].object_name and total_objects_detected[frame][object].camera == filtered_objects[filtered].camera: 

                                if total_objects_detected[frame][object].position_absolute.x != 0 and total_objects_detected[frame][object].position_absolute.y != 0 and total_objects_detected[frame][object].position_absolute.z:
                                    # dist_xy = math.dist((total_objects_detected[frame][object].position_absolute.x, total_objects_detected[frame][object].position_absolute.y), (filtered_objects[filtered].position_absolute.x, filtered_objects[filtered].position_absolute.y))
                                    dist = math.dist((total_objects_detected[frame][object].position_absolute.x, total_objects_detected[frame][object].position_absolute.y, total_objects_detected[frame][object].position_absolute.z), (filtered_objects[filtered].position_absolute.x, filtered_objects[filtered].position_absolute.y, filtered_objects[filtered].position_absolute.z))
                                    # print("new:", total_objects_detected[frame][object].index, total_objects_detected[frame][object].object_name, ", old:", filtered_objects[filtered].index, filtered_objects[filtered].object_name, ", dist:", round(dist,3)) # , dist_xy) 
                                    
                                    if dist < MIN_DIST_DIFFERENT_FRAMES:
                                        same_object_ctr+=1
                                        same_object_old = filtered_objects[filtered]
                                        same_object_new = total_objects_detected[frame][object]
                                        # print("SAME OBJEcCT")                        

                        # the criteria for selecting which image is saved, is which center pixel of an object is closer to center of the image (avoiding images where object may be cut) 
                        if same_object_ctr > 0:

                            image_center = (self.node.CAM_IMAGE_WIDTH/2, self.node.CAM_IMAGE_HEIGHT/2)
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
                conf = f"{o.confidence * 100:.0f}%"
                x_ = f"{o.position_absolute.x:4.2f}"
                y_ = f"{o.position_absolute.y:5.2f}"
                z_ = f"{o.position_absolute.z:5.2f}"
                print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({x_}, {y_}, {z_})")
            print()


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

        self.set_face("charmie_face")    
        self.set_neck(position=[0, 0], wait_for_end_of=False)
        self.set_rgb(YELLOW+HALF_ROTATE)

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

    def detected_object_to_face_path(self, object=DetectedObject(), send_to_face=False, bb_color=(0,0,255)):

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
    
        object_image = cf[max(object.box_top_left_y-thresh_v,0):min(object.box_top_left_y+object.box_height+thresh_v,self.node.CAM_IMAGE_HEIGHT), max(object.box_top_left_x-thresh_h,0):min(object.box_top_left_x+object.box_width+thresh_h,self.node.CAM_IMAGE_WIDTH)]
        # cv2.imshow("Search for Person", object_image)
        # cv2.waitKey(100)
        
        face_path = current_datetime + str(object.index) + " " + str(object.object_name) + " " + str(object.camera)
        face_path = face_path.replace(" ","_").lower()
        
        cv2.imwrite(self.node.complete_path_custom_face + face_path + ".jpg", object_image) 
        time.sleep(0.1)
        
        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path
    
    def get_point_cloud(self, camera, wait_for_end_of=True):

        # small example that must be continued adter point cloud is complete

        match camera:
            case "head":
                depth_img = self.node.depth_head_img
            case "hand":
                depth_img = self.node.depth_hand_img
            case "base":
                depth_img = self.node.depth_base_img
        
        self.node.point_cloud.convert_bbox_to_3d_point(depth_img=depth_img, camera=camera, bbox=None)

    def get_robot_localization(self):

        return self.node.robot_pose
    
    def get_gripper_localization(self):

        return self.node.gripper_point
    
    def get_base_gripper_localization(self):

        return self.node.base_gripper_point


    def get_head_rgb_image(self):

        if self.node.first_rgb_head_image_received:
            current_frame_rgb_head = self.node.br.imgmsg_to_cv2(self.node.rgb_head_img, "bgr8")
        else:
            current_frame_rgb_head = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH, 3), dtype=np.uint8)
        
        return self.node.first_rgb_head_image_received, current_frame_rgb_head

    def get_head_depth_image(self):

        if self.node.first_depth_head_image_received:
            current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_head_img, desired_encoding="passthrough")
        else:
            current_frame_depth_head = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH), dtype=np.uint8)
        
        return self.node.first_depth_head_image_received, current_frame_depth_head

    def get_hand_rgb_image(self):

        if self.node.first_rgb_hand_image_received:
            current_frame_rgb_hand = self.node.br.imgmsg_to_cv2(self.node.rgb_hand_img, "bgr8")
        else:
            current_frame_rgb_hand = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH, 3), dtype=np.uint8)
        
        return self.node.first_rgb_hand_image_received, current_frame_rgb_hand

    def get_hand_depth_image(self):

        if self.node.first_depth_hand_image_received:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
        else:
            current_frame_depth_hand = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH), dtype=np.uint8)
        
        return self.node.first_depth_hand_image_received, current_frame_depth_hand

    def get_base_rgb_image(self):

        if self.node.first_rgb_base_image_received:
            current_frame_rgb_base = self.node.br.imgmsg_to_cv2(self.node.rgb_base_img, "bgr8")
        else:
            current_frame_rgb_base = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH, 3), dtype=np.uint8)
        
        return self.node.first_rgb_base_image_received, current_frame_rgb_base

    def get_base_depth_image(self):

        if self.node.first_depth_base_image_received:
            current_frame_depth_base = self.node.br.imgmsg_to_cv2(self.node.depth_base_img, desired_encoding="passthrough")
        else:
            current_frame_depth_base = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH), dtype=np.uint8)
        
        return self.node.first_depth_base_image_received, current_frame_depth_base

    def update_gamepad_controller_state(self):

        self.node.previous_gamepad_controller_state = self.node.current_gamepad_controller_state
        self.node.current_gamepad_controller_state = self.node.gamepad_controller_state

        temp_has_new_message = self.node.new_gamepad_controller_msg
        self.node.new_gamepad_controller_msg = False

        return self.node.gamepad_controller_state, temp_has_new_message
    
    def get_gamepad_axis(self, axis_name):

        return self.node.current_gamepad_controller_state.axes[axis_name]

    def get_gamepad_button_pressed(self, button_name, button_status):

        match button_status:
            case self.ON:
                return self.node.current_gamepad_controller_state.buttons[button_name]
            case self.RISING:
                return self.node.current_gamepad_controller_state.buttons[button_name] \
                        and not self.node.previous_gamepad_controller_state.buttons[button_name]
            case self.OFF:
                return not self.node.current_gamepad_controller_state.buttons[button_name]
            case self.FALLING:
                return not self.node.current_gamepad_controller_state.buttons[button_name] \
                        and self.node.previous_gamepad_controller_state.buttons[button_name]
            
    def get_gamepad_timeout(self, button_status):

        match button_status:
            case self.ON:
                return self.node.current_gamepad_controller_state.timeout
            case self.RISING:
                return self.node.current_gamepad_controller_state.timeout \
                        and not self.node.previous_gamepad_controller_state.timeout
            case self.OFF:
                return not self.node.current_gamepad_controller_state.timeout
            case self.FALLING:
                return not self.node.current_gamepad_controller_state.timeout \
                        and self.node.previous_gamepad_controller_state.timeout

    def get_llm_demonstration(self, wait_for_end_of=True):

        self.calibrate_audio(wait_for_end_of=True)
        # self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
        random_question = str(random.randint(1, 3))
        command = self.get_audio(gpsr=True, question="demonstration/llm_get_question_"+random_question, wait_for_end_of=True)

        # add generic sentence so it is not so long quiet
        self.set_speech(filename="generic/uhm", wait_for_end_of=False)
        random_wait = str(random.randint(1, 3))
        self.set_speech(filename="demonstration/llm_wait_for_answer_"+random_wait, wait_for_end_of=False)

        request = GetLLMDemo.Request()
        request.command = command
        self.node.call_llm_demonstration_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_llm_demonstration:
                pass
            self.node.waited_for_end_of_llm_demonstration = False

        print(self.node.llm_demonstration_response)

        self.set_speech(command=self.node.llm_demonstration_response, quick_voice=True, wait_for_end_of=True)

    def get_llm_gpsr(self, wait_for_end_of=True):

        self.calibrate_audio(wait_for_end_of=True)
        # self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
        random_question = str(random.randint(1, 3))
        command = self.get_audio(gpsr=True, question="demonstration/llm_get_question_"+random_question, wait_for_end_of=True)

        # add generic sentence so it is not so long quiet
        self.set_speech(filename="generic/uhm", wait_for_end_of=False)
        random_wait = str(random.randint(1, 3))
        self.set_speech(filename="gpsr/llm_wait_for_gpsr_"+random_wait, wait_for_end_of=False)

        
        ### EXAMPLE FOR LLM CONFIRM COMMAND - SLENDER


        request = GetLLMConfirmCommand.Request()
        request.command = command
        self.node.call_llm_confirm_command_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_llm_confirm_command:
                pass
            self.node.waited_for_end_of_llm_confirm_command = False

        print(self.node.llm_confirm_command_response)

        self.set_speech(command=self.node.llm_confirm_command_response, quick_voice=True, wait_for_end_of=True)


        ### END OF EXAMPLE



        request = GetLLMGPSR.Request()
        request.command = command
        self.node.call_llm_gpsr_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_llm_gpsr:
                pass
            self.node.waited_for_end_of_llm_gpsr = False

        # print(self.node.llm_gpsr_response)
        for task in self.node.llm_gpsr_response.strings:
            task_split = task.split("-")
            
            print("Task type:", task_split[0], " Task info:", task_split[1])

            match task_split[0]:

                case "Navigation":
                    # self.set_navigation() ...
                    pass 

                case "SearchForObject":
                    # self.search_for_object() ...
                    pass
                
                case "SearchForPerson":
                    # self.search_for_person() ...
                    pass
                
                case "Speak":
                    # self.set_speech() ...
                    pass
                
                case "ArmPick":
                    # self.set_arm() ...
                    pass
                
                case "ArmPlace":
                    # self.set_arm() ...
                    pass

    def get_detected_person_characteristics(self, detected_person=DetectedPerson(), first_sentence="", ethnicity=False, age=False, gender=False, height=False, shirt_color=False, pants_color=False):

        # Still need to add percentages to ethnicity, gender and age_estimate. If too low than don't say.

        if detected_person.ethnicity != "None" and detected_person.ethnicity != "": # Minor corrections to value received
            if detected_person.ethnicity == "Middle Eastern" or detected_person.ethnicity == "Hispanic" or detected_person.ethnicity == "Indian":
                detected_person.ethnicity = "Caucasian"
        else: # If the robot can not compute, it guesses the following
            detected_person.ethnicity = "Caucasian"
        
        if detected_person.age_estimate != "None" and detected_person.age_estimate != "": # Minor corrections to value received
            if detected_person.age_estimate == "Over 60":
                detected_person.age_estimate = "Between 40 and 60"
            elif detected_person.age_estimate == "Under 20":
                detected_person.age_estimate = "Between 18 and 32"
            # detected_person.age_estimate = detected_person.age_estimate.replace(' ', '_')
        else: # If the robot can not compute, it guesses the following
            detected_person.age_estimate = "Between 18 and 32"
        detected_person.age_estimate = detected_person.age_estimate.replace(' ', '_')
        
        if detected_person.gender != "None" and detected_person.gender != "": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            detected_person.gender = "Male"
        
        # print("height is ", height)
        temp_height_string = ""
        if detected_person.height != 0.0: # Minor corrections to value received
            if detected_person.height > 1.55: 
                temp_height_string='taller'
            elif detected_person.height < 1.40:
                temp_height_string='smaller'
            else:
                temp_height_string='equal'
        else: # If the robot can not compute, it guesses the following
            temp_height_string = "taller"
        
        if detected_person.shirt_color != "None" and detected_person.shirt_color != "": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            detected_person.shirt_color = "White"
        
        if detected_person.pants_color != "None" and detected_person.pants_color != "": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            detected_person.pants_color = "Blue"
        
        if height or age or gender or ethnicity: # characteristics that require an introduction by a generic first sentence
            if first_sentence == "":
                self.set_speech(filename="receptionist/the_first_guest_is", wait_for_end_of=True)
            else:
                self.set_speech(filename=first_sentence, wait_for_end_of=True)
        
        if height:
            self.set_speech(filename="receptionist/characteristics/height_"+temp_height_string.lower(), wait_for_end_of=True)
        if age:
            self.set_speech(filename="receptionist/characteristics/age_"+detected_person.age_estimate.lower(), wait_for_end_of=True)
        if gender:
            self.set_speech(filename="receptionist/characteristics/gender_"+detected_person.gender.lower(), wait_for_end_of=True)
        if ethnicity:
            self.set_speech(filename="receptionist/characteristics/race_"+detected_person.ethnicity.lower(), wait_for_end_of=True)
        if shirt_color:
            self.set_speech(filename="receptionist/the_shirt_color_is", wait_for_end_of=True)
            self.set_speech(filename="receptionist/characteristics/color_"+detected_person.shirt_color.lower(), wait_for_end_of=True)
        if pants_color:
            self.set_speech(filename="receptionist/the_pants_color_is", wait_for_end_of=True)
            self.set_speech(filename="receptionist/characteristics/color_"+detected_person.pants_color.lower(), wait_for_end_of=True)


    def ask_help_pick_object_gripper(self, object_d=DetectedObject(), look_judge=[45, 0], wait_time_show_detection=0.0, wait_time_show_help_face=0.0, attempts_at_receiving=2, show_detection=True, alternative_help_pick_face = "", bb_color=(0, 255, 0)):

        object_name_for_files = object_d.object_name.replace(" ","_").lower()
        print("ask_help_pick_object_gripper:", object_name_for_files)

        if show_detection:
            self.detected_object_to_face_path(object=object_d, send_to_face=True, bb_color=bb_color)

        self.set_neck(position=look_judge, wait_for_end_of=False)

        if show_detection:
            self.set_speech(filename="generic/found_the", wait_for_end_of=False)
            self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=False)
            
            self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)

        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)

        if show_detection:
            # 3.0 is the amount of time necessary for previous speak to end, so 3 will always exist even if dont use sleep, 
            # this way is more natural, since it only opens the gripper before asking to receive object
            time.sleep(3.0 + wait_time_show_detection) 
        
        self.set_arm(command="open_gripper", wait_for_end_of=False)

        self.set_speech(filename="generic/check_face_put_object_hand_p1", wait_for_end_of=True)
        self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=True)
        self.set_speech(filename="generic/check_face_put_object_hand_p2", wait_for_end_of=True)
        
        if not alternative_help_pick_face:
            self.set_face("help_pick_"+object_name_for_files)
        else:
            self.set_face(alternative_help_pick_face)

        time.sleep(wait_time_show_help_face)
    
        object_in_gripper = False
        gripper_ctr = 0
        while not object_in_gripper and gripper_ctr < attempts_at_receiving:
            
            gripper_ctr += 1
            
            self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

            object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
            
            if not object_in_gripper:
        
                if gripper_ctr < attempts_at_receiving:

                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                
                self.set_arm(command="open_gripper", wait_for_end_of=False)

        if not object_in_gripper and gripper_ctr >= attempts_at_receiving:

            self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

        self.set_face("charmie_face")

        return object_in_gripper

    def ask_help_pick_object_tray(self, object_d=DetectedObject(), look_judge=[45, 0], first_help_request=False, wait_time_show_detection=0.0, wait_time_show_help_face=0.0, bb_color=(0, 255, 0), audio_confirmation=False):
    
        object_name_for_files = object_d.object_name.replace(" ","_").lower()
        # print("ask_help_pick_object_tray:", object_name_for_files)

        self.detected_object_to_face_path(object=object_d, send_to_face=True, bb_color=bb_color)

        self.set_neck(position=look_judge, wait_for_end_of=False)

        self.set_speech(filename="generic/found_the", wait_for_end_of=True)
        self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=True)
        
        self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)

        if first_help_request:
            time.sleep(0.5 + 2.5)
        else:
            time.sleep(0.5 + wait_time_show_detection)
            
        self.set_face("place_"+object_name_for_files+"_in_tray")

        self.set_speech(filename="generic/check_face_put_object_hand_p1", wait_for_end_of=True)
        self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=True)
        self.set_speech(filename="generic/check_face_put_object_tray_p2", wait_for_end_of=True)
        
        if first_help_request:
            time.sleep(0.5 + 2.5)
        else:
            time.sleep(0.5 + wait_time_show_help_face)

        self.set_face("charmie_face")

        confirmation = "yes"
        if audio_confirmation:

            if first_help_request:
                self.set_speech(filename="generic/hear_green_face", wait_for_end_of=True)
                self.set_speech(filename="generic/say_robot_yes_no", wait_for_end_of=True)
        
            ##### AUDIO: Listen "YES" OR "NO"
            confirmation = self.get_audio(yes_or_no=True, question="generic/question_detect_object_and_put_in_tray", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
            print("Finished:", confirmation)

        return confirmation
    
    def place_object(self, arm_command="", speak_before=False, speak_after=False, verb="", object_name="", preposition="", furniture_name=""):
        
        object_name_for_files = object_name.replace(" ","_").lower()
        # print("place_object_name:", object_name_for_files)
        furniture_name_for_files = furniture_name.replace(" ","_").lower()
        # print("place_object_furniture:", furniture_name_for_files)
        verb_name_for_files = verb.replace(" ","_").lower()
        # print("place_object_verb:", verb_name_for_files)
        preposition_name_for_files = preposition.replace(" ","_").lower()
        # print("place_object_preposition:", preposition_name_for_files)

        if speak_before:
            self.set_speech(filename="place_objects/going_to_"+verb_name_for_files, wait_for_end_of=False)
            self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=False)
            self.set_speech(filename="place_objects/"+preposition_name_for_files, wait_for_end_of=False)
            if verb_name_for_files == "place": 
                self.set_speech(filename="furniture/"+furniture_name_for_files, wait_for_end_of=False)
            else: # pour
                self.set_speech(filename="objects_names/"+furniture_name_for_files, wait_for_end_of=False)

        if arm_command: # checks if string is not empty, otherwise just speaks
            self.set_arm(command=arm_command, wait_for_end_of=True)

        if speak_after:
            self.set_speech(filename="place_objects/completed_"+verb_name_for_files, wait_for_end_of=False)
            self.set_speech(filename="objects_names/"+object_name_for_files, wait_for_end_of=False)
            self.set_speech(filename="place_objects/"+preposition_name_for_files, wait_for_end_of=False)
            if verb_name_for_files == "place": 
                self.set_speech(filename="furniture/"+furniture_name_for_files, wait_for_end_of=False)
            else: # pour
                self.set_speech(filename="objects_names/"+furniture_name_for_files, wait_for_end_of=False)

    def get_object_class_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return str(obj["class"]).replace(" ","_").lower()  # Return the class
        return None  # Return None if the object is not found
    
    def get_object_width_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return float(obj["width"])  # Return the value
        return None  # Return None if the object is not found
    
    def get_object_length_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return float(obj["length"])  # Return the value
        return None  # Return None if the object is not found
    
    def get_object_height_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return float(obj["height"])  # Return the value
        return None  # Return None if the object is not found
    
    def get_object_shape_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return str(obj["shape"]).replace(" ","_").lower()  # Return the shape
        return None  # Return None if the object is not found
    
    def get_how_object_can_be_picked_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return str(obj["can_pick"]).replace(" ","_").lower()  # Return the can_pick
        return None  # Return None if the object is not found
    
    def get_standard_pick_from_object(self, object_name):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_name).replace(" ","_").lower():  # Check if the name matches
                return str(obj["std_pick"]).replace(" ","_").lower()  # Return the std_pick
        return None  # Return None if the object is not found

    def get_furniture_from_object_class(self, object_class):

        # Iterate through the list of dictionaries
        for obj in self.node.objects_classes_file:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(object_class).replace(" ","_").lower():  # Check if the name matches
                return str(obj["location"]).replace(" ","_").lower()  # Return the class
        return None  # Return None if the object is not found

    def get_room_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return str(obj["room"]).replace(" ","_").lower()  # Return the class
        return None  # Return None if the object is not found

    # In python, lists are mutable, so we have to return a copy of the list to avoid any mistakes that modify the original one
    def get_navigation_coords_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return obj["nav_coords"].copy()  # Return the class
        return None  # Return None if the object is not found

    # In python, lists are mutable, so we have to return a copy of the list to avoid any mistakes that modify the original one
    def get_navigation_coords_from_room(self, room):

        # Iterate through the list of dictionaries
        for obj in self.node.rooms:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(room).replace(" ","_").lower():  # Check if the name matches
                return obj["nav_coords"].copy()  # Return the class
        return None  # Return None if the object is not found
    
    # In python, lists are mutable, so we have to return a copy of the list to avoid any mistakes that modify the original one (in this case is not necessary because I am creating a new list)
    def get_location_coords_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return [round((obj['top_left_coords'][0] + obj['bot_right_coords'][0])/2, 2), round((obj['top_left_coords'][1] + obj['bot_right_coords'][1])/2, 2), obj['height'][0]] # Return the class
        return None  # Return None if the object is not found
    
    def get_top_coords_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return obj['top_left_coords'].copy()
        return None  # Return None if the object is not found
    
    def get_bottom_coords_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return obj['bot_right_coords'].copy()
        return None  # Return None if the object is not found

    def get_height_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return obj['height'].copy() # Return the height
        return None  # Return None if the object is not found

    def get_look_orientation_from_furniture(self, furniture):

        # Iterate through the list of dictionaries
        for obj in self.node.furniture:
            # To make sure there are no errors due to spaces/underscores and upper/lower cases
            if str(obj["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():  # Check if the name matches
                return obj['look'] # Return the look
        return None  # Return None if the object is not found


    def set_continuous_tracking_with_coordinates(self):

        request = TrackContinuous.Request()

        ### TURN ON CONTINUOUS TRACKING
        request.status = True
        request.tracking_type = "person_head"
        request.tracking_position = Point()
        self.node.call_neck_continuous_tracking_server(request=request, wait_for_end_of=False)
        
        self.node.detected_people.persons = [] # clears detected_people after receiving them to make sure the objects from previous frames are not considered again
        # self.activate_yolo_pose(activate=True) 
        # self.activate_yolo_objects(activate_objects=True) 
        
        start_time = time.time()
        tracking_condition = True
        selected_object_to_track = "bowl"
        
        self.set_rgb(MAGENTA+ALTERNATE_QUARTERS)
        while tracking_condition:

            ### PERSON
            correct_track_per = DetectedPerson()
            local_detected_people = self.node.detected_people.persons
            if self.node.new_person_frame_for_tracking:
            
                if len(local_detected_people) > 0:
                    correct_track_per = local_detected_people[0]
                
                    # enviar valores 
                    coords = Point()
                    coords.x = float(correct_track_per.head_center_x)
                    coords.y = float(correct_track_per.head_center_y)
                    # coords.z = correct_person_to_track.position_absolute_head.z
                    self.node.continuous_tracking_position_publisher.publish(coords)
                    print(coords)
                self.node.new_person_frame_for_tracking = False

            ### OBJECTS
            # correct_track_obj = DetectedObject()  
            # local_detected_objects = self.node.detected_objects.objects
            # if self.node.new_object_frame_for_tracking:
            # 
            #     for o in local_detected_objects:
            #         if o.object_name.lower() == selected_object_to_track:
            #             correct_track_obj = o
            # 
            #     if correct_track_obj.object_name.lower() == selected_object_to_track:  
            #         # enviar valores 
            #         coords = Point()
            #         coords.x = float(correct_track_obj.box_center_x)
            #         coords.y = float(correct_track_obj.box_center_y)
            #         # coords.z = correct_person_to_track.position_absolute_head.z
            #         self.node.continuous_tracking_position_publisher.publish(coords)
            #         print(coords)
            #     self.node.new_object_frame_for_tracking = False

            # confirmar condição de fim de tracking
            # if time.time() - start_time > 60.0:
            #     tracking_condition = False

        self.set_rgb(CYAN+BREATH)
        self.activate_yolo_pose(activate=False) 
        self.activate_yolo_objects(activate_objects=False) 
        
        ### TURN OFF CONTINUOUS TRACKING
        request.status = False
        self.node.call_neck_continuous_tracking_server(request=request, wait_for_end_of=False)

    def set_follow_person(self):

        self.activate_yolo_pose(activate=True) 
        
        while len(self.node.detected_people.persons) == 0:
            pass

        p = self.node.detected_people.persons[0]

        self.activate_yolo_pose(activate=False) 

        points = ListOfPoints()
        
        if p.kp_nose_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_nose_x), y=float(p.kp_nose_y), z=1.0))
        if p.kp_eye_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_eye_left_x), y=float(p.kp_eye_left_y), z=1.0))
        if p.kp_eye_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_eye_right_x), y=float(p.kp_eye_right_y), z=1.0))
        if p.kp_ear_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_ear_left_x), y=float(p.kp_ear_left_y), z=1.0))
        if p.kp_ear_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_ear_right_x), y=float(p.kp_ear_right_y), z=1.0))
        if p.kp_shoulder_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_shoulder_left_x), y=float(p.kp_shoulder_left_y), z=1.0))
        if p.kp_shoulder_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_shoulder_right_x), y=float(p.kp_shoulder_right_y), z=1.0))
        if p.kp_elbow_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_elbow_left_x), y=float(p.kp_elbow_left_y), z=1.0))
        if p.kp_elbow_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_elbow_right_x), y=float(p.kp_elbow_right_y), z=1.0))
        if p.kp_wrist_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_wrist_left_x), y=float(p.kp_wrist_left_y), z=1.0))
        if p.kp_wrist_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_wrist_right_x), y=float(p.kp_wrist_right_y), z=1.0))
        if p.kp_hip_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_hip_left_x), y=float(p.kp_hip_left_y), z=1.0))
        if p.kp_hip_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_hip_right_x), y=float(p.kp_hip_right_y), z=1.0))
        if p.kp_knee_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_knee_left_x), y=float(p.kp_knee_left_y), z=1.0))
        if p.kp_knee_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_knee_right_x), y=float(p.kp_knee_right_y), z=1.0))
        if p.kp_ankle_left_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_ankle_left_x), y=float(p.kp_ankle_left_y), z=1.0))
        if p.kp_ankle_right_conf > 0.5:
            points.coords.append(Point(x=float(p.kp_ankle_right_x), y=float(p.kp_ankle_right_y), z=1.0))

        # points.coords.append(Point(x=640.0//2, y=480.0//2, z=1.0))
        # points.coords.append(Point(x=320.0, y=150.0, z=1.0))
        # points.coords.append(Point(x=420.0, y=150.0, z=0.0))
        # points.coords.append(Point(x=220.0, y=150.0, z=0.0))

        bb = BoundingBox()
        # bb.box_top_left_x = 200
        # bb.box_top_left_y = 100
        # bb.box_width = 640
        # bb.box_height = 480


        self.activate_tracking(activate=True, points=points, bbox=bb)
        # self.activate_tracking(activate=False)

    def set_face_touchscreen_menu(self, choice_category=[], custom_options=[], timeout=15.0, mode="single", instruction="", alphabetical_order=True, speak_results=True, start_speak_file="face_touchscreen_menu/init_touchscreen_menu", end_speak_file_error="face_touchscreen_menu/problem_touchscreen_menu", wait_for_end_of=True):

        options = []

        files_for_speech = []
        folder_for_speech = []

        object_classes = [] # this was added to have an automatic way to get all objects classes for the match case
        for obj_class in self.node.objects_classes_file:
            object_classes.append(obj_class["name"].lower())

        rooms = [] # this was added to have an automatic way to get all rooms for the match case
        for rms in self.node.rooms:
            rooms.append(rms["name"].lower())

        for c in choice_category:
            c = c.replace("_"," ").lower()
            match c:
                case _ if c in object_classes: # this way by just changing the configuration_files about objects_classes new categories are added without having to remember to manually change this std_function
                    files_for_speech.append(self.node.objects_file)
                    folder_for_speech.append("objects_names")
                    for obj in self.node.objects_file:
                        if obj["class"].lower() == c:
                            options.append(obj["name"])
                case _ if c in rooms: # this way by just changing the configuration_files about objects_classes new categories are added without having to remember to manually change this std_function
                    files_for_speech.append(self.node.furniture)
                    folder_for_speech.append("furniture")
                    for furnit in self.node.furniture:
                        if furnit["room"].lower() == c:
                            options.append(furnit["name"])
                case "names":
                    files_for_speech.append(self.node.names)
                    folder_for_speech.append("person_names")
                    for names in self.node.names:
                        options.append(names["name"])
                case "furniture":
                    files_for_speech.append(self.node.furniture)
                    folder_for_speech.append("furniture")
                    for obj in self.node.furniture:
                        options.append(obj["name"])
                case "rooms":
                    files_for_speech.append(self.node.rooms)
                    folder_for_speech.append("rooms")
                    for obj in self.node.rooms:
                        options.append(obj["name"])
                case "object classes":
                    files_for_speech.append(self.node.objects_classes_file)
                    folder_for_speech.append("objects_classes")
                    for obj in self.node.objects_classes_file:
                        options.append(obj["name"])
                case "custom":
                    for opt in custom_options:
                        options.append(opt)
                case _:
                    print("WRONG FACE TOUCHSCREEN MENU OPTION! DOES NOT EXIST!")
                    pass

        if alphabetical_order:
            options = sorted(options)
        # print("OPTIONS: ", options)
        
        if options or mode == "keyboard" or mode == "numpad": # keybord and numpad modes do not need options to work
            
            self.set_speech(filename=start_speak_file, wait_for_end_of=True)
    
            request = SetFaceTouchscreenMenu.Request()
            request.command     = options
            request.timeout     = float(timeout)
            request.mode        = str(mode)
            request.instruction = str(instruction)
            
            self.node.call_face_set_touchscreen_menu_server(request=request)

            if wait_for_end_of:
                while not self.node.waited_for_end_of_face_touchscreen_menu:
                    pass
            self.node.waited_for_end_of_face_touchscreen_menu = False

            if not self.node.selected_list_options_touchscreen_menu:
                self.set_speech(filename=end_speak_file_error, wait_for_end_of=True)
                return ["ERROR"]
            
            elif self.node.selected_list_options_touchscreen_menu[0] == "TIMEOUT":
                self.set_speech(filename=end_speak_file_error, wait_for_end_of=True)
                return self.node.selected_list_options_touchscreen_menu
        
            else:
                if speak_results:
                    self.set_speech(filename="face_touchscreen_menu/selected_touchscreen_menu", wait_for_end_of=True)
                    if mode == "keyboard" or mode == "numpad":
                        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                        self.save_speech(command=self.node.selected_list_options_touchscreen_menu[0].lower(), filename=current_datetime, quick_voice=False, play_command=True, show_in_face=True, wait_for_end_of=True)

                    else:
                        # function just for automatically search for the speak file amongst the sppech folder
                        said = False    
                        for so in self.node.selected_list_options_touchscreen_menu:
                            said = False
                            for file, folder in zip(files_for_speech, folder_for_speech):
                                # print(file)
                                for obj in file:
                                    # print(so.lower(), "| " ,obj["name"].lower())
                                    if so.lower() == obj["name"].lower():
                                        if not said:
                                            self.set_speech(filename=folder+"/"+so.replace(" ","_").lower())
                                            said = True

                return self.node.selected_list_options_touchscreen_menu
        
        else:
            print("FACE TOUCHSCREEN MENU SKIPPED! NO VALID OPTIONS!")
            self.set_speech(filename=end_speak_file_error, wait_for_end_of=True)
            return ["ERROR"]
        
    def add_face_to_face_recognition(self, person=DetectedPerson(), image_path="", name=""):

        success = False
        message = ""
        
        if image_path != "": # uses image_path
            image_path = self.node.home+'/'+image_path
        else: # uses DetectedPerson
            image_path = self.detected_person_to_face_path(person=person, just_face=True, send_to_face=False)
            image_path = self.node.complete_path_custom_face + image_path + ".jpg"

        # print(image_path)

        if os.path.isfile(image_path):

            image = face_recognition.load_image_file(image_path)
            encoding_entry = face_recognition.face_encodings(image)

            if len(encoding_entry) > 0:
                self.node.face_recognition_encoding.append(encoding_entry[0])
                self.node.face_recognition_names.append(name)
                success = True
                message = "Face encoding added to list successfully."
                self.node.get_logger().info("Face encoding added to list successfully. - " + str(name))
                print("Face encoding added to list successfully.")
                # print(self.node.face_recognition_encoding)
                # print(self.node.face_recognition_names)

            else:
                self.node.get_logger().warn("No face found in the image. Encodings not added to list.")
                print("No face found in the image. Encodings not added to list.")
                success = False
                message = "No face found in the image. Encodings not added to list."
        
        else:
            self.node.get_logger().warn("Image path does not exist. Encodings not added to list.")
            print("Image path does not exist. Encodings not added to list.")
            success = False
            message = "Image path does not exist. Encodings not added to list."

        return success, message

    def recognize_face_from_face_recognition(self, person=DetectedPerson(), image_path="", tolerance=0.4):
        
        if image_path != "": # uses image_path
            image_path = self.node.home+'/'+image_path
        else: # uses DetectedPerson
            image_path = self.detected_person_to_face_path(person=person, just_face=True, send_to_face=False)
            image_path = self.node.complete_path_custom_face + image_path + ".jpg"

        # print(image_path)

        if os.path.isfile(image_path):
    
            if not self.node.face_recognition_encoding:
                self.node.get_logger().warn("Face recognition encoding List is empty.")
                return "error", 0.0
            
            image = face_recognition.load_image_file(image_path)
            encoding_entry = face_recognition.face_encodings(image)

            if len(encoding_entry) == 0:
                self.node.get_logger().warn("No face found in the image. Could not comapre to encodings list.")
                return "error", 0.0
            encoding_entry = encoding_entry[0]  

            all_percentages = []
            for enc in self.node.face_recognition_encoding:
            
                distance = face_recognition.face_distance([enc], encoding_entry)[0]
                confidence = (1 - distance)
                all_percentages.append(confidence)

            name_recognized, biggest_conf_recognized = max(zip(self.node.face_recognition_names, all_percentages), key=lambda x: x[1])
            if biggest_conf_recognized < tolerance:
                name_recognized = "unknown"

            print("RECOGNITION COMPARE TABLE:")
            for prob, name in zip(all_percentages, self.node.face_recognition_names):
                print(str(round(prob,2))+" -> "+name)
            print("OUTCOME:", name_recognized, str(round(biggest_conf_recognized,2)))

            return name_recognized.lower(), biggest_conf_recognized

        else:
            self.node.get_logger().warn("Image path does not exist.")
            return "error", 0.0
            
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
		Convert an Euler angle to a quaternion.
		
		Input
			:param roll: The roll (rotation around x-axis) angle in radians.
			:param pitch: The pitch (rotation around y-axis) angle in radians.
			:param yaw: The yaw (rotation around z-axis) angle in radians.
		
		Output
			:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
		"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		
        #print(qx,qy,qz,qw)
  
        return [qx, qy, qz, qw]

    def get_yaw_from_quaternion(self, x, y, z, w):
        """ Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis). """
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians
    
    # Missing Functions:
    # 
    # count obj/person e specific conditions (in living room, in sofa, in kitchen table, from a specific class...)

    def pick_object(self, selected_object="", pick_mode="", first_search_tetas=[], furniture="", navigation = True, search_with_head_camera = True, return_arm_to_initial_position = True):

        ###########
        # Inputs:
        #
        # selected_object -> object name to grab
        # pick_mode -> if the robot should pick from the front or top, should be only "top" or "front"
        # first_search_tetas -> only relevant for using the head camera, will define what angles the head camera looks at to look for object
        # navigation -> if True the robot will use navigation to get closer to the object, should be left at True unless testing purposes or special cases
        # is_object_in_furniture_check -> if True will verify and ONLY PICK UP OBJECTS IN THEIR OBJECT CLASS' USUAL LOCATION, turn False only if dealing with objects on the ground or expected objects outside furniture, otherwise there can be false positives
        # search_with_head_camera -> if True the robot will use the head camera to locate objects, otherwise it will use base camera, turn to False if dealing with objects on floor. IF FALSE ALSO TURN PREVIOUS FLAG TO FALSE AS OBJECTS ON FLOOR WILL NEVER BE IN EXACT FURNITURE
        # return_arm_to_initial_position -> if True will return arm to initial position at the end, otherwise will leave gripper at ask_for_object position
        #
        # Outputs, in order of output:
        #
        # picked_height -> at which height the gripper picked the object at, ONLY CORRECT IF is_object_in_furniture_check AND ROBOT DIDNT RETURN asked_help TRUE
        # asked_help -> if robot asked for help anytime during the routine, IF TRUE picked_height WILL BE INCORRECT AS THERE WAS NOT A PICKED HEIGHT, ROBOT WAS HANDED THE OBJECT
        #
        ############
        
        # 1) Detect if the selected object is around and filter out additionals of the same in case of multiple being detected 

        valid_detected_object = DetectedObject()
        not_validated = True
        is_object_in_furniture_check = False
        selected_object = selected_object.replace(" ","_").lower()

        MIN_OBJECT_DISTANCE_X = 0.05
        MAX_OBJECT_DISTANCE_X = 2
        MIN_OBJECT_DISTANCE_Y = -1
        MAX_OBJECT_DISTANCE_Y = 1


        if furniture != "":
                is_object_in_furniture_check = True
        
        if first_search_tetas == []:

            if self.get_look_orientation_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))) == "horizontal":
                first_search_tetas = [[0, -45], [-40, -45], [40, -45]]

            elif self.get_look_orientation_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))) == "vertical":
                first_search_tetas = [[0, -15], [0, -35], [0, 15]]

        if pick_mode == "":
            pick_mode = self.get_standard_pick_from_object(selected_object)


        pick_mode = pick_mode.lower()
        furniture = furniture.replace(" ","_").lower()


        ### While cycle to get a valid detected object ###
        while not_validated:

            # If search_with_head_camera is true the first object detection will be made with the head camera, otherwise the robot will use the base camera instead
            if search_with_head_camera:
                self.set_face(camera="head", show_detections=True)
                objects_found = self.search_for_objects(tetas = first_search_tetas, time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, list_of_objects=[selected_object], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
            else:
                objects_found = self.search_for_objects(tetas = [[0.0,0.0]], time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, list_of_objects=[selected_object], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=True)
        
        
            print("LIST OF DETECTED OBJECTS:")


            for obj in objects_found:
                self.asked_help = False
                conf   = f"{obj.confidence * 100:.0f}%"
                cam_x_ = f"{obj.position_relative.x:5.2f}"
                cam_y_ = f"{obj.position_relative.y:5.2f}"
                cam_z_ = f"{obj.position_relative.z:5.2f}"

                print(f"{'ID:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({cam_x_},{cam_y_},{cam_z_} {obj.furniture_location})")


                object_location = (obj.furniture_location).replace(" ","_").lower()

                # In case the robot finds an object, if it is outside the designated furniture (verified if is_object_in_furniture_check = True) 
                # , it will try to reverse the order in which it searches for the object so as to hopefully not always see the unwanted object first 

                if selected_object == "" and obj.confidence >= 0.5 and cam_z_ < 0.4:
                        valid_detected_object = obj
                        not_validated = False


                if  obj.object_name == selected_object \
                    and object_location != furniture \
                    and is_object_in_furniture_check:

                    self.set_speech(filename="generic/Object_may_not_be_on_furniture.wav", wait_for_end_of=True)

                    first_search_tetas.reverse()
                        

                if obj.object_name == selected_object and MIN_OBJECT_DISTANCE_X < obj.position_relative.x < MAX_OBJECT_DISTANCE_X and MIN_OBJECT_DISTANCE_Y < obj.position_relative.y < MAX_OBJECT_DISTANCE_Y :
                    
                    if  (object_location == furniture and is_object_in_furniture_check) \
                        or is_object_in_furniture_check == False: 

                        if not_validated == False and (valid_detected_object.confidence < obj.confidence):
                            valid_detected_object = obj
                        elif not_validated == True:
                            valid_detected_object = obj

                        not_validated = False

        # 2) DEPENDING ON DETECTED OBJECT LOCATION, MOVE TOWARDS OBJECT (IF NAVIGATION = TRUE) AND POSITON ARM DEPENDING ON OBJECT HEIGHT
            
        if not_validated == False:

            # ANNOUNCE THE FOUND OBJECT
            self.set_speech(filename="generic/found_following_items", wait_for_end_of=False)
            self.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
            print(f"Initial pose to search for objects")

            # CONSTANTS NEEDED TO DECIDE ARM POSITIONS AND NAVIGATION, VALUES GOTTEN THROUGH TESTING, DO NOT CHANGE UNLESS NECESSARY !!!!!
            MAXIMUM_ADJUST_DISTANCE = 0.5 
            DISTANCE_IN_FRONT_X     = 0.6 
            DISTANCE_IN_FRONT_Y     = 0.3 
            DISTANCE_IN_TOP_X       = 0.58
            DISTANCE_IN_TOP_Y       = 0.15
            MINIMUM_FRONT_HEIGHT    = 0.55
            MAXIMUM_FRONT_HEIGHT    = 1.70
            HALFWAY_FRONT_HEIGHT    = 1.2 
            FLOOR_TOP_HEIGHT        = 0.3 
            HALFWAY_TOP_HEIGHT      = 0.6 
            MAXIMUM_TOP_HEIGHT      = 1.10
            
            if pick_mode == "front":

                print("OBJECT_HEIGHT", valid_detected_object.position_relative.z)

                # ASK FOR HELP IF OBJECT IS OUT OF ROBOT'S ARM RANGE
                if MINIMUM_FRONT_HEIGHT > valid_detected_object.position_relative.z or valid_detected_object.position_relative.z > MAXIMUM_FRONT_HEIGHT:
                    self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                    self.ask_help_pick_object_gripper(object_d=objects_found[0])
                    self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                    picked_height = 0.0
                    self.asked_help = True
                    return picked_height, self.asked_help

                self.set_arm(command="initial_pose_to_search_table_front", wait_for_end_of=False)

                # ADJUST ROBOT POSITION IN RELATION TO THE OBJECT
                if navigation:
                    self.adjust_x_      = valid_detected_object.position_relative.x - DISTANCE_IN_FRONT_X

                    if self.adjust_x_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = MAXIMUM_ADJUST_DISTANCE

                    elif self.adjust_x_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = -MAXIMUM_ADJUST_DISTANCE

                    self.adjust_y_      = valid_detected_object.position_relative.y + DISTANCE_IN_FRONT_Y

                    if self.adjust_y_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = MAXIMUM_ADJUST_DISTANCE

                    elif self.adjust_y_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = -MAXIMUM_ADJUST_DISTANCE

                    print("FINAL ADJUST:", self.adjust_x_, self.adjust_y_)

                    s,m = self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_)

                # IF ADJUST IS NOT POSSIBLE DUE TO OBSTACLES ASK FOR HELP
                if not s:
                    self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                    self.ask_help_pick_object_gripper(object_d=objects_found[0])
                    self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                    picked_height = 0.0
                    self.asked_help = True
                    return picked_height, self.asked_help


                # ADJUST ARM POSITION DEPENDING ON OBJECT HEIGHT
                if MINIMUM_FRONT_HEIGHT <= valid_detected_object.position_relative.z <= HALFWAY_FRONT_HEIGHT:

                    self.set_arm(command="search_front_min_z", wait_for_end_of=True)
                    gripper_search_height = self.get_gripper_localization().z
                    low_z = (valid_detected_object.position_relative.z - gripper_search_height)*100
                    print("LOW", low_z)
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = [low_z, 0.0, 0.0, 0.0, 0.0, 0.0], wait_for_end_of=True)
                    
                elif HALFWAY_FRONT_HEIGHT < valid_detected_object.position_relative.z <= MAXIMUM_FRONT_HEIGHT:

                    self.set_arm(command="search_front_max_z", wait_for_end_of=True)
                    gripper_search_height = self.get_gripper_localization().z
                    high_z = (valid_detected_object.position_relative.z - gripper_search_height)*100
                    print("HIGH", high_z)
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = [high_z, 0.0, 0.0, 0.0, 0.0, 0.0], wait_for_end_of=True)

                # BEGIN HAND SEARCH AND OBJECT GRAB, AND RETURN THE PICKED HEIGHT OR IF IT ASKED FOR HELP
                return self.hand_search(selected_object, pick_mode, navigation, return_arm_to_initial_position)

            #BEGIN PICK TOP IF SELECTED
            elif pick_mode == "top":

                # ADJUST TORSO AND ARM DEPENDING ON OBJECT HEIGHT
                if FLOOR_TOP_HEIGHT >= valid_detected_object.position_relative.z:

                    self.set_torso_position(legs=0, torso=61)

                elif HALFWAY_TOP_HEIGHT > valid_detected_object.position_relative.z:

                    self.set_arm(command="initial_pose_to_search_table_top", wait_for_end_of=False)
                    self.set_torso_position(legs=80, torso=8, wait_for_end_of=False) 
                    self.wait_until_camera_stable(timeout=120, check_interval=0.7, stable_duration=0.3, get_gripper=False)

                elif HALFWAY_TOP_HEIGHT < valid_detected_object.position_relative.z < MAXIMUM_TOP_HEIGHT:
                    self.set_arm(command="initial_pose_to_search_table_top", wait_for_end_of=False)

                # ADJUST ROBOT POSITION IN RELATION TO THE OBJECT
                if navigation:
                    self.adjust_x_      = valid_detected_object.position_relative.x - DISTANCE_IN_TOP_X

                    if self.adjust_x_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = MAXIMUM_ADJUST_DISTANCE

                    elif self.adjust_x_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = -MAXIMUM_ADJUST_DISTANCE

                    self.adjust_y_      = valid_detected_object.position_relative.y - DISTANCE_IN_TOP_Y

                    if self.adjust_y_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = MAXIMUM_ADJUST_DISTANCE

                    elif self.adjust_y_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = -MAXIMUM_ADJUST_DISTANCE

                    s,m = self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_)

                    print("Front X:", self.adjust_x_, " Front y:", self.adjust_y_)

                    # IF ADJUST IS NOT POSSIBLE DUE TO OBSTACLES ASK FOR HELP
                    if not s:
                        self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                        self.ask_help_pick_object_gripper(object_d=objects_found[0])
                        self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                        picked_height = 0.0
                        self.asked_help = True
                        return picked_height, self.asked_help
                
                    _, _ = self.adjust_angle(45)
                    #rotate_coordinates = self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(o.object_name))))
                    #self.move_to_position(move_coords=rotate_coordinates, wait_for_end_of=True)


                # BEGIN HAND SEARCH AND OBJECT GRAB, AND RETURN THE PICKED HEIGHT OR IF IT ASKED FOR HELP
                return self.hand_search(selected_object, pick_mode, navigation, return_arm_to_initial_position)

            else:
                    self.set_speech(filename="generic/could_not_find_any_objects", wait_for_end_of=True)

            #self.set_rgb(CYAN+HALF_ROTATE))
    
    def hand_search(self, selected_object, pick_mode, navigation, return_arm_to_initial_position):

        # 1) SEARCH FOR OBJECTS USING HAND CAMERA
        self.set_face(camera="hand", show_detections=True)
        table_objects = self.search_for_objects(tetas=[[0, 0]], time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[selected_object], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False)
        self.set_face(camera="hand", show_detections=True)

        #self.set_face(camera="hand",show_detections=True, wait_for_end_of=False)

        # print("LIST OF DETECTED OBJECTS:")
        # print(len(table_    '''objects))
        for obj in table_objects:
            ow = self.get_object_width_from_object(obj.object_name)
            oh = self.get_object_height_from_object(obj.object_name)
            conf = f"{obj.confidence * 100:.0f}%"

            #SAVE NEW X,Y,Z
            hand_x_ = f"{obj.position_cam.x:5.2f}"
            hand_y_ = f"{obj.position_cam.y:5.2f}"
            hand_z_ = f"{obj.position_cam.z:5.2f}"

            tf_x = 0.145
            tf_y = -0.006
            tf_z = -0.075

            print(f"{'ID:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} {obj.orientation} ({hand_x_},{hand_y_},{hand_z_})")
            if pick_mode == "front":
                correct_x = ((obj.position_cam.x - tf_x)*1000) - 200
            elif pick_mode == "top":
                correct_x = ((obj.position_cam.x - tf_x)*1000) - 200
                #if ((correct_x + 200) / 1000) <= self.get_height_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object))):
                    #correct_x = ( (1.012 - (self.get_height_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object)))) + tf_x ) * 1000) - 200
            correct_y = (obj.position_cam.y - tf_y)*1000
            correct_z = (obj.position_cam.z - tf_z)*1000

            #CORRECT ROTATION CALCULATIONS
            if obj.orientation < 0.0:
                correct_rotation = obj.orientation +90.0
            else:
                correct_rotation = obj.orientation -90.0

            #DEFINE AND CALCULATE KEY ARM POSITIONS
            if pick_mode == "top":
                object_position = [correct_z, -correct_y, correct_x, 0.0, 0.0, correct_rotation]
            elif pick_mode == "front":
                object_position = [correct_z, -correct_y, correct_x, 0.0, 0.0, 0.0]

            security_position_front   = [100.0*math.cos(math.radians(correct_rotation)), -100.0*math.sin(math.radians(correct_rotation)), -200.0, 0.0, 0.0, 0.0] #Rise the gripper in table orientation
            security_position_top     = [0.00, 0.0, -200.0, 0.0, 0.0, 0.0]
            object_reajust            = [0.0, 0.0, 0.0, 0.0, 0.0, -correct_rotation]
            initial_position_joints   = [-225.0, 83.0, -65.0, -1.0, 75.0, 270.0] 
            safe_top_second_joints    = [-197.5, 85.4, -103.3, 28.7, 86.1, 279.5]
            
            search_table_top_joints   = [-152.2, 59.4, -129.4, -85.2, 116.7, 66.7]
            search_table_front_joints = [-215.0, -70.0, -16.0, 80.0, 30.0, 182.0]
            
            #IF OBJECT FOUND
            if obj.object_name == selected_object:

                #OPEN GRIPPER
                self.set_arm(command="open_gripper", wait_for_end_of=True)

                #MOVE ARM IN THAT DIRECTION
                self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position, wait_for_end_of=True)
                self.wait_until_camera_stable() # Temporary measure, while wait_for_end_of is not working for adjust_move finish
                
                #CALIBRATE GRIPPER BEFORE GRABBING
                final_objects = self.search_for_objects(tetas=[[0, 0]], time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[selected_object], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False)
                self.set_face(camera="hand", show_detections=True)

                #self.set_face(camera="hand",show_detections=True,wait_for_end_of=False)
                for obj in final_objects:
                    conf = f"{obj.confidence * 100:.0f}%"
                    hand_y_grab    = f"{obj.position_cam.y:5.2f}"
                    hand_z_grab    = f"{obj.position_cam.z:5.2f}"
                    hand_x_grab    = f"{obj.position_cam.z:5.2f}"
                    correct_y_grab = (obj.position_cam.y - tf_y)*1000
                    correct_z_grab = (obj.position_cam.z - tf_z)*1000
                    if pick_mode == "front":
                        correct_x_grab = (obj.position_cam.x + ow/1.5 - tf_x)*1000
                        print("OBJECT WIDTH:", ow)

                    if pick_mode == "top":
                        correct_x_grab = (obj.position_cam.x + oh/1.5 - tf_x)*1000
                        
                        # To prevent the gripper from going so foward, the object would crash into the gripper itself, a limit is established. DO NOT CHANGE UNLESS TESTED

                        MAX_MOVE_LIMIT = 245
                        if correct_x_grab > MAX_MOVE_LIMIT:
                            correct_x_grab = MAX_MOVE_LIMIT
                    
                    print(f"{'BEFORE GRIP ID AND ADJUST:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({hand_y_grab}, {hand_z_grab}, {hand_x_grab})")
                    
                    if pick_mode == "front":
                        object_position_grab = [correct_z_grab, -correct_y_grab, correct_x_grab, 0.0, 0.0, 0.0]

                    if pick_mode == "top":

                        # THE FOLLOWING ARE SPECIAL CASES WHERE DIFFERENT MANUAL INFORMATION IS USED TO PICK THEM, DO NOT CHANGE UNLESS NECESSARY !!!!
                        if obj.object_name == "bowl":
                            correct_y_grab += 90
                        if obj.object_name == "cup":
                            correct_y_grab += 40
                            correct_x_grab = 210

                        object_position_grab = [0.0, -correct_y_grab, correct_x_grab, 0.0, 0.0, 0.0]

                #APPLY ADJUSTEMENT BEFORE GRABBING
                self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position_grab, wait_for_end_of=True)

                #MOVE ARM TO FINAL POSITION

                current_gripper_height = self.get_gripper_localization()
                height_furniture = self.get_shelf_from_height( object_height = current_gripper_height.z, furniture = self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object)))
                if (height_furniture >= 0):                                                  
                    picked_height = current_gripper_height.z - height_furniture
                else:
                    self.asked_help = True
                print("HEIGHT FURNITURE:", height_furniture)
                print("Picked Height: ", picked_height)

                #CHECK CLOSE GRIPPER
                
                object_in_gripper = False

                object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                if not object_in_gripper:
                    
                    self.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                    

                #MOVE TO SAFE POSITION DEPENDING ON MODE SELECTED

                if pick_mode == "front":
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = security_position_front, wait_for_end_of=False)
                    self.set_face("charmie_face", wait_for_end_of=False)

                    
                    if navigation:
                        self.adjust_x_ = - self.adjust_x_
                        self.adjust_y_ = - self.adjust_y_
                        self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_)

                    #MOVE TO SEARCH TABLE
                    self.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_front_joints, wait_for_end_of=True)

                    if not object_in_gripper:
                        self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))))
                        self.ask_help_pick_object_gripper(object_d=final_objects[0])
                        self.set_neck([0.0,0.0],wait_for_end_of=False)

                    #MOVE ARM TO INITIAL POSITION
                    if return_arm_to_initial_position:
                        self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                    else:
                        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)


                elif pick_mode == "top":

                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = security_position_top, wait_for_end_of=True)
                    #self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_reajust, wait_for_end_of=True)
                    #self.set_torso_position(legs=140, torso=8) 

                    self.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_top_joints, wait_for_end_of=False)
                    self.set_face("charmie_face", wait_for_end_of=False)


                    if navigation:

                        dx = self.adjust_x_
                        dy = self.adjust_y_
                        self.adjust_x_  = (- dx ) * math.cos(-math.radians(45)) - (- dy) * math.sin(-math.radians(45))
                        self.adjust_y_  = (- dx ) * math.sin(-math.radians(45)) + (- dy) * math.cos(-math.radians(45))

                        print("Reverse X:", self.adjust_x_, " Reverse y:", self.adjust_y_)

                        self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_)


                    if not object_in_gripper:
                        self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))))
                        self.ask_help_pick_object_gripper(object_d=final_objects[0])
                        self.set_neck([0.0,0.0],wait_for_end_of=False)
                    
                    if return_arm_to_initial_position:
                        self.set_arm(command="adjust_joint_motion", joint_motion_values = safe_top_second_joints, wait_for_end_of=True)
                        self.set_arm(command="adjust_joint_motion", joint_motion_values = initial_position_joints, wait_for_end_of=True)
                    else:
                        self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                    
                    #self.set_torso_position(legs=140, torso=8, wait_for_end_of=False) 
                    #self.wait_until_camera_stable(timeout=120, check_interval=0.7, stable_duration=0.3, get_gripper=False)            
                
                print(f"Bring object to initial pose")

                # Return the distance which the gripper was at in relation to the furniture
               
                return picked_height, self.asked_help

            #IF AN OBJECT WAS NOT FOUND
            else:
                self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                print(f"Could not bring object to initial pose")

    def pick_object_risky(self, selected_object="", pick_mode="", first_search_tetas=[], furniture="", navigation = True, search_with_head_camera = True, return_arm_to_initial_position = "", list_of_objects_detected_as = []):

        ###########
        # Inputs:
        #
        # selected_object -> object name to grab
        # pick_mode -> if the robot should pick from the front or top, should be only "top" or "front"
        # first_search_tetas -> only relevant for using the head camera, will define what angles the head camera looks at to look for object
        # navigation -> if True the robot will use navigation to get closer to the object, should be left at True unless testing purposes or special cases
        # is_object_in_furniture_check -> if True will verify and ONLY PICK UP OBJECTS IN THEIR OBJECT CLASS' USUAL LOCATION, turn False only if dealing with objects on the ground or expected objects outside furniture, otherwise there can be false positives
        # search_with_head_camera -> if True the robot will use the head camera to locate objects, otherwise it will use base camera, turn to False if dealing with objects on floor. IF FALSE ALSO TURN PREVIOUS FLAG TO FALSE AS OBJECTS ON FLOOR WILL NEVER BE IN EXACT FURNITURE
        # return_arm_to_initial_position -> if True will return arm to initial position at the end, otherwise will leave gripper at ask_for_object position
        #
        # Outputs, in order of output:
        #
        # picked_height -> at which height the gripper picked the object at, ONLY CORRECT IF is_object_in_furniture_check AND ROBOT DIDNT RETURN asked_help TRUE
        # asked_help -> if robot asked for help anytime during the routine, IF TRUE picked_height WILL BE INCORRECT AS THERE WAS NOT A PICKED HEIGHT, ROBOT WAS HANDED THE OBJECT
        #
        ############
        
        # 1) Detect if the selected object is around and filter out additionals of the same in case of multiple being detected 

        valid_detected_object = DetectedObject()
        not_validated = True
        is_object_in_furniture_check = False
        selected_object = selected_object.replace(" ","_").lower()

        MIN_OBJECT_DISTANCE_X = 0.05
        MAX_OBJECT_DISTANCE_X = 2
        MIN_OBJECT_DISTANCE_Y = -1
        MAX_OBJECT_DISTANCE_Y = 1


        if furniture != "":
                is_object_in_furniture_check = True
        
        if first_search_tetas == []:

            if self.get_look_orientation_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))) == "horizontal":
                first_search_tetas = [[0, -45], [-40, -45], [40, -45]]

            elif self.get_look_orientation_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))) == "vertical":
                first_search_tetas = [[0, -15], [0, -35], [0, 15]]

        if pick_mode == "":
            pick_mode = self.get_standard_pick_from_object(selected_object)


        pick_mode = pick_mode.lower()
        furniture = furniture.replace(" ","_").lower()


        ### While cycle to get a valid detected object ###
        while not_validated:

            # If search_with_head_camera is true the first object detection will be made with the head camera, otherwise the robot will use the base camera instead
            if search_with_head_camera:
                self.set_face(camera="head", show_detections=True)
                objects_found = self.search_for_objects(tetas = first_search_tetas, time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, list_of_objects=[selected_object], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
            else:
                objects_found = self.search_for_objects(tetas = [[0.0,0.0]], time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=1.0, list_of_objects=[selected_object], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=True)
        
        
            print("LIST OF DETECTED OBJECTS:")


            for obj in objects_found:
                asked_help = False
                conf   = f"{obj.confidence * 100:.0f}%"
                cam_x_ = f"{obj.position_relative.x:5.2f}"
                cam_y_ = f"{obj.position_relative.y:5.2f}"
                cam_z_ = f"{obj.position_relative.z:5.2f}"

                print(f"{'ID:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({cam_x_},{cam_y_},{cam_z_} {obj.furniture_location})")


                object_location = (obj.furniture_location).replace(" ","_").lower()

                # In case the robot finds an object, if it is outside the designated furniture (verified if is_object_in_furniture_check = True) 
                # , it will try to reverse the order in which it searches for the object so as to hopefully not always see the unwanted object first 

                if selected_object == "" and obj.confidence >= 0.5 and cam_z_ < 0.4:
                        valid_detected_object = obj
                        not_validated = False


                if  obj.object_name == selected_object \
                    and object_location != furniture \
                    and is_object_in_furniture_check:

                    self.set_speech(filename="generic/Object_may_not_be_on_furniture.wav", wait_for_end_of=True)

                    first_search_tetas.reverse()
                        

                if obj.object_name == selected_object and MIN_OBJECT_DISTANCE_X < obj.position_relative.x < MAX_OBJECT_DISTANCE_X and MIN_OBJECT_DISTANCE_Y < obj.position_relative.y < MAX_OBJECT_DISTANCE_Y :
                    
                    if  (object_location == furniture and is_object_in_furniture_check) \
                        or is_object_in_furniture_check == False: 

                        if not_validated == False and (valid_detected_object.confidence < obj.confidence):
                            valid_detected_object = obj
                        elif not_validated == True:
                            valid_detected_object = obj

                        not_validated = False

        # 2) DEPENDING ON DETECTED OBJECT LOCATION, MOVE TOWARDS OBJECT (IF NAVIGATION = TRUE) AND POSITON ARM DEPENDING ON OBJECT HEIGHT
            
        if not_validated == False:

            # ANNOUNCE THE FOUND OBJECT
            self.set_speech(filename="generic/found_following_items", wait_for_end_of=False)
            self.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
            print(f"Initial pose to search for objects")

            # CONSTANTS NEEDED TO DECIDE ARM POSITIONS AND NAVIGATION, VALUES GOTTEN THROUGH TESTING, DO NOT CHANGE UNLESS NECESSARY !!!!!
            MAXIMUM_ADJUST_DISTANCE = 0.5 
            DISTANCE_IN_FRONT_X     = 0.6 
            DISTANCE_IN_FRONT_Y     = 0.3 
            DISTANCE_IN_TOP_X       = 0.58
            DISTANCE_IN_TOP_Y       = -0.05
            MINIMUM_FRONT_HEIGHT    = 0.55
            MAXIMUM_FRONT_HEIGHT    = 1.70
            HALFWAY_FRONT_HEIGHT    = 1.2 
            FLOOR_TOP_HEIGHT        = 0.3 
            HALFWAY_TOP_HEIGHT      = 0.6 
            MAXIMUM_TOP_HEIGHT      = 1.10

            tf_x = 0.145
            tf_y = -0.006
            tf_z = -0.075
            ow = self.get_object_width_from_object(valid_detected_object.object_name)
            oh = self.get_object_height_from_object(valid_detected_object.object_name)
            
            if pick_mode == "front":

                print("OBJECT_HEIGHT", valid_detected_object.position_relative.z)

                # ASK FOR HELP IF OBJECT IS OUT OF ROBOT'S ARM RANGE
                if MINIMUM_FRONT_HEIGHT > valid_detected_object.position_relative.z or valid_detected_object.position_relative.z > MAXIMUM_FRONT_HEIGHT:
                    self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                    self.ask_help_pick_object_gripper(object_d=objects_found[0])
                    self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                    picked_height = 0.0
                    asked_help = True

                # ADJUST ROBOT POSITION IN RELATION TO THE OBJECT
                if navigation:
                    self.adjust_x_      = valid_detected_object.position_relative.x - DISTANCE_IN_FRONT_X

                    # if self.adjust_x_   > MAXIMUM_ADJUST_DISTANCE:
                    #     self.adjust_x_  = MAXIMUM_ADJUST_DISTANCE

                    # elif self.adjust_x_ < -MAXIMUM_ADJUST_DISTANCE:
                    #     self.adjust_x_  = -MAXIMUM_ADJUST_DISTANCE

                    self.adjust_y_      = valid_detected_object.position_relative.y + DISTANCE_IN_FRONT_Y

                    # if self.adjust_y_   > MAXIMUM_ADJUST_DISTANCE:
                    #     self.adjust_y_  = MAXIMUM_ADJUST_DISTANCE

                    # elif self.adjust_y_ < -MAXIMUM_ADJUST_DISTANCE:
                    #     self.adjust_y_  = -MAXIMUM_ADJUST_DISTANCE

                    print("FINAL ADJUST:", self.adjust_x_, self.adjust_y_)

                    s,m = self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_, wait_for_end_of=False)

                # IF ADJUST IS NOT POSSIBLE DUE TO OBSTACLES ASK FOR HELP
                if not s:
                    self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                    self.ask_help_pick_object_gripper(object_d=objects_found[0])
                    self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                    picked_height = 0.0
                    asked_help = True

                # self.set_arm(command="initial_pose_to_search_table_front", wait_for_end_of=True)

                # ADJUST ARM POSITION DEPENDING ON OBJECT HEIGHT
                if MINIMUM_FRONT_HEIGHT <= valid_detected_object.position_relative.z <= MAXIMUM_FRONT_HEIGHT:

                    self.set_arm(command="search_front_risky", wait_for_end_of=True)

                    gripper_search_height = self.get_gripper_localization().z
                    height_furniture = self.get_shelf_from_height(object_height = valid_detected_object.position_relative.z, furniture = valid_detected_object.furniture_location)
                    object_height = self.get_object_height_from_object(valid_detected_object.object_name)
                    adjust_z = (height_furniture + (object_height/2) - gripper_search_height)*1000

                    print("GRIPPER HEIGHT",gripper_search_height)
                    print("FURNITURE HEIGHT",height_furniture)
                    print("OBJECT HEIGHT",object_height)
                    print("ADJUST Z", adjust_z)

                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = [adjust_z, 0.0, 0.0, 0.0, 0.0, 0.0], wait_for_end_of=True)
                    

            #BEGIN PICK TOP IF SELECTED
            elif pick_mode == "top":

                if navigation:
                    _ , _ , furniture_distance = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                    self.adjust_x_      = furniture_distance - 0.04 

                    if self.adjust_x_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = MAXIMUM_ADJUST_DISTANCE   

                    elif self.adjust_x_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_x_  = -MAXIMUM_ADJUST_DISTANCE  

                    self.adjust_y_      = valid_detected_object.position_relative.y - DISTANCE_IN_TOP_Y 

                    if self.adjust_y_   > MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = MAXIMUM_ADJUST_DISTANCE   

                    elif self.adjust_y_ < -MAXIMUM_ADJUST_DISTANCE:
                        self.adjust_y_  = -MAXIMUM_ADJUST_DISTANCE

                    s,m = self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_, wait_for_end_of=False)

                # ADJUST TORSO AND ARM DEPENDING ON OBJECT HEIGHT
                if FLOOR_TOP_HEIGHT >= valid_detected_object.position_relative.z:

                    self.set_torso_position(legs=0, torso=61)

                elif HALFWAY_TOP_HEIGHT > valid_detected_object.position_relative.z:

                    self.set_arm(command="initial_pose_to_search_table_top", wait_for_end_of=False)
                    self.set_torso_position(legs=80, torso=8, wait_for_end_of=False) 
                    self.wait_until_camera_stable(timeout=120, check_interval=0.7, stable_duration=0.3, get_gripper=False)

                elif HALFWAY_TOP_HEIGHT < valid_detected_object.position_relative.z < MAXIMUM_TOP_HEIGHT:
                    search_table_top_risky_joints =			[-146.5, 55.7, -88, -61.3, 109.5, 64.2]
                    safe_top_second_joints    = [-197.5, 85.4, -103.3, 28.7, 86.1, 279.5]

                    #self.set_arm(command="adjust_joint_motion", joint_motion_values = safe_top_second_joints, wait_for_end_of=True)
                    #self.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_top_risky_joints, wait_for_end_of=True)
                    self.set_arm(command="initial_pose_to_search_table_top_risky", wait_for_end_of=True)


                    gripper_position = self.get_gripper_localization()
                    height_furniture = self.get_shelf_from_height( object_height = valid_detected_object.position_relative.z, furniture = valid_detected_object.furniture_location)
                    correct_x = (gripper_position.z - tf_x - oh - height_furniture)*1000 - 210
                    print("Gripper Position z: ",gripper_position.z," || Tf_X: ", tf_x, " || OH : ", oh, " || height_furniture", height_furniture, " || Correct_X: ", correct_x )
                    object_position = [0.0, 0.0, correct_x, 0.0, 0.0, 0.0]
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position, wait_for_end_of=True)
                    
                    while not self.adjust_omnidirectional_position_is_done():
                        pass

                # ADJUST ROBOT POSITION IN RELATION TO THE OBJECT

                    print("Front X:", self.adjust_x_, " Front y:", self.adjust_y_)

                    # IF ADJUST IS NOT POSSIBLE DUE TO OBSTACLES ASK FOR HELP
                    if not s:
                        self.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=False)
                        self.ask_help_pick_object_gripper(object_d=objects_found[0])
                        self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                        picked_height = 0.0
                        asked_help = True
                
                    #_, _ = self.adjust_angle(45)
                    #rotate_coordinates = self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(o.object_name))))
                    #self.move_to_position(move_coords=rotate_coordinates, wait_for_end_of=True)


                # BEGIN HAND SEARCH AND OBJECT GRAB, AND RETURN THE PICKED HEIGHT OR IF IT ASKED FOR HELP
                # return self.hand_search(selected_object, pick_mode, navigation, return_arm_to_initial_position)

            else:
                    self.set_speech(filename="generic/could_not_find_any_objects", wait_for_end_of=True)

            if not asked_help:

                # if pick_mode == "front":
                #     correct_x = ((obj.position_cam.x - tf_x)*1000) - 200
                #     correct_z = tf_z*1000
                    #if ((correct_x + 200) / 1000) <= self.get_height_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object))):
                        #correct_x = ( (1.012 - (self.get_height_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object)))) + tf_x ) * 1000) - 200
                
                #CORRECT ROTATION CALCULATIONS
                if valid_detected_object.orientation < 0.0:
                    correct_rotation = valid_detected_object.orientation +90.0
                else:
                    correct_rotation = valid_detected_object.orientation -90.0

                #DEFINE AND CALCULATE KEY ARM POSITIONS
                #if pick_mode == "top":
                # elif pick_mode == "front":
                #     object_position = [correct_z, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                security_position_front   = [100.0*math.cos(math.radians(correct_rotation)), -100.0*math.sin(math.radians(correct_rotation)), -200.0, 0.0, 0.0, 0.0] #Rise the gripper in table orientation
                security_position_top     = [0.00, 0.0, -200.0, 0.0, 0.0, 0.0]
                object_reajust            = [0.0, 0.0, 0.0, 0.0, 0.0, -correct_rotation]
                initial_position_joints   = [-225.0, 83.0, -65.0, -1.0, 75.0, 270.0] 
                safe_top_second_joints    = [-197.5, 85.4, -103.3, 28.7, 86.1, 279.5]
                
                search_table_top_joints   = [-151.5, 75, -123.2, -72.4, 110.8, 41.7]
                search_table_front_joints = [-215.0, -70.0, -16.0, 80.0, 30.0, 182.0]
                
                #OPEN GRIPPER
                if obj.object_name != "plate":
                    self.set_arm(command="open_gripper", wait_for_end_of=True)
                #MOVE ARM IN THAT DIRECTION
                #if pick_mode = "top":
                #self.wait_until_camera_stable(timeout = 0.75, stable_duration = 0.03, check_interval= 0.01, get_gripper = True) # Temporary measure, while wait_for_end_of is not working for adjust_move finish
                
                while not self.adjust_omnidirectional_position_is_done():
                    pass

                #CALIBRATE GRIPPER BEFORE GRABBING
                # self.wait_for_start_button()
                final_objects = self.search_for_objects(tetas=[[0, 0]], time_in_each_frame=2.0, time_wait_neck_move_pre_each_frame=0.0, list_of_objects=[selected_object], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False, list_of_objects_detected_as=list_of_objects_detected_as)
                self.set_face(camera="hand", show_detections=True)
                #self.set_face(camera="hand",show_detections=True,wait_for_end_of=False)
                
                for obj in final_objects:

                    conf = f"{obj.confidence * 100:.0f}%"
                    hand_y_grab    = f"{obj.position_cam.y:5.2f}"
                    hand_z_grab    = f"{obj.position_cam.z:5.2f}"
                    hand_x_grab    = f"{obj.position_cam.z:5.2f}"
                    correct_y_grab = (obj.position_cam.y - tf_y)*1000
                    correct_z_grab = (obj.position_cam.z - tf_z)*1000

                    if pick_mode == "front":
                        correct_x_grab = (obj.position_cam.x + ow/1.5 - tf_x)*1000
                        print("OBJECT WIDTH:", ow)
                    if pick_mode == "top":
                        correct_x_grab = (obj.position_cam.x + oh/1.4 - tf_x)*1000
                        
                        # To prevent the gripper from going so foward, the object would crash into the gripper itself, a limit is established. DO NOT CHANGE UNLESS TESTED
                        if pick_mode == "front":
                            MAX_MOVE_LIMIT = 245
                            if correct_x_grab > MAX_MOVE_LIMIT:
                                correct_x_grab = MAX_MOVE_LIMIT
                        if pick_mode == "top":
                            MAX_MOVE_LIMIT = 235
                            if correct_x_grab > MAX_MOVE_LIMIT:
                                correct_x_grab = MAX_MOVE_LIMIT
                    
                    print(f"{'BEFORE GRIP ID AND ADJUST:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({hand_y_grab}, {hand_z_grab}, {hand_x_grab})")
                    
                    if pick_mode == "front":
                        object_position_grab = [correct_z_grab, -correct_y_grab, correct_x_grab, 0.0, 0.0, 0.0]

                    if pick_mode == "top":
                        # THE FOLLOWING ARE SPECIAL CASES WHERE DIFFERENT MANUAL INFORMATION IS USED TO PICK THEM, DO NOT CHANGE UNLESS NECESSARY !!!!
                        if obj.object_name == "bowl":
                            correct_y_grab += 90
                            correct_rotation = 0.0
                        if obj.object_name == "cup":
                            correct_y_grab += 45
                            correct_x_grab = 210
                        if obj.object_name == "plate":
                            correct_z_grab -= 20
                            correct_rotation = 90.0
                            correct_x_grab -= 19
                            #navigation = gripper pos -  lowest y


                        object_position_grab = [correct_z_grab, -correct_y_grab, correct_x_grab, 0.0, 0.0, correct_rotation]

                #APPLY ADJUSTEMENT BEFORE GRABBING
                if obj.object_name != "cup":
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position_grab, wait_for_end_of=True)
                else:
                    object_position_grab = [correct_z_grab, -correct_y_grab, 0.0, 0.0, 0.0, correct_rotation]
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position_grab, wait_for_end_of=True)
                    object_position_grab = [0.0, 0.0, correct_x_grab, 0.0, 0.0, 0.0]
                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position_grab, wait_for_end_of=True)
                
                #MOVE ARM TO FINAL POSITION
                current_gripper_height = self.get_gripper_localization()

                height_furniture = self.get_shelf_from_height( object_height = current_gripper_height.z, furniture = self.get_furniture_from_object_class(self.get_object_class_from_object(object_name = selected_object)))
                if (height_furniture >= 0):                              

                    picked_height = current_gripper_height.z - height_furniture
                else:
                    asked_help = True

                print("HEIGHT FURNITURE:", height_furniture)
                print("Picked Height: ", picked_height)
                #CHECK CLOSE GRIPPER
                
                if obj.object_name != "plate":
                    object_in_gripper = False
                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    if not object_in_gripper:

                        self.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                    
                #MOVE TO SAFE POSITION DEPENDING ON MODE SELECTED
                if pick_mode == "front":
                    self.set_face("charmie_face", wait_for_end_of=False)
                    
                    if navigation:
                        self.adjust_x_ = - self.adjust_x_
                        self.adjust_y_ = - self.adjust_y_
                        self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_,wait_for_end_of=False)

                    #MOVE TO SEARCH TABLE
                    #self.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_front_joints, wait_for_end_of=True)

                    self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = security_position_front, wait_for_end_of=True)

                    if not object_in_gripper:
                        self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))))
                        self.ask_help_pick_object_gripper(object_d=final_objects[0])
                        self.set_neck([0.0,0.0],wait_for_end_of=False)

                    #MOVE ARM TO INITIAL POSITION
                    if return_arm_to_initial_position == "":
                        self.set_arm(command="search_front_risky_to_initial_pose", wait_for_end_of=True)
                    else:
                        if obj.object_name != "plate":    
                            self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                            self.set_arm(command=return_arm_to_initial_position, wait_for_end_of=True)

                    while not self.adjust_omnidirectional_position_is_done():
                        pass

                elif pick_mode == "top":
                    if obj.object_name != "plate":
                        print("OBJ NAME:", obj.object_name)
                        self.set_arm(command="search_table_top_risky", wait_for_end_of=True)
                    #self.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_top_risky_joints, wait_for_end_of=True)

                    if navigation:
                        if obj.object_name == "plate":

                            time.sleep(1.2)
                            plate_x_ = self.get_base_gripper_localization()
                            plate_adjust_x_ = 0.295 - plate_x_.x

                            print("Reverse X:", plate_adjust_x_, " Reverse y: 0.0", )
                            self.adjust_omnidirectional_position(dx = plate_adjust_x_, dy = 0.0, wait_for_end_of=True)

                            self.set_arm(command="pick_plate_top", wait_for_end_of=True)

                            _ , _ , furniture_distance = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                            self.adjust_x_ = - 0.59 + furniture_distance
                            self.adjust_y_ = - self.adjust_y_

                            plate_grab_fifth = [0.0, 0.0, 0.0, 00.0, -30.0, 120.0]
                            self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_,wait_for_end_of=False)
                            self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = plate_grab_fifth, wait_for_end_of=True)
                            plate_grab_sixth = [-176.5, 78.3, -98.2, -34.5, 92.2, 265.3]
                            self.set_arm(command="adjust_joint_motion", joint_motion_values = plate_grab_sixth, wait_for_end_of=True)
                            self.set_arm(command="adjust_joint_motion", joint_motion_values = initial_position_joints, wait_for_end_of=True)
                            #self.wait_for_start_button()

                            #self.wait_for_start_button()

                            while not self.adjust_omnidirectional_position_is_done():
                                pass

                            #plate_grab_final = [-190.0, 69.3, -70.5, 31.5, 64.1, 271]
                            #self.set_arm(command="adjust_joint_motion", joint_motion_values = plate_grab_final, wait_for_end_of=True)
                            object_in_gripper = True
                            #self.wait_for_start_button()

                            #self.wait_for_start_button()

                            return picked_height, asked_help
                            

                        else:
                            #dx = self.adjust_x_
                            #dy = self.adjust_y_
                            #self.adjust_x_  = (- dx ) * math.cos(-math.radians(0)) - (- dy) * math.sin(-math.radians(0))
                            #self.adjust_y_  = (- dx ) * math.sin(-math.radians(0)) + (- dy) * math.cos(-math.radians(0))
                            self.adjust_x_ = - self.adjust_x_
                            self.adjust_y_ = - self.adjust_y_
                            print("Reverse X:", self.adjust_x_, " Reverse y:", self.adjust_y_)
                            self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_, wait_for_end_of=False)

                    #self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_reajust, wait_for_end_of=True)
                    #self.set_torso_position(legs=140, torso=8) 
                    self.set_face("charmie_face", wait_for_end_of=False)

                    if not object_in_gripper and obj.object_name != "plate" :
                        self.add_rotation_to_pick_position(move_coords=self.get_navigation_coords_from_furniture(self.get_furniture_from_object_class(self.get_object_class_from_object(selected_object))))
                        self.ask_help_pick_object_gripper(object_d=final_objects[0])
                        self.set_neck([0.0,0.0],wait_for_end_of=False)
                    
                    if return_arm_to_initial_position == "":
                        self.set_arm(command="search_table_top_risky_to_initial_pose", wait_for_end_of=True)
                        #self.set_arm(command="adjust_joint_motion", joint_motion_values = safe_top_second_joints, wait_for_end_of=True)
                        #self.set_arm(command="adjust_joint_motion", joint_motion_values = initial_position_joints, wait_for_end_of=True)
                        while not self.adjust_omnidirectional_position_is_done():
                            pass

                    else:
                        if obj.object_name == "spoon" or obj.object_name == "knife" or obj.object_name == "fork":    
                            ###
                            self.set_arm(command="initial_pose_to_search_table_top_risky", wait_for_end_of=True)
                            self.set_arm(command=return_arm_to_initial_position, wait_for_end_of=True)
                            while not self.adjust_omnidirectional_position_is_done():
                                pass
                        else:
                            self.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                            self.set_arm(command=return_arm_to_initial_position, wait_for_end_of=True)
                            while not self.adjust_omnidirectional_position_is_done():
                                pass
                    #self.set_torso_position(legs=140, torso=8, wait_for_end_of=False) 
                    #self.wait_until_camera_stable(timeout=120, check_interval=0.7, stable_duration=0.3, get_gripper=False)            
                
                print(f"Bring object to initial pose")
                # Return the distance which the gripper was at in relation to the furniture
            
                return picked_height, asked_help

                #IF AN OBJECT WAS NOT FOUND
            else:
                self.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                print(f"Could not bring object to initial pose")
            #SEARCH FOR OBJECT

    
    def place_object_in_furniture(self, selected_object="", place_mode="", furniture="", shelf_number=0, asked_help = False, furniture_distance = -1.0, base_adjust_y = 0.0, place_height = -1.0):

        # CHECK OBJECT NAME FOR SPECIAL CASES
        # CHECK PICK/PLACE MODE
        # CHECK DISTANCE TO FURNITURE
        # MOVE ARM TO PLACE POSITION
        # ADJUST BASE IN RELATION TO THE FURNITURE
        # PLACE OBJECT
        # MOVE ARM TO SAFE POSITION

        selected_object = selected_object.replace(" ","_").lower()

        #### PLACE ARM POSITIONS ####

        self.arm_initial_position = [-225, 83, -65, -1, 75, 270]
        self.arm_safe_first = [ -177.2, 72.8, -112.8, -47.3, 105.7, 258.5]
        self.arm_safe_second = [-151.5, 75, -123.2, -72.4, 110.8, 41.7]


        #### VARIABLES ####

        if furniture == "":
            print(" YOU NEED TO DEFINE THE FURNITURE WHERE THE ROBOT IS GOING TO PLACE THE OBJECT !!!!!!!!!!")
            return
        
        else:
            verified = False
            for furn in self.node.furniture:
                if str(furn["name"]).replace(" ","_").lower() == str(furniture).replace(" ","_").lower():
                    verified = True
            if not verified:
                print("THAT FURNITURE DOES NOT EXIST, UNABLE TO PLACE OBJECT !!!")
                return
            
        print("Place 1:", place_mode)
        if place_mode == "":
            place_mode = self.get_standard_pick_from_object(selected_object)
            print("Place 2:", place_mode)
        furniture_height = self.get_height_from_furniture(furniture)

        if shelf_number < 0:
            furniture_height =  furniture_height[0]

        elif shelf_number <= len(furniture_height) - 1:
            furniture_height = furniture_height[shelf_number]

        elif shelf_number > len(furniture_height) -1:
            furniture_height = furniture_height[len(furniture_height)-1]

        if place_height < 0.0:
            asked_help = True

        #### CONSTANTS ####

        TOLERANCE_ERROR = 0.02
        FRONT_Z_ADJUST_LIMIT = 450

        if furniture_distance == -1:

            front_base_adjust_x = 0.15
            front_base_adjust_y = base_adjust_y

            top_base_adjust_x = 0.08
            top_base_adjust_y = base_adjust_y

        else:

            front_base_adjust_x = furniture_distance
            front_base_adjust_y = base_adjust_y

            top_base_adjust_x = furniture_distance
            top_base_adjust_y = base_adjust_y

        #### FUNCTION ####

        _ , _ , furniture_gap = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)

        if place_mode == "front":

            self.set_arm(command="initial_pose_to_place_front", wait_for_end_of=True)

            gripper_place_position = self.get_gripper_localization()

            if asked_help:
                final_z = (gripper_place_position.z - furniture_height - (self.get_object_height_from_object(selected_object)/1.25) - TOLERANCE_ERROR)*1000
            else:
                final_z = (gripper_place_position.z - furniture_height - place_height - TOLERANCE_ERROR)*1000

            print("Final_Z: ", final_z," Current Gripper Height:  ", gripper_place_position.z, " furniture z : ", furniture_height, " picked height : ", place_height)


            if final_z > FRONT_Z_ADJUST_LIMIT:
                final_z = FRONT_Z_ADJUST_LIMIT

            self.safe_place_final = [-final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
            self.safe_rise_gripper = [final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]

            self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)

            dx = furniture_gap - front_base_adjust_x
            dy = front_base_adjust_y 
            self.adjust_omnidirectional_position(dx=0.0,dy=dy, safety=True)
            self.adjust_omnidirectional_position(dx=dx, dy=0.0, safety=True)  

            time.sleep(0.5)
            self.set_arm(command="slow_open_gripper", wait_for_end_of=True)
            time.sleep(0.5)

            self.adjust_omnidirectional_position(dx=-dx,dy=0.0)
            self.adjust_omnidirectional_position(dx=0.0,dy=-dy) 

            self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)

            #self.set_arm(command="place_front_to_initial_pose", wait_for_end_of=True)

        elif place_mode == "top":

            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)

            gripper_place_position = self.get_gripper_localization()                                                                    

            dx = furniture_gap - top_base_adjust_x
            dy = top_base_adjust_y                                                                                                            

            self.adjust_omnidirectional_position(dx=0.0, dy=dy, safety=True)     
            self.adjust_omnidirectional_position(dx=dx, dy=0.0, safety=False)                                                     

            final_x = (gripper_place_position.z - furniture_height - place_height - 0.02)*1000                                     

            self.safe_place_final = [0.0 , 0.0 , final_x , 0.0 , 0.0 , 0.0]                                                         
            self.safe_rise_gripper = [0.0 , 0.0 , -final_x , 0.0 , 0.0 , 0.0]                                                       

            self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)        

            time.sleep(0.5)                                                                                                     
            self.set_arm(command="slow_open_gripper", wait_for_end_of=True)                                                     
            time.sleep(0.5)                                                                                                  

            self.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)    

            self.adjust_omnidirectional_position(dx=-dx,dy=-dy, wait_for_end_of=True)                                                                     

            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)
            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
            self.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)

            #self.set_arm(command="close_gripper", wait_for_end_of=True)
        

    def wait_until_camera_stable(self, timeout = 2.5, stable_duration = 0.4, check_interval= 0.1, get_gripper = True):

        #INITIATE VARIABLES REPRESENTING TIMER
        image_time_out = 0.0
        stable_image = 0.0
        threshold = 0.88

        #STAY IN WHILE LOOP UNTIL CALIBRATION
        while (stable_image <= stable_duration) and (image_time_out < timeout):

            #GET FIRST FRAME
            if get_gripper:
                _, prev_frame = self.get_hand_rgb_image()
            else:
                _, prev_frame = self.get_head_rgb_image()
            print("Waiting for camera to stabilize...", image_time_out, stable_image)

            #WAIT INTERVAL
            time.sleep(check_interval)

            #GET SECOND FRAME TO COMPARE
            if get_gripper:
                _, curr_frame = self.get_hand_rgb_image()
            else:
                _, curr_frame = self.get_head_rgb_image()
                
            is_stable = False

            #CONVERT IMG TO GRAYSCALE TO REDUCE LIGHT EFFECT
            prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

            #CALCULATE DIFFERENCE SCORE FROM LAST TO CURRENT FRAME
            score, _ = ssim(prev_gray, curr_gray, full=True)
            print("Score", score)

            #RETURN BOOLEAN ON IF SCORE IS ABOVE THRESHOLD
            if score >= threshold:
                is_stable = True

            #IF IMAGES ARE CLOSE BASED ON THRESHOLD ADD TO STABLE TIMER, IF NOT RESET STABLE TIMER 
            if (is_stable) and (image_time_out >= ( 7 * check_interval) ):
                stable_image += check_interval
            else:
                stable_image = 0.0
            image_time_out += check_interval

    def get_shelf_from_height(self, object_height = 0, furniture = ""):

        furniture_height = self.get_height_from_furniture(furniture)
        print("FURNITURE:", furniture)
        print("FURNITURE HEIGHTS:", furniture_height)
        print("OBJECT HEIGHT:", object_height)

        for h in furniture_height:
            print("HEIGHT COMPARATION:", h)
            if h < object_height:
                print("Altura do objeto:", object_height)
                print("Altura da shelf:", h)
                return float(h)
            
        print("Cannot get height from shelf")
        return False
