# import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Point
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedPerson, DetectedObject, TarNavSDNL, BoundingBox, BoundingBoxAndPoints, ListOfDetectedPerson, ListOfDetectedObject, Obstacles, ArmController
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger, SetFace, ActivateObstacles, GetPointCloud, SetAcceleration, NodesUsed, ContinuousGetAudio, SetRGB, GetVCCs, GetLowLevelButtons, GetTorso, SetTorso, ActivateBool

import cv2 
# import threading
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
    

class ROS2TaskNode(Node):

    def __init__(self, ros2_modules):
        super().__init__("ROS2TaskCHARMIE")
        self.get_logger().info("Initialised CHARMIE ROS2Task Node")

        self.ros2_modules = ros2_modules

        # path to save detected people in search for person
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        # Intel Realsense Subscribers 
        # Head
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        # Hand
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)  
        # Low Level
        self.torso_pos_publisher = self.create_publisher(Pose2D, "torso_pos", 10)
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.objects_filtered_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered', self.object_detected_filtered_callback, 10)
        self.objects_filtered_hand_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered_hand', self.object_detected_filtered_hand_callback, 10)
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
        self.continuous_get_audio_client = self.create_client(ContinuousGetAudio, "continuous_audio")
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
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")
        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")
        # Low level
        self.set_acceleration_ramp_client = self.create_client(SetAcceleration, "set_acceleration_ramp")
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode")
        self.get_vccs_client = self.create_client(GetVCCs, "get_vccs")
        self.get_low_level_buttons_client = self.create_client(GetLowLevelButtons, "get_start_button")
        self.get_torso_position_client = self.create_client(GetTorso, "get_torso_position")
        self.set_torso_position_client = self.create_client(SetTorso, "set_torso_position")
        self.activate_motors_client = self.create_client(ActivateBool, "activate_motors")
        #GUI
        self.nodes_used_client = self.create_client(NodesUsed, "nodes_used_gui")

    
        self.send_node_used_to_gui()

        """
            "charmie_arm": False,
            "charmie_audio": True,
            "charmie_face": False,
        "charmie_head_camera": False,
        "charmie_hand_camera": False,
        "charmie_lidar": False,
        "charmie_localisation": False,
            "charmie_low_level": False,
            "charmie_navigation": False,
            "charmie_neck": False,
            "charmie_obstacles": False,
        "charmie_odometry": False,
            "charmie_point_cloud": False,
        "charmie_ps4_controller": False,
            "charmie_speakers": False,
            "charmie_yolo_objects": False,
            "charmie_yolo_pose": False,
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

        if self.ros2_modules["charmie_low_level"]:
            while not self.set_acceleration_ramp_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Low Level Acceleration Command...")
            while not self.set_rgb_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Low Level RGB Command...")

        if self.ros2_modules["charmie_navigation"]:
            while not self.nav_trigger_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Navigation Trigger Command...")

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

        if self.ros2_modules["charmie_obstacles"]:
            while not self.activate_obstacles_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Activate Obstacles Command...")

        if self.ros2_modules["charmie_point_cloud"]:
            while not self.point_cloud_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Point Cloud...")

        if self.ros2_modules["charmie_speakers"]:
            while not self.speech_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Speech Command...")
            while not self.save_speech_command_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Save Speech Command...")

        if self.ros2_modules["charmie_yolo_objects"]:
            while not self.activate_yolo_objects_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")

        if self.ros2_modules["charmie_yolo_pose"]:
            while not self.activate_yolo_pose_client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        
        
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_continuous_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_save_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_arm = False
        self.waited_for_end_of_face = False
        self.waited_for_end_of_get_vccs = False
        self.waited_for_end_of_get_low_level_buttons = False
        self.waited_for_end_of_get_torso_position = False
        self.waited_for_end_of_set_torso_position = False
        self.waiting_for_pcloud = False

        self.br = CvBridge()
        self.rgb_head_img = Image()
        self.rgb_hand_img = Image()
        self.depth_head_img = Image()
        self.depth_hand_img = Image()
        self.first_rgb_head_image_received = False
        self.first_rgb_hand_image_received = False
        self.first_depth_head_image_received = False
        self.first_depth_hand_image_received = False
        self.detected_people = ListOfDetectedPerson()
        self.detected_objects = ListOfDetectedObject()
        self.detected_objects_hand = ListOfDetectedObject()
        self.flag_navigation_reached = False
        self.point_cloud_response = GetPointCloud.Response()
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
        self.torso_success = True
        self.torso_message = ""
        self.audio_success = True
        self.audio_message = ""
        self.continuous_audio_success = True
        self.continuous_audio_message = ""
        self.calibrate_audio_success = True
        self.calibrate_audio_message = ""
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
        self.activate_motors_success = True
        self.activate_motors_message = ""

        self.audio_command = ""
        self.received_continuous_audio = False

        self.get_neck_position = [1.0, 1.0]
        self.legs_position = 0.0
        self.torso_position = 0.0
        self.battery_voltage = 0.0
        self.emergency_stop = False
        self.start_button  = False
        self.debug_button1 = False
        self.debug_button2 = False
        self.debug_button3 = False


    def send_node_used_to_gui(self):

        nodes_used = NodesUsed.Request()

        nodes_used.charmie_arm              = self.ros2_modules["charmie_arm"]
        nodes_used.charmie_audio            = self.ros2_modules["charmie_audio"]
        nodes_used.charmie_face             = self.ros2_modules["charmie_face"]
        nodes_used.charmie_head_camera      = self.ros2_modules["charmie_head_camera"]
        nodes_used.charmie_hand_camera      = self.ros2_modules["charmie_hand_camera"]
        nodes_used.charmie_lidar            = self.ros2_modules["charmie_lidar"]
        nodes_used.charmie_localisation     = self.ros2_modules["charmie_localisation"]
        nodes_used.charmie_low_level        = self.ros2_modules["charmie_low_level"]
        nodes_used.charmie_navigation       = self.ros2_modules["charmie_navigation"]
        nodes_used.charmie_neck             = self.ros2_modules["charmie_neck"]
        nodes_used.charmie_obstacles        = self.ros2_modules["charmie_obstacles"]
        nodes_used.charmie_odometry         = self.ros2_modules["charmie_odometry"]
        nodes_used.charmie_point_cloud      = self.ros2_modules["charmie_point_cloud"]
        nodes_used.charmie_ps4_controller   = self.ros2_modules["charmie_ps4_controller"]
        nodes_used.charmie_speakers         = self.ros2_modules["charmie_speakers"]
        nodes_used.charmie_yolo_objects     = self.ros2_modules["charmie_yolo_objects"]
        nodes_used.charmie_yolo_pose        = self.ros2_modules["charmie_yolo_pose"]

        self.nodes_used_client.call_async(nodes_used)

    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)

    def object_detected_filtered_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects = det_object

    def object_detected_filtered_hand_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects_hand = det_object

    def get_color_image_head_callback(self, img: Image):
        self.rgb_head_img = img
        self.first_rgb_head_image_received = True
        # print("Received HEAD RGB Image")

    def get_color_image_hand_callback(self, img: Image):
        self.rgb_hand_img = img
        self.first_rgb_hand_image_received = True
        # print("Received HAND RGB Image")   
    
    def get_aligned_depth_image_head_callback(self, img: Image):
        self.depth_head_img = img
        self.first_depth_head_image_received = True
        # print("Received HEAD Depth Image")

    def get_aligned_depth_image_hand_callback(self, img: Image):
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

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, request=GetPointCloud.Request()):
    
        future = self.point_cloud_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud)

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_response = future.result()
            self.waiting_for_pcloud = False
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    ### ACTIVATE YOLO POSE SERVER FUNCTIONS ###
    def call_activate_yolo_pose_server(self, request=ActivateYoloPose.Request()):

        self.activate_yolo_pose_client.call_async(request)


    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, request=ActivateYoloObjects.Request()):

        self.activate_yolo_objects_client.call_async(request)


    ### ACTIVATE OBSTACLES SERVER FUNCTIONS ###
    def call_activate_obstacles_server(self, request=ActivateObstacles.Request()):

        self.activate_obstacles_client.call_async(request)


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
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.continuous_audio_success = response.success
            self.continuous_audio_message = response.message
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

    #### LOW LEVEL SERVER FUNCTIONS #####
    def call_rgb_command_server(self, request=SetRGB.Request(), wait_for_end_of=True):
        
        self.set_rgb_client.call_async(request)
        
        self.rgb_success = True
        self.rgb_message = "Value Sucessfully Sent"

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

    def call_low_level_buttons_command_server(self, request=GetLowLevelButtons.Request()):
    
        future = self.get_low_level_buttons_client.call_async(request)
        future.add_done_callback(self.callback_call_low_level_buttons_command)
        
    def callback_call_low_level_buttons_command(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            # self.get_logger().info("Start_Button: "+str(response.start_button) + ", Debug_Button1: " + str(response.debug_button1) + \
            #                        ", Debug_Button2: " + str(response.debug_button2) + ", Debug_Button3: " + str(response.debug_button3))
            self.start_button = response.start_button
            self.debug_button1 = response.debug_button1
            self.debug_button2 = response.debug_button2
            self.debug_button3 = response.debug_button3
            self.waited_for_end_of_get_low_level_buttons = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))  

    def call_get_torso_position_server(self, request=GetTorso.Request()):
    
        future = self.get_torso_position_client.call_async(request)
        future.add_done_callback(self.callback_call_get_torso_position)
        
    def callback_call_get_torso_position(self, future): 

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            # self.get_logger().info("Torso: "+str(response.legs) + ", Legs: " + str(response.torso))
            self.legs_position = response.legs
            self.torso_position = response.torso
            self.waited_for_end_of_get_torso_position = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))  

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




















class RobotStdFunctions():

    def __init__(self, node: ROS2TaskNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, breakable_play=False, break_play=False, wait_for_end_of=True):

        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.show_in_face = show_in_face
        request.breakable_play = breakable_play
        request.break_play = break_play

        self.node.call_speech_command_server(request=request, wait_for_end_of=wait_for_end_of)
        
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

            request = SaveSpeechCommand.Request()
            request.filename = file
            request.command = comm
            request.quick_voice = quick_voice
            request.play_command = play_command
            request.show_in_face = show_in_face

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
    
        request=GetVCCs.Request()

        self.node.call_vccs_command_server(request=request)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_vccs:
            pass
        self.node.waited_for_end_of_get_vccs = False

        return self.node.battery_voltage, self.node.emergency_stop 

    def get_low_level_buttons(self, wait_for_end_of=True):
    
        request=GetLowLevelButtons.Request()

        self.node.call_low_level_buttons_command_server(request=request)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_low_level_buttons:
            pass
        self.node.waited_for_end_of_get_low_level_buttons = False

        return self.node.start_button, self.node.debug_button1, self.node.debug_button2, self.node.debug_button3 

    def wait_for_start_button(self):

        start_button_state = False

        while not start_button_state:
            start_button_state, d1b, d2b, d3b = self.get_low_level_buttons()
            print("Start Button State:", start_button_state)
            time.sleep(0.1)

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
    
    def set_torso_position(self, legs=0.0, torso=0.0, wait_for_end_of=True):
        
        request = SetTorso.Request()
        request.legs = int(legs)
        request.torso = int(torso)

        MAX_ERROR_TORSO_READING = 3
        MAX_ERROR_LEGS_READING = 3

        self.node.call_set_torso_position_server(request=request)

        if wait_for_end_of:
            # must check the position until it has arrived 
            while not self.node.waited_for_end_of_set_torso_position:
                
                l, t = self.get_torso_position()
                error_l = abs(request.legs - l)
                error_t = abs(request.torso - t)
                # print(l, t, error_l, error_t)

                if error_l <= MAX_ERROR_LEGS_READING and error_t<=MAX_ERROR_TORSO_READING:
                    self.node.waited_for_end_of_set_torso_position = True
                else:
                    time.sleep(0.25)
        
            # time.sleep(0.5) # the max error may make robot think it has reached before it has acutally reached

        self.node.waited_for_end_of_set_torso_position = False

        return self.node.torso_success, self.node.torso_message
    
    def get_torso_position(self,  wait_for_end_of=True):
    
        request=GetTorso.Request()

        self.node.call_get_torso_position_server(request=request)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_get_torso_position:
                pass
        self.node.waited_for_end_of_get_torso_position = False

        return self.node.legs_position, self.node.torso_position

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

    def get_continuous_audio(self, keywords=[], max_number_attempts=3, wait_for_end_of=True):

        request = ContinuousGetAudio.Request()
        request.keywords = keywords
        request.max_number_attempts = max_number_attempts
            
        self.node.call_continuous_audio_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_continuous_audio:
                pass
        self.node.waited_for_end_of_continuous_audio = False

        return self.node.continuous_audio_success, self.node.continuous_audio_message 
    
    def is_get_continuous_audio_done(self):

        if self.node.received_continuous_audio:
            self.node.received_continuous_audio = False
            return True, self.node.continuous_audio_success, self.node.continuous_audio_message
        
        return False, False, ""
    
    def calibrate_audio(self, wait_for_end_of=True):

        request = CalibrateAudio.Request()
            
        self.node.call_calibrate_audio_server(request=request, wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_calibrate_audio:
                pass
        self.node.waited_for_end_of_calibrate_audio = False

        return self.node.calibrate_audio_success, self.node.calibrate_audio_message 
    
    def set_face(self, command="", custom="", wait_for_end_of=False):
        
        request = SetFace.Request()
        request.command = command
        request.custom = custom
        
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

    def activate_yolo_objects(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5, wait_for_end_of=True):
        
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.activate_objects_hand = activate_objects_hand
        request.activate_shoes_hand = activate_shoes_hand
        request.activate_doors_hand = activate_doors_hand
        request.minimum_objects_confidence = float(minimum_objects_confidence)
        request.minimum_shoes_confidence = float(minimum_shoes_confidence)
        request.minimum_doors_confidence = float(minimum_doors_confidence)

        self.node.call_activate_yolo_objects_server(request=request)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def activate_motors(self, activate=True, wait_for_end_of=True):
        
        request = ActivateBool.Request()
        request.activate = activate

        self.node.call_activate_motors_server(request=request)

        self.node.activate_motors_success = True
        self.node.activate_motors_message = "Activated with selected parameters"

        return self.node.activate_motors_success, self.node.activate_motors_message

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

    def set_arm(self, command="", pose=[], adjust_position=0.0, wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False
        
        temp = ArmController()
        temp.command = command
        temp.adjust_position = float(adjust_position)
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
    
    def set_navigation(self, movement="", target=[0.0, 0.0], max_speed=15.0, absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, avoid_people=False, wait_for_end_of=True):

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
            navigation.avoid_people = avoid_people
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

        task_initialpose.pose.pose.position.x = float(initial_position[1])
        task_initialpose.pose.pose.position.y = float(-initial_position[0])
        task_initialpose.pose.pose.position.z = float(0.0)

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

    def search_for_person(self, tetas, delta_t=3.0, break_if_detect=False, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=False, only_detect_person_right_in_front=False):

        self.activate_yolo_pose(activate=True, characteristics=characteristics, only_detect_person_arm_raised=only_detect_person_arm_raised, only_detect_person_legs_visible=only_detect_person_legs_visible, only_detect_person_right_in_front=only_detect_person_right_in_front) 
        self.set_speech(filename="generic/search_people", wait_for_end_of=False)
        # self.set_rgb(WHITE+ALTERNATE_QUARTERS)
        # time.sleep(0.5)
        
        total_person_detected = []
        person_detected = []
        people_ctr = 0

        ### MOVES NECK AND SAVES DETECTED PEOPLE ###
        
        for t in tetas:
            self.set_rgb(RED+SET_COLOUR)
            self.set_neck(position=t, wait_for_end_of=True)
            time.sleep(1.0) # 0.5
            self.node.detected_people.persons = [] # clears detected_objects after receiving them to make sure the objects from previous frames are not considered again
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
        time.sleep(0.1)

        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path

    def search_for_objects(self, tetas, delta_t=3.0, list_of_objects = [], list_of_objects_detected_as = [], use_arm=False, detect_objects=False, detect_shoes=False, detect_furniture=False, detect_objects_hand=False, detect_shoes_hand=False, detect_furniture_hand=False):

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

            self.activate_yolo_objects(activate_objects=detect_objects, activate_shoes=detect_shoes, activate_doors=detect_furniture,
                                        activate_objects_hand=detect_objects_hand, activate_shoes_hand=detect_shoes_hand, activate_doors_hand=detect_furniture_hand,
                                        minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5)
            self.set_speech(filename="generic/search_objects", wait_for_end_of=False)
            # self.set_rgb(WHITE+ALTERNATE_QUARTERS)
            # time.sleep(0.5)

            ### MOVES NECK AND SAVES DETECTED OBJECTS ###
            for t in tetas:
                rgb_found_list_of_objects = False
                self.set_rgb(RED+SET_COLOUR)
                self.set_neck(position=t, wait_for_end_of=True)
                time.sleep(1.0) # 0.5
                self.node.detected_objects.objects = [] # clears detected_objects after receiving them to make sure the objects from previous frames are not considered again
                self.set_rgb(WHITE+SET_COLOUR)

                start_time = time.time()
                while (time.time() - start_time) < delta_t:      

                    # if detect_objects: 
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
                            
                            # checks for previous tetas
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

                            # checks for current teta (added to stop doing the full delta_t if all objects have already been detected)
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
        time.sleep(0.1)
        
        if send_to_face:
            self.set_face(custom=face_path)
        
        return face_path

    def get_point_cloud(self, bb=BoundingBox(), camera="head", wait_for_end_of=True):

        requested_objects = []
            
        # bb = BoundingBox()
        # bb.box_top_left_x = 0
        # bb.box_top_left_y = 0
        # bb.box_width = 1280
        # bb.box_height = 720

        get_pc = BoundingBoxAndPoints()
        get_pc.bbox = bb

        requested_objects.append(get_pc)

        request = GetPointCloud.Request()
        request.data = requested_objects
        request.retrieve_bbox = False
        request.camera = camera

        self.node.waiting_for_pcloud = True
        self.node.call_point_cloud_server(request=request)

        if wait_for_end_of:
            while self.node.waiting_for_pcloud:
                pass

        return self.node.point_cloud_response.coords[0]

    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        request = ActivateObstacles.Request()
        request.activate_lidar_up = obstacles_lidar_up
        request.activate_lidar_bottom = obstacles_lidar_bottom
        request.activate_camera_head = obstacles_camera_head

        self.node.call_activate_obstacles_server(request=request)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message

    def get_robot_localization(self):

        return self.node.robot_x, self.node.robot_y, self.node.robot_t

    def get_head_rgb_image(self):

        if self.node.first_rgb_head_image_received:
            current_frame_rgb_head = self.node.br.imgmsg_to_cv2(self.node.rgb_head_img, "bgr8")
        else:
            current_frame_rgb_head = np.zeros((360, 640, 3), dtype=np.uint8)
        
        return self.node.first_rgb_head_image_received, current_frame_rgb_head

    def get_head_depth_image(self):

        if self.node.first_depth_head_image_received:
            current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_head_img, desired_encoding="passthrough")
        else:
            current_frame_depth_head = np.zeros((360, 640), dtype=np.uint8)
        
        return self.node.first_depth_head_image_received, current_frame_depth_head

    def get_hand_rgb_image(self):

        if self.node.first_rgb_hand_image_received:
            current_frame_rgb_hand = self.node.br.imgmsg_to_cv2(self.node.rgb_hand_img, "bgr8")
        else:
            current_frame_rgb_hand = np.zeros((360, 640, 3), dtype=np.uint8)
        
        return self.node.first_rgb_hand_image_received, current_frame_rgb_hand

    def get_hand_depth_image(self):

        if self.node.first_depth_hand_image_received:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
        else:
            current_frame_depth_hand = np.zeros((360, 640), dtype=np.uint8)
        
        return self.node.first_depth_hand_image_received, current_frame_depth_hand
