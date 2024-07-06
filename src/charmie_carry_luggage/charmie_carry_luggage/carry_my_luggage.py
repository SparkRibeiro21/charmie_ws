#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, BoundingBox, BoundingBoxAndPoints, ArmController, ListOfDetectedPerson, ListOfDetectedObject, Obstacles
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, GetPointCloud, SetFace, ActivateObstacles

import cv2 
import threading
import time
from cv_bridge import CvBridge
import math
from pathlib import Path
import numpy as np
from datetime import datetime

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class CarryMyLuggageNode(Node):

    def __init__(self):
        super().__init__("CarryMyLuggage")
        self.get_logger().info("Initialised CHARMIE CarryMyLuggage Node")

        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.torso_test_publisher = self.create_publisher(Pose2D, "torso_test" , 10)
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        
        # Intel Realsense Subscribers
        # self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_head_image_callback, 10)
        # Hand
        # self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_hand_image_callback, 10)
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
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")
        
        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        while not self.get_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        # while not self.neck_track_person_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        # while not self.neck_track_object_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Yolos
        while not self.activate_yolo_pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        while not self.activate_yolo_objects_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        # Face
        # while not self.face_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Face Command...")
        # Obstacles
        while not self.activate_obstacles_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Activate Obstacles Command...")
        # Point Cloud
        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
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
        self.point_cloud_response = GetPointCloud.Response()
        self.obstacles = Obstacles()

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.face_success = True
        self.face_message = ""
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
        self.flag_navigation_reached = False

        self.get_neck_position = [1.0, 1.0]

    def get_aligned_depth_head_image_callback(self, img: Image):
        self.depth_head_img = img
        self.first_depth_head_image_received = True
        # print("Received Depth Image")

    def get_aligned_depth_hand_image_callback(self, img: Image):
        self.depth_hand_img = img
        self.first_depth_hand_image_received = True
        # print("Received HAND Depth Image")

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people
    
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

    ### OBSTACLES
    def obstacles_callback(self, obs: Obstacles):
        self.obstacles = obs
        
    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta

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

    def get_objects_callback(self, objects: Yolov8Objects):
        #print(objects.objects)
        self.nr_objects = objects.num_objects
        self.objects = objects.objects
        self.image = objects.image_rgb

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req, camera):
        request = GetPointCloud.Request()
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
    node = CarryMyLuggageNode()
    th_main = threading.Thread(target=ThreadMainCarryMyLuggage, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainCarryMyLuggage(node: CarryMyLuggageNode):
    main = CarryMyLuggageMain(node)
    main.main()

class CarryMyLuggageMain():

    def __init__(self, node: CarryMyLuggageNode):
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

    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        self.node.call_face_command_server(command=command, custom=custom, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_face:
                pass
        self.node.waited_for_end_of_face = False

        return self.node.face_success, self.node.face_message
    
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

    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message
    
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
       
    def get_bag_pick_cordinates(self):

        DEBUG = True
        MIN_BAG_PIXEL_AREA = 40000
        f_coords = []

        self.node.first_depth_hand_image_received = False

        while not self.node.first_depth_hand_image_received:
            print(".")
            time.sleep(0.1)

        c_areas = []
        
        while not c_areas or max(c_areas) < MIN_BAG_PIXEL_AREA:
            c_areas.clear()
            current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
            height, width = current_frame_depth_head.shape
            print(height, width)
            current_frame_depth_head = cv2.resize(current_frame_depth_head, (1280, 720), interpolation = cv2.INTER_NEAREST)
            height, width = current_frame_depth_head.shape
            print(height, width)
            
            # current_frame_depth_head[int(0.80*height):height,int(0.29*width):int(0.71*width)] = 0 # remove the robot from the image

            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head != 0) & (current_frame_depth_head >= self.top_bag_dist) & (current_frame_depth_head <= self.floor_dist)

            blank_image_bw = np.zeros((height,width), np.uint8)
            blank_image_bw2 = np.zeros((height,width), np.uint8)
            blank_image_bw[mask_near] = [255]

            contours, hierarchy = cv2.findContours(blank_image_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # print("areas")
            for a in contours: # create list with area size 
                # print(cv2.contourArea(a))
                c_areas.append(cv2.contourArea(a))
            # if c_areas:
            #     print(max(c_areas), " ...")

        cnt = contours[c_areas.index(max(c_areas))] # extracts the largest area 
        # print(c_areas.index(max(c_areas)))
        
        M = cv2.moments(cnt) # calculates centroide
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        xi,yi,w,h = cv2.boundingRect(cnt)

        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
        theta = math.degrees(math.atan2(vy,vx))

        if theta < 0.0:
            theta_gripper = theta+90.0
        else:
            theta_gripper = theta-90.0

        # bb_thresh = 40 
        # bb = BoundingBox() # centroide of bag bounding box 
        # bb.box_top_left_x = max(cx-bb_thresh, 0)
        # bb.box_top_left_y = max(cy-bb_thresh, 0)
        # if bb.box_top_left_x + 2*bb_thresh > width:
        #     bb.box_width = width - bb.box_top_left_x 
        # else:
        #     bb.box_width = 2*bb_thresh
        # if bb.box_top_left_y + 2*bb_thresh > height:
        #     bb.box_height = height - bb.box_top_left_y 
        # else:
        #     bb.box_height = 2*bb_thresh
        # coords_centroide = self.get_point_cloud(bb=bb)
        
        bb = BoundingBox() # full bag bounding box
        bb.box_top_left_x = max(xi, 0)
        bb.box_top_left_y = max(yi, 0)
        bb.box_width = w
        bb.box_height = h

        coords_full = self.get_point_cloud(bb=bb)
        selected_coords = coords_full

        # adds difference between camera center to gripper center
        selected_coords.center_coords.z += 70

        # x = bag height
        # y = move front and back robot, or left and right for camera
        # z = move right and left robot, or up and down for camera
        # print("xc = ", round(coords_centroide.center_coords.x,0), "yc = ", round(coords_centroide.center_coords.y,0), "zc = ", round(coords_centroide.center_coords.z,0))
        # print("xf = ", round(coords_full.center_coords.x,0), "yf = ", round(coords_full.center_coords.y,0), "zf = ", round(coords_full.center_coords.z,0))

        ang_to_bag = -math.degrees(math.atan2(selected_coords.center_coords.z, selected_coords.center_coords.y))
        dist_to_bag = (math.sqrt(selected_coords.center_coords.y**2 + selected_coords.center_coords.z**2))/1000
        
        f_coords.append(selected_coords.center_coords.x/1000)
        f_coords.append(selected_coords.center_coords.y/1000)
        f_coords.append(selected_coords.center_coords.z/1000)
        f_coords.append(ang_to_bag)
        f_coords.append(dist_to_bag)
        f_coords.append(theta_gripper)
        
        # print(ang_to_bag, dist_to_bag)
        
        if DEBUG:
            mask_remaining = (current_frame_depth_head > self.floor_dist) # just for debug, floor level
            blank_image = np.zeros((height,width,3), np.uint8)
            
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

            cv2.drawContours(blank_image, [cnt], 0, (0, 255, 0), 3) 
            cv2.drawContours(blank_image_bw2, [cnt], 0, (255), thickness=cv2.FILLED) 
            cv2.circle(blank_image, (cx, cy), 10, (0, 255, 0), -1)
            cv2.circle(blank_image_bw2, (cx, cy), 10, (128), -1)

            cv2.rectangle(blank_image,(xi,yi),(xi+w,yi+h), (255, 0, 255),2)
            cv2.rectangle(blank_image_bw2,(xi,yi),(xi+w,yi+h),(128),2)

            rows,cols = blank_image_bw2.shape[:2]
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(blank_image,(cols-1,righty),(0,lefty),(255, 0, 255),2)
            cv2.line(blank_image_bw2,(cols-1,righty),(0,lefty),(128),2)

            # print("bag theta =", round(math.degrees(theta), 2), round(theta,2))
            # print("bag centroide =", cx, cy)

            cv2.rectangle(blank_image, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x+bb.box_width, bb.box_top_left_y+bb.box_height), (255, 0, 255),2)
            cv2.rectangle(blank_image_bw2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x+bb.box_width, bb.box_top_left_y+bb.box_height), (128), 2)

            cv2.imshow("New Img Distance Inspection", blank_image)
            cv2.imshow("New Img Distance Inspection BW", blank_image_bw2)

            k = cv2.waitKey(100)
            if k == ord('w'):
                self.floor_dist += 10
            if k == ord('q'):
                self.floor_dist -= 10
            if k == ord('s'):
                self.top_bag_dist += 10
            if k == ord('a'):
                self.top_bag_dist -= 10

            # print(self.floor_dist, self.top_bag_dist)

        return f_coords

    def check_bag_grabbed_camera(self):

        DEBUG = False
        MIN_PERCENTAGE_AREA_BAG  = 0.05
        bag_grabbed_camera = False

        # while True:
        # print("IN")

        self.node.first_depth_hand_image_received = False
        while not self.node.first_depth_hand_image_received:
            print(".")
            time.sleep(0.1)
        
        # if self.node.first_depth_hand_image_received:

        current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
        height, width = current_frame_depth_head.shape
        tot_pixeis = height*width 
        
        mask_zero = (current_frame_depth_head == 0)
        mask_near = (current_frame_depth_head != 0) & (current_frame_depth_head <= self.bag_in_gripper_dist)

        blank_image_bw = np.zeros((height,width), np.uint8)
        blank_image_bw[mask_near] = [255]

        pixel_count_near = np.count_nonzero(mask_near)
        full_image_near_err = pixel_count_near/tot_pixeis

        if DEBUG:
            mask_remaining = (current_frame_depth_head > self.bag_in_gripper_dist) # just for debug, floor level
            blank_image = np.zeros((height,width,3), np.uint8)
            
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

            cv2.imshow("New Img Check Bag Grabbed", blank_image)
            cv2.imshow("New Img Check Bag Grabbed BW", blank_image_bw)

            k = cv2.waitKey(10)
            if k == ord('x'):
                self.bag_in_gripper_dist += 10
            if k == ord('z'):
                self.bag_in_gripper_dist -= 10

            print(self.bag_in_gripper_dist, full_image_near_err)

        if full_image_near_err > MIN_PERCENTAGE_AREA_BAG:
            bag_grabbed_camera = True

        return bag_grabbed_camera

    # main state-machine function
    def main(self):
        
        # States in CarryMyLuggage Task
        self.Waiting_for_task_to_start = 0
        self.Recognize_bag = 1
        self.Go_to_bag = 2
        self.Camera_pick_bag = 3
        self.Go_back_to_initial_position = 4
        self.Follow_to_the_car = 5
        self.Deliver_the_bag = 6
        self.Return_to_the_house = 7
        self.Final_State = 8
        
        # CML Vars ...
        self.floor_dist = 580 # 660
        self.top_bag_dist = 440
        self.bag_in_gripper_dist = 200
        self.INITIAL_REACHED_RADIUS = 0.8
        self.slight_angular_adjust_depending_on_side = 0.0
        self.bag_side = "none"
        self.task_room_location = "Office"
        self.DETECT_BAG_FILTER = True # if i want to turn off the bag detection, if not reliable
        self.move_bag_coords = Point()
        self.pointing_person = DetectedPerson()

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -50]

        self.initial_position = [-4.5, 1.0, 0.0]
        self.initial_angle_temp = 0.0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_to_start

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Carry My Luggage Main...")
                
        while True:

            if self.state == self.Waiting_for_task_to_start:
                print("State:", self.state, "- Waiting_for_task_to_start")

                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")                
                
                # set rgb's to cyan
                self.set_rgb(CYAN+HALF_ROTATE)

                time.sleep(1)
                
                self.activate_yolo_objects(activate_objects=False, activate_doors=False, activate_shoes=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False)
                # self.activate_yolo_pose(activate=False)
                self.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)
        
                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                time.sleep(1)

                self.set_navigation(movement="orientate", absolute_angle = float(self.initial_angle_temp), flag_not_obs=True, wait_for_end_of=True)

                # Speech: "I am ready to start the carry my luggage task"                
                self.set_speech(filename="carry_my_luggage/carry_luggage_ready_start", wait_for_end_of=True)
                
                # Speech: "Waiting for start button to be pressed."
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True)

                # wait for start_button
                self.wait_for_start_button()

                # self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                # self.wait_for_door_start()

                self.set_rgb(BLUE+SET_COLOUR)

                # next state
                self.state = self.Recognize_bag

            elif self.state == self.Recognize_bag:
                print("State:", self.state, "- Recognize_bag")
                
                # Speech: "Please point to the bag you want me to carry."
                self.set_speech(filename="carry_my_luggage/point_to_bag", wait_for_end_of=True)
                
                # Detect person pointing to the bag
                pointing_person_found = False
                while not pointing_person_found:
                    tetas = [[0, -10]]
                    people_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_legs_visible=True)
                    print("-")
                    for p in people_found:
                        print(p.room_location, p.pointing_at)
                        if p.room_location == self.task_room_location and p.pointing_at != "None":
                            self.pointing_person = p
                            self.bag_side = p.pointing_at.lower()
                            pointing_person_found = True
                
                print(self.pointing_person.position_relative)
                self.set_rgb(GREEN+HALF_ROTATE)

                # Speech: "You are pointing to the bag on your right"
                self.set_speech(filename="carry_my_luggage/detected_bag_"+self.bag_side, wait_for_end_of=True)
                
                bag_found = False
                if self.DETECT_BAG_FILTER:
                    # Check for bag object without moving neck
                    correct_bag = DetectedObject()
                    tetas = [[0, -30]]
                    objects_found = self.search_for_objects(tetas=tetas, delta_t=2.0, detect_objects=True)
                    for o in objects_found:
                        print(o.position_relative)
                        if self.bag_side == "left":
                            if o.object_name == "Bag" and o.room_location == self.task_room_location and o.position_relative.x > self.pointing_person.position_relative.x:
                                correct_bag = o
                                bag_found = True
                        else: # == "right"
                            if o.object_name == "Bag" and o.room_location == self.task_room_location and o.position_relative.x < self.pointing_person.position_relative.x:
                                correct_bag = o
                                bag_found = True

                    if not bag_found:
                        # Check for bag object using multiple neck positions (x3)
                        tetas = [[-30, -30], [0, -30], [30, -30]]
                        objects_found = self.search_for_objects(tetas=tetas, delta_t=2.0, detect_objects=True)
                        for o in objects_found:
                            if self.bag_side == "left":
                                if o.object_name == "Bag" and o.room_location == self.task_room_location and o.position_relative.x > self.pointing_person.position_relative.x:
                                    correct_bag = o
                            else: # == "right"
                                if o.object_name == "Bag" and o.room_location == self.task_room_location and o.position_relative.x < self.pointing_person.position_relative.x:
                                    correct_bag = o

                if bag_found:
                    self.set_rgb(MAGENTA+HALF_ROTATE)
                    self.move_bag_coords = correct_bag.position_absolute
                    
                else: # if the robot does not find a bag, it calculates the coordinates according to the pointing person position
                    self.set_rgb(CYAN+HALF_ROTATE)
                
                    relative_not_detected_bag_position = Point()
                    relative_not_detected_bag_position.y = self.pointing_person.position_relative.y - 0.2
                    if self.bag_side == "left":
                        relative_not_detected_bag_position.x = self.pointing_person.position_relative.x + 0.6
                    else: # == "right"
                        relative_not_detected_bag_position.x = self.pointing_person.position_relative.x - 0.6
                
                    # However the coordinates can not be relative to the robot, because if the robot is not facing forward, the axis don't make sense
                    # So we need to convert realtive coordinates to absolute coordinates
                    
                    # calculate the absolute position according to the robot localisation
                    angle_obj = math.atan2(relative_not_detected_bag_position.x, relative_not_detected_bag_position.y)
                    dist_obj = math.sqrt(relative_not_detected_bag_position.x**2 + relative_not_detected_bag_position.y**2)

                    theta_aux = math.pi/2 - (angle_obj - self.node.robot_t)

                    target_x = dist_obj * math.cos(theta_aux) + self.node.robot_x
                    target_y = dist_obj * math.sin(theta_aux) + self.node.robot_y

                    self.move_bag_coords.x = target_x
                    self.move_bag_coords.y = target_y
                    
                # next state
                self.state = self.Go_to_bag 

            elif self.state == self.Go_to_bag:
                print("State:", self.state, "- Go_to_bag")

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                # Speech: "I am moving towards the bag you pointed at."
                self.set_speech(filename="carry_my_luggage/going_to_bag", wait_for_end_of=True)
                print('Navigate to: ', self.move_bag_coords.x, self.move_bag_coords.y)

                # Speech: "Just a warning. I might slightly push the bag. However this is intended since I have to position myself as close as possible to the bag."
                # self.set_speech(filename="carry_my_luggage/might_touch_bag", wait_for_end_of=False)

                self.set_navigation(movement="rotate", target = [self.move_bag_coords.x, self.move_bag_coords.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                
                self.set_navigation(movement="move", target = [self.move_bag_coords.x, self.move_bag_coords.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                
                print('received bag: ', self.bag_side)
                if self.bag_side == "left":
                    self.slight_angular_adjust_depending_on_side = -20.0
                else: # == "right"
                    self.slight_angular_adjust_depending_on_side = 20.0

                self.set_navigation(movement="orientate", absolute_angle = 90.0+self.slight_angular_adjust_depending_on_side, flag_not_obs=True, wait_for_end_of=True)

                self.state = self.Camera_pick_bag
            
            elif self.state == self.Camera_pick_bag:

                # Speech: "Please step away from the bag as I might need some space to pick up the bag."
                self.set_speech(filename="carry_my_luggage/step_away_from_bag", wait_for_end_of=False)
                
                s,m = self.set_arm(command="carry_my_luggage_pre_pick", wait_for_end_of=True)
                print("carry_my_luggage_pre_pick", s,m)
                
                # Speech: "I am going to pick up the bag now."
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(0.5)

                bag_grabbed = False
                while not bag_grabbed:

                    bag_coords = self.get_bag_pick_cordinates()
                    print(bag_coords)
                    self.set_navigation(movement="adjust", adjust_distance=bag_coords[4], adjust_direction=bag_coords[3], wait_for_end_of=True)
                    time.sleep(2.0)

                    bag_grabbed_gripper, m = self.set_arm(command="carry_my_luggage_pick_bag", adjust_position=bag_coords[5], wait_for_end_of=True)
                    print("carry_my_luggage_pick_bag", bag_grabbed_gripper, m)
                    bag_grabbed_camera = self.check_bag_grabbed_camera()
                    bag_grabbed = bag_grabbed_gripper or bag_grabbed_camera
                    print("BAG GRABBED:", bag_grabbed, "via camera:", bag_grabbed_camera, "via gripper:", bag_grabbed_gripper)
                    # time.sleep(1.0)

                    if not bag_grabbed:
                        
                        # Speech: "Oops. Lets try again."
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=False)
                                            
                        if self.bag_side == "left":
                            self.slight_angular_adjust_depending_on_side += 15.0
                        else: # == "right"
                            self.slight_angular_adjust_depending_on_side -= 15.0
                        
                        self.set_navigation(movement="orientate", absolute_angle = 90.0+self.slight_angular_adjust_depending_on_side, flag_not_obs=True, wait_for_end_of=True)
                        time.sleep(2.5)
                
                # Speech: "I have successfully picked up the bag."
                self.set_speech(filename="carry_my_luggage/success_pick_bag", wait_for_end_of=False)
                
                s,m = self.set_arm(command="carry_my_luggage_post_pick", wait_for_end_of=True)
                print("carry_my_luggage_post_pick", s,m)
                
                self.state = self.Go_back_to_initial_position

            elif self.state == self.Go_back_to_initial_position:
               
                # Speech: "Moving to the initial position."
                self.set_speech(filename="carry_my_luggage/move_initial_position", wait_for_end_of=False)

                self.set_navigation(movement="rotate", target = [self.initial_position[0], self.initial_position[1]], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                
                self.set_navigation(movement="move", target = [self.initial_position[0], self.initial_position[1]], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                
                self.set_navigation(movement="rotate", target = [self.pointing_person.position_absolute.x, self.pointing_person.position_absolute.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                
                self.set_neck(position=self.look_forward, wait_for_end_of=False)
                
                self.state = self.Follow_to_the_car

            elif self.state == self.Follow_to_the_car:

                # Speech: "I am ready to follow you. Let's go."
                self.set_speech(filename="carry_my_luggage/ready_to_follow", wait_for_end_of=True)

                self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # FOLLOW PERSON CODE HERE ...
                # Speech: "Unfortunately my team has not developed the follow me function yet, and so I cannot follow you. I am sorry."
                self.set_speech(filename="carry_my_luggage/unfortunately_cannot_follow", wait_for_end_of=True)

                self.state = self.Deliver_the_bag

            elif self.state == self.Deliver_the_bag:

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                # Speech: "Here is your bag."
                self.set_speech(filename="carry_my_luggage/here_is_your_bag", wait_for_end_of=True)
                s,m = self.set_arm(command="carry_my_luggage_give_bag", wait_for_end_of=True)
                print("carry_my_luggage_give_bag", s,m)
                
                # Speech: "Please take your bag from my hand."
                self.set_speech(filename="carry_my_luggage/take_bag_from_hand", wait_for_end_of=True)

                # Speech: "I will open my hand. In 3. 2. 1.
                self.set_speech(filename="arm/arm_open_gripper", wait_for_end_of=True)
                s,m = self.set_arm(command="open_gripper", wait_for_end_of=True)
                print("open_gripper", s,m)

                time.sleep(5.0)

                s,m = self.set_arm(command="carry_my_luggage_post_give_bag", wait_for_end_of=True)
                print("carry_my_luggage_post_give_bag", s,m)
                
                self.state = self.Return_to_the_house

            elif self.state == self.Return_to_the_house:

                # Speech: "I will return to the house. See you soon."
                self.set_speech(filename="carry_my_luggage/return_to_house", wait_for_end_of=True)

                self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # RETURN HOUSE CODE HERE ...
                
                self.state = self.Final_State

                
            elif self.state == self.Final_State:
                
                print("State:", self.state, "- Final_State")
                
                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                # set rgb's to static green
                self.set_rgb(GREEN+BREATH)

                # Speech: "I have finished my carry my luggage task."
                self.set_speech(filename="carry_my_luggage/cml_finished", wait_for_end_of=True)

                while True:
                    pass

            else:
                pass