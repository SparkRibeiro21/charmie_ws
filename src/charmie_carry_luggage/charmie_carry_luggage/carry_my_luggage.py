#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, BoundingBox, BoundingBoxAndPoints
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, GetPointCloud

import cv2 
import threading
import time
from cv_bridge import CvBridge
import math
import numpy as np

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class CarryMyLuggageNode(Node):

    def __init__(self):
        super().__init__("CarryMyLuggage")
        self.get_logger().info("Initialised CHARMIE CarryMyLuggage Node")

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
        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")
        
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

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        # while not self.speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Speech Command...")
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        # while not self.neck_track_person_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        # while not self.neck_track_object_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        
        # Point Cloud
        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")


        # Variables 
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_arm = False
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

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
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

    def get_aligned_depth_head_image_callback(self, img: Image):
        self.depth_head_img = img
        self.first_depth_head_image_received = True
        # print("Received Depth Image")

    def get_aligned_depth_hand_image_callback(self, img: Image):
        self.depth_hand_img = img
        self.first_depth_hand_image_received = True
        # print("Received Depth Image")

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        
        # cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        # cv2.waitKey(10)

    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

        # current_frame = self.br.imgmsg_to_cv2(self.detected_objects.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()

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

    def set_navigation(self, movement="", target=[0.0, 0.0], absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, wait_for_end_of=True):

        if movement.lower() != "move" and movement.lower() != "rotate" and movement.lower() != "orientate" and movement.lower() != "adjust" and movement.lower() != "adjust_obstacle" :   
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

            if adjust_direction < 0:
                adjust_direction += 360

            navigation.target_coordinates.x = target[0]
            navigation.target_coordinates.y = target[1]
            navigation.move_or_rotate = movement
            navigation.orientation_absolute = absolute_angle
            navigation.flag_not_obs = flag_not_obs
            navigation.reached_radius = reached_radius
            navigation.avoid_people = False
            navigation.adjust_distance = adjust_distance
            navigation.adjust_direction = adjust_direction
            navigation.adjust_min_dist = adjust_min_dist

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
    

    def detect_bag(self, position_of_referee_x, received_bag):
        correct_bag  = False
        self.activate_yolo_objects(activate_objects=True)
        objects_stored = self.node.detected_objects
        obj = Point()
                
        for object in objects_stored.objects:
            print('Objeto: ', object)
            if object.object_name ==  'bag' or object.object_name ==  'Bag':
                print('x do saco relativo ao robô: ', object.position_relative.x)
                if object.position_relative.x <= position_of_referee_x and received_bag == 'right':
                    correct_bag = True
                    print('Coordenadas do saco (mundo): ', object.position_absolute)
                    obj = object.position_absolute
                    break
                elif object.position_relative.x > position_of_referee_x and received_bag == 'left':
                    correct_bag = True
                    print('Coordenadas do saco (mundo): ', object.position_absolute)
                    obj = object.position_absolute
                    break
                else: 
                    correct_bag = False
                    
        return correct_bag, obj
        
    def detect_person_pointing(self):
        detected_person_temp = Yolov8Pose()
        self.activate_yolo_pose(activate=True)
        
        self.set_rgb(WHITE+HALF_ROTATE)

        time.sleep(2.5)
        detected_person_temp = self.node.detected_people  


        bag_side = ""
        self.set_rgb(BLUE+HALF_ROTATE)

        person_detected = False
        while not person_detected:
            
            detected_person_temp = self.node.detected_people  

            for p in detected_person_temp.persons:
                if p.room_location == "Office" and p.pointing_at != "None":
                    person_detected = True
                    bag_side = p.pointing_at.lower()
                    self.set_rgb(GREEN+HALF_ROTATE)

                print(p.room_location, p.pointing_at, person_detected)
            
            time.sleep(0.05)

        self.activate_yolo_pose(activate=False)

        return bag_side


    def get_bag_pick_cordinates(self):

        DEBUG = False
        MIN_BAG_PIXEL_AREA = 40000
        f_coords = []

        self.node.first_depth_hand_image_received = False

        while not self.node.first_depth_hand_image_received:
            pass

        c_areas = []
        
        while not c_areas or max(c_areas) < MIN_BAG_PIXEL_AREA:
            c_areas.clear()
            current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
            height, width = current_frame_depth_head.shape
            
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

            k = cv2.waitKey(1)
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
    
    # main state-machine function
    def main(self):
        
        # States in CarryMyLuggage Task
        self.Waiting_for_task_to_start = 0
        self.Recognize_bag = 1
        self.Go_to_bag = 2
        self.Pick_bag_left = 3
        self.Pick_bag_right = 4
        self.Camera_pick_bag = 10
        self.Final_State = 5
        
        self.floor_dist=660
        self.top_bag_dist=440

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]

        self.initial_position = [-4.5, 1.0, 0.0]

        self.IMU_ANGLE = 90.0
        self.INITIAL_REACHED_RADIUS = 0.9

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Camera_pick_bag

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Carry My Luggage Main...")

        while True:

            if self.state == self.Waiting_for_task_to_start:
                print("State:", self.state, "- Waiting_for_task_to_start")

                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")

                time.sleep(1)
        
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                # set rgb's to cyan
                self.set_rgb(CYAN+SET_COLOUR)

                self.set_speech(filename="carry_my_luggage/carry_luggage_ready_start", wait_for_end_of=True)
                
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True)

                # wait for start_button
                self.wait_for_start_button()

                self.set_rgb(BLUE+SET_COLOUR)

                # set rgb's to static green
                # self.set_rgb(GREEN+SET_COLOUR)

                # next state
                self.state = self.Recognize_bag

            elif self.state == self.Recognize_bag:
                print("State:", self.state, "- Recognize_bag")
                
                # set rgb's to blue

                # speech: "Please point to the bag you want me to take."
                self.set_speech(filename="carry_my_luggage/point_to_bag", wait_for_end_of=True)
                # how to get pose and bag????
                
                # set rgb's to static green
                # self.set_rgb(GREEN+SET_COLOUR)

                # speech: "I have detected the bag"
                # speck: dizer qual o saco: criar fase para esquerda e direita
                
                #### TIAGO METE O YOLO POSE AQUI
                received_bag = self.detect_person_pointing()
                
                
                # received_bag tem de ser dado pela função de deteção
                self.set_speech(filename="carry_my_luggage/detected_bag_"+received_bag.lower(), wait_for_end_of=True)
                
                
                ### Quero retirar coordenadas do saco detetado. Enquanto não tiver um saco detetado dolado correto que me foi  dado, mexo  pescoço
                
                
                self.activate_yolo_objects(activate_objects=True)

                list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]
                position_index = 0

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                correct_bag_detected = False
                
                while not correct_bag_detected:
                    pos_offset = list_of_neck_position_search[position_index]
                    new_neck_pos = [self.look_navigation[0] + pos_offset[0], self.look_navigation[1] + pos_offset[1]]
                    print('pescoço: ', new_neck_pos)
                    
                    self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                    
                    time.sleep(5)
                
                    # self.current_image = self.node.detected_objects.image_rgb
                    # bridge = CvBridge()
                    # Convert ROS Image to OpenCV image
                    # cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                    # self.image_most_obj_detected= cv_image
                    
                    
                    ### VARIÁVEL REFEREE_X TEM DE ME SER DADA PELO POINT CLOUD DA PESSOA
                    referee_x = 0.0
                
                    correct_bag_detected, relative_position_of_bag = self.detect_bag(position_of_referee_x= referee_x, received_bag= received_bag)
                
                    # Move to the next position
                    position_index = (position_index + 1) % len(list_of_neck_position_search)
                    print(position_index)
                
                self.activate_yolo_objects(activate_objects=False)

                # next state
                self.state = self.Go_to_bag 

            elif self.state == self.Go_to_bag:
                print("State:", self.state, "- Go_to_bag")

                # set rgb's to rotating green
                self.set_rgb(GREEN+ROTATE)

                # speech: "I'm going to go to the bag."
                self.set_speech(filename="carry_my_luggage/going_to_bag", wait_for_end_of=True)

                print('Navigate to: ', relative_position_of_bag.x, relative_position_of_bag.y)

                

                self.set_speech(filename="carry_my_luggage/might_touch_bag", wait_for_end_of=False)

                self.set_navigation(movement="move", target = [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                print('received bag: ', received_bag)
                if received_bag == 'left':
                    self.IMU_ANGLE -=20.0
                    print('left')
                    state = self.Pick_bag_left
                elif received_bag == 'right':
                    self.IMU_ANGLE += 20.0
                    print('\n \n \n \n right \n \n \n')
                    state = self.Pick_bag_right
                else:
                    self.IMU_ANGLE += 0.0
                    state = self.Go_to_bag

                self.set_navigation(movement="orientate", absolute_angle= self.IMU_ANGLE, flag_not_obs=True, wait_for_end_of=True)

                # look at bag (set neck with the bag's coordinates)

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                # navigation (how?)

                # next state
                self.state = state
            
            elif self.state == self.Pick_bag_left:
                print("State:", self.state, "- Pick_bag")
                # your code here ...  

                # set rgb's to purple
                self.set_rgb(MAGENTA+ROTATE)

                # self.torso_pos.x = -1.0
                # self.torso_test_publisher.publish(self.torso_pos)

                # speech: "I'm picking up the bag now."
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)

                # self.torso_pos.x = 0.0
                # self.torso_test_publisher.publish(self.torso_pos)

                # move arm to bag's position (how?)
                self.set_arm(command="carry_my_luggage_pre_check_bag", wait_for_end_of=True)

                list_of_rotations = [0.0, -30.0, -60.0]
                raio = [1.0, 0.7, 0.4, 0.0]
                counter = 0

                i = 0
                first_time = True
                object_in_gripper = False
                while not object_in_gripper:
                    while self.INITIAL_REACHED_RADIUS > 0.0:
                        for pos in list_of_rotations:
                            if pos == 0.0 and first_time == True:
                                first_time = False
                                i += 1
                                if i == len(list_of_rotations):
                                    print('já rodei 2 vezes pelo imu')
                                    break
                            else:
                                print('ciclo de orientation imu')
                                # self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                                object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                                # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                                
                                if not object_in_gripper:
                                    self.set_rgb(command=RED+BLINK_LONG)
                            
                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=False)
                                    
                                    self.set_arm(command="carry_my_luggage_if_failed_pick", wait_for_end_of=True)
                                    
                                    print('before navigation orientate')

                                    self.set_navigation(movement="orientate", absolute_angle= self.IMU_ANGLE + pos, flag_not_obs=True, wait_for_end_of=True)
                                    print('Angle: ', self.IMU_ANGLE + pos )

                                    i += 1

                                    print('Antes de voltar a movimentar braço para saco')
                                    self.set_arm(command="carry_my_luggage_pick_bag_after_failing", wait_for_end_of=True)

                                    if i == len(list_of_rotations):
                                        print('Iterations', i)
                                        object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                        if not object_in_gripper:
                                            self.set_rgb(command=RED+BLINK_LONG)
                                            self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=False)
                                            self.set_arm(command="carry_my_luggage_if_failed_pick", wait_for_end_of=True)
                                        else:
                                            print('Funcionou!')
                                            self.set_rgb(command=GREEN+BLINK_LONG)
                                            self.set_arm(command="carry_my_luggage_bag_picked_correctly", wait_for_end_of=True)
                                            object_in_gripper = True
                                        break
                                else:
                                    print('Funcionou!')
                                    self.set_rgb(command=GREEN+BLINK_LONG)
                                    self.set_arm(command="carry_my_luggage_bag_picked_correctly", wait_for_end_of=True)
                                    object_in_gripper = True
                                    break
                        if not object_in_gripper:
                            counter += 1
                            self.INITIAL_REACHED_RADIUS = raio[counter]
                            print('Radius ',self.INITIAL_REACHED_RADIUS)
                            i = 0
                            print('vou rodar para saco e andar para ele')
                            self.set_speech(filename="carry_my_luggage/might_touch_bag", wait_for_end_of=False)
                            list_of_rotations[0] -= 5.0 
                            list_of_rotations[1] -= 5.0 
                            list_of_rotations[2] -= 5.0 
                            self.set_navigation(movement="rotate", target= [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, wait_for_end_of=True)
                            self.set_navigation(movement="move", target = [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                            print('andei para a frente, estou pronto para testar de novo')
                        else:
                            print('funcionou')
                            break
                    

                # close claw (how?)
                # raise arm


                # next state
                self.state = self.Final_State 

            elif self.state == self.Pick_bag_right:
                print("State:", self.state, "- Pick_bag")
                # your code here ...  

                # set rgb's to purple
                self.set_rgb(MAGENTA+ROTATE)

                # self.torso_pos.x = -1.0
                # self.torso_test_publisher.publish(self.torso_pos)

                # speech: "I'm picking up the bag now."
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)

                # self.torso_pos.x = 0.0
                # self.torso_test_publisher.publish(self.torso_pos)

                # move arm to bag's position (how?)
                self.set_arm(command="carry_my_luggage_pre_check_bag", wait_for_end_of=True)
                
                list_of_rotations = [0.0, 30.0, 60.0]

                #raio = [0.9, 0.4, 0.2, 0.0]
                raio = [1.0, 0.7, 0.4, 0.0]
                counter = 0

                i = 0
                first_time = True
                object_in_gripper = False
                while not object_in_gripper:
                    while self.INITIAL_REACHED_RADIUS > 0.0:
                        for pos in list_of_rotations:
                            if pos == 0.0 and first_time == True:
                                first_time = False
                                i += 1
                                if i == len(list_of_rotations):
                                    print('já rodei 2 vezes pelo imu')
                                    break
                            else:
                                print('ciclo de orientation imu')
                                # self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                                object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                                # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                                
                                if not object_in_gripper:
                                    self.set_rgb(command=RED+BLINK_LONG)
                            
                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=False)
                                    
                                    self.set_arm(command="carry_my_luggage_if_failed_pick", wait_for_end_of=True)
                                    
                                    print('before navigation orientate')

                                    self.set_navigation(movement="orientate", absolute_angle= self.IMU_ANGLE + pos, flag_not_obs=True, wait_for_end_of=True)

                                    i += 1

                                    print('Antes de voltar a movimentar braço para saco')
                                    self.set_arm(command="carry_my_luggage_pick_bag_after_failing", wait_for_end_of=True)

                                    if i == len(list_of_rotations):
                                        print('Iterations', i)
                                        object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                        if not object_in_gripper:
                                            self.set_rgb(command=RED+BLINK_LONG)
                                            self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=False)
                                            self.set_arm(command="carry_my_luggage_if_failed_pick", wait_for_end_of=True)
                                        else:
                                            print('Funcionou!')
                                            self.set_rgb(command=GREEN+BLINK_LONG)
                                            self.set_arm(command="carry_my_luggage_bag_picked_correctly", wait_for_end_of=True)
                                            object_in_gripper = True
                                        break
                                else:
                                    print('Funcionou!')
                                    self.set_rgb(command=GREEN+BLINK_LONG)
                                    self.set_arm(command="carry_my_luggage_bag_picked_correctly", wait_for_end_of=True)
                                    object_in_gripper = True
                                    break
                        if not object_in_gripper:
                            counter += 1
                            self.INITIAL_REACHED_RADIUS = raio[counter]
                            print('Radius ',self.INITIAL_REACHED_RADIUS)
                            i = 0
                            print('vou rodar para saco e andar para ele')
                            self.set_speech(filename="carry_my_luggage/might_touch_bag", wait_for_end_of=False)
                            list_of_rotations[0] += 5.0 
                            list_of_rotations[1] += 5.0 
                            list_of_rotations[2] += 5.0 
                            self.set_navigation(movement="rotate", target= [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, wait_for_end_of=True)
                            self.set_navigation(movement="move", target = [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                            print('andei para a frente, estou pronto para testar de novo')
                        else:
                            print('funcionou')
                            break
                    



                # close claw (how?)
                # raise arm


                # next state
                self.state = self.Final_State 

            elif self.state == self.Camera_pick_bag:


                
                s,m = self.set_arm(command="carry_my_luggage_pre_pick", wait_for_end_of=True)
                print("carry_my_luggage_pre_pick", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)

                # bag_coords = self.get_bag_pick_cordinates()
                # print(bag_coords)
                # self.set_navigation(movement="adjust", adjust_distance=bag_coords[4], adjust_direction=bag_coords[3], wait_for_end_of=True)

                
                s,m = self.set_arm(command="carry_my_luggage_pick_bag", wait_for_end_of=True)
                print("carry_my_luggage_pick_bag", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)

                s,m = self.set_arm(command="carry_my_luggage_post_pick", wait_for_end_of=True)
                print("carry_my_luggage_post_pick", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)

                s,m = self.set_arm(command="carry_my_luggage_give_bag", wait_for_end_of=True)
                print("carry_my_luggage_give_bag", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)

                s,m = self.set_arm(command="open_gripper", wait_for_end_of=True)
                print("open_gripper", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)

                s,m = self.set_arm(command="carry_my_luggage_post_give_bag", wait_for_end_of=True)
                print("carry_my_luggage_post_give_bag", s,m)
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)
                time.sleep(3)


            elif self.state == self.Final_State:
                print("State:", self.state, "- Final_State")
                # your code here ...

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                # set rgb's to static cyan
                self.set_rgb(CYAN+ROTATE)

                # Speech: "I have picked up the bag, however I cannot follow you. I finished my task."
                self.set_speech(filename="carry_my_luggage/end_of_carry_luggage", wait_for_end_of=True)

                # wait for start_button
                self.wait_for_start_button()

                # set rgb's to static green
                self.set_rgb(GREEN+SET_COLOUR)

                self.set_arm(command="carry_my_luggage_back_to_initial_position", wait_for_end_of=True)


                while True:
                    pass

            else:
                pass