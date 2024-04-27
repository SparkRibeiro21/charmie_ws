#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger

import cv2 
import threading
import time
from cv_bridge import CvBridge
from pathlib import Path
from datetime import datetime
import math
import numpy as np
import face_recognition
import os

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("Receptionist")
        self.get_logger().info("Initialised CHARMIE Receptionist Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        # self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        # Arm CHARMIE
        # self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        # self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        # self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        # self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        # self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")

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
        # while not self.neck_track_object_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        # Yolos
        while not self.activate_yolo_pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")

        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_track_person = False
        # self.waited_for_end_of_track_object = False
        # self.waited_for_end_of_arm = False # not used, but here to be in conformity with other uses

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        # self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        # self.arm_ready = True

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
        # self.track_object_success = True
        # self.track_object_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        # self.activate_yolo_objects_success = True
        # self.activate_yolo_objects_message = ""
        self.arm_success = True
        self.arm_message = ""

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        
        # cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        # cv2.waitKey(10)

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

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
            self.speech_sucecss = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_coords = True
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


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ReceptionistNode()
    th_main = threading.Thread(target=ThreadMainReceptionist, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainReceptionist(node: ReceptionistNode):
    main = ReceptionistMain(node)
    main.main()

class ReceptionistMain():

    def __init__(self, node: ReceptionistNode):
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
        
    def get_audio(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, question="", wait_for_end_of=True):

        if yes_or_no or receptionist or gpsr or restaurant:

            # this code continuously asks for new audio info eveytime it gets an error for mishearing
            audio_error_counter = 0
            keywords = "ERROR"
            while keywords=="ERROR":
                
                self.set_speech(filename=question, wait_for_end_of=True)                
                self.set_face("demo7")
                self.node.call_audio_server(yes_or_no=yes_or_no, receptionist=receptionist, gpsr=gpsr, restaurant=restaurant, wait_for_end_of=wait_for_end_of)
                
                if wait_for_end_of:
                    while not self.node.waited_for_end_of_audio:
                        pass
                self.node.waited_for_end_of_audio = False
                self.set_face("demo5")

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
        
        if custom == "":
            temp = String()
            temp.data = command
            self.node.image_to_face_publisher.publish(temp)
        else:
            temp = String()
            temp.data = custom
            self.node.custom_image_to_face_publisher.publish(temp)

        self.node.face_success = True
        self.node.face_message = "Value Sucessfully Sent"

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
        
    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def track_person(self, person, body_part="Head", wait_for_end_of=True):

        self.node.call_neck_track_person_server(person=person, body_part=body_part, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_person:
            pass
        self.node.waited_for_end_of_track_person = False

        return self.node.track_person_success, self.node.track_person_message
 
    def set_initial_position(self, initial_position):

        task_initialpose = PoseWithCovarianceStamped()

        task_initialpose.header.frame_id = "map"
        task_initialpose.header.stamp = self.node.get_clock().now().to_msg()

        task_initialpose.pose.pose.position.x = initial_position[1]
        task_initialpose.pose.pose.position.y = -initial_position[0]
        task_initialpose.pose.pose.position.z = 0.0

        quaternion = self.get_quaternion_from_euler(0,0,math.radians(initial_position[2]))

        task_initialpose.pose.pose.orientation.x = quaternion[0]
        task_initialpose.pose.pose.orientation.y = quaternion[1]
        task_initialpose.pose.pose.orientation.z = quaternion[2]
        task_initialpose.pose.pose.orientation.w = quaternion[3] 
        
        self.node.initialpose_publisher.publish(task_initialpose)

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
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    

    # main state-machine function
    def main(self):
        # examples of names of states
        # use the names of states rather than just numbers to ease 3rd person code analysis
        Waiting_for_start_button = 0
        Receive_first_guest = 1
        Navigation_to_sofa = 2
        Presentation_host_first_guest = 3
        Navigate_to_starting_point = 4
        Receive_second_guest = 5
        Navigation_to_sofa_second = 6
        Presentation_host_first_second_guest = 7
        Final_State = 8

        self.SIDE_TO_LOOK = "right"

        self.state = Waiting_for_start_button

        self.look_forward = [0, 0]
        self.look_navigation = [0, -40]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
        self.look_torso = [0, -5]
        self.look_down_sofa = [0, -10]

        self.look_empty_place = [1.0, 2.0]
        self.look_sofa = [-2.5, 3.0]

        self.host_name = "John"
        self.host_drink = "Milk"
        self.host_drink = self.host_drink.replace(' ', '_') # if someone forgets to write correctly
        self.host_filename = ""
        self.host_position = ""

        self.guest1_name = ""
        self.guest1_drink = ""
        self.guest1_filename = ""
        self.guest1_ethnicity = ""
        self.guest1_age = ""
        self.guest1_gender = ""
        self.guest1_height = ""
        self.guest1_shirt_color = ""
        self.guest1_pants_color = ""

        self.guest2_name = "" 
        self.guest2_drink = ""
        
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_receptionist/charmie_receptionist/images"
        self.complete_path_save_images = home+'/'+midpath+'/'

        # create a look sofa

        # debug print
        print("IN NEW MAIN")

        while True:

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')

                self.set_face("demo5")

                self.activate_yolo_pose(activate=False)

                self.set_initial_position([-2.5, 1.5, 0])

                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                self.set_rgb(MAGENTA+ALTERNATE_QUARTERS)
                
                self.set_speech(filename="receptionist/start_receptionist", wait_for_end_of=True)
                
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)
                
                self.wait_for_start_button()

                self.set_rgb(CYAN+ALTERNATE_QUARTERS)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ### NAVIGATION MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                
                self.state = Receive_first_guest

            elif self.state == Receive_first_guest:
                print('State 1 = Receive first guest')

                ### OPEN THE DOOR ???

                self.calibrate_audio(wait_for_end_of=True)

                self.set_neck(position=self.look_torso, wait_for_end_of=True)

                self.set_rgb(YELLOW+ROTATE)

                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)

                self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True, characteristics=True)
                
                self.guest1_filename, self.guest1_ethnicity, self.guest1_age, self.guest1_gender, self.guest1_height, self.guest1_shirt_color, self.guest1_pants_color = self.search_for_guest_and_get_info() # search for guest 1 and returns all info regarding guest 1
                print(self.guest1_filename, self.guest1_ethnicity, self.guest1_age, self.guest1_gender, self.guest1_height, self.guest1_shirt_color, self.guest1_pants_color)

                ### RENATA: PROCESS CHARACTERISTICS
                
                self.activate_yolo_pose(activate=False)

                self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)

                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                print("Finished:", command)
                keyword_list= command.split(" ")
                self.guest1_name = keyword_list[0] 
                self.guest1_drink = keyword_list[1]
                print(self.guest1_name, self.guest1_drink)
                # self.set_speech(filename="receptionist/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                # self.set_speech(filename="receptionist/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)

                ########## ADICIONAR UM: NICE TO MEET YOU + NOME DA PESSOA

                self.set_rgb(GREEN+BLINK_LONG)
            
                self.state = Navigation_to_sofa

            elif self.state == Navigation_to_sofa:
                print('State 3 = Navigation to sofa')

                self.set_speech(filename="receptionist/please_follow_me", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                ### NAVIGATION: MOVING TO THE SOFA
                
                if self.SIDE_TO_LOOK.lower() == "right":

                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_guest

            elif self.state == Presentation_host_first_guest:
                print('State 4 = Presentation host and first guest')

                self.set_neck(position=self.look_down_sofa, wait_for_end_of=True)

                self.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                self.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                self.activate_yolo_pose(activate=True, only_detect_person_legs_visible=True)

                self.host_filename, self.host_position = self.search_for_host()
                print(self.host_filename, self.host_position)

                self.activate_yolo_pose(activate=False)
                
                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)
                
                if self.SIDE_TO_LOOK.lower() == "right":
                
                    self.set_neck(position=self.look_right, wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":

                    self.set_neck(position=self.look_left, wait_for_end_of=True)
                

                ########## SE MUDARMOS A ORDEM DAS FALAS É MENOS UMA ROTAÇÃO DO NECK E FICA MAIS INTUITIVO PORQUE ESTAMOS
                ########## PORQUE ESTAMOS A OLHAR PARA O HOST, DIZEMOS AO GUEST PARA ELE SE SENTAR PARA ONDE ESTAMOS A OLHAR
                ########## MAS COMEÇAMOS A FALAR DE REPENDE PARA O GUEST, ELE NEM SE APERCEBE

                ### SPEAK: HOST INFORMATION
                self.set_speech(filename="receptionist/recep_host_"+self.host_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+self.host_drink.lower(), wait_for_end_of=True)

                ### NECK: TURN TO HOST
                self.set_neck_coords(position=self.host_position, ang=-10)

                ### SPEAK GUEST NAME AND FAVOURITE DRINK
                self.set_speech(filename="receptionist/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)

                ### NECK LOOK AT SOFA
                self.set_neck_coords(position=self.look_sofa, ang=-20, wait_for_end_of=True)

                ### SEARCH FOR AN EMPTY SEAT: ONLY FOR ROBOCUP

                self.set_speech(filename="receptionist/found_empty_seat", wait_for_end_of=True)

                self.set_speech(filename="receptionist/please_sit_sofa", wait_for_end_of=True)
                
                self.set_rgb(GREEN+BLINK_LONG)
                
                self.state = Navigate_to_starting_point

            elif self.state == Navigate_to_starting_point:
                print('State 1 = Navigate_to_starting_point')

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                ### NAVIGATION : MOVE TO RECEIVE THE SECOND GUEST POSITION

                self.state = Receive_second_guest

            elif self.state == Receive_second_guest:
                print('State 1 = Receive second guest')




                # TEMPORARY 
                # self.guest1_name = "Axel"
                # self.guest1_drink = "Red Wine"
                # self.guest1_drink = self.guest1_drink.replace(' ', '_') # if someone forgets to write correctly



                ### OPEN THE DOOR ???                

                self.calibrate_audio(wait_for_end_of=True)

                self.set_neck(position=self.look_torso, wait_for_end_of=True)

                self.set_rgb(YELLOW+ROTATE)

                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)

                self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True, characteristics=False)
                
                self.search_for_guest() # search for guest 2 - only need to look at guest 

                self.activate_yolo_pose(activate=False)

                self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)

                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                print("Finished:", command)
                keyword_list= command.split(" ")
                self.guest2_name = keyword_list[0] 
                self.guest2_drink = keyword_list[1]
                print(self.guest2_name, self.guest2_drink)
                # self.set_speech(filename="receptionist/recep_second_guest_"+self.guest2_name.lower(), wait_for_end_of=True)
                # self.set_speech(filename="receptionist/recep_drink_"+self.guest2_drink.lower(), wait_for_end_of=True)

                self.set_rgb(GREEN+BLINK_LONG)
                self.state = Navigation_to_sofa_second
                
            elif self.state == Navigation_to_sofa_second:
                print('State 3 = Navigation to sofa')

                self.set_speech(filename="receptionist/please_follow_me", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                ### NAVIGATION: MOVING TO THE SOFA
                
                if self.SIDE_TO_LOOK.lower() == "right":

                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_second_guest

            elif self.state == Presentation_host_first_second_guest:
                print('State 4 = Presentation host, first and second guest')

                self.set_neck(position=self.look_down_sofa, wait_for_end_of=True)

                self.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                self.set_speech(filename="receptionist/dear_guest", wait_for_end_of=True)
                self.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                self.activate_yolo_pose(activate=True, only_detect_person_legs_visible=True)
                
                ### SEARCH FOR HOST AND GUEST1 AND RECEIVE THEIR LOCATION
                #AJUDA
                person_name_1=self.face_recognition(self.host_filename,self.complete_path_save_images)
                person_name_2=self.face_recognition(self.guest1_filename,self.complete_path_save_images)

                self.activate_yolo_pose(activate=False)
                
                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)

                if self.SIDE_TO_LOOK.lower() == "right":
                
                    self.set_neck(position=self.look_right, wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":

                    self.set_neck(position=self.look_left, wait_for_end_of=True)

                ### SPEAK: HOST INFORMATION
                self.set_speech(filename="receptionist/recep_host_"+self.host_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+self.host_drink.lower(), wait_for_end_of=True)

                ### SPEAK: GUEST1 INFORMATION
                self.set_speech(filename="receptionist/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)

                ### SPEAK GUEST1 CHARACTERISTICS
                #self.guest1_age,self.guest1_gender,self.guest1_ethnicity,self.guest1_height,self.guest1_shirt_color, self.guest1_pants_color = self.Get_characteristics(self.guest1_age,self.guest1_gender,self.guest1_ethnicity,self.guest1_height,self.guest1_shirt_color, self.guest1_pants_color)
                self.get_characteristics(self.guest1_age,self.guest1_gender,self.guest1_ethnicity,self.guest1_height,self.guest1_shirt_color, self.guest1_pants_color)
                
                # self.set_speech(filename="receptionist/found_empty_seat", wait_for_end_of=True) # missing color
                

                self.set_neck(position=self.look_forward, wait_for_end_of=True)

                ### SPEAK: GUEST2 INFORMATION
                self.set_speech(filename="receptionist/recep_second_guest_"+self.guest2_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+self.guest2_drink.lower(), wait_for_end_of=True)

                self.set_neck_coords(position=self.look_sofa, ang=-20, wait_for_end_of=True)

                ### SEARCH FOR AN EMPTY SEAT: ONLY FOR ROBOCUP                
                
                self.set_speech(filename="receptionist/found_empty_seat", wait_for_end_of=True)

                self.set_speech(filename="receptionist/please_sit_sofa", wait_for_end_of=True)

                self.set_rgb(GREEN+BLINK_LONG)
                
                self.state = Final_State

            elif self.state == Final_State:
                
                print("Finished task!!!")
                #NECK: LOOK IN FRONT
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                #SPEAK: Thank you. I finished my receptionist task
                self.set_speech(filename="receptionist/finish_receptionist", wait_for_end_of=True)
                #NECK: LOOK TO THE FLOOR
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                self.set_rgb(BLUE+ROTATE)
                
                # After finishing the task stays in this loop 
                while True:
                    pass

            else:
                pass

    def search_for_host(self):
    
        self.set_rgb(MAGENTA+HALF_ROTATE)
        time.sleep(0.5)

        detected_person_temp = Yolov8Pose()
        start_time = time.time()
        host = DetectedPerson()
        host_found = False

        while not host_found:
            while time.time() - start_time < 1.0:
                detected_person_temp = self.node.detected_people  
                if detected_person_temp.num_person == 0:  
                    start_time = time.time()
                    self.set_rgb(RED+HALF_ROTATE)
                else:
                    self.set_rgb(GREEN+HALF_ROTATE)
                time.sleep(0.2)
            
            is_cropped = False
            for p in detected_person_temp.persons:
                print(p.room_location, p.furniture_location)
                
                if p.room_location == "Living Room" and p.furniture_location == "Sofa":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA YES")

                # if the robot localisation is a bit off and i do not detect anyone in the sofa, i just check for people in the living room
                elif p.room_location == "Living Room":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA NO")
                else:
                    print("CLOSEST PERSON")
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("OUTSIDE")

        self.set_rgb(WHITE+HALF_ROTATE)



        """
        # time.sleep(0.5)
    
        # host = DetectedPerson()
        # detected_person_temp = Yolov8Pose()
        # host_found = False

        self.set_rgb(MAGENTA+HALF_ROTATE)
        
        while detected_person_temp.num_person == 0 or host_found == False: #  and host.room_location:
            detected_person_temp = self.node.detected_people

            for p in detected_person_temp.persons:
                
                print(p.room_location, p.furniture_location)
                
                if p.room_location == "Living Room" and p.furniture_location == "Sofa":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA YES")
                # if the robot localisation is a bit off and i do not detect anyone in the sofa, i just check for people in the living room
                elif p.room_location == "Living Room":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA NO")
                else:
                    print("CLOSEST PERSON")
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("OUTSIDE")
        self.set_rgb(BLUE+HALF_ROTATE)

        # self.set_rgb(GREEN+BLINK_LONG)

        """
        
        self.set_neck_coords(position=[host.position_absolute.x, host.position_absolute.y], ang=-10, wait_for_end_of=True)

        return filename, [host.position_absolute.x, host.position_absolute.y]


    def search_for_host_and_guest1(self):
        pass

                
    def search_for_guest_and_get_info(self):

        self.search_for_guest()

        time.sleep(0.5)

        detected_person_temp = Yolov8Pose()
        start_time = time.time()
        guest = DetectedPerson()
        is_cropped = False
        while not is_cropped:
            while time.time() - start_time < 1.0:
                detected_person_temp = self.node.detected_people  
                if detected_person_temp.num_person == 0:  
                    start_time = time.time()
                    self.set_rgb(YELLOW+HALF_ROTATE)
                else:
                    self.set_rgb(GREEN+HALF_ROTATE)
                time.sleep(0.2)
            guest = detected_person_temp.persons[0]
            is_cropped, filename = self.crop_face(guest, detected_person_temp.image_rgb)

        self.set_rgb(WHITE+HALF_ROTATE)

        # filename is the full path of this guest image
        return filename, guest.ethnicity, guest.age_estimate, guest.gender, guest.height, guest.shirt_color, guest.pants_color

    def search_for_guest(self):
    
        self.set_rgb(MAGENTA+HALF_ROTATE)
        time.sleep(0.5)

        detected_person_temp = Yolov8Pose()
        start_time = time.time()
        while time.time() - start_time < 3.0:
            detected_person_temp = self.node.detected_people  
            if detected_person_temp.num_person == 0:  
                start_time = time.time()
                self.set_rgb(RED+HALF_ROTATE)
            else:
                self.set_rgb(YELLOW+HALF_ROTATE)
            time.sleep(0.2)

        self.set_rgb(WHITE+HALF_ROTATE)

        self.track_person(person=detected_person_temp.persons[0], wait_for_end_of=True)


    def crop_face(self, new_person, current_frame_image_msg):

        MIN_KP_CONF_VALUE = 0.5

        current_frame = self.node.br.imgmsg_to_cv2(current_frame_image_msg, "bgr8")        

        # y1 = top of bounding box y
        # y2 = y of lowest height shoulder
        # x1 = keypoint more to the left
        # x2 = keypoint more to the right
        
        # using all face and shoulders keypoints to make sure face is correctly detected
        if new_person.kp_shoulder_right_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_shoulder_left_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_eye_right_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_eye_left_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_nose_conf > MIN_KP_CONF_VALUE:
            # new_person.kp_ear_right_conf > MIN_KP_CONF_VALUE and \
            # new_person.kp_ear_left_conf > MIN_KP_CONF_VALUE and \
            
            y1 = new_person.box_top_left_y
            y2 = max(new_person.kp_shoulder_right_y, new_person.kp_shoulder_left_y)

            x1 = min(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)
            x2 = max(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)
                
            # same time for all obejcts
            current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
            filename = self.complete_path_save_images+current_datetime+".jpg"

            cv2.imwrite(filename, current_frame[y1:y2, x1:x2])
            return True, filename
            
        else:
            return False, "None"
    
    #TIAGO AJUDA A RECEBER AS IMAGENS CERTAS E A UTILIZAR A PASTA CORRETA ONDE VÃO ESTAR AS IMAGENS
    def face_recognition(self, image, folder_images):
       
        image = face_recognition.load_image_file(image)

        encoding_entry = face_recognition.face_encodings(image)

        if len(encoding_entry) == 0:
            return [("Unknown", 0)], None, 0  

        encoding_entry = encoding_entry[0]  

        encoding_knowns = []
        names = []

        for nome_arquivo in os.listdir(folder_images):
            caminho_arquivo = os.path.join(folder_images, nome_arquivo)
            imagem = face_recognition.load_image_file(caminho_arquivo)
            encoding = face_recognition.face_encodings(imagem)

            if len(encoding) == 0:
                continue 

            encoding = encoding[0]  
            encoding_knowns.append(encoding)
            nome_conhecido = os.path.splitext(nome_arquivo)[0] 
            names.append(nome_conhecido)

        if not encoding_knowns:  
            return [("Unknown", 0)], None, 0

        all_percentages = []
        for encoding_knowns in encoding_knowns:
            distancia = face_recognition.face_distance([encoding_knowns], encoding_entry)[0]
            confidance = (1 - distancia) * 100
            all_percentages.append(confidance)

        person_recognized, biggest_confidance = max(zip(names, all_percentages), key=lambda x: x[1])

        if biggest_confidance < 40:
            person_recognized = "Unknown"

        return person_recognized



    def get_characteristics(self, race, age, gender, height, shirt_color, pant_color):
        characteristics = []
        none_variables = []

        if race is not None:
            characteristics.append(race)
        else:
            none_variables.append("race")

        if age is not None:
            characteristics.append(age)
        else:
            none_variables.append("age")

        if gender is not None:
            characteristics.append(gender)
        else:
            none_variables.append("gender")

        if height is not None:
            if height > 1.55: 
                height='taller'
            elif height < 1.40:
                height='smaller'
            else:
                height='equal'
            characteristics.append(height)
        else:
            none_variables.append("height")

        if shirt_color is not None:
            characteristics.append(shirt_color)
        else:
            none_variables.append("shirt_color")

        if pant_color is not None:
            characteristics.append(pant_color)
        else:
            none_variables.append("pant_color")

        if not characteristics:  # Se nenhuma característica foi fornecida
            print("Nenhuma característica fornecida")
            return None

        for variable in none_variables:
            if variable == "age":
                age = 'Between 25 and 32'
                characteristics.append(age)
            elif variable == "gender":
                gender = "Male"
                characteristics.append(gender)
            elif variable == "race":
                race = "Caucasian"
                characteristics.append(race)
            elif variable == "height":
                height = "Taller than me"
                characteristics.append(height)
            elif variable == "shirt_color":
                shirt_color = "White"
                characteristics.append(shirt_color)
            elif variable == "pant_color":
                pass  # Deixa a cor da calça vazia
            else:
                print("Empty:", variable)

        self.set_speech(filename="receptionist/the_first_guest_is", wait_for_end_of=True)
        self.set_speech(filename="receptionist/race_"+race.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/gender_"+gender.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/age_"+age.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/height_"+height.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/the_shirt_color_is"+shirt_color.lower(), wait_for_end_of=True)

        #return age,gender,race,height,shirt_color, pant_color