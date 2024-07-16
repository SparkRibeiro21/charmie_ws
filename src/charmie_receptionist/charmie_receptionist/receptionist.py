#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL, ListOfDetectedPerson, ArmController
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger, SetFace

import os
import cv2 
import threading
import time
from cv_bridge import CvBridge
from pathlib import Path
from datetime import datetime
import math
import numpy as np
import face_recognition


# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("Receptionist")
        self.get_logger().info("Initialised CHARMIE Receptionist Node")

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
        # self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10) 
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Search for person and object 
        self.search_for_person_detections_publisher = self.create_publisher(ListOfDetectedPerson, "search_for_person_detections", 10)

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
        # self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        # self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        # self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        while not self.face_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Face Command...")
        # Audio
        while not self.get_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Audio Server...")
        while not self.calibrate_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Calibrate Audio Server...")
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
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")

        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_face = False
        # self.waited_for_end_of_track_object = False
        # self.waited_for_end_of_arm = False # not used, but here to be in conformity with other uses

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        # self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        # self.arm_ready = True
        self.flag_navigation_reached = False

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
        self.navigation_success = True
        self.navigation_message = ""
        self.waited_for_end_of_arm = False


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

        self.SIDE_TO_LOOK = "left"
        self.OPEN_DOOR = False

        # Start localisation position: ### LAR ###
        # self.initial_position = [-1.7, 3.5, 180.0]
        # self.initial_position_with_door = [-0.33, 1.4, -135.0]

        # Navigation Positions: ### LAR ###
        # self.front_of_sofa = [-2.5, 1.5]
        # self.sofa = [0.91, 8.925]
        # self.receive_guests = [-1.0, 2.5]
        # self.where_guest_is_received = [0.0, 1.5]
        # self.map_initial_position = [0.25, 0.0]

        # Start Localisation: ### RoboCup24 ###
        self.initial_position_debug = [0.45, 5.35, 0.0]
        self.initial_position = [0.0, 2.10, 180.0]

        self.receive_guests = [0.0, 1.45]
        self.pre_room_door = [0.85, 3.15]
        self.post_room_door = [0.85, 4.70]
        self.front_of_sofa = [0.85, 5.35]

        self.map_initial_position = [0.0, 0.0]


        # Start Localisation Position: ### FNR24 ###
        # self.initial_position = [-1.0, 3.0, -90.0]
        # self.initial_position_with_door = [-1.3, 1.7, -135.0]

        # Navigation Positions: ### FNR24 ###
        # self.front_of_sofa = [-0.3, 3.7]
        # self.sofa = [3.0, 3.2]
        # self.centro_cadeiras = [3.0, 3.0]
        # self.receive_guests = [1.0, 1.8]
        # self.where_guest_is_received = [-2.8, 1.5]
        # self.map_initial_position = [0.0, 0.0]
 

        self.look_forward = [0, 0]
        self.look_navigation = [0, -40]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
        self.look_torso = [0, 5]
        self.look_down_sofa = [0, -25]
        self.look_door = [-45, 0]

        self.look_empty_place = [1.0, 2.0]
        # LAR: self.look_sofa = [-2.5, 3.0]
        self.look_sofa = [0.91, 8.92] 

        self.MAX_SPEED = 30

        self.host_name = "Noah"
        self.host_drink = "Iced Tea"
        self.host_drink = self.host_drink.replace(' ', '_') # if someone forgets to write correctly
        self.host_filename = ""
        self.host_position = ""

        self.guest1_name = ""
        self.guest1_drink = ""
        self.guest1_filename = ""
        self.guest1_ethnicity = ""
        self.guest1_age = ""
        self.guest1_gender = ""
        self.guest1_height = 0.0
        self.guest1_shirt_color = ""
        self.guest1_pants_color = ""

        self.guest2_name = "" 
        self.guest2_drink = ""
        
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_receptionist/charmie_receptionist/images"
        self.complete_path_save_images = home+'/'+midpath+'/'

        # self.set_initial_position(self.initial_position)
        self.state = Waiting_for_start_button

        
        
        # debug print
        print("IN NEW MAIN")

        while True:

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')
                
                if self.OPEN_DOOR == True:
                    self.set_initial_position(self.initial_position)
                else:
                    # self.set_initial_position(self.initial_position)
                    self.set_initial_position(self.initial_position)

                print("SET INITIAL POSITION")

                time.sleep(1)

                self.set_face("charmie_face")

                self.activate_yolo_pose(activate=False)

                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                self.set_rgb(MAGENTA+ALTERNATE_QUARTERS)
                
                # self.set_navigation(movement="rotate", target=self.receive_guests, flag_not_obs=True, wait_for_end_of=True)
                
                self.set_speech(filename="receptionist/start_receptionist", wait_for_end_of=True)
                
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True)
                
                self.wait_for_start_button()

                self.set_rgb(CYAN+ALTERNATE_QUARTERS)
                
                if self.OPEN_DOOR == True:

                    self.set_speech(filename="receptionist/waiting_door_bell", wait_for_end_of=True)

                    self.set_neck(position=self.look_door, wait_for_end_of=False)

                    time.sleep(5)
                
                    self.set_speech(filename="receptionist/guest_arrived_open_door", wait_for_end_of=True)

                    self.set_arm(command="open_door_LAR", wait_for_end_of= True)

                    self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                
                self.set_navigation(movement="move", target=self.receive_guests, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.map_initial_position, flag_not_obs=True, wait_for_end_of=True)

                time.sleep(5)
                
                self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.pre_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.post_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.front_of_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                
                time.sleep(5)

                self.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.post_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.pre_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.receive_guests, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.receive_guests, max_speed=self.MAX_SPEED, reached_radius=1.0, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.map_initial_position, flag_not_obs=True, wait_for_end_of=True)
                
                time.sleep(5)
                
                self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.pre_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.post_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.front_of_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)

                while True:
                    pass

                self.state = Receive_first_guest


            elif self.state == Receive_first_guest:
                print('State 1 = Receive first guest')

                ### OPEN THE DOOR ???

                self.calibrate_audio(wait_for_end_of=True)

                self.set_neck(position=self.look_torso, wait_for_end_of=True)

                self.set_rgb(YELLOW+ROTATE)

                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                
                self.guest1_filename, self.guest1_ethnicity, self.guest1_age, self.guest1_gender, self.guest1_height, self.guest1_shirt_color, self.guest1_pants_color = self.search_for_guest_and_get_info() # search for guest 1 and returns all info regarding guest 1
                print(self.guest1_filename, self.guest1_ethnicity, self.guest1_age, self.guest1_gender, self.guest1_height, self.guest1_shirt_color, self.guest1_pants_color)
                
                # self.activate_yolo_pose(activate=False)

                
                # self.get_characteristics(race=self.guest1_ethnicity, age=self.guest1_age, gender=self.guest1_gender, height=self.guest1_height, shirt_color=self.guest1_shirt_color, pants_color=self.guest1_pants_color)
                
                # old: self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)
                self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
                print("Finished:", command)
                keyword_list= command.split(" ")
                self.guest1_name = keyword_list[0] 
                self.guest1_drink = keyword_list[1]
                print(self.guest1_name, self.guest1_drink)
                # self.set_speech(filename="receptionist/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                # self.set_speech(filename="receptionist/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)

                ########## ADICIONAR UM: NICE TO MEET YOU + NOME DA PESSOA ???

                self.set_rgb(GREEN+BLINK_LONG)
            
                self.state = Navigation_to_sofa

            elif self.state == Navigation_to_sofa:
                print('State 3 = Navigation to sofa')

                self.set_speech(filename="receptionist/please_follow_me", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                # self.set_navigation(movement="rotate", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.front_of_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="orientate", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)

                if self.SIDE_TO_LOOK.lower() == "right":

                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_guest

            elif self.state == Presentation_host_first_guest:
                print('State 4 = Presentation host and first guest')

                # temp:
                # self.guest1_name = "Evelyn"
                # self.guest1_drink = "Cola"
                # self.set_initial_position([1.0, 6.0, 0.0])


                self.set_neck(position=self.look_down_sofa, wait_for_end_of=True)

                self.set_speech(filename="receptionist/names/recep_dear_"+self.host_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                # self.activate_yolo_pose(activate=True, only_detect_person_legs_visible=True, characteristics=False)

                self.host_filename, self.host_position = self.search_for_host2()
                print(self.host_filename, self.host_position)

                # self.activate_yolo_pose(activate=False)
                
                ### NECK: TURN TO HOST
                self.set_neck_coords(position=self.host_position, ang=-10)
                time.sleep(0.5)
                self.set_speech(filename="receptionist/names/recep_dear_"+self.host_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)

                ### SPEAK GUEST NAME AND FAVOURITE DRINK
                self.set_speech(filename="receptionist/names/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/favourite_drink/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)

                ### NECK TURN TO GUEST
                if self.SIDE_TO_LOOK.lower() == "right":                
                    self.set_neck(position=self.look_right, wait_for_end_of=True)
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.set_neck(position=self.look_left, wait_for_end_of=True)
                
                self.set_speech(filename="receptionist/names/recep_dear_"+self.guest1_name.lower(), wait_for_end_of=True)
                time.sleep(0.5)

                ### SPEAK: HOST INFORMATION
                self.set_speech(filename="receptionist/names/recep_host_name", wait_for_end_of=True)
                self.set_speech(filename="receptionist/favourite_drink/recep_drink_"+self.host_drink.lower(), wait_for_end_of=True)

                ### NECK LOOK AT SOFA
                self.set_neck_coords(position=self.look_sofa, ang=-10, wait_for_end_of=True)

                ### SEARCH FOR AN EMPTY SEAT: ONLY FOR ROBOCUP

                self.set_speech(filename="receptionist/found_empty_seat", wait_for_end_of=True)

                self.set_speech(filename="receptionist/please_sit_sofa", wait_for_end_of=True)
                
                self.set_rgb(GREEN+BLINK_LONG)

                self.state = Navigate_to_starting_point

            elif self.state == Navigate_to_starting_point:
                print('State 1 = Navigate_to_starting_point')

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                
                # self.set_navigation(movement="rotate", target=self.receive_guests, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.receive_guests, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="rotate", target=self.where_guest_is_received, flag_not_obs=True, wait_for_end_of=True)
                
                
                self.state = Receive_second_guest

            elif self.state == Receive_second_guest:
                print('State 1 = Receive second guest')

                ### OPEN THE DOOR ???                

                self.calibrate_audio(wait_for_end_of=True)

                self.set_neck(position=self.look_torso, wait_for_end_of=True)

                self.set_rgb(YELLOW+ROTATE)

                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)

                # self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True, characteristics=False)
                
                self.search_for_guest() # search for guest 2 - only need to look at guest 

                self.activate_yolo_pose(activate=False)

                # old: self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)
                self.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
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
    
                # self.set_navigation(movement="rotate", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.front_of_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                
                # self.set_navigation(movement="orientate", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)

                if self.SIDE_TO_LOOK.lower() == "right":

                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_second_guest

            elif self.state == Presentation_host_first_second_guest:
                print('State 4 = Presentation host, first and second guest')

                # temp:
                # self.guest1_name = "Evelyn"
                # self.guest1_drink = "Cola"
                # self.guest2_name = "Kai"
                # self.guest2_drink = "Red_Wine"
                
                self.set_neck(position=self.look_down_sofa, wait_for_end_of=True)

                self.set_speech(filename="receptionist/names/recep_dear_"+self.host_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/names/recep_dear_"+self.guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                self.activate_yolo_pose(activate=True, only_detect_person_legs_visible=True, characteristics=False)

                ### SEARCH FOR ALL DETECTED PEOPLE AND RECEIVE THEIR LOCATION
                # total_photos, total_coords = self.search_for_host_and_guest1()
                total_photos, total_coords = self.search_for_host_and_guest1_v2()
                                
                self.activate_yolo_pose(activate=False)

                print("tot_photos:", total_photos)
                print("tot_coords:", total_coords)

                # could only detect one person
                if len(total_photos) == 0:
                    print("NO PERSON DETECTED")
                    self.set_neck_coords(position=self.look_sofa, ang=-10)
                    time.sleep(0.5)

                # could only detect one person
                elif len(total_photos) == 1:
                    print("JUST ONE DETECTED PERSON ON THE SOFA")
                    self.set_neck_coords(position=total_coords[0], ang=-10)
                    time.sleep(0.5)

                else: # if detects two or more people
                    print("TWO ORE MORE PEOPLE DETECTED ON THE SOFA OR CHAIRS")
                
                    # temp: debug
                    # self.host_filename = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/images/2024-04-28_15-49-22_.jpg"
                    # self.guest1_filename = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/images/2024-04-26_10-42-17_.jpg" 

                    # check faces to see which is HOST, GUEST OR EVEN UNKNOWN
                    host_coords, guest1_coords = self.get_host_and_guest1_coordinates(total_photos=total_photos, total_coords=total_coords)

                    print(host_coords, guest1_coords)

                    self.set_neck_coords(position=guest1_coords, ang=-20)
                    time.sleep(0.5)
                    self.set_speech(filename="receptionist/names/recep_dear_"+self.guest1_name.lower(), wait_for_end_of=True)
                
                    self.set_neck_coords(position=host_coords, ang=-20)
                    time.sleep(0.5)
                    self.set_speech(filename="receptionist/names/recep_dear_"+self.host_name.lower(), wait_for_end_of=True)

                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)

                ### SPEAK: GUEST2 INFORMATION
                self.set_speech(filename="receptionist/names/recep_second_guest_"+self.guest2_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/favourite_drink/recep_drink_"+self.guest2_drink.lower(), wait_for_end_of=True)

                if self.SIDE_TO_LOOK.lower() == "right":
                    self.set_neck(position=self.look_right, wait_for_end_of=True)
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.set_neck(position=self.look_left, wait_for_end_of=True)
                self.set_speech(filename="receptionist/names/recep_dear_"+self.guest2_name.lower(), wait_for_end_of=True)
                
                ### SPEAK: GUEST1 INFORMATION
                self.set_speech(filename="receptionist/names/recep_first_guest_"+self.guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/favourite_drink/recep_drink_"+self.guest1_drink.lower(), wait_for_end_of=True)
                
                ### SPEAK GUEST1 CHARACTERISTICS
                self.get_characteristics(race=self.guest1_ethnicity, age=self.guest1_age, gender=self.guest1_gender, height=self.guest1_height, shirt_color=self.guest1_shirt_color, pants_color=self.guest1_pants_color)
                
                ### SPEAK: HOST INFORMATION
                self.set_speech(filename="receptionist/names/recep_host_name", wait_for_end_of=True)
                self.set_speech(filename="receptionist/favourite_drink/recep_drink_"+self.host_drink.lower(), wait_for_end_of=True)
                
                self.set_neck_coords(position=self.look_sofa, ang=-10, wait_for_end_of=True)

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
                
                self.set_rgb(RAINBOW_ROT)
                
                # After finishing the task stays in this loop 
                while True:
                    pass

            else:
                pass

    def search_for_host2(self):

        host_found = False
        tetas = [[-20, -10], [20, -10]]
        is_cropped = False
        sfp_ctr = 0

        while not host_found and sfp_ctr < 2:
            sfp_ctr += 1
            people_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_legs_visible=True)

            print("FOUND:", len(people_found)) 
            # for p in people_found:
            #     print("ID:", p.index_person)

            self.set_rgb(BLUE+HALF_ROTATE)
            self.set_neck(position=[0, 0], wait_for_end_of=True)
            # time.sleep(0.5)

            host_detected_person_chair = DetectedPerson()
            host_detected_person_living_room = DetectedPerson()
            chairs_ctr = 0
            living_room_ctr = 0
            
            for p in people_found:
                print("ID:", p.index_person, p.furniture_location, p.room_location)
                # self.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                # time.sleep(3)

            for p in people_found:
                if p.furniture_location == "Sofa" or p.furniture_location == "Chair":
                    chairs_ctr += 1
                    host_detected_person_chair = p
                    print("in sofa")
                    # TIAGO : # usar distancia da pesso, e ver quem é a pessoa mais proxima!!!
                    
            for p in people_found:
                if p.room_location == "Living Room":
                    living_room_ctr += 1
                    host_detected_person_living_room = p
                    print("in living room")
                    # TIAGO : # usar distancia da pesso, e ver quem é a pessoa mais proxima!!!
            
            # if living_room_ctr == 0:
            #     for p in people_found:
            #         living_room_ctr += 1
            #         host_detected_person_living_room = p
            #         print("dont know where")

            selected_host = DetectedPerson
            if chairs_ctr > 0:
                selected_host = host_detected_person_chair
                is_cropped, filename = self.crop_face(new_person=selected_host, current_frame_image_msg=selected_host.image_rgb_frame)
                print("is_cropped:", is_cropped)
            elif living_room_ctr > 0:
                selected_host = host_detected_person_living_room
                is_cropped, filename = self.crop_face(new_person=selected_host, current_frame_image_msg=selected_host.image_rgb_frame)
                print("is_cropped:", is_cropped)
            

            if is_cropped:
                host_found = True

            # o is_cropped tem que estar dentro destes ifs para só sair do while quanto tiver um cropped face correcto 

        if sfp_ctr >= 2:
            return "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/images/2024-05-01_21-05-27_8i_.jpg", [self.look_sofa[0], self.look_sofa[1]]
        else:  
            return filename, [selected_host.position_absolute.x, selected_host.position_absolute.y]


        print("loc counters:", chairs_ctr, living_room_ctr)
        # verify if there is anyone with cropped face = True
        #   in the sofa or chairs
        #   in the living room 
        # if not assumes the closest person 
        # saves and returns that info
    
    """
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
                
                # if p.room_location == "Living Room" and p.furniture_location == "Sofa":
                #     is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                #     if is_cropped:
                #         host = p
                #         host_found = True
                #         print("SOFA YES")
                #         break

                # if the robot localisation is a bit off and i do not detect anyone in the sofa, i just check for people in the living room
                if p.room_location == "Living Room":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA NO")
                        break
                else:
                    # TIAGO : # usar distancia da pesso, e ver quem é a pessoa mais proxima!!!


                    print("CLOSEST PERSON")
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("OUTSIDE")

        self.set_rgb(WHITE+HALF_ROTATE)

        self.set_neck_coords(position=[host.position_absolute.x, host.position_absolute.y], ang=-10, wait_for_end_of=True)

        return filename, [host.position_absolute.x, host.position_absolute.y]
        """
    """
    def search_for_host_and_guest1(self):
        total_photos = []
        total_coords = []


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
                    self.set_rgb(YELLOW+HALF_ROTATE)
                time.sleep(0.2)
            
            for p in detected_person_temp.persons:
                print(p.room_location, p.furniture_location)

                is_cropped = False
                filename = ""
                
                if p.room_location == "Living Room" and p.furniture_location == "Sofa":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb, filename_with_index=True)
                    # filename += "_"+str(p.index_person)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA YES")

                # if the robot localisation is a bit off and i do not detect anyone in the sofa, i just check for people in the living room
                elif p.room_location == "Living Room":
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb, filename_with_index=True)
                    # filename += "_"+str(p.index_person)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("SOFA NO")
                else:
                    print("CLOSEST PERSON")
                    is_cropped, filename = self.crop_face(p, detected_person_temp.image_rgb, filename_with_index=True)
                    # filename += "_"+str(p.index_person)
                    if is_cropped:
                        host = p
                        host_found = True
                        print("OUTSIDE")

                if is_cropped:
                    total_photos.append(filename)
                    total_coords.append([host.position_absolute.x, host.position_absolute.y])

        if len(total_photos) == 1:
            self.set_rgb(BLUE+HALF_ROTATE)
        elif len(total_photos) == 2:
            self.set_rgb(GREEN+HALF_ROTATE)
        elif len(total_photos) > 2:
            self.set_rgb(WHITE+HALF_ROTATE)

        return total_photos, total_coords
        """

    def search_for_host_and_guest1_v2(self):

        both_people_found = False
        tetas = [[-20, -10], [20, -10]]
        is_cropped = False
        sfp_ctr = 0
        total_photos = []
        total_coords = []


        while not both_people_found and sfp_ctr < 2:

            total_photos.clear()
            total_coords.clear()

            sfp_ctr += 1
            people_found = self.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_legs_visible=True)

            print("FOUND:", len(people_found)) 
            # for p in people_found:
            #     print("ID:", p.index_person)

            self.set_rgb(BLUE+HALF_ROTATE)
            self.set_neck(position=[0, 0], wait_for_end_of=True)
            
            living_room_ctr = 0
            
            for p in people_found:
                print("ID:", p.index_person, p.furniture_location, p.room_location)
                # self.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                # time.sleep(3)

            for p in people_found:

                if p.room_location == "Living Room":
                    living_room_ctr += 1
                    
                    is_cropped, filename = self.crop_face(p, p.image_rgb_frame, filename_with_index=True)
                    if is_cropped:
                        host = p
                        print("is_cropped:", is_cropped)
                        
                    print("in living room")
            
                if is_cropped:
                    total_photos.append(filename)
                    total_coords.append([host.position_absolute.x, host.position_absolute.y])

            if len(total_photos) >= 2:
                both_people_found = True
                
            # if living_room_ctr == 0:
            #     for p in people_found:
            #         living_room_ctr += 1
            #         host_detected_person_living_room = p
            #         print("dont know where")

        if len(total_photos) == 1:
            self.set_rgb(BLUE+HALF_ROTATE)
        elif len(total_photos) == 2:
            self.set_rgb(GREEN+HALF_ROTATE)
        elif len(total_photos) > 2:
            self.set_rgb(WHITE+HALF_ROTATE)

        return total_photos, total_coords
    

    def search_for_guest_and_get_info(self):

        self.search_for_guest()

        self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True, characteristics=True)

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

        self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True, characteristics=False)
    
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

    def get_host_and_guest1_coordinates(self, total_photos, total_coords):
            
            
        total_identifications = []
        total_host_errors = []
        total_guest_errors = []
        for photo in total_photos:
            identification, host_error, guest_error = self.charmie_face_recognition(photo)
            total_identifications.append(identification)
            total_host_errors.append(host_error)
            total_guest_errors.append(guest_error)
        print(total_identifications)

        print("total_coords:", total_coords)
        print("total_host_errors:", total_host_errors)
        print("total_guest_errors:", total_guest_errors)

        # CHECK WHICH PERSON IS THE HOST
        max_host = max(total_host_errors)
        max_host_index = 0
        for index, value in enumerate(total_host_errors):
            if value == max_host:
                max_host_index = index
                break # no need to keep going on, already have the value
        print(max_host_index)
        host_coords = total_coords[max_host_index]

        # Remove the detected host from the person list
        total_coords.pop(max_host_index)
        total_host_errors.pop(max_host_index)
        total_guest_errors.pop(max_host_index)
        
        print("total_coords:", total_coords)
        print("total_host_errors:", total_host_errors)
        print("total_guest_errors:", total_guest_errors)

        # CHECK WHICH PERSON IS THE GUEST1
        max_guest = max(total_guest_errors)
        max_guest_index = 0
        for index, value in enumerate(total_guest_errors):
            if value == max_guest:
                max_guest_index = index
                break # no need to keep going on, already have the value
        print(max_guest_index)
        guest1_coords = total_coords[max_guest_index]

        print(host_coords, guest1_coords)

        return host_coords, guest1_coords
    
    def crop_face(self, new_person, current_frame_image_msg, filename_with_index=False):

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
            
            y1_ = new_person.box_top_left_y
            y2_ = max(new_person.kp_shoulder_right_y, new_person.kp_shoulder_left_y)

            x1_ = min(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)
            x2_ = max(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)

            # this is an attempt to debug: imwrite sometimes sames corrupted files ... 
            y1 = min(y1_, y2_) 
            y2 = max(y1_, y2_)
            x1 = min(x1_, x2_) 
            x2 = max(x1_, x2_)

            # same time for all obejcts
            current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))   
            if filename_with_index: 
                filename = self.complete_path_save_images+current_datetime+str(new_person.index_person)+"i_.jpg"
                # filename = self.complete_path_save_images+current_datetime+".jpg"    
            else:
                filename = self.complete_path_save_images+current_datetime+".jpg"    

            cv2.imwrite(filename, current_frame[y1:y2, x1:x2])
            time.sleep(0.1)

            return True, filename

        else:
            return False, "None"

    def get_characteristics(self, race, age, gender, height, shirt_color, pants_color):
        
        if race != "None": # Minor corrections to value received
            if race == "Middle Eastern" or race == "Hispanic" or race == "Indian":
                race = "Caucasian"
        else: # If the robot can not compute, it guesses the following
            race = "Caucasian"
        
        if age != "None": # Minor corrections to value received
            if age == "Over 60":
                age = "Between 40 and 60"
            elif age == "Under 20":
                age = "Between 18 and 32"
            age = age.replace(' ', '_')
        else: # If the robot can not compute, it guesses the following
            age = 'Between18_32'
        
        if gender != "None": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            gender = "Male"
        
        # print("height is ", height)
        temp_height_string = ""
        if height != 0.0: # Minor corrections to value received
            if height > 1.55: 
                temp_height_string='taller'
            elif height < 1.40:
                temp_height_string='smaller'
            else:
                temp_height_string='equal'
        else: # If the robot can not compute, it guesses the following
            temp_height_string = "taller"
        
        if shirt_color != "None": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            shirt_color = "White"
        
        if pants_color != "None": # Minor corrections to value received
            pass
        else: # If the robot can not compute, it guesses the following
            pants_color = "Blue"
        
        self.set_speech(filename="receptionist/the_first_guest_is", wait_for_end_of=True)
        self.set_speech(filename="receptionist/characteristics/race_"+race.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/characteristics/gender_"+gender.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/characteristics/age_"+age.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/characteristics/height_"+temp_height_string.lower(), wait_for_end_of=True)
        self.set_speech(filename="receptionist/the_shirt_color_is", wait_for_end_of=True)
        self.set_speech(filename="receptionist/characteristics/color_"+shirt_color.lower(), wait_for_end_of=True)

        # Not using the pants color at the moment, since the robot had to look down and 
        # we lose time that we do not actually since the other 5 characteristics are one more than enough 
        # self.set_speech(filename="receptionist/the_pants_color_is", wait_for_end_of=True)
        # self.set_speech(filename="receptionist/characteristics/color_"+pants_color.lower(), wait_for_end_of=True)

    def charmie_face_recognition(self, image):
       
        image = face_recognition.load_image_file(image)
        encoding_entry = face_recognition.face_encodings(image)

        if len(encoding_entry) == 0:
            # return [("Unknown", 0)], None, 0  
            return "Unknown", 0.0, 0.0  

        encoding_entry = encoding_entry[0]  


        encoding_knowns = []
        names = []

        host_guest_paths = [self.host_filename, self.guest1_filename]
        host_guest_names = ["host", "guest1"]


        for i in range(2):
            img = face_recognition.load_image_file(host_guest_paths[i])
            encoding = face_recognition.face_encodings(img)

            if len(encoding) == 0:
                continue 

            encoding = encoding[0]  
            encoding_knowns.append(encoding)
            names.append(host_guest_names[i])
        
        if not encoding_knowns:  
            # return [("Unknown", 0)], None, 0  
            return "Unknown", 0.0, 0.0
        
        all_percentages = []
        for encoding_knowns in encoding_knowns:
            distancia = face_recognition.face_distance([encoding_knowns], encoding_entry)[0]
            confidance = (1 - distancia) * 100
            all_percentages.append(confidance)

        person_recognized, biggest_confidance = max(zip(names, all_percentages), key=lambda x: x[1])

        # if biggest_confidance < 40:
        #     person_recognized = "Unknown"

        if len(all_percentages) == 2:
            return person_recognized, all_percentages[0], all_percentages[1]
        else:
            print("LEN FACE DETECTED ERRADO", len(all_percentages))
            return person_recognized, 0.0, 0.0
    
        """
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
        """


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
