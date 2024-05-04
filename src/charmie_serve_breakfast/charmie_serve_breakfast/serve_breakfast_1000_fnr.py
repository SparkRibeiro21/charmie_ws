#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

import cv2 
import threading
import time
from cv_bridge import CvBridge
from pathlib import Path
from datetime import datetime
import math
import numpy as np

class ServeBreakfastNode(Node):

    def __init__(self):
        super().__init__("ServeBreakfast")
        self.get_logger().info("Initialised CHARMIE ServeBreakfast Node")


        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Door Start
        self.start_door_subscriber = self.create_subscription(Bool, 'get_door_start', self.get_door_start_callback, 10) 
        self.flag_door_start_publisher = self.create_publisher(Bool, 'flag_door_start', 10) 
        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)
        # Yolo Pose
        # self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered_hand", self.object_detected_filtered_hand_callback, 10)
        # Arm 
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Audio
        # self.get_audio_client = self.create_client(GetAudio, "audio_command")
        # self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        # self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")


        """
        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
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
        while not self.get_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        while not self.neck_track_person_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        while not self.neck_track_object_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        while not self.activate_yolo_objects_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        """


        # TEMP:
        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
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

        self.br = CvBridge()
        # self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.detected_objects_hand = Yolov8Objects()
        self.start_button_state = False
        self.door_start_state = False
        self.flag_navigation_reached = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        # self.calibrate_audio_success = True
        # self.calibrate_audio_message = ""
        # self.audio_command = ""
        self.face_success = True
        self.face_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        self.track_object_success = True
        self.track_object_message = ""
        # self.activate_yolo_pose_success = True
        # self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.arm_success = True
        self.arm_message = ""
        self.navigation_success = True
        self.navigation_message = ""

        self.get_neck_position = [1.0, 1.0]
        


    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

    def object_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_objects_hand = det_object

    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        self.arm_ready = True
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

    ### DOOR START ###
    def get_door_start_callback(self, state: Bool):
        self.door_start_state = state.data
        # print("Received Start Button:", state.data)

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_objects_confidence=0.5):
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.minimum_objects_confidence = minimum_objects_confidence

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
            self.speech_success = response.success
            self.speech_message = response.message
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

        self.node.door_start_state = False

        t = Bool()
        t.data = True
        self.node.flag_door_start_publisher.publish(t)

        while not self.node.door_start_state:
            pass

        t.data = False 
        self.node.flag_door_start_publisher.publish(t)
    
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
    
    def get_neck(self, wait_for_end_of=True):
    
        self.node.call_get_neck_position_server()
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False


        return self.node.get_neck_position[0], self.node.get_neck_position[1] 
    
    def activate_yolo_objects(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_objects_confidence=0.5, wait_for_end_of=True):
        
        # self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, characteristics=characteristics)
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, minimum_objects_confidence=minimum_objects_confidence)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

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
    
    def set_navigation(self, movement="", target=[0.0, 0.0], absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_time=0.0, adjust_direction=0.0, adjust_min_dist=0.0, wait_for_end_of=True):


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
            # bool follow_me

            navigation.target_coordinates.x = target[0]
            navigation.target_coordinates.y = target[1]
            navigation.move_or_rotate = movement
            navigation.orientation_absolute = absolute_angle
            navigation.flag_not_obs = flag_not_obs
            navigation.reached_radius = reached_radius
            navigation.avoid_people = False
            navigation.adjust_time = adjust_time
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

    def main(self):
        
        # Task Related Variables
        """
        self.Waiting_for_task_start = 0
        self.Approach_kitchen_counter = 1
        self.Detect_all_objects = 2
        self.Picking_up_spoon = 3
        self.Picking_up_milk = 4
        self.Picking_up_cornflakes = 5
        self.Picking_up_bowl = 6
        self.Approach_kitchen_table = 7
        self.Placing_bowl = 8
        self.Placing_cornflakes = 9
        self.Placing_milk = 10
        self.Placing_spoon = 11
        self.Final_State = 12
        """

        self.Waiting_for_task_start = 0

        self.Approach_milk_location = 1
        self.Detect_milk = 2
        self.Picking_up_milk = 3

        self.Approach_cornflakes_location = 4
        self.Detect_cornflakes = 5
        self.Picking_up_cornflakes = 6
        
        self.Approach_dishes_location = 7
        self.Detect_dishes = 8
        self.Picking_up_dishes = 9

        self.Approach_kitchen_table = 10
        self.Placing_bowl = 11
        
        self.Placing_milk = 12
        self.Placing_cornflakes = 13
        self.Placing_spoon = 14
        self.Final_State = 15
        


        # Configurables
        self.wait_time_to_put_objects_in_hand = 0
        self.MULTIPLE_IMAGES_FACE_SAME_TIME = False

        # Custom Faces:
        self.custom_face_filename = "" 

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45] # temp while debugging! Correct value: [-45, -45], Debug Value [45, -45]
        self.look_tray = [0, -60]

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # Navigation Positions
        self.front_of_door = [0.3, 2.5]
        self.almost_kitchen = [1.5, 5.3]
        self.inside_kitchen = [1.5, 6.8]
        # self.cabinet = [-2.0, 7.5]
        self.midway_kitchen_counter = [-0.6, 8.0]
        self.kitchen_counter = [-0.6, 9.5]


        self.door_kitchen_bedroom_inside = [-0.8, 8.2]
        self.door_kitchen_bedroom_outside = [-4.2, 8.2]

        self.door_kitchen_bedroom_outside2 = [-4.0, 7.5]
        self.door_kitchen_bedroom_inside2 = [-1.5, 7.3]

        self.intermedio_dishes_pour = [-0.6, 9.2] 


        self.cofee_table = [0.8, 3.5]


        # Detect Objects Variables
        self.detect_object_total = [DetectedObject(), DetectedObject()]
        self.images_of_detected_object_total = [Image(), Image()]
        self.flag_object_total = [False, False] 

        
        # only for milk
        self.detect_object_total_milk = DetectedObject()
        self.images_of_detected_object_total_milk = Image()
        self.flag_object_total_milk = False 

        
        # only for cornflakes
        self.detect_object_total_cornflakes = DetectedObject()
        self.images_of_detected_object_total_cornflakes = Image()
        self.flag_object_total_cornflakes = False 



        self.state = self.Waiting_for_task_start

        self.node.get_logger().info("IN SERVE THE BREAKFAST MAIN")

        while True:

            if self.state == self.Waiting_for_task_start:

                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")

                time.sleep(1)
        
                self.activate_yolo_objects(activate_objects=False)

                self.set_face("demo5")

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)

                self.wait_for_start_button()
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)

                self.wait_for_door_start()

                self.state = self.Approach_milk_location

            elif self.state == self.Approach_milk_location:

                self.set_speech(filename="serve_breakfast/moving_to_coffee_table", wait_for_end_of=False)

                
                self.set_navigation(movement="move", target=self.front_of_door, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="rotate", target=self.almost_kitchen, flag_not_obs=True, wait_for_end_of=True)
                
                self.set_navigation(movement="move", target=self.cofee_table, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)
                


                self.set_speech(filename="serve_breakfast/arrived_at_coffee_table", wait_for_end_of=True)
                
                self.state = self.Detect_milk

            elif self.state == self.Detect_milk:

               #  self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=False)

                # temp comment
                # self.set_arm(command="search_for_objects", wait_for_end_of=True)

                # self.search_for_serve_breakfast_objects()

                self.search_for_milk()

                # This used to be here, but we lost a lot of time that the arm could be moving at the same time as we speak, so it has been changed to the 
                # detection function, before speaking that the objects have been found
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=True)

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                #self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)

                self.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=False)

                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                
                # self.set_speech(filename="serve_breakfast/will_show_objects_one_by_one", wait_for_end_of=True)
                
                # already shows the detection for the next object while moving the arm from previous action, this way we save time
                
                self.set_face(custom=self.custom_face_filename + "milk")

                # self.set_speech(filename="serve_breakfast/found_the_spoon", wait_for_end_of=True)  
                self.set_speech(filename="serve_breakfast/found_the_milk", wait_for_end_of=True)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  

                time.sleep(2.0)

                self.state = self.Picking_up_milk

            elif self.state == self.Picking_up_milk:

                # post FNR2024: this is here to try to pick up the objects rather than using Deus Ex Machina 

                self.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=True)

                self.set_neck(position=self.look_judge, wait_for_end_of=True)    

                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_face("help_pick_milk") 
                
                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                                        
                
                # self.set_neck(position=self.look_tray, wait_for_end_of=True) # almost bumps into arm and is not necessary
                
                # self.set_face(custom=self.custom_face_filename + "cornflakes")

                # self.set_speech(filename="serve_breakfast/found_the_cornflakes", wait_for_end_of=False)  
                
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)


                self.state = self.Approach_cornflakes_location


            elif self.state == self.Approach_cornflakes_location:



                
                self.set_navigation(movement="move", target=self.almost_kitchen, flag_not_obs=True, wait_for_end_of=True)

                self.set_navigation(movement="rotate", target=self.inside_kitchen, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_kitchen, flag_not_obs=False, wait_for_end_of=True)
                
                # temp só debug
                # self.set_initial_position([self.inside_kitchen[0], self.inside_kitchen[1]-0.5, 0.0])
                # print("SET INITIAL POSITION")

                # time.sleep(3)

                self.set_navigation(movement="rotate", target=self.door_kitchen_bedroom_inside, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.door_kitchen_bedroom_inside, flag_not_obs=True, wait_for_end_of=True)
                
                
                self.set_navigation(movement="rotate", target=self.door_kitchen_bedroom_outside, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.door_kitchen_bedroom_outside, flag_not_obs=False, wait_for_end_of=True)

                self.set_navigation(movement="orientate", absolute_angle= 90.0, flag_not_obs = True, wait_for_end_of=True)
                
                self.set_navigation(movement="adjust_obstacle", flag_not_obs=True, adjust_direction=0.0, adjust_min_dist=0.50, wait_for_end_of=True)

                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=8.0, adjust_direction=-90.0+360, wait_for_end_of=True)

                self.set_navigation(movement="orientate", absolute_angle= 135.0, flag_not_obs = True, wait_for_end_of=True)



                




                """
                self.set_navigation(movement="rotate", target=self.midway_kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.midway_kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                
                self.set_navigation(movement="orientate", absolute_angle= 45.0, flag_not_obs = True, wait_for_end_of=True)
                """





                self.state = self.Detect_cornflakes

            elif self.state == self.Detect_cornflakes:



                self.search_for_cornflakes()

                # This used to be here, but we lost a lot of time that the arm could be moving at the same time as we speak, so it has been changed to the 
                # detection function, before speaking that the objects have been found
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=True)

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                #self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)

                self.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=False)

                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                
                # self.set_speech(filename="serve_breakfast/will_show_objects_one_by_one", wait_for_end_of=True)
                
                # already shows the detection for the next object while moving the arm from previous action, this way we save time
                
                self.set_face(custom=self.custom_face_filename + "cornflakes")

                # self.set_speech(filename="serve_breakfast/found_the_spoon", wait_for_end_of=True)  
                self.set_speech(filename="serve_breakfast/found_the_cornflakes", wait_for_end_of=True)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  

                time.sleep(2.0)

                

                self.state = self.Picking_up_cornflakes
                
            elif self.state == self.Picking_up_cornflakes:


                # post FNR2024: this is here to try to pick up the objects rather than using Deus Ex Machina 

                self.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=True)

                self.set_neck(position=self.look_judge, wait_for_end_of=True)    

                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_face("help_pick_cornflakes") 
                
                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object_cornflakes", wait_for_end_of=True)
                    
                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                                        
                
                # self.set_neck(position=self.look_tray, wait_for_end_of=True) # almost bumps into arm and is not necessary
                
                # self.set_face(custom=self.custom_face_filename + "cornflakes")

                # self.set_speech(filename="serve_breakfast/found_the_cornflakes", wait_for_end_of=False)  
                
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)
                self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)


                self.state = self.Approach_dishes_location


            elif self.state == self.Approach_dishes_location:

                self.set_face("demo5")

                self.set_neck(position=self.look_navigation, wait_for_end_of=False) # almost bumps into arm and is not necessary

                self.set_speech(filename="serve_breakfast/sb_moving_kitchen_counter", wait_for_end_of=False)


                self.set_navigation(movement="rotate", target=self.door_kitchen_bedroom_outside2, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.door_kitchen_bedroom_outside2, flag_not_obs=True, wait_for_end_of=True)

                self.set_navigation(movement="rotate", target=self.door_kitchen_bedroom_inside2, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.door_kitchen_bedroom_inside2, flag_not_obs=False, wait_for_end_of=True)

                self.set_navigation(movement="rotate", target=self.intermedio_dishes_pour, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.intermedio_dishes_pour, flag_not_obs=True, wait_for_end_of=True)

                self.set_navigation(movement="orientate", absolute_angle= -90.0, flag_not_obs = True, wait_for_end_of=True)

                self.set_navigation(movement="adjust_obstacle", flag_not_obs=True, adjust_time=1.0, adjust_direction=0.0, adjust_min_dist=0.50, wait_for_end_of=True)

                self.set_navigation(movement="orientate", absolute_angle= 45.0, flag_not_obs = True, wait_for_end_of=True)

                """
                self.set_navigation(movement="move", target=self.almost_kitchen, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.inside_kitchen, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_kitchen, flag_not_obs=False, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.midway_kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.midway_kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                
                self.set_navigation(movement="orientate", absolute_angle= 45.0, flag_not_obs = True, wait_for_end_of=True)
                """

                self.set_speech(filename="serve_breakfast/sb_arrived_kitchen_counter", wait_for_end_of=True)
                
                self.state = self.Detect_dishes

            elif self.state == self.Detect_dishes:





                # self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=False)

                # self.set_arm(command="search_for_objects", wait_for_end_of=True)

                self.search_for_serve_breakfast_objects()

                # This used to be here, but we lost a lot of time that the arm could be moving at the same time as we speak, so it has been changed to the 
                # detection function, before speaking that the objects have been found
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=True)

                self.set_neck(position=self.look_judge, wait_for_end_of=False)

                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)

                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                
                # self.set_speech(filename="serve_breakfast/will_show_objects_one_by_one", wait_for_end_of=True)
                
                # already shows the detection for the next object while moving the arm from previous action, this way we save time
                

                self.state = self.Picking_up_dishes

            elif self.state == self.Picking_up_dishes:

                self.set_face(custom=self.custom_face_filename + "bowl")

                self.set_speech(filename="serve_breakfast/found_the_bowl", wait_for_end_of=True)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  

                self.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=True)

                time.sleep(2.0)


                # post FNR2024: this is here to try to pick up the objects rather than using Deus Ex Machina 

                
                # self.set_neck(position=self.look_judge, wait_for_end_of=True)    

                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_face("help_pick_bowl") 
                
                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                time.sleep(self.wait_time_to_put_objects_in_hand) # waits for person to put object in hand
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                                        

                self.set_face(custom=self.custom_face_filename + "spoon")
                
                self.set_speech(filename="serve_breakfast/found_the_spoon", wait_for_end_of=False)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                time.sleep(5)

                self.set_face(command="spoon_inside_traycup")


                self.set_speech(filename="serve_breakfast/place_object_in_funilocopo", wait_for_end_of=True)

            
                time.sleep(3)

            

                self.state = self.Approach_kitchen_table

            elif self.state == self.Approach_kitchen_table:

                self.set_face("demo5")

                self.set_speech(filename="serve_breakfast/sb_moving_kitchen_table", wait_for_end_of=False)

                self.set_navigation(movement="orientate", absolute_angle= -50.0, flag_not_obs = True, wait_for_end_of=True)


                """
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=7.0, adjust_direction=135.0, wait_for_end_of=True)

                self.set_navigation(movement="orientate", absolute_angle=-45.0, flag_not_obs=True, wait_for_end_of=True)

                self.set_navigation(movement="adjust_obstacle", flag_not_obs=True, adjust_time=1.0, adjust_direction=-45.0+360, adjust_min_dist=0.60, wait_for_end_of=True)

                self.set_speech(filename="serve_breakfast/sb_arrived_kitchen_table", wait_for_end_of=True)
                """

                # self.set_speech(filename="generic/place_object_table", wait_for_end_of=True)

                # self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=True)

                self.state = self.Placing_bowl

            elif self.state == self.Placing_bowl:

                self.set_neck(position=self.look_table_objects, wait_for_end_of=True)
                # time.sleep(1)

                ##### ARM MOVE TO TABLE

                ##### ARM PLACE OBJECT
                
                self.set_arm(command="place_bowl_table", wait_for_end_of=True)

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                
                self.state = self.Placing_cornflakes 
            
            elif self.state == self.Placing_cornflakes:

                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                # time.sleep(1)
                
                ##### ARM MOVE TRAY

                ##### ARM PICK OBJECT 

                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### ARM MOVE TO TABLE
                
                ##### ARM POUR IN BOWL
                self.set_arm(command="pour_cereals_bowl", wait_for_end_of=True)
                self.set_speech(filename="serve_breakfast/cornflakes_poured", wait_for_end_of=False)
                
                ##### ARM PLACE OBJECT
                self.set_arm(command="place_cereal_table", wait_for_end_of=True)
                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)
                
                self.state = self.Placing_milk
           
            elif self.state == self.Placing_milk:

                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                # time.sleep(1)

                ##### ARM MOVE TRAY

                ##### ARM PICK OBJECT 

               #  self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### ARM MOVE TO TABLE

                ##### ARM POUR IN BOWL
                self.set_arm(command="pour_milk_bowl", wait_for_end_of=True)
                self.set_speech(filename="serve_breakfast/milk_poured", wait_for_end_of=False)

                ##### ARM PLACE OBJECT
                self.set_arm(command="place_milk_table", wait_for_end_of=True)
                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                self.state = self.Placing_spoon

            elif self.state == self.Placing_spoon:

                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                # time.sleep(1)

                ##### ARM MOVE TRAY

                ##### ARM PICK OBJECT 

                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### ARM MOVE TO TABLE

                ##### ARM PLACE OBJECT
                self.set_arm(command="place_spoon_table", wait_for_end_of=True)

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                self.state = self.Final_State 
            
            elif self.state == self.Final_State:
                
                self.set_arm(command="arm_go_rest", wait_for_end_of=True)

                # self.set_neck(position=self.look_judge) 
                
                self.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=False)

                while True:
                    pass

            else:
                pass
            

    def search_for_milk(self):


        # self.detect_object_total_milk = DetectedObject()
        # self.images_of_detected_object_total_milk = Image()
        # self.flag_object_total_milk = False 


        all_objects_detected = False

        # TOTAL_OBJ = 4
        # list_sb_objects=[
        #     "spoon",
        #     "milk",
        #     "cornflakes",
        #     "bowl"
        # ]

        while not all_objects_detected:

            # FIRST TYPE OF SEARCH: JUST THE NECK WITH SMALL ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:

                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_just_milk(delta_t=2.0, with_hand=False)    
                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=1.0, with_hand=False)    

                if finished_detection:
                    break

                if self.flag_object_total_milk:
                    print("FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if self.flag_object_total_milk:
                self.create_image_just_milk() 
                # self.create_image_four_sb_objects_separately() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break

            print("SEARCH TYPE 2")
            """
            # SECOND TYPE OF SEARCH: ARM WITH SMALL ADJUSTEMENTS AND NECK WITH BIGGER ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [15,10], [-15,10], [-15,-10], [15,-10]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:
                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_just_milk(delta_t=2.0, with_hand=False)    
                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=5.0, with_hand=True)    

                if finished_detection:
                    break

                if self.flag_object_total_milk:
                    print("FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if self.flag_object_total_milk:
                self.create_image_just_milk() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break
            """
            
            self.set_neck(position=self.look_judge, wait_for_end_of=False)
            # if i can not detect both times, i will ask the judge to move and rotate the objects I could not detect
            self.set_speech(filename="generic/problem_detecting_change_object", wait_for_end_of=True) 
            self.set_speech(filename="objects_names/milk", wait_for_end_of=False)  


    def search_for_cornflakes(self):


        # self.detect_object_total_cornflakes = DetectedObject()
        # self.images_of_detected_object_total_cornflakes = Image()
        # self.flag_object_total_cornflakes = False 


        all_objects_detected = False

        # TOTAL_OBJ = 4
        # list_sb_objects=[
        #     "spoon",
        #     "milk",
        #     "cornflakes",
        #     "bowl"
        # ]
        self.look_table_special_cornflakes = [-45, -20]


        while not all_objects_detected:

            # FIRST TYPE OF SEARCH: JUST THE NECK WITH SMALL ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:

                print(pos)
                new_neck_pos = [self.look_table_special_cornflakes[0] + pos[0], self.look_table_special_cornflakes[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_just_cornflakes(delta_t=2.0, with_hand=False)    
                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=1.0, with_hand=False)    

                if finished_detection:
                    break

                if self.flag_object_total_cornflakes:
                    print("FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if self.flag_object_total_cornflakes:
                self.create_image_just_cornflakes() 
                # self.create_image_four_sb_objects_separately() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break

            print("SEARCH TYPE 2")
            """"
            # SECOND TYPE OF SEARCH: ARM WITH SMALL ADJUSTEMENTS AND NECK WITH BIGGER ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [15,10], [-15,10], [-15,-10], [15,-10]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:
                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_just_cornflakes(delta_t=2.0, with_hand=False)    
                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=5.0, with_hand=True)    

                if finished_detection:
                    break

                if self.flag_object_total_cornflakes:
                    print("FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if self.flag_object_total_cornflakes:
                self.create_image_just_cornflakes() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break

            """
            self.set_neck(position=self.look_judge, wait_for_end_of=False)
            # if i can not detect both times, i will ask the judge to move and rotate the objects I could not detect
            self.set_speech(filename="generic/problem_detecting_change_object", wait_for_end_of=True) 
            self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=False)  


    def search_for_serve_breakfast_objects(self):

        all_objects_detected = False

        TOTAL_OBJ = 2
        list_sb_objects=[
            "spoon",
            "bowl"
        ]

        while not all_objects_detected:

            # FIRST TYPE OF SEARCH: JUST THE NECK WITH SMALL ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [10,8], [-10,8], [-10,-5], [10,-5]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:

                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_four_serve_breakfast_objects(delta_t=2.0, with_hand=False)    
                # finished_detection = self.detect_four_serve_breakfast_objects(delta_t=1.0, with_hand=False)    

                if finished_detection:
                    break

                if all(self.flag_object_total):
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #      self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if all(self.flag_object_total):
                self.create_image_four_sb_objects_separately() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break

            print("SEARCH TYPE 2")
            """
            # SECOND TYPE OF SEARCH: ARM WITH SMALL ADJUSTEMENTS AND NECK WITH BIGGER ADJUSTEMENTS
            list_of_neck_position_search = [[0, 0], [15,10], [-15,10], [-15,-10], [15,-10]]

            self.activate_yolo_objects(activate_objects=True)
            finished_detection = False
            for pos in list_of_neck_position_search:
                print(pos)
                new_neck_pos = [self.look_table_objects[0] + pos[0], self.look_table_objects[1] + pos[1]]
                self.set_neck(position=new_neck_pos, wait_for_end_of=True)
                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                # time.sleep(1)

                finished_detection = self.detect_four_serve_breakfast_objects(delta_t=2.0, with_hand=True)    

                if finished_detection:
                    break

                if all(self.flag_object_total):
                    break

            # if finished_detection:
            #     self.set_neck(position=self.look_judge, wait_for_end_of=False)
            #     self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
            #     self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
            #     self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/spoon", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/milk", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/cornflakes", wait_for_end_of=True)  
            #     self.set_speech(filename="objects_names/bowl", wait_for_end_of=True)  
            #     all_objects_detected = True 

            if all(self.flag_object_total):
                self.create_image_four_sb_objects_separately() 
                # self.set_neck(position=self.look_judge, wait_for_end_of=False)
                # self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                # self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)  
                # self.create_image_four_sb_objects_separately() 
                all_objects_detected = True

            if all_objects_detected:
                self.activate_yolo_objects(activate_objects=False)
                break
            """
            self.set_neck(position=self.look_judge, wait_for_end_of=False)
            # if i can not detect both times, i will ask the judge to move and rotate the objects I could not detect
            self.set_speech(filename="generic/problem_detecting_change_object", wait_for_end_of=True) 
            for obj in range(TOTAL_OBJ):
                if not self.flag_object_total[obj]:
                    self.set_speech(filename="objects_names/"+list_sb_objects[obj], wait_for_end_of=False)  

    def detect_four_serve_breakfast_objects(self, delta_t, with_hand=False):

        actual_object = [
            "spoon", 
            "bowl"
            ]
        
        actual_object_with_spaces = [
            "SPOON     ", 
            "BOWL      "
            ]
        TOTAL_OBJ = 2

        detect_as = [
            ["Spoon"], # detect as 'spoon'
            ["Bowl"] # detect as 'bowl'
        ]

        # detect_as = [
        #     ["Spoon", "Fork", "Knife"], # detect as 'spoon'
        #     ["Bowl", "Plate", "Cup"] # detect as 'bowl'
        # ]

        detect_object = [DetectedObject(), DetectedObject()]
        flag_object = [False, False] 
        
        start_time = time.time()
        while (time.time() - start_time) < delta_t:        
            local_detected_objects = self.node.detected_objects
            for object in local_detected_objects.objects:
                for obj in range(TOTAL_OBJ):
                    if object.object_name in detect_as[obj]:
                        
                        if self.MULTIPLE_IMAGES_FACE_SAME_TIME: # JOHANNES said that this is not the correct deus ex machina ask for help to help with handing over the objects
                            if object.confidence > detect_object[obj].confidence:
                                # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                                detect_object[obj] = object
                                detect_object[obj].object_name = actual_object[obj]
                                flag_object[obj] = True
                            
                        if object.confidence > self.detect_object_total[obj].confidence:
                            self.detect_object_total[obj] = object
                            self.detect_object_total[obj].object_name = actual_object[obj]
                            self.flag_object_total[obj] = True
                            self.images_of_detected_object_total[obj] = local_detected_objects.image_rgb

            local_detected_objects_hand = self.node.detected_objects_hand
            for object in local_detected_objects_hand.objects:
                for obj in range(TOTAL_OBJ):
                    if object.object_name in detect_as[obj]:
                        # print(object.object_name, "-", object.confidence, "-", object.index)

                        # The hand objects can not be considered for the show the four objects in the same image case since the images are not the same
                        # if object.confidence > detect_object[obj].confidence:
                        #     # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                        #     detect_object[obj] = object
                        #     flag_object[obj] = True
                        
                        if object.confidence > self.detect_object_total[obj].confidence:
                            self.detect_object_total[obj] = object
                            self.detect_object_total[obj].object_name = actual_object[obj]
                            self.flag_object_total[obj] = True
                            self.images_of_detected_object_total[obj] = local_detected_objects_hand.image_rgb

        # for obj in range(TOTAL_OBJ):
        #     print(actual_object_with_spaces[obj], "|", detect_object[obj].object_name, "-", detect_object[obj].confidence, "-", detect_object[obj].index, "-", flag_object[obj] )

        for obj in range(TOTAL_OBJ):
            print(actual_object_with_spaces[obj], "|", self.detect_object_total[obj].object_name, "-", self.detect_object_total[obj].confidence, "-", self.detect_object_total[obj].index, "-", self.flag_object_total[obj] )

        # print("FINAL:", all(flag_object))

        if all(flag_object):
            self.create_image_four_sb_objects_same_time(local_detected_objects.image_rgb, detect_object) # sends the last image analysed 
            return True
        else:
            return False

    def detect_just_milk(self, delta_t, with_hand=False):

        actual_object = "milk"
        actual_object_with_spaces = "MILK      "
        # TOTAL_OBJ = 1

        # self.detect_object_total_milk = DetectedObject()
        # self.images_of_detected_object_total_milk = Image()
        # self.flag_object_total_milk = False 

        detect_as = ["Milk", "Cleanser"] # detect as 'milk'

        detect_object = DetectedObject()
        flag_object = False 

        # print("WHAT?")
        
        start_time = time.time()
        while (time.time() - start_time) < delta_t:        
            local_detected_objects = self.node.detected_objects
            for object in local_detected_objects.objects:
                # for obj in range(TOTAL_OBJ):

                print(object.object_name)

                if object.object_name in detect_as:
                    print("Decteting milk")
                    
                    if self.MULTIPLE_IMAGES_FACE_SAME_TIME: # JOHANNES said that this is not the correct deus ex machina ask for help to help with handing over the objects
                        if object.confidence > detect_object.confidence:
                            # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                            detect_object = object
                            detect_object.object_name = actual_object
                            flag_object = True
                        
                    if object.confidence > self.detect_object_total_milk.confidence:
                        print("INSIDEEEEEEEEE")
                        self.detect_object_total_milk = object
                        self.detect_object_total_milk.object_name = actual_object
                        self.flag_object_total_milk = True
                        self.images_of_detected_object_total_milk = local_detected_objects.image_rgb
                        # flag_object_total_milk
            
            local_detected_objects_hand = self.node.detected_objects_hand
            for object in local_detected_objects_hand.objects:
                # for obj in range(TOTAL_OBJ):
                if object.object_name in detect_as:
                    # print(object.object_name, "-", object.confidence, "-", object.index)

                    # The hand objects can not be considered for the show the four objects in the same image case since the images are not the same
                    # if object.confidence > detect_object[obj].confidence:
                    #     # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                    #     detect_object[obj] = object
                    #     flag_object[obj] = True
                    
                    if object.confidence > self.detect_object_total_milk.confidence:
                        self.detect_object_total_milk = object
                        self.detect_object_total_milk.object_name = actual_object
                        self.flag_object_total_milk = True
                        self.images_of_detected_object_total_milk = local_detected_objects_hand.image_rgb
        
        # for obj in range(TOTAL_OBJ):
        #     print(actual_object_with_spaces[obj], "|", detect_object[obj].object_name, "-", detect_object[obj].confidence, "-", detect_object[obj].index, "-", flag_object[obj] )

        # for obj in range(TOTAL_OBJ):
        print(actual_object_with_spaces, "|", self.detect_object_total_milk.object_name, "-", self.detect_object_total_milk.confidence, "-", self.detect_object_total_milk.index, "-", self.flag_object_total_milk)

        # print("FINAL:", all(flag_object))

        if flag_object:
            # self.create_image_four_sb_objects_same_time(local_detected_objects.image_rgb, detect_object) # sends the last image analysed 
            # self.create_image_just_one_object(local_detected_objects.image_rgb, detect_object)
            return True
        else:
            return False
        


    def detect_just_cornflakes(self, delta_t, with_hand=False):

        actual_object = "cornflakes"
        actual_object_with_spaces = "CORNFLAKES "
        # TOTAL_OBJ = 1

        # self.detect_object_total_cornflakes = DetectedObject()
        # self.images_of_detected_object_total_cornflakes = Image()
        # self.flag_object_total_cornflakes = False 

        detect_as = ["Cornflakes", "Strawberry_jello", "Chocolate_jello"] # detect as 'cornflakes'

        detect_object = DetectedObject()
        flag_object = False 

        # print("WHAT?")
        
        start_time = time.time()
        while (time.time() - start_time) < delta_t:        
            local_detected_objects = self.node.detected_objects
            for object in local_detected_objects.objects:
                # for obj in range(TOTAL_OBJ):

                print(object.object_name)

                if object.object_name in detect_as:
                    print("Decteting cornflakes")
                    
                    if self.MULTIPLE_IMAGES_FACE_SAME_TIME: # JOHANNES said that this is not the correct deus ex machina ask for help to help with handing over the objects
                        if object.confidence > detect_object.confidence:
                            # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                            detect_object = object
                            detect_object.object_name = actual_object
                            flag_object = True
                        
                    if object.confidence > self.detect_object_total_cornflakes.confidence:
                        print("INSIDEEEEEEEEE")
                        self.detect_object_total_cornflakes = object
                        self.detect_object_total_cornflakes.object_name = actual_object
                        self.flag_object_total_cornflakes = True
                        self.images_of_detected_object_total_cornflakes = local_detected_objects.image_rgb
                        # flag_object_total_cornflakes
            
            local_detected_objects_hand = self.node.detected_objects_hand
            for object in local_detected_objects_hand.objects:
                # for obj in range(TOTAL_OBJ):
                if object.object_name in detect_as:
                    # print(object.object_name, "-", object.confidence, "-", object.index)

                    # The hand objects can not be considered for the show the four objects in the same image case since the images are not the same
                    # if object.confidence > detect_object[obj].confidence:
                    #     # print(" - ", object.object_name, "-", object.confidence, "-", object.index)
                    #     detect_object[obj] = object
                    #     flag_object[obj] = True
                    
                    if object.confidence > self.detect_object_total_cornflakes.confidence:
                        self.detect_object_total_cornflakes = object
                        self.detect_object_total_cornflakes.object_name = actual_object
                        self.flag_object_total_cornflakes = True
                        self.images_of_detected_object_total_cornflakes = local_detected_objects_hand.image_rgb
        
        # for obj in range(TOTAL_OBJ):
        #     print(actual_object_with_spaces[obj], "|", detect_object[obj].object_name, "-", detect_object[obj].confidence, "-", detect_object[obj].index, "-", flag_object[obj] )

        # for obj in range(TOTAL_OBJ):
        print(actual_object_with_spaces, "|", self.detect_object_total_cornflakes.object_name, "-", self.detect_object_total_cornflakes.confidence, "-", self.detect_object_total_cornflakes.index, "-", self.flag_object_total_cornflakes)

        # print("FINAL:", all(flag_object))

        if flag_object:
            # self.create_image_four_sb_objects_same_time(local_detected_objects.image_rgb, detect_object) # sends the last image analysed 
            # self.create_image_just_one_object(local_detected_objects.image_rgb, detect_object)
            return True
        else:
            return False
        


    def create_image_four_sb_objects_same_time(self, image, serve_breakfast_objects):

        current_frame = self.node.br.imgmsg_to_cv2(image, "bgr8")
        current_frame_draw = current_frame.copy()

        thresh_h = 80
        thresh_v = 220

        x_min = 1280
        x_max = 0
        y_min = 720
        y_max = 0

        for object in serve_breakfast_objects:      
        
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
            cv2.rectangle(current_frame_draw, start_point, end_point, (0,255,0) , 4) 
            # cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
        
        for object in serve_breakfast_objects:      
                        
            if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
            else:
                start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)

            text_size, _ = cv2.getTextSize(f"{object.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            text_w, text_h = text_size
            cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (0,255,0), -1)
            cv2.putText(current_frame_draw, f"{object.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
        
        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
        cv2.imwrite(self.node.complete_path_custom_face + current_datetime + ".jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        time.sleep(0.5)
        self.set_face(custom=current_datetime)


    def create_image_four_sb_objects_separately(self):
        
        TOTAL_OBJ = 2
        list_sb_objects=[
            "spoon",
            "bowl"
        ]

        # same time for all obejcts
        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        self.custom_face_filename = current_datetime

        for i in range(TOTAL_OBJ):
            current_frame = self.node.br.imgmsg_to_cv2(self.images_of_detected_object_total[i], "bgr8")
            current_frame_draw = current_frame.copy()

            thresh_h = 220
            thresh_v = 220

            x_min = 1280
            x_max = 0
            y_min = 720
            y_max = 0

            object = self.detect_object_total[i]

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
            cv2.rectangle(current_frame_draw, start_point, end_point, (255,0,0) , 4) 
            # cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
            
            if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
            else:
                start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
                
            text_size, _ = cv2.getTextSize(f"{object.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            text_w, text_h = text_size
            cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,0,0), -1)
            cv2.putText(current_frame_draw, f"{object.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
        
            cv2.imwrite(self.node.complete_path_custom_face + current_datetime + list_sb_objects[i] + ".jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
            
            # time.sleep(0.5)
            # self.set_face(custom=current_datetime)
            # self.set_speech(filename="objects_names/"+list_sb_objects[i], wait_for_end_of=False)  
            # time.sleep(3)
        

    def create_image_just_milk(self):


        # self.detect_object_total_milk = DetectedObject()
        # self.images_of_detected_object_total_milk = Image()
        # self.flag_object_total_milk = False 
        
        # TOTAL_OBJ = 4
        list_sb_objects = "milk"

        # same time for all obejcts
        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        self.custom_face_filename = current_datetime

        # for i in range(TOTAL_OBJ):
        current_frame = self.node.br.imgmsg_to_cv2(self.images_of_detected_object_total_milk, "bgr8")
        current_frame_draw = current_frame.copy()

        thresh_h = 220
        thresh_v = 220

        x_min = 1280
        x_max = 0
        y_min = 720
        y_max = 0

        object = self.detect_object_total_milk

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
        # cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
        
        if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
        else:
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
            
        text_size, _ = cv2.getTextSize(f"{object.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        text_w, text_h = text_size
        cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
        cv2.putText(current_frame_draw, f"{object.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    
        cv2.imwrite(self.node.complete_path_custom_face + current_datetime + "milk" + ".jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        
        # time.sleep(0.5)
        # self.set_face(custom=current_datetime)
        # self.set_speech(filename="objects_names/"+list_sb_objects[i], wait_for_end_of=False)  
        # time.sleep(3)
    

    def create_image_just_cornflakes(self):


        # self.detect_object_total_cornflakes = DetectedObject()
        # self.images_of_detected_object_total_cornflakes = Image()
        # self.flag_object_total_cornflakes = False 
        
        # TOTAL_OBJ = 4
        list_sb_objects = "cornflakes"

        # same time for all obejcts
        current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S_"))    
        self.custom_face_filename = current_datetime

        # for i in range(TOTAL_OBJ):
        current_frame = self.node.br.imgmsg_to_cv2(self.images_of_detected_object_total_cornflakes, "bgr8")
        current_frame_draw = current_frame.copy()

        thresh_h = 220
        thresh_v = 220

        x_min = 1280
        x_max = 0
        y_min = 720
        y_max = 0

        object = self.detect_object_total_cornflakes

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
        # cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
        
        if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
        else:
            start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
            
        text_size, _ = cv2.getTextSize(f"{object.object_name}", cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        text_w, text_h = text_size
        cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
        cv2.putText(current_frame_draw, f"{object.object_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    
        cv2.imwrite(self.node.complete_path_custom_face + current_datetime + "cornflakes" + ".jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        
        # time.sleep(0.5)
        # self.set_face(custom=current_datetime)
        # self.set_speech(filename="objects_names/"+list_sb_objects[i], wait_for_end_of=False)  
        # time.sleep(3)
    