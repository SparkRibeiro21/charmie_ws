#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger

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
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
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
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
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
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # Arm (CHARMIE)
        """ while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...")
         """
        # Variables 
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_arm = False

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.start_button_state = False

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
            self.speech_sucecss = response.success
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

    def activate_yolo_objects(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_objects_confidence=0.5, wait_for_end_of=True):
        
        # self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, characteristics=characteristics)
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, minimum_objects_confidence=minimum_objects_confidence)

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

    def set_navigation(self, movement="", target=[0.0, 0.0], absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, wait_for_end_of=True):


        if movement.lower() != "move" and movement.lower() != "rotate" and movement.lower() != "orientate":
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

        time.sleep(0.5)
        detected_person_temp = self.node.detected_people  


        bag_side = ""
        self.set_rgb(BLUE+HALF_ROTATE)

        person_detected = False
        while not person_detected:
            
            detected_person_temp = self.node.detected_people  

            for p in detected_person_temp.persons:
                if p.room_location == "Corridor" and p.pointing_at != "None":
                    person_detected = True
                    bag_side = p.pointing_at.lower()
                    self.set_rgb(GREEN+HALF_ROTATE)

                print(p.room_location, p.pointing_at, person_detected)
            
            time.sleep(0.05)

        self.activate_yolo_pose(activate=False)

        return bag_side
        
        
    # main state-machine function
    def main(self):
        
        # States in CarryMyLuggage Task
        self.Waiting_for_task_to_start = 0
        self.Recognize_bag = 1
        self.Go_to_bag = 2
        self.Pick_bag = 3
        self.Final_State = 4
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]

        self.initial_position = [-3.5, 1.5, -90.0]

        self.IMU_ANGLE = 0.0
        self.INITIAL_REACHED_RADIUS = 1.0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_to_start

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
                    
                    time.sleep(3)
                
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
                self.set_navigation(movement="move", target = [relative_position_of_bag.x, relative_position_of_bag.y], flag_not_obs=True, reached_radius=self.INITIAL_REACHED_RADIUS, wait_for_end_of=True)
                print('received bag: ', received_bag)
                if received_bag == 'left':
                    self.IMU_ANGLE = -20.0
                    print('left')
                elif received_bag == 'right':
                    self.IMU_ANGLE = 20.0
                    print('\n \n \n \n right \n \n \n')
                else:
                    self.IMU_ANGLE = 0.0

                self.set_navigation(movement="orientate", absolute_angle= self.IMU_ANGLE, flag_not_obs=True, wait_for_end_of=True)

                # look at bag (set neck with the bag's coordinates)

                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                # navigation (how?)

                # next state
                self.state = self.Pick_bag
            
            elif self.state == self.Pick_bag:
                print("State:", self.state, "- Pick_bag")
                # your code here ...  

                # set rgb's to purple
                self.set_rgb(MAGENTA+ROTATE)

                # speech: "I'm picking up the bag now."
                self.set_speech(filename="carry_my_luggage/picking_up_bag", wait_for_end_of=True)


                # move arm to bag's position (how?)
                self.set_arm(command="carry_my_luggage_pre_check_bag", wait_for_end_of=True)

                list_of_rotations = [0.0, 20.0, -20.0]

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
                            
                                    self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                                    
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
                                            self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
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
                            self.INITIAL_REACHED_RADIUS -= 0.3
                            print('Radius ',self.INITIAL_REACHED_RADIUS)
                            i = 0
                            print('vou rodar para saco e andar para ele')
                
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
