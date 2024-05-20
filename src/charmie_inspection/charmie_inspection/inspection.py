#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, TarNavSDNL
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger

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


class InspectionNode(Node):

    def __init__(self):
        super().__init__("Inspection")
        self.get_logger().info("Initialised CHARMIE Inspection Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Door Start
        self.start_door_subscriber = self.create_subscription(Bool, 'get_door_start', self.get_door_start_callback, 10) 
        self.flag_door_start_publisher = self.create_publisher(Bool, 'flag_door_start', 10) 
        # Yolo Pose
        # self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        

        ### Services (Clients) ###
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")        
        # Navigation
        while not self.nav_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Navigation Trigger Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False

        self.br = CvBridge()
        # self.detected_people = Yolov8Pose()
        self.start_button_state = False
        self.door_start_state = False
        self.flag_navigation_reached = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.navigation_success = True
        self.navigation_message = ""

    # def person_pose_filtered_callback(self, det_people: Yolov8Pose):
    #     self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        # cv2.waitKey(10)

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


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    th_main = threading.Thread(target=ThreadMainInspection, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainInspection(node: InspectionNode):
    main = InspectionMain(node)
    main.main()

class InspectionMain():

    def __init__(self, node: InspectionNode):
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

        self.node.door_start_state = False

        t = Bool()
        t.data = True
        self.node.flag_door_start_publisher.publish(t)

        while not self.node.door_start_state:
            pass

        t.data = False 
        self.node.flag_door_start_publisher.publish(t)

    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message

    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def set_navigation(self, movement="", target=[0.0, 0.0], absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, avoid_people=False, wait_for_end_of=True):


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
            navigation.avoid_people = avoid_people

            self.node.flag_navigation_reached = False

            if navigation.avoid_people:
                self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)

            self.node.target_pos_publisher.publish(navigation)

            if wait_for_end_of:
                while not self.node.flag_navigation_reached:
                    pass
                self.node.flag_navigation_reached = False


            if navigation.avoid_people:
                self.activate_yolo_pose(activate=False)

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

    # main state-machine function
    def main(self):
        
        # Wait for START button
        # Wait for door to open
        # Go to inspection point
        # Go to exit door
        # Final state

        # States in Inspection Task
        self.Waiting_for_start_button = 0
        self.Waiting_for_door_to_open = 1
        self.Go_to_inspection_point = 2
        self.Waiting_for_end_of_inspection = 3
        self.Go_to_exit_door = 4
        self.Final_State = 5
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -20]

        # Navigation Coordinates [x, y]
        """ self.front_of_door = [0.0, 1.5] 
        self.outside_bedroom_door = [-0.7, 4.7]
        self.inside_bedroom_door = [-2.8, 4.5]
        self.inside_bedroom_top_door = [-3.5, 5.0] """

        self.front_of_door = [0.3, 2.5]
        self.door_office = [-2.2, 3.0]
        self.inside_office = [-3.7, 3.0]
        self.final_point = [-4.7, 3.0]
        self.almost_outside_door = [-4.5, 1.5]
        self.outside_house = [-4.5, -1.5]
        
        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_start_button

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Inspection Main...")

        while True:

            if self.state == self.Waiting_for_start_button:
                print("State:", self.state, "- Waiting_for_start_button")
                
                self.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")

                time.sleep(1)

                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                # set rgb's to cyan
                self.set_rgb(CYAN+SET_COLOUR)

                ### FAZER ESTA FRASE
                self.set_speech(filename="inspection/inspection_ready_start", wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=True)

                # wait for start_button
                self.wait_for_start_button()

                # set rgb's to static green
                self.set_rgb(GREEN+SET_COLOUR)

                # next state
                self.state = self.Waiting_for_door_to_open

            elif self.state == self.Waiting_for_door_to_open:
                print("State:", self.state, "- Waiting_for_door_to_open")
                
                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)

                self.wait_for_door_start()

                time.sleep(3.5)

                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                # set rbg's to rotating green
                self.set_rgb(GREEN+ROTATE)

                # next state
                self.state = self.Go_to_inspection_point 

            elif self.state == self.Go_to_inspection_point:
                print("State:", self.state, "- Go_to_inspection_point")

                # temp
                # self.set_initial_position([-0.7, 4.2, 90])

                self.set_speech(filename="inspection/going_to_inspection_point", wait_for_end_of=True)

                """ self.set_navigation(movement="move", target=self.front_of_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.outside_bedroom_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.outside_bedroom_door, flag_not_obs=True, avoid_people=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.inside_bedroom_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_bedroom_door, flag_not_obs=False,  avoid_people=True, wait_for_end_of=True) """

                self.set_navigation(movement="move", target=self.front_of_door, flag_not_obs=True,  avoid_people=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.door_office, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.door_office, flag_not_obs=True, avoid_people=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.inside_office, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.inside_office, flag_not_obs=False,  avoid_people=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.final_point, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.final_point, flag_not_obs=False,  avoid_people=True, wait_for_end_of=True)

                # set rgb's to static green
                self.set_rgb(GREEN+SET_COLOUR)

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_speech(filename="inspection/arrived_inspection_point", wait_for_end_of=True)

                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                # next state
                self.state = self.Waiting_for_end_of_inspection
            
            elif self.state == self.Waiting_for_end_of_inspection:
                print("State:", self.state, "- Waiting_for_end_of_inspection")

                # set rgb's to rotating rainbow  
                self.set_rgb(RAINBOW_ROT)

                self.set_speech(filename="inspection/inspection_introduction", wait_for_end_of=True)

                # set rgb's to red
                self.set_rgb(CYAN+SET_COLOUR)

                # wait for start button
                self.wait_for_start_button()

                self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                # set rgb's to rotating green
                self.set_rgb(GREEN+ROTATE)

                # next state
                self.state = self.Go_to_exit_door 

            elif self.state == self.Go_to_exit_door:
                print("State:", self.state, "- Go_to_exit_door")
                                
                self.set_speech(filename="inspection/moving_exit_door", wait_for_end_of=True)
                
                self.set_navigation(movement="rotate", target=self.almost_outside_door, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.almost_outside_door, flag_not_obs=False,  avoid_people=True, wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.outside_house, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.outside_house, flag_not_obs=False, avoid_people=True, wait_for_end_of=True)

                # next state
                self.state = self.Final_State 

            elif self.state == self.Final_State:
                print("State:", self.state, "- Final_State")


                self.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.set_speech(filename="inspection/arrived_exit_door", wait_for_end_of=True)

                # set rgb's to static cyan  
                self.set_rgb(CYAN+SET_COLOUR)

                while True:
                    pass

            else:
                pass
