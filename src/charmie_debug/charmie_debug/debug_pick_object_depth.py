#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Bool, Float32, Int16, String 
from geometry_msgs.msg import Point
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, ListOfPoints, NeckPosition, BoundingBox, BoundingBoxAndPoints
from charmie_interfaces.srv import TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, SetFace, GetPointCloud
from sensor_msgs.msg import Image

import cv2 
import threading
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

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        # path to save detected people in search for person
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        # Intel Realsense Subscribers
        # self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_head_image_callback, 10)
        # Hand
        # self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_hand_image_callback, 10)
        
        ### Topics (Publisher and Subscribers) ###  
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)

        # Search for Person debug publisher
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")

        ### Services (Clients) ###
        
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")

        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # while not self.neck_track_person_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Neck Track Person ...")
        # while not self.face_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Face Command...")
        # Point Cloud
        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        
        # Variables
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_face = False
        self.waiting_for_pcloud = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.rgb_success = True
        self.rgb_message = ""
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

        self.br = CvBridge()
        self.depth_head_img = Image()
        self.depth_hand_img = Image()
        self.first_depth_head_image_received = False
        self.first_depth_hand_image_received = False
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.point_cloud_response = GetPointCloud.Response()


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
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)


    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object


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

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: TestNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: TestNode):
        self.node = node
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message

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
    

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7
        
        # VARS ...
        self.state = Waiting_for_start_button

        self.floor_dist=600
        self.top_bag_dist=350

        print("IN NEW MAIN")

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:
                # print('State 0 = Initial')

                ### SEARCH FOR PERSON EXAMPLE ###
                
                # self.set_face(command="please_say_restaurant")
                # self.set_face(command="please_say_yes_or_no")
                # self.set_face(command="please_say_receptionist")
                # self.set_neck(position=[0.0, -20.0], wait_for_end_of=True)

                # while True:
                #     pass

                # time.sleep(2.0)

                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...

                bag_coords = self.get_bag_pick_cordinates() #half_image_zero_or_near_percentage=0.4, full_image_near_percentage=0.1, near_max_dist=800)
                print(bag_coords)
                
                # next state
                # self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                print("Finished task!!!")

                while True:
                    pass

            else:
                pass

    def get_bag_pick_cordinates(self):

        DEBUG = True
        f_coords = []

        self.node.first_depth_hand_image_received = False

        while not self.node.first_depth_hand_image_received:
            pass

        current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_hand_img, desired_encoding="passthrough")
        height, width = current_frame_depth_head.shape
        
        current_frame_depth_head[int(0.80*height):height,int(0.29*width):int(0.71*width)] = 0 # remove the robot from the image

        mask_zero = (current_frame_depth_head == 0)
        mask_near = (current_frame_depth_head != 0) & (current_frame_depth_head >= self.top_bag_dist) & (current_frame_depth_head <= self.floor_dist)

        blank_image_bw = np.zeros((height,width), np.uint8)
        blank_image_bw2 = np.zeros((height,width), np.uint8)
        blank_image_bw[mask_near] = [255]

        contours, hierarchy = cv2.findContours(blank_image_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        c_areas = []
        for a in contours: # create list with area size 
            # print(cv2.contourArea(a))
            c_areas.append(cv2.contourArea(a))

        cnt = contours[c_areas.index(max(c_areas))] # extracts the largest area 
        # print(c_areas.index(max(c_areas)))
        
        M = cv2.moments(cnt) # calculates centroide
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        xi,yi,w,h = cv2.boundingRect(cnt)

        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
        theta = -math.atan2(vy,vx)

        bb_thresh = 40 
        bb = BoundingBox() # centroide of bag bounding box 
        bb.box_top_left_x = max(cx-bb_thresh, 0)
        bb.box_top_left_y = max(cy-bb_thresh, 0)
        bb.box_width = min(2*bb_thresh, width)
        bb.box_height = min(2*bb_thresh, height)

        coords_centroide = self.get_point_cloud(bb=bb)

        bb = BoundingBox() # full bag bounding box
        bb.box_top_left_x = max(xi, 0)
        bb.box_top_left_y = max(yi, 0)
        bb.box_width = w
        bb.box_height = h

        coords_full = self.get_point_cloud(bb=bb)
        
        # x = bag height
        # y = move front and back robot, or left and right for camera
        # y = move right and left robot, or up and down for camera
        print("xc = ", round(coords_centroide.center_coords.x,0), "yc = ", round(coords_centroide.center_coords.y,0), "zc = ", round(coords_centroide.center_coords.z,0))
        print("xf = ", round(coords_full.center_coords.x,0), "yf = ", round(coords_full.center_coords.y,0), "zf = ", round(coords_full.center_coords.z,0))

        f_coords.append(coords_centroide.center_coords.x)
        f_coords.append(coords_centroide.center_coords.y)
        f_coords.append(coords_centroide.center_coords.z)
        f_coords.append(theta)
        
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

            print("bag theta =", round(math.degrees(theta), 2), round(theta,2))
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

            print(self.floor_dist, self.top_bag_dist)

        return f_coords