#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Bool, Float32, Int16, String 
from geometry_msgs.msg import Point
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, ListOfPoints, NeckPosition
from charmie_interfaces.srv import TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, SetNeckPosition, GetNeckPosition, SetNeckCoordinates
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
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)

        ### Topics (Publisher and Subscribers) ###  
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered_hand", self.object_detected_filtered_hand_callback, 10)

        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)
        
        # Search for Person debug publisher
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        # Arm 
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_value_publisher = self.create_publisher(Float32, 'arm_value', 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)


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

        # self.objects_filtered_hand_publisher = self.create_subscription(Yolov8Objects, 'objects_detected_filtered_hand', self.hand_objects_callback, 10)
        
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
        
        # Variables
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_arm = False

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
        self.arm_success = True
        self.arm_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""

        self.br = CvBridge()
        self.depth_img = Image()
        self.first_depth_image_received = False
        self.detected_people = Yolov8Pose()
        self.detected_objects_hand = Yolov8Objects()
        self.detected_objects = Yolov8Objects()


    def get_aligned_depth_image_callback(self, img: Image):
        self.depth_img = img
        self.first_depth_image_received = True
        # print("Received Depth Image")


    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)

    def object_detected_filtered_hand_callback(self, det_object_hand: Yolov8Objects):
        self.detected_objects_hand = det_object_hand


    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

        """
        current_frame = self.br.imgmsg_to_cv2(self.detected_objects.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()


        # img = [0:720, 0:1280]
        corr_image = False
        thresh_h = 50
        thresh_v = 200

        if self.detected_objects.num_objects > 0:

            x_min = 1280
            x_max = 0
            y_min = 720
            y_max = 0

            for object in self.detected_objects.objects:      
            
                if object.object_class == "Dishes":
                    corr_image = True

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

                    cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
                    
            
            for object in self.detected_objects.objects:      
                
                if object.object_class == "Dishes":
                
                    if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
                    else:
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
                        
                    # just to test for the "serve the breakfast" task...
                    aux_name = object.object_name
                    if object.object_name == "Fork" or object.object_name == "Knife":
                        aux_name = "Spoon"
                    elif object.object_name == "Plate" or object.object_name == "Cup":
                        aux_name = "Bowl"

                    text_size, _ = cv2.getTextSize(f"{aux_name}", cv2.FONT_HERSHEY_DUPLEX, 1, 1)
                    text_w, text_h = text_size
                    cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
                    cv2.putText(current_frame_draw, f"{aux_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        if corr_image:
            # current_frame_draw = current_frame_draw[x_min:y_min, x_max,y_max]
            # img = current_frame_draw[y_min:y_max, x_min,x_max]
            cv2.imshow("c", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)])
            cv2.waitKey(1)
        # cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
        # cv2.waitKey(10)

        # cv2.imwrite("object_detected_test4.jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        # cv2.waitKey(10)
        """


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
        
        # VARS ...
        self.state = 0
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message

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

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

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

                # self.detected_people = det_people
                # if self.detected_people.num_person > 0:


                overall = self.check_door_depth() #half_image_zero_or_near_percentage=0.4, full_image_near_percentage=0.1, near_max_dist=800)
                      
                

                """ if overall: 
                    # print("STOP", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("STOP") 
                else:
                    # print("GO", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("GO") """
                        
                
                
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

    def check_door_depth(self, half_image_zero_or_near_percentage=0.3, full_image_near_percentage=0.1, near_max_dist = 400, near_max_dist_1=350, near_max_dist_2 = 900):
        overall = False
        DEBUG = True

        if self.node.first_depth_image_received:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            height, width = current_frame_depth_hand.shape
            center_height = height / 2
            center_width = width / 2

            

            # print(height, width)
            # print(current_frame_depth_hand)

            tot_pixeis = height*width 
            blank_image = self.node.detected_objects.image_rgb
            """  mask_zero = (current_frame_depth_hand == 0)
            mask_near = (current_frame_depth_hand > 0) & (current_frame_depth_hand <= near_max_dist_1)
            mask_remaining = (current_frame_depth_hand > near_max_dist_1)
            blank_image = np.zeros((height,width,3), np.uint8)
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            full_image_near_err = 0.0
            
            full_image_near_err = pixel_count_near/tot_pixeis 
            # print('Percentage of image under ', near_max_dist_1, ' = ',full_image_near_err*100 - 0.7) """

            if hasattr(self.node.detected_objects_hand, 'image_rgb'):
                if hasattr(self.node.detected_objects_hand, 'objects'): 
                    if self.node.detected_objects_hand.objects:
                        object_detected_hand = self.node.detected_objects_hand.objects[0]
                        current_frame_rgb_hand = self.node.br.imgmsg_to_cv2(self.node.detected_objects_hand.image_rgb, desired_encoding="passthrough")
                    
                        print(self.node.detected_objects_hand.objects)

                        error_x_bbox = width / 2 - object_detected_hand.box_center_x
                        error_y_bbox = height / 2 - object_detected_hand.box_center_y

                        print(error_x_bbox)
                        print(error_y_bbox)
                        cv2.circle(current_frame_rgb_hand, (object_detected_hand.box_center_x, object_detected_hand.box_center_y), 100, [255, 0, 0], 10)
                        print(center_width, center_height)
                        cv2.circle(current_frame_rgb_hand, (int(center_width), int(center_height)), 20, [0, 0, 255], 5)
                        
                        cv2.imshow("New Img Distance Inspection", current_frame_rgb_hand)
                        cv2.waitKey(10)

                        """ self.set_arm(command="go_left", wait_for_end_of=True)
                        # time.sleep(3)
                        self.set_arm(command="go_up", wait_for_end_of=True)
                        # time.sleep(3)

                        self.set_arm(command="go_right", wait_for_end_of=True)
                        # time.sleep(3)
                        self.set_arm(command="go_down", wait_for_end_of=True)
                        # time.sleep(3) """
                        arm_value = Float32()



                        if error_x_bbox < -20.0:
                            print('move right')
                            arm_value.data = abs(error_x_bbox)
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_right", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                            
                        elif error_x_bbox > 20.0: 
                            print('move left')
                            arm_value.data = -error_x_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox < -20.0:
                            print('move down')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_down", wait_for_end_of=True)

                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox > 20.0:
                            print('move up')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_up", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')


                            
                        else:
                            print('Cheguei')
                            while True:
                                pass
                       
                        """ if error_x_bbox < 30.0 and error_y_bbox < 30.0:
                            print('CHEGAMOS')
                            print(error_x_bbox, error_y_bbox)
                            time.sleep(3)
                        else:
                            print('segue jogo') """
                        
                        print('\n \n \n \n')
                        # time.sleep(5)
            
            """ i = 0
            j = 0

            for a in current_frame_depth_hand:
                i += 1
                # print(a)
                print('---')
                j = 0

                mask_remaining = (current_frame_depth_hand > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)

                for b in a:
                    j += 1
                    print(b)
                    print('---')

            print(i)
            print(j)
            
            print('\n\n -- \n\n')

            time.sleep(3)

            """


            

            

            """ current_frame_depth_head_half = current_frame_depth_head[height//2:height,:]
            
            # FOR THE FULL IMAGE

            tot_pixeis = height*width 
            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head > 0) & (current_frame_depth_head <= near_max_dist)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_head > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            # FOR THE BOTTOM HALF OF THE IMAGE

            mask_zero_half = (current_frame_depth_head_half == 0)
            mask_near_half = (current_frame_depth_head_half > 0) & (current_frame_depth_head_half <= near_max_dist)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_head_half > near_max_dist) # just for debug
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
            
            full_image_near_err = pixel_count_near/tot_pixeis
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True
            
            
            if half_image_zero_or_near or full_image_near:
                overall = True """

            # just for debug
            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
            # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err
        
        return overall
        

    def search_for_person(self, tetas, delta_t=3.0):

        self.activate_yolo_pose(activate=True, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=False)                
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

        # print("FILTERED:")
        # for p in filtered_persons:
        #     print(p.index_person)

        return filtered_persons


    def detected_person_to_face_path(self, person, send_to_face):

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        
        cf = self.node.br.imgmsg_to_cv2(person.image_rgb_frame, "bgr8")
        just_person_image = cf[person.box_top_left_y:person.box_top_left_y+person.box_height, person.box_top_left_x:person.box_top_left_x+person.box_width]
        # cv2.imshow("Search for Person", just_person_image)
        # cv2.waitKey(100)
        cv2.imwrite(self.node.complete_path_custom_face + current_datetime + str(person.index_person) + ".jpg", just_person_image) 
        time.sleep(0.5)
        
        if send_to_face:
            self.set_face(custom=current_datetime + str(person.index_person))
        
        return current_datetime + str(person.index_person)
