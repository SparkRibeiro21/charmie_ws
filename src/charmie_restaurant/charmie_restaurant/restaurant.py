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


class RestaurantNode(Node):

    def __init__(self):
        super().__init__("Restaurant")
        self.get_logger().info("Initialised CHARMIE Restaurant Node")

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
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        # Localisation
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Navigation
        #self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        #self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)


        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # Audio
        """
        while not self.get_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Audio Server...")
        while not self.calibrate_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Calibrate Audio Server...")
        """
        """
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
        while not self.activate_yolo_pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        while not self.activate_yolo_objects_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        """
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.start_button_state = False

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
        self.track_object_success = True
        self.track_person_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()
        
        cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        cv2.waitKey(10)

    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

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
            cv2.waitKey(10)
            pass
        # cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
        # cv2.waitKey(10)

        # cv2.imwrite("object_detected_test4.jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
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
    node = RestaurantNode()
    th_main = threading.Thread(target=ThreadMainRestaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainRestaurant(node: RestaurantNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: RestaurantNode):
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
                self.node.call_audio_server(yes_or_no=yes_or_no, receptionist=receptionist, gpsr=gpsr, restaurant=restaurant, wait_for_end_of=wait_for_end_of)
                
                if wait_for_end_of:
                    while not self.node.waited_for_end_of_audio:
                        pass
                self.node.waited_for_end_of_audio = False

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
        
        # States in Restaurant Task
        self.Waiting_for_task_start = 0
        self.Detecting_waving_customer = 1
        self.Approach_customer_table = 2
        self.Receiving_order_listen_and_confirm = 3
        self.Collect_order_from_barman = 4
        self.Delivering_customer_order = 5
        self.Final_State = 6
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.look_customers = [0, 0]

        # Start Localisation Position
        self.initial_position = [0.0, 0.1, 0.0]

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Receiving_order_listen_and_confirm

        self.state_aux = 0
        self.state_var = 0
        self.customer_index = 0
        self.delivery_index = 0
        self.all_orders = []

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Restaurant Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")

                # moves the neck to look down for navigation
                #self.set_neck(position=self.look_navigation, wait_for_end_of=False)

                # send speech command to speakers voice, intrucing the robot 
                self.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                
                # sends RGB value for debug
                #self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)
                
                ##### SPEAK : Hello! I am ready to start the restaurant task
                self.set_speech(filename="restaurant/start_restaurant", wait_for_end_of=True)

                # waiting for start button
                #self.wait_for_start_button()

                #self.set_rgb(command=CYAN+ALTERNATE_QUARTERS)

                # calibrate the background noise for better voice recognition
                # self.calibrate_audio(wait_for_end_of=True)

                # change face, to standard face
                #self.set_face("demo5")

                ##### START TURN TO BARMAN TABLE!!!

                ##### NECK LOOKS AT Barman
                #self.set_neck(position=self.look_forward, wait_for_end_of=True)

                ##### SPEAK : Hello! Nice to meet you! My name is charmie and I am here to help you serve the customers.
                self.set_speech(filename="restaurant/barman_meeting", wait_for_end_of=True)

                ##### SPEAK : I am going to turn around and search for possible customers. See you soon
                self.set_speech(filename="restaurant/turn_search", wait_for_end_of=True)

                ##### TURN AROUND to the customer zone
                #### NAVIGATION

                # next state
                self.state = self.Detecting_waving_customer

            elif self.state == self.Detecting_waving_customer:
                print("State:", self.state, "- Detecting_waving_customer")

                error_detected = False # Assume no error by default

                ##### SPEAK: Searching for waving customers
                self.set_speech(filename="restaurant/search_customers", wait_for_end_of=True)

                #self.set_rgb(command=ORANGE+HALF_ROTATE)

                ##### NECK: Rotate left and right for better view
                ##### !!!!!!!TIAGO ESTÁ A FAZER A NOVA VERSÃO DESTA FUNÇÃO!!!!!!!!

                ##### YOLO POSE SEARCH FOR ONLY_DETECT_PERSON_ARM_RAISED
                #self.activate_yolo_pose(activate=True, only_detect_person_arm_raised=True)
                #print("activated yolo pose")
                #time.sleep(5)

                ##### IF AN ERROR IS DETECTED:
                if error_detected:

                    #self.set_rgb(command=RED+BLINK_QUICK)

                    ##### SPEAK: No costumers detected
                    self.set_speech(filename="restaurant/no_customers", wait_for_end_of=True)

                    ##### BACK TO initial searching
                    self.state = self.Detecting_waving_customer

                else:

                    #self.set_rgb(command=GREEN+BLINK_QUICK)

                    ##### SPEAK: Found waving customers
                    self.set_speech(filename="restaurant/found_customer", wait_for_end_of=True)
                    #self.customer_index = self.customer_index + 1

                    ##### NECK: look waving customers
                    ##### !!!!!!!self.look_customers está a zero! Tem de se ir buscar as coordenadas dos customers detetados!!!!!!!
                    #self.set_neck(position=self.look_customers, wait_for_end_of=True)
                    
                    ##### SPEAK: Check face to see customers detected
                    self.set_speech(filename="restaurant/face_customer", wait_for_end_of=True)

                    ##### SHOW FACE DETECTED CUSTOMER 
                    ##### !!!!!!!FALTA IR AO YOLO BUSCAR A DETEÇÃO DOS WAVING CUSTOMERS E METER AQUI !!!!!!!!!!
                    #self.set_face("customer1")
                    #self.set_face("customer2")
                                    
                    # next state
                    self.state = self.Approach_customer_table
                
            elif self.state == self.Approach_customer_table:
                print("State:", self.state, "- Approach_customer_table")

                ##### NECK MOVEMENT FORWARD POSITION
                #self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                ##### BACK TO STANDART FACE
                self.set_face("demo5")
                
                ##### SPEAK: Start Movement Alert
                self.set_speech(filename="restaurant/movement_alert", wait_for_end_of=True)

                #self.set_rgb(command=BLUE+ROTATE)

                ##### MOVE TO CUSTOMER
                # MISSING NAVIGATION ... (TIAGO)
                                
                # next state
                if self.state_aux == 0:

                    ##### NECK: Look to customer
                    #self.set_neck(position=self.look_customers, wait_for_end_of=True)

                    ##### SPEAK: Hello
                    self.set_speech(filename="restaurant/hello_customer", wait_for_end_of=True)
                    self.state = self.Receiving_order_listen_and_confirm 

                elif self.state_aux == 1:
                    self.state = self.Delivering_customer_order

            elif self.state == self.Receiving_order_listen_and_confirm:
                print("State:", self.state, "- Receiving_order_listen_and_confirm")

                while True:

                    #self.set_rgb(command=BLUE+ALTERNATE_QUARTERS)

                    ##### AUDIO: Listen the order and repeat for confirmation
                    command = self.get_audio(restaurant=True, question="restaurant/what_is_your_order", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list = command.split(" ")
                    self.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                    for kw in keyword_list:
                        print(kw)
                        self.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)

                    ##### AUDIO: Listen "YES" OR "NO"
                    ##### "Please say yes or no to confirm the order"
                    confirmation = self.get_audio(yes_or_no=True, question="restaurant/yes_no_question", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                        #self.set_rgb(command=GREEN+BLINK_LONG)
                        ##### SPEAK: Thank you
                        self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
                        break  # Sai do loop se a confirmação for "yes"
                    elif confirmation.lower() == "no":
                        #self.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry, TRY AGAIN
                        self.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                    else:
                        #self.set_rgb(command=YELLOW+BLINK_LONG)
                        ##### ERROR
                        print("ERROR")

                print("teste final")
                self.customer_index = self.customer_index + 1

                # next state
                if self.customer_index == 1:
                    self.state = self.Detecting_waving_customer 
                elif self.customer_index == 2:
                    self.state = self.Collect_order_from_barman

            elif self.state == self.Collect_order_from_barman:
                print("State:", self.state, "- Collect_order_from_barman")
                
                ##### SPEAK: Start moving
                self.set_speech(filename="restaurant/move_barman", wait_for_end_of=True)

                #self.set_rgb(command=BLUE+ROTATE)

                ##### MOVE TO THE BARMAN TABLE
                # MISSING NAVIGATION ... (TIAGO)

                ##### SPEAK: Say the order to barman
                self.set_speech(filename="restaurant/say_order_to_barman", wait_for_end_of=True)

                ##### SPEAK: Say order elements
                ##### Itera sobre cada elemento do pedido
                for pedido in self.all_orders:
                    for elemento in pedido:
                        # Define o nome do arquivo correspondente ao elemento
                        filename = "objects_names/" + elemento.lower().replace(" ", "_")  # Supondo que os arquivos estejam na pasta "restaurant" e tenham o mesmo nome que os elementos em minúsculas
                        
                        # SPEAK: Diz o elemento do pedido
                        self.set_speech(filename=filename, wait_for_end_of=True)

                ##### SPEAK: Say to the barman put the order in the tray
                self.set_speech(filename="restaurant/tray_order", wait_for_end_of=True)

                # next state
                self.state_aux = 1
                self.state = self.Approach_customer_table

            elif self.state == self.Delivering_customer_order:
                print("State:", self.state, "- Delivering_customer_order")
                
                ##### SPEAK: Here are the items, CAN YOU PICK FROM MY TRAY?
                self.set_speech(filename="restaurant/pick_tray", wait_for_end_of=True)

                ##### SPEAK: Hope you enjoy!
                self.set_speech(filename="restaurant/enjoy_order", wait_for_end_of=True)

                self.delivery_index = self.delivery_index + 1
                                
                # next state
                if self.delivery_index == 1:
                    self.state = self.Approach_customer_table 
                elif self.delivery_index == 2:
                    self.state = self.Final_State 
                
            elif self.state == self.Final_State:

                ##### SPEAK: FINISHING RESTAURANT TASK
                
                self.set_speech(filename="restaurant/finish_restaurant", wait_for_end_of=True)

                #self.set_rgb(command=RAINBOW_ROT)

                # Lock after finishing task
                while True:
                    pass

            else:
                pass
