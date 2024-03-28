#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Bool, Float32, Int16, String 
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject
from charmie_interfaces.srv import TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects
from sensor_msgs.msg import Image

import cv2 
import threading
import time
from cv_bridge import CvBridge

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        ### Topics (Publisher and Subscribers) ###  
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)

        # self.only_detect_person_legs_visible_subscriber = self.create_subscription(Bool, "only_det_per_legs_vis", self.get_only_detect_person_legs_visible_callback, 10)
        # self.minimum_person_confidence_subscriber = self.create_subscription(Float32, "min_per_conf", self.get_minimum_person_confidence_callback, 10)
        # self.minimum_keypoints_to_detect_person_subscriber = self.create_subscription(Int16, "min_kp_det_per", self.get_minimum_keypoints_to_detect_person_callback, 10)
        # self.only_detect_person_right_in_front_subscriber = self.create_subscription(Bool, "only_det_per_right_in_front", self.get_only_detect_person_right_in_front_callback, 10)
        # self.only_detect_person_arm_raised_subscriber = self.create_subscription(Bool, "only_det_per_arm_raised", self.get_only_detect_person_arm_raised_callback, 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        ### Services (Clients) ###
        # Neck
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        
        # while not self.neck_track_person_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Neck Track Person ...")
        
        # Variables
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False

        # Sucess and Message confirmations for all set_(something) CHARMIE functions
        self.rgb_sucess = True
        self.rgb_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        self.track_object_success = True
        self.track_person_message = ""
        self.activate_yolo_pose_sucess = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_sucess = True
        self.activate_yolo_objects_message = ""

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()
        
        cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
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
            cv2.waitKey(1)
        # cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
        # cv2.waitKey(10)

        # cv2.imwrite("object_detected_test4.jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        # cv2.waitKey(10)

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
    def call_activate_yolo_objects_server(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_object_confidence=0.5):
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.minimum_object_confidence = minimum_object_confidence

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

        self.node.rgb_sucess = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_sucess, self.node.rgb_message


    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_sucess = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_sucess, self.node.activate_yolo_pose_message

    def activate_yolo_objects(self, activate_objects=True, activate_shoes=False, activate_doors=False, minimum_object_confidence=0.5, wait_for_end_of=True):
        
        # self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, characteristics=characteristics)
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, minimum_object_confidence=minimum_object_confidence)

        self.node.activate_yolo_objects_sucess = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_sucess, self.node.activate_yolo_pose_message

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
        time.sleep(2)


        p_ = DetectedPerson()

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

                ### EXAMPLE TO TRACK PERSON
                # if self.node.detected_people.num_person > 0:
                    # p_=self.node.detected_people.persons[0]
                    # print(p_.head_center_x, p_.head_center_y)
                    # self.track_person(p_, body_part="Head", wait_for_end_of=True)
                    # print(".")
                    # time.sleep(5)
                
                ### EXAMPLE TO TRACK OBJECT
                if self.node.detected_objects.num_objects > 0:
                    p_=self.node.detected_objects.objects[0]
                    print(p_.object_name)
                    self.track_object(p_,wait_for_end_of=True)
                    print(".")
                    time.sleep(5)

                ### EXAMPLES TO ACTIVATE/DEACTIVATE AND CONFIGURE YOLO POSE AND TOLO OBJECTS 
                # self.activate_yolo_pose(activate=True)
                # self.activate_yolo_objects(activate_objects=False)
                # print("activated yolo pose")
                # time.sleep(10)
                # self.activate_yolo_pose(activate=True, minimum_keypoints_to_detect_person=7)
                # self.activate_yolo_objects(activate_objects=True, minimum_object_confidence=0.3)
                # print("deactivated yolo pose - 0.8")
                # time.sleep(5)
                # self.activate_yolo_pose(activate=True, minimum_keypoints_to_detect_person=10)
                # self.activate_yolo_objects(activate_objects=True, minimum_object_confidence=0.8)
                # print("deactivated yolo pose - right in front")
                # time.sleep(5)


                """
                s, m = self.set_rgb(RED+MOON)
                print(s, m)
                success, message = self.set_speech(filename="arm/arm_close_gripper", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)
                self.set_rgb(BLUE+ROTATE)
                success, message = self.set_speech(filename="arm/introduction_full", command="", wait_for_end_of=False)
                print(success, message)
                time.sleep(5)
                self.set_rgb(WHITE+ALTERNATE_QUARTERS)
                success, message = self.set_speech(filename="arm/arm_close_grippe", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)

                """

                """
                files = ["recep_characteristic_1", "recep_characteristic_2", "recep_characteristic_3", "recep_characteristic_4"]
                commands = ["The first guest shirt is black", "Its age is between 23 and 32", "The guest is a bit taller than me", "and its ethnicity is white."]
                self.save_speech(files, commands)
                print("...")
                time.sleep(5)
                print("...")


                self.set_speech(filename="temp/recep_characteristic_1", wait_for_end_of=True)
                

                # self.set_speech(command="Hello brother", wait_for_end_of=True)
                
                # self.set_speech(filename="generic/introduction_full", command="", wait_for_end_of=True)
                # time.sleep(2)

                self.set_speech(filename="receptionist/recep_drink_milk", command="", wait_for_end_of=True)
                self.set_face("help_pick_cup")
                time.sleep(3)


                # self.set_speech(filename="generic/introduction_ful", command="", wait_for_end_of=True)




                # self.node.test_custom_image_face_str.data = "clients_temp"
                # self.node.custom_image_to_face_publisher.publish(self.node.test_custom_image_face_str)
                # time.sleep(5)

                # self.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                
                # self.set_face(custom="clients_temp")
                # time.sleep(3)


                self.set_face("help_pick_bowl")
                time.sleep(3)

                self.set_speech(filename="receptionist/recep_drink_orange_juice", show_in_face=True, wait_for_end_of=True)


                self.set_face("demo8")
                time.sleep(3)

                self.set_face(custom="clients_tem")
                time.sleep(3)

                # self.set_speech(filename="arm/arm_close_gripper", command="", wait_for_end_of=True)
                # time.sleep(2)


                self.set_speech(filename="receptionist/recep_drink_red_wine", wait_for_end_of=True)

                self.set_face("help_pick_milk")
                time.sleep(3)

                self.set_face("help_pick_spoon")
                time.sleep(3)


                self.set_speech(filename="generic/introduction_full", show_in_face=True, wait_for_end_of=True)
                # start = time.time()
                # while time.time() < start + 3: # in seconds
                #     pass
                    # print(",", end='')
                # print()

                """

                #print('State 0 = Initial')

                # your code here ...
                                
                # next state
                # self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass