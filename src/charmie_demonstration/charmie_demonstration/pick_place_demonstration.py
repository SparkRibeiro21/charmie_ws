#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String, Int16
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Obstacles, SpeechType, RobotSpeech, TarNavSDNL, Yolov8Pose, NeckPosition, Yolov8Objects, SearchForPerson, ListOfPoints
from charmie_interfaces.srv import SpeechCommand

import time
from datetime import datetime, timedelta
from std_srvs.srv import SetBool
from functools import partial

import cv2
from math import sin, cos, pi, atan2, sqrt

class RestaurantNode(Node):

    def __init__(self):
        super().__init__("Restaurant")
        self.get_logger().info("Initialised CHARMIE Restaurant Node")
        
        ###         PUBs/SUBs       

        # Arm
        self.flag_arm_finish_subscriber = self.create_subscription(Bool, 'flag_arm_finished_movement', self.flag_arm_finish_callback, 10)  
        self.barman_or_client_publisher = self.create_publisher(Int16, "barman_or_client", 10)

        #Speaker
        self.client = self.create_client(SpeechCommand, "speech_command")
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Neck
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)

        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Low Level: RGB
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        self.speech_str = RobotSpeech()
        self.talk_neck = NeckPosition()
        self.table_neck = NeckPosition()
        self.turn_around_neck = NeckPosition()
        self.neck_pick_object_from_hand = NeckPosition()
        self.place_objects_tray = NeckPosition()
        self.check_object = NeckPosition()
        self.flag_arm_finish = False

        self.flag_navigation_done = False

        self.start_button_state = False
        self.flag_audio_done = False

        self.rgb_ctr = 2
        self.rgb = Int16()

        self.flag_speech_done = False

        self.int_error = -500

        self.robot_x = 0.0
        self.robot_y = 0.0


        self.turn_around_neck.pan = 360.0
        self.turn_around_neck.tilt = 180.0

        self.talk_neck.pan = 180.0
        self.talk_neck.tilt = 180.0

        self.table_neck.pan = 135.0
        self.table_neck.tilt = 150.0

        self.neck_pick_object_from_hand.pan = 150.0
        self.neck_pick_object_from_hand.tilt = 180.0

        self.place_objects_tray.pan = 180.0
        self.place_objects_tray.tilt = 110.0

        self.check_object.pan = 120.0
        self.check_object.tilt = 187.0

        self.waited_for_end_of_speaking = False
        
    def call_service(self):
        request = SetBool.Request()
        request.data = True

        print("Calling SetBool TR Service")

    
        future = self.bool_service_client.call_async(request)
        future.add_done_callback(partial(self.callback_service_tr))
        print('teste')

        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     response = future.result()
        #     self.get_logger().info('Service call result: Sucess: {response.sucess}, Message:{response.message}}')
        # else:
        #     self.get_logger().error('Service call failed')

    def callback_service_tr(self, future):
        try:
            response = future.result()
            print("acho que deu caralho")
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

    def get_odometry_robot_callback(self, loc:Odometry):
        self.robot_x = loc.pose.pose.position.x
        self.robot_y = loc.pose.pose.position.y

        qx = loc.pose.pose.orientation.x
        qy = loc.pose.pose.orientation.y
        qz = loc.pose.pose.orientation.z
        qw = loc.pose.pose.orientation.w

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)

        self.robot_theta = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

    def get_speech_done_callback(self, state: Bool):
        self.flag_speech_done = state.data
        # print("Received Speech Flag:", state.data)

    def flag_arm_finish_callback(self, flag: Bool):
        self.flag_arm_finish = flag.data
        print("Received Arm Flag:", flag.data)

    # def flag_listening_callback(self, flag: Bool):
    #     print("Finished Listening, now analising...")

    def get_speech_callback(self, keywords : String):
        print("Received Audio:", keywords.data)
        self.keywords = keywords
        self.flag_audio_done = True

    def flag_pos_reached_callback(self, state: Bool):
        self.flag_navigation_done = state.data
        #print("Received Navigation Flag:", state.data)

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

    def color_img_callbcak(self, img: Image):
        #print('camera callback')
        self.image_color = img
        #self.get_logger().info('Receiving color video frame')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        """ cv2.imshow("c_camera", current_frame)   
        cv2.waitKey(1) """

    def get_depth_callback(self, img: Image):
        #print('camera callback')
        self.depth_img = img

    def yolov8_pose_callback(self, yolov8: Yolov8Pose):

        #self.get_logger().info('Receiving Yolov8 Pose Info')
        """ if(yolov8.persons[0].kp_nose_y != None):
            self.nose = yolov8.persons[0].kp_nose_y

        else:
            self.nose = self.int_error """
        

        self.yolo_poses = yolov8

    def yolov8_objects_callback(self, yolov8_objects: Yolov8Objects):
        self.yolo_objects = yolov8_objects

    def search_for_person_point_callback(self, LoP: ListOfPoints):
        self.person_list_of_points = LoP
        self.flag_search_for_person_done = True

    def coordinates_to_navigation(self, p1, p2, bool1, bool2):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool1
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        nav.follow_me = bool2
        self.target_position_publisher.publish(nav)

    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
    
        future = self.client.call_async(request)
        print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
    


    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success)+str(response.message))
            # print("oi")
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = RestaurantNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: RestaurantNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: RestaurantNode):
        self.node = node
        
        self.state = 0
        #self.state = 1
        self.count = 0
        self.hand_raised = 0
        self.person_coordinates = Pose2D()
        self.person_coordinates.x = 0.0
        self.person_coordinates.y = 0.0

        self.neck_pose = NeckPosition()
        self.neck_pose.pan = 180.0
        self.neck_pose.tilt = 193.0

        self.target_x = 0.0
        self.target_y = 0.0

        self.pedido = ''

        self.i = 0
        
    def wait_for_end_of_arm(self):
        print('11111')
        while not self.node.flag_arm_finish:
            pass
        self.node.flag_arm_finish = False

    def wait_for_end_of_navigation(self):
        self.node.flag_navigation_done = False
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False
        print("Finished Navigation")

    def speech_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False
    

    def main(self):
        Initial_state = 0
        Go_grab_first_object = 1
        Go_place_first_object_tray = 2
        Go_grab_second_object = 3
        Go_place_second_object_tray = 4
        Go_grab_third_object = 5
        Go_place_third_object_tray = 6
        Go_place_first_object_table = 7
        Go_place_second_object_table = 8
        Go_place_third_object_table = 9
        Go_rest = 10

        print("IN NEW MAIN")
        time.sleep(1)

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Initial_state:

                # self.node.neck_position_publisher.publish(self.node.talk_neck)

                ### @@@ Começar por fazer slam?
                
                # Print State
                self.node.rgb_ctr = 100
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                print('State 0 = Initial')

                self.speech_server(filename="introduction_full", command="", wait_for_end_of=True)

                print('after speak 1')

                self.wait_for_end_of_navigation()

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="demo_three_objects", command="", wait_for_end_of=True)

                print('after speak 2 ')

                self.speech_server(filename="demo_ready_objects", command="", wait_for_end_of=True)

                print('after speak 3')

                #self.state = 999
                self.state = Go_grab_first_object

                # COLOCAR SPEAKER AQUI

            elif self.state == Go_grab_first_object:

                print('State 1 = Go grab first object')

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.neck_position_publisher.publish(self.node.neck_pick_object_from_hand)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 0
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.speech_server(filename="arm_place_object_gripper", command="", wait_for_end_of=True)

                self.state = Go_place_first_object_tray

            elif self.state == Go_place_first_object_tray:

                print('State 2 = Go place first object tray')

                time.sleep(1)

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="arm_close_gripper", command="", wait_for_end_of=True)
                
                self.node.neck_position_publisher.publish(self.node.place_objects_tray)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 1
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.state = Go_grab_second_object

            elif self.state == Go_grab_second_object:

                print('State 3 = Go grab second object')

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.neck_position_publisher.publish(self.node.neck_pick_object_from_hand)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 2
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()
                
                self.speech_server(filename="arm_place_object_gripper", command="", wait_for_end_of=True)

                self.state = Go_place_second_object_tray

            elif self.state == Go_place_second_object_tray:
                
                time.sleep(1)

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="arm_close_gripper", command="", wait_for_end_of=True)

                print('State 4 = Go place second object tray')

                self.node.neck_position_publisher.publish(self.node.place_objects_tray)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 3
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.state = Go_grab_third_object

            elif self.state == Go_grab_third_object:

                print('State 5 = Go grab third object')

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.neck_position_publisher.publish(self.node.neck_pick_object_from_hand)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 4
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.speech_server(filename="arm_place_object_gripper", command="", wait_for_end_of=True)

                self.state = Go_place_third_object_tray

            elif self.state == Go_place_third_object_tray:

                time.sleep(1)

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="arm_close_gripper", command="", wait_for_end_of=True)

                print('State 6 = Go place third object tray')

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 5
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                ##### MUDAR ESTE MOVE TABLE DE SITIO PARA UM ESTADO QUE FAÇA MAIS SENTIDO

                self.speech_server(filename="move_table", command="", wait_for_end_of=True)

                self.node.rgb_ctr = 46
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.wait_for_end_of_navigation()
                
                self.state = Go_place_first_object_table

            elif self.state == Go_place_first_object_table:

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="place_object_table", command="", wait_for_end_of=True)

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="place_stay_clear", command="", wait_for_end_of=True)

                print('State 7 = Go place first object table')

                self.node.neck_position_publisher.publish(self.node.table_neck)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 6
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.speech_server(filename="place_object_placed", command="", wait_for_end_of=True)
                
                self.state = Go_place_second_object_table

            elif self.state == Go_place_second_object_table:

                print('State 8 = Go place second object table')

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                #self.node.neck_position_publisher.publish(self.node.table_neck)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 7
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.speech_server(filename="place_object_placed", command="", wait_for_end_of=True)
                
                self.state = Go_place_third_object_table

            elif self.state == Go_place_third_object_table:

                print('State 9 = Go place third object table')

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                #self.node.neck_position_publisher.publish(self.node.table_neck)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 8
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.speech_server(filename="place_object_placed", command="", wait_for_end_of=True)
                
                self.state = Go_rest

            elif self.state == Go_rest:

                print('State 10 = Go rest')

                self.node.rgb_ctr = 14
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 9
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()
                
                self.state = 999
            

            elif self.state == 999:
                #self.node.speech_str.command = "I finished my restaurant task." 
                #self.node.speaker_publisher.publish(self.node.speech_str)
                #self.wait_for_end_of_speaking() 

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.speech_server(filename="demo_end", command="", wait_for_end_of=True)

                
                #self.node.neck_position_publisher.publish(self.node.place_objects_tray)

                place_to_go = Int16()
                place_to_go.data = 999
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                print('FINISHED')
