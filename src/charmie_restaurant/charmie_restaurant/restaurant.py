#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from geometry_msgs.msg import Pose2D
from example_interfaces.msg import Bool, String, Int16
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Obstacles, SpeechType, RobotSpeech, TarNavSDNL, Yolov8Pose, NeckPosition, Yolov8Objects, SearchForPerson, ListOfPoints

from cv_bridge import CvBridge
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
        # PORTA FEZ AQUI TESTE  

        # Arm
        self.flag_arm_finish_subscriber = self.create_subscription(Bool, 'flag_arm_finished_movement', self.flag_arm_finish_callback, 10)  
        self.barman_or_client_publisher = self.create_publisher(Int16, "barman_or_client", 10)
        
        # Door Start
        self.start_door_publisher = self.create_publisher(Bool, 'start_door', 10) 
        #self.done_start_door_subscriber = self.create_subscription(Bool, 'done_start_door', self.done_start_door_callback, 10) 
        
        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Audio
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)
        self.get_speech_subscriber = self.create_subscription(String, "get_speech", self.get_speech_callback, 10)
        self.calibrate_ambient_noise_publisher = self.create_publisher(Bool, "calib_ambient_noise", 10)
        
        # Neck
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        self.neck_to_coords_publisher = self.create_publisher(Pose2D, "neck_to_coords", 10)
        
        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Low Level: Start Button
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        # Intel Realsense
        self.camera_subscriber = self.create_subscription(Image, "/color/image_raw", self.color_img_callbcak, 10)
        self.depth_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_depth_callback, 10)

        # Odometry
        self.odometry_subscriber = self.create_subscription(Odometry, "odom",self.get_odometry_robot_callback, 10)

        #Yolo
        self.yolov8_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.yolov8_pose_callback, 10)
        self.objects_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected', self.yolov8_objects_callback, 10)
        self.only_detect_person_arm_raised_publisher = self.create_publisher(Bool, "only_det_per_arm_raised", 10)

        # Navigation 
        ###self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        ###self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        #self.dummy_publisher = self.create_publisher(Bool, "Dummy", 10)
        
        #Search For Person
        self.search_for_person_publisher = self.create_publisher(SearchForPerson, 'search_for_person', 10)
        self.search_for_person_subscriber = self.create_subscription(ListOfPoints, "search_for_person_points", self.search_for_person_point_callback, 10)



        # SERVICE TR
        # self.bool_service_client = self.create_client(SetBool, 'bool_service')

        # while not self.bool_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')


        self.br = CvBridge()
        self.image_color = Image()
        self.depth_img = Image()
        self.speech_str = RobotSpeech()
        self.talk_neck = NeckPosition()
        self.look_down = NeckPosition()
        self.turn_around_neck = NeckPosition()
        self.neck_a = NeckPosition()
        self.neck_b = NeckPosition()
        self.check_object = NeckPosition()
        self.flag_arm_finish = False
        self.flag_search_for_person_done = False

        self.speech_type = SpeechType() # this variable must be deleted everywhere
        self.request_audio_yes_or_no = SpeechType()
        self.request_audio_yes_or_no.receptionist = False
        self.request_audio_yes_or_no.yes_or_no = True
        self.request_audio_yes_or_no.restaurant = False
        self.request_audio_yes_or_no.gpsr = False
        self.request_audio_restaurant = SpeechType()
        self.request_audio_restaurant.receptionist = False
        self.request_audio_restaurant.yes_or_no = False
        self.request_audio_restaurant.restaurant = True
        self.request_audio_restaurant.gpsr = False

        self.foods = []
        self.get_foods = 0
        self.collect_point = (0.0, 0.0)
        self.target_customer = Pose2D()
        self.x_relative = 0
        self.y_relative = 0
        self.aux = 0

        self.start_button_state = False
        self.flag_audio_done = False

        self.rgb_ctr = 2
        self.rgb = Int16()

        self.nose = 0

        self.target_time = 2.0

        self.i = 0
        self.j = 0
        self.ctr = 0
        self.flag_navigation_done = False

        self.yolo_poses = Yolov8Pose()
        self.yolo_objects = Yolov8Objects()
        self.person_list_of_points = ListOfPoints()

        self.flag_speech_done = False

        self.int_error = -500

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.t = 0.0

        self.counter = 0
        self.tempo = 4.0

        self.look_down.pan = 180.0
        self.look_down.tilt = 140.0

        self.turn_around_neck.pan = 360.0
        self.turn_around_neck.tilt = 180.0

        self.talk_neck.pan = 180.0
        self.talk_neck.tilt = 180.0

        self.neck_a.pan = 135.0
        self.neck_a.tilt = 180.0

        self.neck_b.pan = 225.0
        self.neck_b.tilt = 180.0

        self.check_object.pan = 120.0
        self.check_object.tilt = 187.0
        
        self.customer_position =[]

        self.keywords = "Nothing"

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
        #     self.get_logger().info('Service call result: Success: {response.success}, Message:{response.message}}')
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

    def flag_pos_reached_callback(self, state: Bool):
        print("Received Navigation Flag:", state.data)
        self.flag_navigation_done = True

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

    ###def flag_pos_reached_callback(self, state: Bool):
    ###   self.flag_navigation_done = state.data
    ###   #print("Received Navigation Flag:", state.data)

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
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False
        print("Finished wait for end of speaking")

    def wait_for_end_of_arm(self):
        print('11111')
        while not self.node.flag_arm_finish:
            pass
        self.node.flag_arm_finish = False

    def wait_for_end_of_audio(self):
        while not self.node.flag_audio_done:
            pass
        self.node.flag_audio_done = False
        self.node.flag_speech_done = False

    def wait_for_start_button(self):
        while not self.node.start_button_state:
            pass
        f = Bool()
        f.data = False 
        self.node.flag_start_button_publisher.publish(f)

    """ def wait_for_end_of_navigation(self, p_x=None, p_y=None):
        while not self.node.flag_navigation_done:
            if p_x is not None:
                time.sleep(0.05)
                pose = Pose2D()
                pose.x = p.x
                pose.y = p.y
                pose.theta = 160.0
                self.node.neck_to_coords_publisher.publish(pose)
            pass
        self.node.flag_navigation_done = False
        time.sleep(1)
        print("Finished Navigation") """

    def wait_for_end_of_navigation(self):
        self.node.flag_navigation_done = False
        self.node.rgb_ctr = 42
        self.node.rgb.data = self.node.rgb_ctr
        self.node.rgb_mode_publisher.publish(self.node.rgb)
        print("Started wait for end of navigation")
        while not self.node.flag_navigation_done:
            pass
        print("Finished wait for end of navigation")
        self.node.flag_navigation_done = False
        self.node.rgb_ctr = 62
        self.node.rgb.data = self.node.rgb_ctr
        self.node.rgb_mode_publisher.publish(self.node.rgb)
        time.sleep(1)
        print("Finished Navigation")

    def wait_for_end_of_search_for_person(self):
        while not self.node.flag_search_for_person_done:
            pass
        self.node.flag_search_for_person_done = False
        print("Finished search for person")

    def coordinates_to_navigation(self, p1, p2, bool1, bool2):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool1
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        nav.follow_me = bool2
        self.node.target_position_publisher.publish(nav)


    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6

        print("IN NEW MAIN")
        # time.sleep(1)

        ###while True:
        ###    self.wait_for_end_of_navigation()
        ###    self.node.speech_str.command = "Navigation Completed."
        ###    self.node.speaker_publisher.publish(self.node.speech_str)
        ###    self.wait_for_end_of_speaking()

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

                self.node.neck_position_publisher.publish(self.node.talk_neck)


                ########## DEMONSTRATION CODE FOR PRESS. GRAB 3 OBJECTS AND PLACE THEM IN TRAY AND IN TABLE #################

                """ print('a')

                place_to_go = Int16()
                place_to_go.data = 11
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                print('b')


                place_to_go = Int16()
                place_to_go.data = 0
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go.data = 10
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                print('c')

                place_to_go = Int16()
                place_to_go.data = 1
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                print('d')

                place_to_go = Int16()
                place_to_go.data = 2
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go.data = 10
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                place_to_go = Int16()
                place_to_go.data = 3
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go = Int16()
                place_to_go.data = 4
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go.data = 10
                print(place_to_go)
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                self.node.neck_position_publisher.publish(self.node.talk_neck)


                place_to_go = Int16()
                place_to_go.data = 5
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.look_down)

                place_to_go = Int16()
                place_to_go.data = 6
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go = Int16()
                place_to_go.data = 7
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go = Int16()
                place_to_go.data = 8
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                place_to_go = Int16()
                place_to_go.data = 9
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                while True:
                    pass  """

                ########## END OF DEMONSTRATION CODE


                ### @@@ Começar por fazer slam?
                
                # Print State
                self.node.rgb_ctr = 100
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                place_to_go = Int16()
                place_to_go.data = 11
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                print('State 0 = Initial')

                # COLOCAR SPEAKER AQUI
                

                self.node.speech_str.command = "Hello! I am ready to start the restaurant task! Waiting for my start button to be pressed."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()


                self.node.rgb_ctr = 26
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                # START BUTTON
                t = Bool()
                t.data = True
                self.node.flag_start_button_publisher.publish(t)
                self.wait_for_start_button()
                
                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.rgb_ctr = 41
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                time.sleep(1)
                 #calib = Bool()
                # calib.data = True
                # self.node.calibrate_ambient_noise_publisher.publish(calib)
                # time.sleep(3) # obrigatorio depois de recalibrar audio

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                
                # from now on only detected waving or calling people
                only_detect_person_arm_raised = Bool()
                only_detect_person_arm_raised.data = True
                self.node.only_detect_person_arm_raised_publisher.publish(only_detect_person_arm_raised)

                self.node.speech_str.command = "Hello! My name is charmie and I am here to help you serve the customers. Are you the barman?"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()


                ### AUDIO YES NO ###
                # self.node.audio_command_publisher.publish(self.node.request_audio_yes_or_no)
                # self.wait_for_end_of_audio()
                time.sleep(1)

                #time.sleep(1.0) # this is just so we can see the lights  
                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                """
                if self.node.keywords.data == "yes":
                    # print("YES SIR")
                    self.node.rgb_ctr = 11
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                else:
                    # in this case we should do something... for now I will just leave it like this, since they will never say no
                    # print("NO SIR")
                    self.node.rgb_ctr = 1
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                """
                self.node.speech_str.command = "Nice to meet you. I am going to turn around and search for possible customers. See you soon"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                #self.wait_for_end_of_navigation() 

                """ self.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (-1.0, 0.0), False, False)
                self.wait_for_end_of_navigation() 

                self.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (0.0, -1.0), False, False)
                self.wait_for_end_of_navigation() """
                                
                #Next State
                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                ### @@@ AQUI VOU-ME VIRAR 180 E VOU CHAMAR A FUNÇÃO DO SCAN DO PESCOÇO DO TIAGO ...
                # Algo do tipo wait for end of tracking 

                self.node.speech_str.command = "Searching for calling customers."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-120, -60, 0, 60, 120]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                print(self.node.person_list_of_points.coords)

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                # just for debug purposes, it has its own switch case so it can be done right after receving and not wait for neck
                if len(self.node.person_list_of_points.coords) == 0:
                    self.node.rgb_ctr = 0
                elif len(self.node.person_list_of_points.coords) == 1:
                    self.node.rgb_ctr = 30
                else:
                    self.node.rgb_ctr = 10
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                ### @@@ Diria também virar pelo menos pescoço para o barman, se não virar corpo todo...
                self.node.neck_position_publisher.publish(self.node.turn_around_neck)
                time.sleep(1)

                if len(self.node.person_list_of_points.coords) > 0:

                    if len(self.node.person_list_of_points.coords) == 1: # this if is just to say a different sentence because of singular and plurals
                        self.node.speech_str.command = "I have detected "+ str(len(self.node.person_list_of_points.coords))+" customer calling. Please look at my screen to see the customer."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                    else: # higher than 1
                        self.node.speech_str.command = "I have detected "+ str(len(self.node.person_list_of_points.coords))+" customers calling. Please look at my screen to see the customers."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()


                    p_ctr = 0 # just to count the people detected and add that info to the string that the robot will speak
                    for p in self.node.person_list_of_points.coords:
                        p_ctr+=1 # just to count the people detected and add that info to the string that the robot will speak
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        self.node.neck_to_coords_publisher.publish(pose)
                        time.sleep(1) # this is just so the looking at customers is not oo quick
                        self.node.speech_str.command = "I am looking at the detected calling customer "+str(p_ctr)+"." 
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        time.sleep(0.5) # this is just so the looking at customers is not oo quick


                    self.node.neck_position_publisher.publish(self.node.talk_neck)
                        
                    self.state = Navigation_to_person
                    self.node.aux = Searching_for_clients

                else:
                    self.node.speech_str.command = "I could not find any customers calling. I am going to search again for calling customers."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.state == Searching_for_clients

                    
            elif self.state == Navigation_to_person:
                print('State 2 = Navigation to Person')
                # Navigation

                """ self.node.customer_point.x = self.person_coordinates.x 
                self.node.customer_point.y = self.person_coordinates.y  """

                ### @@@ Tenho de retirar as coordenadas da point cloud, ou se no detect calling já faz isso usar essas
        

                if self.count == 0:

                    self.node.speech_str.command = "Moving to first customer."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                
                    self.node.target_customer = self.node.customer_position[0]

                    #self.wait_for_end_of_navigation()

                    """ self.coordinates_to_navigation((self.node.robot_x, self.node.robot_y),(self.node.target_customer.x, self.node.target_customer.y), False, False)
                    self.wait_for_end_of_navigation()

                    self.coordinates_to_navigation((self.node.target_customer.x, self.node.target_customer.y),(self.node.target_customer.x, self.node.target_customer.y), False, False)
                    self.wait_for_end_of_navigation() """

                    # self.node.speech_str.command = "I am going to move to the first customer."
                    # self.node.speaker_publisher.publish(self.node.speech_str)
                    # self.wait_for_end_of_speaking()

                elif self.count == 1:

                    self.node.speech_str.command = "Moving to next customer."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.target_customer = self.node.customer_position[0] ##### TEM QUE SER 1 porque é a segunda pessoa! mudamos só para demonstracao

                    #self.wait_for_end_of_navigation()

                    """ self.coordinates_to_navigation((self.node.robot_x, self.node.robot_y),(self.node.target_customer.x, self.node.target_customer.y), False, False)
                    self.wait_for_end_of_navigation(p_x=self.node.target_customer.x, p_y=self.node.target_customer.y)

                    self.coordinates_to_navigation((self.node.target_customer.x, self.node.target_customer.y),(self.node.target_customer.x, self.node.target_customer.y-1.0), False, False)
                    self.wait_for_end_of_navigation(p_x=self.node.target_customer.x, p_y=self.node.target_customer.y) """

                    # self.node.speech_str.command = "I am going to move to the next customer."
                    # self.node.speaker_publisher.publish(self.node.speech_str)
                    # self.wait_for_end_of_speaking()

                #Next State
                if self.node.aux == Searching_for_clients:
                    self.state = Receiving_order_speach
                elif self.node.aux == Navigation_to_person:
                    self.state = Delivering_order_to_client

            elif self.state == Receiving_order_speach:

                # Print State
                print('State 3 = Receive Order - Speech')
                #Next State
                self.state = Receiving_order_listen_and_confirm

            elif self.state == Receiving_order_listen_and_confirm:

                # Print State
                print('State 4 = Receive Order - Listening and Confirm ')

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                # Speech Receive Order

                # RECALIBRAR AUDIO 
                ### @@@  acho que as 2 próximas linhas de código são inúteis

                time.sleep(1)
                # calib = Bool()
                # calib.data = True
                # self.node.calibrate_ambient_noise_publisher.publish(calib)
                # time.sleep(3) # obrigatorio depois de recalibrar audio

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.speech_str.command = "Hello! My name is charmie and I am here to help you."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()


                ########## AUXILIAR CODE JUST FOR TESTING ...

                # this must be inside a while loop
                order_confirmation = False
                while not order_confirmation:
                    self.node.speech_str.command = "Please say your order."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    # Listening Audio

                    """ self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) """

                    ### AUDIO ###
                    # self.node.audio_command_publisher.publish(self.node.request_audio_restaurant)
                    # self.wait_for_end_of_audio()
                    time.sleep(1)

                    #time.sleep(1.0) # this is just so we can see the lights  
                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)    

                    self.pedido = self.node.keywords.data       

                    self.node.speech_str.command = "Did you say "+self.pedido+"?"
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    ### AUDIO YES NO ###
                    # self.node.audio_command_publisher.publish(self.node.request_audio_yes_or_no)
                    # self.wait_for_end_of_audio()
                    time.sleep(1)

                    #time.sleep(1.0) # this is just so we can see the lights  
                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    if self.node.keywords.data == "yes":
                        # print("YES SIR")
                        self.node.rgb_ctr = 11
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)
                                                
                        self.node.speech_str.command = "Thank you. I am going to get your order."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)
                        
                        order_confirmation = True

                        self.state = Collect_order_from_barman


                    else:
                        # in this case we should do something... for now I will just leave it like this, since they will never say no
                        # print("NO SIR")
                        self.node.rgb_ctr = 1
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)

                        self.node.speech_str.command = "Sorry for my mistake, let's try again."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)

                        self.pedido = ''


                order_confirmation = False

                ########## END OF AUXILIAR CODE JUST FOR TESTING ...

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                
            elif self.state == Collect_order_from_barman:

                # Print State
                print('State 5 = Collect Order')

                self.node.speech_str.command = "Moving to the barman."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                #self.wait_for_end_of_navigation()

                """ self.coordinates_to_navigation((self.node.robot_x ,self.node.robot_y), (0.0, 0.0), False, False)
                self.wait_for_end_of_navigation()

                self.coordinates_to_navigation((0.0, 1.0),(0.0, 2.5), False, False)
                self.wait_for_end_of_navigation() """

                time.sleep(1)
                # calib = Bool()
                # calib.data = True
                # self.node.calibrate_ambient_noise_publisher.publish(calib)
                # time.sleep(3) # obrigatorio depois de recalibrar audio

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)


                # Collect Order
                #if self.node.get_foods == 1:
                self.node.speech_str.command ="Hello again, barman. The customer order is" + self.pedido + ". I will get ready to receive it."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                # Listening Audio

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                ### @@@ Preparing to grab the first item
                place_to_go = Int16()
                place_to_go.data = 0
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()
                print('a')

                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "Please let me know when you place the order on my hand by saying, yes robot."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                # self.node.audio_command_publisher.publish(self.node.request_audio_yes_or_no)
                # self.wait_for_end_of_audio()
                
                ### @@@ Preparing to check the first item 

                place_to_go = Int16()
                place_to_go.data = 10
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "This object is correct."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.neck_position_publisher.publish(self.node.talk_neck)
                        
                ### @@@ Preparing to place the first item on tray
                place_to_go = Int16()
                place_to_go.data = 1
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()


                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)


                ### @@@ Preparing to grab the second item
                place_to_go = Int16()
                place_to_go.data = 2
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()


                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "Please let me know when you place the order on my hand by saying, yes robot."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                # self.node.audio_command_publisher.publish(self.node.request_audio_yes_or_no)
                # self.wait_for_end_of_audio()
                
                ### @@@ Preparing to check the first item 

                place_to_go = Int16()
                place_to_go.data = 10
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()
                print('a')                

                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "This object is correct."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                ### @@@ Preparing to place the second item on tray
                place_to_go = Int16()
                place_to_go.data = 3
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                ### @@@ Preparing to grab the third item
                place_to_go = Int16()
                place_to_go.data = 4
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()


                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "Please let me know when you place the order on my hand by saying, yes robot."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                """ self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) """

                # self.node.audio_command_publisher.publish(self.node.request_audio_yes_or_no)
                # self.wait_for_end_of_audio()
                
                ### @@@ Preparing to check the first item 

                place_to_go = Int16()
                place_to_go.data = 10
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()
                print('a')                
                
                self.node.neck_position_publisher.publish(self.node.check_object)

                time.sleep(2.0)

                #Adicionar fala para colocar o objeto na mão
                self.node.speech_str.command = "This object is correct."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                ### @@@ Preparing to place the third item on initial position
                place_to_go = Int16()
                place_to_go.data = 5
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                ### @@@ Talvez para fase intermédia de testes tente apenas virar sobre ele 180

                self.node.aux = Navigation_to_person
                self.state = Navigation_to_person

            elif self.state == Delivering_order_to_client:

                # Print State
                print('State 6 = Final Speech')

                
                self.node.neck_position_publisher.publish(self.node.look_down)

                #Deliver Order
                self.node.speech_str.command = "Here are the items you have ordered. Please wait while I place them in front of you." 
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()                
                #self.state = 1

                place_to_go = Int16()
                place_to_go.data = 6
                self.node.barman_or_client_publisher.publish(place_to_go)
                self.wait_for_end_of_arm()

                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)


                if self.count == 0:
                    ### Primeiro cliente
                
                    self.node.speech_str.command = "Here is your pear." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 

                    place_to_go = Int16()
                    place_to_go.data = 7
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)


                    self.node.speech_str.command = "Here is your orange juice." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 

                    place_to_go = Int16()
                    place_to_go.data = 8
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.node.speech_str.command = "Here is your milk." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 
                    
                    place_to_go = Int16()
                    place_to_go.data = 9
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                elif self.count == 1:
                    ### Segunfo cliente
                
                    self.node.speech_str.command = "Here is your water." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 

                    place_to_go = Int16()
                    place_to_go.data = 7
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)


                    self.node.speech_str.command = "Here are your pringles." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 

                    place_to_go = Int16()
                    place_to_go.data = 8
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.node.speech_str.command = "Here is your seven up." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking() 
                    
                    place_to_go = Int16()
                    place_to_go.data = 9
                    self.node.barman_or_client_publisher.publish(place_to_go)
                    self.wait_for_end_of_arm()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.speech_str.command = "Hope you enjoy it. See you soon." 
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()  

                time.sleep(0.5)


                self.node.neck_position_publisher.publish(self.node.talk_neck)


                self.state = Navigation_to_person
                self.node.aux = Searching_for_clients
                self.count += 1
                if self.count == 2:
                    self.state = 99
                else:
                    self.node.speech_str.command = "Moving to the barman." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()  

                    #self.wait_for_end_of_navigation()

                    """ self.coordinates_to_navigation((self.node.robot_x ,self.node.robot_y), (0.0, 0.0), False, False)
                    self.wait_for_end_of_navigation()

                    self.coordinates_to_navigation((0.0, 1.0),(0.0, 2.5), False, False)
                    self.wait_for_end_of_navigation() """

     
            
            elif self.state == 99:
                self.node.speech_str.command = "I have finished my restaurant task." 
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()  
                self.state += 1

            else:
                pass