#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String, Int16
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Obstacles, SpeechType, RobotSpeech, TarNavSDNL, Yolov8Pose

from cv_bridge import CvBridge
import time
from datetime import datetime, timedelta

import cv2
from math import sin, cos, pi, atan2, sqrt

class RestaurantNode(Node):

    def __init__(self):
        super().__init__("Restaurant")
        self.get_logger().info("Initialised CHARMIE Restaurant Node")
        
        ###         PUBs/SUBs        
        
        # Door Start
        self.start_door_publisher = self.create_publisher(Bool, 'start_door', 10) 
        #self.done_start_door_subscriber = self.create_subscription(Bool, 'done_start_door', self.done_start_door_callback, 10) 
        
        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Audio
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)
        self.flag_listening_subscriber = self.create_subscription(Bool, "flag_listening", self.flag_listening_callback, 10)
        self.get_speech_subscriber = self.create_subscription(String, "get_speech", self.get_speech_callback, 10)
        self.calibrate_ambient_noise_publisher = self.create_publisher(Bool, "calib_ambient_noise", 10)
        
        # Neck
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        
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
        self.yolov8_pose_subscriber = self.create_subscription(Yolov8Pose, "/yolov8_pose", self.yolov8_callback, 10)

        #self.dummy_publisher = self.create_publisher(Bool, "Dummy", 10)


        self.br = CvBridge()
        self.image_color = Image()
        self.depth_img = Image()
        self.speech_type = SpeechType()
        self.speech_type.receptionist = False
        self.speech_type.yes_or_no = False
        self.speech_type.restaurant = False
        self.speech_type.gpsr = False
        self.speech_str = RobotSpeech()
        self.talk_neck = Pose2D()

        self.foods = []
        self.get_foods = 0
        self.collect_point = (0.0, 0.0)
        self.customer_point = Pose2D()
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

        self.flag_speech_done = False

        self.int_error = -500

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.t = 0.0

        self.counter = 0
        self.tempo = 4.0

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
        print("Received Speech Flag:", state.data)


    def flag_listening_callback(self, flag: Bool):
        print("Finished Listening, now analising...")


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

    def yolov8_callback(self, yolov8: Yolov8Pose):

        #self.get_logger().info('Receiving Yolov8 Pose Info')
        """ if(yolov8.keypoints[0].key_p0_y != None):
            self.nose = yolov8.keypoints[0].key_p0_y

        else:
            self.nose = self.int_error """
        

        self.yolo_poses = yolov8

    def coordinates_to_navigation(self, p1, p2, bool1, bool2):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool1
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        nav.follow_me = bool2
        self.target_position_publisher.publish(nav)

    """ def follow_center_face(self, erro_x, erro_y):
        print('Center face')

        erro = Pose2D()
        dist_2 = erro_x**2 + erro_y**2
        dist = math.sqrt(dist_2)
        

        while dist >= 10:
            print('erro: ', abs(erro.x), abs(erro.y))
            cv2_img = self.br.imgmsg_to_cv2(self.image_color, "bgr8")
            #num_faces, center_face_x, center_face_y, center_x, center_y  = self.detect_face(cv2_img)
            #print('Variaveis função renata:', num_faces, center_face_x, center_face_y, center_x, center_y)
            #error_x = center_face_x - center_x
            #error_y = center_face_y - center_y
            #num_faces, face_x, face_y, shoulder, hip, center_x, center_y = self.found_landmarks(cv2_img)
            error_x = face_x - center_x
            error_y = face_y - center_y

            print('erro_atualizado =', int(error_x), int(error_y))
            dist_2 = error_x**2 + error_y**2
            dist = math.sqrt(dist_2)
            erro.x= float(error_x)
            erro.y= float(error_y)
            print("erro:", erro)
            self.neck_error_publisher.publish(erro)
            # print('dist 2:', dist_2)
            print('dist:', dist)
            #time.sleep(0.05)
            cv2.imshow("c_camera", cv2_img)
            cv2.waitKey(1)

                   

        print('centrei')
         """

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
        self.hand_raised = 0
        self.person_coordinates = Pose2D()
        self.person_coordinates.x = 0.0
        self.person_coordinates.y = 0.0

        self.neck_pose = Pose2D()
        self.neck_pose.x = 180.0
        self.neck_pose.y = 193.0

        self.target_x = 0.0
        self.target_y = 0.0

        self.pedido = ''

        self.i = 0
        
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False


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


    def wait_for_end_of_navigation(self):
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False
        print("Finished Navigation")

    def check_time(self):
        
        if self.node.ctr == 0:
            self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (0.0, 1.0), False, False)
        elif self.node.ctr == 1:
            self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (0.5, 1.0), False, False)
        elif self.node.ctr == 2:
            self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (0.0, 1.0), False, False)
        elif self.node.ctr == 3:
            self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (-0.5, 1.0), False, False)
            self.node.ctr = -1


        self.node.ctr += 1
        """ t1 = time.time()
        #print(t1 - self.node.t)
        if (t1-self.node.t) > self.node.tempo:
            self.node.counter += 1
            if self.node.counter == 1:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x + 1.0, self.node.robot_y + 1.0), False, False)
                print('1')
                self.node.tempo += 5.0
            
            elif self.node.counter == 2:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x + 1.0, self.node.robot_y + 0.0), False, False)
                print('2')
                self.node.tempo += 5.0

            elif self.node.counter == 3:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x + 1.0, self.node.robot_y - 1.0), False, False)
                self.node.tempo += 5.0

            elif self.node.counter == 4:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x + 0.0, self.node.robot_y - 1.0), False, False)
                self.node.tempo += 5.0

            elif self.node.counter == 5:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x - 1.0, self.node.robot_y - 1.0), False, False)
                self.node.tempo += 5.0

            elif self.node.counter == 6:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x - 1.0, self.node.robot_y + 0.0), False, False)
                self.node.tempo += 5.0

            elif self.node.counter == 7:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x - 1.0, self.node.robot_y + 1.0), False, False)
                self.node.tempo += 5.0

            elif self.node.counter == 8:
                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.robot_x + 0.0, self.node.robot_y + 1.0), False, False)
                self.node.tempo += 5.0
 """

            
        
            
        



    def check_for_hand_raising(self):
        #print('Hand raising function')
        yolo = self.node.yolo_poses
        if yolo.num_person > 0:
            print('persons detected =', yolo.num_person)
            for i in range(len(yolo.keypoints)):
                if yolo.keypoints[i].key_p9_y < yolo.keypoints[i].key_p0_y or yolo.keypoints[i].key_p10_y < yolo.keypoints[i].key_p0_y:
                    print('cumpri requisitos')
                   
                    
                    #print(self.hand_raised)

                    self.node.x_relative = yolo.keypoints[i].x_person_relative
                    self.node.y_relative = yolo.keypoints[i].average_distance
                    #self.hand_raised = 1

                    self.node.i+=1


                    #print('x = ', self.node.x_relative)
                    #print('y = ', self.node.y_relative)

                    #print(yolo.keypoints[i].key_p9_y, yolo.keypoints[i].key_p5_y)
                    #print(yolo.keypoints[i].key_p10_y, yolo.keypoints[i].key_p6_y)
                    
                    """ current_frame = self.node.br.imgmsg_to_cv2(self.node.image_color, "bgr8")
                    cv2.imshow("c_camera", current_frame)
                    cv2.waitKey(1) """

                else:
                    pass
                    #print('No hands up')
                    #self.hand_raised = 0
                    #print(self.hand_raised)

        else:
            #print('no persons detected')
            self.hand_raised = 0




    def main(self):

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

            if self.state == 0:
                
                # Print State
                self.node.rgb_ctr = 0
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                print('State 0 = Initial')

                # COLOCAR SPEAKER AQUI
                

                self.node.speech_str.command = "Hello! I am ready to start the restaurant task! Waiting for my start button to be pressed."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()


                self.node.rgb_ctr = 22
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                # START BUTTON
                t = Bool()
                t.data = True
                self.node.flag_start_button_publisher.publish(t)
                self.wait_for_start_button()
                
                
                self.node.rgb_ctr = 41
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                
                time.sleep(2)
                
                
                self.node.speech_str.command = "I am ready to attend new clients. Please raise your hands if you want me to serve you."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                time.sleep(0.5)

               

                #Next State
                self.state = 1 
                self.node.t = time.time()

            elif self.state == 1:

                # Print State
                #print('State 1 = Hand Raising Detect')

                #rais your hands

                # Hand Raising Algorithm
            
                self.check_for_hand_raising()

                #self.node.j += 1

                #self.check_time()

                """ if self.node.j > 80000:
                    self.check_time()
                    self.node.j = 0 """
                    

                #start_time = time.time()
                if self.node.counter == 1:
                    print('1')
                elif self.node.counter == 2:
                    print('2')

                if self.node.i > 10000:         
                    self.hand_raised == 1
                    
                    self.node.i = 0
                    print('Mão levantada')

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.node.speech_str.command = "I detected a costumer calling for me. Please wait for me."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.state = 2
                    self.node.aux = 1
                    self.person_coordinates.x = self.node.x_relative
                    self.person_coordinates.y = self.node.y_relative

                else:
                    pass
                    #print('mão em baixo')
        
            elif self.state == 2:

                # Print State
                print('State 2 = Navigation to Person')

                # Navigation

                """ self.node.customer_point.x = self.person_coordinates.x 
                self.node.customer_point.y = self.person_coordinates.y  """

                if self.node.aux == 1:
                    person_x = self.person_coordinates.x
                    person_y = self.person_coordinates.y
                    #print(person_x)
                    #print(person_y)
                    angle_person = atan2(person_x, person_y)
                    dist_person = sqrt(person_x**2 + person_y**2)
                    #print(self.node.robot_t, angle_person, angle_person - self.node.robot_t)
                    theta_aux = pi/2 - (angle_person - self.node.robot_theta)

                    self.target_x = dist_person * cos(theta_aux) + self.node.robot_x
                    self.target_y = dist_person * sin(theta_aux) + self.node.robot_y

                

                # target_x = person_x + self.node.robot_x
                # target_y = person_y + self.node.robot_y

                """ self.node.customer_point.x = self.person_coordinates.x * sin(self.node.robot_theta) - self.person_coordinates.y * cos(self.node.robot_theta)
                self.node.customer_point.y = self.person_coordinates.x * cos(self.node.robot_theta) + self.person_coordinates.y * sin(self.node.robot_theta)
                """
                print('cliente x e y: ', self.target_x, self.target_y)
                print('robot x e y: ', self.node.robot_x, self.node.robot_y)

                self.node.coordinates_to_navigation((self.target_x, self.target_y),(self.target_x, self.target_y), False, False)
                self.wait_for_end_of_navigation()


                # Look in the eyes of the person and ask is the person has called him,
                # se sim prosseguir, se não virar para um dos lados, encontrar otra pessoa e ir

                #if self.node.nose != self.node.int_error:

                #Next State
                if self.node.aux == 1:
                    self.state = 3
                elif self.node.aux == 2:
                    self.state = 6

            elif self.state == 3:

                # Print State
                print('State 3 = Receive Order - Speech')
                
                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                # Speech Receive Order

                # RECALIBRAR AUDIO  RECALIBRAR AUDIO 

                self.node.coordinates_to_navigation((self.target_x, self.target_y),(self.target_x, self.target_y), False, False)
                self.wait_for_end_of_navigation()

                time.sleep(1)
                calib = Bool()
                calib.data = True
                self.node.calibrate_ambient_noise_publisher.publish(calib)
                time.sleep(3) # obrigatorio depois de recalibrar audio


                self.node.speech_str.command = "Hello! I am ready to receive your order. Please take your request after the green lights under my wheels."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
        
                #Next State
                self.state = 4

            elif self.state == 4:

                # Print State
                print('State 4 = Receive Order - Listening and Confirm ')

                # Listening Audio

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.speech_type.restaurant = True
                
                #self.node.speech_type.yes_or_no = True
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()

                self.node.speech_type.restaurant = False
                #self.node.speech_type.yes_or_no = False
                self.pedido = self.node.keywords.data
                self.pedido.replace(" ", ".")
                keyword_list = self.node.keywords.data.split(" ")
                print(keyword_list)
                if len(keyword_list) == 1:
                    self.node.get_foods = 1
                    self.node.foods.append(keyword_list[0])
                elif len(keyword_list) == 2:
                    self.node.get_foods = 2
                    self.node.foods.append(keyword_list[0])
                    self.node.foods.append(keyword_list[1])
                elif len(keyword_list) == 3:
                    self.node.get_foods = 3
                    self.node.foods.append(keyword_list[0])
                    self.node.foods.append(keyword_list[1])
                    self.node.foods.append(keyword_list[2])

                self.node.speech_str.command =f"Does your order include the following items. {self.pedido}? Please answer with yes Robot or no Robot after the green light underneath my wheels."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                # Confirm Order

                """ if self.node.get_foods == 1:
                    self.node.speech_str.command =f"Does your order include {self.node.foods[0]}? Please answer with yes or no after the green light underneath my wheels."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                elif self.node.get_foods == 2:
                    self.node.speech_str.command =f"Does your order include {self.node.foods[0]} and {self.node.foods[1]}? Please answer with yes or no after the green light underneath my wheels"
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                elif self.node.get_foods == 3:
                    self.node.speech_str.command = f"Does your order include {self.node.foods[0]}, {self.node.foods[1]} and {self.node.foods[2]}? Please answer with yes or no after the green light underneath my wheels"
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                else:
                    # +perguntartiago ACRESCENTAR FALA DE COULD YOU PLEASE REPEAT
                    print("Error") """

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                
                # Listening Audio

                self.node.speech_type.yes_or_no = True
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()
                self.node.speech_type.yes_or_no = False

                # Receive Order Confirmation
                if self.node.keywords.data == 'yes':  # VERIFICAR COM O SPARK
                    self.state = 5
                    self.node.speech_str.command = "Thank you. I will get you your order."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                else:
                    self.node.speech_str.command = "I am sorry for the confusion. Can you please repeat the order after the green lights underneath my wheels?"
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    
                    self.state = 4
                

                ##### SUPOSIÇÃO PARA TESTE
                #self.state = 5

            elif self.state == 5:

                # Print State
                print('State 5 = Collect Order')
                
                #rotate 

                self.node.coordinates_to_navigation((self.node.robot_x ,self.node.robot_y), (self.node.robot_x+1.0 ,self.node.robot_y+1.0), False, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation((self.node.robot_x,self.node.robot_y), self.node.collect_point, False, False)
                self.wait_for_end_of_navigation()

                time.sleep(1)

                # Navigation to the Collect Point
                self.node.coordinates_to_navigation((0.0, 0.0), (0.0, 0.0), False, False)
                self.wait_for_end_of_navigation()


                time.sleep(1)
                calib = Bool()
                calib.data = True
                self.node.calibrate_ambient_noise_publisher.publish(calib)
                time.sleep(3) # obrigatorio depois de recalibrar audio


                # Collect Order
                #if self.node.get_foods == 1:
                self.node.speech_str.command =f"I want a {self.pedido}. As you can see, I forgot my arms today. Please place the products in the basket on my left shoulder. Please say yes Robot after my green lights whenever the request is ready."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                    # Listening Audio


                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.speech_type.yes_or_no = True
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()
                self.node.speech_type.yes_or_no = False

                self.node.speech_str.command = "Thank you."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.coordinates_to_navigation((self.node.robot_x ,self.node.robot_y), (self.node.robot_x+1.0 ,self.node.robot_y+1.0), False, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation((self.node.robot_x,self.node.robot_y), (self.target_x, self.target_y), False, False)
                self.wait_for_end_of_navigation()



                """     elif self.node.get_foods == 2:
                    self.node.speech_str.command =f"I want a {self.node.foods[0]} and {self.node.foods[1]}. As you can see, I forgot my arms today. Please place the products in my tray. Please say yes Charmie after my green lights whenever the request is ready."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()


                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.node.speech_type.yes_or_no = True
                    self.node.audio_command_publisher.publish(self.node.speech_type)
                    #self.node.start_audio()
                    self.wait_for_end_of_audio()
                    self.node.speech_type.yes_or_no = False

                elif self.node.get_foods == 3:
                    self.node.speech_str.command = f"I want a {self.node.foods[0]}, {self.node.foods[1]} and {self.node.foods[2]}. As you can see, I forgot my arms today. Please place the products in my tray. Please say yes Charmie after my green lights whenever the request is ready."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.node.speech_type.yes_or_no = True
                    self.node.audio_command_publisher.publish(self.node.speech_type)
                    #self.node.start_audio()
                    self.wait_for_end_of_audio()
                    self.node.speech_type.yes_or_no = False

                else:
                    print("Error") """

                

                self.node.aux = 2
                self.state = 2

            elif self.state == 6:

                # Print State
                print('State 6 = Final Speech')


                #Deliver Order
                self.node.speech_str.command = "Your order is here. As you can see, I forgot my arms today. Can you please collect the request from the basket on my left shoulder? Please say yes Robot after my green lights whenever the request is taken." 
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()                
                #self.state = 1

                self.node.speech_type.yes_or_no = True
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()
                self.node.speech_type.yes_or_no = False

                self.node.speech_str.command = "Hope you enjoy it. See you soon." 
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()  

                self.node.coordinates_to_navigation((self.node.robot_x ,self.node.robot_y), (self.node.robot_x+1.0 ,self.node.robot_y+1.0), False, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation((self.node.robot_x,self.node.robot_y), self.node.collect_point, False, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation(self.node.collect_point, self.node.collect_point, False, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation((self.node.robot_x,self.node.robot_y), (self.target_x, self.target_y), False, False)
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "I am ready to attend new clients. Please raise your hands if you want me to serve you."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                time.sleep(0.5)

                self.state = 1
     
            
            else:
                pass