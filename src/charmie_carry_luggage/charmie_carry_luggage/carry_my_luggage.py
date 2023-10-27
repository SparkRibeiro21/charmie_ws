#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading

from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Int16, Bool, String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import RobotSpeech, SpeechType, TarNavSDNL, Yolov8Pose, NeckPosition

from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import math
import numpy as np
import time

door_out = (0.0, 0.0)
door_in = (0.0, 1.0)
aux_1 = (1.4, 1.9)
task_pos = (-2.0, 1.0)
ref_pos = (0.6, 9.05)
#--------------------------------

class CarryLuggageNode(Node):

    def __init__(self):
        super().__init__("CarryLuggage")
        self.get_logger().info("Initiliased Carry Luggage Node")

        # Neck Topics
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        # self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)

        # Low Level Topics
        # Low Level: RGB
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        #self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        #self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        #self.vccs_subscriber = self.create_subscription(Pose2D, "get_vccs", self.get_vccs_callback, 10)
        #self.flag_vccs_publisher = self.create_publisher(Bool, "flag_vccs", 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        #self.get_encoders_subscriber = self.create_subscription(Encoders, "get_encoders", self.get_encoders_callback, 10)
        #self.flag_encoders_publisher = self.create_publisher(Bool, "flag_encoders", 10)
    
        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Audio
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)
        #self.flag_listening_subscriber = self.create_subscription(Bool, "flag_listening", self.flag_listening_callback, 10)
        self.get_speech_subscriber = self.create_subscription(String, "get_speech", self.get_speech_callback, 10)

        # Odometry
        self.odometry_subscriber = self.create_subscription(Odometry, "odom", self.get_odometry_robot_callback, 10)

        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Intel Realsense
        self.camera_subscriber = self.create_subscription(Image, "/color/image_raw", self.color_img_callbcak, 10)
        self.depth_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_depth_callback, 10)

                # Low Level: Start Button
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        
        # Timer
        # self.create_timer(0.1, self.timer_callback)

        # Changing Variables
        self.state = 0

        self.start_button_state = False


        """ self.person_points = Yolov8Pose()
        self.person_position = Pose2D() """

        self.speech_str = RobotSpeech()
        self.speech_type = SpeechType()
        self.speech_type.yes_or_no = True
        self.get_speech = String()

        self.flag_speech_done = False
        self.flag_navigation_done = False
        self.flag_audio_done = False
        self.flag_stop_signal = False
        self.flag_arrived = False
        self.flag_bag_put = False

        self.bridge = CvBridge()
        self.image_color = Image()
        self.depth_img = Image()
        
        #NECK
        self.neck_pose = NeckPosition()
        self.neck_pose.pan = 180.0
        self.neck_pose.tilt = 180.0

        self.distance = 0.0
        self.robot_t = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        #RGB
        self.color = Int16()
        #NAV COORD DECLARATION
        self.coordinates = TarNavSDNL()
        self.point_arena = Pose2D()
        self.begin_coords = Pose2D()
        self.begin_orientation = Pose2D()
        self.start_task_coords = Pose2D()
        self.start_task_orientation = Pose2D()
        self.inter_coords = Pose2D()
        self.inter_orientation = Pose2D()
        #target = (pixel, dist)
        self.person_target = Pose2D()

        #audio
        self.keywords = String()
        self.left_right = []
        #--------------------------------  

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.8)

    def color_img_callbcak(self, img: Image):
        #print('camera callback')
        self.image_color = img

    def get_depth_callback(self, img: Image):
        #print('camera callback')
        self.depth_img = img
       
    def get_speech_callback(self, keywords : String):
        print("Received Audio:", keywords.data)
        self.keywords = keywords
        self.flag_audio_done = True

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
    
    def flag_pos_reached_callback(self, state: Bool):
        print("Received Navigation Flag:", state.data)
        self.flag_navigation_done = True
    
    def start_audio(self):
        self.audio_command_publisher.publish(self.speech_type)

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

    def get_odometry_robot_callback(self, odom:Odometry):
        self.robot_current_position = odom
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

    def choose_bag(self):
        self.get_logger().info('FUNCTION CHOOSE BAG')

        img = self.bridge.imgmsg_to_cv2(self.image_color, "bgr8")
        height, witdh, _ = img.shape
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)
        #print("RESULTS")
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2), self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=2))
            #LEFT_SIDE_IMG
            point_12 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y*height,2))
            point_16 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].y*height,2))
            point_24 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_HIP].y*height,2))
            
            point_11 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y*height,2))
            point_15 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].y*height,2))
            point_23 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP].y*height,2))
            
            #ANGLES
            theta_1 = calculate_3angle(point_12, point_16, point_24)
            theta_2 = calculate_3angle(point_11, point_15, point_23)

            angle_text1 = f"{theta_1:.1f}"
            angle_text2 = f"{theta_2:.1f}"
            cv2.putText(image, angle_text1, (int(point_12[0]+10), int(point_12[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.putText(image, angle_text2, (int(point_11[0]+10), int(point_11[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            if (theta_1 > 15 and theta_1 > theta_2) or point_15[0] < point_23[0]:
                #print("Pointing to My Left")
                #print('-------------------')
                #cv2.putText(image, "RIGHT", (15,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                self.left_right.append('right')
                self.state = 2
            elif (theta_2 > 15 and theta_1 < theta_2) or point_16[0] > point_24[0]:
                #print("Pointing to My Right")
                #print('--------------------')
                #cv2.putText(image, "LEFT", (15,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                self.left_right.append('left')
                self.state = 2

        cv2.imshow("CHOOSE BAG", image)
        cv2.waitKey(1)

        #return self.state
        
    def arm_raise(self):
        self.get_logger().info('FUNCTION ARM RAISE')

        img = self.bridge.imgmsg_to_cv2(self.image_color, "bgr8")
        height, witdh, _ = img.shape
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)
        #print("RESULTS")
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2), self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=2))
            #LEFT_SIDE_IMG
            point_0 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE].y*height,2))
            point_15 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].y*height,2))
            point_16 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].y*height,2))
            
            """ point_11 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y*height,2))
            point_15 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].y*height,2)) """
                        
            if (point_15 < 100 and point_15 < point_0) or (point_16 < 100 and point_16 < point_0):
                #print("ARM RAISE")
                #print('-------------------')
                cv2.putText(image, "ARM RAISE", (15,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                self.state = 7
                self.flag_stop_signal = True

        cv2.imshow("ARM RAISE", image)
        cv2.waitKey(1)

        return self.flag_stop_signal

    def follow_function(self):
        #NAVIGATION TO TARGET (PERSON)
        #self.get_logger().info('FUNCTION FOLLOW')
        img = self.bridge.imgmsg_to_cv2(self.image_color, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(self.depth_img, "32FC1")
        height, witdh, _ = img.shape
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        results = self.pose.process(image)
        #print("RESULTS")
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        dist_x = 0

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2), self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=2))
            
            point_12 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y*height,2))
            point_11 = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y*height,2))
            
            aux_x_pixel = (point_11[0] + point_12[0])/2
            aux_y_pixel = (point_11[1] + point_12[1])/2

            center_x = witdh/2
            dist_aux_x = center_x - aux_x_pixel
            dist_x = -((dist_aux_x * 0.01)/3.8)
            
            self.depth_img_array = np.array(depth_img, dtype=np.dtype('f8'))
            try:
                self.distance = (self.depth_img_array[int(aux_y_pixel), int(aux_x_pixel)])/1000
            except:
                self.distance = 0.0
                print("ERRO INDEX")

            dist = f"({dist_x:.2f},{self.distance:.2f})"
            #dist_text = f"{dist_x:.2f}"

            """ theta1 = math.tan(self.distance/dist_x)
            theta_degrees = math.degrees(theta1) """
            """print("THETA: ", theta_degrees)
            print("X: ", dist_x)
            print("Y: ", self.distance) """
            #angle = f"{theta_degrees:.2f}"

            #self.person_target.x = 

            cv2.line(image, (int(aux_x_pixel), int(aux_y_pixel)), (int(center_x), int(aux_y_pixel)), (0,0,255), 2)
            #cv2.putText(image, angle, (int(aux_x_pixel), int(aux_y_pixel+20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(image, dist, (int(aux_x_pixel), int(aux_y_pixel-10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            #aux_1 = (aux_x_pixel, aux_y_pixel)
            cv2.circle(image, (int(aux_x_pixel), int(aux_y_pixel)), 3, (255,0,0), 2)
            cv2.circle(depth_img, (int(aux_x_pixel), int(aux_y_pixel)), 3, (0,0,0), 2)

        else:
            dist_x = 0
            self.distance = 0

        cv2.imshow("FOLLOW TARGET", image)
        #cv2.imshow("Depth", depth_img)
        cv2.waitKey(1)

        return dist_x, self.distance


    def coordinates_to_navigation(self, p1, p2, bool_obs=False, bool_follow_me=False):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool_obs
        nav.follow_me = bool_follow_me
        print(nav.follow_me)
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        self.target_position_publisher.publish(nav)

def calculate_3angle(p1, p2, p3):
    vector_1 = (p2[0] - p1[0], p2[1] - p1[1])
    vector_2 = (p3[0] - p1[0], p3[1] - p1[1])

    dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
    magnitude_1 = math.sqrt(vector_1[0]**2 + vector_1[1]**2)
    magnitude_2 = math.sqrt(vector_2[0]**2 + vector_2[1]**2)

    theta = math.acos(dot_product / (magnitude_1 * magnitude_2))
    theta_degrees = math.degrees(theta)

    return theta_degrees

def main(args=None):
    rclpy.init(args=args)
    node = CarryLuggageNode()
    th_main = threading.Thread(target=thread_main_carry, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_carry(node: CarryLuggageNode):
    main = CarryLuggageMain(node)
    main.main()


class CarryLuggageMain():
    
    def __init__(self, node: CarryLuggageNode):
        self.node = node
        self.first_time_without_person = False
        self.first_time_person_close = False
        self.target_x = 0.0 
        self.target_y = 0.0 
        stop_orientation_x = 0.0
        stop_orientation_y = 0.0
        self.first_time_slow_down = True
        
        # self.detected_person_first_time = True
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False
    
    def wait_for_end_of_navigation(self):
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False

    def wait_for_end_of_audio(self):
        while not self.node.flag_audio_done:
            pass
        self.node.flag_audio_done = False
        self.node.flag_speech_done = False  
        # Since audio also uses speaker for errors

    def wait_for_start_button(self):
        while not self.node.start_button_state:
            pass
        f = Bool()
        f.data = False 
        self.node.flag_start_button_publisher.publish(f)


    def main(self):
        self.node.get_logger().info('NEW MAIN')

        #DEFINE NECK POSE FOR ALL THE TASK
        self.node.neck_position_publisher.publish(self.node.neck_pose)

        while True:

            #STATE 0 - NAV + READY
            if self.node.state == 0:
                self.node.get_logger().info('State 0')
                #READY FOR TASK
                self.node.speech_str.command = 'I am ready to start the Carry My Luggage Task.'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                
                # t = Bool()
                # t.data = True
                # self.node.flag_start_button_publisher.publish(t)
                # self.wait_for_start_button()
                # time.sleep(5)

                # self.node.color.data = 41
                # self.node.rgb_mode_publisher.publish(self.node.color.data) 
                
                self.node.state = 1

                
            #STATE 1 - CHOOSE BAG
            if self.node.state == 1:
                self.node.speech_str.command = 'Please point to the bag.'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.get_logger().info('State 1')
                try:
                    self.node.choose_bag()
                    #self.node.arm_raise()
                    #self.node.follow_function()
                except CvBridgeError:
                    pass #print('Erro')
            
            #STATE 2 - CHECK LEFT + ASK TO PUT
            if self.node.state == 2:
                self.node.get_logger().info('State 2')
                #CONFIRM BAG
                self.node.speech_str.command = f"Please answer me after the green light under my wheels." #RIGHT ROBOT
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()


                self.node.speech_str.command = f"Did you choose the bag on your {self.node.left_right[0]}. Please say yes or no and then say robot" #RIGHT ROBOT
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.color.data = 12
                self.node.rgb_mode_publisher.publish(self.node.color)
                

                #YES OR NO ANSWER
                self.node.audio_command_publisher.publish(self.node.speech_type)
                self.wait_for_end_of_audio()
                print(self.node.keywords.data)
                #IF YES - PLACE BAG
                if self.node.keywords.data == 'yes':    
                    self.node.speech_str.command = 'ok, please put the bag on the black hook on my left shoulder.'
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                    self.node.state = 4
                elif self.node.keywords.data == 'no':
                    self.node.speech_str.command = 'ah ok, sorry, pick up the other bag and put it on my hook.'
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                    self.node.state = 4

            #STATE 4 - CHECK BAG PLACE
            if self.node.state == 4:
                self.node.get_logger().info('State 4')
                #CONFIRM PLACE BAG
                time.sleep(3)
                self.node.speech_str.command = 'Did you place the bag on my hook? Please say yes or no and then say robot'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                #YES OR NO ANSWER
                self.node.audio_command_publisher.publish(self.node.speech_type)
                self.wait_for_end_of_audio()
                #IF YES - PLACE BAG
                if self.node.keywords.data == 'yes':    
                    self.node.flag_bag_put = True
                    self.node.state = 5
                

            #STATE 5 - READY TO FOLLOW + STOP SIGNAL
            if self.node.state == 5 and self.node.flag_bag_put:
                self.node.get_logger().info('State 5')
                self.node.speech_str.command = 'I am ready to follow you. When we arrive at our destination, please raise your arm. '
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                #self.node.speech_str.command = 'Second, I have RGB leds unders my wheels, if they are red you are too close and may move on, if they are green, I am able to follow you, If they are magenta you are too far away. Please look at me while you are walking and consider my RGB lights. '
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()
                self.node.state = 6

            

            #STATE 6 - FOLLOW ME
            if self.node.state == 6:
                try:
                    #self.node.choose_bag()
                    #self.node.arm_raise()
                    person_x, person_y = self.node.follow_function()
                    if person_y > 0:
                        #print(person_x)
                        #print(person_y)
                        angle_person = math.atan2(person_x, person_y)
                        dist_person = math.sqrt(person_x**2 + person_y**2)
                        #print(self.node.robot_t, angle_person, angle_person - self.node.robot_t)
                        theta_aux = math.pi/2 - (angle_person - self.node.robot_t)

                        target_x = dist_person * math.cos(theta_aux) + self.node.robot_x
                        target_y = dist_person * math.sin(theta_aux) + self.node.robot_y

                        # target_x = person_x + self.node.robot_x
                        # target_y = person_y + self.node.robot_y

                        h_target_inter = 1.5
                        dist_max_person = 4.5
                        # target_inter_x = (dist_person - h_target_inter) * math.sin(angle_person)
                        # target_inter_y = (dist_person - h_target_inter) * math.cos(angle_person)

                        # angle_target_inter = math.atan2(target_inter_x, target_inter_y)
                        # dist_target_inter = math.sqrt(target_inter_x**2 + target_inter_y**2)
                        # print(self.node.robot_t, angle_target_inter, angle_target_inter - self.node.robot_t)
                        # theta_inter_aux = math.pi/2 - (angle_target_inter - self.node.robot_t)

                        # tar_int_x = dist_target_inter * math.cos(theta_inter_aux) + self.node.robot_x
                        # tar_int_y = dist_target_inter * math.sin(theta_inter_aux) + self.node.robot_y

                        stop_orientation_x = h_target_inter*2 * math.sin(-self.node.robot_t) + self.node.robot_x
                        stop_orientation_y = h_target_inter*2 * math.cos(-self.node.robot_t) + self.node.robot_y


                        ### for stability reasons dist_person must be replaced by person_y
                        #COLOCAR CORES NOS ESTADOS S1(green) S2(blue) S3(red)
                        #NORMAL
                        if person_y > h_target_inter and person_y < dist_max_person: 
                            self.node.color.data = 11 #Fix Green
                            self.node.rgb_mode_publisher.publish(self.node.color)
                            self.node.coordinates_to_navigation((target_x, target_y), (stop_orientation_x, stop_orientation_y), False, True)
                            print('ESTADO 1')
                            self.first_time_person_close = False
                            self.first_time_slow_down = True

                        #PERSON TOO FAR - ASK TO GET CLOSER
                        elif person_y > dist_max_person: 
                            self.node.color.data = 41 #Fix Kinda Pink
                            self.node.rgb_mode_publisher.publish(self.node.color)
                            print('ESTADO 2')
                            self.first_time_person_close = False

                            print(self.first_time_slow_down)

                            if self.first_time_slow_down:
                                self.node.speech_str.command = 'Please come back and stand in front of me.'
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                # time.sleep(1)
                                self.first_time_slow_down = False

                            self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (target_x, target_y), False, True)
                            # self.node.coordinates_to_navigation((target_x, target_y), (stop_orientation_x, stop_orientation_y), False, True)


                            # self.wait_for_end_of_speaking()
                            
                            # self.node.coordinates_to_navigation((target_x, target_y), (stop_orientation_x, stop_orientation_y), False, True)
                            # self.wait_for_end_of_navigation()

                            # self.node.speech_str.command = 'Place yourself in front of me less than a meter away.'
                            # self.node.speaker_publisher.publish(self.node.speech_str)
                            # self.wait_for_end_of_speaking()

                            # time.sleep(5)
                            

                        #PERSON TOO CLOSE - dist < h_target    
                        else:   
                            self.node.color.data = 1 #Fix Red
                            self.node.rgb_mode_publisher.publish(self.node.color)
                            stop_signal = self.node.arm_raise()
                            if stop_signal == True:
                                self.node.state = 7

                            print('ESTADO 3')
                            if not self.first_time_person_close:
                                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (target_x, target_y), False, True)
                                self.first_time_person_close = True


                        
                        #self.node.coordinates_to_navigation((target_x, target_y), task_pos, False)
                        # if dist_person < h_target_inter:

                        #     self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (stop_orientation_x, stop_orientation_y), False, True)
                        # else:
                        #     self.node.coordinates_to_navigation((tar_int_x, tar_int_y), (target_x, target_y), False, True)
                        
                        # self.node.coordinates_to_navigation((person_x, person_y), task_pos, False)
                        #self.wait_for_end_of_navigation()
                        self.first_time_without_person = True

                    #NO PERSON
                    else:    
                        if self.first_time_without_person:
                            # self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), task_pos, False, False)
                            self.node.coordinates_to_navigation((target_x, target_y), (stop_orientation_x, stop_orientation_y), False, False)
                            print('ESTADO 4')

                        self.first_time_without_person = False


                except CvBridgeError:
                    pass #print('Erro')
                
            #STATE 7 - CHECK DESTINATION
            if self.node.state == 7 and self.node.flag_stop_signal:
                self.node.get_logger().info('State 7')
                #CONFIRM ARRIVED
                self.node.speech_str.command = 'Have we arrived yet? Please say yes or no and then say robot'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                #YES OR NO ANSWER
                self.node.audio_command_publisher.publish(self.node.speech_type)
                self.wait_for_end_of_audio()
                #IF YES RECEIVED
                if self.node.keywords.data == 'yes':    
                    self.node.flag_arrived = True
                    self.node.state = 8
                
            #STATE 8 - ASK TO TAKE BAG OUT
            if self.node.state == 8 and self.node.flag_arrived:
                self.node.get_logger().info('State 6')
                #CONFIRM ARRIVED
                self.node.speech_str.command = 'Please take the bag out.'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.state = 9
            
            #STATE 9 - CHECK BAG OUT
            if self.node.state == 9:
                self.node.get_logger().info('State 9')
                #CONFIRM BAG OUT
                self.node.speech_str.command = 'Did you take out the bag of my board? Please say yes or no and then say robot'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                #YES OR NO ANSWER
                self.node.audio_command_publisher.publish(self.node.speech_type)
                self.wait_for_end_of_audio()
                #IF YES RECEIVED
                if self.node.keywords.data == 'yes':   
                    self.node.flag_bag_put = False
                    self.node.state = 10

            #STATE 10 - GO BACK ARENA
            if self.node.state == 10 and not self.node.flag_bag_put:
                self.node.get_logger().info('State 8')
                self.node.speech_str.command = 'Now I will go back to the arena.'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.coordinates_to_navigation((0.0, 2.0), (0.0, 0.0), False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation((0.0, 0.0), (0.0, 1.0), False)
                self.wait_for_end_of_navigation()
                #WHEN INSIDE - take position from odom
                self.node.state = 11

            #STATE 11 - INSIDE ARENA + FINISH TASK
            if self.node.state == 11:
                self.node.get_logger().info('State 11')
                #CONFIRM ARRIVED
                self.node.speech_str.command = 'I arrived at the arena. My task is over.'
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.color.data = 2 #Fix Blue
                self.node.rgb_mode_publisher.publish(self.node.color)
                #FINISH TASK