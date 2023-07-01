#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from std_msgs.msg import Bool
# from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Obstacles, RobotSpeech, TarNavSDNL

after_door_point = (0.0, 1.7)
# aux1_point = (2.2, 1.7)
inspection_point = (1.0, 5.80 + 1.7)
sofas = (3.0, 5.8 + 1.7)

exit_first_point = (1.0, 5.80 + 1.7 + 1.0)
exit_point = (1.00, 5.80 + 1.7 + 2.0)

import time

class InspectionNode(Node):

    def __init__(self):
        super().__init__("Inspection")
        self.get_logger().info("Initialised CHARMIE Inspection Node")
        

        ###         PUBs/SUBs        
        
        # Door Start
        self.start_door_publisher = self.create_publisher(Bool, 'start_door', 10) 
        self.done_start_door_subscriber = self.create_subscription(Bool, 'done_start_door', self.done_start_door_callback, 10) 
        
        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        # Neck
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        
        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Low Level: Start Button
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        # Initial Pose for Localisation
        self.initial_pose_amcl_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10) # aux temp


        ###         Vars
        self.done_start_door = False
        self.flag_speech_done = False
        self.flag_navigation_done = False
        self.start_button_state = False
        self.speech_str = RobotSpeech()
        self.talk_neck = Pose2D()
        self.talk_neck.x = 180.0
        self.talk_neck.y = 150.0


    def done_start_door_callback(self, state:Bool):
        self.done_start_door = state.data
        print("Finished Door Start")

    def get_speech_done_callback(self, state: Bool):
        self.flag_speech_done = state.data
        print("Received Speech Flag:", state.data)

    def flag_pos_reached_callback(self, state: Bool):
        self.flag_navigation_done = state.data
        print("Received Navigation Flag:", state.data)

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)


    def publish_initial_pose(self, x:float, y:float):

        pose = PoseWithCovarianceStamped()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.pose.position.x = y
        pose.pose.pose.position.y = -x

        # it must start always facing the map axis (facing inside the house from the outise of the entrance door)
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0

        print("SENT INITIAL POSE")

        self.initial_pose_amcl_publisher.publish(pose)




# FLUXOGRAM
# START
# Wait for Door to Open
# Go to Position
# aVoid obstacles
# avoid human passing
# fala
# fica à espera que diga YES
# vai para a saida 


def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    th_main = threading.Thread(target=thread_main_inspection, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_inspection(node: InspectionNode):
    main = ReceptionistMain(node)
    main.main()


class ReceptionistMain():

    def __init__(self, node: InspectionNode):
        self.node = node
        self.state = 0
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False
    
    def wait_for_end_of_start_door(self):
        while not self.node.done_start_door:
            pass
        self.node.done_start_door = False

    def wait_for_end_of_navigation(self):
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False
        print("Finished Navigation")

    def wait_for_start_button(self):
        while not self.node.start_button_state:
            pass
        f = Bool()
        f.data = False 
        self.node.flag_start_button_publisher.publish(f)


    def coordinates_to_navigation(self, p1, p2, bool):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        self.node.target_position_publisher.publish(nav)
        print("Published Navigation")
        
    
    def main(self):
        print("IN NEW MAIN")
        time.sleep(1)

        while True:

            # State Machine

            if self.state == 0:
                
                print("0")

                self.node.publish_initial_pose(x=0.0, y=0.0)
                
                self.coordinates_to_navigation((0.0,4.0), (1.0, 5.0), False)
                self.wait_for_end_of_navigation()
                

                # Says it is ready to start its Inspection
                self.node.speech_str.command = "I am ready to start my Inspection."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                
                print("1")
                # wait for door
                door_start = Bool()
                door_start.data = True
                self.node.start_door_publisher.publish(door_start)
                self.wait_for_end_of_start_door()
                
                
                # Says: Oh thanks for opening the door. On my way to the Inspection Point
                self.node.speech_str.command = "Thank you for opening the door. On my way to the Inspection Point."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.state = 1

                #OLHAR PARA A POSIÇÃO DO GUEST
                self.node.neck_position_publisher.publish(self.node.talk_neck)


                #Navegação para o Ponto de Inspeção
                self.coordinates_to_navigation(after_door_point, inspection_point, True)
                self.wait_for_end_of_navigation()
                self.coordinates_to_navigation(inspection_point, sofas, False)
                self.wait_for_end_of_navigation()
                # self.coordinates_to_navigation(inspection_point, exit_first_point, False)
                # self.wait_for_end_of_navigation()


                self.node.speech_str.command = "Hello my name is charmie and I am very happy to make my debut on robocup at home. Hoppefully I am able to show some of my skills on the upcoming days. \
                I am ready to move on to the exit, when you are ready just stand in front of me and say Yes. Meanwhile I will wait here."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                # quando chegar fala: Hello my name is CHARMIE and I am very happy to make my debut on RoboCup@Home, Hoppefully I am able to show my skills on the upcoming days.
                # I am ready to move on to the exit, when you are ready just stand in front of me and say Yes. Meanwhile I will wait here.
                # self.node.speech_str.command = "I am ready to receive a new guest. Please stand in front of me."
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()
                t = Bool()
                t.data = True
                self.node.flag_start_button_publisher.publish(t)
                self.wait_for_start_button()


                self.coordinates_to_navigation(inspection_point, exit_first_point, True)
                self.wait_for_end_of_navigation()
                self.coordinates_to_navigation(exit_first_point, exit_point, False)
                self.wait_for_end_of_navigation()

                self.state = 1

                # wait pela audição
                # self.node.audio_command_publisher.publish(self.node.speech_type)
                # self.wait_for_end_of_audio()


                # Time to move to the exit, see you soon.
                #self.node.coordinates.move_target_coordinates = self.node.door_coordinates
                #self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                #self.node.coordinates.flag_not_obs = False
                #self.node.target_position_publisher.publish(self.node.coordinates)
                #self.wait_for_end_of_navigation()
                


            elif self.state == 1:
                pass