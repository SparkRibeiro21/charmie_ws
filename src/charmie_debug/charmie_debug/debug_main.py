#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Bool, Int16, String
from sensor_msgs.msg import LaserScan, Image
from charmie_interfaces.msg import Encoders, PS4Controller, RobotSpeech


from cv_bridge import CvBridge
import cv2 

# import time

class TRNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Node")
        
        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback, 10)
        self.flag_neck_position_publisher = self.create_publisher(Bool, "flag_neck_pos", 10)
        
        # Low Level Topics
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        self.vccs_subscriber = self.create_subscription(Pose2D, "get_vccs", self.get_vccs_callback, 10)
        self.flag_vccs_publisher = self.create_publisher(Bool, "flag_vccs", 10)
        self.torso_pos_publisher = self.create_publisher(Pose2D, "torso_pos", 10)
        self.get_torso_pos_subscriber = self.create_subscription(Pose2D, "get_torso_pos", self.get_torso_pos_callback, 10)
        self.flag_torso_pos_publisher = self.create_publisher(Bool, "flag_torso_pos", 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        self.get_encoders_subscriber = self.create_subscription(Encoders, "get_encoders", self.get_encoders_callback, 10)
        self.flag_encoders_publisher = self.create_publisher(Bool, "flag_encoders", 10)
        
        # PS4 Controller
        self.controller_subscriber = self.create_subscription(PS4Controller, "controller_state", self.get_controller_callback, 10)

        # LIDAR Hokuyo
        self.lidar_subscriber = self.create_subscription(LaserScan, "lidar_scan", self.get_lidar_callback, 10)

        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)


        # Timers
        self.counter = 1 # starts at 1 to avoid initial 
        self.create_timer(0.05, self.timer_callback)
        self.create_timer(5, self.timer_callback2)

        # Get Flags
        self.flag_get_neck_position = False 
        self.flag_get_start_button = False 
        self.flag_get_vccs = False 
        self.flag_get_torso = False 
        self.flag_get_encoders = False 

        # Get Variables
        self.ps4_controller = PS4Controller()
        self.controller_updated = False

        self.br = CvBridge()

    def get_neck_position_callback(self, pos: Pose2D):
        print("Received Neck Position: pan =", int(pos.x), " tilt = ", int(pos.y))

    def get_start_button_callback(self, state: Bool):
        print("Received Start Button: ", state.data)

    def get_vccs_callback(self, vcc: Pose2D):
        print("Received VCC: ", vcc.x, ", and Emergency: ", bool(vcc.y))

    def get_torso_pos_callback(self, torso_pos: Pose2D):
        print("Received Legs Angle: ", torso_pos.x, ", and Torso Angle: ", torso_pos.y)

    def get_encoders_callback(self, encoders: Encoders):
        print("Received Encoders: ", encoders.enc_m1, encoders.enc_m2, encoders.enc_m3, encoders.enc_m4)

    def get_controller_callback(self, controller: PS4Controller):
        print("TRIANGLE = ", controller.triangle, "CIRCLE = ", controller.circle, "CROSS = ", controller.cross, "SQUARE = ", controller.square)
        print("UP = ", controller.arrow_up, "RIGHT = ", controller.arrow_right, "DOWN = ", controller.arrow_down, "LEFT = ", controller.arrow_left)
        print("L1 = ", controller.l1, "R1 = ", controller.r1, "L3 = ", controller.l3, "R3 = ", controller.r3)
        print("SHA = ", controller.share, "OPT = ", controller.options, "PS = ", controller.ps)
        print("L2 = ", controller.l2, "R2 = ", controller.r2)
        print("L3_ang = ", controller.l3_ang, "L3_dis = ", controller.l3_dist, "L3_xx = ", controller.l3_xx, "L3_yy = ", controller.l3_yy)
        print("R3_ang = ", controller.r3_ang, "R3_dis = ", controller.r3_dist, "R3_xx = ", controller.r3_xx, "R3_yy = ", controller.r3_yy)
        self.ps4_controller = controller
        self.controller_updated = True

    def get_lidar_callback(self, scan: LaserScan):
        # print(scan)
        pass

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag: ", state.data)

    def get_color_image_callback(self, img: Image):
        # print(img)
        print("---")
        self.get_logger().info('Receiving color video frame')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        cv2.imshow("c_camera", current_frame)   
        cv2.waitKey(1)

    def get_depth_image_callback(self, img: Image):
        # print(img)
        print("---")
        self.get_logger().info('Receiving depth video frame')
        current_frame = self.br.imgmsg_to_cv2(img)
        cv2.imshow("d_camera", current_frame)   
        cv2.waitKey(1)


    def timer_callback2(self):
        speech_str = RobotSpeech()
        speech_str.command = "Hello my name is Tiago."
        # speech_str.language = 'en'
        # speech_str.command = "Bom dia, o meu nome é Tiago e gosto de Robôs. Já agora, qual é a comida na cantina hoje?"
        # speech_str.command = "Qual é a comida na cantina hoje?"
        speech_str.command = "Hello my name is Tiago."
        speech_str.language = 'en'
        self.speaker_publisher.publish(speech_str)


    def timer_callback(self):
        neck = Pose2D()
        flag_neck = Bool()
        flag_start_button = Bool()
        flag_vccs = Bool()
        flag_torso = Bool()
        rgb_mode = Int16()
        torso_pos = Pose2D()
        omni_move = Vector3()
        flag_encoders = Bool()

        if self.controller_updated:
            if self.ps4_controller.cross == 2: # RISING
                rgb_mode.data = 4
                self.rgb_mode_publisher.publish(rgb_mode)
            elif self.ps4_controller.cross == 1: # FALLING:
                rgb_mode.data = 2
                self.rgb_mode_publisher.publish(rgb_mode)

            if self.ps4_controller.l3_dist > 0.0:
                omni_move.x = self.ps4_controller.l3_ang
                omni_move.y = self.ps4_controller.l3_dist*100/5
            else:
                omni_move.x = 0.0
                omni_move.y = 0.0


            if self.ps4_controller.r3_dist > 0.0:
                omni_move.z = 100 + self.ps4_controller.r3_xx*10
            else:
                omni_move.z = 100.0
            
           
            self.omni_move_publisher.publish(omni_move)
            self.controller_updated = False
            
        """
        if self.counter == 0:
            neck.x = 180.0
            neck.y = 180.0 
            self.flag_get_neck_position = False
            # self.flag_get_start_button = False
            # self.flag_get_vccs = False
            self.flag_get_torso = False
            self.flag_get_encoders = False
            rgb_mode.data = 1
            self.rgb_mode_publisher.publish(rgb_mode)
            torso_pos.x = 66.0
            torso_pos.y = 85.0
            self.torso_pos_publisher.publish(torso_pos)
            omni_move.x = 0.0
            omni_move.y = 0.0
            omni_move.z = 100.0 - 10.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 1:
            neck.x = 270.0
            neck.y = 180.0 
            omni_move.x = 0.0
            omni_move.y = 0.0
            omni_move.z = 100.0 + 10.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 2:
            neck.x = 180.0
            neck.y = 180.0 
            self.flag_get_neck_position = True
            # self.flag_get_start_button = True
            # self.flag_get_vccs = True
            self.flag_get_torso = True
            self.flag_get_encoders = True
            rgb_mode.data = 2
            self.rgb_mode_publisher.publish(rgb_mode)
            torso_pos.x = 64.0
            torso_pos.y = 75.0
            self.torso_pos_publisher.publish(torso_pos)
            omni_move.x = 0.0
            omni_move.y = 20.0
            omni_move.z = 100.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 3:
            neck.x = 90.0
            neck.y = 180.0 
            omni_move.x = 180.0
            omni_move.y = 20.0
            omni_move.z = 100.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 4:
            neck.x = 180.0
            neck.y = 180.0 
            self.flag_get_neck_position = False
            # self.flag_get_start_button = False
            # self.flag_get_vccs = False
            self.flag_get_torso = False
            self.flag_get_encoders = False
            rgb_mode.data = 3
            self.rgb_mode_publisher.publish(rgb_mode)
            torso_pos.x = 66.0
            torso_pos.y = 85.0
            self.torso_pos_publisher.publish(torso_pos)
            omni_move.x = 90.0
            omni_move.y = 20.0
            omni_move.z = 100.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 5:
            neck.x = 180.0
            neck.y = 120.0 
            omni_move.x = 270.0
            omni_move.y = 20.0
            omni_move.z = 100.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 6:
            neck.x = 180.0
            neck.y = 180.0 
            self.flag_get_neck_position = True
            # self.flag_get_start_button = True
            # self.flag_get_vccs = True
            self.flag_get_torso = True
            self.flag_get_encoders = True
            rgb_mode.data = 4
            self.rgb_mode_publisher.publish(rgb_mode)
            torso_pos.x = 64.0
            torso_pos.y = 75.0
            self.torso_pos_publisher.publish(torso_pos)
            omni_move.x = 0.0
            omni_move.y = 20.0
            omni_move.z = 100.0 - 10.0
            self.omni_move_publisher.publish(omni_move)
        if self.counter == 7:
            neck.x = 180.0
            neck.y = 235.0 
            self.counter = -1
            omni_move.x = 0.0
            omni_move.y = 20.0
            omni_move.z = 100.0 + 10.0
            self.omni_move_publisher.publish(omni_move)

        self.neck_position_publisher.publish(neck)

        flag_neck.data = self.flag_get_neck_position
        self.flag_neck_position_publisher.publish(flag_neck)

        # flag_start_button.data = self.flag_get_start_button
        # self.flag_start_button_publisher.publish(flag_start_button)

        # flag_vccs.data = self.flag_get_vccs
        # self.flag_vccs_publisher.publish(flag_vccs)

        flag_torso.data = self.flag_get_torso
        self.flag_torso_pos_publisher.publish(flag_torso)

        flag_encoders.data = self.flag_get_encoders
        self.flag_encoders_publisher.publish(flag_encoders)

        print("DATA SENT ", self.counter)
        # self.counter+=1
        """

def main(args=None):
    rclpy.init(args=args)
    node = TRNode()

    cmd = Pose2D()
    cmd.x = 180.0
    cmd.y = 180.0
    node.neck_position_publisher.publish(cmd)
    print("INITIAL STATE")

    rclpy.spin(node)
    rclpy.shutdown()
