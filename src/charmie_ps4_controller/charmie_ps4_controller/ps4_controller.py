#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from charmie_interfaces.msg import PS4Controller, NeckPosition
from charmie_interfaces.srv import SpeechCommand
from geometry_msgs.msg import Pose2D, Vector3
from example_interfaces.msg import Bool, Int16, String

import math
import numpy as np

from pyPS4Controller.controller import Controller
import threading


# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

# rgb leds used for demonstration, can be added any other necessary for demonstration
rgb_demonstration = [100, 0, 13, 24, 35, 46, 57, 68, 79, 100, 101, 102, 103, 104, 105, 106, 255]

# rgb leds used for demonstration, can be added any other necessary for demonstration
face_demonstration = ["demo1", "demo2", "demo3", "demo4", "demo5", "demo6", "demo7", "demo8", "demo9"]

# Controller Class, what communicates with the physical controller
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        self.analogs_updated = False
        self.buttons_updated = False
        self.L2R2_updated = False
        self.values_updated = False

        self.L3ang = 0.0
        self.L3dist = 0.0
        self.L3yy = 0.0
        self.L3xx = 0.0

        self.R3ang = 0.0
        self.R3dist = 0.0
        self.R3yy = 0.0
        self.R3xx = 0.0

        self.L2dist = 0.0
        self.R2dist = 0.0
        self.L2dist_ant = 0.0
        self.R2dist_ant = 0.0

        NUM_BUTTONS = 15
        self.buttons = np.zeros(NUM_BUTTONS, dtype=np.int8)
        self.buttons_ant = np.zeros(NUM_BUTTONS, dtype=np.int8)

        self.pow15 = 32767   

        self.ARROW_UP = 0
        self.ARROW_RIGHT = 1
        self.ARROW_DOWN = 2
        self.ARROW_LEFT = 3
        self.TRIANGLE = 4
        self.CIRCLE = 5
        self.CROSS = 6
        self.SQUARE = 7
        self.L1 = 8
        self.R1 = 9
        self.L3 = 10
        self.R3 = 11
        self.SHARE = 12
        self.OPTIONS = 13
        self.PS = 14

        self.OFF = 0
        self.FALLING = 1
        self.RISING = 2
        self.ON = 3

        # just to help ease code reading, necessary for multiple buttons pressed at the same time commands
        self.RISING_OR_ON = 2  # to help 'if' commands that must consider both values: if x >=  RISING_OR_ON:
        self.FALLING_OR_OFF = 1  # to help 'if' commands that must consider both values: if x <=  FALLING_OR_OFF:

        # 0 LOW  -> LOW  = OFF
        # 1 HIGH -> LOW  = FALLING
        # 2 LOW  -> HIGH = RISING
        # 3 HIGH -> HIGH = ON


    def on_up_arrow_press(self):
        self.buttons[self.ARROW_UP] = 1
        self.values_updated = True

    def on_down_arrow_press(self):
        self.buttons[self.ARROW_DOWN] = 1
        self.values_updated = True

    def on_up_down_arrow_release(self):
        self.buttons[self.ARROW_UP] = 0
        self.buttons[self.ARROW_DOWN] = 0
        self.values_updated = True

    def on_left_arrow_press(self):
        self.buttons[self.ARROW_LEFT] = 1
        self.values_updated = True

    def on_right_arrow_press(self):
        self.buttons[self.ARROW_RIGHT] = 1
        self.values_updated = True

    def on_left_right_arrow_release(self):
        self.buttons[self.ARROW_LEFT] = 0
        self.buttons[self.ARROW_RIGHT] = 0
        self.values_updated = True

    def on_x_press(self):
        self.buttons[self.CROSS] = 1
        self.values_updated = True

    def on_x_release(self):
        self.buttons[self.CROSS] = 0
        self.values_updated = True

    def on_triangle_press(self):
        self.buttons[self.TRIANGLE] = 1
        self.values_updated = True

    def on_triangle_release(self):
        self.buttons[self.TRIANGLE] = 0
        self.values_updated = True

    def on_circle_press(self):
        self.buttons[self.CIRCLE] = 1
        self.values_updated = True

    def on_circle_release(self):
        self.buttons[self.CIRCLE] = 0
        self.values_updated = True

    def on_square_press(self):
        self.buttons[self.SQUARE] = 1
        self.values_updated = True

    def on_square_release(self):
        self.buttons[self.SQUARE] = 0
        self.values_updated = True

    def on_L1_press(self):
        self.buttons[self.L1] = 1
        self.values_updated = True

    def on_L1_release(self):
        self.buttons[self.L1] = 0
        self.values_updated = True

    def on_R1_press(self):
        self.buttons[self.R1] = 1
        self.values_updated = True

    def on_R1_release(self):
        self.buttons[self.R1] = 0
        self.values_updated = True

    def on_L3_press(self):
        self.buttons[self.L3] = 1
        self.values_updated = True

    def on_L3_release(self):
        self.buttons[self.L3] = 0
        self.values_updated = True

    def on_R3_press(self):
        self.buttons[self.R3] = 1
        self.values_updated = True

    def on_R3_release(self):
        self.buttons[self.R3] = 0
        self.values_updated = True

    def on_options_press(self):
        self.buttons[self.OPTIONS] = 1
        self.values_updated = True

    def on_options_release(self):
        self.buttons[self.OPTIONS] = 0
        self.values_updated = True

    def on_share_press(self):
        self.buttons[self.SHARE] = 1
        self.values_updated = True

    def on_share_release(self):
        self.buttons[self.SHARE] = 0
        self.values_updated = True

    def on_playstation_button_press(self):
        self.buttons[self.PS] = 1
        self.values_updated = True

    def on_playstation_button_release(self):
        self.buttons[self.PS] = 0
        self.values_updated = True

    def every_button_update(self):

        self.buttons_ant = np.array(self.buttons)

        # what made sense was for this flag to be activated only after the buttons attribution on the event function
        # however, probably something having to do with the threading process, when multiple buttons are pressed or
        # released, the buttons_ant array is not correctly updated. Only when the flag is before the button attribution
        # is works as should
        # maybe if a future bug appears this may be the cause

        # self.buttons_ant = self.buttons
        # pass  # print(self.buttons_ant)

    def button_state(self, value):
        return self.buttons[value]*2 + self.buttons_ant[value]
        # 0 LOW  -> LOW  = OFF
        # 1 LOW  -> HIGH = FALLING
        # 2 HIGH -> LOW  = RISING
        # 3 HIGH -> HIGH = ON

    def on_L2_press(self, value):
        self.L2dist_ant = self.L2dist
        self.L2dist = ((value+self.pow15) / (2*self.pow15))
        self.L2R2_updated = True
        self.values_updated = True

    def on_L2_release(self):
        self.L2dist = 0.0
        self.values_updated = True

    def on_R2_press(self, value):
        self.R2dist = ((value+self.pow15) / (2*self.pow15))
        self.values_updated = True

    def on_R2_release(self):
        self.R2dist = 0.0
        self.values_updated = True

    def on_L3_up(self, value):
        self.L3yy = -(value / self.pow15)
        self.update_L3()

    def on_L3_down(self, value):
        self.L3yy = -(value / self.pow15)
        self.update_L3()

    def on_L3_left(self, value):
        self.L3xx = (value / self.pow15)
        self.update_L3()

    def on_L3_right(self, value):
        self.L3xx = (value / self.pow15)
        self.update_L3()

    def on_L3_y_at_rest(self):
        # L3 joystick is at rest after the joystick was moved and let go off # up and down
        self.L3yy = 0.0
        self.update_L3()

    def on_L3_x_at_rest(self):
        # L3 joystick is at rest after the joystick was moved and let go off  # left and right
        self.L3xx = 0.0
        self.update_L3()

    def update_L3(self):
        # with this the angles are the same as the trigonometric circle
        self.L3ang = math.atan2(-self.L3xx, self.L3yy)/math.pi*180+0  # 0 -> front # the +0 is to ignore negative zero
        self.L3dist = math.sqrt(self.L3yy**2 + self.L3xx**2)
        if self.L3dist > 1.0:
            self.L3dist = 1.0
        if self.L3ang < 0.0:
            self.L3ang += 360
        self.values_updated = True

    def on_R3_up(self, value):
        self.R3yy = -(value / self.pow15)
        self.update_R3()

    def on_R3_down(self, value):
        self.R3yy = -(value / self.pow15)
        self.update_R3()

    def on_R3_left(self, value):
        self.R3xx = (value / self.pow15)
        self.update_R3()

    def on_R3_right(self, value):
        self.R3xx = (value / self.pow15)
        self.update_R3()

    def on_R3_y_at_rest(self):
        # R3 joystick is at rest after the joystick was moved and let go off  # up and down
        self.R3yy = 0.0
        self.update_R3()

    def on_R3_x_at_rest(self):
        # R3 joystick is at rest after the joystick was moved and let go off  # left and right
        self.R3xx = 0.0
        self.update_R3()

    def update_R3(self):
        # with this the angles are the same as the trigonometric circle
        self.R3ang = math.atan2(-self.R3xx, self.R3yy)/math.pi*180+0  # 0 -> front # the +0 is to ignore negative zero
        self.R3dist = math.sqrt(self.R3yy**2 + self.R3xx**2)
        if self.R3dist > 1.0:
            self.R3dist = 1.0
        if self.R3ang < 0.0:
            self.R3ang += 360
        self.values_updated = True
        

class ControllerNode(Node):

    def __init__(self):
        super().__init__("PS4_Controller")
        self.get_logger().info("Initialised CHARMIE PS4 Controller Node")

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("control_arm", True) 
        self.declare_parameter("control_face", True)
        self.declare_parameter("control_motors", True)
        self.declare_parameter("control_neck", True) 
        self.declare_parameter("control_rgb", True) 
        self.declare_parameter("control_set_movement", True) 
        self.declare_parameter("control_speakers", True)
        self.declare_parameter("control_torso", True)
        self.declare_parameter("control_wait_for_end_of_navigation", True)

        # Create Controller object
        self.controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.torso_test_publisher = self.create_publisher(Pose2D, "torso_test" , 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.set_movement_publisher = self.create_publisher(Bool, "set_movement", 10)

        # Navigation
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_pos_reached", 10)

        # PS4 Controller
        self.controller_publisher = self.create_publisher(PS4Controller, "controller_state", 10)

        # Disgnostic
        self.ps4_diagnostic_publisher = self.create_publisher(Bool, "ps4_diagnostic", 10)

        # Neck
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)

        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        
        # Arm 
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # CONTROL VARIABLES, this is what defines which modules will the ps4 controller control
        self.CONTROL_ARM = self.get_parameter("control_arm").value
        self.CONTROL_FACE = self.get_parameter("control_face").value
        self.CONTROL_MOTORS = self.get_parameter("control_motors").value
        self.CONTROL_NECK = self.get_parameter("control_neck").value
        self.CONTROL_RGB = self.get_parameter("control_rgb").value
        self.CONTROL_SET_MOVEMENT = self.get_parameter("control_set_movement").value
        self.CONTROL_SPEAKERS = self.get_parameter("control_speakers").value
        self.CONTROL_TORSO = self.get_parameter("control_torso").value
        self.CONTROL_WAIT_FOR_END_OF_NAVIGATION = self.get_parameter("control_wait_for_end_of_navigation").value

        # timer that checks the controller every 50 ms 
        self.create_timer(0.05, self.timer_callback)

        self.rgb_demo_index = 0
        self.face_demo_index = 0
        self.waited_for_end_of_speaking = False # not used, but here to be in conformity with other uses
        self.waited_for_end_of_arm = False # not used, but here to be in conformity with other uses
        self.arm_ready = True

        # Sucess and Message confirmations for all set_(something) CHARMIE functions
        self.speech_sucess = True
        self.speech_message = ""
        self.arm_sucess = True
        self.arm_message = ""

        self.wfeon = Bool()
        self.torso_pos = Pose2D()
        self.select_movement = String()

        # initial commands send to different nodes  
        flag_diagn = Bool()
        flag_diagn.data = True
        self.ps4_diagnostic_publisher.publish(flag_diagn)

        if self.CONTROL_SET_MOVEMENT:
            self.set_movement = Bool()
            self.set_movement.data = True
            self.set_movement_publisher.publish(self.set_movement)

        if self.CONTROL_MOTORS:
            self.omni_move = Vector3()
            self.omni_move.x = 0.0
            self.omni_move.y = 0.0
            self.omni_move.z = 100.0
            self.omni_move_publisher.publish(self.omni_move)

        if self.CONTROL_RGB:
            self.rgb_mode = Int16()
            self.rgb_mode.data = RAINBOW_ROT
            self.rgb_mode_publisher.publish(self.rgb_mode)

        if self.CONTROL_NECK:
            self.neck_pos = NeckPosition()
            self.neck_pos.pan = 180.0
            self.neck_pos.tilt = 180.0
            self.neck_position_publisher.publish(self.neck_pos)

        if self.CONTROL_FACE:
            self.face_mode = String()
            self.face_mode.data = "demo5"
            self.image_to_face_publisher.publish(self.face_mode)

        self.watchdog_timer = 0
        self.watchdog_flag = False
    
    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        self.arm_ready = True
        self.waited_for_end_of_arm = True
        self.arm_sucess = flag.data
        if flag.data:
            self.arm_message = "Arm sucessfully moved"
        else:
            self.arm_message = "Wrong Movement Received"
        
    ### SPEECH SERVICE ###
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
    
        future = self.speech_command_client.call_async(request)
        #print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_sucess = True
            self.speech_message = "Wait for answer not needed"



    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucess = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



    ### SET FUNCTIONS ###

    # function to be called in tasks to send commands to speakers
    def set_speech(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.waited_for_end_of_speaking:
            pass
        self.waited_for_end_of_speaking = False

        return self.speech_sucess, self.speech_message

    # function to be called in tasks to send commands to arm
    def set_arm(self, command="", wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.waited_for_end_of_arm = False
        
        temp = String()
        temp.data = command
        self.arm_command_publisher.publish(temp)

        if wait_for_end_of:
            while not self.waited_for_end_of_arm:
                pass
            self.waited_for_end_of_arm = False
        else:
            self.arm_sucess = True
            self.arm_message = "Wait for answer not needed"


        self.get_logger().info("Set Arm Response: %s" %(str(self.arm_sucess) + " - " + str(self.arm_message)))

        return self.arm_sucess, self.arm_message


    def timer_callback(self):

        # create ps4 controller object
        ps_con = PS4Controller()

        if self.controller.values_updated == True:
            self.watchdog_timer = 0            
            self.watchdog_flag = False

            # attribute controller data to ps4 controller object   
            ps_con.arrow_up = int(self.controller.button_state(self.controller.ARROW_UP))
            ps_con.arrow_right = int(self.controller.button_state(self.controller.ARROW_RIGHT))
            ps_con.arrow_down = int(self.controller.button_state(self.controller.ARROW_DOWN))
            ps_con.arrow_left = int(self.controller.button_state(self.controller.ARROW_LEFT))

            ps_con.triangle = int(self.controller.button_state(self.controller.TRIANGLE))
            ps_con.circle = int(self.controller.button_state(self.controller.CIRCLE))
            ps_con.cross = int(self.controller.button_state(self.controller.CROSS))
            ps_con.square = int(self.controller.button_state(self.controller.SQUARE))

            ps_con.l1 = int(self.controller.button_state(self.controller.L1))
            ps_con.r1 = int(self.controller.button_state(self.controller.R1))
            ps_con.l3 = int(self.controller.button_state(self.controller.L3))
            ps_con.r3 = int(self.controller.button_state(self.controller.R3))

            ps_con.share = int(self.controller.button_state(self.controller.SHARE))
            ps_con.options = int(self.controller.button_state(self.controller.OPTIONS))
            ps_con.ps = int(self.controller.button_state(self.controller.PS))

            ps_con.l2 = float(self.controller.L2dist)
            ps_con.r2 = float(self.controller.R2dist)

            ps_con.l3_ang = float(self.controller.L3ang)
            ps_con.l3_dist = float(self.controller.L3dist)
            ps_con.l3_xx = float(self.controller.L3xx)
            ps_con.l3_yy = float(self.controller.L3yy)

            ps_con.r3_ang = float(self.controller.R3ang)
            ps_con.r3_dist = float(self.controller.R3dist)
            ps_con.r3_xx = float(self.controller.R3xx)
            ps_con.r3_yy = float(self.controller.R3yy)

            # prevents small noises in joy sticks
            if ps_con.l3_dist < 0.1:
                ps_con.l3_ang = 0.0

            if ps_con.r3_dist < 0.1:
                ps_con.r3_ang = 0.0
            
            # overall debug print
            print("\n", ps_con.arrow_up, ps_con.arrow_right, ps_con.arrow_down, ps_con.arrow_left, "|",
                  ps_con.triangle, ps_con.circle, ps_con.cross, ps_con.square, "|",
                  ps_con.l1, ps_con.r1, round(ps_con.l2, 1), round(ps_con.r2, 1), ps_con.l3, ps_con.r3, "|",
                  ps_con.share, ps_con.ps, ps_con.options, "|",
                  str(round(ps_con.l3_ang)).rjust(3), round(ps_con.l3_dist, 1), str(round(ps_con.l3_xx, 1)).rjust(4), str(round(ps_con.l3_yy, 1)).rjust(4), "|", 
                  str(round(ps_con.r3_ang)).rjust(3), round(ps_con.r3_dist, 1), str(round(ps_con.r3_xx, 1)).rjust(4), str(round(ps_con.r3_yy, 1)).rjust(4), "|",
                  end='')

            # publishes ps4 controller object so if any other needs it, can use controller data  
            self.controller_publisher.publish(ps_con)
            
            # control code to send commands to other nodes if CONTROL variables are set to true (ros2 params)
            self.control_robot(ps_con)

            # gets values ready for next iteration
            self.controller.every_button_update()
            self.controller.values_updated = False

        else:
            if not self.watchdog_flag:
                self.watchdog_timer += 1


        if self.watchdog_timer == 40: # since the ps4 controller checks every 50 ms. 20*50ms is 1 second. 40 is 2 seconds.
            # this is set if in any kind of emergency the controller stops communicating. 
            # If the system continues with the last received variables it may result in physical damages.
            # Therefore, to every moving part that may continue moving is set stop commands
            self.watchdog_flag = True
            self.watchdog_timer += 1

            print("WATCHDOG BLOCK - NO COMMUNICATIONS IN THE LAST 2 SECONDS")

            # only does this if not in locked motors mode, this prevents always sending the following commands continuously even when it is already locked (mainly stops speaking a lot of times)
            if self.set_movement.data == True:
                    
                # locks motors
                if self.CONTROL_SET_MOVEMENT:
                    self.set_movement.data = False
                    self.set_movement_publisher.publish(self.set_movement)

                # sends command to stop torso
                if self.CONTROL_TORSO:
                    self.torso_pos.x = 0.0
                    self.torso_pos.y = 0.0
                    self.torso_test_publisher.publish(self.torso_pos)

                # changes to a red value to notify it entered in motors locked mode
                if self.CONTROL_RGB:
                    self.rgb_mode.data = RED+HALF_ROTATE
                    self.rgb_mode_publisher.publish(self.rgb_mode)

                # in case it is not intended for the robot to speak since it may disturb other packages
                if self.CONTROL_SPEAKERS:
                    self.set_speech(filename="motors_locked", wait_for_end_of=False)

        # print(self.watchdog_timer, end='')
        print(".", end='')


    # control code to send commands to other nodes if CONTROL variables are set to true (ros2 params)
    def control_robot(self, ps4_controller):

        if self.CONTROL_TORSO:
            if ps4_controller.arrow_up >= 2:
                self.torso_pos.x = 1.0
            elif ps4_controller.arrow_down >= 2:
                self.torso_pos.x = -1.0
            else:
                self.torso_pos.x = 0.0

            if ps4_controller.arrow_right >= 2:
                self.torso_pos.y = 1.0
            elif ps4_controller.arrow_left >= 2:
                self.torso_pos.y = -1.0
            else:
                self.torso_pos.y = 0.0

            self.torso_test_publisher.publish(self.torso_pos)

        if self.CONTROL_WAIT_FOR_END_OF_NAVIGATION:
            self.wfeon.data = True
            if ps4_controller.options:
                self.flag_pos_reached_publisher.publish(self.wfeon)

        if self.CONTROL_MOTORS:
            # left joy stick to control x and y movement (direction and linear speed) 
            if ps4_controller.l3_dist >= 0.1:
                self.omni_move.x = ps4_controller.l3_ang
                self.omni_move.y = ps4_controller.l3_dist*100/5
            else:
                self.omni_move.x = 0.0
                self.omni_move.y = 0.0

            # right joy stick to control angular speed
            if ps4_controller.r3_dist >= 0.1:
                self.omni_move.z = 100 + ps4_controller.r3_xx*10
            else:
                self.omni_move.z = 100.0
                       
            self.omni_move_publisher.publish(self.omni_move)

        if self.CONTROL_RGB:
            # only does this if not in locked motors mode, otherwise it will change the rgb and being in this mode might be unoticed
            if self.set_movement.data == True:
                if ps4_controller.r1 == 2:
                    self.rgb_demo_index+=1
                    if self.rgb_demo_index >= len(rgb_demonstration):
                        self.rgb_demo_index-=len(rgb_demonstration)

                    # print(self.rgb_demo_index)
                    self.rgb_mode.data = rgb_demonstration[self.rgb_demo_index]
                    self.rgb_mode_publisher.publish(self.rgb_mode)

        if self.CONTROL_SPEAKERS:
            # examples of two different speech commands
            if ps4_controller.r3 == 2:
                success, message = self.set_speech(filename="introduction_full", wait_for_end_of=False)
                print(success, message)
            elif ps4_controller.l3 == 2:
                success, message = self.set_speech(filename="receptionist_question", wait_for_end_of=False)
                print(success, message)
                
        if self.CONTROL_NECK:
            # circle and square to move neck left and right
            # triangle and cross to move the neck up and down
            neck_inc = 5.0
            if ps4_controller.circle >= 2:
                self.neck_pos.pan -= neck_inc
                if self.neck_pos.pan < 0.0:
                    self.neck_pos.pan = 0.0
                self.neck_position_publisher.publish(self.neck_pos)
                # print(self.neck_pos)

            elif ps4_controller.square >= 2:
                self.neck_pos.pan += neck_inc
                if self.neck_pos.pan > 359.0:
                    self.neck_pos.pan = 359.0
                self.neck_position_publisher.publish(self.neck_pos)
                # print(self.neck_pos)

            if ps4_controller.cross >= 2:
                self.neck_pos.tilt -= neck_inc
                if self.neck_pos.tilt < 120.0:
                    self.neck_pos.tilt = 120.0
                self.neck_position_publisher.publish(self.neck_pos)
                # print(self.neck_pos)

            elif ps4_controller.triangle >= 2:
                self.neck_pos.tilt += neck_inc
                if self.neck_pos.tilt > 235.0:
                    self.neck_pos.tilt = 235.0
                self.neck_position_publisher.publish(self.neck_pos)
                # print(self.neck_pos)

        if self.CONTROL_SET_MOVEMENT:
            if ps4_controller.ps == 2:
                if self.set_movement.data == False:
                    
                    # stops locked motor mode, motors can now run
                    self.set_movement.data = True
                    self.set_movement_publisher.publish(self.set_movement)

                    # returns to the value it was previously
                    if self.CONTROL_RGB:
                        self.rgb_mode.data = rgb_demonstration[self.rgb_demo_index]
                        self.rgb_mode_publisher.publish(self.rgb_mode)

                    # in case it is not intended for the robot to speak since it may disturb other packages
                    if self.CONTROL_SPEAKERS:
                        self.set_speech(filename="motors_unlocked", wait_for_end_of=False)

                else:

                    # locks motors
                    self.set_movement.data = False
                    self.set_movement_publisher.publish(self.set_movement)

                    # sends command to stop torso
                    if self.CONTROL_RGB:
                        self.torso_pos.x = 0.0
                        self.torso_pos.y = 0.0
                        self.torso_test_publisher.publish(self.torso_pos)
                
                    # changes to a red value to notify it entered in motors locked mode    
                    if self.CONTROL_RGB:
                        self.rgb_mode.data = RED+HALF_ROTATE
                        self.rgb_mode_publisher.publish(self.rgb_mode)

                    # in case it is not intended for the robot to speak since it may disturb other packages
                    if self.CONTROL_SPEAKERS:
                        self.set_speech(filename="motors_locked", wait_for_end_of=False)

        if self.CONTROL_FACE:
            if ps4_controller.l1 == 2:
                self.face_demo_index+=1
                if self.face_demo_index >= len(face_demonstration):
                    self.face_demo_index-=len(face_demonstration)

                # print(self.rgb_demo_index)
                self.face_mode.data = face_demonstration[self.face_demo_index]
                self.image_to_face_publisher.publish(self.face_mode)

        if self.CONTROL_ARM:
            if self.arm_ready: 
                if ps4_controller.share == 2:
                    self.arm_ready = False
                    # Command to say hello 
                    success, message = self.set_arm(command="debug_initial", wait_for_end_of=False)
                    print(success, message)


def thread_controller(node):
    node.controller.listen()

def main(args=None):

    rclpy.init(args=args)
    node = ControllerNode()

    # crete thread to listen to the controller since it blocks the liten function
    th_con = threading.Thread(target=thread_controller, args=(node,), daemon=True)
    th_con.start()

    rclpy.spin(node)
    rclpy.shutdown()
