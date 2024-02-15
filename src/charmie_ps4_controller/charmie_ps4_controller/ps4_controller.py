#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from charmie_interfaces.msg import PS4Controller, NeckPosition
from charmie_interfaces.srv import SpeechCommand
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Bool, Int16

import math
import numpy as np

from pyPS4Controller.controller import Controller
import threading

NUM_BUTTONS = 15

# ARROW_UP = 0
# ARROW_RIGHT = 1
# ARROW_DOWN = 2
# ARROW_LEFT = 3
# TRIANGLE = 4
# CIRCLE = 5
# CROSS = 6
# SQUARE = 7
# L1 = 8
# R1 = 9
# L3 = 10
# R3 = 11
# OPTIONS = 12
# SHARE = 13
# PS = 14

# analogs
# LR2

rgb_demonstration = [100, 0, 11, 22, 33, 44, 55, 66, 77, 88, 99, 100, 101, 102, 103, 104, 105, 106, 255]


CONTROL_TORSO = True
CONTROL_WAIT_FOR_END_OF_NAVIGATION = True
CONTROL_MOTORS = True
CONTROL_RGB = True
CONTROL_SPEAKERS = True
CONTROL_NECK = True
CONTROL_ARM = True

pow15 = 32767


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

        self.buttons = np.zeros(NUM_BUTTONS, dtype=np.int8)
        self.buttons_ant = np.zeros(NUM_BUTTONS, dtype=np.int8)

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

        # Esta logica não está ao contrário????????? so o LOw e o HIGH do falling e rising
        # 0 LOW  -> LOW  = OFF
        # 1 LOW  -> HIGH = FALLING
        # 2 HIGH -> LOW  = RISING
        # 3 HIGH -> HIGH = ON

        # for x in range(NUM_BUTTONS):
        #     # print(x)
        #     self.buttons[x] = x
        #     x+=1

        # self.buttons[PS] = 0

    def on_up_arrow_press(self):
        # print("on_up_arrow_press")
        # self.every_button_update()
        self.buttons[self.ARROW_UP] = 1
        # self.buttons_updated = True
        self.values_updated = True

    def on_down_arrow_press(self):
        # print("on_down_arrow_press")
        # self.every_button_update()
        self.buttons[self.ARROW_DOWN] = 1
        # self.buttons_updated = True
        self.values_updated = True

    def on_up_down_arrow_release(self):
        # print("on_up_down_arrow_release")
        # self.every_button_update()
        self.buttons[self.ARROW_UP] = 0
        self.buttons[self.ARROW_DOWN] = 0
        # self.buttons_updated = True
        self.values_updated = True

    def on_left_arrow_press(self):
        # print("on_left_arrow_press")
        # self.every_button_update()
        self.buttons[self.ARROW_LEFT] = 1
        # self.buttons_updated = True
        self.values_updated = True

    def on_right_arrow_press(self):
        # print("on_right_arrow_press")
        # self.every_button_update()
        self.buttons[self.ARROW_RIGHT] = 1
        # self.buttons_updated = True
        self.values_updated = True

    def on_left_right_arrow_release(self):
        # print("on_left_right_arrow_release")
        # self.every_button_update()
        self.buttons[self.ARROW_LEFT] = 0
        self.buttons[self.ARROW_RIGHT] = 0
        # self.buttons_updated = True
        self.values_updated = True

    def on_x_press(self):
        # self.every_button_update()
        # print("Hello world")
        self.buttons[self.CROSS] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # pass

    def on_x_release(self):
        # self.every_button_update()
        # print("Goodbye world")
        self.buttons[self.CROSS] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # self.a = False
        # h.set_var_false()
        # print(controller.a)

    def on_triangle_press(self):
        # self.every_button_update()
        self.buttons[self.TRIANGLE] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_triangle_press")

    def on_triangle_release(self):
        # self.every_button_update()
        self.buttons[self.TRIANGLE] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_triangle_release")

    def on_circle_press(self):
        # self.every_button_update()
        self.buttons[self.CIRCLE] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_circle_press")

    def on_circle_release(self):
        # self.every_button_update()
        self.buttons[self.CIRCLE] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_circle_release")

    def on_square_press(self):
        # self.every_button_update()
        self.buttons[self.SQUARE] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_square_press")

    def on_square_release(self):
        # self.every_button_update()
        self.buttons[self.SQUARE] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_square_release")

    def on_L1_press(self):
        # self.every_button_update()
        self.buttons[self.L1] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_L1_press")

    def on_L1_release(self):
        # self.every_button_update()
        self.buttons[self.L1] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_L1_release")

    def on_R1_press(self):
        # self.every_button_update()
        self.buttons[self.R1] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_R1_press")

    def on_R1_release(self):
        # self.every_button_update()
        self.buttons[self.R1] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_R1_release")

    def on_L3_press(self):
        # self.every_button_update()
        self.buttons[self.L3] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_L3_press")

    def on_L3_release(self):
        # self.every_button_update()
        self.buttons[self.L3] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_L3_release")

    def on_R3_press(self):
        # self.every_button_update()
        self.buttons[self.R3] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_R3_press")

    def on_R3_release(self):
        # self.every_button_update()
        self.buttons[self.R3] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("on_R3_release")

    def on_options_press(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.OPTIONS] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_options_press")

    def on_options_release(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.OPTIONS] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_options_release")

    def on_share_press(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.SHARE] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_share_press")

    def on_share_release(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.SHARE] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_share_release")

    def on_playstation_button_press(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.PS] = 1
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_playstation_button_press")

    def on_playstation_button_release(self):
        # self.buttons_ant = self.buttons
        # self.every_button_update()
        self.buttons[self.PS] = 0
        # self.buttons_updated = True
        self.values_updated = True
        # print("XXX BUTTONS =", self.buttons, "\t\t", "BUTTONS_ANT =", self.buttons_ant)
        # print("on_playstation_button_release")

    def every_button_update(self):

        # np.copyto(self.buttons_ant, self.buttons)
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
        self.L2dist = ((value+pow15) / (2*pow15))
        self.L2R2_updated = True
        self.values_updated = True
            
        # print("TR: L2: {}".format(value), ",\t{:.2f}%".format(self.L2dist*100), ",\t{}".format(self.L2dist), ",\t{:.2f}%".format(self.L2dist_ant*100), ",\t{}".format(self.L2dist_ant))
        # int("TR: L2_ANT: {}".format(value))
        # print("L2: {}".format(value), ",\t{:.2f}".format(self.L2dist), ",\t{:.2f}".format(self.L2dist_ant))

        # if int(self.L2dist*100) is not int(self.L2dist_ant*100):  # NAO E NECESSARIO, POIS  NAO???
        #     self.L2R2_updated = True
            # print("DIFFERENT")
        #else:
        #    pass
            # print("SAME")

    def on_L2_release(self):
        self.L2dist = 0.0
        # print("TR: L2_release", ",\t{}".format(self.L2dist))
        # self.L2R2_updated = True
        self.values_updated = True

    def on_R2_press(self, value):
        self.R2dist = ((value+pow15) / (2*pow15))
        # print("TR: R2: {}".format(value), ",\t{:.2f}%".format(self.R2dist*100), ",\t{}".format(self.R2dist))
        # self.L2R2_updated = True
        self.values_updated = True

    def on_R2_release(self):
        self.R2dist = 0.0
        # print("TR: R2_release", ",\t{}".format(self.R2dist))
        # self.L2R2_updated = True
        self.values_updated = True

    def on_L3_up(self, value):
        self.L3yy = -(value / pow15)
        # print("TR: L3_up: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.L3yy))
        self.update_L3()

    def on_L3_down(self, value):
        self.L3yy = -(value / pow15)
        # print("TR: L3_down: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.L3yy))
        self.update_L3()

    def on_L3_left(self, value):
        self.L3xx = (value / pow15)
        # print("TR: L3_left: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.L3xx))
        self.update_L3()

    def on_L3_right(self, value):
        self.L3xx = (value / pow15)
        # print("TR: L3_right: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.L3xx))
        self.update_L3()

    def on_L3_y_at_rest(self):
        # L3 joystick is at rest after the joystick was moved and let go off # up and down
        self.L3yy = 0.0
        # print("TR: L3_y_at_rest", ",\t{}".format(self.L3yy))
        # print("REST YY")
        self.update_L3()

    def on_L3_x_at_rest(self):
        # L3 joystick is at rest after the joystick was moved and let go off  # left and right
        self.L3xx = 0.0
        # print("TR: L3_x_at_rest", ",\t{}".format(self.L3xx))
        # print("REST XX")
        self.update_L3()

    def update_L3(self):
        # self.every_button_update() # added in the ROS version
        # contas: para angulo, é a tangente yy/xx para dist é a hipotenusa
        # self.L3ang = math.atan2(self.L3yy, self.L3xx)/math.pi*180
        # with this the angles are the same as the trigonometric circle
        self.L3ang = math.atan2(-self.L3xx, self.L3yy)/math.pi*180+0  # 0 -> front # the +0 is to ignore negative zero
        self.L3dist = math.sqrt(self.L3yy**2 + self.L3xx**2)
        if self.L3dist > 1.0:
            self.L3dist = 1.0
        if self.L3ang < 0.0:
            self.L3ang += 360
        # print("TR: ang: {:.2f}".format(self.L3ang), "\tdist: {:.2f}".format(self.L3dist))
        # self.analogs_updated = True
        self.values_updated = True

    def on_R3_up(self, value):
        self.R3yy = -(value / pow15)
        # print("TR: R3_up: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.R3yy))
        self.update_R3()

    def on_R3_down(self, value):
        self.R3yy = -(value / pow15)
        # print("TR: R3_down: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.R3yy))
        self.update_R3()

    def on_R3_left(self, value):
        self.R3xx = (value / pow15)
        # print("TR: R3_left: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.R3xx))
        self.update_R3()

    def on_R3_right(self, value):
        self.R3xx = (value / pow15)
        # print("TR: R3_right: {}".format(value), ",\t{:.2f}%".format(abs(value/pow15)*100), ",\t{}".format(self.R3xx))
        self.update_R3()

    def on_R3_y_at_rest(self):
        # R3 joystick is at rest after the joystick was moved and let go off  # up and down
        self.R3yy = 0.0
        # print("TR: R3_y_at_rest", ",\t{}".format(self.R3yy))
        self.update_R3()

    def on_R3_x_at_rest(self):
        # R3 joystick is at rest after the joystick was moved and let go off  # left and right
        self.R3xx = 0.0
        # print("TR: R3_x_at_rest", ",t{}".format(self.R3xx))
        self.update_R3()

    def update_R3(self):
        # contas: para angulo, é a tangente yy/xx para dist é a hipotenusa
        # self.R3ang = math.atan2(self.R3yy, self.R3xx)/math.pi*180
        # with this the angles are the same as the trigonometric circle
        self.R3ang = math.atan2(-self.R3xx, self.R3yy)/math.pi*180+0  # 0 -> front # the +0 is to ignore negative zero
        self.R3dist = math.sqrt(self.R3yy**2 + self.R3xx**2)
        if self.R3dist > 1.0:
            self.R3dist = 1.0
        if self.R3ang < 0.0:
            self.R3ang += 360
        # print("TR: ang: {:.2f}".format(self.R3ang), "\tdist: {:.2f}".format(self.R3dist), "X=", self.R3xx, "Y=", self.R3yy)
        # self.analogs_updated = True
        self.values_updated = True
        

class ControllerNode(Node):

    def __init__(self):
        super().__init__("PS4_Controller")
        self.get_logger().info("Initialised CHARMIE PS4 Controller Node")

        self.controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

        self.controller_publisher = self.create_publisher(PS4Controller, "controller_state", 10)
        self.ps4_diagnostic_publisher = self.create_publisher(Bool, "ps4_diagnostic", 10)
        self.torso_test_publisher = self.create_publisher(Pose2D, "torso_test" , 10)
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_pos_reached", 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.client = self.create_client(SpeechCommand, "speech_command")# Neck
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        

        self.create_timer(0.05, self.timer_callback)

        self.rgb_demo_index = 0
        self.waited_for_end_of_speaking = False # not used, but here to be in conformity with other uses

        flag_diagn = Bool()
        flag_diagn.data = True
        self.ps4_diagnostic_publisher.publish(flag_diagn)

        self.neck_pos = NeckPosition()
        self.neck_pos.pan = 180.0
        self.neck_pos.tilt = 180.0
        self.neck_position_publisher.publish(self.neck_pos)
    

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


    def speech_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.waited_for_end_of_speaking:
            pass
        self.waited_for_end_of_speaking = False

        
    def timer_callback(self):
        ps_con = PS4Controller()
        if self.controller.values_updated == True:

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

            if ps_con.l3_dist < 0.1:
                ps_con.l3_ang = 0.0

            if ps_con.r3_dist < 0.1:
                ps_con.r3_ang = 0.0

            print(ps_con.arrow_up, ps_con.arrow_right, ps_con.arrow_down, ps_con.arrow_left, "|",
                  ps_con.triangle, ps_con.circle, ps_con.cross, ps_con.square, "|",
                  ps_con.l1, ps_con.r1, round(ps_con.l2, 1), round(ps_con.r2, 1), ps_con.l3, ps_con.r3, "|",
                  ps_con.share, ps_con.ps, ps_con.options, "|",
                  str(round(ps_con.l3_ang)).rjust(3), round(ps_con.l3_dist, 1), str(round(ps_con.l3_xx, 1)).rjust(4), str(round(ps_con.l3_yy, 1)).rjust(4), "|", 
                  str(round(ps_con.r3_ang)).rjust(3), round(ps_con.r3_dist, 1), str(round(ps_con.r3_xx, 1)).rjust(4), str(round(ps_con.r3_yy, 1)).rjust(4)
                  )

            self.controller_publisher.publish(ps_con)
            
            # control code
            self.control_robot(ps_con)

            # gets values ready for next iteration
            self.controller.every_button_update()
            self.controller.values_updated = False
    

    def control_robot(self, ps4_controller):

        if CONTROL_TORSO:
            pos = Pose2D()
            if ps4_controller.arrow_up >= 2:
                pos.x = float(1)
                # print("LEGS UP")
            elif ps4_controller.arrow_down >= 2:
                pos.x = float(-1)
                # print("LEGS DOWN")
            else:
                pos.x = float(0)
                # print("LEGS STOP")

            if ps4_controller.arrow_right >= 2:
                pos.y = float(1)
                # print("TORSO UP")
            elif ps4_controller.arrow_left >= 2:
                pos.y = float(-1)
                # print("TORSO DOWN")
            else:
                pos.y = float(0)
                # print("TORSO STOP")

            self.torso_test_publisher.publish(pos)


        if CONTROL_WAIT_FOR_END_OF_NAVIGATION:
            pos = Bool()
            pos.data = True
            if ps4_controller.options:
                self.flag_pos_reached_publisher.publish(pos)
            # print("NAVIGATION DONE")


        if CONTROL_MOTORS:
            omni_move = Vector3()
            if ps4_controller.l3_dist >= 0.1:
                omni_move.x = ps4_controller.l3_ang
                omni_move.y = ps4_controller.l3_dist*100/5
            else:
                omni_move.x = 0.0
                omni_move.y = 0.0

            if ps4_controller.r3_dist >= 0.1:
                omni_move.z = 100 + ps4_controller.r3_xx*10
            else:
                omni_move.z = 100.0
                       
            self.omni_move_publisher.publish(omni_move)

            
        if CONTROL_RGB:
            rgb_mode = Int16()

            if ps4_controller.r1 == 2:
                self.rgb_demo_index+=1
                if self.rgb_demo_index >= len(rgb_demonstration):
                    self.rgb_demo_index-=len(rgb_demonstration)

                print(self.rgb_demo_index)
                rgb_mode.data = rgb_demonstration[self.rgb_demo_index]
                self.rgb_mode_publisher.publish(rgb_mode)
            
            elif ps4_controller.l1 == 2:
                self.rgb_demo_index-=1
                if self.rgb_demo_index < 0:
                    self.rgb_demo_index+=len(rgb_demonstration)

                print(self.rgb_demo_index)
                rgb_mode.data = rgb_demonstration[self.rgb_demo_index]
                self.rgb_mode_publisher.publish(rgb_mode)

        if CONTROL_SPEAKERS:
            if ps4_controller.r3 == 2:
                self.speech_server(filename="introduction_full", wait_for_end_of=False)
            elif ps4_controller.l3 == 2:
                self.speech_server(filename="receptionist_question", wait_for_end_of=False)
                

        if CONTROL_NECK:
            pass
            
            neck_inc = 5.0
            if ps4_controller.circle >= 2:
                self.neck_pos.pan -= neck_inc
                if self.neck_pos.pan < 0.0:
                    self.neck_pos.pan = 0.0
                self.neck_position_publisher.publish(self.neck_pos)
                print(self.neck_pos)

            elif ps4_controller.square >= 2:
                self.neck_pos.pan += neck_inc
                if self.neck_pos.pan > 359.0:
                    self.neck_pos.pan = 359.0
                self.neck_position_publisher.publish(self.neck_pos)
                print(self.neck_pos)

            if ps4_controller.cross >= 2:
                self.neck_pos.tilt -= neck_inc
                if self.neck_pos.tilt < 120.0:
                    self.neck_pos.tilt = 120.0
                self.neck_position_publisher.publish(self.neck_pos)
                print(self.neck_pos)

            elif ps4_controller.triangle >= 2:
                self.neck_pos.tilt += neck_inc
                if self.neck_pos.tilt > 235.0:
                    self.neck_pos.tilt = 235.0
                self.neck_position_publisher.publish(self.neck_pos)
                print(self.neck_pos)
            

        if CONTROL_ARM:
            pass


        # DONE - motores movimentacao    
        # DONE - torso
        # DONE - wait for end of navigation
        # DONE - rgb
        # DONE - speakers
        #      - neck
        #      - arm
    

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
