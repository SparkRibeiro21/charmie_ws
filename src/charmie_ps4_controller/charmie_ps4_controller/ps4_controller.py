#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import threading
import numpy as np

from charmie_interfaces.msg import PS4Controller
from pyPS4Controller.controller import Controller

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

        self.controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.ps4_controller_publisher = self.create_publisher(PS4Controller, "controller_state", 10) # used only for ps4 controller
        
        self.previous_time = 0.0

        self.create_timer(0.025, self.check_controller_to_publish_data)


    def check_controller_to_publish_data(self):
        
        if self.controller.values_updated == True:

            elapsed_time = time.perf_counter() - self.previous_time
            self.previous_time = time.perf_counter()

            ps_con = PS4Controller()

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
                str(round(elapsed_time,3)), end='')

            # publishes ps4 controller object so if any other needs it, can use controller data  
            self.ps4_controller_publisher.publish(ps_con)
            
            # gets values ready for next iteration
            self.controller.every_button_update()
            self.controller.values_updated = False


def thread_controller(node):
    node.controller.listen()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    # create thread to listen to the controller
    th_con = threading.Thread(target=thread_controller, args=(node,), daemon=True)
    th_con.start()
    rclpy.spin(node)
    rclpy.shutdown()
