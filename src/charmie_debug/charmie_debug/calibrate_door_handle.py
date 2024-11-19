#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from sensor_msgs.msg import Image
from charmie_interfaces.srv import SetNeckPosition

import cv2 
import threading
import time
from cv_bridge import CvBridge






# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

DOOR_HANDLE_POSITION_TOP_LEFT = (751, 466)
DOOR_HANDLE_POSITION_BOTTOM_RIGHT = (887, 512)

class ServeBreakfastNode(Node):

    def __init__(self):
        super().__init__("ServeBreakfast")
        self.get_logger().info("Initialised CHARMIE ServeBreakfast Node")

        ### Topics (Publisher and Subscribers) ###   
        # Intel Realsense Subscribers
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        
        ### Services (Clients) ###)
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        
        # Variables 
        self.waited_for_end_of_neck_pos = False
       
        self.br = CvBridge()
        self.head_image = Image()
        
        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.neck_success = True
        self.neck_message = ""
        
    def get_color_image_head_callback(self, image: Image):
        self.head_image = image

        current_frame = self.br.imgmsg_to_cv2(self.head_image, "bgr8")
        current_frame_draw = current_frame.copy()

        cv2.rectangle(current_frame_draw, DOOR_HANDLE_POSITION_TOP_LEFT, DOOR_HANDLE_POSITION_BOTTOM_RIGHT, (0,0,255), 4)
        
        cv2.imshow("Door Handle Calibration", current_frame_draw)
        cv2.waitKey(10)
    
    ### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, position=[0, 0], wait_for_end_of=True):
        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])
        
        future = self.set_neck_position_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.neck_success = response.success
            self.neck_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ServeBreakfastNode()
    th_main = threading.Thread(target=ThreadMainServeBreakfast, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainServeBreakfast(node: ServeBreakfastNode):
    main = ServeBreakfastMain(node)
    main.main()

class ServeBreakfastMain():

    def __init__(self, node: ServeBreakfastNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node
    
    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message

    # main state-machine function
    def main(self):
        
        # States in ServeBreakfast Task
        self.Calibrate_door_handle_position = 0
        self.Final_State = 1
        
        # Neck Positions
        self.look_door_handle = [-45, -20]
        
        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Calibrate_door_handle_position

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Calibrate Door Handle Position Main...")

        while True:

            if self.state == self.Calibrate_door_handle_position:
                print("State:", self.state, "- Waiting_for_task_start")

                self.set_neck(position=self.look_door_handle, wait_for_end_of=False)

                # next state
                self.state = self.Final_State 
                
            elif self.state == self.Final_State:
                
                # Lock after finishing task
                while True:
                    pass

            else:
                pass
