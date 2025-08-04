#!/usr/bin/env python3
import rclpy
import threading
import time
import numpy as np
import cv2
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              False,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      False,
    "charmie_base_camera":      False,
    "charmie_gamepad":          False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_lidar_livox":      False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_radar":            False,
    "charmie_speakers":         False,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     False,
    "charmie_yolo_pose":        False,
}

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ROS2TaskNode(ros2_modules)
    robot = RobotStdFunctions(node)
    th_main = threading.Thread(target=ThreadMainTask, args=(robot,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainTask(robot: RobotStdFunctions):
    main = TaskMain(robot)
    main.main()

class TaskMain():

    def __init__(self, robot: RobotStdFunctions):
        # create a robot instance so use all standard CHARMIE functions
        self.robot = robot

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        self.state = Waiting_for_start_button

        print("IN NEW MAIN")

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
                # print('State 0 = Initial')

                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                s = False
                while not s:
                    s, cam = self.robot.get_head_depth_image()
                    if not s:
                        time.sleep(0.1)
                
                overall = self.check_person_depth_head(current_frame_depth_head=cam) #half_image_zero_or_near_percentage=0.4, full_image_near_percentage=0.1, near_max_dist=800)
                      
                if overall: 
                    # print("STOP", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("STOP")
                else:
                    # print("GO", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("GO")
                        
            elif self.state == Final_State:
                print("Finished task!!!")

                while True:
                    pass

            else:
                pass


    def check_person_depth_head(self, current_frame_depth_head, half_image_zero_or_near_percentage=0.6, full_image_near_percentage=0.3, near_max_dist=800):

        overall = False
        DEBUG = True

        height, width = current_frame_depth_head.shape
        current_frame_depth_head_half = current_frame_depth_head[height//2:height,:]
        
        # FOR THE FULL IMAGE

        tot_pixeis = height*width 
        mask_zero = (current_frame_depth_head == 0)
        mask_near = (current_frame_depth_head > 0) & (current_frame_depth_head <= near_max_dist)
        
        if DEBUG:
            mask_remaining = (current_frame_depth_head > near_max_dist) # just for debug
            blank_image = np.zeros((height,width,3), np.uint8)
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

        pixel_count_zeros = np.count_nonzero(mask_zero)
        pixel_count_near = np.count_nonzero(mask_near)

        # FOR THE BOTTOM HALF OF THE IMAGE

        mask_zero_half = (current_frame_depth_head_half == 0)
        mask_near_half = (current_frame_depth_head_half > 0) & (current_frame_depth_head_half <= near_max_dist)
        
        if DEBUG:
            mask_remaining_half = (current_frame_depth_head_half > near_max_dist) # just for debug
            blank_image_half = np.zeros((height//2,width,3), np.uint8)
            blank_image_half[mask_zero_half] = [255,255,255]
            blank_image_half[mask_near_half] = [255,0,0]
            blank_image_half[mask_remaining_half] = [0,0,255]
                
        pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
        pixel_count_near_half = np.count_nonzero(mask_near_half)
        
        if DEBUG:
            cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
            cv2.imshow("New Img Distance Inspection", blank_image)
            cv2.waitKey(10)

        half_image_zero_or_near = False
        half_image_zero_or_near_err = 0.0
        
        full_image_near = False
        full_image_near_err = 0.0


        half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
        if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
            half_image_zero_or_near = True
        
        full_image_near_err = pixel_count_near/tot_pixeis
        if full_image_near_err >= full_image_near_percentage:
            full_image_near = True
        
        
        if half_image_zero_or_near or full_image_near:
            overall = True

        # just for debug
        # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
        # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err
        
        return overall