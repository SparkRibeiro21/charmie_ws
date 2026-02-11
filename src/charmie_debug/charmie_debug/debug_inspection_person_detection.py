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
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                False,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         False,
    "charmie_low_level":            False,
    "charmie_navigation":           False,
    "charmie_nav2":                 False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
    "charmie_speakers":             False,
    "charmie_tracking":             False,
    "charmie_yolo_objects":         False,
    "charmie_yolo_pose":            False,
    "charmie_yolo_world":           False,
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

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -10]

    def main(self):
        Waiting_for_start_button = 0
        Inspection_detect_people_with_inspection_camera = 1
        Final_State = 2

        self.state = Waiting_for_start_button

        print("IN NEW MAIN")

        while True:

            # State Machine
            # State 0 = Waiting_for_start_button
            # State 1 = Inspection_detect_people_with_inspection_camera
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:
                # print('State 0 = Waiting_for_start_button')

                self.state = Inspection_detect_people_with_inspection_camera

            elif self.state == Inspection_detect_people_with_inspection_camera:
                #print('State 1 = Inspection_detect_people_with_inspection_camera')

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False) # if we need to check in the real neck position (copied from inspection task)
                overall = self.robot.safety_navigation_check_depth_head_camera() # half_image_zero_or_near_percentage=0.6, full_image_near_percentage=0.3, near_max_dist=0.8
                      
                if overall:
                    print("STOP")
                else:
                    print("GO")
                        
            elif self.state == Final_State:
                print("Finished task!!!")

                while True:
                    pass

            else:
                pass
