#!/usr/bin/env python3
import rclpy
import threading
import time
from datetime import datetime
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              False,
    "charmie_audio":            True,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_lidar":            False,
    "charmie_llm":              True,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_point_cloud":      False,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         True,
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

    # main state-machine function
    def main(self):
        # Waiting_for_start_button = 0
        LLM_demo = 1
        LLM_gpsr = 2
        Final_State = 3

        # VARS ...
        self.state = LLM_gpsr
    
        self.robot.set_face("charmie_face")
        print("IN NEW MAIN")
        
        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Audio Receptionist
            # State 2 = Audio Restaurant
            # State 3 = Audio EGPSR
            # State 4 = Calibrate Audio
            # State 5 = Final Speech

            if self.state == LLM_demo:

                print("New LLM Demo")
                self.robot.get_llm_demonstration()
                print("Finished LLM Demo")
                time.sleep(5)
                
            if self.state == LLM_gpsr:

                print("New LLM GPSR")
                self.robot.get_llm_gpsr()
                print("Finished LLM GPSR")
                time.sleep(5)
                            
            elif self.state == Final_State:
                
                self.state += 1
                print("Finished task!!!")

            else:
                pass