#!/usr/bin/env python3

import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106



ros2_modules = {
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          False,
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
        
        # States in DebugMoveit Task
        self.Waiting_for_task_start = 0
        self.test_state_1 = 1
        self.Final_State = 2

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start


        # debug print to know we are on the main start of the task
        print("In DebugMoveit Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")

                while True:

                    front_pick_joints = [-3.7524, -1.2217, -0.2793, 1.3963, 0.5236, 3.1765]

                    self.robot.set_joint_target_arm(front_pick_joints, wait_for_end_of=True)

                    time.sleep(2.0)

                    self.robot.set_named_target_arm("home", wait_for_end_of=True)

                    pass
                    
                    

            elif self.state == self.Final_State:
                
                print("State:", self.state, "- Final_State")

                # Lock after finishing task
                while True:
                    pass

            else:
                pass
