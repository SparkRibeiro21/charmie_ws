#!/usr/bin/env python3

### LAUNCH FILE ARM: ros2 launch xarm_api xarm6_driver.launch.py

import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_lidar":            False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
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
        
        # States in DebugArm Task
        self.Waiting_for_task_start = 0
        self.Approach_kitchen_counter = 1
        self.Picking_up_spoon = 2
        self.Picking_up_milk = 3
        self.Picking_up_cereal = 4
        self.Picking_up_bowl = 5
        self.Approach_kitchen_table = 6
        self.Placing_bowl = 7
        self.Placing_cereal = 8
        self.Placing_milk = 9
        self.Placing_spoon = 10
        self.Final_State = 11

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start

        self.ATTEMPTS_AT_RECEIVING = 2
        self.SHOW_OBJECT_DETECTED_WAIT_TIME = 3.0

        # debug print to know we are on the main start of the task
        print("In DebugArm Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")
                

                # INITIAL STATE

                self.robot.set_speech(filename="serve_breakfast/found_the_bowl", wait_for_end_of=False)  
                
                self.robot.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  

                self.robot.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=True)

                time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)
                
                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.robot.set_face("help_pick_bowl") 
                
                object_in_gripper = False
                gripper_ctr = 0
                while not object_in_gripper and gripper_ctr < self.ATTEMPTS_AT_RECEIVING:
                    
                    gripper_ctr += 1
                    
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        if gripper_ctr < self.ATTEMPTS_AT_RECEIVING:

                            self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                if not object_in_gripper and gripper_ctr >= self.ATTEMPTS_AT_RECEIVING:

                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                    self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)
                        
                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                

                # example of arm function to say hello
                # self.robot.set_arm(command="hello", wait_for_end_of=False)

                # next state
                self.state = self.Approach_kitchen_counter

            elif self.state == self.Approach_kitchen_counter:
                print("State:", self.state, "- Approach_kitchen_counter")
                # your code here ...
                                
                # next state
                self.state = self.Final_State

            elif self.state == self.Final_State:
                
                self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                # Lock after finishing task
                while True:
                    pass

            else:
                pass
