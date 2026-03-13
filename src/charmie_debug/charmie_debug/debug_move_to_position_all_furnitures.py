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
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 False,
    "charmie_radar":                True,
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

    def main(self):
        Waiting_for_start_button = 0
        Move_through_furnitures = 1
        Final_State = 2
        
        # self.initial_position = [0.0, 0.0, 0.0]
        self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        # self.initial_position = [2.5, -4.50, 0.0] # temp (near Tiago desk for testing)
        # self.initial_position = [2.6, -3.60, 45.0] # temp (near Tiago desk for testing)
        # self.initial_position = [0.0, 0.0, 0.0] # temp (near Tiago desk for testing)
        self.NAVIGATION_TARGET = "couch"
        self.NAVIGATION_TARGET2 = "desk"

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        
        # VARS ...
        self.state = Waiting_for_start_button

        while True:

            if self.state == Waiting_for_start_button:

                # self.robot.set_initial_position(self.initial_position)
                self.robot.wait_for_start_button()

                self.state = Move_through_furnitures

            elif self.state == Move_through_furnitures:
    
                    ### special case for rooms (speak)
                    while True:
                        # self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                        self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/"+self.NAVIGATION_TARGET, wait_for_end_of=False)

                        move_coords = self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET)                 
                        self.robot.move_to_position(move_coords=move_coords, wait_for_end_of=False)

                        while not self.robot.move_to_position_is_done():
                            print("Waiting...")
                            time.sleep(0.5)
                            pass
                        
                        self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                        self.robot.set_speech(filename="furniture/"+self.NAVIGATION_TARGET, wait_for_end_of=True)                    
        
                    self.state = Final_State

            elif self.state == Final_State:

                self.state += 1
                print("Finished task!!!")

            else:
                pass