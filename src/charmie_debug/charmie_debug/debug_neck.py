#!/usr/bin/env python3
import rclpy
import threading
import time
from datetime import datetime
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import os
import signal

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
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
    "charmie_speakers":             False,
    "charmie_speakers_save":        False,
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
        Move_neck = 1
        Final_State = 2

        self.neck_limits = {
                "h": {"min": -180, "max": 179},
                "v": {"min": -60,  "max": 60},
            }
        
        self.current_neck_position = [0, 0]

        # Neck Positions
        self.look_forward = [0, 0]


        self.state = Waiting_for_start_button

        print("IN NEW MAIN")
        # time.sleep(2)

        while True:

            if self.state == Waiting_for_start_button:

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.current_neck_position = self.look_forward.copy()

                self.h_min = self.neck_limits["h"]["min"]
                self.h_max = self.neck_limits["h"]["max"]
                self.v_min = self.neck_limits["v"]["min"]
                self.v_max = self.neck_limits["v"]["max"]

                print("\n--- Neck Control ---")
                print("Enter neck command in the format: [axis][angle]")
                print("Axis: 'h' for horizontal, 'v' for vertical")
                print("'h' axis positivve is right, negative is left")
                print("'v' axis positive is up, negative is down")
                print(f"Angle int value within (h: {self.h_min} to {self.h_max}, v: {self.v_min} to {self.v_max})")
                print("Example commands: h90, v0, v-30")
                print("Enter 'q' to quit")

                # next state
                self.state = Move_neck

            elif self.state == Move_neck:

                
                print(f"Current neck position: h={self.current_neck_position[0]}, v={self.current_neck_position[1]}")

                user_input = input("\nEnter neck command: ").strip().lower()

                if not user_input:
                    print("No input provided. Try again.")
                    continue

                if user_input == "q":
                    print("Exiting neck control.")
                    self.state = Final_State
                    continue

                axis = user_input[0]
                value_str = user_input[1:]

                # Check axis
                if axis not in self.neck_limits:
                    print("Invalid axis. Use 'h' for horizontal or 'v' for vertical.")
                    continue

                # Check numeric part exists
                if not value_str:
                    print("Missing angle value. Example: h90 or v-30")
                    continue

                # Check numeric value
                try:
                    value = int(value_str)
                except ValueError:
                    print("Invalid angle value. Please enter an integer, e.g. h90 or v-30")
                    continue

                # Check limits
                min_val = self.neck_limits[axis]["min"]
                max_val = self.neck_limits[axis]["max"]

                if value < min_val or value > max_val:
                    print(f"Value out of range for axis '{axis}'. Allowed range: {min_val} to {max_val}")
                    continue

                if axis == "h":
                    print(f"Set horizontal neck axis to {value} degrees")
                    self.current_neck_position[0] = value
                elif axis == "v":
                    print(f"Set vertical neck axis to {value} degrees")
                    self.current_neck_position[1] = value

                self.robot.set_neck(position=self.current_neck_position, wait_for_end_of=False)
                                
                # next state
                self.state = Move_neck
            
            elif self.state == Final_State:
                print("Finished task!!!")
                break

            else:
                pass