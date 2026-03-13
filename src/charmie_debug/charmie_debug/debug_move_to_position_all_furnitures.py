#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import sys
import select

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
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 False,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
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
        
        self.initial_position = [0.0, 0.0, 0.0]
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        
        # VARS ...
        self.state = Waiting_for_start_button

        while True:

            if self.state == Waiting_for_start_button:

                # self.list_of_furnitures = [item["name"] for item in self.robot.node.furniture]
                self.list_of_furnitures = [item["name"] for item in self.robot.node.furniture]
                self.list_of_rooms = [item["name"] for item in self.robot.node.rooms]

                self.navigation_options = []

                for name in self.list_of_furnitures:
                    self.navigation_options.append(("furniture", name))

                for name in self.list_of_rooms:
                    self.navigation_options.append(("rooms", name))

                print(self.navigation_options)

                self.NAVIGATION_TARGET = ""
                self.NAVIGATION_TARGET_TYPE = ""
                self.NAVIGATION_TARGET_SPEAK = ""

                # self.robot.set_initial_position(self.initial_position)
                self.robot.wait_for_start_button()

                self.state = Move_through_furnitures

            elif self.state == Move_through_furnitures:
                    
                # Selects correct furniture number
                print("\nSelect a furniture for navigation (0 to finish):\n")
                print("   FURNITURES:")
                for i, name in enumerate(self.list_of_furnitures, start=1):
                    room = self.robot.get_room_from_furniture(name)
                    print(f"{i:2d}. {str(room).upper()+':':<12} {name}")
                print("\n   ROOMS:")
                for i, name in enumerate(self.list_of_rooms, start=len(self.list_of_furnitures)+1):
                    print(f"{i:2d}. {name.upper()}")
                print("\n   FINISH:")
                print(" 0. Finish")

                choice = input("\nEnter number: ").strip()
                if not choice:
                    print("No input provided.")
                    continue
                if not choice.isdigit():
                    print("Please enter a valid furniture number.")
                    continue
                choice = int(choice)
                # Option to finish
                if choice == 0:
                    print("\nFinished navigation selection.")
                    self.state = Final_State
                    continue
                if choice < 1 or choice > len(self.navigation_options):
                    print("Invalid option. Try again.")
                    continue

                self.NAVIGATION_TARGET_TYPE, self.NAVIGATION_TARGET = self.navigation_options[choice - 1]
                self.NAVIGATION_TARGET_SPEAK = (self.NAVIGATION_TARGET_TYPE + "/" + self.NAVIGATION_TARGET).replace(" ", "_").lower()
                print(f"\nSelected {self.NAVIGATION_TARGET_TYPE}: {self.NAVIGATION_TARGET} - Speak: {self.NAVIGATION_TARGET_SPEAK}")

                if self.NAVIGATION_TARGET_TYPE == "furniture":
                    move_coords = self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET)

                elif self.NAVIGATION_TARGET_TYPE == "rooms":
                    move_coords = self.robot.get_navigation_coords_from_room(self.NAVIGATION_TARGET)

                ### NAVIGATE TO CORRECT FURNITURE
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename=self.NAVIGATION_TARGET_SPEAK, wait_for_end_of=False)
                self.robot.move_to_position(move_coords=move_coords, wait_for_end_of=False)

                navigation_cancelled = False
                print("Press 'c' + Enter to cancel navigation and select a new furniture.")
                while not self.robot.move_to_position_is_done():

                    # check if user typed something without blocking
                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        user_input = sys.stdin.readline().strip().lower()

                        if user_input == "c":
                            self.robot.move_to_position_cancel()
                            navigation_cancelled = True
                            print("Navigation cancelled!")
                            break
                        
                        else:
                            print("Invalid input. Press 'c' + Enter to cancel navigation.")

                    time.sleep(0.05)

                if navigation_cancelled:
                    self.robot.set_speech(filename="generic/navigation_cancelled", wait_for_end_of=False)
                    time.sleep(0.2)
                else:
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename=self.NAVIGATION_TARGET_SPEAK, wait_for_end_of=False)                 

                self.state = Move_through_furnitures  # loop to select another furniture, finish is set by selecting 0 in furniture menu

            elif self.state == Final_State:

                self.state += 1
                print("Finished task!!!")

            else:
                pass