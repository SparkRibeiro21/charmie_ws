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
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_lidar":            False,
    "charmie_llm":              False,
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
        # time.sleep(2)

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

                o = "juice pack"
                c = self.robot.get_object_class_from_object(o)
                f = self.robot.get_furniture_from_object_class(c)
                r = self.robot.get_room_from_furniture(f)
                fnc = self.robot.get_navigation_coords_from_furniture(f)
                flc = self.robot.get_location_coords_from_furniture(f)
                rnc = self.robot.get_navigation_coords_from_room(r)
                print(o, "|", c, "|", f, "|", fnc, "|", flc, "|", r, "|", rnc)

                if self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)) is not None:
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)), wait_for_end_of=True)
                    time.sleep(1.0)
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)), wait_for_end_of=True)
                else:
                    print("Wrong object name! Skipping speaking and navigation...")
                
                # self.robot.set_neck_coords(self.robot.get_location_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o))), 
                #                            wait_for_end_of=True)
                # self.robot.set_neck_coords(self.robot.get_location_coords_from_furniture("dishwasher"), 
                #                            wait_for_end_of=True)

                while True:
                    pass
                
                self.robot.get_detected_person_characteristics(first_sentence="demonstration/demo_characteristics_first_sentence", shirt_color=True, age=True)

                
                ##### SAVE SPEAK
                # current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                # self.robot.save_speech(command="This is just a test with a play command", filename=current_datetime, quick_voice=True, play_command=True, show_in_face=True, wait_for_end_of=True)

                ##### SPEAK: (repeats the command)
                # self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                self.robot.set_face("help_pick_milk")
                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_milk", command="", wait_for_end_of=True)
                time.sleep(3)

                # self.robot.set_speech(filename="generic/introduction_ful", command="", wait_for_end_of=True)

                self.robot.set_face("help_pick_orange_juice")
                time.sleep(3)

                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_orange_juice", show_in_face=True, wait_for_end_of=True)



                self.robot.set_face("help_pick_red_wine")
                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_red_wine", wait_for_end_of=True)
                time.sleep(3)

                self.robot.set_face("help_pick_spoon")
                time.sleep(3)

                # next state
                # self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                self.state += 1
                print("Finished task!!!")

            else:
                pass