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
    "charmie_arm":                  True,
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  True,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
    "charmie_speakers_save":        True,
    "charmie_tracking":             False,
    "charmie_tray_gripper":         False,
    "charmie_yolo_objects":         True,
    "charmie_yolo_pose":            True,
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

    # main state-machine function
    def main(self):
        # Waiting_for_start_button = 0
        Initialization = 1
        Get_command = 2
        Set_command_order = 3
        Execute_commands = 4
        Final_State = 5

        # VARS ...
        self.state = Initialization

        self.number_of_requests = 3
        self.curr_request = 0
        self.llps = []
        USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS = True
    
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
                
            if self.state == Initialization:

                print("New LLM GPSR")

                self.robot.wait_for_start_button()

                self.curr_room = "living_room"
                self.curr_furniture = "shelf"
                self.curr_result = "NONE"
                self.curr_obj_list =[]
                self.curr_picked_height= 0.0
                self.curr_asked_help = False

                robot_pose= self.robot.get_robot_localization()
                initial_position = [robot_pose.x, robot_pose.y, robot_pose.theta]
                print(f"Initial Robot Position: {initial_position}")

                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                print("SPEAK: 'Hello! My name is Charmie and I am here to help you with whatever you need.'")
                
                self.robot.calibrate_audio()

                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)

                self.state= Get_command

            if self.state == Get_command:

                self.curr_request+=1
                
                llp = self.robot.receive_command_and_generate_low_level_planner(use_touchscreen_for_yes_no_questions=USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS)
                print("Low-level planner " + str(self.curr_request) + ": " + llp)
                self.llps.append(llp)

                # checks if there is another command to be received, if not proceeds to deciding and executing the commands in the list
                if not USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS:
                    confirmation = self.robot.get_audio(yes_or_no=True, question="gpsr/do_you_have_another_command", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                else: # if touchscreen is used
                    answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Do you have another command?", speak_results=False, start_speak_file="gpsr/do_you_have_another_command", wait_for_end_of=True)
                    confirmation = answer[0]
                    
                if confirmation.lower() == "no":
                    self.state = Set_command_order   
                
            if self.state == Set_command_order:
                pass

                # CHECK LLP LIST



            if self.state == Execute_commands:
                pass




            elif self.state == Final_State:
                
                self.robot.set_speech(filename="gpsr/end_of_gpsr", wait_for_end_of=True)
                print("Finished LLM GPSR")
                
                self.state += 1
                print("Finished task!!!")

            else:
                pass