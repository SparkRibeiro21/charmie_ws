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
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
    "charmie_speakers":             False,
    "charmie_tracking":             False,
    "charmie_yolo_objects":         False,
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

    def main(self):
        Waiting_for_start_button = 0
        Face_recognition = 1
        Final_State = 2

        self.state = Waiting_for_start_button

        print("IN NEW MAIN")
        # time.sleep(2)

        while True:

            if self.state == Waiting_for_start_button:

                self.robot.activate_yolo_pose(activate=True)

                self.state = Face_recognition
                
             
            if self.state == Face_recognition:   
                
                # input("\nPress Enter to continue... (DEBUG)")
                choice = input("\nWrite the name of the person to add the encoding, or 'c' to compare. Press Enter to continue... (DEBUG)").strip().lower()
                # print(choice)

                det_ppl = self.robot.node.detected_people.persons
                # print(len(det_ppl))
                
                if choice == 'c':
                    print("COMPARE ENCODING MODE")
                    for person in det_ppl:
                        # print("ID:", person.index)
                        pred, pred_perc, conf_table = self.robot.recognize_face_from_face_recognition(person=person)
                        print("COMPARE OUTCOME:", pred, round(pred_perc, 2))
                        print("CONFIDENCE TABLE:", conf_table)
                
                else: # name of person to add
                    print("ADD ENCODING MODE, name:", choice)
                    for person in det_ppl:
                        # print("ID:", person.index)
                        s, m = self.robot.add_face_to_face_recognition(person=person, name=choice)
                        print("ADD OUTCOME:", s, m)
    
            elif self.state == Final_State:
                self.state += 1
                print("Finished task!!!")

            else:
                pass