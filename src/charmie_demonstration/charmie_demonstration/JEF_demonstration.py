#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject, DetectedPerson
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            True,
    "charmie_face":             True,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_base_camera":      False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       True,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     True,
    "charmie_yolo_pose":        True,
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

        # Task Name
        self.TASK_NAME = "JEF Demonstration"

        # Task States
        self.task_states ={
            "Locate_person":          0,
            "Greet_person":           1,
            "Grab_book":              2,
            "Offer_book":             3,
            "Final_state":            4,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        self.initial_neck_position = [0, 0] 
        self.search_person_tetas = [0, 0]
        self.search_book_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]]
        self.release_timer = 0.5
        self.gripper_release_timer = 2

        self.GRAB_BOOK = False

    # main state-machine function
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.robot.set_face("charmie_face") #PERGUNTAR SE VAI SER SEMPRE UMA

        self.state = self.task_states["Waiting_for_task_start"]

        print("IN " + self.TASK_NAME.upper() + " MAIN")
        if self.DEMO_MODE:
            print("DEMO MODE:", self.DEMO_MODE)

        while True:
            self.robot.set_current_task_state_id(current_state=self.state) # Necessary to visualize current task state in GUI
            
            if self.state == self.task_states["Locate_person"]:

                self.robot.set_neck(position=self.initial_neck_position, wait_for_end_of=True)
                self.robot.wait_for_start_button()
                #LOCATE PERSON; ADD CASE IF NOT;ASK HOW MANY PEOPLE
                #HOW TO KNOW IF EMPTY
                people_found = self.robot.search_for_person(tetas=self.search_person_tetas, time_in_each_frame=2.0)

                self.state = self.task_states["Greet_person"]

            elif self.state == self.task_states["Greet_person"]:
                
                closest = 9999999
                prev_closest = 9999998
                #ADD WAY TO FILTER LOWEST DISTANCE DETECTED; SAVE p FOR LATER USE
                for p in people_found:
                    closest = p.position_relative.x + p.position_relative.y + p.position_relative.z
                    if closest < prev_closest:
                        saved_p = p
                self.robot.set_neck_coords(position=[saved_p.position_relative.x, saved_p.position_relative.y, saved_p.position_relative.z], wait_for_end_of=True)
                self.robot.set_speech(filename="generic/JEF_hello", wait_for_end_of=True) #NEED TO GENERATE

                self.state = self.task_states["Grab_book"]


            elif self.state == self.task_states["Grab_book"]:
                
                #ROTATE 90 BASE def set_navigation(self, movement="", target=[0.0, 0.0], max_speed=15.0, absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, avoid_people=False, wait_for_end_of=True):
                #ADD MANUAL GRAB
                if self.GRAB_BOOK:
                    self.robot.pick_obj(selected_object="Book",mode="front",first_tetas=self.search_book_tetas) #NEED TO ADD BOOK
                else:
                    self.robot.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                    self.robot.set_arm(command="open_gripper", wait_for_end_of=True)
                    self.robot.wait_for_start_button()
                    self.robot.set_arm(command="close_gripper", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                self.state = self.task_states["Offer_Book"]

            elif self.state == self.task_states["Offer_Book"]:
                
                #ROTATE 90, MOVE NECK TO LAST PERSON LOCATED
                self.robot.set_neck_coords(position=[saved_p.position_relative.x, saved_p.position_relative.y, saved_p.position_relative.z], wait_for_end_of=True)

                self.robot.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                self.robot.set_speech(filename="generic/book_release_countdown", wait_for_end_of=True) #NEED TO GENERATE; BOOK RELESE AND EXPLANATION
                time.sleep(self.release_timer)
                self.robot.set_arm(command="open_gripper", wait_for_end_of=True)
                time.sleep(self.gripper_release_timer)

                self.state = self.task_states["Final_state"]


            elif self.state == self.task_states["Final_state"]:
                # your code here ...
                self.robot.set_speech(filename="generic/JEF_book_introduction", wait_for_end_of=False) #NEED TO GENERATE
                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                self.robot.set_arm(command="close_gripper", wait_for_end_of=True)
                # next state
                self.state = self.task_states["Locate_person"]

                while True:
                    pass

            else:
                pass

            # This part is essential for the task_demo to work properly
            if self.state == self.DEMO_STATE: # Essential for task_demo to work
                self.robot.set_speech(filename="generic/done", wait_for_end_of=False)
                while not self.robot.get_received_new_demo_task_state():
                    time.sleep(1.0)
                    print(".")
                self.state = self.robot.get_new_demo_task_state()
                print("OUT:", self.state)
            
            elif self.DEMO_MODE:
                self.state = self.DEMO_STATE # set state to -1 to wait for new state to be set by task_demo
