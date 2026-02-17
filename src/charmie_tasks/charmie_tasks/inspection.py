#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
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
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
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

        # Task Name
        self.TASK_NAME = "Inspection"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":       0,
            "Move_to_inspection_point":     1,
            "Robot_intro_and_checks":       2,
            "Move_to_exit_door":            3,
            "Final_State":                  4,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Name of the table where breakfast is served
        self.INSPECTION_POINT_COORDS = [2.5, -4.0, 90.0] # Middle fo the Office
        self.EXIT_LOCATION_NAME = "Exit"
        
        # Initial Position
        self.initial_position = [0.0, 0.0, 0.0]
        # self.initial_position = 
        print(self.initial_position)
        
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.EXIT_LOCATION_NAME = self.EXIT_LOCATION_NAME.lower().replace(" ", "_")
        # Checks if there is any error in the furniture variables:
        if self.robot.get_room_from_furniture(self.EXIT_LOCATION_NAME) == None:
            print("ERROR!!! - FURNITURE:", self.EXIT_LOCATION_NAME, "DOES NOT EXIST IN furniture.json")
            while True:
                pass
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -10]
        
        self.state = self.task_states["Waiting_for_task_start"]

        print("IN " + self.TASK_NAME.upper() + " MAIN")
        if self.DEMO_MODE:
            print("DEMO MODE:", self.DEMO_MODE)

        while True:
            self.robot.set_current_task_state_id(current_state=self.state) # Necessary to visualize current task state in GUI
            
            if self.state == self.task_states["Waiting_for_task_start"]:

                self.robot.set_initial_position(self.initial_position)                
                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="inspection/inspection_ready_start", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.wait_for_door_opening()
                self.robot.enter_house_after_door_opening(speed=0.15) # slower than usual to match inspection nav speed

                self.state = self.task_states["Move_to_inspection_point"]
                

            elif self.state == self.task_states["Move_to_inspection_point"]:
                                        
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="inspection/inspection_point", wait_for_end_of=False)

                # self.robot.move_to_position(move_coords=self.INSPECTION_POINT_COORDS, wait_for_end_of=True)
                self.robot.move_to_position_with_safety_navigation(move_coords=self.INSPECTION_POINT_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="inspection/inspection_point", wait_for_end_of=False)
            
                self.state = self.task_states["Robot_intro_and_checks"]


            elif self.state == self.task_states["Robot_intro_and_checks"]:

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="inspection/inspection_introduction", wait_for_end_of=True)
                self.robot.wait_for_start_button()

                self.state = self.task_states["Move_to_exit_door"]


            elif self.state == self.task_states["Move_to_exit_door"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.EXIT_LOCATION_NAME, wait_for_end_of=False)

                # self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.EXIT_LOCATION_NAME), wait_for_end_of=True)
                self.robot.move_to_position_with_safety_navigation(move_coords=self.robot.get_navigation_coords_from_furniture(self.EXIT_LOCATION_NAME), wait_for_end_of=True)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.EXIT_LOCATION_NAME, wait_for_end_of=False)
                                
                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="inspection/inspection_finished", wait_for_end_of=False)

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
