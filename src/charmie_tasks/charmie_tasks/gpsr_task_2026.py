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
    "charmie_arm":                  False, # True
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          False, 
    "charmie_hand_camera":          False, # True
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  True,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_neck":                 True,
    "charmie_radar":                False, # True
    "charmie_sound_classification": True,
    "charmie_speakers":             True,
    "charmie_tracking":             True,
    "charmie_yolo_objects":         True,
    "charmie_yolo_pose":            True,
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
        self.TASK_NAME = "GPSR Challenge"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":          0,
            
            "Move_to_instruction_point":       1,

            "Receive_requests":                2,
            "Show_generated_plans":            3,

            "Execute_request1":                4,
            "Execute_request2":                5,
            "Execute_request3":                6,

            "Return_to_instruction_point":     7,

            "Final_State":                     8,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Initial Position
        self.initial_position = [2.0, 4.0, 45.0]

        self.instruction_point = [2.0, 4.0, 45.0]
        print(self.initial_position)
        
        # self.start_follow_position = self.initial_position
        self.start_follow_position = [2.0, 4.0, 90.0] # position to start following host after introducing guests
        # print(self.start_follow_position)

        self.number_of_requests = 3
        self.curr_request = 1


    def main(self):

        self.configurables() # set all the configuration variables

        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo
  
        # Added variables
        self.request1 = ""
        self.request2 = ""
        self.request3 = ""

        self.request_order = [self.request1, self.request2, self.request3]
        # End of added variables


        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]] # , [-45-10, -45-5], [-45+10, -45-5]]

        self.state = self.task_states["Waiting_for_task_start"]

        print("IN " + self.TASK_NAME.upper() + " MAIN")
        if self.DEMO_MODE:
            print("DEMO MODE:", self.DEMO_MODE)

        while True:
            self.robot.set_current_task_state_id(current_state=self.state) # Necessary to visualize current task state in GUI
            
            if self.state == self.task_states["Waiting_for_task_start"]:

                self.robot.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")
                
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                #change to GPSR Challenge
                # self.robot.set_speech(filename="hri/start_hri_task", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.state = self.task_states["Move_to_instruction_point"]
                

            elif self.state == self.task_states["Move_to_instruction_point"]:
                                        
                pass
                # your code here ... 

                # Look at the navigation
                # self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # Announce navigation
                # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                
                # Announce the instruction point

                # Move to the instruction point
                # self.robot.move_to_position(move_coords=self.initial_position, wait_for_end_of=True)
                
                # Announce arrival
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                
                # Announce the instruction point

                self.state = self.task_states["Receive_requests"]


            elif self.state == self.task_states["Receive_requests"]:
                
                for self.curr_request in range(self.number_of_requests):
                    # your code here ...

                    # Look at the judge
                    # self.robot.set_neck

                    # Ask for the requests 
                    # ("Hello! My name is Charmie and I am here to help you with what you need.")
                    # self.robot.set_speech with quick_voice 
                    
                    # Hear and confirm requests
                    # "Please tell me your curr_request request"
                    # self.set_speech

                    self.request1 = self.robot.get_llm_confirm_command(wait_for_end_of=True)

                    if self.request1 == "ERROR":
                        print("Error in request " + str(self.curr_request + 1))
                        # Announce error
                        # ("I was not able to understand your request. Let's move on to the next one.")
                        # self.robot.set_speech

                    else:
                        print("Request " + str(self.curr_request + 1) + ": " + self.request1)
                        # "Okay, I understood your curr_request request. Let's move on to the next one."
               
                self.state = self.task_states["Show_generated_plans"]


            elif self.state == self.task_states["Show_generated_plans"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Execute_request1"]


            elif self.state == self.task_states["Execute_request1"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Execute_request2"]


            elif self.state == self.task_states["Execute_request2"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Execute_request3"]


            elif self.state == self.task_states["Execute_request3"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Return_to_instruction_point"]


            elif self.state == self.task_states["Return_to_instruction_point"]:
                                        
                pass
                # your code here ...

                # Look at the navigation
                # self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # Announce navigation
                # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                
                # Announce the instruction point

                # Move to the instruction point
                # self.robot.move_to_position(move_coords=self.initial_position, wait_for_end_of=True)
                
                # Announce arrival
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                
                # Announce the instruction point

                self.state = self.task_states["Final_State"]

            elif self.state == self.task_states["Final_State"]:
                
                ### self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                #change to GPSR Challenge
                # self.robot.set_speech(filename="hri/finish_hri_task", wait_for_end_of=False)

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
