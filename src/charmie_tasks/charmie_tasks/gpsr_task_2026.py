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
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 True,
    "charmie_radar":                False, # True
    "charmie_sound_classification": True,
    "charmie_speakers":             True,
    "charmie_speakers_save":        True,
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

        # Task Name
        self.TASK_NAME = "GPSR Challenge"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":          0,
            
            "Move_to_instruction_point":       1,

            "Receive_request1":                2,
            "Receive_request2":                3,
            "Receive_request3":                4,
            "Show_generated_plans":            5,

            "Execute_request1":                6,
            "Execute_request2":                7,
            "Execute_request3":                8,

            "Return_to_instruction_point":     9,

            "Final_State":                     10,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Initial Position
        self.initial_position = [0.0, 0.0, 0.0]

        self.instruction_point =  [ 7.80, 3.84, 180]
        print(self.initial_position)
        print(self.instruction_point)
        

    def main(self):

        self.configurables() # set all the configuration variables

        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo
  
        # Added variables
        self.request1 = ""
        self.request2 = ""
        self.request3 = ""


        self.number_of_requests = 3
        self.curr_request = 1

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

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.wait_for_door_opening()

                self.robot.enter_house_after_door_opening()
                
                self.state = self.task_states["Move_to_instruction_point"]
                

            elif self.state == self.task_states["Move_to_instruction_point"]:
                                        
                pass
                # your code here ... 

                # Look at the navigation
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # Announce navigation
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="gpsr/instruction_point", wait_for_end_of=True)
                

                # Move to the instruction point
                self.robot.move_to_position(move_coords=self.instruction_point, wait_for_end_of=True)
                # Look at the judge
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                
                # Announce arrival
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                self.robot.set_speech(filename="gpsr/instruction_point", wait_for_end_of=True)
                
                
                self.state = self.task_states["Receive_request1"]


            elif self.state == self.task_states["Receive_request1"]:

                self.curr_request = 1
                
                self.robot.calibrate_audio()

                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)

                self.curr_request = 1

                request = self.robot.get_llm_confirm_command()

                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    print("Request " + str(self.curr_request) + ": " + request)

                    ##### SPEAK: "Okay, I understood your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    ##### SPEAK: "Please give me a moment while I proccess your requests"
                    self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=False)

                    #### SPEAK: "This may take more than a minute"
                    self.robot.set_speech(filename="gpsr/may_take_a_while", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)

                    ##### SPEAK: "I am almost done with your request, please wait a little bit more."
                    self.robot.set_speech(filename="gpsr/please_wait", wait_for_end_of=False)
                    
                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request1", quick_voice=True ,wait_for_end_of=True)
                    self.robot.set_speech(filename="gpsr/say_plan1", wait_for_end_of= False)
                    self.robot.set_speech(filename="temp/gpsr_request1", wait_for_end_of= True)

                self.state = self.task_states["Receive_request2"]

            elif self.state == self.task_states["Receive_request2"]:
                
                ##### SPEAK: "Let's move on to the second request"
                #

                self.curr_request = 2

                # Look at the judge
                # self.robot.set_neck
                
                request = self.robot.get_llm_confirm_command()


                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    # self.request2 = request
                    print("Request " + str(self.curr_request) + ": " + request)

                    ##### SPEAK: "Okay, I understood your curr_request request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    ##### SPEAK: "Please give me a moment while I proccess your requests"
                    self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=False)

                    #### SPEAK: "This may take more than a minute"
                    self.robot.set_speech(filename="gpsr/may_take_a_while", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)

                    ##### SPEAK: "I am almost done with your request, please wait a little bit more."
                    self.robot.set_speech(filename="gpsr/please_wait", wait_for_end_of=False)

                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request2", quick_voice=True ,wait_for_end_of=True)

                    self.robot.set_speech(filename="gpsr/say_plan2", wait_for_end_of= False)
                    self.robot.set_speech(filename="temp/gpsr_request2", wait_for_end_of= True)
                    # self.request2 = hlp_request[0]

                self.state = self.task_states["Receive_request3"]
            
            elif self.state == self.task_states["Receive_request3"]:
            
                self.curr_request = 3
                # your code here ...

                # Look at the judge
                # self.robot.set_neck
                
                request = self.robot.get_llm_confirm_command()


                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    # self.request3 = request
                    print("Request " + str(self.curr_request) + ": " + request)

                    ##### SPEAK: "Okay, I understood your curr_request request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    ##### SPEAK: "Please give me a moment while I proccess your requests"
                    self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=False)

                    #### SPEAK: "This may take more than a minute"
                    self.robot.set_speech(filename="gpsr/may_take_a_while", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)

                    ##### SPEAK: "I am almost done with your request, please wait a little bit more."
                    self.robot.set_speech(filename="gpsr/please_wait", wait_for_end_of=False)

                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request3", quick_voice=True ,wait_for_end_of=True)

                    self.robot.set_speech(filename="gpsr/say_plan3", wait_for_end_of= False)
                    self.robot.set_speech(filename="temp/gpsr_request3", wait_for_end_of= True)
                    # self.request3 = hlp_request[0]

                self.state = self.task_states["Show_generated_plans"]


            elif self.state == self.task_states["Show_generated_plans"]:
                
                # ##### SPEAK: "To execute the first request"  
                # self.robot.set_speech(filename="gpsr/say_plan1", wait_for_end_of= True)
                # self.robot.set_speech(filename="temp/gpsr_request1", wait_for_end_of= True)

                
                # ##### SPEAK: "To execute the second request"  
                # self.robot.set_speech(filename="gpsr/say_plan2", wait_for_end_of= True)
                # self.robot.set_speech(filename="temp/gpsr_request2", wait_for_end_of= True)
                
                # ##### SPEAK: "To execute the first request"  
                # self.robot.set_speech(filename="gpsr/say_plan3", wait_for_end_of= True)
                # self.robot.set_speech(filename="temp/gpsr_request3", wait_for_end_of= True)
                                        
                # # your code here ...

                self.state = self.task_states["Execute_request1"]


            elif self.state == self.task_states["Execute_request1"]:

                # ##### SPEAK: "I will start by executing the first request."
                # self.robot.set_speech(filename="gpsr/execute_request1", wait_for_end_of=True)
                # self.robot.execute_gpsr_plan(command=self.request1,wait_for_end_of=True)

                # ##### SPEAK: "I have finished executing the first task."
                # self.robot.set_speech(filename="gpsr/finished_request1", wait_for_end_of=True)

                

                self.state = self.task_states["Execute_request2"]


            elif self.state == self.task_states["Execute_request2"]:
                                        
                # ##### SPEAK: "I will start by executing the second request."
                # self.robot.set_speech(filename="gpsr/execute_request2", wait_for_end_of=True)

                # self.robot.execute_gpsr_plan(command=self.request2,wait_for_end_of=True)

                # ##### SPEAK: "I have finished executing the second task."
                # self.robot.set_speech(filename="gpsr/finished_request2", wait_for_end_of=True)


                self.state = self.task_states["Execute_request3"]


            elif self.state == self.task_states["Execute_request3"]:
                                        
                # ##### SPEAK: "I will start by executing the third request."
                # self.robot.set_speech(filename="gpsr/execute_request3", wait_for_end_of=True)

                # self.robot.execute_gpsr_plan(command=self.request3,wait_for_end_of=True)

                # ##### SPEAK: "I have finished executing the third task."
                # self.robot.set_speech(filename="gpsr/finished_request3", wait_for_end_of=True)

                self.state = self.task_states["Return_to_instruction_point"]


            elif self.state == self.task_states["Return_to_instruction_point"]:
                                        
                # Inform that cannot perform requests (for now)
                self.robot.set_speech(filename="gpsr/cannot_perform_low_level", wait_for_end_of=True)

                # # Look at the navigation
                # self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # # Announce navigation
                # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                # ##ADICIONAR FALA PARA INSTRUCTION POINT
                # self.robot.set_speech(filename="gpsr/instruction_point", wait_for_end_of=True)
                

                # # Move to the instruction point
                # self.robot.move_to_position(move_coords=self.instruction_point, wait_for_end_of=True)
                
                # # Announce arrival
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                # ##ADICIONAR FALA PARA INSTRUCTION POINT
                # self.robot.set_speech(filename="gpsr/instruction_point", wait_for_end_of=True)
                

                self.state = self.task_states["Final_State"]

            elif self.state == self.task_states["Final_State"]:
                
                ### self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                #change to GPSR Challenge
                self.robot.set_speech(filename="gpsr/end_of_gpsr", wait_for_end_of=False)

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
