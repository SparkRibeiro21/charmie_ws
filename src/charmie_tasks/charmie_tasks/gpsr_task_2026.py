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
    "charmie_arm":                  True, # True
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True, 
    "charmie_hand_camera":          True, # True
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

        # Task Name
        self.TASK_NAME = "GPSR Challenge"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":          0,
            
            "Move_to_instruction_point":       1,

            "Receive_requests":                2,
            "Set_command_order":               3,
            "Execute_gpsr_commands":           4,

            "Return_to_instruction_point":     5,

            "Final_State":                     6,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Initial Position
        self.initial_position = [0.0, 0.0, 0.0]

        self.instruction_point =  [ 6.09, -1.33, 0]

        print(self.initial_position)
        print(self.instruction_point)


    def main(self):

        self.configurables() # set all the configuration variables

        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo
  
        # Added variables
        self.llps=[]
        self.order_to_execute=[]

        self.curr_request = 0

        USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS=True

        # End of added variables

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        

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

                # self.robot.wait_for_door_opening()

                # self.robot.enter_house_after_door_opening()
                
                self.state = self.task_states["Move_to_instruction_point"]
                

            elif self.state == self.task_states["Move_to_instruction_point"]:
                    
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

                self.robot.calibrate_audio()

                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)
                
                
                self.state = self.task_states["Receive_requests"]


            elif self.state == self.task_states["Receive_requests"]:

                self.curr_request+=1
                
                llp = self.robot.receive_command_and_generate_low_level_planner(command_no=self.curr_request, use_touchscreen_for_yes_no_questions=USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS)
                print("Low-level planner " + str(self.curr_request) + ": ", llp)
                self.llps.append(llp)

                # checks if there is another command to be received, if not proceeds to deciding and executing the commands in the list
                if not USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS:
                    confirmation = self.robot.get_audio(yes_or_no=True, question="gpsr/do_you_have_another_command", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                else: # if touchscreen is used
                    if self.curr_request == 1: # for time efficiency, only inform user that needs to press the face for the first command
                        self.robot.set_speech(filename="generic/press_correct_option_touchscreen", wait_for_end_of=True) # SAY: Please press the correct option on my face.
                    answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Do you have another command?", speak_results=False, speak_timeout=False, start_speak_file="gpsr/do_you_have_another_command", wait_for_end_of=True)
                    confirmation = answer[0]

                if not confirmation.lower() == "yes": # "no" or "TIMEOUT"
                    self.state = self.task_states["Set_command_order"]    


            elif self.state == self.task_states["Set_command_order"]:
          
                plan_feasibility = []

                for plan in self.llps:
                    cannot_perform_task = False
                    points = 0

                    for step in plan:
                        action = step.split("-")

                        match action[0]:                          
                            case "say_info" | "follow_person":
                                cannot_perform_task = True

                            case "compare_objects" | "count_objects":
                                points += 6

                            case "pick_object" | "place_object":
                                points += 3

                            case "move_to" | "say_result":
                                points += 2

                            case "go_to_person" | "look_for_person" | "guide_person":
                                points += 1

                            case "ERROR":
                                points -= 4

                    if cannot_perform_task:
                        points = float('-inf')

                    i = 0
                    while i < len(plan_feasibility) and plan_feasibility[i] >= points:
                        i += 1                                    

                    plan_feasibility.insert(i, points)            
                    self.order_to_execute.insert(i, plan)
               
                self.state = self.task_states["Execute_gpsr_commands"]


            elif self.state == self.task_states["Execute_gpsr_commands"]:
                
                for plan in self.order_to_execute:

                    for i, plan_check in enumerate(self.llps):

                        curr_plan = i +1

                        print(plan)
                        print(plan_check)

                        if plan == self.llps[i]:

                            
                            print("I will start executing request number"+str(curr_plan))
                            self.robot.set_speech(filename="gpsr/execute_request"+str(i+1), wait_for_end_of=True)

                    self.robot.execute_gpsr_plan(command=plan, instruction_point=self.instruction_point)

                self.state = self.task_states["Return_to_instruction_point"]


            elif self.state == self.task_states["Return_to_instruction_point"]:
                                        
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


                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
         
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
