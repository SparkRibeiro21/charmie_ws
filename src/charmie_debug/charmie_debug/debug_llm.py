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
    "charmie_head_camera":          False,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                False,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  True,
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
    "charmie_tray_gripper":         False,
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

    # main state-machine function
    def main(self):
        # Waiting_for_start_button = 0
        LLM_demo = 1
        LLM_gpsr = 2
        LLM_person_pose = 3
        LLM_gpsr_llp= 4
        LLM_hri= 5
        LLM_Ollama_first_tests = 6
        Test_individual_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr = 7
        Test_together_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr = 8
        Final_State = 9

        # VARS ...
        self.state = LLM_demo

        self.number_of_requests = 3
        self.curr_request = 1
    
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

            if self.state == LLM_demo:

                print("New LLM Demo")


            commands = [
            "Please identify which object on the refrigerator is the lightest.",
            "Tell me which object on the refrigerator is the lightest.",
            "What's the lightest object on the refrigerator?",

            "Please verbally communicate the day of the month to the waving person in the bedroom.",
            "Tell the day of the month to the waving person in the bedroom.",
            "Tell the waving person in the bedroom the day of the month.",

            "Please proceed to the living room, introduce yourself to the person wearing a black jacket, and thereafter follow them.",
            "Go to the living room, introduce yourself to the person wearing a black jacket, then follow them.",
            "In the living room, introduce yourself to the person wearing a black jacket and follow them.",

            "Kindly address the person who is raising their left arm in the bedroom with a statement about yourself.",
            "Tell the person raising their left arm in the bedroom something about yourself.",
            "Talk about yourself to the person raising their left arm in the bedroom.",

            "Please direct the person pointing to the left, ensuring they move from the sofa to the waste basket.",
            "Guide the person who is pointing to the left from the sofa over to the waste basket.",
            "Help the person pointing to the left go from the sofa to the waste basket.",

            "Please proceed to the kitchen, at which point you should locate a toothpaste, take it, and deliver it to me.",
            "Go to the kitchen, find a toothpaste, take it, and deliver it to me.",
            "Head to the kitchen, grab a toothpaste, and deliver it to me.",

            "First, proceed to the storage rack; upon arrival, identify the sitting person and subsequently follow them.",
            "Go to the storage rack, find the sitting person, and then follow them.",
            "Head to the storage rack, spot the sitting person, and follow them.",

            "Proceed to the chairs first, after which you should meet Scarlett and subsequently follow them.",
            "Go to the chairs, then meet Scarlett, and then follow them.",
            "Head to the chairs, meet Scarlett, and follow them.",

            "Proceed to the bed to retrieve a lemon, and subsequently deliver it to the standing person in the kitchen.",
            "Grab a lemon from the bed and take it to the standing person in the kitchen.",
            "Take a lemon from the bed to the standing person in the kitchen.",

            "Proceed to the armchair, locate the lying person, and subsequently lead them to the bedroom.",
            "Go to the armchair, find the lying person, and then lead them to the bedroom.",
            "Head to the armchair, spot the lying person, and take them to the bedroom.",

            "Please retrieve a chocolate jello from the bed and subsequently dispose of it by throwing it in the trash.",
            "Pick up a chocolate jello from the bed, then toss it in the trash.",
            "Grab a chocolate jello from the bed and throw it in the trash.",

            "Proceed to navigate to the bedroom; upon arrival, locate a food, retrieve it, and subsequently deliver it to the waving person in the kitchen.",
            "Go to the bedroom, find a food, pick it up, and bring it to the waving person in the kitchen.",
            "Head to the bedroom, find a food, get it, and take it to the waving person in the kitchen.",

            "Identify, within the bedroom, a person who is pointing to the left, and proceed to follow them to the office.",
            "In the bedroom, find a person pointing to the left and follow them to the office.",
            "Find a person pointing to the left in the bedroom, then follow them to the office.",

            "Kindly inform the waving person in the bedroom as to what day today is.",
            "Tell the waving person in the bedroom what day it is today.",
            "Tell the waving person in the bedroom what day today is.",

            "Would you be so kind as to bring me a cleanser from the side tables?",
            "Please bring me a cleanser from the side tables.",
            "Grab me a cleanser from the side tables.",

            "Locate a fruit within the living room, retrieve it, and present it to the person pointing to the right in the living room.",
            "Find a fruit in the living room, pick it up, and give it to the person pointing to the right in the living room.",
            "Search the living room for a fruit, take it, and hand it to the person pointing to the right in the living room.",

            "Please retrieve an iced tea from the side tables and proceed to deliver it to Matthew in the bedroom.",
            "Grab an iced tea from the side tables and deliver it to Matthew in the bedroom.",
            "Get an iced tea from the side tables and bring it to Matthew in the bedroom",

            "First, navigate to the cabinet; thereafter, meet Connor and subsequently follow them.",
            "Navigate to the cabinet, then meet Connor, and follow them.",
            "Go to the cabinet, meet Connor, and follow them.",

            "Please proceed to meet Michael in the living room, after which you are to guide them to the side tables.",
            "Meet Michael in the living room, then guide them to the side tables.",
            "Meet Michael in the living room and lead them to the side tables.",

            "Could you determine and inform me of the number of dishes that are on the sofa?",
            "Can you tell me how many dishes are on the sofa?",
            "How many dishes are on the sofa?"
        ]

            for i, command in enumerate(commands):

                print(f"============== {i}/{len(commands)}")

                start_time = time.time()

                hlp = self.robot.get_llm_ollama_gpsr_high_level(
                    command=command,
                    mode="",
                    wait_for_end_of=True
                )

                start_llp_time = time.time()

                llp_output = self.robot.get_llm_ollama_gpsr_low_level(
                    command=hlp[0],
                    mode="",
                    wait_for_end_of=True
                )

                end_time = time.time()

                print(f"Command: {command}")
                print(f"HLP: {hlp[0]}")
                print(f"LLP: {llp_output}")
                
                # print(f"Total Time taken for GPSR task {i}: {end_time - start_time:.3f} s")
                # print(f"Total Time taken for HLP task {i}: {start_llp_time - start_time:.3f} s")
                # print(f"Total Time taken for LLP task {i}: {end_time - start_llp_time:.3f} s")


            print("Finished LLM GPSR LLP")
            while True:
                pass
                
            if self.state == LLM_gpsr:

                print("New LLM GPSR")

                self.robot.wait_for_start_button()

                time.sleep(2)

                self.curr_room = "living_room"
                self.curr_furniture = "shelf"
                self.curr_result = "NONE"
                self.curr_obj_list =[]
                self.curr_picked_height= 0.0
                self.curr_asked_help = False

                robot_pose= self.robot.get_robot_localization()
                initial_position = [robot_pose.x, robot_pose.y, robot_pose.theta]
                print(f"Initial Robot Position: {initial_position}")


            ##Receive guest1
                
                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                print("SPEAK: 'Hello! My name is Charmie and I am here to help you with whatever you need.'")

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
                    # self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=False)

                    #### SPEAK: "This may take more than a minute"
                    # self.robot.set_speech(filename="gpsr/may_take_a_while", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)

                    ##### SPEAK: "I am almost done with your request, please wait a little bit more."
                    self.robot.set_speech(filename="gpsr/please_wait", wait_for_end_of=False)
                    
                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request1", quick_voice=True ,wait_for_end_of=True)
                    self.robot.set_speech(filename="gpsr/say_plan1", wait_for_end_of= False)
                    self.robot.set_speech(filename="temp/gpsr_request1", wait_for_end_of= False)
                    llp_plan_1 = self.robot.get_llm_ollama_gpsr_low_level(command=hlp_request[0], mode="", wait_for_end_of=True)

            ##Receive guest2
                
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

                    ##### SPEAK: "Okay, I understood your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    ##### SPEAK: "Please give me a moment while I proccess your requests"
                    # self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=False)

                    #### SPEAK: "This may take more than a minute"
                    # self.robot.set_speech(filename="gpsr/may_take_a_while", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)

                    ##### SPEAK: "I am almost done with your request, please wait a little bit more."
                    # self.robot.set_speech(filename="gpsr/please_wait", wait_for_end_of=False)

                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request2", quick_voice=True ,wait_for_end_of=True)

                    self.robot.set_speech(filename="gpsr/say_plan2", wait_for_end_of= False)
                    self.robot.set_speech(filename="temp/gpsr_request2", wait_for_end_of= False)
                    llp_plan_2 = self.robot.get_llm_ollama_gpsr_low_level(command=hlp_request[0], mode="", wait_for_end_of=True)
                    # self.request2 = hlp_request[0]

            ##Receive guest3
            
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
                    self.robot.set_speech(filename="temp/gpsr_request3", wait_for_end_of= False)
                    llp_plan_3 = self.robot.get_llm_ollama_gpsr_low_level(command=hlp_request[0], mode="", wait_for_end_of=True)

            # # ##execute request1

            # #     ##### SPEAK: "I will start by executing the first request."
            #     self.robot.set_speech(filename="gpsr/execute_request1", wait_for_end_of=True)
            #     for i, step in enumerate(llp_plan_1):
            #         print(f"Step {i+1}: {step}")
            #         self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=step, instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
            #         print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")

            # #     ##### SPEAK: "I have finished executing the first task."
            #     self.robot.set_speech(filename="gpsr/finished_request1", wait_for_end_of=True)

                

            # # ##execute request2
                                        
            # #     ##### SPEAK: "I will start by executing the second request."
            #     self.robot.set_speech(filename="gpsr/execute_request2", wait_for_end_of=True)
            #     for i, step in enumerate(llp_plan_2):
            #         print(f"Step {i+1}: {step}")
            #         self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=step, instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
            #         print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")


            # #     ##### SPEAK: "I have finished executing the second task."
            #     self.robot.set_speech(filename="gpsr/finished_request2", wait_for_end_of=True)

            # #     # your code here ...

            # # ##execute request3
                                        
            # #     ##### SPEAK: "I will start by executing the third request."
            #     self.robot.set_speech(filename="gpsr/execute_request3", wait_for_end_of=True)
            #     for i, step in enumerate(llp_plan_3):
            #         print(f"Step {i+1}: {step}")
            #         self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=step, instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
            #         print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")

            # #     ##### SPEAK: "I have finished executing the third task."
            #     self.robot.set_speech(filename="gpsr/finished_request3", wait_for_end_of=True)

                self.robot.set_speech(filename="gpsr/end_of_gpsr", wait_for_end_of=True)

                print("Finished LLM GPSR")
                time.sleep(5)

            if self.state == LLM_person_pose:

                self.robot.wait_for_start_button()

                time.sleep(5)

                llp = "go_to_person-pose-pointing left"

                self.curr_room = "living_room"
                self.curr_furniture = "shelf"
                self.curr_result = "NONE"
                self.curr_obj_list =[]
                self.curr_picked_height= 0.0
                self.curr_asked_help = False
                initial_position = [0, 0, 0]

                print(f"Step: {llp}")
                self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=llp, instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
                print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")

                while True:
                    pass

            
            if self.state == LLM_gpsr_llp:

                # self.robot.wait_for_start_button()
                
                # print("New LLM GPSR LLP")

                # start_time = time.time()

                # # robot_pose= self.robot.get_robot_localization()
                # # initial_position = [robot_pose.x, robot_pose.y, robot_pose.theta]
                # initial_position = [0, 0, 0]
                # # print(f"Initial Robot Position: {initial_position}")

                # hlp= self.robot.get_llm_ollama_gpsr_high_level(command= "Go to Anna in the living room", mode="", wait_for_end_of=True)

                # start_llp_time = time.time()

                # llp_output=self.robot.get_llm_ollama_gpsr_low_level(command=hlp[0], mode="", wait_for_end_of=True)

                # self.curr_room = "living_room"
                # self.curr_furniture = "shelf"
                # self.curr_result = "NONE"
                # self.curr_obj_list =[]
                # self.curr_picked_height= 0.0
                # self.curr_asked_help = False

                # for i, step in enumerate(llp_output):
                #     print(f"Step {i+1}: {step.strip()}")
                #     self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=step.strip(), instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
                #     print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")

                # end_time = time.time()
                # print(f"Total Time taken for GPSR task: {end_time - start_time}")
                # print(f"Total Time taken for HLP task: {start_llp_time - start_time}")
                # print(f"Total Time taken for LLP task: {end_time - start_llp_time}")

                # self.robot.set_rgb(command=GREEN+SET_COLOUR, wait_for_end_of=True)

                # #================= Init =================
                # self.llps = []
                # USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS = True

                # self.robot.calibrate_audio()

                # ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                # self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)

                # #================= Get_commands & generate plans =================
                # self.curr_request=1

                # while self.curr_request < 3:
                #     self.curr_request+=1
                
                #     llp = self.robot.receive_command_and_generate_low_level_planner(use_touchscreen_for_yes_no_questions=USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS)
                #     print("Low-level planner " + str(self.curr_request) + ": " + llp)
                #     self.llps.append(llp)

                #     # checks if there is another command to be received, if not proceeds to deciding and executing the commands in the list
                #     if not USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS:
                #         confirmation = self.robot.get_audio(yes_or_no=True, question="gpsr/do_you_have_another_command", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                #     else: # if touchscreen is used
                #         answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Do you have another command?", speak_results=False, start_speak_file="gpsr/do_you_have_another_command", wait_for_end_of=True)
                #         confirmation = answer[0]

                #================= Set_command_order =================
                order_to_execute = []
                plan_feasibility = []

                self.llps = [
                    ["move_to-shelf",
                    "pick_object-cup",
                    "move_to-table",
                    "place_object"],

                    ["move_to-location1",
                    "compare_objects-heaviest-drinks",
                    "move_to-location1",
                    "say_result"],

                    ["go_to_person-name-john",
                    "ERROR",
                    "move_to-kitchen",
                    "ERROR",],

                    ["move_to-location1",
                    "compare_objects-heaviest-drinks",
                    "move_to-location1",
                    "follow_person-living_room"],
                ]

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
                    order_to_execute.insert(i, plan)


                for plan in order_to_execute:

                    for i, plan_check in enumerate(self.llps):

                        print(plan)
                        print(plan_check)

                        if plan == self.llps[i]:
                            
                            print("I will start executing request number"+str(i+1))
                            # self.robot.set_speech(filename="gpsr/execute_request"+(i+1), wait_for_end_of=True)

                    initial_position=[0, 0, 0]
                    # self.robot.execute_gpsr_plan(command=plan, instruction_point=initial_position)


                print("Finished LLM GPSR LLP")
                while True:
                    pass

            if self.state == LLM_hri:

                print("New LLM HRI")
                ##  TESTING INFO EXTRACTION STANDARDFUNCTION ##
                #self.robot.set_speech(filename="receptionist/get_name_and_drink", wait_for_end_of=True)
                # command = self.robot.get_audio(gpsr=True, question="receptionist/get_name_and_drink", wait_for_end_of=True)
                command = "Milk, iced tea, tennis ball, dice"

                heaviest_object = self.robot.get_llm_ollama_information(command, mode="smallest object", wait_for_end_of=True)
                # favorite_drink = self.robot.get_llm_ollama_information(command, mode="favorite drink", wait_for_end_of=True)
                print ("Heaviest object:"+ heaviest_object)
                # print ("GUEST INFO- name:"+name+", favorite drink:"+favorite_drink)
                # self.robot.save_speech(command=name, filename=name, quick_voice=False, wait_for_end_of=True)
                # self.robot.save_speech(command=favorite_drink, filename=favorite_drink, quick_voice=False, wait_for_end_of=True)

                # self.robot.set_speech(filename="temp/"+name, wait_for_end_of=False)
                # self.robot.set_speech(filename="temp/"+favorite_drink, wait_for_end_of=False)

                ##  END OF TESTING INFO EXTRACTION STANDARDFUNCTION ##
                print("Finished LLM HRI")
                while True:
                    pass
            
            if self.state == LLM_Ollama_first_tests:

                # resp = self.robot.get_llm_ollama_demonstration(command="Test D", mode="HRI X", wait_for_end_of=True)
                # print(resp)
                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Hi my name is Sarah I like orange juice", mode="name", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Hi my name is Sarah I like orange juice", mode="favorite drink", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="I'm David and I drink coffee", mode="name", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="I'm David and I drink coffee", mode="favorite drink", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")


                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Hello im Jhon and i like capachino", mode="name", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Hello im Jhon and i like capachino", mode="favorite drink", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Name is Maik and favorite drink ice tree", mode="name", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                start_time = time.time()
                resp = self.robot.get_llm_ollama_information(command="Name is Maik and favorite drink ice tree", mode="favorite drink", wait_for_end_of=True)
                end_time = time.time()
                print(f"Response: {resp}, Time taken: {end_time - start_time}")

                # resp = self.robot.get_llm_ollama_gpsr_high_level(command="Test H", mode="HRI X", wait_for_end_of=True)
                # print(resp)

                # resp = self.robot.get_llm_ollama_gpsr_low_level(command="Test L", mode="HRI X", wait_for_end_of=True)
                # print(resp)

                while True:
                    pass

            elif self.state == Test_individual_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr:

                ### 2 OPTIONS: MUSIC, OR PERIODIC SPEAKS

                request = "Bring me the milk from the coffee table." 
                request2 = "Bring me the wine from the coffee table." 
                request3 = "Bring me the beer from the coffee table." 

                ### LOGIC FOR LLM w/ MUSIC
                # self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)
                """ hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=False)
                self.robot.set_speech(filename="music/music_soy_el_fuego", breakable_play=True, wait_for_end_of=False)
                
                while not self.robot.get_llm_ollama_gpsr_high_level_is_done():
                    print("checking llm")
                    time.sleep(0.05)
                
                self.robot.set_speech(filename="generic/yes", break_play=True)
                command = self.robot.node.llm_ollama_gpsr_high_level_response[0] """

                hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request2, mode="", wait_for_end_of=True)

                ### LOGIC FOR LLM w/ PERIODIC SPEAKS
                # self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)
                hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=False)
                
                start_time = time.time()
                speak_period = 3.0
                self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                while not self.robot.get_llm_ollama_gpsr_high_level_is_done():
                    print("checking llm")
                    if time.time() - start_time >= speak_period:
                        start_time = time.time()
                        self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                    time.sleep(0.05)
                    
                self.robot.set_speech(filename="generic/yes", break_play=True)
                command = self.robot.node.llm_ollama_gpsr_high_level_response[0]


                hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request3, mode="", wait_for_end_of=True)

                ### LOGIC FOR SAVE FILE W/ MUSIC
                # SAVE SPEAKER: WFEO FALSE
                # SPEAK: MUSIC WITH BREAKABLE PLAY AS TRUE
                # WHILE not self.robot.speaker_save_is_done()
                #     time.sleep(0.05)
                # SPEAK: w/ BREAK PLAY AS TRUE                    
                """ 
                # command = "I will move to the kitchen, I will search for the cabinet. I will move to the cabinet. I will search for the milk. If found I will pick the milk. I will move to the instruction point. I will give you the milk."
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                self.robot.set_speech(filename="music/music_soy_el_fuego", breakable_play=True, wait_for_end_of=False)
                
                while not self.robot.save_speech_is_done():
                    print("checking")
                    time.sleep(0.05)
                
                self.robot.set_speech(filename="generic/yes", break_play=True)
                self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True) """
            
                ### LOGIC FOR SAVE FILE WITH PERIODIC SPEAKS
                # command = "I will move to the kitchen, I will search for the cabinet. I will move to the cabinet. I will search for the milk. If found I will pick the milk. I will move to the instruction point. I will give you the milk."
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                
                start_time = time.time()
                speak_period = 3.0
                self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                while not self.robot.save_speech_is_done():
                    print("checking save speak")
                    if time.time() - start_time >= speak_period:
                        start_time = time.time()
                        self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                    time.sleep(0.05)
                
                self.robot.set_speech(filename="generic/yes", break_play=True)
                self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)
            
                while True:
                    pass
   
            elif self.state == Test_together_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr:

                MUSIC_OR_PERIODIC_SPEAKS = "periodic_speaks" # OPTIONS: "music", "periodic_speaks"
                MUSIC_OR_PERIODIC_SPEAKS = MUSIC_OR_PERIODIC_SPEAKS.replace(" ","_").lower()
                request = "Bring me the milk from the coffee table."
                periodic_speak_period = 6.0
                music_file = "music/music_soy_el_fuego"
                periodic_speak_file = "generic/waiting_start_button"

                
                hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=False)
                if MUSIC_OR_PERIODIC_SPEAKS == "music":
                    self.robot.set_speech(filename=music_file, breakable_play=True, wait_for_end_of=False)
                else:
                    self.robot.set_speech(filename=periodic_speak_file, breakable_play=True, wait_for_end_of=False)
                    start_time = time.time()
                while not self.robot.get_llm_ollama_gpsr_high_level_is_done():
                    print("checking llm")
                    if MUSIC_OR_PERIODIC_SPEAKS != "music":
                        if time.time() - start_time >= periodic_speak_period:
                            start_time = time.time()
                            self.robot.set_speech(filename=periodic_speak_file, breakable_play=True, wait_for_end_of=False)
                    time.sleep(0.05)
                command = self.robot.node.llm_ollama_gpsr_high_level_response[0]
                print(command)
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                while not self.robot.save_speech_is_done():
                    print("checking save speaker")
                    if MUSIC_OR_PERIODIC_SPEAKS != "music":
                        if time.time() - start_time >= periodic_speak_period:
                            start_time = time.time()
                            self.robot.set_speech(filename=periodic_speak_file, breakable_play=True, wait_for_end_of=False)
                    time.sleep(0.05)
                self.robot.set_speech(filename="generic/yes", break_play=True)
                self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)


                while True:
                    pass




                """ if MUSIC_OR_PERIODIC_SPEAKS == "music":

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=False)
                    self.robot.set_speech(filename="music/music_soy_el_fuego", breakable_play=True, wait_for_end_of=False)
                    while not self.robot.get_llm_ollama_gpsr_high_level_is_done():
                        print("checking llm")
                        time.sleep(0.05)
                    command = self.robot.node.llm_ollama_gpsr_high_level_response[0]
                    print(command)
                    current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                    self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                    while not self.robot.save_speech_is_done():
                        print("checking save speaker")
                        time.sleep(0.05)
                    self.robot.set_speech(filename="generic/yes", break_play=True)
                    self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                else: # if MUSIC_OR_PERIODIC_SPEAKS == "periodic_speaks":

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=False)
                    start_time = time.time()
                    self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                    while not self.robot.get_llm_ollama_gpsr_high_level_is_done():
                        print("checking llm")
                        if time.time() - start_time >= periodic_speak_period:
                            start_time = time.time()
                            self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                        time.sleep(0.05)
                    command = self.robot.node.llm_ollama_gpsr_high_level_response[0]
                    print(command)
                    current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                    self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                    while not self.robot.save_speech_is_done():
                        print("checking save speaker")
                        if time.time() - start_time >= periodic_speak_period:
                            start_time = time.time()
                            self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                        time.sleep(0.05)
                    self.robot.set_speech(filename="generic/yes", break_play=True)
                    self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                while True:
                    pass """


            elif self.state == Final_State:
                
                self.state += 1
                print("Finished task!!!")

            else:
                pass