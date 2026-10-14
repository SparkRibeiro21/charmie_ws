#!/usr/bin/env python3
import rclpy
import threading
import time
from datetime import datetime
from pathlib import Path
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
        LLM_gpsr_hlp= 3
        LLM_gpsr_llp= 4
        LLM_hri= 5
        LLM_Ollama_first_tests = 6
        Test_individual_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr = 7
        Test_together_save_speaker_and_llm_with_periodic_updates_from_robot_for_gpsr = 8
        Final_State = 8

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

                print("New LLM GPSR Demo")


                commands = [
                "Please identify which object on the table is the lightest."
                ]

                for i, command in enumerate(commands):

                    print(f"============== {i}/{len(commands)}")

                    # start_time = time.time()

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

                    task_type=self.robot.get_task_type(task=llp_output)
                    corrected_llp=self.robot.correct_task_plan(raw_task_plan=llp_output,task_type=task_type)

                    print(f"CORRECTED LLP: {corrected_llp}")
                    print(f"TASK TYPE: {task_type}")


                    # print(f"Total Time taken for GPSR task {i}: {end_time - start_time:.3f} s")
                    # print(f"Total Time taken for HLP task {i}: {start_llp_time - start_time:.3f} s")
                    # print(f"Total Time taken for LLP task {i}: {end_time - start_llp_time:.3f} s")


                print("Finished LLM GPSR Demo")
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


                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)

                ## Receive command and generate both high-level plan and low-level plan
                self.curr_request+=1
                
                llp = self.robot.receive_command_and_generate_low_level_planner(command_no=self.curr_request, use_touchscreen_for_yes_no_questions=USE_TOUCHSCREEN_FOR_YES_NO_QUESTIONS)
                print("Low-level planner " + str(self.curr_request) + ": ", llp)
                self.llps.append(llp)

                corrected_plans_list=[]
                
                for plan in self.llps:
                    task_type=self.robot.get_task_type(task=plan)
                    corrected_plan=self.robot.correct_task_plan(raw_task_plan=plan, task_type=task_type)
                    print("Corrected plan", corrected_plan)
                    print("task_type", task_type)
                    corrected_plans_list.append(corrected_plan)
                
                self.order_to_execute = self.robot.set_order_to_execute_gpsr_plan(corrected_plans_list)

                for plan in self.order_to_execute:

                    for i, plan_check in enumerate(self.llps):

                        curr_plan = i +1

                        print(plan)
                        print(plan_check)

                        if plan == self.llps[i]:

                            print("I will start executing request number"+str(curr_plan))
                            self.robot.set_speech(filename="gpsr/execute_request"+str(i+1), wait_for_end_of=True)

                    self.robot.execute_gpsr_plan(command=plan, instruction_point=self.instruction_point)


                print("Finished LLM GPSR")
                time.sleep(5)

            if self.state == LLM_gpsr_hlp:

                print("New LLM GPSR HLP")

                use_touchscreen_for_yes_no_questions=True
                command_no=1

                current_datetime = ""
                hlp_comm = ""
                l_command = [] # can only be used after the .split(";")
                got_hlp = False

                def check_received_hlp_and_save_speaker_sentences(): # checks if hlp has been received and sends all sentence speak files
                    nonlocal current_datetime, hlp_comm, l_command, got_hlp
                    if not got_hlp and self.robot.get_llm_ollama_gpsr_high_level_is_done():
                        print("CHECKED RECEIVE HLP - STARTING TO GENERATE SPEECH FILES")
                        hlp_comm = self.node.llm_ollama_gpsr_high_level_response[0]
                        if hlp_comm.endswith(";"): # edge case where hlp ended with ; . This made a sentence be empty and thus not generated, which maskes the checks if can do speaks block because it is waiting for file to be created
                            hlp_comm = hlp_comm[:-1]
                        l_command = hlp_comm.split(";")
        
                        # SAVE ALL SPEAKS
                        for index, value in enumerate(l_command):
                            # print("hlp_" + current_datetime + "_" + str(index))
                            self.robot.save_speech(command=value, filename="hlp_" + current_datetime + "_" + str(index), quick_voice=True, play_command=False, show_in_face=False, wait_for_end_of=False)
        
                        got_hlp = True

                def speech_file_is_ready(filename): # helper to check if file exists, wav + txt because txt is created after wav, which means wav is not under construction
                            temp_path = (
                                Path.home()
                                / "charmie_ws"
                                / "src"
                                / "charmie_speakers"
                                / "charmie_speakers"
                                / "list_of_sentences"
                                / "temp"
                            )
                            wav_path = Path(temp_path) / f"{filename}.wav"
                            txt_path = Path(temp_path) / f"{filename}.txt"
                            return wav_path.is_file() and txt_path.is_file()
                
                command_confirmed = False
                max_characters = 250

                ##### SPEAK: "What is your request?"
                gpsr_command = self.robot.get_audio(gpsr=True, question="gpsr/gpsr_question_2", face_hearing="charmie_face_green", wait_for_end_of= True)
                print("Finished:", gpsr_command)
    
                ##### SPEAK: "Please give me a moment to process your command"
                self.robot.set_speech(filename="gpsr/gpsr_process_command", wait_for_end_of=False)
                
                if len(gpsr_command) > max_characters:
                    self.robot.set_speech(filename="gpsr/could_not_process", wait_for_end_of=True)
                else:
    
                    self.robot.get_llm_ollama_gpsr_high_level(command=gpsr_command, mode="", wait_for_end_of=False)
                
                    self.robot.set_face(loadbar=10.0, command=gpsr_command)
                    self.robot.save_speech(command= gpsr_command, filename="gpsr_command", quick_voice=True, wait_for_end_of=True)
                    self.robot.set_face("charmie_face")
                    check_received_hlp_and_save_speaker_sentences() # if hlp has been received, starts creating the sentences speak files
    
                    ##### SPEAK: "I have understood the following command."
                    self.robot.set_speech(filename="gpsr/check_command", wait_for_end_of=True)
                    check_received_hlp_and_save_speaker_sentences() # if hlp has been received, starts creating the sentences speak files
                    self.robot.set_speech(filename="temp/gpsr_command", show_in_face=True, wait_for_end_of=True)
                    check_received_hlp_and_save_speaker_sentences() # if hlp has been received, starts creating the sentences speak files
                    
                    if not use_touchscreen_for_yes_no_questions:
                        ##### SPEAK: "Is the command correct? Please say yes robot, or no robot to confirm."
                        self.robot.set_speech(filename="gpsr/confirm_command", wait_for_end_of= True)
                        confirmation = self.robot.get_audio(yes_or_no=True, question="generic/say_robot_yes_no", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    else: # if touchscreen is used
                        if command_no == 1: # for time efficiency, only inform user that needs to press the face for the first command
                            self.robot.set_speech(filename="generic/press_correct_option_touchscreen", wait_for_end_of=True) # SAY: Please press the correct option on my face.
                        answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Is this command correct?", speak_results=False, speak_timeout=False, start_speak_file="gpsr/confirm_command", wait_for_end_of=True)
                        confirmation = answer[0]
                    
                    # worst case scenario, it waits until is received here, 
                    while not got_hlp:
                        check_received_hlp_and_save_speaker_sentences() # if hlp has been received, starts creating the sentences speak files
                        time.sleep(0.05)
                    
                    # print("confirmation:", confirmation)
                    if confirmation.lower() == "yes":
                        self.robot.set_rgb(command=GREEN+BLINK_LONG)
                        command_confirmed = True
                    else: # "no" or "TIMEOUT"
                        self.robot.set_speech(filename="generic/sorry_for_my_mistake_lets_try_again", wait_for_end_of=True) # SAY: Sorry for my mistake let's try again

                    
                    # time_efficient_high_level_sentence_saver()
                    self.robot.set_speech(filename="gpsr/say_plan", wait_for_end_of= True)
                    for index, value in enumerate(l_command):
                        while not speech_file_is_ready("hlp_" + current_datetime + "_" + str(index)):
                            time.sleep(0.05)
                        self.robot.set_speech(filename="temp/hlp_" + current_datetime + "_" + str(index), wait_for_end_of=True)
            

                print("Finished LLM GPSR HLP")

                while True:
                    pass

            if self.state == LLM_gpsr_llp:

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
                command = "right,right,right,right,right,right,right,right,right,right,right,right,right,right,right,right,right,right,right"

                # heaviest_object = self.robot.get_llm_ollama_information(command, mode="smallest object", wait_for_end_of=True)
                name = self.robot.get_llm_ollama_information(command, mode="name", wait_for_end_of=True)
                favorite_drink = self.robot.get_llm_ollama_information(command, mode="favorite drink", wait_for_end_of=True)
                # print ("Heaviest object:"+ heaviest_object)
                print ("GUEST INFO- name:"+name+", favorite drink:"+favorite_drink)
                # self.robot.save_speech(command=name, filename=name, quick_voice=False, wait_for_end_of=True)
                # self.robot.save_speech(command=favorite_drink, filename=favorite_drink, quick_voice=False, wait_for_end_of=True)

                # self.robot.set_speech(filename="temp/"+name, wait_for_end_of=False)
                # self.robot.set_speech(filename="temp/"+favorite_drink, wait_for_end_of=False)

                ##  END OF TESTING INFO EXTRACTION STANDARDFUNCTION ##
                print("Finished LLM HRI")

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