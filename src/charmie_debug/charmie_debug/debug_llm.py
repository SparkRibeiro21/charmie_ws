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
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          False,
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
        self.state = LLM_gpsr_llp

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

                start_time = time.time()

                hlp= self.robot.get_llm_ollama_gpsr_high_level(command= "Bring me the milk from the coffee table", mode="", wait_for_end_of=True)

                start_llp_time = time.time()

                llp_output=self.robot.get_llm_ollama_gpsr_low_level(command=hlp[0], mode="", wait_for_end_of=True)

                end_time = time.time()


                print(f"Total Time taken for 1st GPSR task: {end_time - start_time}")
                print(f"Total Time taken for 1st HLP task: {start_llp_time - start_time}")
                print(f"Total Time taken for 1st LLP task: {end_time - start_llp_time}")


                print("Finished LLM GPSR LLP")
                while True:
                    pass
                
            if self.state == LLM_gpsr:

                print("New LLM GPSR")
                # self.robot.get_llm_gpsr()

            ##Receive guest1
                
                ##### SPEAK: "Hello! My name is Charmie and I am here to help you with whatever you need."
                print("SPEAK: 'Hello! My name is Charmie and I am here to help you with whatever you need.'")
                self.robot.set_speech(filename="gpsr/gpsr_intro", wait_for_end_of=True)

                self.curr_request = 1
                # your code here ...

                # Look at the judge
                # self.robot.set_neck
                print("Getting command")
                request = self.robot.get_llm_confirm_command()

                # request = "Move to the living room and pick the milk"

                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    self.request1 = request
                    print("Request " + str(self.curr_request) + ": " + request)

                    ##### SPEAK: "Okay, I understood your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)
                    #TODO: get low-level planner with wfeo a falso
                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request"+str(self.curr_request), quick_voice=True ,wait_for_end_of=False)
                    
                    self.request1 = hlp_request

            ##Receive guest2
                
                ##### SPEAK: "Let's move on to the second request"
                #

                self.curr_request = 2

                # Look at the judge
                # self.robot.set_neck
                
                request = self.robot.get_llm_confirm_command()

                # request = "Go to the bedroom and then move to the bedside table"

                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    self.request2 = request
                    print("Request " + str(self.curr_request) + ": " + request)

                    ##### SPEAK: "Okay, I understood your curr_request request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)
                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request"+str(self.curr_request), quick_voice=True ,wait_for_end_of=False)
                    self.request2 = hlp_request

            ##Receive guest3
            
                self.curr_request = 3
                # your code here ...

                # Look at the judge
                # self.robot.set_neck
                
                request = self.robot.get_llm_confirm_command()

                # request = "Move to the kitchen"

                if request == "ERROR":
                    print("Error in request " + str(self.curr_request))
                    ##### SPEAK: "I was not able to understand your request. Let's move on."
                    self.robot.set_speech(filename="gpsr/unsucessful_hearing_command", wait_for_end_of=True)

                else:
                    # Save current request
                    self.request3 = request
                    print("Request " + str(self.curr_request + 1) + ": " + request)

                    ##### SPEAK: "Okay, I understood your curr_request request. Let's move on."
                    self.robot.set_speech(filename="gpsr/sucessful_hearing_command", wait_for_end_of=False)

                    hlp_request = self.robot.get_llm_ollama_gpsr_high_level(command=request, mode="", wait_for_end_of=True)
                    self.robot.save_speech(command=hlp_request[0], filename="gpsr_request"+str(self.curr_request), quick_voice=True ,wait_for_end_of=False)
                    self.request3 = hlp_request

            ##show plans 

                ##### SPEAK: "Please give me a moment while I proccess your requests"
                self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=True)
                
                ##### SPEAK: "To execute the first request"  
                self.robot.set_speech(filename="gpsr/say_plan1", wait_for_end_of= True)
                self.robot.set_speech(filename="temp/gpsr_request1", wait_for_end_of= True)

                
                ##### SPEAK: "To execute the second request"  
                self.robot.set_speech(filename="gpsr/say_plan2", wait_for_end_of= True)
                self.robot.set_speech(filename="temp/gpsr_request2", wait_for_end_of= True)
                
                ##### SPEAK: "To execute the first request"  
                self.robot.set_speech(filename="gpsr/say_plan3", wait_for_end_of= True)
                self.robot.set_speech(filename="temp/gpsr_request3", wait_for_end_of= True)
                                        
                # your code here ...
            
                self.robot.set_speech(filename="gpsr/cannot_perform_low_level", wait_for_end_of=True)

            # ##execute guest1

            #     ##### SPEAK: "I will start by executing the first request."
            #     self.robot.set_speech(filename="gpsr/execute_request1", wait_for_end_of=True)
            #     self.robot.execute_gpsr_plan(command=self.request1,wait_for_end_of=True)

            #     ##### SPEAK: "I have finished executing the first task."
            #     self.robot.set_speech(filename="gpsr/finished_request1", wait_for_end_of=True)

                

            # ##execute guest1
                                        
            #     ##### SPEAK: "I will start by executing the second request."
            #     self.robot.set_speech(filename="gpsr/execute_request2", wait_for_end_of=True)

            #     self.robot.execute_gpsr_plan(command=self.request2,wait_for_end_of=True)

            #     ##### SPEAK: "I have finished executing the second task."
            #     self.robot.set_speech(filename="gpsr/finished_request2", wait_for_end_of=True)

            #     # your code here ...

            # ##execute guest1
                                        
            #     ##### SPEAK: "I will start by executing the third request."
            #     self.robot.set_speech(filename="gpsr/execute_request3", wait_for_end_of=True)

            #     self.robot.execute_gpsr_plan(command=self.request3,wait_for_end_of=True)

            #     ##### SPEAK: "I have finished executing the third task."
            #     self.robot.set_speech(filename="gpsr/finished_request3", wait_for_end_of=True)

            #     self.state = self.task_states["Return_to_instruction_point"]
                

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
                
                print("New LLM GPSR LLP")

                start_time = time.time()

                # robot_pose= self.robot.get_robot_localization()
                # initial_position = [robot_pose.x, robot_pose.y, robot_pose.theta]
                initial_position = [0, 0, 0]
                # print(f"Initial Robot Position: {initial_position}")

                hlp= self.robot.get_llm_ollama_gpsr_high_level(command= "Count the number of drinks on the office table and tell the result to the person waving in the kitchen", mode="", wait_for_end_of=True)

                start_llp_time = time.time()

                llp_output=self.robot.get_llm_ollama_gpsr_low_level(command=hlp[0], mode="", wait_for_end_of=True)

                self.curr_room = "living_room"
                self.curr_furniture = "shelf"
                self.curr_result = "NONE"
                self.curr_obj_list =[]
                self.curr_picked_height= 0.0
                self.curr_asked_help = False

                for i, step in enumerate(llp_output):
                    print(f"Step {i+1}: {step.strip()}")
                    self.curr_room, self.curr_furniture, self.curr_result, self.curr_obj_list, self.curr_picked_height, self.curr_asked_help = self.robot.execute_gpsr_plan(command=step.strip(), instruction_point=initial_position, curr_room=self.curr_room, curr_furniture=self.curr_furniture, curr_result=self.curr_result, curr_obj_list=self.curr_obj_list, curr_picked_height=self.curr_picked_height, curr_asked_help=self.curr_asked_help, wait_for_end_of=True)
                    print(f"Updated State - Room: {self.curr_room}, Furniture: {self.curr_furniture}, Result: {self.curr_result}, Object List: {self.curr_obj_list}")

                end_time = time.time()
                print(f"Total Time taken for GPSR task: {end_time - start_time}")
                print(f"Total Time taken for HLP task: {start_llp_time - start_time}")
                print(f"Total Time taken for LLP task: {end_time - start_llp_time}")

                self.robot.set_rgb(command=GREEN+SET_COLOUR, wait_for_end_of=True)

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