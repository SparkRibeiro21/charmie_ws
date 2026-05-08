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
    "charmie_llm":                  False,
    "charmie_localisation":         False,
    "charmie_low_level":            False,
    "charmie_navigation":           False,
    "charmie_nav2":                 False,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
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

    # main state-machine function
    def main(self):
        # Waiting_for_start_button = 0
        LLM_demo = 1
        LLM_gpsr = 2
        LLM_hri= 3
        LLM_Ollama_first_tests = 4
        Test_speaks_with_breaks_for_gpsr = 5
        Final_State = 6

        # VARS ...
        self.state = Test_speaks_with_breaks_for_gpsr

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

                # self.robot.get_llm_ollama_gpsr_high_level(command="greet the person wearing a red sweater in the bedroom and guide them to the waste basket", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="find a cleaning supply in the bathroom then take it and deliver it to the sitting person in the bathroom", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="navigate to the bed then locate the sitting person and follow them to the kitchen", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="look for a red wine in the living room then get it and put it on the bed", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="tell me how many drinks there are on the shelf", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="locate a soccer ball in the office then take it and deliver it to Charlie in the office", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="go to the office then find the person pointing to the left and say your teams name", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="introduce yourself to Jane in the bedroom and follow them", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="greet Morgan in the living room and tell the day of the week", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="tell me what is the largest toy on the refrigerator", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="go to the living room then look for a fruit and fetch it and place it on the refrigerator", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="say your teams country to the person raising their left arm in the bathroom", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="go to the bathroom then find the standing person and tell the time", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="follow the waving person in the living room", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="tell me how many dishes there are on the sofa", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="locate a cleanser in the office then get it and put it on the side tables", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="tell me how many toys there are on the side tables", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="bring me a mustard from the refrigerator", mode="", wait_for_end_of=True)
                # self.robot.get_llm_ollama_gpsr_high_level(command="escort the person wearing a black shirt from the chairs to the chairs", mode="", wait_for_end_of=True)
 
                gpsr_command = "Hello"
                print("Hello")

                self.robot.save_speech(command= gpsr_command, filename="gpsr_command", quick_voice=True, wait_for_end_of=True)

                print("Hello1")

                self.robot.set_speech(filename="temp/gpsr_command", wait_for_end_of=True)

                print("Hello2")

                print("Finished first LLM Demo")
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
            
            if self.state == LLM_hri:

                print("New LLM HRI")
                ##  TESTING INFO EXTRACTION STANDARDFUNCTION ##
                #self.robot.set_speech(filename="receptionist/get_name_and_drink", wait_for_end_of=True)
                command = self.robot.get_audio(gpsr=True, question="receptionist/get_name_and_drink", wait_for_end_of=True)
                # command = "My name is John and my favorite drink is coffee"

                name = self.robot.get_info_from_llm(command, info_type="name", wait_for_end_of=True)
                favorite_drink = self.robot.get_info_from_llm(command, info_type="favorite drink", wait_for_end_of=True)
                print ("GUEST INFO- name:"+name+", favorite drink:"+favorite_drink)

                ##  END OF TESTING INFO EXTRACTION STANDARDFUNCTION ##
                print("Finished LLM HRI")
                time.sleep(5)
            
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

            elif self.state == Test_speaks_with_breaks_for_gpsr:

                ### 2 OPTIONS: MUSIC, OR PERIODIC SPEAKS

                ### LOGIC FOR LLM


                ### LOGIC FOR SAVE FILE W/ MUSIC
                # SAVE SPEAKER: WFEO FALSE
                # SPEAK: MUSIC WITH BREAKABLE PLAY AS TRUE
                # WHILE not self.robot.speaker_save_is_done()
                #     time.sleep(0.05)
                # SPEAK: w/ BREAK PLAY AS TRUE                    
                """ command = "I will move to the kitchen, I will search for the cabinet. I will move to the cabinet. I will search for the milk. If found I will pick the milk. I will move to the instruction point. I will give you the milk."
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                self.robot.set_speech(filename="music/music_soy_el_fuego", breakable_play=True, wait_for_end_of=False)
                
                while not self.robot.save_speech_is_done():
                    print("checking")
                    time.sleep(0.05)
                
                self.robot.set_speech(filename="generic/yes", break_play=True)
                self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True) """
            
                ### LOGIC FOR SAVE FILE WITH PERIODIC SPEAKS
                command = "I will move to the kitchen, I will search for the cabinet. I will move to the cabinet. I will search for the milk. If found I will pick the milk. I will move to the instruction point. I will give you the milk."
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                self.robot.save_speech(command=command, filename=current_datetime, quick_voice=False, wait_for_end_of=False)
                
                start_time = time.time()
                speak_period = 3.0
                while not self.robot.save_speech_is_done():
                    print("checking")
                    if time.time() - start_time >= speak_period:
                        start_time = time.time()
                        self.robot.set_speech(filename="generic/waiting_start_button", breakable_play=True, wait_for_end_of=False)
                    time.sleep(0.05)
                
                self.robot.set_speech(filename="generic/yes", break_play=True)
                self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)
            
                while True:
                    pass
                                           
            elif self.state == Final_State:
                
                self.state += 1
                print("Finished task!!!")

            else:
                pass