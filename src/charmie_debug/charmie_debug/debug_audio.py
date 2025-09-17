#!/usr/bin/env python3
import rclpy
import threading
import time
from datetime import datetime
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import random

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              False,
    "charmie_audio":            True,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_base_camera":      False,
    "charmie_gamepad":          False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_lidar_livox":      False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             False,
    "charmie_radar":            False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     False,
    "charmie_yolo_pose":        False,
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
        Audio_receptionist = 1
        Audio_restaurant = 2
        Audio_egpsr = 3
        Continuous_audio = 4
        Calibrate_audio = 5
        Sound_classification = 6
        Continuous_sound_classification = 7
        Final_State = 8

        # VARS ...
        self.state = Continuous_audio
    
        self.robot.set_face("charmie_face")
        print("IN NEW MAIN")
        
        while True:

            if self.state == Audio_receptionist:
                print('State 1 = Audio Receptionist')

                ### RECEPTIONIST EXAMPLE
                print("Started")
                self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                command = self.robot.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
                print("Finished:", command)
                
                if command == "ERR_MAX":
                    print("MAX HEARING ATTEMPTS REACHED")
                    self.robot.set_speech(filename="generic/could_not_hear_max_attempts", wait_for_end_of=True)
                else:
                    keyword_list= command.split(" ")
                    guest_name = keyword_list[0] 
                    guest_drink = keyword_list[1]
                    print(guest_name, guest_drink)

                    # self.robot.set_speech(filename="receptionist/dear", wait_for_end_of=True)
                    # self.robot.set_speech(filename="person_names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="receptionist/first_guest_name_is", wait_for_end_of=True)
                    self.robot.set_speech(filename="person_names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
                    # self.robot.set_speech(filename="receptionist/second_guest_name_is", wait_for_end_of=True)
                    # self.robot.set_speech(filename="person_names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
                    # self.robot.set_speech(filename="receptionist/host_name_is", wait_for_end_of=True)
                    # self.robot.set_speech(filename="person_names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
                    # self.robot.set_speech(filename="person_names/recep_first_guest_"+keyword_list[0].lower(), wait_for_end_of=True)
                    self.robot.set_speech(filename="receptionist/favourite_drink_is", wait_for_end_of=True)
                    self.robot.set_speech(filename="objects_names/"+keyword_list[1].lower(), wait_for_end_of=True)

                time.sleep(5)
                
            if self.state == Audio_restaurant:
                print('State 2 = Audio Restaurant')

                ### RESTAURANT EXAMPLE
                is_command_confirmed = False
                while not is_command_confirmed:
                    self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    command = self.robot.get_audio(restaurant=True, question="restaurant/what_is_your_order", face_hearing="charmie_face_green_my_order", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list= command.split(" ")
                    self.robot.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                    for kw in keyword_list:
                        print(kw)
                        self.robot.set_speech(filename="objects_names/"+kw.lower(), wait_for_end_of=True)

                    ##### AUDIO: Listen "YES" OR "NO"
                    ##### "Please say yes or no to confirm the order"
                    confirmation = self.robot.get_audio(yes_or_no=True, question="restaurant/yes_no_question", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        # self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                        self.robot.set_rgb(command=GREEN+BLINK_LONG)

                        self.robot.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                        for kw in keyword_list:
                            print(kw)
                            self.robot.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                        ##### SPEAK: Thank you
                        # self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
                        is_command_confirmed = True

                    else: #  confirmation.lower() == "no":
                        self.robot.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry, TRY AGAIN
                        self.robot.set_speech(filename="restaurant/no_order", wait_for_end_of=True)

                time.sleep(5)
                
            if self.state == Audio_egpsr:
                print('State 3 = Audio EGPSR')

                ### EGPSR EXAMPLE
                is_command_confirmed = False
                while not is_command_confirmed:

                    ### GPSR EXAMPLE
                    self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    ##### SPEAK: "Hello, you seem to be needing my help. How can I help you?"
                    audio_gpsr_command = self.robot.get_audio(gpsr=True, question="gpsr/gpsr_question", wait_for_end_of=True)
                    print("Finished:", audio_gpsr_command)

                    ##### SPEAK: "Please give me a moment to process your command"
                    self.robot.set_speech(filename="gpsr/gpsr_process_command", wait_for_end_of=True)
                    
                    ##### SAVE SPEAK
                    current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                    self.robot.save_speech(command=audio_gpsr_command, filename=current_datetime)
                    
                    ##### SPEAK: "I have understood the following command."
                    self.robot.set_speech(filename="gpsr/check_command", wait_for_end_of=True)

                    ##### SPEAK: (repeats the command)
                    self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)
                    
                    confirmation = self.robot.get_audio(yes_or_no=True, question="gpsr/confirm_command", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.robot.set_rgb(command=GREEN+BLINK_LONG)

                        ##### SPEAK: "Good. I have successfully understood your command. The requested command is."
                        self.robot.set_speech(filename="gpsr/sucess_hearing_command", wait_for_end_of=True)

                        ##### SPEAK: (repeats the command)
                        self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                        self.robot.set_rgb(command=WHITE+ROTATE)
                        
                        ##### SPEAK: "Please give me a minute to analyse the whole task. Divide in subtasks. And analyse the feasibility of all subtasks."
                        self.robot.set_speech(filename="gpsr/analyse_command", wait_for_end_of=True)
                        time.sleep(2.0)

                        command_feasibility = False

                        if command_feasibility:
                            self.robot.set_rgb(command=GREEN+BREATH)
                        else:
                            self.robot.set_rgb(command=RED+BREATH)

                            ##### SPEAK: "I have successfully understood the command."
                            self.robot.set_speech(filename="gpsr/understood_command", wait_for_end_of=True)
                        
                            ##### SPEAK: (repeats the command)
                            self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                            ##### SPEAK: "But unfortunately the task you require me to do, has some parts that I still need to learn how to perform. So, I am unable to help you with this task. I am sorry for this. I will keep moving and search for new commands."
                            self.robot.set_speech(filename="gpsr/can_not_perform_command", wait_for_end_of=True)
                        
                        is_command_confirmed = True

                    else:
                        self.robot.set_rgb(command=RED+BLINK_LONG)
                        ##### SPEAK: Sorry for my mistake, lets try again.
                        self.robot.set_speech(filename="gpsr/no_order", wait_for_end_of=True)
                    
                time.sleep(5)

            if self.state == Continuous_audio:

                ### CONTINUOUS AUDIO EXAMPLE
                ### WAIT FOR END OF = TRUE
                s, m, detected_keyword = self.robot.get_continuous_audio(keywords=["stop", "finish", "end"], max_number_attempts=3, speak_pre_hearing=True, speak_post_hearing=True, wait_for_end_of=True)
                print(s, m, detected_keyword)

                ### WAIT FOR END OF = FALSE
                # s, m, detected_keyword = self.robot.get_continuous_audio(keywords=["stop", "finish", "end"], max_number_attempts=3, speak_pre_hearing=True, speak_post_hearing=True, wait_for_end_of=False)
                # print(s, m, detected_keyword)
                # message_received = False
                # while not message_received:
                #     message_received, s, m, detected_keyword = self.robot.is_get_continuous_audio_done(speak_post_hearing=True)
                #     print(message_received, s, m, detected_keyword)
                #     time.sleep(0.5)
                
                print("Continuous Audio Mode Done")
                
                # next state
                self.state = Final_State
                
            if self.state == Calibrate_audio:

                ### CALIBRATION EXAMPLE
                s, m = self.robot.calibrate_audio(wait_for_end_of=True)
                print("Finished:", s, m)
                
                # next state
                self.state = Final_State

            if self.state == Sound_classification:
                print('State 6 = Sound Classification')

                labels, scores = self.robot.get_sound_classification(question="sound_classification/sound_classification_start_"+str(random.randint(1, 6)), duration=3.0, wait_for_end_of=True)
                
                print("Heard Sounds:")
                print("Scores:\tLabels:")
                for l, s in zip(labels, scores):
                    print(f"{s:.2f} - {l}")
                print("\n")

                time.sleep(5)

            if self.state == Continuous_sound_classification:
                print('State 7 = Sound Classification Continuous')

                ### CONTINUOUS SOUND CLASSIFICATION EXAMPLE
                # WAIT FOR END OF = TRUE
                # s, m, label, score = self.robot.get_continuous_sound_classification(break_sounds=["finger snapping", "Whistling"], timeout=10, wait_for_end_of=True)
                # print(s, m, label, score)
                # if m.lower() == "timeout":
                #     print("TIMEOUT REACHED")

                # WAIT FOR END OF = FALSE
                # self.robot.get_continuous_sound_classification(break_sounds=["finger snapping", "Whistling"], timeout=0, wait_for_end_of=False)
                # message_received = False
                # while not message_received:
                    # message_received, s, m, label, score = self.robot.is_get_continuous_sound_classification_done()
                    # print(message_received, s, m, label, score)
                    # time.sleep(0.5)
                
                print("Continuous Sound Classification Mode Done")
                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass