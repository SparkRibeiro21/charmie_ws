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
    "charmie_arm":              False,
    "charmie_audio":            True,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_lidar":            False,
    "charmie_llm":              True,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
    "charmie_point_cloud":      False,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         True,
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
        Final_State = 6

        # VARS ...
        self.state = Audio_receptionist
    
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

            if self.state == Audio_receptionist:

                self.robot.get_llm_gpsr()
                
                # self.robot.get_llm_demonstration()

                while True:
                    pass
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
                    print(keyword_list[0], keyword_list[1])
                    self.robot.set_speech(filename="receptionist/names/recep_first_guest_"+keyword_list[0].lower(), wait_for_end_of=True)
                    self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_"+keyword_list[1].lower(), wait_for_end_of=True)

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

            
            elif self.state == Continuous_audio:

                ### CONTINUOUS AUDIO EXAMPLE
                # WAIT FOR END OF = TRUE
                s, m = self.robot.get_continuous_audio(keywords=["stop", "finish", "end"], max_number_attempts=3, wait_for_end_of=True)
                print(s, m)

                # WAIT FOR END OF = FALSE
                # s, m = self.robot.get_continuous_audio(keywords=["stop", "finish", "end"], max_number_attempts=3, wait_for_end_of=False)
                # print(s, m)
                
                # message_received = False
                # while not message_received:
                #     message_received, s, m = self.robot.is_get_continuous_audio_done()
                #     print(message_received, s, m)
                #     time.sleep(0.5)
                
                print("Continuous Mode Audio Done")
                
                while True:
                    pass
                
            elif self.state == Calibrate_audio:

                ### CALIBRATION EXAMPLE
                s, m = self.robot.calibrate_audio(wait_for_end_of=True)
                print("Finished:", s, m)
                
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