#!/usr/bin/env python3
import rclpy
import threading
import time
import random

from geometry_msgs.msg import Twist
from charmie_interfaces.msg import DetectedObject, DetectedPerson, GamepadController
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_base_camera":      True,
    "charmie_gamepad":          True,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_lidar_livox":      False,
    "charmie_llm":              True,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_radar":            False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     True,
    "charmie_yolo_pose":        True,
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
        
        # Task States
        self.Waiting_for_task_start = 0
        self.Demo_actuators_with_tasks = 1
        self.Demo_actuators_without_tasks = 2
        self.Search_for_objects_demonstration = 3
        self.Search_for_people_demonstration = 4
        self.Introduction_demonstration = 5
        self.Serve_breakfast_demonstration = 6
        self.Audio_receptionist_and_restaurant_demonstration = 7
        self.LLM_demonstration = 8
        self.Open_door = 9
        self.Final_State = 10

        self.TIMEOUT_FLAG = True
        self.MOTORS_ACTIVE_FLAG = True

        self.MAX_LINEAR_SPEED = 0.40  # m/s
        self.MAX_ANGULAR_SPEED = 0.5  # rad/s
        
        self.SB_Waiting_for_task_start = 0
        self.SB_Detect_and_receive_milk = 1
        self.SB_Detect_and_receive_cornflakes = 2
        self.SB_Detect_and_receive_dishes = 3
        self.SB_Place_and_pour_objects = 4

        self.A_Receptionist = 0
        self.A_Restaurant = 1

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_forward_down = [0, -20]
        self.look_judge = [45, 0]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]] # , [-45-10, -45-5], [-45+10, -45-5]]
        # self.look_navigation = [0, -30]
        self.look_table_objects = [-45, -35]
        # self.look_tray = [0, -60]

        self.neck_pos_pan = self.look_forward[0]
        self.neck_pos_tilt = self.look_forward[1]

        self.current_task = 0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start
        self.state_SB = self.SB_Waiting_for_task_start
        self.state_A = self.A_Receptionist

        while True:

            if self.state == self.Waiting_for_task_start:
                # Initialization State

                if ros2_modules["charmie_face"]:
                    self.robot.set_face("charmie_face")

                if ros2_modules["charmie_neck"]:
                    self.robot.set_neck(self.look_forward, wait_for_end_of=True)

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_speakers"]:
                    self.robot.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                # to initially set TIMEOUT FLAGS and RGB 
                gamepad_controller, new_message = self.robot.update_gamepad_controller_state()
                if self.robot.get_gamepad_timeout(self.robot.OFF):
                    self.TIMEOUT_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        self.MOTORS_ACTIVE_FLAG = True
                        self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)
                else:
                    self.TIMEOUT_FLAG = True
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                        self.MOTORS_ACTIVE_FLAG = False
                        self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Demo_actuators_with_tasks:

                gamepad_controller, new_message = self.robot.update_gamepad_controller_state()
                
                # WATCHDOG VERIFICATIONS
                if self.robot.get_gamepad_timeout(self.robot.FALLING):
                    self.TIMEOUT_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(MAGENTA+HALF_ROTATE)
                        # only allow reactivation of motors via LOGO button (SAFETY)
                        # self.MOTORS_ACTIVE_FLAG = True
                        # self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                if self.robot.get_gamepad_timeout(self.robot.RISING):
                    self.TIMEOUT_FLAG = True
                    self.MOTORS_ACTIVE_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                    
                    self.safety_stop_modules()
    
                    if ros2_modules["charmie_speakers"]:
                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)


                if new_message and not self.TIMEOUT_FLAG: # if we receive a new message and there is no timeout
                    
                    # Activate motors: only activates if gamepad controller messages are being received
                    if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_LOGO, self.robot.RISING):

                        if ros2_modules["charmie_low_level"]:

                            self.MOTORS_ACTIVE_FLAG = not self.MOTORS_ACTIVE_FLAG
                            self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                            if self.MOTORS_ACTIVE_FLAG:
                                self.robot.set_rgb(BLUE+HALF_ROTATE)
                                if ros2_modules["charmie_speakers"]:
                                    self.robot.set_speech(filename="demonstration/motors_unlocked", wait_for_end_of=False)
                            else:
                                self.robot.set_rgb(MAGENTA+HALF_ROTATE)
                                if ros2_modules["charmie_speakers"]:
                                    self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)
                            
                    if self.MOTORS_ACTIVE_FLAG: # robot is fully operational 

                        if ros2_modules["charmie_low_level"]: # Motors

                            cmd_vel = Twist()

                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L1, self.robot.ON) and self.robot.get_gamepad_button_pressed(self.robot.BUTTON_R1, self.robot.ON):
                                ang_speed_percentage = 100
                                lin_speed_percentage = 100
                            elif self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L1, self.robot.ON):
                                ang_speed_percentage = 100
                                lin_speed_percentage = 75
                            else:
                                ang_speed_percentage = 100
                                lin_speed_percentage = 50

                            cmd_vel.linear.x =  float(self.percentage_to_linear_speed(  self.robot.get_gamepad_axis(self.robot.AXIS_L3_YY), lin_speed_percentage))
                            cmd_vel.linear.y =  float(self.percentage_to_linear_speed( -self.robot.get_gamepad_axis(self.robot.AXIS_L3_XX), lin_speed_percentage))
                            cmd_vel.angular.z = float(self.percentage_to_angular_speed(-self.robot.get_gamepad_axis(self.robot.AXIS_R3_XX), ang_speed_percentage))
                            
                            self.robot.node.cmd_vel_publisher.publish(cmd_vel)

                    if ros2_modules["charmie_neck"]:
                        
                        # circle and square to move neck left and right
                        # triangle and cross to move the neck up and down
                        neck_inc_hor = 2
                        neck_inc_ver = 1

                        if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_CIRCLE, self.robot.ON):
                            self.neck_pos_pan -= neck_inc_hor
                            if self.neck_pos_pan < -180:
                                self.neck_pos_pan = -180
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                            
                        elif self.robot.get_gamepad_button_pressed(self.robot.BUTTON_SQUARE, self.robot.ON):
                            self.neck_pos_pan += neck_inc_hor
                            if self.neck_pos_pan > 180:
                                self.neck_pos_pan = 180
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        
                        if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_CROSS, self.robot.ON):
                            self.neck_pos_tilt -= neck_inc_ver
                            if self.neck_pos_tilt < -60:
                                self.neck_pos_tilt = -60
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        
                        elif self.robot.get_gamepad_button_pressed(self.robot.BUTTON_TRIANGLE, self.robot.ON):
                            self.neck_pos_tilt += neck_inc_ver
                            if self.neck_pos_tilt > 60:
                                self.neck_pos_tilt = 60
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)


                    if not self.current_task:
                        if ros2_modules["charmie_neck"] and ros2_modules["charmie_yolo_objects"] and ros2_modules["charmie_head_camera"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_SHARE, self.robot.RISING):
                                self.state = self.Search_for_objects_demonstration
                        
                        if ros2_modules["charmie_neck"] and ros2_modules["charmie_yolo_pose"] and ros2_modules["charmie_head_camera"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_OPTIONS, self.robot.RISING):
                                self.state = self.Search_for_people_demonstration

                        if ros2_modules["charmie_speakers"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_R3, self.robot.RISING):
                                self.state = self.Introduction_demonstration

                        if ros2_modules["charmie_speakers"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L3, self.robot.RISING):
                                self.state = self.Serve_breakfast_demonstration
                                self.current_task = self.Serve_breakfast_demonstration
                                self.state_SB = self.SB_Waiting_for_task_start

                        if ros2_modules["charmie_llm"] and ros2_modules["charmie_speakers"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_R2, self.robot.RISING):
                                self.state = self.LLM_demonstration

                        if ros2_modules["charmie_arm"] and ros2_modules["charmie_speakers"]:
                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L2, self.robot.RISING):
                                self.state = self.Audio_receptionist_and_restaurant_demonstration
                    else:
                        # Similar to wait_for_end_of_navigation, allows navigation between subparts of task
                        if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L1, self.robot.ON) and self.robot.get_gamepad_button_pressed(self.robot.BUTTON_R1, self.robot.ON):
                            self.state = self.current_task

                        # Allows to cancel a task midway (in navigation part)
                        if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_SHARE, self.robot.ON) and self.robot.get_gamepad_button_pressed(self.robot.BUTTON_OPTIONS, self.robot.ON):
                            self.robot.set_speech(filename="demonstration/stopped_task_demo", wait_for_end_of=False)
                            self.current_task = 0
    

                time.sleep(0.05)


            elif self.state == self.Search_for_objects_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()
    
                tetas = [[35, -35], [45, -15], [55, -35]]
                # objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=3.0, list_of_objects=["Milk", "Cornflakes"], list_of_objects_detected_as=[["cleanser"], ["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=2.0, use_arm=False, detect_objects=True, detect_furniture=False)
                
                self.robot.set_neck(self.look_forward, wait_for_end_of=True)

                if len(objects_found):
                    if ros2_modules["charmie_face"]:
                        self.robot.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/i_have_found", wait_for_end_of=True)
                    
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RAINBOW_ROT)
                    
                    for o in objects_found:
                        if ros2_modules["charmie_face"]:
                            path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(255,255,0))
                            time.sleep(0.5)
                        
                        self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                        if ros2_modules["charmie_face"]:
                            time.sleep(2.5) # time to people check face
                
                    if ros2_modules["charmie_face"]:
                        self.robot.set_face("charmie_face")
    
                else:
                    self.robot.set_speech(filename="generic/could_not_find_any_objects", wait_for_end_of=True)
                
                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Search_for_people_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                tetas = [[-60, -10], [0, -10], [60, -10]]
                people_found = self.robot.search_for_person(tetas=tetas, time_in_each_frame=2.0)

                self.robot.set_neck(self.look_forward, wait_for_end_of=True)

                if len(people_found):
                    if ros2_modules["charmie_face"]:
                        self.robot.set_speech(filename="generic/check_face_person_detected", wait_for_end_of=True) ###
                    
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RAINBOW_ROT)
                    
                    for p in people_found:
                        if ros2_modules["charmie_face"]:
                            path = self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                            # time.sleep(0.5)
                        
                        self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute_head.z], wait_for_end_of=True)
                    
                        if ros2_modules["charmie_face"]:
                            time.sleep(2.5) # time to people check face
                
                    if ros2_modules["charmie_face"]:
                        self.robot.set_face("charmie_face")
    
                else:
                    self.robot.set_speech(filename="generic/could_not_find_any_people", wait_for_end_of=True) ###

                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Introduction_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_arm"]:
                    self.robot.set_arm(command="hello", wait_for_end_of=False)
                    time.sleep(7.0)

                # self.robot.set_speech(filename="generic/welcome_roboparty", wait_for_end_of=False)
                self.robot.set_speech(filename="demonstration/introduction_demo", wait_for_end_of=True)

                if ros2_modules["charmie_arm"]: # the delays only make sense when arm is operational 
                    time.sleep(10.2)
                
                self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.LLM_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_llm"]:
                    self.robot.get_llm_demonstration(wait_for_end_of=True)
                    
                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.Audio_receptionist_and_restaurant_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                if self.state_A == self.A_Receptionist:
                
                    ### audio receptionist code here
                    self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)


                    
                    ##### JA GEREI ALGUMAS FRASES, PLEASE CHECK: restaurant_demo_intro e demo_please_wave
                    


                    time.sleep(1.0)
                    # Reconhecer a pessoa
                    # Olhar para a pessoa
                    # person with characteristics received from search for person (closest person + ...)
                    recep_detected_person = DetectedPerson()
                    recep_detected_person.height = 1.00
                    recep_detected_person.gender = "Female"
                    recep_detected_person.shirt_color = "Pink"
                    
                    self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                    command = self.robot.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
                    print("Finished:", command)
                    keyword_list= command.split(" ")
                    guest_name = keyword_list[0] 
                    guest_drink = keyword_list[1]
                    
                    print(guest_name, guest_drink)

                    self.robot.set_speech(filename="demonstration/nice_to_meet_you", wait_for_end_of=True)
                    self.robot.set_speech(filename="person_names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
                    random_drink = str(random.randint(1, 3))
                    self.robot.set_speech(filename="demonstration/favourite_drink_demo_"+random_drink, wait_for_end_of=True)
                    self.robot.set_speech(filename="objects_names/"+keyword_list[1].lower(), wait_for_end_of=True)

                    confirmation = self.robot.get_audio(yes_or_no=True, question="demonstration/know_some_characteristics", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    ##### Verifica a resposta recebida
                    if confirmation.lower() == "yes":
                        self.robot.get_detected_person_characteristics(detected_person=recep_detected_person, first_sentence="demonstration/demo_characteristics_first_sentence", \
                                                                       ethnicity=True, age=True, gender =True, height=True, shirt_color=True, pants_color=True)
                        time.sleep(1.0)
                        
                    else: #  confirmation.lower() == "no":
                        self.robot.set_speech(filename="demonstration/ok_i_understand", wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="demonstration/see_you_soon", wait_for_end_of=True)
                    
                    self.state_A = self.A_Restaurant

                elif self.state_A == self.A_Restaurant:

                    self.robot.set_speech(filename="demonstration/restaurant_demo_intro", wait_for_end_of=True)
                    self.robot.set_speech(filename="demonstration/demo_please_wave", wait_for_end_of=True)

                    list_customers = []                     
                    # SFP

                    list_filtered_customers = []

                    # Olhar e mostrar cara da pessoa
                    # Is this a guest?
                    # Yes or No (continua para o próximo)
                    # Yes: Olha para ele
                    # Yes: please stand in front of me. What is your order?
                    # Ouve
                    # confirma 
                    # Ok, I will try to get you (pedido)
                    # (continua para o próximo pedido)
                    # Now i should pick the foods and drinks and bring to my friends. See you soon my friends
                
                    ### audio restaurant code here
                    self.robot.set_speech(filename="restaurant/start_restaurant", wait_for_end_of=True)

                    self.state_A = self.A_Receptionist

                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.Serve_breakfast_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                if self.state_SB == self.SB_Waiting_for_task_start:
                    
                    self.robot.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)
                    self.robot.set_neck(self.look_forward, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=True)
                    self.state_SB = self.SB_Detect_and_receive_milk

                elif self.state_SB == self.SB_Detect_and_receive_milk:
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=True)

                    ### MILK 
                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=2.0, wait_time_show_help_face=2.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                    self.robot.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
                    self.robot.set_neck(self.look_forward, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=True)
                    self.state_SB = self.SB_Detect_and_receive_cornflakes

                elif self.state_SB == self.SB_Detect_and_receive_cornflakes:

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=True)

                    ### CORNFLAKES
                    object_in_gripper = False                    
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Cornflakes"], list_of_objects_detected_as=[["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=2.0, wait_time_show_help_face=2.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                    self.robot.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
                    self.robot.set_neck(self.look_forward, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=True)
                    self.state_SB = self.SB_Detect_and_receive_dishes
                    
                elif self.state_SB == self.SB_Detect_and_receive_dishes:

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=True)

                    ### BOWL
                    object_in_gripper = False
                    correct_object_bowl = DetectedObject()
                    correct_object_spoon = DetectedObject()
                    while not object_in_gripper:

                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Spoon", "Bowl"], use_arm=False, detect_objects=True, detect_furniture=False)
                        # objects_found = self.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Spoon", "Bowl"], list_of_objects_detected_as=[["Fork", "Knife"],["Plate"]], use_arm=False, detect_objects=True, detect_furniture=False)
                    
                        for of in objects_found:
                            print(of.object_name.lower(), of.index)
                            if of.object_name.lower() == "bowl":
                                correct_object_bowl = of
                            elif of.object_name.lower() == "spoon":
                                correct_object_spoon = of

                        print("correct_bowl:", correct_object_bowl.object_name, correct_object_bowl.index)
                        print("correct_spoon:", correct_object_spoon.object_name, correct_object_spoon.index)

                        # BOWL
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=correct_object_bowl, look_judge=self.look_judge, wait_time_show_detection=2.0, wait_time_show_help_face=2.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                                
                    ### HAVE TO CHANGE THIS wait_for_end_of to False for after adding SPOON
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                    ### SPOON
                    self.robot.ask_help_pick_object_tray(object_d=correct_object_spoon, look_judge=self.look_judge, first_help_request=False, bb_color=(0, 255, 0), audio_confirmation=False)
                    
                    self.robot.set_neck(self.look_forward, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/dinner_table", wait_for_end_of=True)

                    self.state_SB = self.SB_Place_and_pour_objects

                elif self.state_SB == self.SB_Place_and_pour_objects:
                    
                    self.robot.set_neck(self.look_table_objects, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/dinner_table", wait_for_end_of=True)
                    
                    ### PLACE BOWL
                    self.robot.set_arm(command="place_bowl_table", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                    ### POUR CORNFLAKES
                    self.robot.set_arm(command="pour_cereals_bowl", wait_for_end_of=True)
                    self.robot.set_speech(filename="serve_breakfast/cornflakes_poured", wait_for_end_of=False)
                    
                    ### PLACE CORNFLAKES
                    self.robot.set_arm(command="place_cereal_table", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                    ### POUR MILK
                    self.robot.set_arm(command="pour_milk_bowl", wait_for_end_of=True)
                    self.robot.set_speech(filename="serve_breakfast/milk_poured", wait_for_end_of=False)

                    ### PLACE MILK
                    self.robot.set_arm(command="place_milk_table", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                    #### PLACE SPOON: MISSING ...
                    # self.robot.set_arm(command="place_spoon_table_funilocopo_v2", wait_for_end_of=True)
                    self.robot.set_arm(command="place_spoon_table_funilocopo_v2_facing_other_side", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/place_object_placed", wait_for_end_of=False)

                    ### FINISHED SERVING BREAKFAST
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    self.robot.set_neck(self.look_forward, wait_for_end_of=False)
                    self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                    self.state_SB = self.SB_Waiting_for_task_start
                    self.current_task = 0

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks


                """
            elif self.state == self.Example_demonstration:
                
                temp_active_motors = self.MOTORS_ACTIVE_FLAG
                self.safety_stop_modules()

                ### your code here

                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.task_reactivate_after_safety_stop(temp_active_motors)

                self.state = self.Demo_actuators_with_tasks
                """
            
            else:
                pass

    def safety_stop_modules(self):
        # Add here, all safety restrictions for the safety stop
        
        if ros2_modules["charmie_low_level"]:

            # Dirty, but had to do this way because of some commands to low_level being lost                    
            cmd_vel = Twist()
            self.robot.node.cmd_vel_publisher.publish(cmd_vel)  
            time.sleep(0.1)  # wait for the cmd_vel to be published
            self.robot.node.cmd_vel_publisher.publish(cmd_vel)  
            time.sleep(0.1)  # wait for the cmd_vel to be published
            self.robot.node.cmd_vel_publisher.publish(cmd_vel)  

            self.MOTORS_ACTIVE_FLAG = False
            self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

    def task_reactivate_after_safety_stop(self, temp_active_motors):

        if ros2_modules["charmie_low_level"]:
                
            self.MOTORS_ACTIVE_FLAG = temp_active_motors
            self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

            if self.MOTORS_ACTIVE_FLAG:
                self.robot.set_rgb(BLUE+HALF_ROTATE)
            else:
                self.robot.set_rgb(MAGENTA+HALF_ROTATE)

    def percentage_to_angular_speed(self, value, percentage):
        # Convert percentage to angular speed
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100

        return value * percentage/100 * self.MAX_ANGULAR_SPEED
    
    def percentage_to_linear_speed(self, value, percentage):
        # Convert percentage to angular speed
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100

        return value * percentage/100 * self.MAX_LINEAR_SPEED