#!/usr/bin/env python3
import rclpy
import threading
import time
import random

from geometry_msgs.msg import Vector3, Pose2D
from charmie_interfaces.msg import DetectedObject, DetectedPerson, PS4Controller
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            True,
    "charmie_face":             False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_lidar":            True,
    "charmie_llm":              True,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
    "charmie_point_cloud":      True,
    "charmie_ps4_controller":   True,
    "charmie_speakers":         True,
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
        # self.look_table_objects = [-45, -45]
        # self.look_tray = [0, -60]

        self.OFF = 0     # LOW  -> LOW
        self.FALLING = 1 # HIGH -> LOW
        self.RISING = 2  # LOW  -> HIGH
        self.ON = 3      # HIGH -> HIGH

        self.ON_AND_RISING = 2   # used with <= 
        self.OFF_AND_FALLING = 1 # used with >=

        self.neck_pos_pan = self.look_forward[0]
        self.neck_pos_tilt = self.look_forward[1]

        # self.previous_message = False
        self.PREVIOUS_WATCHDOG_SAFETY_FLAG = True # just for RGB debug
        self.WATCHDOG_SAFETY_FLAG = True
        self.WATCHDOG_CUT_TIME = 1.5
        self.iteration_time = 0.01
        self.watchdog_timer_ctr = self.WATCHDOG_CUT_TIME/self.iteration_time

        self.motors_active = False
        self.omni_move = Vector3()
        self.torso_pos = Pose2D()

        # Start localisation position
        # self.initial_position = [-1.0, 1.5, -90.0]

        # navigation positions
        # self.front_of_sofa = [-2.5, 1.5]
        # self.sofa = [-2.5, 3.0]
        self.current_task = 0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start
        self.state_SB = self.SB_Waiting_for_task_start
        self.state_A = self.A_Receptionist

        while True:

            if self.state == self.Waiting_for_task_start:
                # Initialization State

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(CLEAR)
                    self.motors_active = False
                    self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    self.robot.node.torso_movement_publisher.publish(self.torso_pos)

                if ros2_modules["charmie_face"]:
                    self.robot.set_face("charmie_face")

                if ros2_modules["charmie_neck"]:
                    self.robot.set_neck(self.look_forward, wait_for_end_of=True)

                time.sleep(0.5)

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_speakers"]:
                    self.robot.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                # to initially set WATCHDOG TIMER FLAGS and RGB 
                ps4_controller, new_message = self.robot.get_controller_state()
                self.controller_watchdog_timer(new_message)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        self.motors_active = True
                        self.robot.activate_motors(activate=self.motors_active)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                        self.motors_active = False
                        self.robot.activate_motors(activate=self.motors_active)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Demo_actuators_with_tasks:

                ps4_controller, new_message = self.robot.get_controller_state()
                self.controller_watchdog_timer(new_message)

                if self.WATCHDOG_SAFETY_FLAG:
                    ps4_controller = PS4Controller() # cleans ps4_controller -> sets everything to 0

                # print(self.WATCHDOG_SAFETY_FLAG, self.PREVIOUS_WATCHDOG_SAFETY_FLAG)
                
                # WATCHDOG VERIFICATIONS
                if not self.WATCHDOG_SAFETY_FLAG and self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        # only allow reactivation via PS button (safety)
                        # self.motors_active = True
                        # self.robot.activate_motors(activate=self.motors_active)

                elif self.WATCHDOG_SAFETY_FLAG and not self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                    
                    self.safety_stop_modules()
    
                    if ros2_modules["charmie_speakers"]:
                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)


                if new_message:

                    # Activate motors: only activates if ps4 controller messages are being received
                    if ros2_modules["charmie_low_level"]:

                        # Watchdog Verifications
                        if ps4_controller.ps == self.RISING:
                            if not self.WATCHDOG_SAFETY_FLAG:
                                self.motors_active = not self.motors_active
                                self.robot.activate_motors(activate=self.motors_active)

                                self.torso_pos.x = 0.0
                                self.torso_pos.y = 0.0
                                self.robot.node.torso_movement_publisher.publish(self.torso_pos)

                                if self.motors_active:
                                    if ros2_modules["charmie_speakers"]:
                                        self.robot.set_speech(filename="demonstration/motors_unlocked", wait_for_end_of=False)
                                else:
                                    if ros2_modules["charmie_speakers"]:
                                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)
                        
                    
                    if ros2_modules["charmie_low_level"]:
                        # Robot Omni Movement
                        # left joy stick to control x and y movement (direction and linear speed) 
                        if ps4_controller.l3_dist >= 0.1:
                            self.omni_move.x = ps4_controller.l3_ang
                            self.omni_move.y = ps4_controller.l3_dist*100/5
                        else:
                            self.omni_move.x = 0.0
                            self.omni_move.y = 0.0

                        # right joy stick to control angular speed
                        if ps4_controller.r3_dist >= 0.1:
                            self.omni_move.z = 100 + ps4_controller.r3_xx*10
                        else:
                            self.omni_move.z = 100.0
                        
                        if self.motors_active:
                            self.robot.node.omni_move_publisher.publish(self.omni_move)

                    if ros2_modules["charmie_low_level"]:

                        if self.motors_active:

                            # Torso Movement
                            if ps4_controller.arrow_up >= 2:
                                self.torso_pos.x = 1.0
                            elif ps4_controller.arrow_down >= 2:
                                self.torso_pos.x = -1.0
                            else:
                                self.torso_pos.x = 0.0

                            if ps4_controller.arrow_right >= 2:
                                self.torso_pos.y = 1.0
                            elif ps4_controller.arrow_left >= 2:
                                self.torso_pos.y = -1.0
                            else:
                                self.torso_pos.y = 0.0

                            self.robot.node.torso_movement_publisher.publish(self.torso_pos)

                    if ros2_modules["charmie_neck"]:
                        
                        # circle and square to move neck left and right
                        # triangle and cross to move the neck up and down
                        neck_inc_hor = 2
                        neck_inc_ver = 1
                        if ps4_controller.circle >= self.ON_AND_RISING:
                            self.neck_pos_pan -= neck_inc_hor
                            if self.neck_pos_pan < -180:
                                self.neck_pos_pan = -180
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                            
                        elif ps4_controller.square >= self.ON_AND_RISING:
                            self.neck_pos_pan += neck_inc_hor
                            if self.neck_pos_pan > 180:
                                self.neck_pos_pan = 180
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        
                        if ps4_controller.cross >= self.ON_AND_RISING:
                            self.neck_pos_tilt -= neck_inc_ver
                            if self.neck_pos_tilt < -60:
                                self.neck_pos_tilt = -60
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        
                        elif ps4_controller.triangle >= self.ON_AND_RISING:
                            self.neck_pos_tilt += neck_inc_ver
                            if self.neck_pos_tilt > 45:
                                self.neck_pos_tilt = 45
                            self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)


                    if not self.current_task:
                        if ros2_modules["charmie_neck"] and ros2_modules["charmie_yolo_objects"] and ros2_modules["charmie_head_camera"] and ros2_modules["charmie_point_cloud"]:
                            if ps4_controller.share == self.RISING:
                                self.state = self.Search_for_objects_demonstration
                        
                        if ros2_modules["charmie_neck"] and ros2_modules["charmie_yolo_pose"] and ros2_modules["charmie_head_camera"] and ros2_modules["charmie_point_cloud"]:
                            if ps4_controller.options == self.RISING:
                                self.state = self.Search_for_people_demonstration

                        if ros2_modules["charmie_audio"] and ros2_modules["charmie_neck"] and ros2_modules["charmie_yolo_pose"] and ros2_modules["charmie_head_camera"] and ros2_modules["charmie_point_cloud"]:
                            if ps4_controller.r1 == self.RISING:
                                self.state = self.Audio_receptionist_and_restaurant_demonstration

                        if ros2_modules["charmie_speakers"]:
                            if ps4_controller.r3 == self.RISING:
                                self.state = self.Introduction_demonstration

                        if ros2_modules["charmie_speakers"]:
                            if ps4_controller.l3 == self.RISING:
                                self.state = self.Serve_breakfast_demonstration
                                self.current_task = self.Serve_breakfast_demonstration
                                self.state_SB = self.SB_Waiting_for_task_start

                        if ros2_modules["charmie_llm"] and ros2_modules["charmie_speakers"]:
                            if ps4_controller.r2 > 0.8:
                                self.state = self.LLM_demonstration

                        if ros2_modules["charmie_arm"] and ros2_modules["charmie_speakers"]:
                            if ps4_controller.l2 > 0.8:
                                # self.state = self.Open_door
                                pass
                    else:
                        # Similar to wait_for_end_of_navigation, allows navigation between subparts of task
                        if ps4_controller.l1 >= self.ON_AND_RISING and ps4_controller.r1 >= self.ON_AND_RISING:
                            self.state = self.current_task

                        # Allows to cancel a task midway (in navigation part)
                        if ps4_controller.share >= self.ON_AND_RISING and ps4_controller.options >= self.ON_AND_RISING:
                            self.robot.set_speech(filename="demonstration/stopped_task_demo", wait_for_end_of=False)
                            self.current_task = 0
    

                time.sleep(self.iteration_time)


            elif self.state == self.Search_for_objects_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()
    
                tetas = [[35, -35], [45, -15], [55, -35]]
                # objects_found = self.robot.search_for_objects(tetas=tetas, delta_t=3.0, list_of_objects=["Milk", "Cornflakes"], list_of_objects_detected_as=[["cleanser"], ["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                objects_found = self.robot.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                
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

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Search_for_people_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                tetas = [[-60, -10], [0, -10], [60, -10]]
                people_found = self.robot.search_for_person(tetas=tetas, delta_t=2.0)

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

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Introduction_demonstration:
                
                temp_active_motors = self.motors_active
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

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.LLM_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_llm"]:
                    self.robot.get_llm_demonstration(wait_for_end_of=True)
                    
                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.Audio_receptionist_and_restaurant_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                if self.state_A == self.A_Receptionist:
                
                    ### audio receptionist code here
                    self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                    
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
                    self.robot.set_speech(filename="receptionist/names/"+guest_name.replace(" ","_").lower(), wait_for_end_of=True)
                    
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

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.Serve_breakfast_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                if self.state_SB == self.SB_Waiting_for_task_start:
                    
                    self.robot.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=True)
                    self.state_SB = self.SB_Detect_and_receive_milk

                elif self.state_SB == self.SB_Detect_and_receive_milk:
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=True)

                    ### MILK 
                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=2.0, wait_time_show_help_face=2.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                    self.robot.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=True)
                    self.state_SB = self.SB_Detect_and_receive_cornflakes

                elif self.state_SB == self.SB_Detect_and_receive_cornflakes:

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=True)

                    ### CORNFLAKES
                    object_in_gripper = False                    
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Cornflakes"], list_of_objects_detected_as=[["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=2.0, wait_time_show_help_face=2.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                    self.robot.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
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

                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Spoon", "Bowl"], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        # objects_found = self.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Spoon", "Bowl"], list_of_objects_detected_as=[["Fork", "Knife"],["Plate"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                    
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
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                    ### SPOON (to be done ...)

                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                    self.robot.set_speech(filename="furniture/dinner_table", wait_for_end_of=True)

                    self.state_SB = self.SB_Place_and_pour_objects

                elif self.state_SB == self.SB_Place_and_pour_objects:
                    
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
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)


                    self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                    self.state_SB = self.SB_Waiting_for_task_start
                    self.current_task = 0


                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks


                """
            elif self.state == self.Example_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                ### your code here

                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks
                """
            
            else:
                pass


    def controller_watchdog_timer(self, new_message):

        if new_message:
            self.watchdog_timer_ctr = 0
        else:
            self.watchdog_timer_ctr += 1

        self.PREVIOUS_WATCHDOG_SAFETY_FLAG = self.WATCHDOG_SAFETY_FLAG

        if self.watchdog_timer_ctr > (self.WATCHDOG_CUT_TIME/self.iteration_time):
            self.WATCHDOG_SAFETY_FLAG = True
        else:
            self.WATCHDOG_SAFETY_FLAG = False

    def safety_stop_modules(self):
        
        if ros2_modules["charmie_low_level"]:
            
            self.omni_move.x = 0.0
            self.omni_move.y = 0.0
            self.omni_move.z = 100.0
            self.robot.node.omni_move_publisher.publish(self.omni_move)

            self.motors_active = False
            self.robot.activate_motors(activate=self.motors_active)

            self.torso_pos.x = 0.0
            self.torso_pos.y = 0.0
            self.robot.node.torso_movement_publisher.publish(self.torso_pos)
