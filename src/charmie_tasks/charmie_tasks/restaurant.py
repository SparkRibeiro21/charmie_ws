#!/usr/bin/env python3
import rclpy
import threading
import time
import math
from charmie_interfaces.msg import DetectedObject
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  True,
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 False,
    "charmie_nav_sdnl":             True,
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
    "charmie_speakers_save":        False,
    "charmie_tracking":             False,
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
        self.TASK_NAME = "Restaurant"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":           0,
            "Looking_for_barman":               1,
            "Detecting_waving_customers":       2,
            "Approach_customer":                3,
            "Receive_order":                    4,
            "Go_back_to_barman_with_order":     5,
            "Collect_order_from_barman":        6,
            "Approch_customer_with_order":      7,
            "Deliver_order":                    8,
            "Move_to_barman_after_delivery":    9,
            "Final_State":                      10,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        self.BARMAN_SIDE = "left" # by default, but if barman is detected in code, it changes

        self.MIN_DISTANCE_TO_CUSTOMER = 0.5 # 
        self.MIN_DISTANCE_TO_BARMAN = 0.35

        
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        # self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED.lower().replace(" ", "_")
        # Checks if there is any error in the furniture variables:
        # if self.robot.get_room_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED) == None:
        #     print("ERROR!!! - FURNITURE:", self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, "DOES NOT EXIST IN furniture.json")
        #     while True:
        #         pass
        # self.SB_TABLE_HEIGHT = self.robot.get_height_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)[0]
        # print("Table Height =", self.SB_TABLE_HEIGHT)
        # # Set the height of the table where breakfast is served, so that the manual arm movements are adapted to this height (placing and pouring)
        # self.robot.set_height_furniture_for_arm_manual_movements(self.SB_TABLE_HEIGHT) #####

        ####ADDED_VARIABLES######        
        self.BARMAN_NAV_COORDS = [0.0, 0.0, 0.0] # x, y, theta
        self.CUSTOMER_NAV_COORDS = [0.0, 0.0, 0.0] # x, y, theta
        self.SAFE_NAV_COORDS_TO_BARMAN = [0.0, 0.0, 0.0] # x, y, theta - this is the intermediate position the robot goes to when it needs to move to the barman
        self.SAFE_NAV_COORDS_TO_CUSTOMER = [0.0, 0.0, 0.0] # x, y, theta - this is the intermediate position the robot goes to when it needs to move to the customer area

        self.BARMAN_COORDS = [0.0, 0.0, 0.0] # x, y, z
        self.CUSTOMER_COORDS = [0.0, 0.0, 0.0] # x, y, z

        self.detected_customers = []
        self.DETECTED_CUSTOMER_INDEX = 0
        self.all_orders = []
        self.NCP = ["Red_Bull", "Tuna"]
        self.NCT = ["Red Wine","Orange Juice","Pringles","Cheezit"]

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
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

                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="restaurant/start_restaurant", wait_for_end_of=True)

                self.robot.wait_for_start_button()

                self.robot.set_speech(filename="restaurant/restaurant_introduction", wait_for_end_of=True)

                # time.sleep(3.0) # time for person who pressed start button leave to not be shown in qualif video

                self.state = self.task_states["Collect_order_from_barman"]
                

            elif self.state == self.task_states["Looking_for_barman"]:
               
                self.robot.set_speech(filename="restaurant/customers_stop_waving", wait_for_end_of=True)
                time.sleep(1.0)

                tetas = [[90, 0], [-90, 0]]
                barman = []
                barman_ctr = 0
                correct_barman = False

                # Check for people BACK, LEFT and RIGHT, to figure out who is the barman
                while not correct_barman:
                    barman.clear()

                    # if for some reason, after two attempts, the robot does not understand who the barman is, it goes back to the default barman side
                    barman_ctr += 1
                    if barman_ctr > 2:

                        if self.BARMAN_SIDE.lower() == "left":
                            self.robot.set_neck(position=self.look_left, wait_for_end_of=True)
                            ### calculate barman angle and position to me 
                            self.BARMAN_COORDS = [0.0, 1.5, 1.7]
                            self.BARMAN_NAV_COORDS = [-1.0, -0.5, 90.0]
                            self.SAFE_NAV_COORDS_TO_BARMAN = [0.0, -1.0, 180.0]
                            self.SAFE_NAV_COORDS_TO_CUSTOMER = [0.0, -1.0, 0.0]

                        else:
                            self.robot.set_neck(position=self.look_right, wait_for_end_of=True)
                            ### calculate barman angle and position to me
                            self.BARMAN_COORDS = [0.0, -1.5, 1.7] 
                            self.BARMAN_NAV_COORDS = [-1.0, 0.5, -90.0]
                            self.SAFE_NAV_COORDS_TO_BARMAN = [0.0, 1.0, 180.0]
                            self.SAFE_NAV_COORDS_TO_CUSTOMER = [0.0, 1.0, 0.0]

                        ##### SPEAK : Hello! Nice to meet you! My name is charmie and I am here to help you serve the customers.
                        self.robot.set_speech(filename="restaurant/barman_meeting", wait_for_end_of=True)
                        ##### SPEAK : I am going to turn around and search for possible customers. See you soon
                        self.robot.set_speech(filename="restaurant/go_search", wait_for_end_of=True)
                        correct_barman = True
                    
                    else: 
                        self.robot.set_speech(filename="restaurant/barman_instructions", wait_for_end_of=True)
                        # self.robot.set_speech(filename="restaurant/search_barman", wait_for_end_of=True)
                        
                        people_found = self.robot.search_for_person(tetas=tetas, only_detect_person_arm_raised=True) ### , only_detect_person_right_in_front=True)

                        print("FOUND:", len(people_found)) 
                        for p in people_found:

                            dist_to_robot = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                            if 0.1 <= dist_to_robot <= 2.0: # this filters for nearby people
                                print("DIST:", dist_to_robot)
                                barman.append(p)
                            
                                # all below can be commented
                                self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute.z], wait_for_end_of=True)
                                print("ID:", p.index)
                                print('Barman position', p.position_relative)
                                # time.sleep(2.0)

                        # this 'for' can be merged with the previous 'for'
                        for b in barman:
                            self.robot.set_neck_coords(position=[b.position_absolute.x, b.position_absolute.y, b.position_absolute.z], wait_for_end_of=True)

                            self.robot.set_speech(filename="restaurant/confirm_barman_touchscreen", wait_for_end_of=True)
                            answer = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=["yes", "no"], timeout=10, instruction="Are you the barman?", speak_results=False, wait_for_end_of=True)

                            if answer == ["yes"]:
                                
                                if b.position_absolute.y > 0:
                                    self.BARMAN_SIDE = "left" 
                                    self.BARMAN_COORDS = [0.0, 1.5, 1.7] 
                                    self.BARMAN_NAV_COORDS = [-1.0, -0.5, 90.0]
                                    self.SAFE_NAV_COORDS_TO_BARMAN = [0.0, -1.0, 180.0]
                                    self.SAFE_NAV_COORDS_TO_CUSTOMER = [0.0, -1.0, 0.0]
                                else:                                
                                    self.BARMAN_SIDE = "right" 
                                    self.BARMAN_COORDS = [0.0, -1.5, 1.7]
                                    self.BARMAN_NAV_COORDS = [-1.0, 0.5, -90.0]
                                    self.SAFE_NAV_COORDS_TO_BARMAN = [0.0, 1.0, 180.0]
                                    self.SAFE_NAV_COORDS_TO_CUSTOMER = [0.0, 1.0, 0.0]


                                ### calculate barman angle and position to me 
                                # self.BARMAN_COORDS = [b.position_absolute.x, b.position_absolute.y, b.position_absolute.z] # moved this into the left right if just above

                                ##### SPEAK : Hello! Nice to meet you! My name is charmie and I am here to help you serve the customers.
                                self.robot.set_speech(filename="restaurant/barman_meeting", wait_for_end_of=True)

                                ##### SPEAK : I am going to turn around and search for possible customers. See you soon
                                self.robot.set_speech(filename="restaurant/go_search", wait_for_end_of=True)

                                correct_barman = True
                                break

                self.state = self.task_states["Detecting_waving_customers"]


            elif self.state == self.task_states["Detecting_waving_customers"]:

                tetas = [[-60, 0], [0, 0], [60, 0]]
                customers_list = []
                self.detected_customers.clear()
                self.DETECTED_CUSTOMER_INDEX = 0
                NUMBER_OF_CUSTOMERS = 2
                moved_to_find_customers = False

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.robot.set_speech(filename="restaurant/customers_wave", wait_for_end_of=True)

                while not self.detected_customers:

                    customers_found = self.robot.search_for_person(tetas=tetas, only_detect_person_arm_raised=True)

                    print("FOUND:", len(customers_found)) 
                    for p in customers_found:

                        dist_to_robot = math.sqrt(p.position_relative.x**2 + p.position_relative.y**2)
                        if 0.5 <= dist_to_robot <= 6.0: # this filters some errors from very far away customers
                            customers_list.append(p)

                    ### REORDER BY DISTANCE ###
                    customers_list.sort(key=lambda p: math.hypot(p.position_absolute.x, p.position_absolute.y))

                    # Just to announce and look at the considered customers
                    if len(customers_list) > 0:
                        # self.robot.set_speech(filename="restaurant/found_one_customer", wait_for_end_of=True)
                        self.robot.set_speech(filename="restaurant/found_customers", wait_for_end_of=True)
                        for c in customers_list:
                            self.robot.detected_person_to_face_path(person=c, send_to_face=True)
                            self.robot.set_neck_coords(position=[c.position_absolute.x, c.position_absolute.y, c.position_absolute.z], wait_for_end_of=True)
                            print("ID:", c.index)
                            print('Customer position', c.position_relative)
                            time.sleep(3.0)
                    self.robot.set_face("charmie_face", wait_for_end_of=False)

                    print('Nr of detected customers waving: ', len(customers_list))

                    if len(customers_list) > 0:
                        
                        # moves back to barman and looks at barman, because it may not be in the position initial position because of move forward searches
                        if moved_to_find_customers:
                            self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                            self.robot.sdnl_move_to_position(move_coords=self.BARMAN_NAV_COORDS, first_rotate=True, orient_after_move=True, reached_radius=0.7, wait_for_end_of=True)
                            self.robot.adjust_obstacles(distance=0.20, direction=0.0, max_speed=0.1, wait_for_end_of=True)
                            moved_to_find_customers = False
                        else:
                            self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                            self.robot.sdnl_move_to_position(move_coords=self.BARMAN_NAV_COORDS, first_rotate=False, orient_after_move=True, reached_radius=5.0, wait_for_end_of=True)
                        
                        self.robot.set_neck_coords(position=self.BARMAN_COORDS, wait_for_end_of=True)

                        self.robot.set_speech(filename="restaurant/barman_help_confirm_customers", wait_for_end_of=True)

                        for p in customers_list:

                            self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                            self.robot.set_speech(filename="restaurant/found_customer_check_face", wait_for_end_of=True)
                            time.sleep(3.0)
                            self.robot.set_speech(filename="restaurant/is_person_customer", wait_for_end_of=True)
                            self.robot.set_speech(filename="restaurant/duplicate_customer_select_no", wait_for_end_of=True)
                            answer = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=["yes", "no"], speak_results=False)
                            self.robot.set_face("charmie_face", wait_for_end_of=False)
                            print("ANSWER:", answer)
                            if answer == ["yes"]:
                                self.robot.set_speech(filename="restaurant/added_person_as_customer", wait_for_end_of=True)
                                self.detected_customers.append(p)
                            else:
                                self.robot.set_speech(filename="restaurant/not_added_person_as_customer", wait_for_end_of=True)

                            if len(self.detected_customers) >= NUMBER_OF_CUSTOMERS:
                                break

                        if self.detected_customers:
            
                            # final customer confirmation
                            self.robot.set_speech(filename="restaurant/final_check_saved_customers", wait_for_end_of=True)
                            for p in self.detected_customers:
                                self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                                time.sleep(3.0)
                                                        
                            # jumps to next customer in list (if available)
                            self.state = self.task_states["Approach_customer"]
                            if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                            0.0 ] ### should be changed later
                                self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                self.DETECTED_CUSTOMER_INDEX += 1
                            else:
                                self.detected_customers.clear()
                                customers_list.clear()
                                self.DETECTED_CUSTOMER_INDEX = 0
                                self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process
                                self.robot.adjust_omnidirectional_position(dx=-0.2, dy=0.0, wait_for_end_of=True)
                                if self.BARMAN_SIDE.lower() == "left":
                                    self.robot.adjust_angle(angle=-90, wait_for_end_of=True)
                                else:
                                    self.robot.adjust_angle(angle=90, wait_for_end_of=True)
                        else:
                            self.detected_customers.clear()
                            customers_list.clear()
                            self.DETECTED_CUSTOMER_INDEX = 0
                            self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process
                            self.robot.adjust_omnidirectional_position(dx=-0.2, dy=0.0, wait_for_end_of=True)
                            if self.BARMAN_SIDE.lower() == "left":
                                self.robot.adjust_angle(angle=-90, wait_for_end_of=True)
                            else:
                                self.robot.adjust_angle(angle=90, wait_for_end_of=True)
                        
                    else:
                        # if no customer is found, moves a little bit forward, with safety radar ON
                        self.robot.set_speech(filename="restaurant/no_customers", wait_for_end_of=False)
                        if not moved_to_find_customers:
                            if self.BARMAN_SIDE.lower() == "left":
                                self.robot.adjust_omnidirectional_position(dx=0.0, dy=-0.2, wait_for_end_of=True)
                                self.robot.adjust_angle(angle=-45, wait_for_end_of=True)
                                s, m = self.robot.adjust_omnidirectional_position(dx=0.707, dy=0.0, safety=True, max_speed=0.07, wait_for_end_of=True) # 0.707 = sqrt(0.5^2 + 0.5^2), to move diagonally
                                self.robot.adjust_angle(angle=45, wait_for_end_of=True) # independently of the result of the forward movement, the robot should come back to its original angle
                                if not s:
                                    self.robot.adjust_omnidirectional_position(dx=0.5, dy=0.0, safety=True, wait_for_end_of=True)                     
                            else:
                                self.robot.adjust_omnidirectional_position(dx=0.0, dy=0.2, wait_for_end_of=True)
                                self.robot.adjust_angle(angle=45, wait_for_end_of=True)
                                s, m = self.robot.adjust_omnidirectional_position(dx=0.707, dy=0.0, safety=True, max_speed=0.07, wait_for_end_of=True) # 0.707 = sqrt(0.5^2 + 0.5^2), to move diagonally
                                self.robot.adjust_angle(angle=-45, wait_for_end_of=True)
                                if not s:
                                    self.robot.adjust_omnidirectional_position(dx=0.5, dy=0.0, safety=True, wait_for_end_of=True)
                        else:
                                self.robot.adjust_omnidirectional_position(dx=0.5, dy=0.0, safety=True, wait_for_end_of=True)
                        moved_to_find_customers = True


            elif self.state == self.task_states["Approach_customer"]:

                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.adjust_omnidirectional_position(dx=-0.2, dy=0.0, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.SAFE_NAV_COORDS_TO_CUSTOMER, first_rotate=True, orient_after_move=True, reached_radius=0.7, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.CUSTOMER_NAV_COORDS, first_rotate=False, orient_after_move=False, reached_radius=2.0, wait_for_end_of=True)

                s, m, d = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                if d > self.MIN_DISTANCE_TO_CUSTOMER:
                    self.robot.adjust_obstacles(distance=self.MIN_DISTANCE_TO_CUSTOMER, direction=0.0, max_speed=0.1, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                
                self.state = self.task_states["Receive_order"]
                # self.state = self.task_states["Go_back_to_barman_with_order"] # for debug
                
            elif self.state == self.task_states["Receive_order"]:

                ### pergunta sim ou nao se tem um pedido
                    ### se nao, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                        ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                    ### se counter > 5, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                        ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                    ### se sim continua
                    
                ### faz a pergunta do pedido
                    ### se counter > 5, passa para a cara e tenta duas vezes
                        ### caso exceda as duas vezes da cara, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                    ### se sim vai para pergunta de confrirmacao

                ### pergunta de confirmacao
                    ### se sim, avança para o proximo estado 
                    ### se nao volta a fazer o pedido     
                     
                self.robot.set_rgb(command=BLUE+ALTERNATE_QUARTERS)
                # self.robot.set_neck_coords(position=self.CUSTOMER_COORDS, wait_for_end_of=True)
                self.robot.detected_person_to_face_path(person=self.detected_customers[self.DETECTED_CUSTOMER_INDEX-1], send_to_face=True)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                self.robot.set_speech(filename="generic/please_answer_robot_yes_no", wait_for_end_of=True)

                customer_has_order = False
                customer_has_order_ctr = 0
                max_asks_customer_has_order = 3
                while not customer_has_order:
                    customer_has_order_ctr+= 1

                    if customer_has_order_ctr < max_asks_customer_has_order:

                        # "Do you have an order?"
                        confirmation = self.robot.get_audio(yes_or_no=True, question="restaurant/have_an_order", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                        print("Finished:", confirmation)

                        if confirmation == "yes":
                            customer_has_order = True
                        elif confirmation == "no":

                            customer_has_order = True
                            self.robot.set_speech(filename="restaurant/not_have_an_order", wait_for_end_of=True)

                            # jumps to next customer in list (if available)
                            self.state = self.task_states["Approach_customer"]
                            if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                            0.0 ] ### should be changed later
                                self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                self.DETECTED_CUSTOMER_INDEX += 1
                            else:
                                self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process

                    else:
                        self.robot.set_speech(filename="restaurant/have_an_order_touchscreen", wait_for_end_of=True)
                        answer = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=["yes", "no"], timeout=10, instruction="Do you have an order?", speak_results=False, wait_for_end_of=True)
                        print("ANSWER:", answer)

                        if answer == ["yes"]:
                            customer_has_order = True
                            self.robot.set_speech(filename="restaurant/i_will_ask_you_for_your_order", wait_for_end_of=True)
                        else: # elif answer == ["no"]: # it may also be a timeout
                            customer_has_order = True
                            self.robot.set_speech(filename="restaurant/not_have_an_order", wait_for_end_of=True)

                            # jumps to next customer in list (if available)
                            self.state = self.task_states["Approach_customer"]
                            if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                            0.0 ] ### should be changed later
                                self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                self.DETECTED_CUSTOMER_INDEX += 1
                            else:
                                self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process


                if self.state == self.task_states["Receive_order"]:

                    order_received = False
                    order_received_ctr = 0
                    max_asks_audio_order_received = 3
                    max_asks_touchscreen_order_received = 2


                    # clear all orders received from previous customers, to not mix them with the new ones.
                    # In practice we should only delete the order that we are serving, because the whole system is prepared for multiple orders
                    # But to be safe in this version, we will clear all orders
                    self.all_orders.clear() 

                    while not order_received:
                        order_received_ctr+=1

                        if order_received_ctr <= max_asks_audio_order_received:

                            self.robot.set_speech(filename="restaurant/please_answer", wait_for_end_of=True)
                            time.sleep(0.5)
                            self.robot.set_speech(filename="restaurant/my_order_is", wait_for_end_of=True)
                            time.sleep(0.5)
                            self.robot.set_speech(filename="restaurant/and_then_say_your_order", wait_for_end_of=True)
                
                            ##### AUDIO: Listen the order and repeat for confirmation
                            command = self.robot.get_audio(restaurant=True, question="restaurant/what_is_your_order", face_hearing="charmie_face_green_my_order", wait_for_end_of=True)
                            print("Finished:", command)
                            keyword_list = command.split(" ")
                            self.robot.set_speech(filename="restaurant/order_consists_of", wait_for_end_of=True)
                            for kw in keyword_list:
                                print(kw)
                                self.robot.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)

                            ##### AUDIO: Listen "YES" OR "NO"
                            ##### "Please say yes or no to confirm the order"
                            confirmation = self.robot.get_audio(yes_or_no=True, question="restaurant/yes_no_question", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                            print("Finished:", confirmation)

                            ##### Verifica a resposta recebida
                            if confirmation.lower() == "yes":

                                if len(keyword_list) != 2:
                                        # Speak: "Ohh, there seems to be a problem with your order. Please select exactly two items for your order. Let's try again."
                                    self.robot.set_speech(filename="restaurant/invalid_order", wait_for_end_of=True)
                                else:
                                    self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                                    self.robot.set_rgb(command=GREEN+BLINK_LONG)
                                    # print(self.all_orders)

                                    self.robot.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                                    for kw in keyword_list:
                                        print(kw)
                                        self.robot.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                                    
                                    order_received = True  # Sai do loop se a confirmação for "yes"
                                    self.state = self.task_states["Go_back_to_barman_with_order"]

                            elif confirmation.lower() == "no":
                                self.robot.set_rgb(command=RED+BLINK_LONG)
                                ##### SPEAK: Sorry, TRY AGAIN
                                self.robot.set_speech(filename="restaurant/no_order", wait_for_end_of=True)
                            
                            else:
                                self.robot.set_rgb(command=YELLOW+BLINK_LONG)
                                ##### ERROR
                                print("ERROR")

                        elif order_received_ctr <= max_asks_audio_order_received + max_asks_touchscreen_order_received:
                            
                            self.robot.set_speech(filename="restaurant/have_an_order", wait_for_end_of=True)
                            answer = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=["yes", "no"], speak_results=False)
                            
                            print("ANSWER:", answer)

                            if answer == ["yes"]:

                                valid_order = False
                                while not valid_order:
                                    self.robot.set_speech(filename="restaurant/what_is_your_order", wait_for_end_of=True)
                                    keyword_list = self.robot.set_face_touchscreen_menu(["foods", "drinks", "snacks", "fruits", "custom"], custom_options=["Red Bull"], timeout=30, mode="multi")
                                    print("ORDER KEYWORDS:", keyword_list)
                                
                                    if keyword_list == ['TIMEOUT']: # this is just so i can get out of this loop
                                        valid_order = True 
                                    elif len(keyword_list) != 2 or keyword_list == ['ERROR']:
                                        # Speak: "Ohh, there seems to be a problem with your order. Please select exactly two items for your order. Let's try again."
                                        self.robot.set_speech(filename="restaurant/invalid_order", wait_for_end_of=True)
                                    else:
                                        valid_order = True

                                if keyword_list == ['TIMEOUT']: 
                                    order_received = True
                                    self.robot.set_speech(filename="restaurant/not_have_an_order", wait_for_end_of=True)
                                    self.state = self.task_states["Approach_customer"]
                                    ### incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                                        ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                                    if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                        self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                                    self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                                    0.0 ] ### should be changed later
                                        self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                                self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                                self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                        self.DETECTED_CUSTOMER_INDEX += 1
                                    else:
                                        self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process
                                
                                else: # a correct (already filtered for 2 items) order

                                    self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                                    
                                    # print(self.all_orders)
                                    self.robot.set_rgb(command=GREEN+BLINK_LONG)
                                    # print(self.all_orders)

                                    self.robot.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                                    for kw in keyword_list:
                                        print(kw)
                                        self.robot.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                                    
                                    order_received = True  # Sai do loop se a confirmação for "yes"
                                    self.state = self.task_states["Go_back_to_barman_with_order"]

                            else: # if answer == ["no"]: # also consider timeout as a "no" answer, to not be stuck in this part 
                                order_received = True
                                self.robot.set_speech(filename="restaurant/not_have_an_order", wait_for_end_of=True)
                                self.state = self.task_states["Approach_customer"]
                                ### incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                                    ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                                if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                    self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                                self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                                0.0 ] ### should be changed later
                                    self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                    self.DETECTED_CUSTOMER_INDEX += 1
                                else:
                                    self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process

                        else:

                            ### SPEAK: Unfortunetely there was a problem with my understanding of your order. I am sorry for this. I have to move on to other customers.
                            self.robot.set_speech(filename="restaurant/could_not_understand_order_fatal", wait_for_end_of=True)

                            order_received = True
                            self.state = self.task_states["Approach_customer"]
                            ### incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                                ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                            if self.DETECTED_CUSTOMER_INDEX < len(self.detected_customers):
                                self.CUSTOMER_NAV_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                            self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                            0.0 ] ### should be changed later
                                self.CUSTOMER_COORDS = [self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.x,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.y,
                                                        self.detected_customers[self.DETECTED_CUSTOMER_INDEX].position_absolute.z ] ### should be changed later
                                self.DETECTED_CUSTOMER_INDEX += 1
                            else:
                                self.state = self.task_states["Move_to_barman_after_delivery"] # to restart the searching process
                            

            elif self.state == self.task_states["Go_back_to_barman_with_order"]:

                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.adjust_omnidirectional_position(dx=-0.1, dy=0.0, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.SAFE_NAV_COORDS_TO_BARMAN, first_rotate=True, orient_after_move=True, reached_radius=0.7, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.BARMAN_NAV_COORDS, first_rotate=True, orient_after_move=True, reached_radius=1.0, wait_for_end_of=True)
                self.robot.adjust_obstacles(distance=self.MIN_DISTANCE_TO_BARMAN, direction=0.0, max_speed=0.1, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Collect_order_from_barman"]
                # self.state = self.task_states["Approch_customer_with_order"] # debug


            elif self.state == self.task_states["Collect_order_from_barman"]:

                # self.robot.set_neck_coords(position=self.BARMAN_COORDS, wait_for_end_of=True)
                self.is_object_in_hand = True

                ##### SPEAK: Barman, please give me the following items:
                self.robot.set_speech(filename="restaurant/say_order_to_barman", wait_for_end_of=True)
                #  TEST PREDEFINED ORDERS:  
                self.all_orders = [["Mustard", "Sugar"]]

                print("ALL ORDERS: ", self.all_orders)
                current_order = []
                for orders in self.all_orders:
                    for pedido in orders:

                        # Define o nome do arquivo correspondente ao elemento
                        filename = "objects_names/" + pedido.lower().replace(" ", "_")  # Supondo que os arquivos estejam na pasta "restaurant" e tenham o mesmo nome que os elementos em minúsculas
                        
                        current_order.append(pedido.lower().replace(" ", "_"))

                        # SPEAK: Diz o pedido
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)

                print(" CHECK FOR ALLO RDERS ", self.all_orders)
                tetas = [[0, -45], [-40, -45], [40, -45]]
                ########## HERE YOU HAVE TO USE: current_order and not all.orders !!!!!!!!!!!!!!!!!!!!!!!!!
                #try
                
                counter = 0
                for obj in self.NCP:
                    self.NCP[counter] = obj.lower().replace(" ", "_")
                    counter = counter + 1

                counter = 0
                for obj in self.NCT:
                    self.NCT[counter] = obj.lower().replace(" ", "_")
                    counter = counter + 1


                for order_names in self.all_orders:
                    counter = 0
                    for obj in order_names:
                        order_names[counter] = obj.lower().replace(" ", "_")
                        counter = counter + 1

                    NCP_check = set(self.NCP).intersection(order_names)
                    NCT_check = set(self.NCT).intersection(order_names)
                    counter = 0
                    if len(NCP_check)==0:
                        if len(NCT_check)==0:
                            self.all_orders = self.robot.sort_for_pick(objects= order_names)
                            print("Order to pick ", order_names)
                            #try for o in current_order: 
                    
                            for o in order_names:
                                    
                                if counter == 0:
                                    #### SPEAK: please place these object on the bar counter
                                    self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                    filename = "objects_names/" + o.lower().replace(" ", "_")
                                    self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                    self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                    time.sleep(5.0)

                                    _,_ = self.robot.pick_object_risky(selected_object=o, return_arm_to_initial_position = "initial_position_to_ask_for_objects", first_search_tetas=tetas)

                                    object_in_gripper = False
                                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                    if not object_in_gripper:
                                        self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                        o_aux = DetectedObject()
                                        o_aux.object_name = o.lower().replace(" ", "_")
                                        self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)

                                    self.robot.place_object_in_furniture(selected_object=o, asked_help= True, furniture="Tray", place_mode = self.robot.get_standard_pick_from_object(o))
                                else:
                                    #### SPEAK: please place these object on the bar counter
                                    self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                    filename = "objects_names/" + o.lower().replace(" ", "_")
                                    self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                    self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                    time.sleep(5.0)
                                    picked_height_1 ,asked_help_1 = self.robot.pick_object_risky(selected_object=o, first_search_tetas=tetas)
        
                                    object_in_gripper = False
                                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                    if not object_in_gripper:
                                        self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                        o_aux = DetectedObject()
                                        o_aux.object_name = o.lower().replace(" ", "_")
                                        self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)

                                counter = counter + 1         
                        elif len(NCT_check)==1:

                            if list(NCT_check)[0] == order_names[0]:
                                obj = order_names[0]
                                order_names[0] = order_names[1]
                                order_names[1] = obj

                            for o in order_names:
                                    
                                if counter == 0:
                                    #### SPEAK: please place these object on the bar counter
                                    self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                    filename = "objects_names/" + o.lower().replace(" ", "_")
                                    self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                    self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                    time.sleep(5.0)
                                    _,_ = self.robot.pick_object_risky(selected_object=o, return_arm_to_initial_position = "initial_position_to_ask_for_objects", first_search_tetas=tetas) 

                                    object_in_gripper = False
                                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                    if not object_in_gripper:
                                        self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                        o_aux = DetectedObject()
                                        o_aux.object_name = o.lower().replace(" ", "_")
                                        self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)


                                    self.robot.place_object_in_furniture(selected_object=o, asked_help= True, furniture="Tray", place_mode = self.robot.get_standard_pick_from_object(o))
                                else:
                                    #### SPEAK: please place these object on the bar counter
                                    self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                    filename = "objects_names/" + o.lower().replace(" ", "_")
                                    self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                    self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                    time.sleep(5.0)
                                    picked_height_1 ,asked_help_1 = self.robot.pick_object_risky(selected_object=o, first_search_tetas=tetas)

                                    object_in_gripper = False
                                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                    if not object_in_gripper:
                                        self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                        o_aux = DetectedObject()
                                        o_aux.object_name = o.lower().replace(" ", "_")
                                        self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)

                                counter = counter + 1

                        elif len(NCT_check)==2:

                            for o in NCT_check:
                                    
                                    if counter == 0:
                                        #### SPEAK: please place these object on the bar counter
                                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                        filename = "objects_names/" + o.lower().replace(" ", "_")
                                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                        self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                        time.sleep(5.0)
                                        _,_ = self.robot.pick_object_risky(selected_object=o, return_arm_to_initial_position = "initial_position_to_ask_for_objects", first_search_tetas=tetas) 

                                        object_in_gripper = False
                                        object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                        if not object_in_gripper:
                                            self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                            o_aux = DetectedObject()
                                            o_aux.object_name = o.lower().replace(" ", "_")
                                            self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)


                                        self.robot.place_object_in_furniture(selected_object=o, asked_help= True, furniture="Tray", place_mode = "top", NCT=True)
                                    else:
                                        #### SPEAK: please place these object on the bar counter
                                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                                        filename = "objects_names/" + o.lower().replace(" ", "_")
                                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                                        self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                                        time.sleep(5.0)
                                        picked_height_1 ,asked_help_1 = self.robot.pick_object_risky(selected_object=o, first_search_tetas=tetas)

                                        object_in_gripper = False
                                        object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                                        if not object_in_gripper:
                                            self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                                            o_aux = DetectedObject()
                                            o_aux.object_name = o.lower().replace(" ", "_")
                                            self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)


                                    counter = counter + 1


                    elif len(NCP_check)==1:

                        if list(NCP_check)[0] == order_names[0]:
                            obj = order_names[0]
                            order_names[0] = order_names[1]
                            order_names[1] = obj
                        
                        #### SPEAK: please place these object on the bar counter
                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                        filename = "objects_names/" + order_names[0].lower().replace(" ", "_")
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                        self.robot.set_speech(filename="restaurant/in_bar_table", wait_for_end_of=True)
                        time.sleep(5.0)

                        picked_height_1 ,asked_help_1 = self.robot.pick_object_risky(selected_object=order_names[0], first_search_tetas=tetas)

                        object_in_gripper = False
                        object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                        if not object_in_gripper:
                            self.robot.set_speech("generic/problem_pick_object", wait_for_end_of=False)
                            o_aux = DetectedObject()
                            o_aux.object_name = o.lower().replace(" ", "_")
                            self.robot.ask_help_pick_object_gripper(object_d=o_aux, look_judge=[0,0], show_detection=False)

                        ## CREATE SPEECH
                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                        filename = "objects_names/" + order_names[1].lower().replace(" ", "_")
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                        self.robot.set_speech(filename="restaurant/on_my_tray", wait_for_end_of=True)
                        time.sleep(5.0)
                        
                        print("PUT IN TRAY", order_names[1])

                    elif len(NCP_check)==2:

                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                        filename = "objects_names/" + order_names[0].lower().replace(" ", "_")
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                        self.robot.set_speech(filename="restaurant/on_my_tray", wait_for_end_of=True)
                        time.sleep(5.0)


                        print("PUT IN TRAY", order_names[0])

                        self.robot.set_speech(filename="restaurant/please_place", wait_for_end_of=True)
                        filename = "objects_names/" + order_names[1].lower().replace(" ", "_")
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)
                        self.robot.set_speech(filename="restaurant/on_my_tray", wait_for_end_of=True)
                        time.sleep(5.0)

                        print("PUT IN TRAY", order_names[1])

                        self.is_object_in_hand = False



                self.state = self.task_states["Approch_customer_with_order"]


            elif self.state == self.task_states["Approch_customer_with_order"]:

                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.adjust_omnidirectional_position(dx=-0.2, dy=0.0, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.SAFE_NAV_COORDS_TO_CUSTOMER, first_rotate=True, orient_after_move=True, reached_radius=0.7, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.CUSTOMER_NAV_COORDS, first_rotate=False, orient_after_move=False, reached_radius=2.0, wait_for_end_of=True)

                s, m, d = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                if d > self.MIN_DISTANCE_TO_CUSTOMER:
                    self.robot.adjust_obstacles(distance=self.MIN_DISTANCE_TO_CUSTOMER, direction=0.0, max_speed=0.1, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Deliver_order"]
                # self.state = self.task_states["Move_to_barman_after_delivery"] # debug

            elif self.state == self.task_states["Deliver_order"]:

                self.robot.detected_person_to_face_path(person=self.detected_customers[self.DETECTED_CUSTOMER_INDEX-1], send_to_face=True)
                
                self.robot.set_speech(filename="restaurant/i_have_your_order", wait_for_end_of=True)
                self.robot.set_speech(filename="restaurant/please_take_order_from_tray", wait_for_end_of=True)

                # self.is_object_in_hand = True ### FOR PORTUGAL OPEN WE ASSUME THERE IS ALWAYS ONE OF THE ITEMS IN THE ROBOT HAND
                
                if self.is_object_in_hand:
                    # time.sleep(5.0)
                    self.robot.set_speech(filename="restaurant/give_order_from_gripper", wait_for_end_of=True)
                    self.robot.set_arm(command="initial_position_to_ask_for_objects", wait_for_end_of=True)
                    self.robot.set_speech(filename="restaurant/open_gripper_3_2_1", wait_for_end_of=True)
                    self.robot.set_arm(command="open_gripper", wait_for_end_of=True)
                    time.sleep(3.0)
                    self.robot.set_arm(command="close_gripper", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    self.robot.set_speech(filename="restaurant/enjoy_your_order", wait_for_end_of=True)
                else:
                    order_taken = False
                    order_taken_ctr = 0
                    max_attempts = 3
                    while not order_taken and order_taken_ctr <= max_attempts:
                        order_taken_ctr+=1
                        self.robot.set_speech(filename="restaurant/take_an_order_touchscreen", wait_for_end_of=True)
                        answer = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=["yes", "no"], timeout=10, instruction="Did you take your order?", speak_results=False, wait_for_end_of=True)

                        if answer == ["yes"]:
                            order_taken = True
                            self.robot.set_speech(filename="restaurant/enjoy_your_order", wait_for_end_of=True)
                        else:
                            self.robot.set_speech(filename="restaurant/i_will_wait_longer", wait_for_end_of=True)

                        if not order_taken_ctr and order_taken_ctr == max_attempts:
                            self.robot.set_speech(filename="restaurant/problem_taking_order", wait_for_end_of=True)


                self.state = self.task_states["Move_to_barman_after_delivery"] 
            

            elif self.state == self.task_states["Move_to_barman_after_delivery"]:

                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.adjust_omnidirectional_position(dx=-0.1, dy=0.0, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.SAFE_NAV_COORDS_TO_BARMAN, first_rotate=True, orient_after_move=True, reached_radius=0.7, wait_for_end_of=True)
                self.robot.sdnl_move_to_position(move_coords=self.BARMAN_NAV_COORDS, first_rotate=True, orient_after_move=True, reached_radius=1.0, wait_for_end_of=True)
                self.robot.adjust_obstacles(distance=0.20, direction=0.0, max_speed=0.1, wait_for_end_of=True)
                if self.BARMAN_SIDE.lower() == "left":
                    self.robot.adjust_angle(angle=-90, wait_for_end_of=True)
                else:
                    self.robot.adjust_angle(angle=90, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Detecting_waving_customers"]


            elif self.state == self.task_states["Final_State"]:

                # code here ...
                
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
