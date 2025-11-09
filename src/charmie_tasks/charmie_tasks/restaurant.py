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
    "charmie_arm":                  True,
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
    "charmie_tracking":             True,
    "charmie_yolo_objects":         True,
    "charmie_yolo_pose":            True,
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

        # Which objects should be acquired
        # self.GET_MILK = True
        # self.GET_CORNFLAKES = True
        # self.GET_DISHES = True
        # self.IS_CORNFLAKES_BIG = False # choose whether the cornflakes package is a big one (False) or a small one (True)

        # Name of the table where breakfast is served
        # self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = "Dinner Table"

        # Initial Position
        self.initial_position = [0.0, 0.0, 0.0]
        # self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        print(self.initial_position)
        
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
        
        self.BARMAN_NAV_COORDS = [0.0, 0.0, 0.0] # x, y, theta
        self.CUSTOMER_NAV_COORDS = [3.0, 3.0, 0.0] # x, y, theta

        self.BARMAN_COORDS = [0.0, 0.0, 0.0] # x, y, z
        self.CUSTOMER_COORDS = [] # x, y, z

        self.all_orders = []

        # Neck Positions
        self.look_forward = [0, 0]
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

                self.robot.set_initial_position(self.initial_position)

                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="restaurant/start_restaurant", wait_for_end_of=True)

                self.robot.wait_for_start_button()

                time.sleep(3.0) # time for person who pressen start button leave to not be shown in qualif video

                self.state = self.task_states["Looking_for_barman"]
                

            elif self.state == self.task_states["Looking_for_barman"]:
               
                tetas = [[180, 0], [90, 0], [-90, 0]]
                barman = []

                # Check for people BACK, LEFT and RIGHT, to figure out who is the barman
                while not barman:
                    self.robot.set_speech(filename="restaurant/search_barman", wait_for_end_of=True)
                    
                    people_found = self.robot.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True, only_detect_person_right_in_front=True)

                    print("FOUND:", len(people_found)) 
                    for p in people_found.people:
                        barman.append(p)
                        
                        # all below can be commented
                        self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute.z], wait_for_end_of=True)
                        print("ID:", p.index)
                        print('Barman position', p.position_relative)
                        time.sleep(2.0)

                    # this 'for' can be merged with the previous 'for'
                    for b in barman:
                        self.robot.set_neck_coords(position=[b.position_absolute.x, b.position_absolute.y, b.position_absolute.z], wait_for_end_of=True)

                        self.robot.set_speech(filename="confirm_barman_touchscreen", wait_for_end_of=True)
                        answer = self.robot.set_face_touchscreen_menu(custom_options=["yes", "no"], timeout=10, instruction="Are you the barman?", speak_results=True, wait_for_end_of=True)

                        if answer == "yes":
                            ### calculate barman angle and position to me 
                            self.BARMAN_NAV_COORDS[2] = 0.0 ### CALCULATE ANGLE TO BARMAN, SO WHEN WE COME BACK WE ARE ALREADY FACING THE BARMAN
                            self.BARMAN_COORDS = [b.position_absolute.x, b.position_absolute.y, b.position_absolute.z]

                            ##### SPEAK : Hello! Nice to meet you! My name is charmie and I am here to help you serve the customers.
                            self.robot.set_speech(filename="restaurant/barman_meeting", wait_for_end_of=True)

                            ##### SPEAK : I am going to turn around and search for possible customers. See you soon
                            self.robot.set_speech(filename="restaurant/go_search", wait_for_end_of=True)

                            break

                
                self.state = self.task_states["Detecting_waving_customers"]


            elif self.state == self.task_states["Detecting_waving_customers"]:

                tetas = [[-60, 0], [0, 0], [60, 0]]
                customers_list = []
            
                while not customers_list:

                    customers_found = self.robot.search_for_person(tetas=tetas, delta_t=2.0, only_detect_person_arm_raised=True)

                    print("FOUND:", len(customers_found)) 
                    for p in customers_found.people:
                        # customers_list.append(p)
                        
                        # all below can be commented
                        self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute.z], wait_for_end_of=True)
                        print("ID:", p.index)
                        print('Customer position', p.position_relative)


                    ### REORDER BY DISTANCE ###


                    print('Nr of detected customers waving: ', len(customers_list))

                    if len(customers_found) > 0:
                        # moves back to barman and looks at barman
                        self.robot.move_to_position(move_coords=self.BARMAN_NAV_COORDS, wait_for_end_of=True) 
                        self.robot.set_neck_coords(position=self.BARMAN_COORDS, wait_for_end_of=True)

                        # SPEAK: i have found some customers, lets confirm these are customers
                        # for customers
                        #   SPEAK: i have found the following customer.
                        #   SPEAK: please check my face to see the detected customer
                        #   CARA: mostra customer
                        #   SPEAK: Is this a customer, please press my face "yes" or "no"
                        #   FACE: TOUCHSCREEEN
                        #   if "yes"
                        #       add to customers_list = []
                        #   aqui posso por um contador, para se ele detetar 20 pessoas, não ir a todas, pôr um maximo de 2 ou 3 

                        # Confirm, while looking at barman.
                        # SPEAK: I confirm I have the following customers
                        # FOR
                        #   FACE: mostrar caras de todos os que foram ditos que sim
                        #   

                        # set self.CUSTOMER_COORDS.append() customer_list[0] ,...
                        # set self.CUSTOMER_NAV_COORDS = customer_list[0]

                    else:
                        # if no customer is found, moves a little bit forward, with safety radar ON
                        self.robot.set_speech(filename="restaurant/no_customers", wait_for_end_of=False)
                        self.robot.adjust_omnidirectional_position(dx=0.5, dy=0.0, wait_for_end_of=True)

                self.state = self.task_states["Approach_customer"]


            elif self.state == self.task_states["Approach_customer"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.CUSTOMER_NAV_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Receive_order"]


            elif self.state == self.task_states["Receive_order"]:

                ### pergunta sim ou nao se tem um pedido
                    ### se nao, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                        ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                    ### se conter > 5, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                        ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                    ### se sim continua
                     
                ### faz a pergunta do pedido
                    ### se counter > 5, passa para a cara e tenta duas vezes
                        ### caso exceda as duas vezes da cara, incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                    ### se sim vai para pergunta de confrirmacao

                ### pergunta de confirmacao
                    ### se sim, avança para o proximo estado 
                    ### se nao volta a fazer o pedido     
                     
                      
                # order_collected = False
                # while not order_collected:

                self.robot.set_rgb(command=BLUE+ALTERNATE_QUARTERS)
                self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)

                customer_has_order = False
                customer_has_order_ctr = 0
                max_asks_customer_has_order = 5
                while not customer_has_order:
                    customer_has_order_ctr+= 1

                    # "Do you have an order?"
                    confirmation = self.robot.get_audio(yes_or_no=True, question="restaurant/have_an_order", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
                    print("Finished:", confirmation)

                    if confirmation == "yes":
                        customer_has_order = True
                    elif confirmation == "no" or customer_has_order_ctr >= max_asks_customer_has_order:
                        customer_has_order = True
                        ### incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                            ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                        self.state = self.task_states["Approach_customer"]


                if self.state == self.task_states["Receive_order"]:

                    order_received = False
                    order_received_ctr = 0
                    max_asks_audio_order_received = 5
                    max_asks_touchscreen_order_received = 2
                    while not order_received:
                        order_received_ctr+=1

                        if order_received_ctr <= max_asks_audio_order_received:
                
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
                                self.all_orders.append(keyword_list)  # Adiciona o pedido à lista de todos os pedidos
                                self.robot.set_rgb(command=GREEN+BLINK_LONG)

                                self.robot.set_speech(filename="restaurant/reforce_order", wait_for_end_of=True)
                                for kw in keyword_list:
                                    print(kw)
                                    self.robot.set_speech(filename="objects_names/" + kw.lower().replace(" ", "_"), wait_for_end_of=True)
                                ##### SPEAK: Thank you
                                # self.set_speech(filename="restaurant/yes_order", wait_for_end_of=True)
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
                            
                            pass
                            ### pedido com touchscreen

                        else:
                            order_received = True
                            ### incrementa o CUSTOMER_NAV_COORDS e vai para o approach customer table
                                ### caso o CUSTOMER_NAV_COORDS tenha chegado ao fim, volta para o estado: Detecting_waving_customers
                            self.state = self.task_states["Approach_customer"]
                            


            elif self.state == self.task_states["Go_back_to_barman_with_order"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.BARMAN_NAV_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Collect_order_from_barman"]


            elif self.state == self.task_states["Collect_order_from_barman"]:

                self.robot.set_neck_coords(position=self.BARMAN_COORDS, wait_for_end_of=True)

                ##### SPEAK: Barman, please give me the following items:
                self.robot.set_speech(filename="restaurant/say_order_to_barman", wait_for_end_of=True)

                current_order = []
                for pedido in self.all_orders:
                    for elemento in pedido:
                        # Define o nome do arquivo correspondente ao elemento
                        filename = "objects_names/" + elemento.lower().replace(" ", "_")  # Supondo que os arquivos estejam na pasta "restaurant" e tenham o mesmo nome que os elementos em minúsculas
                        
                        current_order.append(elemento.lower().replace(" ", "_"))

                        # SPEAK: Diz o elemento do pedido
                        self.robot.set_speech(filename=filename, wait_for_end_of=True)

                #### SPEAK: please place these object on the bar counter

                time.sleep(10.0)

                tetas = [[0, -45], [-40, -45], [40, -45]]
                for o in current_order:
                    self.robot.pick_object(selected_object=o, first_search_tetas=tetas, return_arm_to_initial_position=False)


                # code here ...

                self.state = self.task_states["Approch_customer_with_order"]


            elif self.state == self.task_states["Approch_customer_with_order"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.CUSTOMER_NAV_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Deliver_order"]


            elif self.state == self.task_states["Deliver_order"]:

                ### INVERT LIST ORDER
                ### PLACE OBJECTS ON TABLE - FROM AUTO PICK DEMONSTRATION

                self.state = self.task_states["Move_to_barman_after_delivery"] 
            

            elif self.state == self.task_states["Move_to_barman_after_delivery"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.BARMAN_NAV_COORDS, wait_for_end_of=True)
                
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
