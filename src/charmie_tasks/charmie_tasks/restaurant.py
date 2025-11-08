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
        
        self.BARMAN_COORDS = [0.0, 0.0, 0.0]
        self.CUSTOMER_COORDS = [3.0, 3.0, 0.0]

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
                                        
                # code here ...
                
                self.state = self.task_states["Detecting_waving_customers"]


            elif self.state == self.task_states["Detecting_waving_customers"]:

                # code here ...

                self.state = self.task_states["Approach_customer"]


            elif self.state == self.task_states["Approach_customer"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.CUSTOMER_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Receive_order"]


            elif self.state == self.task_states["Receive_order"]:

                # code here ...

                self.state = self.task_states["Go_back_to_barman_with_order"]


            elif self.state == self.task_states["Go_back_to_barman_with_order"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.BARMAN_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Collect_order_from_barman"]


            elif self.state == self.task_states["Collect_order_from_barman"]:

                # code here ...

                self.state = self.task_states["Approch_customer_with_order"]


            elif self.state == self.task_states["Approch_customer_with_order"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.CUSTOMER_COORDS, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/customer_table", wait_for_end_of=False)
                        
                self.state = self.task_states["Deliver_order"]


            elif self.state == self.task_states["Deliver_order"]:

                # code here ...

                self.state = self.task_states["Move_to_barman_after_delivery"] 
            

            elif self.state == self.task_states["Move_to_barman_after_delivery"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="restaurant/barman_table", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.BARMAN_COORDS, wait_for_end_of=True)
                
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
