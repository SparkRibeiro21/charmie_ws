"""
HOW THE CODE OF A TASK SHOULD BE MADE:

->  ->  ->  ->  ->  HOW TO CREATE A TASK?

1) COPY THE TASK TEMPLATE TO YOUR TASK PKG. ALL STD FUNCTIONS ARE INCLUDED FROM CHARMIE_STD_FUNCTIONS (if any doubt ask Tiago Ribeiro)
2) PLAN THE STATES AND SET THE STATES FOR YOUR TASK AND SET THE TASK NAME:

        # Task Name
        self.TASK_NAME = "Serve Breakfast"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":       0,
            "Move_milk_location":           1,
            "Detect_and_pick_milk":         2,
            "Move_cornflakes_location":     3,
            "Detect_and_pick_cornflakes":   4,
            "Move_dishes_location":         5,
            "Detect_and_pick_dishes":       6,
            "Move_kitchen_table":           7,
            "Placing_bowl":                 8,
            "Placing_cornflakes":           9,
            "Placing_milk":                 10,
            "Placing_spoon":                11,
            "Final_State":                  12,
        }

# 3) ADAPT THE MODULES BEING USED ALLONG THE TESTING ONE BY ONE:

    ros2_modules = {
        "charmie_arm":              True,
        "charmie_audio":            False,
        "charmie_face":             False,
        "charmie_head_camera":      True,
        "charmie_hand_camera":      True,
        "charmie_base_camera":      True,
        "charmie_gamepad":          False,
        "charmie_lidar":            True,
        "charmie_lidar_bottom":     True,
        "charmie_lidar_livox":      False,
        "charmie_llm":              False,
        "charmie_localisation":     True,
        "charmie_low_level":        True,
        "charmie_navigation":       False,
        "charmie_nav2":             True,
        "charmie_neck":             True,
        "charmie_radar":        False,
        "charmie_speakers":         True,
        "charmie_tracking":         False,
        "charmie_yolo_objects":     True,
        "charmie_yolo_pose":        False,
    }

# 4) CREATE THE STATE STRUCTURE:
        
        if self.state == self.task_states["Waiting_for_task_start"]:
            # your code here ...
                            
            # next state
            self.state = self.task_states["Move_milk_location"]

        elif self.state == self.task_states["Move_milk_location"]:
            # your code here ...
                            
            # next state
            self.state = self.task_states["Detect_and_pick_milk"]

        elif self.state == self.task_states["Detect_and_pick_milk"]:
            # your code here ...
                            
            # next state
            self.state = self.task_states["Move_cornflakes_location"]

            (...)

# 5) CREATE THE CONFIGURABES FOR THE TASK: (PARAMETERS THAT HAVE TO BE CHANGED DEPENDING ON THE ARENA)

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Which objects should be acquired
        self.GET_MILK = True
        self.GET_CORNFLAKES = True
        self.GET_DISHES = True
        self.IS_CORNFLAKES_BIG = False # choose whether the cornflakes package is a big one (False) or a small one (True)

        # Name of the table where breakfast is served
        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = "Dinner Table"

        # Initial Position
        self.initial_position = self.robot.get_navigation_coords_from_furniture("Entrance")

# 6) CREATE THE PSEUDOCODE OF EACH STATE:
            
            self.state = self.task_states["Detect_and_pick_dishes"]
                
                ##### NECK LOOKS AT TABLE

                ##### MOVES ARM TO TOP OF TABLE POSITION

                ##### SPEAK: Searching for objects

                ##### YOLO OBJECTS SEARCH FOR SPOON, FOR BOTH CAMERAS
                
                ##### SPEAK: Found spoon
                
                ##### SPEAK: Check face to see object detected

                ##### SHOW FACE DETECTED OBJECT

                ##### MOVE ARM TO PICK UP OBJECT 

                ##### IF AN ERROR IS DETECTED:

                    ##### SPEAK: There is a problem picking up the object

                    ##### MOVE ARM TO ERROR POSITION 
                
                    ##### NECK LOOK JUDGE

                    ##### SPEAK: Need help, put object on my hand as it is on my face

                    ##### SHOW FACE GRIPPER SPOON 

                    ##### WHILE OBJECT IS NOT IN GRIPPER:

                        ##### SPEAK: Close gripper in 3 2 1 

                        ##### ARM: CLOSE GRIPPER

                        ##### IF OBJECT NOT GRABBED: 

                            ##### SPEAK: There seems to be a problem, please retry.

                            ##### ARM OPEN GRIPPER
                        
                ##### NECK LOOK TRAY
                        
                ##### ARM PLACE OBJECT IN TRAY

                self.state = self.Picking_up_milk

# 7) REPLACE ALL THE SPEAKS IN PSEUDOCODE WITH self.robot.set_speech(...), CREATE FILES IN ros2 run charmie_speakers save_audio

# 8) TEST ALL SENTENCES ALONE TO SEE IF EVERYTHING IS OK

# 9) REPLACE ALL THE FACE IN PSEUDOCODE WITH self.robot.set_face(...)

# 10) TEST ALL FACES WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 11) REPLACE ALL THE START_BUTTON AND RGB IN PSEUDOCODE WITH self.robot.set_rgb(...) and self.robot.wait_for_start_button()

# 12) TEST ALL START_BUTTON AND RGB WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 13) REPLACE ALL THE AUDIO IN PSEUDOCODE WITH self.robot.get_audio(...)

# 14) TEST ALL AUDIO WITH THE PREVIOUS GETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 15) REPLACE ALL THE NECK IN PSEUDOCODE WITH self.robot.set_neck(...) OR ANY OF ALL THE OTHER FORMS TO SET THE NECK

# 16) TEST ALL NECK WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 17) REPLACE ALL THE YOLO DETECTIONS IN PSEUDOCODE WITH self.robot.search_for_person(...) and self.robot.search_for_objects

# 18) TEST ALL YOLOS WITH THE PREVIOUS SEARCHES_FOR ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 19) REPLACE ALL THE ARM MOVE WITH self.robot.set_arm(...)

# 20) TEST ALL ARM MOVE WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 21) REPLACE THE SET INITIAL POSITION MOVE WITH self.robot.set_initial_position(...)

# 22) TEST ALL SET INITIAL POSITION WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

# 23) REPLACE THE NAVIGATION MOVE, ROTATE AND ORIENTATE WITH self.robot.move_to_position(...)

# 24) TEST ALL SET NAVIGATION WITH THE PREVIOUS SETs ALREADY IMPLEMENTED TO SEE IF EVERYTHING IS OK

"""

#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject, DetectedPerson
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

        # Task Name
        self.TASK_NAME = "Serve Breakfast"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":       0,
            "Move_milk_location":           1,
            "Detect_and_pick_milk":         2,
            "Move_cornflakes_location":     3,
            "Detect_and_pick_cornflakes":   4,
            "Move_dishes_location":         5,
            "Detect_and_pick_dishes":       6,
            "Move_kitchen_table":           7,
            "Placing_bowl":                 8,
            "Placing_cornflakes":           9,
            "Placing_milk":                 10,
            "Placing_spoon":                11,
            "Final_State":                  12,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Which objects should be acquired
        self.GET_MILK = True
        self.GET_CORNFLAKES = True
        self.GET_DISHES = True
        self.IS_CORNFLAKES_BIG = False # choose whether the cornflakes package is a big one (False) or a small one (True)

        # Name of the table where breakfast is served
        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = "Dinner Table"

        # Initial Position
        self.initial_position = self.robot.get_navigation_coords_from_furniture("Entrance")
        print(self.initial_position)
        self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)

    # main state-machine function
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED.lower().replace(" ", "_")
        # Checks if there is any error in the furniture variables:
        if self.robot.get_room_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED) == None:
            print("ERROR!!! - FURNITURE:", self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, "DOES NOT EXIST IN furniture.json")
            while True:
                pass
        self.SB_TABLE_HEIGHT = self.robot.get_height_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
        print("Table Height =", self.SB_TABLE_HEIGHT)
        
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
                # your code here ...
                
                # moves the neck to look down for navigation
                # self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                """
                # send speech command to speakers voice, intrucing the robot 
                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
                
                # sends RGB value for debug
                self.set_rgb(command=CYAN+HALF_ROTATE)

                # change face, to face picking up cup
                # self.set_face("help_pick_cup")
                
                # waiting for start button
                self.wait_for_door_start()

                self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)

                self.set_speech(filename="serve_breakfast/sb_moving_kitchen_counter", wait_for_end_of=True)

                print("DONE2 - ", self.node.start_button_state)
                # calibrate the background noise for better voice recognition
                # self.calibrate_audio(wait_for_end_of=True)

                # moves the neck to look to absolute cordinates in the map
                # self.set_neck_coords(position=[1.0, 1.0], ang=30, wait_for_end_of=True)
                """
                self.robot.set_initial_position(self.initial_position)

                
                ### RECEPTIONIST AUDIO EXAMPLE
                # command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                # print("Finished:", command)
                # keyword_list= command.split(" ")
                # print(keyword_list[0], keyword_list[1])
                # self.set_speech(filename="receptionist/recep_first_guest_"+keyword_list[0].lower(), wait_for_end_of=True)
                # self.set_speech(filename="receptionist/recep_drink_"+keyword_list[1].lower(), wait_for_end_of=True)  

                # change face, to standard face
                # self.set_face("charmie_face")

                # moves the neck to look forward
                # self.set_neck(position=self.look_forward, wait_for_end_of=False)

                # example of arm function to say hello
                # self.set_arm(command="hello", wait_for_end_of=False)

                # next state
                self.state = self.task_states["Move_milk_location"]

            elif self.state == self.task_states["Move_milk_location"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Detect_and_pick_milk"]


            elif self.state == self.task_states["Detect_and_pick_milk"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Move_cornflakes_location"]

            elif self.state == self.task_states["Move_cornflakes_location"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Detect_and_pick_cornflakes"]


            elif self.state == self.task_states["Detect_and_pick_cornflakes"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Move_dishes_location"]


            elif self.state == self.task_states["Move_dishes_location"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Detect_and_pick_dishes"]


            elif self.state == self.task_states["Detect_and_pick_dishes"]:

                ##### NECK LOOKS AT TABLE
                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### MOVES ARM TO TOP OF TABLE POSITION

                ##### SPEAK: Searching for objects
                # self.set_speech(filename="generic/search_objects", wait_for_end_of=True)

                ##### YOLO OBJECTS SEARCH FOR SPOON, FOR BOTH CAMERAS
                # self.set_neck(position=self.look_judge, wait_for_end_of=True)
                
                ##### SPEAK: Found spoon
                # self.set_speech(filename="serve_breakfast/sb_found_spoon", show_in_face=True, wait_for_end_of=True)
                
                ##### SPEAK: Check face to see object detected
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)

                ##### SHOW FACE DETECTED OBJECT (CUSTOM)

                ##### MOVE ARM TO PICK UP OBJECT 

                ##### IF AN ERROR IS DETECTED:

                    ##### SPEAK: There is a problem picking up the object
                    # self.set_speech(filename="generic/problem_pick_object", wait_for_end_of=True) # False

                    ##### MOVE ARM TO ERROR POSITION 
                
                    ##### NECK LOOK JUDGE
                    # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                    ##### SPEAK: Need help, put object on my hand as it is on my face
                    # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                    ##### SHOW FACE GRIPPER SPOON 
                    # self.set_face("help_pick_spoon") 

                    ##### WHILE OBJECT IS NOT IN GRIPPER:

                        ##### SPEAK: Close gripper in 3 2 1 
                        # self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                        ##### ARM: CLOSE GRIPPER

                        ##### IF OBJECT NOT GRABBED: 

                            ##### SPEAK: There seems to be a problem, please retry.
                            # self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)

                            ##### ARM OPEN GRIPPER
                
                ##### SET FACE TO STANDARD FACE
                # self.set_face("charmie_face")
                        
                ##### NECK LOOK TRAY
                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                        
                ##### ARM PLACE OBJECT IN TRAY

                self.state = self.task_states["Move_kitchen_table"]


            elif self.state == self.task_states["Move_kitchen_table"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Placing_bowl"]


            elif self.state == self.task_states["Placing_bowl"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Placing_cornflakes"] 
            

            elif self.state == self.task_states["Placing_cornflakes"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Placing_milk"]

           
            elif self.state == self.task_states["Placing_milk"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Placing_spoon"]


            elif self.state == self.task_states["Placing_spoon"]:
                # your code here ...
                                
                # next state
                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                # self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                # self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                # self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=False)

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
