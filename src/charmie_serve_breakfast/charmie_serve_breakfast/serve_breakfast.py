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
    "charmie_radar":            False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     True,
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

                self.robot.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")
                print("GET_MILK:", self.GET_MILK, "GET_CORNFLAKES:", self.GET_CORNFLAKES, "GET_DISHES:", self.GET_DISHES)
        
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                ### self.robot.wait_for_door_start()

                ### self.robot.initial_move_past_entrance_door() # to do ...

                self.state = self.task_states["Move_milk_location"]


            elif self.state == self.task_states["Move_milk_location"]:
                                        
                if self.GET_MILK:

                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=False)

                    self.robot.move_to_position(move_coords=self.robot.add_rotation_to_pick_position(self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")))), wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=False)
                
                self.state = self.task_states["Detect_and_pick_milk"]


            elif self.state == self.task_states["Detect_and_pick_milk"]:

                if self.GET_MILK:

                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                self.state = self.task_states["Move_cornflakes_location"]


            elif self.state == self.task_states["Move_cornflakes_location"]:

                if self.GET_CORNFLAKES:
                    
                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=False)

                    self.robot.move_to_position(move_coords=self.robot.add_rotation_to_pick_position(self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")))), wait_for_end_of=True)

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=False)
                                    
                self.state = self.task_states["Detect_and_pick_cornflakes"]


            elif self.state == self.task_states["Detect_and_pick_cornflakes"]:

                if self.GET_CORNFLAKES:

                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Cornflakes"], list_of_objects_detected_as=[["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                        
                        if self.IS_CORNFLAKES_BIG:
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, alternative_help_pick_face="help_pick_cornflakes1", bb_color=(0, 255, 0))
                        else:
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                self.state = self.task_states["Move_dishes_location"]


            elif self.state == self.task_states["Move_dishes_location"]:

                if self.GET_DISHES:

                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)
                    
                    self.robot.move_to_position(move_coords=self.robot.add_rotation_to_pick_position(self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")))), wait_for_end_of=True)

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)
                
                self.state = self.task_states["Detect_and_pick_dishes"]


            elif self.state == self.task_states["Detect_and_pick_dishes"]:

                if self.GET_DISHES:
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
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=correct_object_bowl, look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                    # SPOON
                    self.robot.ask_help_pick_object_tray(object_d=correct_object_spoon, look_judge=self.look_judge, first_help_request=False, bb_color=(0, 255, 0), audio_confirmation=False)

                self.state = self.task_states["Move_kitchen_table"]


            elif self.state == self.task_states["Move_kitchen_table"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)

                # must be removed after the update to minimize as much as possivle the final orientation error 
                move_coords = self.robot.add_rotation_to_pick_position(self.robot.get_navigation_coords_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED))                
                self.robot.move_to_position(move_coords=move_coords, wait_for_end_of=True)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_table_objects, wait_for_end_of=False)
                
                self.state = self.task_states["Placing_bowl"]


            elif self.state == self.task_states["Placing_bowl"]:

                if self.GET_DISHES:
                    
                    self.robot.place_object(arm_command="place_bowl_table", speak_before=False, speak_after=True, verb="place", object_name="bowl", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    
                self.state = self.task_states["Placing_cornflakes"] 
            

            elif self.state == self.task_states["Placing_cornflakes"]:

                if self.GET_CORNFLAKES:
                    ##### ARM POUR IN BOWL
                    self.robot.place_object(arm_command="pour_cereals_bowl", speak_before=False, speak_after=True, verb="pour", object_name="cornflakes", preposition="into", furniture_name="bowl")
                    
                    ##### ARM PLACE OBJECT
                    self.robot.place_object(arm_command="place_cereal_table", speak_before=False, speak_after=True, verb="place", object_name="cornflakes", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    
                self.state = self.task_states["Placing_milk"]

           
            elif self.state == self.task_states["Placing_milk"]:

                if self.GET_MILK:
                    ##### ARM POUR IN BOWL
                    self.robot.place_object(arm_command="pour_milk_bowl", speak_before=False, speak_after=True, verb="pour", object_name="milk", preposition="into", furniture_name="bowl")
                
                    ##### ARM PLACE OBJECT
                    self.robot.place_object(arm_command="place_milk_table", speak_before=False, speak_after=True, verb="place", object_name="milk", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    
                self.state = self.task_states["Placing_spoon"]


            elif self.state == self.task_states["Placing_spoon"]:

                if self.GET_DISHES:
                    self.robot.place_object(arm_command="place_spoon_table_funilocopo_v2", speak_before=False, speak_after=True, verb="place", object_name="spoon", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                
                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=False)

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
