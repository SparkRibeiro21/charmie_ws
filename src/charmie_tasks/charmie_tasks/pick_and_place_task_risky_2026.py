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
    "charmie_audio":                False,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           False,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 True,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
    "charmie_speakers_save":        False,
    "charmie_tracking":             False,
    "charmie_yolo_objects":         True,
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

        # Task Name
        self.TASK_NAME = "Pick & Place"

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
            "Move_cutlery_location":        12,
            "Detect_and_pick_cutlery":      13,
            "Move_dishwasher_location":     14,
            "Placing_cutlery":              15,
            "Final_State":                  16,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Which objects should be acquired
        self.GET_MILK            = True
        self.GET_CORNFLAKES      = True
        self.GET_BOWL            = True
        self.GET_BREAKFAST_SPOON = False
        self.GET_CUTLERY         = True
        self.IS_CORNFLAKES_BIG   = False # choose whether the cornflakes package is a big one (False) or a small one (True)

        # whether we know in advance that one of the objects we want the judge to help CHARMIE due to some physical constraint in picking the object
        self.HELP_PICK_MILK         = False
        self.HELP_PICK_CORNFLAKES   = False
        self.HELP_PICK_SPOON        = False
        self.HELP_PICK_BOWL         = False
        self.HELP_PICK_CUTLERY      = False

        # Name of the table where breakfast is served
        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = "Dinner Table"

        # Name of cutlery to pick (only if GET_CUTLERY is True)
        # self.CUTLERY_TO_PICK = "Fork"
        self.CUTLERY_LOCATION = "Dinner Table"

        # In case there is a need to reverse the order in which milk/cornflakes is picked
        self.MILK_BEFORE_CORNFLAKES = True

        # Objects picked furniture names
        # self.MILK_LOCATION = "Pantry"
        self.MILK_LOCATION = "Cabinet"
        self.CORNFLAKES_LOCATION = "Cabinet"
        self.DISHES_LOCATION = "Kitchen Counter"

        # Initial Position
        #self.initial_position = self.robot.get_navigation_coords_from_furniture("dishwasher")
        self.initial_position = [0.0, 0.0, 0.0]
        # self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        self.SEARCH_CUTLERY_COORDS = [2.58, -2.85, 90.0] # FNR position for where dining table's side is
        self.DISHWASHER_LOCATION = [ 4.08, -3.15, 0]

        self.SELECTED_PICKED_DISH = DetectedObject()

        print(self.initial_position)
        
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED.lower().replace(" ", "_")
        self.CUTLERY_LOCATION = self.CUTLERY_LOCATION.lower().replace(" ", "_")

        self.MILK_LOCATION = self.MILK_LOCATION.lower().replace(" ", "_")
        self.CORNFLAKES_LOCATION = self.CORNFLAKES_LOCATION.lower().replace(" ", "_")
        self.DISHES_LOCATION = self.DISHES_LOCATION.lower().replace(" ", "_")

        # Checks if there is any error in the furniture variables:
        if self.robot.get_room_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED) == None:
            print("ERROR!!! - FURNITURE:", self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, "DOES NOT EXIST IN furniture.json")
            while True:
                pass
        self.SB_TABLE_HEIGHT = self.robot.get_height_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)[0]+0.005
        print("Table Height =", self.SB_TABLE_HEIGHT)
        # Set the height of the table where breakfast is served, so that the manual arm movements are adapted to this height (placing and pouring)
        self.robot.set_height_furniture_for_arm_manual_movements(self.SB_TABLE_HEIGHT) #####
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]]
        self.search_for_cutlery_tetas =[[-15, -20], [15, -20]]

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

                self.robot.set_speech(filename="pick_and_place_task/pp_ready_start", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.wait_for_door_opening()

                self.robot.enter_house_after_door_opening()

                if self.MILK_BEFORE_CORNFLAKES:
                    self.state = self.task_states["Move_milk_location"]
                else:
                    self.state = self.task_states["Move_cornflakes_location"]
                

            elif self.state == self.task_states["Move_milk_location"]:
                                        
                if self.GET_MILK:
                    
                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.MILK_LOCATION, wait_for_end_of=False)

                    self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.MILK_LOCATION), wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.MILK_LOCATION, wait_for_end_of=False)
                
                self.state = self.task_states["Detect_and_pick_milk"]


            elif self.state == self.task_states["Detect_and_pick_milk"]:

                if self.GET_MILK:
                    
                    if not self.HELP_PICK_MILK:
                        #self.robot.set_speech(filename="pick_and_place_task/open_milk_lid", wait_for_end_of=True)
                        #time.sleep(8.0)
                        self.robot.pick_object_risky(selected_object="Milk", return_arm_to_initial_position="collect_milk_to_tray",first_search_tetas = [[0.0, -20.0],[0.0, 0.0], [0.0, -30.0]])
                        ### here logic should be changed because, it does not make sense to go to ask_for_objects_position before initial_position seince ip is already so close
                        # self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
                    else:
                        object_in_gripper = False
                        while not object_in_gripper:
                            objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_furniture=False)
                            
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                            
                            if not object_in_gripper:
                                self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                        self.robot.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)

                
                if self.MILK_BEFORE_CORNFLAKES:

                    self.state = self.task_states["Move_cornflakes_location"]
                else:    
                    self.state = self.task_states["Move_dishes_location"]


            elif self.state == self.task_states["Move_cornflakes_location"]:

                if self.GET_CORNFLAKES:

                    if self.MILK_LOCATION == self.CORNFLAKES_LOCATION:

                        self.state = self.task_states["Detect_and_pick_cornflakes"]

                    else:
                    
                        self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                        self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/"+self.CORNFLAKES_LOCATION , wait_for_end_of=False)
                        #self.robot.set_speech(filename="pick_and_place_task/open_milk_lid", wait_for_end_of=True)

                        self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.CORNFLAKES_LOCATION), wait_for_end_of=True)

                        self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/"+self.CORNFLAKES_LOCATION, wait_for_end_of=False)
                                        
                self.state = self.task_states["Detect_and_pick_cornflakes"]


            elif self.state == self.task_states["Detect_and_pick_cornflakes"]:

                if self.GET_CORNFLAKES:

                    if not self.HELP_PICK_CORNFLAKES:
                        self.robot.pick_object_risky(selected_object="Cornflakes", return_arm_to_initial_position="collect_cornflakes_to_tray",first_search_tetas = [[0.0, -20.0],[0.0, 0.0], [0.0, -30.0]])
                        ### here logic should be changed because, it does not make sense to go to ask_for_objects_position before initial_position seince ip is already so close
                        # self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                    else:
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
                            time.sleep(1.0) # the final arms movements have wfeo as False, so we need a small delay here to make sure the arm is inside the robot before it starts moving while the arm is still going to the initial position

                if self.MILK_BEFORE_CORNFLAKES:
                    self.state = self.task_states["Move_dishes_location"]
                else:
                    self.state = self.task_states["Move_milk_location"]


            elif self.state == self.task_states["Move_dishes_location"]:

                if self.GET_BOWL or self.GET_BREAKFAST_SPOON:

                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.DISHES_LOCATION, wait_for_end_of=False)
                    
                    self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.DISHES_LOCATION), wait_for_end_of=True)

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.DISHES_LOCATION, wait_for_end_of=False)
                
                self.state = self.task_states["Detect_and_pick_dishes"]


            elif self.state == self.task_states["Detect_and_pick_dishes"]:

                if self.GET_BREAKFAST_SPOON:

                    if not self.HELP_PICK_SPOON:
                        self.robot.pick_object_risky(selected_object="Spoon", list_of_objects_detected_as= [["Fork", "Knife"]], return_arm_to_initial_position="collect_spoon_to_tray_funilocopo_v4")

                    else:
                        object_in_gripper = False
                        while not object_in_gripper:
                            objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Spoon"], list_of_objects_detected_as=[["Fork", "Knife"]], use_arm=False, detect_objects=True, detect_furniture=False)
                            
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                            
                            if not object_in_gripper:
                                self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                        self.robot.set_arm(command="collect_spoon_to_tray_funilocopo_v4", wait_for_end_of=True)

                if self.GET_BOWL:
                    
                    if not self.HELP_PICK_BOWL:
                        self.robot.pick_object_risky(selected_object="Bowl")
                        
                    else:
                        object_in_gripper = False
                        while not object_in_gripper:
                            objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=["Bowl"], use_arm=False, detect_objects=True, detect_furniture=False)
                            
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                            
                            if not object_in_gripper:
                                self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                        self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                    
                    # This is left here on purpose because in the legacy version we did spoon and bowl at the same time, if we need this in the future, just uncomment this
                    """ object_in_gripper = False
                    correct_object_
                    bowl = DetectedObject()
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
                    self.robot.ask_help_pick_object_tray(object_d=correct_object_spoon, look_judge=self.look_judge, first_help_request=False, bb_color=(0, 255, 0), audio_confirmation=False) """

                self.state = self.task_states["Move_kitchen_table"]


            elif self.state == self.task_states["Move_kitchen_table"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)
                # self.robot.set_speech(filename="pick_and_place_task/remove_chairs", wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.SEARCH_CUTLERY_COORDS, wait_for_end_of=True)

                cutlery = self.robot.search_for_objects(tetas=self.search_for_cutlery_tetas, list_of_objects=[], use_arm=True, detect_objects=True)

                move_coords = self.robot.add_rotation_to_pick_position(self.robot.get_navigation_coords_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED))
                                
                self.robot.move_to_position(move_coords=move_coords, wait_for_end_of=True)

                self.robot.adjust_obstacles(distance=0.3, direction=-45.0, wait_for_end_of=False)


                #VARIABLE TO ENSURE ONLY CUTLERY IS PICKED
                no_other_cutlery = True

                for c in cutlery:
                    if (c.object_name == "Fork" or c.object_name == "Spoon" or c.object_name == "Knife") and no_other_cutlery:

                        fc = self.robot.get_location_coords_from_furniture(furniture=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                        if c.position_absolute.y > fc[1]:
                            self.robot.set_speech(filename="pick_and_place_task/remove_chair_laundry_room", wait_for_end_of=False)
                        else:
                            self.robot.set_speech(filename="pick_and_place_task/remove_chair_kitchen_counter", wait_for_end_of=False)
                        # time.sleep(8.0)    
                        self.SELECTED_PICKED_DISH = c     
                        no_other_cutlery = False                          
        
                
                if no_other_cutlery == True:
                    for c in cutlery:
                        if c.object_name == "Cup" and no_other_cutlery:

                            fc = self.robot.get_location_coords_from_furniture(furniture=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                            if c.position_absolute.y > fc[1]:
                                self.robot.set_speech(filename="pick_and_place_task/remove_chair_laundry_room", wait_for_end_of=False)
                            else:
                                self.robot.set_speech(filename="pick_and_place_task/remove_chair_kitchen_counter", wait_for_end_of=False)
                            # time.sleep(8.0)  

                            self.SELECTED_PICKED_DISH = c  
                            no_other_cutlery = False                             
        
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                # self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_table_objects, wait_for_end_of=False)
                
                self.state = self.task_states["Placing_bowl"]


            elif self.state == self.task_states["Placing_bowl"]:

                if self.GET_BOWL:
                    
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
                    self.robot.place_object(arm_command="pre_pour_milk_bowl_risky", speak_before=False, speak_after=False)
                    self.robot.set_speech(filename="pick_and_place_task/milk_open_risky", wait_for_end_of=True)
                    time.sleep(7.0)
                    self.robot.place_object(arm_command="post_pour_milk_bowl_risky", speak_before=False, speak_after=True, verb="pour", object_name="milk", preposition="into", furniture_name="bowl")
                
                    ##### ARM PLACE OBJECT
                    self.robot.place_object(arm_command="place_milk_table", speak_before=False, speak_after=True, verb="place", object_name="milk", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                    
                self.state = self.task_states["Placing_spoon"]


            elif self.state == self.task_states["Placing_spoon"]:

                if self.GET_BREAKFAST_SPOON:
                    self.robot.place_object(arm_command="place_spoon_table_funilocopo_v4", speak_before=False, speak_after=True, verb="place", object_name="spoon", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                self.robot.adjust_omnidirectional_position(dx=-0.12, dy=0.12, print_feedback=False, wait_for_end_of=True)
                self.state = self.task_states["Detect_and_pick_cutlery"]


            elif self.state == self.task_states["Detect_and_pick_cutlery"]:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                if self.GET_CUTLERY:
                    if not self.HELP_PICK_CUTLERY:

                        if self.SELECTED_PICKED_DISH.object_name in ["Fork", "Spoon", "Knife"]:
                            self.robot.move_to_pre_pick_position_after_search_for_objects(furniture=self.CUTLERY_LOCATION, object=self.SELECTED_PICKED_DISH)
                            pick_height_cutlery, _ = self.robot.pick_object_risky(selected_object=self.SELECTED_PICKED_DISH.object_name, list_of_objects_detected_as=[["Spoon", "Fork", "Knife"]], say_cutlery=True)
                        else:
                            self.robot.move_to_pre_pick_position_after_search_for_objects(furniture=self.CUTLERY_LOCATION, object=self.SELECTED_PICKED_DISH)
                            pick_height_cutlery, _ = self.robot.pick_object_risky(selected_object=self.SELECTED_PICKED_DISH.object_name)


                        if no_other_cutlery == True:
                            print("NO OBJECT DETECTED")
                            self.state = self.task_states["Final_State"]

                    else:
                        object_in_gripper = False
                        while not object_in_gripper:
                            objects_found = self.robot.search_for_objects(tetas=self.search_tetas, time_in_each_frame=2.0, list_of_objects=[self.CUTLERY_TO_PICK], use_arm=False, detect_objects=True, detect_furniture=False)
                            
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                            
                            if not object_in_gripper:
                                self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                        self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                self.state = self.task_states["Move_dishwasher_location"]


            elif self.state == self.task_states["Move_dishwasher_location"]:

                if self.GET_CUTLERY:

                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/dishwasher", wait_for_end_of=False)

                    self.robot.move_to_position(move_coords = self.DISHWASHER_LOCATION, wait_for_end_of=True)
                    
                    # commented out for time optimization
                    # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    # self.robot.set_speech(filename="furniture/dishwasher", wait_for_end_of=False)

                self.state = self.task_states["Placing_cutlery"]


            elif self.state == self.task_states["Placing_cutlery"]:

                if self.GET_CUTLERY:

                    _ , _ , furniture_distance = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=30)
                    self.robot.set_neck(position=self.look_judge, wait_for_end_of=False) 
                    self.robot.set_speech(filename="pick_and_place_task/help_open_dishwasher_door_and_top_rack", wait_for_end_of=True)
                    time.sleep(4.0)
                    self.robot.set_neck(position=self.look_forward, wait_for_end_of=False) 
                    # self.robot.place_object_in_furniture(selected_object=self.CUTLERY_TO_PICK, place_mode = self.robot.get_standard_pick_from_object(object_name=self.CUTLERY_TO_PICK), place_height = 0.187, furniture = "rack", navigation_distance= furniture_distance - 0.60)
                    self.robot.place_object_in_dishwasher_top_rack_and_close_rack(selected_object=c.object_name, place_mode = self.robot.get_standard_pick_from_object(object_name=c.object_name), place_height = pick_height_cutlery, navigation_distance= furniture_distance - 0.60)
                    # self.robot.set_speech(filename="clean_the_table/ask_to_close_dishwasher_rack", wait_for_end_of=True)
                    # time.sleep(1.0)
                    # self.robot.close_dishwasher(task = "pp")
                
                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="pick_and_place_task/pp_finished", wait_for_end_of=False)

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
