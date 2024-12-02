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
    "charmie_face":             True,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_lidar":            True,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False, # 
    "charmie_neck":             True,
    "charmie_obstacles":        False, # 
    "charmie_odometry":         False, #
    "charmie_point_cloud":      True,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         True,
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

    def main(self):
        
        # Task Related Variables
        self.Waiting_for_task_start = 0
        self.Approach_milk_location = 1
        self.Detect_and_pick_milk = 2
        self.Approach_cornflakes_location = 3
        self.Detect_and_pick_cornflakes = 4
        self.Approach_dishes_location = 5
        self.Detect_and_pick_dishes = 6
        self.Approach_kitchen_table = 7
        self.Placing_bowl = 8
        self.Placing_milk = 9
        self.Placing_cornflakes = 10
        self.Placing_spoon = 11
        self.Final_State = 12
    
        # Configurables
        self.GET_MILK = True
        self.GET_CORNFLAKES = True
        self.GET_DISHES = True
        self.IS_CORNFLAKES_BIG = False # choose whether the cornflakes package is a big one (False) or a small one (True)
        self.SB_TABLE_HEIGHT = 0.76
        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = "Dinner Table"
        
        self.TABLE_APPROACH_OBSTACLES = 0.35
        self.COUNTER_APPROACH_OBSTACLES = 0.25
        self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED = self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED.lower().replace(" ", "_")

        # Checks if there is any error in the furniture variables:
        if self.robot.get_room_from_furniture(self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED) == None:
            print("ERROR!!! - FURNITURE:", self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, "DOES NOT EXIST IN furniture.json")
            while True:
                pass

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]] # , [-45-10, -45-5], [-45+10, -45-5]]

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # Navigation Positions
        self.front_of_door = [0.0, 1.5] 
        self.cofee_table = [-0.4, 3.5]
        self.side_table = [-0.4, 4.5]
        self.kitchen_counter = [-0.4, 5.5]
        self.kitchen_table = [-2.0, 6.8]
        
        ### ROBOCUP24
        self.MAX_SPEED = 40

        self.pre_room_door = [0.45, 3.55]
        self.post_room_door = [0.45, 4.70]
        self.front_of_start_door = [0.0, 1.0]
        self.front_sofa = [0.45, 5.8]
        self.midway_living_room = [-0.9, 8.5]
        self.close_to_garbage_bin = [-2.76, 5.9]
        self.close_to_dishwasher = [-2.26, 8.0]
        self.close_to_table_sb = [-4.26, 8.8]
        self.pre_table = [-4.76, 6.0]

        self.state = self.Waiting_for_task_start

        print("IN SERVE THE BREAKFAST MAIN")

        while True:

            if self.state == self.Waiting_for_task_start:

                self.robot.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")
                print("GET_MILK:", self.GET_MILK, "GET_CORNFLAKES:", self.GET_CORNFLAKES, "GET_DISHES:", self.GET_DISHES)

                time.sleep(1)
        
                self.robot.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)

                self.robot.set_face("charmie_face")

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.wait_for_door_start()

                # self.state = self.Approach_cornflakes_location
                self.state = self.Detect_and_pick_milk # debug without NAV


            elif self.state == self.Approach_milk_location:
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    
                self.robot.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    
                if self.GET_MILK:
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=False)

                    self.robot.set_navigation(movement="rotate", target=self.cofee_table, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.cofee_table, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("milk")), wait_for_end_of=False)
                
                self.state = self.Detect_and_pick_milk


            elif self.state == self.Detect_and_pick_milk:

                if self.GET_MILK:

                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Milk"], list_of_objects_detected_as=[["cleanser"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                # self.state = self.Approach_cornflakes_location
                self.state = self.Detect_and_pick_cornflakes # debug without NAV


            elif self.state == self.Approach_cornflakes_location:

                if self.GET_CORNFLAKES:
                    
                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=False)

                    
                    self.robot.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_rgb(BLUE+ROTATE)
                    # self.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.pre_room_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_rgb(MAGENTA+ROTATE)
                    self.robot.set_navigation(movement="rotate", target=self.post_room_door, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.post_room_door, reached_radius=0.6, flag_not_obs=False, wait_for_end_of=True)
                    self.robot.set_rgb(BLUE+ROTATE)
                    self.robot.set_navigation(movement="rotate", target=self.front_sofa, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.front_sofa, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_rgb(MAGENTA+ROTATE)
                    self.robot.set_navigation(movement="rotate", target=self.midway_living_room, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.midway_living_room, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_rgb(BLUE+ROTATE)

                    self.robot.set_navigation(movement="rotate", target=self.close_to_table_sb, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.close_to_table_sb, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_rgb(BLUE+ROTATE)
                    
                    # debug
                    # self.set_initial_position([0.0, 0.0, 90.0])

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("cornflakes")), wait_for_end_of=False)
                
                    self.robot.set_navigation(movement="orientate", absolute_angle= 45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    
                self.state = self.Detect_and_pick_cornflakes


            elif self.state == self.Detect_and_pick_cornflakes:

                if self.GET_CORNFLAKES:

                    object_in_gripper = False
                    while not object_in_gripper:
                        objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, list_of_objects=["Cornflakes"], list_of_objects_detected_as=[["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                        
                        if self.IS_CORNFLAKES_BIG:
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, alternative_help_pick_face="help_pick_cornflakes1", bb_color=(0, 255, 0))
                        else:
                            object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=objects_found[0], look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)

                # self.state = self.Detect_and_pick_dishes
                self.state = self.Detect_and_pick_dishes # debug without NAV


            elif self.state == self.Approach_dishes_location:

                if self.GET_DISHES:
                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)
                    
                    self.robot.set_navigation(movement="rotate", target=self.kitchen_counter, flag_not_obs=True, wait_for_end_of=True)
                    self.robot.set_navigation(movement="move", target=self.kitchen_counter, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                    
                    self.robot.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)
                    
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)
                
                
                self.state = self.Detect_and_pick_dishes


            elif self.state == self.Detect_and_pick_dishes:

                if self.GET_DISHES:
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
                        object_in_gripper = self.robot.ask_help_pick_object_gripper(object_d=correct_object_bowl, look_judge=self.look_judge, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        if not object_in_gripper:
                            self.robot.set_speech(filename="generic/check_detection_again", wait_for_end_of=True)

                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                    # SPOON
                    self.robot.ask_help_pick_object_tray(object_d=correct_object_spoon, look_judge=self.look_judge, first_help_request=False, bb_color=(0, 255, 0), audio_confirmation=False)

                # self.state = self.Approach_kitchen_table
                self.state = self.Placing_bowl # debug without NAV


            elif self.state == self.Approach_kitchen_table:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)

                self.robot.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs = True, wait_for_end_of=True)
                self.robot.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=self.COUNTER_APPROACH_OBSTACLES, wait_for_end_of=True)
                    
                # self.set_navigation(movement="rotate", target=self.kitchen_table, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="move", target=self.kitchen_table, max_speed=20.0, reached_radius=1.0, flag_not_obs=False, wait_for_end_of=True)

                self.robot.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)
                                
                self.robot.set_navigation(movement="orientate", absolute_angle= 180.0, flag_not_obs=True, wait_for_end_of=True)

                # self.set_navigation(movement="adjust_angle", absolute_angle= 0.0, flag_not_obs=True, wait_for_end_of=True)
                time.sleep(3)
                self.robot.set_speech(filename="serve_breakfast/remove_decorations_table", wait_for_end_of=False)

                self.robot.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=self.TABLE_APPROACH_OBSTACLES, wait_for_end_of=True)
                
                self.robot.set_navigation(movement="orientate", absolute_angle= 225.0, flag_not_obs=True, wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED, wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_table_objects, wait_for_end_of=False)
                
                self.state = self.Placing_bowl


            elif self.state == self.Placing_bowl:

                if self.GET_DISHES:
                    
                    self.robot.place_object(arm_command="place_bowl_table", speak_before=False, speak_after=True, verb="place", object_name="bowl", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    # self.robot.set_arm(command="place_bowl_table", wait_for_end_of=True)

                self.state = self.Placing_cornflakes 
            

            elif self.state == self.Placing_cornflakes:

                if self.GET_CORNFLAKES:
                    ##### ARM POUR IN BOWL
                    self.robot.place_object(arm_command="pour_cereals_bowl", speak_before=False, speak_after=True, verb="pour", object_name="cornflakes", preposition="into", furniture_name="bowl")
                    # self.robot.set_arm(command="pour_cereals_bowl_alternative_robocup_cornflakes", wait_for_end_of=True)
                    # self.robot.set_arm(command="pour_cereals_bowl", wait_for_end_of=True)
                    
                    ##### ARM PLACE OBJECT
                    self.robot.place_object(arm_command="place_cereal_table", speak_before=False, speak_after=True, verb="place", object_name="cornflakes", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    # self.robot.set_arm(command="place_cereal_table_alternative_robocup_cornflakes", wait_for_end_of=True)
                    # self.robot.set_arm(command="place_cereal_table", wait_for_end_of=True)
                
                self.state = self.Placing_milk

           
            elif self.state == self.Placing_milk:

                if self.GET_MILK:
                    ##### ARM POUR IN BOWL
                    self.robot.place_object(arm_command="pour_milk_bowl", speak_before=False, speak_after=True, verb="pour", object_name="milk", preposition="into", furniture_name="bowl")
                    # self.robot.set_arm(command="pour_milk_bowl", wait_for_end_of=True)

                    ##### ARM PLACE OBJECT
                    self.robot.place_object(arm_command="place_milk_table", speak_before=False, speak_after=True, verb="place", object_name="milk", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    # self.robot.set_arm(command="place_milk_table", wait_for_end_of=True)
                    
                self.state = self.Placing_spoon


            elif self.state == self.Placing_spoon:

                if self.GET_DISHES:
                    self.robot.place_object(arm_command="place_spoon_table", speak_before=False, speak_after=True, verb="place", object_name="spoon", preposition="on", furniture_name=self.NAME_TABLE_WHERE_BREAKFAST_IS_SERVED)
                    # self.robot.set_arm(command="place_spoon_table", wait_for_end_of=True)

                self.state = self.Final_State 


            elif self.state == self.Final_State:
                
                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=False)

                while True:
                    pass

            else:
                pass
            