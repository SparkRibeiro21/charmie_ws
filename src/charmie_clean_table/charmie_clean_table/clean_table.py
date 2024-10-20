#!/usr/bin/env python3
import rclpy
import threading
import time
import math
from charmie_interfaces.msg import DetectedObject, DetectedPerson
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            True,
    "charmie_face":             True,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_lidar":            True,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       True,
    "charmie_neck":             True,
    "charmie_obstacles":        True,
    "charmie_odometry":         True,
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
        self.Approach_kitchen_table = 1
        self.Detect_and_pick_all_objects = 2
        self.Detect_and_pick_all_objects_audio = 3
        self.Approach_dishwasher = 4
        self.Open_dishwasher_door = 5
        self.Open_dishwasher_rack = 6
        self.Place_cup = 7
        self.Place_bowl = 8
        self.Place_plate = 9
        self.Place_cutlery1 = 10
        self.Place_cutlery2 = 11
        self.Place_cutlery_funilocopo = 15
        self.Close_dishwasher_rack = 12
        self.Close_dishwasher_door = 13
        self.Final_State = 14

        # Configurables
        self.ATTEMPTS_AT_RECEIVING = 3
        self.SHOW_OBJECT_DETECTED_WAIT_TIME = 4.0
        self.MAX_SPEED = 50
        self.CLOSE_RACK_WITH_PLATE = True
        self.DEBUG_WITHOUT_AUDIO = False
        self.SELECTED_CUTLERY = []
        # self.TABLE_APPROACH_OBSTACLES = 0.45

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.look_dishwasher = [-90, -15]
        self.search_tetas = [[-45, -45], [-45+15, -45+10], [-45-15, -45+10]] # , [-45-10, -45-5], [-45+10, -45-5]]

        # Initial Position
        self.initial_position = [0.0, 0.1, 0.0]

        # Navigation Positions
        self.front_of_door = [0.0, 1.5] 
        self.kitchen_table = [-0.5, 4.5]
        self.dishwasher = [-1.0, 2.0]


        ### ROBOCUP24
        self.MAX_SPEED = 40

        self.pre_room_door = [0.45, 3.55]
        self.post_room_door = [0.45, 4.70]
        self.front_of_start_door = [0.0, 1.0]
        self.front_sofa = [0.45, 5.8]
        self.midway_living_room = [-0.9, 8.5]
        self.close_to_garbage_bin = [-2.76, 5.9]
        self.close_to_dishwasher = [-2.26, 8.0]
        self.close_to_table_sb = [-4.26, 8.5]
        self.pre_table = [-4.76, 6.0]

        self.TABLE_APPROACH_OBSTACLES = 0.25

        self.first_time_giving_audio_instructions = True

        # self.state = self.Place_cup
        self.state = self.Waiting_for_task_start


        while True:

            if self.state == self.Waiting_for_task_start:

                self.robot.set_initial_position(self.initial_position)
                print("SET INITIAL POSITION")
                
                time.sleep(1)
        
                # self.robot.set_navigation(movement="orientate", absolute_angle= -45.0, flag_not_obs = True, wait_for_end_of=True)

                self.robot.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=False)

                self.robot.set_face("charmie_face")

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="clean_the_table/ready_start_ct", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.wait_for_door_start()

                self.state = self.Approach_kitchen_table


            elif self.state == self.Approach_kitchen_table:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                
                # self.robot.set_navigation(movement="move", target=self.front_of_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                
                self.robot.set_speech(filename="serve_breakfast/sb_moving_kitchen_table", wait_for_end_of=False)


                self.robot.set_navigation(movement="move", target=self.front_of_start_door, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.robot.set_rgb(BLUE+ROTATE)
                # self.robot.set_navigation(movement="rotate", target=self.pre_room_door, flag_not_obs=True, wait_for_end_of=True)
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
                

                self.robot.set_speech(filename="serve_breakfast/sb_arrived_kitchen_table", wait_for_end_of=False)

                # self.robot.set_navigation(movement="rotate", target=self.kitchen_table, flag_not_obs=True, wait_for_end_of=True)
                # self.robot.set_navigation(movement="move", target=self.kitchen_table, max_speed=self.MAX_SPEED, reached_radius=0.6, flag_not_obs=True, wait_for_end_of=True)
                self.robot.set_navigation(movement="orientate", absolute_angle= 225.0, flag_not_obs = True, wait_for_end_of=True)

                # self.robot.set_speech(filename="serve_breakfast/sb_arrived_kitchen_table", wait_for_end_of=True)
                
                self.state = self.Detect_and_pick_all_objects_audio


            elif self.state == self.Detect_and_pick_all_objects_audio:
                print ("GO audio")
                
                correct_object = DetectedObject()
                list_of_objects = ["Knife", "Fork", "Spoon", "Plate", "Bowl", "Cup"]
                list_of_objects_copy = []

                self.robot.calibrate_audio()
                
                while list_of_objects:
                
                    list_of_objects_copy = list_of_objects.copy()   
                    
                    objects_found = self.robot.search_for_objects(tetas=self.search_tetas, delta_t=2.0, use_arm=False, detect_objects=True, detect_shoes=False, detect_furniture=False)
                    
                    print("while_start:", list_of_objects) 
                    print("while_start_copy:", list_of_objects_copy)    

                    for o in objects_found:
                        self.robot.set_rgb(command=CYAN+HALF_ROTATE)

                        if o.object_name in list_of_objects_copy:
                            correct_object = o
                            pretended_obj = o.object_name
                        
                            print(pretended_obj, list_of_objects)    
                            print(pretended_obj, list_of_objects_copy) 

                            confirmation = self.ask_judge_for_object(curr_obj=pretended_obj, correct_object=correct_object)
                            print("confirmation:", confirmation)

                            if confirmation.lower() == "yes":
                                self.robot.set_rgb(command=GREEN+HALF_ROTATE)
                                self.robot.set_speech(filename="generic/thank_you", wait_for_end_of=True)

                                list_of_objects_copy.remove(pretended_obj)
                                if pretended_obj == "Knife" or pretended_obj == "Spoon" or pretended_obj == "Fork":
                                    self.SELECTED_CUTLERY.append(pretended_obj)

                                    # the robot must only move two pieces of cutlery
                                    if "Knife" not in list_of_objects_copy and "Spoon" not in list_of_objects_copy:
                                        list_of_objects_copy.remove("Fork")
                                    elif "Fork" not in list_of_objects_copy and "Spoon" not in list_of_objects_copy:
                                        list_of_objects_copy.remove("Knife")
                                    elif "Knife" not in list_of_objects_copy and "Fork" not in list_of_objects_copy:
                                        list_of_objects_copy.remove("Spoon")
                                
                            else:
                                self.robot.set_rgb(command=RED+HALF_ROTATE)
                                self.robot.set_speech(filename="generic/misdetection_move_to_next", wait_for_end_of=True)
                        

                    list_of_objects = list_of_objects_copy   

                    if list_of_objects:
                        # self.robot.set_speech(filename="clean_the_table/search_again_misdetected_objects", wait_for_end_of=True)
                    
                        # Speech: "There seems to be a problem with detecting the objects. Can you please slightly move and rotate the following objects?"
                        self.robot.set_speech(filename="generic/problem_detecting_change_object", wait_for_end_of=True) 
                        for obj in list_of_objects:
                            # Speech: (Name of object)
                            self.robot.set_speech(filename="objects_names/"+obj.replace(" ","_").lower(), wait_for_end_of=True)
                
                print(self.SELECTED_CUTLERY)

                self.state = self.Approach_dishwasher


            elif self.state == self.Approach_dishwasher:
                
                """
                self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.set_speech(filename="clean_the_table/moving_dishwasher", wait_for_end_of=True)
                self.set_navigation(movement="rotate", target=self.dishwasher, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="move", target=self.dishwasher, max_speed=20.0, reached_radius=0.8, flag_not_obs=True, wait_for_end_of=True)
                self.set_navigation(movement="orientate", absolute_angle=90.0, flag_not_obs = True, wait_for_end_of=True)
                """
                # self.robot.set_navigation(movement="adjust_angle", absolute_angle=90.0, flag_not_obs=True, wait_for_end_of=True)
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                # self.robot.set_speech(filename="serve_breakfast/sb_moving_kitchen_table", wait_for_end_of=False)

                self.robot.activate_obstacles(obstacles_lidar_up=True, obstacles_camera_head=True)

                self.robot.set_rgb(BLUE+HALF_ROTATE)


                self.robot.set_navigation(movement="orientate", absolute_angle= 180.0, flag_not_obs = True, wait_for_end_of=True)
                self.robot.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=self.TABLE_APPROACH_OBSTACLES, wait_for_end_of=True)
                    
                # self.robot.set_navigation(movement="rotate", target=self.kitchen_table, flag_not_obs=True, wait_for_end_of=True)
                # self.robot.set_navigation(movement="move", target=self.kitchen_table, max_speed=20.0, reached_radius=1.0, flag_not_obs=False, wait_for_end_of=True)

                                
                self.robot.set_navigation(movement="orientate", absolute_angle= 0.0, flag_not_obs=True, wait_for_end_of=True)

                # self.robot.set_navigation(movement="adjust_angle", absolute_angle= 0.0, flag_not_obs=True, wait_for_end_of=True)

                self.robot.set_speech(filename="clean_the_table/approaching_dishwasher", wait_for_end_of=False)

                perfectly_centered = 2
                perfectly_centered_ctr = 0
                while perfectly_centered_ctr < perfectly_centered:
                    perfectly_centered_ctr += 1

                    self.robot.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)

                    dishwasher_found = False
                    while not dishwasher_found:
                        if perfectly_centered_ctr <= 1:
                            # tetas = [[0, -30], [20, -30], [-20, -30]]
                            tetas = [[0, -30], [20, -30], [-20, -30]]
                        else:
                            tetas = [[0, -30]]
                        # tetas = [[0, -30], [20, -30], [-20, -30], [-40, -30], [40, -30]]
                        objects_found = self.robot.search_for_objects(tetas=tetas, delta_t=2.0, use_arm=False, detect_objects=False, detect_shoes=False, detect_furniture=True)
                        print('pos-search')
                        for obj in objects_found:
                            if obj.object_name == 'Dishwasher':
                                dishwasher_found = True
                                dishwasher_position = obj.position_relative
                                print('Object found')

                        if not dishwasher_found:
                            self.robot.set_rgb(command=MAGENTA+HALF_ROTATE)
                            self.robot.set_navigation(movement="adjust", adjust_distance=0.1, adjust_direction=0.0, wait_for_end_of=True)
                            self.robot.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                        else:
                            self.robot.set_rgb(command=CYAN+HALF_ROTATE)
                            
                    print('rel_pos:', dishwasher_position.x, dishwasher_position.y, dishwasher_position.z)   

                    robot_radius = 0.28

                    if perfectly_centered_ctr == 1:
                        distance_to_dishwasher = 1.00 
                    else:
                        distance_to_dishwasher = 0.80 
                        
                    distance_x_to_center = dishwasher_position.x + 0.05
                    distance_y_to_center = dishwasher_position.y - robot_radius - distance_to_dishwasher - 0.1 # there is always a 10 cm difference, not sure why... added 0.1 to fix it 
                    
                    print('d_lateral:', distance_x_to_center)
                    print('d_frontal:', distance_y_to_center)
                    
                    ang_to_bag = -math.degrees(math.atan2(distance_x_to_center, distance_y_to_center))
                    dist_to_bag = (math.sqrt(distance_x_to_center**2 + distance_y_to_center**2))
                    print(ang_to_bag, dist_to_bag)
                    self.robot.set_navigation(movement="adjust", adjust_distance=dist_to_bag, adjust_direction=ang_to_bag, wait_for_end_of=True)

                    time.sleep(2.0)
                    # alternative solution to make sure i am at the right distance from the dishwasher
                    # self.robot.set_navigation(movement="adjust_obstacle", adjust_direction=0.0, adjust_min_dist=0.5, wait_for_end_of=True)
                        
                self.robot.set_speech(filename="clean_the_table/arrived_dishwasher", wait_for_end_of=False)

                self.state = self.Open_dishwasher_door


            elif self.state == self.Open_dishwasher_door:

                ### CODE HERE (NOT IMPLEMENTED FOR NOW ...)
                time.sleep(3)
                self.robot.set_speech(filename="clean_the_table/can_not_open_dishwasher_door_quick", wait_for_end_of=True)
                time.sleep(3)
                self.robot.set_speech(filename="clean_the_table/remove_cutlery_tray", wait_for_end_of=False)
                time.sleep(5)
                
                ### CONFIRM YES/NO FULLY OPENED THE DISHWASHER

                # self.robot.set_arm(command="open_dishwasher_door", wait_for_end_of=True)

                self.state = self.Open_dishwasher_rack


            elif self.state == self.Open_dishwasher_rack:


                # The 175 rather than 180 is to force the adjustement
                self.robot.set_navigation(movement="orientate", absolute_angle=80.0, flag_not_obs = True, wait_for_end_of=True)    
                self.robot.set_navigation(movement="adjust_angle", absolute_angle=90.0, flag_not_obs=True, wait_for_end_of=True)

                time.sleep(3)

                self.robot.set_navigation(movement="adjust_angle", absolute_angle=90.0, flag_not_obs=True, wait_for_end_of=False)

                ### CODE HERE (NOT IMPLEMENTED FOR NOW ...)
                # time.sleep(5)
                # self.robot.set_speech(filename="clean_the_table/can_not_open_dishwasher_rack", wait_for_end_of=True)

                # self.robot.set_arm(command="open_dishwasher_rack", wait_for_end_of=True)

                # JUST FOR DEBUG
                # self.robot.set_arm(command="initial_pose_to_ask_for_objects", wait_for_end_of=False)
                # self.SELECTED_CUTLERY = ["Fork", "Knife"]

                self.state = self.Place_cup


            elif self.state == self.Place_cup:

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)
                
                self.robot.set_face("help_pick_cup_ct")
                
                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)

                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                object_in_gripper = False
                while not object_in_gripper:
                                        
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:

                        self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)

                self.robot.set_face("charmie_face")
                
                self.robot.set_speech(filename="clean_the_table/placing_cup", wait_for_end_of=False)

                self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher", wait_for_end_of=True)
                
                self.robot.set_arm(command="place_cup_in_dishwasher", wait_for_end_of=True)

                self.state = self.Place_bowl


            elif self.state == self.Place_bowl:

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.robot.set_face("help_pick_bowl")
                
                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)

                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                object_in_gripper = False
                while not object_in_gripper:
                                        
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:

                        self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)

                self.robot.set_face("charmie_face")
                
                self.robot.set_speech(filename="clean_the_table/placing_bowl", wait_for_end_of=False)

                self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher_special_bowl", wait_for_end_of=True)
                
                self.robot.set_arm(command="place_bowl_in_dishwasher", wait_for_end_of=True)

                self.state = self.Place_plate

            elif self.state == self.Place_cutlery_funilocopo:

                # self.robot.set_neck(position=self.look_forwas, wait_for_end_of=False)
                
                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                #### MISSING IN ARM_CLEAN_TABLE
                # self.robot.set_arm(command="ask_for_objects_to_get_funilocopo", wait_for_end_of=True)
                
                self.robot.set_speech(filename="clean_the_table/placing_cutlery", wait_for_end_of=False)

                # self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher", wait_for_end_of=True)
                
                #### MISSING IN ARM_CLEAN_TABLE
                self.robot.set_arm(command="place_cutlery_in_dishwasher", wait_for_end_of=True)

                self.state = self.Place_plate


            elif self.state == self.Place_plate:

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.robot.set_face("help_pick_plate")
                
                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=False)

                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                object_in_gripper = False
                while not object_in_gripper:
                                        
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:

                        self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)
                
                self.robot.set_face("charmie_face")

                self.robot.set_speech(filename="clean_the_table/close_dishwasher_rack", wait_for_end_of=False)

                self.robot.set_speech(filename="clean_the_table/warning_close_dishwasher_rack_with_plate", wait_for_end_of=False)

                self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher", wait_for_end_of=True)
                
                self.robot.set_arm(command="close_dishwasher_rack", wait_for_end_of=True)

                # self.robot.set_arm(command="open_dishwasher_rack", wait_for_end_of=True)

                self.robot.set_torso(legs=0, torso=0) 
                print("TORSO SENT")

                # time.sleep(25)

                self.robot.set_speech(filename="clean_the_table/placing_plate", wait_for_end_of=False)
                
                self.robot.set_arm(command="place_plate_in_dishwasher", wait_for_end_of=True)

                # self.state = self.Place_cutlery_funilocopo
                self.state = self.Close_dishwasher_door

                
            elif self.state == self.Place_cutlery1:

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)
                
                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_face("help_pick_"+self.SELECTED_CUTLERY[0].lower()+"_ct")
                
                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                object_in_gripper = False
                while not object_in_gripper:
                                        
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:

                        self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)
                
                self.robot.set_speech(filename="clean_the_table/placing_cutlery", wait_for_end_of=False)

                self.robot.set_face("charmie_face")

                self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher", wait_for_end_of=True)
                
                self.robot.set_arm(command="place_cutlery_in_dishwasher", wait_for_end_of=True)

                self.state = self.Place_cutlery2


            elif self.state == self.Place_cutlery2:

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)
                
                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_face("help_pick_"+self.SELECTED_CUTLERY[1].lower()+"_ct")
                
                self.robot.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=False)

                object_in_gripper = False
                while not object_in_gripper:
                                        
                    self.robot.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                    if not object_in_gripper:

                        self.robot.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)
                
                self.robot.set_speech(filename="clean_the_table/placing_cutlery", wait_for_end_of=False)

                self.robot.set_face("charmie_face")

                self.robot.set_arm(command="ask_for_objects_to_pre_dishwasher", wait_for_end_of=True)
                
                self.robot.set_arm(command="place_cutlery_in_dishwasher", wait_for_end_of=True)

                self.state = self.Close_dishwasher_door
            

            ### FOR NOW - THE CLOSE RACK MOVEMENT IS INCLUDED IN THE PLACE_PLATE
                """
            elif self.state == self.Close_dishwasher_rack:

                self.robot.set_arm(command="pre_dishwasher_to_ask_for_objects", wait_for_end_of=True)

                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                
                self.robot.set_speech(filename="clean_the_table/close_dishwasher_rack", wait_for_end_of=False)
                
                # while True:
                #     pass
                
                
                self.robot.set_neck(position=self.look_dishwasher, wait_for_end_of=False)
                        
                if self.CLOSE_RACK_WITH_PLATE:
                    ### WITH PLATE
                    self.robot.set_speech(filename="clean_the_table/warning_close_dishwasher_rack_with_plate", wait_for_end_of=False)

                    self.robot.set_arm(command="close_rack_with_plate", wait_for_end_of=True)
                else:
                    ### WITHOUT PLATE
                    self.robot.set_arm(command="close_rack_without_plate", wait_for_end_of=True)

                self.robot.set_speech(filename="clean_the_table/closed_dishwasher_rack", wait_for_end_of=False)

                self.state = self.Close_dishwasher_door
                """

            elif self.state == self.Close_dishwasher_door:

                # self.robot.set_initial_position([0.0, 0.0, 180.0])
                # time.sleep(3)

                self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)

                self.robot.set_speech(filename="clean_the_table/close_bottom_rack", wait_for_end_of=True)

                self.robot.set_arm(command="pre_dishwasher_to_initial_position", wait_for_end_of=False)

                self.robot.set_navigation(movement="orientate", absolute_angle=0.0, flag_not_obs = True, wait_for_end_of=True)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                intended_x_pos = 0.15
                intended_y_pos = 1.20

                distance_x_to_center = 0.0 -intended_x_pos
                distance_y_to_center = 0.75-intended_y_pos
                    
                print('d_lateral:', distance_x_to_center)
                print('d_frontal:', distance_y_to_center)
                    
                ang_to_bag = -math.degrees(math.atan2(distance_x_to_center, distance_y_to_center))
                dist_to_bag = (math.sqrt(distance_x_to_center**2 + distance_y_to_center**2))
                print(ang_to_bag, dist_to_bag)
                self.robot.set_navigation(movement="adjust", adjust_distance=dist_to_bag, adjust_direction=ang_to_bag, wait_for_end_of=False)

                self.robot.set_speech(filename="clean_the_table/close_dishwasher_door", wait_for_end_of=False)

                self.robot.set_arm(command="close_dishwasher_door", wait_for_end_of=False)

                self.robot.set_torso(legs=0, torso=61) 
                
                time.sleep(19)

                self.robot.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)

                intended_y_pos = 0.70
                distance_y_to_center = 1.20-intended_y_pos

                self.robot.set_navigation(movement="adjust", adjust_distance=distance_y_to_center, adjust_direction=0.0, wait_for_end_of=True)
                
                self.robot.set_torso(legs=140, torso=30) 

                # self.robot.set_arm(command="open_gripper", wait_for_end_of=False)
                
                time.sleep(8)

                intended_y_pos = 0.3
                distance_y_to_center = 0.70-intended_y_pos

                self.robot.set_navigation(movement="adjust", adjust_distance=distance_y_to_center, adjust_direction=0.0, wait_for_end_of=True)
                
                self.robot.set_navigation(movement="adjust", adjust_distance=distance_y_to_center, adjust_direction=180.0, wait_for_end_of=True)
                
                self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)

                self.state = self.Final_State 


            elif self.state == self.Final_State:
                
                self.robot.set_speech(filename="clean_the_table/finished_ct", wait_for_end_of=False)
                
                self.robot.set_torso(legs=0, torso=8) 

                # self.set_torso(legs=140, torso=8) 

                while True:
                    pass

            else:
                pass



    def ask_judge_for_object(self, curr_obj, correct_object):

        self.robot.detected_object_to_face_path(object=correct_object, send_to_face=True, bb_color=(0,255,0))

        self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)

        self.robot.set_speech(filename="clean_the_table/found_the_"+curr_obj.lower(), wait_for_end_of=True)  
        
        self.robot.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)

        if self.first_time_giving_audio_instructions:
            time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME) 
        else:
            time.sleep(0.5)     

        self.robot.set_face("place_"+curr_obj.lower()+"_in_tray_ct")

        self.robot.set_speech(filename="clean_the_table/place_"+curr_obj.lower()+"_in_tray", wait_for_end_of=True)  

        if self.first_time_giving_audio_instructions:
            time.sleep(self.SHOW_OBJECT_DETECTED_WAIT_TIME)   
        else:
            time.sleep(0.5)    

        self.robot.set_face("charmie_face")

        confirmation = "yes"
        if not self.DEBUG_WITHOUT_AUDIO:

            if self.first_time_giving_audio_instructions:
                self.robot.set_speech(filename="generic/hear_green_face", wait_for_end_of=True)
                self.robot.set_speech(filename="generic/say_robot_yes_no", wait_for_end_of=True)
                self.first_time_giving_audio_instructions = False
            
            ##### AUDIO: Listen "YES" OR "NO"
            ##### "Please say yes or no to confirm the order"
            confirmation = self.robot.get_audio(yes_or_no=True, question="clean_the_table/question_detect_"+curr_obj.lower()+"_place_tray", face_hearing="charmie_face_green_yes_no", wait_for_end_of=True)
            print("Finished:", confirmation)

        return confirmation 
