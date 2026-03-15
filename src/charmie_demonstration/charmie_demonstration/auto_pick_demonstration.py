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
    "charmie_arm":                  True,
    "charmie_audio":                False,
    "charmie_face":                 True,
    "charmie_gamepad":              False,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           False,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
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
        self.TASK_NAME = "Auto_pick_object"

        # Task States
        self.task_states ={
            "Waiting_for_task_start": 0,  
            #"Hear_Object":           0,
            "Select_object_to_pick":  1,
            "Move_to_Location":       2,
            "Pick_Object":            3,
            "Move_to_place":          4,
            "Place_object":           5,
            "Move_to_home":           6,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 


        #self.place_furniture = "Office Table"
        self.home_furniture = "Pantry"        
        self.initial_position = self.robot.get_navigation_coords_from_furniture(self.home_furniture.replace(" ","_").lower())
        print(self.initial_position)

        self.GET_HEAR = True

        #Furniture which we cannot place with place_front

        self.unreachable_furniture_front = ["Right Lounge Chair", "Left Lounge Chair", "Reading Chair", "Bed", "TV Table", "Workbench", "Tool Cabinet", "Drill Press"]
        self.unreachable_room_front = ["Hallway"]

        #Furniture which we cannot place with place_top

        self.unreachable_furniture_top = ["Right Lounge Chair", "Left Lounge Chair", "Shelf", "Pantry", "Kitchen Counter", "Kitchen Cabinet", "Reading Chair", "Bed", "TV Table", "Workbench", "Tool Cabinet", "Drill Press"]
        self.unreachable_room_top = ["Hallway"]

        # self.initial_position = [2.8, -4.80, 90.0] # temp (near CHARMIE desk for testing)
    

    # main state-machine function
    def main(self):

        self.configurables() # set all the configuration variables

        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo


        self.state = self.task_states["Waiting_for_task_start"]

        print("IN " + self.TASK_NAME.upper() + " MAIN")
        if self.DEMO_MODE:
            print("DEMO MODE:", self.DEMO_MODE)

        while True:
            self.robot.set_current_task_state_id(current_state=self.state) # Necessary to visualize current task state in GUI
            
            #     self.robot.get_audio(restaurant=True, face_hearing="charmie_face_green_no_mouth", wait_for_end_of=True)

            if self.state == self.task_states["Waiting_for_task_start"]:
        
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                #self.robot.set_torso_position(legs=0.10, torso=10, wait_for_end_of=True) 
                #self.robot.wait_until_camera_stable(timeout=120, check_interval=0.3, stable_duration=0.3, get_gripper=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_initial_position(self.initial_position)

                self.robot.wait_for_start_button()
                
                self.robot.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")

                self.state = self.task_states["Select_object_to_pick"]
            
            if self.state == self.task_states["Select_object_to_pick"]:

                if self.GET_HEAR:

                    # selected_category = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_category", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_category)


                    # selected_option = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_object", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_option)

                    # self.object_name = selected_option

                    self.object_name = "Plate"

                    # selected_room = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_room", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_room)

                    # selected_furniture = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_furniture", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_furniture)

                    self.place_furniture = "Dinner Table"

                    self.object_mode = self.robot.get_standard_pick_from_object(self.object_name)

                    self.selected_height = self.robot.get_height_from_furniture(self.place_furniture)[0]



                else:

                    while True:
                        selected_category = self.robot.set_face_touchscreen_menu(["object classes"], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_category", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                        print(selected_category[0])
                        if selected_category[0] != "TIMEOUT" and self.robot.get_furniture_from_object_class(selected_category[0]) != "NONE": #THINK ABOUT REPEAT LIMIT
                            break

                    selected_option = self.robot.set_face_touchscreen_menu([selected_category[0]], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_object", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                    print(selected_option[0])

                    while selected_option[0] == "TIMEOUT": #THINK ABOUT REPEAT LIMIT
                        selected_option = self.robot.set_face_touchscreen_menu([selected_category[0]], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_object", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                        print(selected_option[0])

                    self.object_name = selected_option[0]

                    self.object_mode = self.robot.get_standard_pick_from_object(self.object_name)

                    rooms = []
                    for obj in self.robot.node.rooms:
                        if self.object_mode == "top":
                            if obj["name"] not in self.unreachable_room_top:
                                rooms.append(obj["name"])
                        elif self.object_mode == "front":
                            if obj["name"] not in self.unreachable_room_front:
                                rooms.append(obj["name"])
                    # print("ROOMS", rooms)

                    selected_room = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=rooms, timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_room", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                    print(selected_room[0])
                    self.robot.set_speech(filename="rooms/"+selected_room[0].replace(" ","_").lower())

                    while selected_room[0] == "TIMEOUT": #THINK ABOUT REPEAT LIMIT
                        selected_room = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=rooms, timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_room", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                        print(selected_room[0])
                        self.robot.set_speech(filename="rooms/"+selected_room[0].replace(" ","_").lower())

                    furniture = []
                    for obj in self.robot.node.furniture:
                        if self.object_mode == "top":
                            if obj["name"] not in self.unreachable_furniture_top and obj["room"] in selected_room[0]:
                                furniture.append(obj["name"])
                        elif self.object_mode == "front":
                            if obj["name"] not in self.unreachable_furniture_front and obj["room"] in selected_room[0]:
                                furniture.append(obj["name"])
                    # print("ROOMS", furniture)

                    selected_furniture = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=furniture, timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_furniture")
                    print(selected_furniture[0])
                    self.robot.set_speech(filename="furniture/"+selected_furniture[0].replace(" ","_").lower())

                    self.place_furniture = selected_furniture[0]

                    if selected_furniture[0] == "TIMEOUT":
                        self.place_furniture = selected_furniture[0] = "Office Table"

                    furniture_height = self.robot.get_height_from_furniture(self.place_furniture)

                    max_limit = 1.75
                    min_limit = 0.30

                    reachable_shelfs = []

                    for h in furniture_height:
                        if min_limit <= h <= max_limit:
                            reachable_shelfs.append(h)
                            print("REACHABLE SHELFS VALUES:", reachable_shelfs)

                    if len(reachable_shelfs)>1:
                        shelf_options = []
                        for i in range(len(reachable_shelfs)):
                            if i == len(reachable_shelfs) - 1:
                                shelf_options.append(f"Top Shelf {i+1}")
                            elif i == 0:
                                shelf_options.append(f"Bottom Shelf {i+1}")
                            else:
                                shelf_options.append(f"Shelf {i+1}")

                        self.selected_shelf = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=shelf_options, timeout=10, mode="single", speak_results=False, start_speak_file="face_touchscreen_menu/menu_shelf", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                        print("Selected shelf:", self.selected_shelf)
                        self.robot.set_speech(filename="face_touchscreen_menu/shelf_register")


                        while self.selected_shelf[0] == "TIMEOUT": #THINK ABOUT REPEAT LIMIT
                            self.selected_shelf = self.robot.set_face_touchscreen_menu(choice_category=["custom"], custom_options=shelf_options, timeout=10, mode="single", speak_results=False, start_speak_file="face_touchscreen_menu/menu_shelf", end_speak_file_error = "sound_effects/you_have_to_pick_renata")
                            print("Selected shelf:", self.selected_shelf)
                            self.robot.set_speech(filename="face_touchscreen_menu/shelf_register")

                        shelf_index = shelf_options.index(self.selected_shelf[0])
                        print("Selected index:", shelf_index)
                        self.selected_height = list(reversed(reachable_shelfs))[shelf_index]
                        print("Selected height:", self.selected_height)

                    elif len(reachable_shelfs)==1:
                        self.selected_height = reachable_shelfs[0]

                    else:
                        print("Can't place an object in that furniture")


                #self.object_name = "Pringles"

                # All neck positions
                if self.robot.get_look_orientation_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "horizontal":
                    self.tetas = [[0, -45], [-40, -45], [40, -45]]

                elif self.robot.get_look_orientation_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "vertical":
                    self.tetas = [[0, 0], [0, 15], [0, -35]]

                self.state = self.task_states["Move_to_Location"]



            if self.state == self.task_states["Move_to_Location"]:

                # After the robot has heard what is the object, the next start button press will start the task, enabling CHARMIE to move to location
                print("START ROUTINE NEXT START")

                self.robot.set_speech(filename="generic/careful", wait_for_end_of=True)                
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name)), wait_for_end_of=False)

                #As of now, we are going to make CHARMIE move to a location 
                if self.object_mode == "front":
                    self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))), wait_for_end_of=True)
               
                if self.object_mode == "top":
                    #rotate_coordinates = self.robot.add_rotation_to_pick_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))))
                    #self.robot.move_to_position(move_coords=rotate_coordinates, wait_for_end_of=True)
                    self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))), wait_for_end_of=True)


                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name)), wait_for_end_of=False)


                self.state = self.task_states["Pick_Object"]


            elif self.state == self.task_states["Pick_Object"]:

                if self.object_name == "Bowl":
                    picked_height, asked_help = self.robot.pick_object_risky(selected_object=self.object_name, pick_mode=self.object_mode, first_search_tetas=self.tetas, return_arm_to_initial_position=False)
                else:
                    picked_height, asked_help = self.robot.pick_object_risky(selected_object=self.object_name, pick_mode=self.object_mode, first_search_tetas=self.tetas)
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.state = self.task_states["Move_to_place"]


            elif self.state == self.task_states["Move_to_place"]:

                #CHECK ONCE AGAIN IF OBJ IS IN GRIPPER

                object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                if not object_in_gripper:

                    pass # Add proper ask for help

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/" + self.place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                # Return to initial position
                if self.object_mode == "front":
                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(self.place_furniture.replace(" ","_").lower()), wait_for_end_of=True)

                if self.object_mode == "top":
                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(self.place_furniture.replace(" ","_").lower()), wait_for_end_of=True)
                    #self.robot.adjust_angle(45)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/" + self.place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                object_in_gripper, m = self.robot.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)
                    
                if not object_in_gripper:

                    pass # Add proper ask for help

                # next state
                self.state = self.task_states["Place_object"]

                #while True:
                #    pass

            elif self.state == self.task_states["Place_object"]:

                #self.furniture_z = self.robot.get_height_from_furniture(self.place_furniture)
                #self.object_z = self.robot.get_object_height_from_object(self.object_name)

                if self.object_mode == "front":

                    _ , _ , furniture_distance = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)

                    print("Furniture Distance: ", furniture_distance)

                    self.robot.set_arm(command="initial_pose_to_place_front", wait_for_end_of=True)
                    #self.robot.wait_for_start_button()

                    gripper_place_position = self.robot.get_gripper_localization()
                    if asked_help:
                        final_z = (gripper_place_position.z - self.selected_height - (self.robot.get_object_height_from_object(self.object_name)/1.25) - 0.02)*1000
                    else:
                        final_z = (gripper_place_position.z - self.selected_height - picked_height - 0.02)*1000
                    print("Final_Z: ", final_z," Current Gripper Height:  ", gripper_place_position.z, " furniture z : ", self.selected_height, " picked height : ", picked_height)
                    
                    if final_z > 450:
                        final_z = 450

                    self.safe_place_final = [-final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
                    self.safe_rise_gripper = [final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]

                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)
                    
                    #top_furniture_points = self.robot.get_top_coords_from_furniture(self.place_furniture)
                    #bottom_furniture_points = self.robot.get_bottom_coords_from_furniture(self.place_furniture)
#
                    #x_min = min(top_furniture_points[0], bottom_furniture_points[0])
                    #x_max = max(top_furniture_points[0], bottom_furniture_points[0])
                    #y_min = min(top_furniture_points[1], bottom_furniture_points[1])
                    #y_max = max(top_furniture_points[1], bottom_furniture_points[1])
#
                    #robot_yaw = self.robot.get_robot_localization()
                    #robot_yaw = math.degrees(robot_yaw.theta)
#
                    #print("ROBOT ANGLE:", robot_yaw)
#
                    #dx = 0.0
                    #dy = 0.0
#
                    #if (-45 <= robot_yaw <= 45) and gripper_place_position.x < x_min:
                    #    dx = x_min - gripper_place_position.x
                    #    print("X MIN: ", x_min)
                    #    print("X GRIPPER:", gripper_place_position.x)
                    #elif (45 < robot_yaw <= 135) and gripper_place_position.y < y_min:
                    #    dx = y_min - gripper_place_position.y
                    #    print("Y MIN: ", y_min)
                    #    print("Y GRIPPER:", gripper_place_position.y)
                    #elif ((-180 <= robot_yaw < -135) or (135 < robot_yaw <= 180)) and gripper_place_position.x > x_max:
                    #    dx = x_max - gripper_place_position.x
                    #    print("X MAX: ", x_max)
                    #    print("X GRIPPER:", gripper_place_position.x)
                    #elif (-135 < robot_yaw < -45) and gripper_place_position.y > y_max:
                    #    dx = y_max - gripper_place_position.y
                    #    print("Y MAX: ", y_max)
                    #    print("Y GRIPPER:", gripper_place_position.y)
#
                    #dx = abs(dx)
                    #print("Adjust Movement:", dx)
#
                    #if furniture_distance >= dx:

                    dx = furniture_distance - 0.15
                    dy = 0.0                     
                    self.robot.adjust_omnidirectional_position(dx=dx,dy=dy, safety=False)
                    print("Moving dx: ", dx, " || Moving dy: ", dy)
                    #self.robot.wait_for_start_button()

                    time.sleep(0.5)
                    self.robot.set_arm(command="slow_open_gripper", wait_for_end_of=True)
                    time.sleep(0.5)
                    self.robot.adjust_omnidirectional_position(dx=-dx,dy=-dy)
                    # self.robot.adjust_omnidirectional_position(dx=-0.3,dy=0.0)
                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)

                    self.robot.set_arm(command="place_front_to_initial_pose", wait_for_end_of=True)

                if self.object_mode == "top":
                    self.arm_safe_start_position = [-215, 83.1, -74.8, 9.1, 65.8, 268.8]
                    self.arm_initial_position = [-225, 83, -65, -1, 75, 270]
                    self.arm_safe_first = [ -177.2, 72.8, -112.8, -47.3, 105.7, 258.5]
                    self.arm_safe_second = [-151.5, 75, -123.2, -72.4, 110.8, 41.7]
                    #final_x = (1.075 - self.furniture_z - (self.object_z/1.5)) * 1000  

                    #self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_start_position, wait_for_end_of=True)
                    if self.object_name == "Plate":
                        _ , _ , furniture_plate_distance = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
                        
                        plate_place_first = [-194.3, 82.8, -86.6, 29.6, 69, 272.4]
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = plate_place_first, wait_for_end_of=True)
                        plate_place_second = [-199.1, 34.3, -49.3, 159.7, 72.4, 180.8]
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = plate_place_second, wait_for_end_of=True)

                    elif self.object_name != "Bowl":
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)

                    

                        #self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                        #self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)

                    top_furniture_points = self.robot.get_top_coords_from_furniture(self.place_furniture)
                    bottom_furniture_points = self.robot.get_bottom_coords_from_furniture(self.place_furniture)

                    gripper_place_position = self.robot.get_gripper_localization()

                    _ , _ , furniture_distance = self.robot.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                    dx = furniture_distance - 0.05
                    if self.object_name == "Plate":
                        dx = furniture_plate_distance - 0.06
                    dy = 0.0
                    

                    #x_min = min(top_furniture_points[0], bottom_furniture_points[0])
                    #x_max = max(top_furniture_points[0], bottom_furniture_points[0])
                    #y_min = min(top_furniture_points[1], bottom_furniture_points[1])
                    #y_max = max(top_furniture_points[1], bottom_furniture_points[1])
#
                    #robot_yaw = self.robot.get_robot_localization()
                    #robot_yaw = math.degrees(robot_yaw.theta)
#
                    #print("ROBOT ANGLE:", robot_yaw)
#
                    #dx = 0
#
                    #if (0 <= robot_yaw <= 90) and gripper_place_position.x < x_min:
                    #    dx = x_min - gripper_place_position.x
                    #    print("X MINIMO: ", x_min)
                    #    print("X FURNITURE:", gripper_place_position.x)
                    #elif (90 < robot_yaw <= 180) and gripper_place_position.y < y_min:
                    #    dx = y_min - gripper_place_position.y
                    #    print("Y MINIMO: ", y_min)
                    #    print("Y FURNITURE:", gripper_place_position.y)
                    #elif (-180 < robot_yaw < -90) and gripper_place_position.x < x_max:
                    #    dx = x_max - gripper_place_position.x
                    #    print("X MAXIMO: ", x_max)
                    #    print("X FURNITURE:", gripper_place_position.x)
                    #elif (-90 <= robot_yaw < 0) and gripper_place_position.y < y_max:
                    #    dx = y_max - gripper_place_position.y
                    #    print("Y MAXIMO: ", y_max)
                    #    print("Y FURNITURE:", gripper_place_position.y)
#
                    ##dx = 0.3

                    #print("Distance x:", dx)
                    #dx = (abs(dx) + 0.05) * math.cos(math.radians(45))
                    #dy = -dx

                    #print("Adjust Movement x:", dx," || y:", dy)

                    self.robot.adjust_omnidirectional_position(dx=dx, dy=dy, safety=False)

                    #MAKE SPECIAL CASE MORE IN LINE WITH REST OF CODE LATER:

                    if self.object_name == "Bowl":
                        self.robot.set_height_furniture_for_arm_manual_movements(self.selected_height) #####

                        self.robot.set_arm(command="place_bowl_table", wait_for_end_of=True)
                        self.robot.set_arm(command="arm_go_rest", wait_for_end_of=True)

                    elif self.object_name == "Plate":
                            final_x = (gripper_place_position.z - self.selected_height - 0.10)*1000
                            rise_x = - final_x - 50 

                            self.safe_place_final = [0.0 , final_x , 20.0 , 0.0 , 0.0 , 0.0]
                            self.safe_rise_gripper = [0.0 , -10.0 , 0.0 , -80.0 , 0.0 , 0.0]  
                            self.safe_pull_gripper = [0.0 , 0.0 , -40.0 , 0.0 , 0.0 , 0.0]  

                            self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=False)

                            time.sleep(0.5)
                            self.robot.set_arm(command="slow_open_gripper", wait_for_end_of=True)
                            time.sleep(0.5)

                            self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)
                            self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_pull_gripper, wait_for_end_of=True)

                            self.robot.adjust_omnidirectional_position(dx=-dx,dy=-dy)
                            self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = plate_place_first, wait_for_end_of=True)
                            self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)  

                    else:
                        final_x = (gripper_place_position.z - self.selected_height - picked_height - 0.02)*1000
                        print("Final_X: ", final_x," Current Gripper Height:  ", gripper_place_position.z, " furniture z : ", self.selected_height, " picked height : ", picked_height)
                        self.safe_place_final = [0.0 , 0.0 , final_x , 0.0 , 0.0 , 0.0]
                        self.safe_rise_gripper = [0.0 , 0.0 , -final_x , 0.0 , 0.0 , 0.0]

                        self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)

                        time.sleep(0.5)
                        self.robot.set_arm(command="slow_open_gripper", wait_for_end_of=True)
                        time.sleep(0.5)

                        self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)

                        self.robot.adjust_omnidirectional_position(dx=-dx,dy=-dy)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                        self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)

                    self.robot.set_arm(command="close_gripper", wait_for_end_of=True)
                    
                    #TEST PLACE  OBJ #final_objects = self.search_for_objects(tetas=[[-45, -30]], time_in_each_frame=0.5, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.object_name], use_arm=False, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                    #for obj in final_objects:

                # next state
                self.state = self.task_states["Move_to_home"]


            elif self.state == self.task_states["Move_to_home"]:

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/" + self.home_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                self.robot.move_to_position(move_coords = self.initial_position, wait_for_end_of=True)

                self.state = self.task_states["Select_object_to_pick"]

                #while True:
                #    pass

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
