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
    "charmie_arm":              True,
    "charmie_audio":            True,
    "charmie_face":             True,
    "charmie_gamepad":          False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_base_camera":      False,
    "charmie_lidar":            True,
    "charmie_lidar_bottom":     True,
    "charmie_llm":              False,
    "charmie_localisation":     True,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_nav2":             True,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
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
        self.home_furniture = "Exit"        
        self.initial_position = self.robot.get_navigation_coords_from_furniture(self.home_furniture.replace(" ","_").lower())
        print(self.initial_position)

        self.GET_HEAR = False

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

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.wait_for_start_button()
                
                self.robot.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")

                self.state = self.task_states["Select_object_to_pick"]
            
            if self.state == self.task_states["Select_object_to_pick"]:

                if self.GET_HEAR:

                    # selected_category = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_category", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_category)


                    selected_option = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_object", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    print(selected_option)

                    self.object_name = selected_option

                    # selected_room = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_room", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    # print(selected_room)

                    selected_furniture = self.robot.get_audio(gpsr=True, question="face_touchscreen_menu/menu_furniture", max_attempts=3, face_hearing = "charmie_face_green", wait_for_end_of=True)
                    print(selected_furniture)

                    self.place_furniture = selected_furniture


                else:

                    selected_category = self.robot.set_face_touchscreen_menu(["object classes"], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_category")
                    print(selected_category[0])

                    while True:
                        selected_category = self.robot.set_face_touchscreen_menu(["object classes"], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_category")
                        print(selected_category[0])
                        if selected_category[0] != "TIMEOUT" and self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name)) != "NONE": #THINK ABOUT REPEAT LIMIT
                            break

                    selected_option = self.robot.set_face_touchscreen_menu([selected_category[0]], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_object")
                    print(selected_option[0])

                    while selected_option[0] == "TIMEOUT": #THINK ABOUT REPEAT LIMIT
                        selected_option = self.robot.set_face_touchscreen_menu([selected_category[0]], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_object")
                        print(selected_option[0])

                    self.object_name = selected_option[0]

                    selected_room = self.robot.set_face_touchscreen_menu(["rooms"], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_room")
                    print(selected_room[0])

                    while selected_room[0] == "TIMEOUT": #THINK ABOUT REPEAT LIMIT
                        selected_room = self.robot.set_face_touchscreen_menu(["rooms"], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_room")
                        print(selected_room[0])

                    selected_furniture = self.robot.set_face_touchscreen_menu([selected_room[0]], timeout=10, mode="single", speak_results=True, start_speak_file = "face_touchscreen_menu/menu_furniture")
                    print(selected_furniture[0])

                    self.place_furniture = selected_furniture[0]

                    if selected_furniture[0] == "TIMEOUT":
                        self.place_furniture = "Office Table"


                #self.object_name = "Pringles"

                self.object_mode = self.robot.get_standard_pick_from_object(self.object_name)

                # All neck positions
                if self.robot.get_look_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "horizontal":
                    if self.object_mode == "top":
                        self.tetas = [[-90, -40], [-45, -45], [0, -40]]
                    if self.object_mode == "front":
                        self.tetas = [[-40, -45], [0, -45], [40, -45]]

                elif self.robot.get_look_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "vertical":
                    self.tetas = [[0, 15], [0, 0], [0, -35]]

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
                    rotate_coordinates = self.robot.add_rotation_to_pick_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))))
                    self.robot.move_to_position(move_coords=rotate_coordinates, wait_for_end_of=True)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name)), wait_for_end_of=False)


                self.state = self.task_states["Pick_Object"]


            elif self.state == self.task_states["Pick_Object"]:

                self.robot.pick_obj(selected_object=self.object_name, mode=self.object_mode, first_tetas=self.tetas)
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.state = self.task_states["Move_to_place"]


            elif self.state == self.task_states["Move_to_place"]:

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/" + self.place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                # Return to initial position
                if self.object_mode == "front":
                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(self.place_furniture.replace(" ","_").lower()), wait_for_end_of=True)

                if self.object_mode == "top":
                    rotate_coordinates_furniture = self.robot.add_rotation_to_pick_position(move_coords=(self.robot.get_navigation_coords_from_furniture(self.place_furniture.replace(" ","_").lower())))
                    self.robot.move_to_position(move_coords=rotate_coordinates_furniture, wait_for_end_of=True)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/" + self.place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                # next state
                self.state = self.task_states["Place_object"]

                #while True:
                #    pass

            elif self.state == self.task_states["Place_object"]:

                self.furniture_z = self.robot.get_height_from_furniture(self.place_furniture)
                self.object_z = self.robot.get_object_height_from_object(self.object_name)

                if self.object_mode == "front":  
                    #final_z = (1.125 - self.furniture_z - (self.object_z/1.5)) * 1000
                    final_z = -(1.00 - self.furniture_z - (self.object_z/1.5)) * 1000

                    if final_z<-450:
                        final_z=-450

                    self.safe_place_final = [final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
                    self.safe_rise_gripper = [-final_z , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]

                    self.robot.set_arm(command="initial_pose_to_place_front", wait_for_end_of=True)

                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)

                    self.robot.adjust_omnidirectional_position(dx=0.3,dy=0.0)

                    # final_objects = self.search_for_objects(tetas=[[0, 0]], time_in_each_frame=0.5, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.object_name], use_arm=False, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                    # for obj in final_objects:
                    #     if obj.position_cam.y < 0.1 or obj.position_cam.y > 0.1:
                    #         self.robot.adjust_omnidirectional_position(dx=0.0,dy=0.2)

                    time.sleep(0.5)
                    self.robot.set_arm(command="slow_open_gripper", wait_for_end_of=True)
                    time.sleep(0.5)

                    self.robot.adjust_omnidirectional_position(dx=-0.3,dy=0.0)

                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)

                    self.robot.set_arm(command="place_front_to_initial_pose", wait_for_end_of=True)

                if self.object_mode == "top":
                    self.arm_initial_position = [-225, 83, -65, -1, 75, 270]
                    self.arm_safe_first = [ -177.2, 72.8, -112.8, -47.3, 105.7, 258.5]
                    self.arm_safe_second = [-150, 30.5, -117.1, -93.8, 120.6, 48.2]
                    final_x = (1.075 - self.furniture_z - (self.object_z/1.5)) * 1000  
                    self.safe_place_final = [0.0 , 0.0 , final_x , 0.0 , 0.0 , 0.0]
                    self.safe_rise_gripper = [0.0 , 0.0 , -final_x , 0.0 , 0.0 , 0.0]

                    self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
                    self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                    self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)

                    self.robot.adjust_omnidirectional_position(dx=0.15,dy=-0.15)

                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_place_final, wait_for_end_of=True)

                    time.sleep(0.5)
                    self.robot.set_arm(command="slow_open_gripper", wait_for_end_of=True)
                    time.sleep(0.5)

                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = self.safe_rise_gripper, wait_for_end_of=True)

                    self.robot.adjust_omnidirectional_position(dx=-0.15,dy=0.15)

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
