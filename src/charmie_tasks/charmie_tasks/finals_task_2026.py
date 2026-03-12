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
    "charmie_arm":                  False,
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
    "charmie_neck":                 True,
    "charmie_radar":                False,
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
        self.TASK_NAME = "Finals"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":       0,
            "Search_for_misplaced_objects": 1,
            "Final_State":                  2,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Initial Position
        #self.initial_position = self.robot.get_navigation_coords_from_furniture("dishwasher")
        self.initial_position = [0.0, 0.0, 0.0]
        # self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        print(self.initial_position)
        
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.FURNITURE_WE_WANT_TO_ANALYSE = ["Office Table","Shelf", "Coffee Table"]
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.search_tetas_horizontal = [[0, -45], [-40, -45], [40, -45]]
        self.search_tetas_vertical = [[0, -15], [0, -35], [0, 15]]

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

                self.robot.set_speech(filename="serve_breakfast/sb_ready_start", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                

                self.state = self.task_states["Search_for_misplaced_objects"]

            elif self.state == self.task_states["Search_for_misplaced_objects"]:

                for current_furniture in self.FURNITURE_WE_WANT_TO_ANALYSE:

                    self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+ current_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                    self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(furniture=current_furniture), wait_for_end_of=True)

                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+ current_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                    objects_in_correct_furniture = []
                    objects_in_wrong_furniture = []
                    ignored_objects = []

                    if self.robot.get_look_orientation_from_furniture(furniture=current_furniture) == "horizontal":
                        search_misplaced_obj_tetas = self.search_tetas_horizontal
                    elif self.robot.get_look_orientation_from_furniture(furniture=current_furniture) == "vertical":
                        search_misplaced_obj_tetas = self.search_tetas_vertical

                    object_detected = self.robot.search_for_objects(tetas=search_misplaced_obj_tetas, detect_objects=True)

                    for obj in object_detected:

                        conf   = f"{obj.confidence * 100:.0f}%"
                        cam_x_ = f"{obj.position_relative.x:5.2f}"
                        cam_y_ = f"{obj.position_relative.y:5.2f}"
                        cam_z_ = f"{obj.position_relative.z:5.2f}"

                        print(f"{'ID:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({cam_x_},{cam_y_},{cam_z_} {obj.furniture_location})")
                        print("INFO FROM JSON:", self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(obj.object_name)))

                        if obj.furniture_location.replace(" ","_").lower() == current_furniture.replace(" ","_").lower():

                            if obj.furniture_location.replace(" ","_").lower() != self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(object_name=obj.object_name)):
                                print("OBJECT IS NOT IN CORRECT FURNITURE")

                                self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                                self.robot.set_speech(filename="clean_the_table/object_in_the_wrong_furniture", wait_for_end_of=True)
                                objects_in_wrong_furniture.append(obj.object_name)
                            else:
                                print("OBJECT IN CORRECT FURNITURE")
                                self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                                self.robot.set_speech(filename="clean_the_table/object_in_the_correct_furniture", wait_for_end_of=True)
                                objects_in_correct_furniture.append(obj.object_name)

                        else:
                            print("I don't care about this object")
                            ignored_objects.append(obj.object_name)


                    print("Objects in correct furniture:", objects_in_correct_furniture)
                    print("Objects in wrong furniture:", objects_in_wrong_furniture)
                    print("Ignored objects:", ignored_objects)
                                        
                
                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
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
