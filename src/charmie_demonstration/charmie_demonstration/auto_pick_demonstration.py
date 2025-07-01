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
    "charmie_audio":            False,
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
            "Final_state":            4,
            "Place_object":           5,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Define what object to grab and how

        self.home_position = "Office Table"        
        self.initial_position = self.robot.get_navigation_coords_from_furniture(self.home_position.replace(" ","_").lower())
        print(self.initial_position)
        self.initial_position = [2.8, -4.80, 90.0] # temp (near CHARMIE desk for testing)

        self.object_mode = "pick_front"

        #ARM DEMONSTRATION PLACE OBJECT
        self.arm_initial_position = [-225, 83, -65, -1, 75, 270]
        self.arm_safe_first = [ -215, -70, -16, 80, 30, 182]
        self.arm_safe_second = [ -181, 29, -103.4, 173.3, 13.9, 96.3]
        self.arm_safe_final = [-201.1, 38, -112.2 , 120.9, 25.1, 146.1]
        

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
            
            # if self.state == self.task_states["Hear_Object"]:

            #     # The task begins by using default face 
            #     self.robot.set_face(custom="charmie_face")
            #     # Hears what object is intended to bring to you
            #     self.robot.get_audio(restaurant=True, face_hearing="charmie_face_green_no_mouth", wait_for_end_of=True)

                    
            #     self.state = self.task_states["Move_to_Location"]

            if self.state == self.task_states["Waiting_for_task_start"]:
        
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                ### self.robot.wait_for_door_start()

                self.robot.wait_for_start_button()
                
                self.robot.set_initial_position(self.initial_position)
                
                print("SET INITIAL POSITION")

                self.state = self.task_states["Select_object_to_pick"]
            
            if self.state == self.task_states["Select_object_to_pick"]:

                selected_option = self.robot.set_face_touchscreen_menu(["toys", "drinks", "fruits"], timeout=10, mode="single", speak_results=True)
                print(selected_option[0])
                self.object_name = selected_option[0]
                #self.object_name = "Pringles"

                # All neck positions
                if self.robot.get_look_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "horizontal":
                    self.tetas = [[-30, -45], [0, -45], [30, -45]]

                elif self.robot.get_look_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))) == "vertical":
                    self.tetas = [[0, 15], [0, 0], [0, -35]]

                self.state = self.task_states["Move_to_Location"]



            if self.state == self.task_states["Move_to_Location"]:

                # After the robot has heard what is the object, the next start button press will start the task, enabling CHARMIE to move to location
                print("START ROUTINE NEXT START")

                self.robot.set_speech(filename="generic/careful", wait_for_end_of=True)                
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name)), wait_for_end_of=False)

                #As of now, we are going to make CHARMIE move to a location 
                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(self.object_name))), wait_for_end_of=True)
    
                self.state = self.task_states["Pick_Object"]


            elif self.state == self.task_states["Pick_Object"]:

                self.robot.pick_obj(selected_object=self.object_name, mode=self.object_mode, first_tetas=self.tetas)

                self.state = self.task_states["Final_state"]


            elif self.state == self.task_states["Final_state"]:

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/" + self.home_position.replace(" ","_").lower(), wait_for_end_of=False)

                # Return to initial position
                self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(self.home_position.replace(" ","_").lower()), wait_for_end_of=True)

                # next state
                self.state = self.task_states["Place_object"]

                #while True:
                #    pass

            elif self.state == self.task_states["Place_object"]:

                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_final, wait_for_end_of=True)
                time.sleep(2)
                self.robot.set_arm(command="open_gripper", wait_for_end_of=True)
                time.sleep(2)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_final, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_second, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_safe_first, wait_for_end_of=True)
                self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = self.arm_initial_position, wait_for_end_of=True)
                self.robot.set_arm(command="close_gripper", wait_for_end_of=True)

                # next state
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
