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
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_base_camera":      False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     True,
    "charmie_yolo_pose":        True,
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
        self.TASK_NAME = "Offer_object"

        # Task States
        self.task_states ={
            "Grab_object":            0,
            "Welcome_start":          1,
            "Locate_person":          2,
            "Offer_book":             3,
            "Final_state":            4,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # All neck positions
        self.initial_neck_position = [20, 0]
        self.search_person_tetas = [[20, 0]]
        self.search_object_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]]
        self.only_detect_person_front = False

        # Define what object to grab and how
        self.object_name = "Pringles"
        self.object_mode = "pick_top"

        # What positions does the robot use to handout the object
        self.initial_to_handout_position = "initial_position_to_ask_for_objects"
        self.handout_to_initial_position = "ask_for_objects_to_initial_position"

        # Should the robot grab object automatically or not
        self.grab_object = False

        # What speech files are used for the task
        self.hello_mp4 = "JEF/WelcomeJEF"
        self.why_object_is_being_handed_mp4 = "JEF/GiftJEF"
        self.explain_countdown_mp4 = "JEF/ReleaseExplanationJEF"
        self.countdown_mp4 = "JEF/CountdownJEF"
        self.object_explanation_mp4 = "JEF/BookExplanationJEF"
        self.goodbye_mp4 = "JEF/GoodbyeJEF"

        # All task timers
        self.sleep_after_countdown = 0.3
        self.sleep_after_release = 2
        self.time_search_person = 4

        self.RESET_CLOSEST = 999999 #Ridiculous number to reset to a safe number

        

    # main state-machine function
    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo


        self.state = self.task_states["Grab_object"]

        print("IN " + self.TASK_NAME.upper() + " MAIN")
        if self.DEMO_MODE:
            print("DEMO MODE:", self.DEMO_MODE)

        while True:
            self.robot.set_current_task_state_id(current_state=self.state) # Necessary to visualize current task state in GUI
            
            if self.state == self.task_states["Grab_object"]:

                # The task begings by the face and neck being reset to default, awaiting the start button to be pressed to start
                self.robot.set_face(custom="charmie_face")
                self.robot.set_neck(position=self.initial_neck_position, wait_for_end_of=True)
                print( " AUTOMATICALLY GRAB OBJECT // MANUALLY OPEN GRIPPER NEXT START" )
                self.robot.wait_for_start_button()

                # If flag grab_object is true, the robot will pick up the object automatically, assuming we are launching the correct dataset. If false, the robot will wait for the user to give him the book after pressing start button twice
                if self.grab_object:

                    # Show what robot is detecting,to make it more intresting
                    self.robot.set_face(camera="hand", show_detections=True)
                    self.robot.pick_obj(selected_object=self.object_name,mode=self.object_mode,first_tetas=self.search_object_tetas)
                    self.robot.set_face(custom="charmie_face")
                    
                else:

                    self.robot.set_arm(command="open_gripper", wait_for_end_of=True)

                    # Will wait for start button to close gripper, assuming someone is holding the object betweenthe gripper's fingers
                    print( "CLOSE GRIPPER NEXT START" )
                    self.robot.wait_for_start_button()
                    self.robot.set_arm(command="close_gripper", wait_for_end_of=True)

                    
                self.state = self.task_states["Welcome_start"]

            elif self.state == self.task_states["Welcome_start"]:

                # After the robot has grabbed the object, the next start button press will start the task, greeting the person  
                print("START ROUTINE NEXT START")
                self.robot.wait_for_start_button()

                # Hello.mp4 to grab the attention of people audibly, hopefully making them look at the robot
                self.robot.set_speech(filename=self.hello_mp4, wait_for_end_of=True)
    
                self.state = self.task_states["Locate_person"]


            elif self.state == self.task_states["Locate_person"]:

                # Show what robot is detecting,tomake it more intresting
                self.robot.set_face(camera="head", show_detections=True)

                # Search for closest person
                people_found = self.robot.search_for_person(tetas=self.search_person_tetas, time_in_each_frame=self.time_search_person, only_detect_person_right_in_front=self.only_detect_person_front)

                # Reset variables
                found = False
                closest = self.RESET_CLOSEST
                prev_closest = self.RESET_CLOSEST - 1

                # Calculate who is closest
                for p in people_found:
                    closest = abs( p.position_relative_head.x ) * abs( p.position_relative_head.x )  + abs( p.position_relative_head.y ) * abs( p.position_relative_head.y )
                    if closest < prev_closest:
                        saved_p = p
                        prev_closest = closest
                        found = True

                # If it found a person who is closest, set neck to look at them, otherwise neck will keep looking foward
                if found:
                    self.robot.set_neck_coords(position=[saved_p.position_relative_head.x, saved_p.position_relative_head.y, saved_p.position_relative_head.z], wait_for_end_of=True)
                else:
                    self.robot.set_neck(position=self.initial_neck_position, wait_for_end_of=True)

                # Talk about what the person is about to be handed
                self.robot.set_speech(filename=self.why_object_is_being_handed_mp4, wait_for_end_of=True)

                self.state = self.task_states["Offer_book"]

            elif self.state == self.task_states["Offer_book"]:
                
                self.robot.set_arm(command=self.initial_to_handout_position, wait_for_end_of=False)

                # Explain there will be a countdown
                self.robot.set_speech(filename=self.explain_countdown_mp4, wait_for_end_of=True) 

                # The actual countdown
                self.robot.set_speech(filename=self.countdown_mp4, wait_for_end_of=True) 

                # Slight delay just so the robot does not drop right as it says 1, or whatever number the countdown counts to
                time.sleep(self.sleep_after_countdown)

                self.robot.set_arm(command="open_gripper", wait_for_end_of=True)

                # Slight delay so the arm does not start immediately retracting upon letting go
                time.sleep(self.sleep_after_release)

                self.state = self.task_states["Final_state"]


            elif self.state == self.task_states["Final_state"]:

                # Return arm to initial position
                self.robot.set_arm(command=self.handout_to_initial_position, wait_for_end_of=False)
                self.robot.set_arm(command="close_gripper", wait_for_end_of=False)

                # Offer a more indepth explanation about the object or instructions on what to do with it if necessary, better to divide an explanation into 2 to keep the entire experience fluid
                self.robot.set_speech(filename=self.object_explanation_mp4, wait_for_end_of=True)

                # Say goodbye to person 
                self.robot.set_speech(filename=self.goodbye_mp4, wait_for_end_of=True)

                # next state
                self.state = self.task_states["Grab_object"]

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
