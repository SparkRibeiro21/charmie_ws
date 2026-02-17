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
    "charmie_arm":                  False, # True
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          False, # True
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False, # True
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_neck":                 True,
    "charmie_radar":                False, # True
    "charmie_sound_classification": True,
    "charmie_speakers":             True,
    "charmie_tracking":             True,
    "charmie_yolo_objects":         True,
    "charmie_yolo_pose":            True,
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
        self.TASK_NAME = "HRI Challenge"

        # Task States
        self.task_states ={
            "Waiting_for_task_start":          0,
            
            "Wait_for_guest1_to_arrive":       1,
            "Move_to_entrance_door_guest1":    2,
            "Open_door_guest1":                3,
            "Receive_guest1":                  4,
            "Move_guest1_to_sitting_area":     5,
            "Offer_guest1_free_seat":          6,
            "Move_to_initial_position":        7,
            
            "Wait_for_guest2_to_arrive":       8,
            "Move_to_entrance_door_guest2":    9,
            "Open_door_guest2":                10,
            "Receive_guest2":                  11,
            "Get_guest2_bag":                  12,
            "Move_guest2_to_sitting_area":     13,
            "Introduce_the_guests":            14,
            "Offer_guest2_free_seat":          15,

            "Move_to_start_follow_position":   16,
            "Follow_host_to_bag_drop":         17,
            "Final_State":                     18,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Which furniture will guests and host be sitting at, and how many seats these have
        self.SITTING_FURNITURE = {
            "Couch":               2,
            "Left Lounge Chair":   1,
            "Right Lounge Chair":  1,        
        }

        # Which objects should be acquired
        self.OPEN_DOOR_GUEST1 = True
        self.OPEN_DOOR_GUEST2 = True
        self.HANDOVER_GUEST2_BAG = True
        
        # Initial Position
        self.initial_position = [2.0, 4.0, 45.0]
        # print(self.initial_position)
        
        self.start_follow_position = self.initial_position
        self.start_follow_position = [2.0, 4.0, 90.0] # position to start following host after introducing guests
        # print(self.start_follow_position)
        
    def main(self):

        self.configurables() # set all the configuration variables

        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        ### EXTRAIR JA AQUI AS DIFERENTES POSICOES DAS SITTING_FURNITURE > 1
        # Extract the positions of the sitting furniture with more than 1 seat (top-left and bot-right)
        # Calculate Left center and Right center
        # CL = [TLx+BRx/2, TLy]
        # CR = [TLx+BRx/2, BRy]
        # calculate person loaction dist to each center point
        # closer value is side where person is sitting
                
        # For choosing where we sit guest
        # Should consider all person locations and choose the furniture with the furthest, closest person  

        ### TODO: FILL IN THE REST OF THE TASK BASED ON THE COMMENTS BELOW
        # All navs:
        #     self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
        #     self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
        #     self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)

        #     self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl"))), wait_for_end_of=True)

        #     self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
        #     self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object("bowl")), wait_for_end_of=False)

        # All wait for guest to arrive:
        #     # WAIT FOR THE DOORBELL SOUND USING: CONTINUOUS SOUND CLASSIFICATION WITH WFEO = TRUE
        #     s, m, label, score = self.robot.wait_for_doorbell(timeout=20, score_threshold=0.1)
        #     print("FINISHED WAITING FOR DOORBELL")
        #     if s: # doorbell detected
        #         print("DOORBELL DETECTED!")
        #     else: # timeout or error
        #         print("TIMEOUT OR ERROR WAITING FOR DOORBELL!")

        # All open door:
        #     if self.OPEN_DOOR_GUEST1:
        #         self.robot.open_door(push_pull="pull", left_right="left", wait_for_end_of=True)
        #         # need to add saefty and timeouts

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
                
                self.robot.set_face("charmie_face", wait_for_end_of=False)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

                self.robot.set_speech(filename="hri/start_hri_task", wait_for_end_of=True)

                self.robot.wait_for_start_button()
                
                self.state = self.task_states["Wait_for_guest1_to_arrive"]
                

            elif self.state == self.task_states["Wait_for_guest1_to_arrive"]:
                                        
                pass
                # your code here ... 

                self.state = self.task_states["Move_to_entrance_door_guest1"]


            elif self.state == self.task_states["Move_to_entrance_door_guest1"]:
                                        
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                # your code here ...

                self.state = self.task_states["Open_door_guest1"]


            elif self.state == self.task_states["Open_door_guest1"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Receive_guest1"]


            elif self.state == self.task_states["Receive_guest1"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Move_guest1_to_sitting_area"]


            elif self.state == self.task_states["Move_guest1_to_sitting_area"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Offer_guest1_free_seat"]


            elif self.state == self.task_states["Offer_guest1_free_seat"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Move_to_initial_position"]


            elif self.state == self.task_states["Move_to_initial_position"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Wait_for_guest2_to_arrive"]


            elif self.state == self.task_states["Wait_for_guest2_to_arrive"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Move_to_entrance_door_guest2"]


            elif self.state == self.task_states["Move_to_entrance_door_guest2"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Open_door_guest2"]


            elif self.state == self.task_states["Open_door_guest2"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Receive_guest2"]


            elif self.state == self.task_states["Receive_guest2"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Get_guest2_bag"]


            elif self.state == self.task_states["Get_guest2_bag"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Move_guest2_to_sitting_area"]


            elif self.state == self.task_states["Move_guest2_to_sitting_area"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Introduce_the_guests"]


            elif self.state == self.task_states["Introduce_the_guests"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Offer_guest2_free_seat"]


            elif self.state == self.task_states["Offer_guest2_free_seat"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Move_to_start_follow_position"]


            elif self.state == self.task_states["Move_to_start_follow_position"]:
                                        
                pass
                # your code here ...

                self.state = self.task_states["Follow_host_to_bag_drop"]


            elif self.state == self.task_states["Follow_host_to_bag_drop"]:

                pass

                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:
                
                ### self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=False)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="hri/finish_hri_task", wait_for_end_of=False)

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
