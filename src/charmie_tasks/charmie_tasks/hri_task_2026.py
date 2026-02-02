#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedPerson, ListOfDetectedPerson
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  False, # True (to pick/place bag)
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          False, # True (bag handover)
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False, # True (check name and fav. drink)
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_neck":                 True,
    "charmie_radar":                True, 
    "charmie_sound_classification": True,
    "charmie_speakers":             True,
    "charmie_tracking":             False, # True (dynamic neck following)
    "charmie_yolo_objects":         False,
    "charmie_yolo_pose":            True,
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
            
            "Wait_for_guest1_to_arrive":                    1,
            "Move_to_entrance_door_guest1":                 2,
            "Open_door_guest1":                             3,
            "Receive_guest1":                               4,
            "Move_guest1_to_sitting_area":                  5,
            "Offer_guest1_free_seat":                       6,
            "Move_to_initial_position":                     7,
            
            "Wait_for_guest2_to_arrive":                    8,
            "Move_to_entrance_door_guest2":                 9,
            "Open_door_guest2":                             10,
            "Receive_guest2":                               11,
            "Get_guest2_bag":                               12,
            "Move_guest2_to_sitting_area":                  13,
            "Introduce_guests_and_offer_guest2_free_seat":  14,

            "Move_to_start_follow_position":                15,
            "Follow_host_to_bag_drop":                      16,
            "Final_State":                                  17,
        }

    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Which furniture will guests and host be sitting at, and how many seats these have
        self.SITTING_FURNITURE = {
            "Couch":               2,
            "Left Lounge Chair":   1,
            "Right Lounge Chair":  1,        
        }

        self.ENTRANCE_DOOR_FURNITURE = "exit"
        self.SITTING_AREA_ROOM = "living_room"
        self.SIDE_TO_LOOK = "right" # side where guest2 must stand next to the robot when introducing the guests ("right" or "left")

        # Which objects should be acquired
        self.OPEN_DOOR_GUEST1 = False
        self.OPEN_DOOR_GUEST2 = False
        self.HANDOVER_GUEST2_BAG = False
        
        # Initial Position
        self.initial_position = [2.0, 4.0, 45.0]
        # print(self.initial_position)
        
        # self.start_follow_position = self.initial_position
        self.start_follow_position = [2.0, 4.0, 90.0] # position to start following host after introducing guests
        # print(self.start_follow_position)

        self.guest_communication_position = self.robot.get_navigation_coords_from_furniture("couch")
        # self.guest_communication_position = [2.0, 2.5, 0.0] # position to communicate with guests at sitting area
        # print(self.guest_communication_position)
        
    def main(self):

        self.configurables() # set all the configuration variables

        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        self.GUEST1 = DetectedPerson()
        self.GUEST1_NAME = ""
        self.GUEST1_DRINK = ""

        self.GUEST2_NAME = ""
        self.GUEST2_DRINK = ""
        
        ### EXTRAIR JA AQUI AS DIFERENTES POSICOES DAS SITTING_FURNITURE > 1
        # Extract the positions of the sitting furniture with more than 1 seat (top-left and bot-right)
        # Calculate Left center and Right center
        # CL = [TLx+BRx/2, TLy]
        # CR = [TLx+BRx/2, BRy]
        # calculate person loaction dist to each center point
        # closer value is side where person is sitting
                
        # For choosing where we sit guest
        # Should consider all person locations and choose the furniture with the furthest, closest person  

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
        self.search_tetas = [[-60, -30], [0, -30], [60, -30]]

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
                                        
                s, m, label, score = self.robot.wait_for_doorbell(timeout=20, score_threshold=0.1)
                print(s, m, label, score)
                
                self.state = self.task_states["Move_to_entrance_door_guest1"]


            elif self.state == self.task_states["Move_to_entrance_door_guest1"]:
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.ENTRANCE_DOOR_FURNITURE, wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.ENTRANCE_DOOR_FURNITURE), wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/"+self.ENTRANCE_DOOR_FURNITURE, wait_for_end_of=True)
                
                self.state = self.task_states["Open_door_guest1"]


            elif self.state == self.task_states["Open_door_guest1"]:
                                        
                if self.OPEN_DOOR_GUEST1:
                    self.robot.open_door(push_pull="push", left_right="left", wait_for_end_of=True)

                self.state = self.task_states["Receive_guest1"]


            elif self.state == self.task_states["Receive_guest1"]:

                time.sleep(1.0) # wait time for robot to stop and do an audio calibration
                self.robot.calibrate_audio(wait_for_end_of=True)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                time.sleep(5.0) # time for person to be in front of robot

                people_found = ListOfDetectedPerson()
                while len(people_found.persons) == 0:
                    # still need to check for timeout, and decide what to do in that case
                    people_found = self.robot.search_for_person(tetas=[self.look_forward], time_in_each_frame=10.0, break_if_detect=True, characteristics=True, only_detect_person_right_in_front=True)

                print("People found:", len(people_found.persons))

                self.GUEST1 = people_found.persons[0]
                s, m = self.robot.add_face_to_face_recognition(person=self.GUEST1, name="guest1")
                print("Added to face recognition:", s, m)

                self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                command = self.robot.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
                print("Finished:", command)
                
                if command == "ERR_MAX":
                    print("MAX HEARING ATTEMPTS REACHED")
                    self.robot.set_speech(filename="generic/could_not_hear_max_attempts", wait_for_end_of=True)
                else:
                    keyword_list= command.split(" ")
                    self.GUEST1_NAME = keyword_list[0] 
                    self.GUEST1_DRINK = keyword_list[1]
                    print(self.GUEST1_NAME, self.GUEST1_DRINK)

                    self.robot.set_speech(filename="demonstration/nice_to_meet_you", wait_for_end_of=True)

                self.state = self.task_states["Move_guest1_to_sitting_area"]


            elif self.state == self.task_states["Move_guest1_to_sitting_area"]:
                                        
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="hri/sitting_area", wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.guest_communication_position, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="hri/sitting_area", wait_for_end_of=False)
                
                self.state = self.task_states["Offer_guest1_free_seat"]


            elif self.state == self.task_states["Offer_guest1_free_seat"]:

                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)      
                

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)

                # self.robot.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                # self.robot.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                people_found = self.robot.search_for_person(tetas=self.search_tetas, time_in_each_frame=3.0)

                ### CHECK SITTING LOCATION FROM PEOPLE FOUND
                ### SAY WHERE GUEST SHOULD SIT
                
                self.state = self.task_states["Move_to_initial_position"]


            elif self.state == self.task_states["Move_to_initial_position"]:
                                    
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="generic/initial_position", wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.initial_position, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="generic/initial_position", wait_for_end_of=False)
                
                self.state = self.task_states["Wait_for_guest2_to_arrive"]


            elif self.state == self.task_states["Wait_for_guest2_to_arrive"]:
                                        
                s, m, label, score = self.robot.wait_for_doorbell(timeout=20, score_threshold=0.1)
                print(s, m, label, score)
                
                self.state = self.task_states["Move_to_entrance_door_guest2"]


            elif self.state == self.task_states["Move_to_entrance_door_guest2"]:
                                    
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.ENTRANCE_DOOR_FURNITURE, wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.ENTRANCE_DOOR_FURNITURE), wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/"+self.ENTRANCE_DOOR_FURNITURE, wait_for_end_of=True)
                
                self.state = self.task_states["Open_door_guest2"]


            elif self.state == self.task_states["Open_door_guest2"]:
                                        
                if self.OPEN_DOOR_GUEST2:
                    self.robot.open_door(push_pull="push", left_right="left", wait_for_end_of=True)

                self.state = self.task_states["Receive_guest2"]


            elif self.state == self.task_states["Receive_guest2"]:
                
                time.sleep(1.0) # wait time for robot to stop and do an audio calibration
                self.robot.calibrate_audio(wait_for_end_of=True)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                time.sleep(5.0) # time for person to be in front of robot

                people_found = ListOfDetectedPerson()
                while len(people_found.persons) == 0:
                    # still need to check for timeout, and decide what to do in that case
                    people_found = self.robot.search_for_person(tetas=[self.look_forward], time_in_each_frame=10.0, break_if_detect=True, characteristics=True, only_detect_person_right_in_front=True)

                print("People found:", len(people_found.persons))

                self.robot.set_speech(filename="generic/presentation_green_face_quick", wait_for_end_of=True)
                command = self.robot.get_audio(receptionist=True, question="receptionist/receptionist_question", face_hearing="charmie_face_green_receptionist", wait_for_end_of=True)
                print("Finished:", command)
                
                if command == "ERR_MAX":
                    print("MAX HEARING ATTEMPTS REACHED")
                    self.robot.set_speech(filename="generic/could_not_hear_max_attempts", wait_for_end_of=True)
                else:
                    keyword_list= command.split(" ")
                    self.GUEST2_NAME = keyword_list[0] 
                    self.GUEST2_DRINK = keyword_list[1]
                    print(self.GUEST2_NAME, self.GUEST2_DRINK)

                    self.robot.set_speech(filename="demonstration/nice_to_meet_you", wait_for_end_of=True)

                self.robot.get_detected_person_characteristics(detected_person=self.GUEST1, first_sentence="demonstration/demo_characteristics_first_sentence", \
                                                                       ethnicity=True, age=True, gender =True, height=True, shirt_color=True, pants_color=True)
                        
                self.state = self.task_states["Get_guest2_bag"]


            elif self.state == self.task_states["Get_guest2_bag"]:

                if self.HANDOVER_GUEST2_BAG:
                    pass
                else:
                    pass

                self.state = self.task_states["Move_guest2_to_sitting_area"]


            elif self.state == self.task_states["Move_guest2_to_sitting_area"]:
                                    
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="hri/sitting_area", wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.guest_communication_position, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="hri/sitting_area", wait_for_end_of=False)
                
                self.state = self.task_states["Introduce_guests_and_offer_guest2_free_seat"]


            elif self.state == self.task_states["Introduce_guests_and_offer_guest2_free_seat"]:
                

                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)      
                

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)

                self.robot.set_speech(filename="receptionist/dear_guest", wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                people_found = self.robot.search_for_person(tetas=self.search_tetas, time_in_each_frame=3.0)

                ### CHECK SITTING LOCATION FROM PEOPLE FOUND

                ### FIND GUEST 1
                # pred, pred_perc = self.robot.recognize_face_from_face_recognition(person=person)
                # print("COMPARE OUTCOME:", pred, round(pred_perc, 2))
                ### LOOK AT GUEST 1

                self.robot.set_speech(filename="receptionist/second_guest_name_is", wait_for_end_of=True)
                self.robot.set_speech(filename="person_names/"+self.GUEST2_NAME.replace(" ","_").lower(), wait_for_end_of=True)
                
                self.robot.set_speech(filename="receptionist/favourite_drink_is", wait_for_end_of=True)
                self.robot.set_speech(filename="objects_names/"+self.GUEST2_DRINK.lower(), wait_for_end_of=True)

                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)

                self.robot.set_speech(filename="receptionist/first_guest_name_is", wait_for_end_of=True)
                self.robot.set_speech(filename="person_names/"+self.GUEST1_NAME.replace(" ","_").lower(), wait_for_end_of=True)
                
                self.robot.set_speech(filename="receptionist/favourite_drink_is", wait_for_end_of=True)
                self.robot.set_speech(filename="objects_names/"+self.GUEST1_DRINK.lower(), wait_for_end_of=True)

                ### SAY WHERE GUEST SHOULD SIT

                self.state = self.task_states["Move_to_start_follow_position"]


            elif self.state == self.task_states["Move_to_start_follow_position"]:
                                        
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="rooms/"+self.SITTING_AREA_ROOM, wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.guest_communication_position, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="rooms/"+self.SITTING_AREA_ROOM, wait_for_end_of=False)
                
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
