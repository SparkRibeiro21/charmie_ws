#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedPerson, DetectedObject
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import math

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  True,
    "charmie_audio":                True,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          False, # True (3D obstacles)
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
            "Waiting_for_task_start":                       0,
            
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
        self.SITTING_PLACES_IN_FURNITURE =[
            # {
            #     "name": "couch",  
            #     "furniture": "couch",
            #     "center_coords": self.robot.get_location_coords_from_furniture("couch"),
            #     "speak": "123"
            # },
            ### FOR CASES WHERE COUCH IS HORIZONTAL IN 2D MAP
            {
                "name": "couch_left",  
                "furniture": "couch",
                "center_coords": [(self.robot.get_top_coords_from_furniture("Couch")[0] + self.robot.get_bottom_coords_from_furniture("Couch")[0])/2.0, 
                                  (self.robot.get_top_coords_from_furniture("Couch")[1] + (self.robot.get_top_coords_from_furniture("Couch")[1] + self.robot.get_bottom_coords_from_furniture("Couch")[1])/2.0)/2.0, 
                                  self.robot.get_height_from_furniture("Couch")[0]], # manual values for now
                "speak": "hri/couch_left"
            },
            {
                "name": "couch_right", 
                "furniture": "couch",
                "center_coords": [(self.robot.get_top_coords_from_furniture("Couch")[0] + self.robot.get_bottom_coords_from_furniture("Couch")[0])/2.0, 
                                  (self.robot.get_bottom_coords_from_furniture("Couch")[1] + (self.robot.get_top_coords_from_furniture("Couch")[1] + self.robot.get_bottom_coords_from_furniture("Couch")[1])/2.0)/2.0, 
                                  self.robot.get_height_from_furniture("Couch")[0]], # manual values for now
                "speak": "hri/couch_right"
            },
            ### FOR CASES WHERE COUCH IS VERTICAL IN 2D MAP
            # {
            #     "name": "couch_top",  
            #     "furniture": "couch",
            #     "center_coords": [(self.robot.get_top_coords_from_furniture("Couch")[0] + (self.robot.get_top_coords_from_furniture("Couch")[0] + self.robot.get_bottom_coords_from_furniture("Couch")[0])/2.0)/2.0, 
            #                       (self.robot.get_top_coords_from_furniture("Couch")[1] + self.robot.get_bottom_coords_from_furniture("Couch")[1])/2.0,
            #                       self.robot.get_height_from_furniture("Couch")[0]], # manual values for now
            #     "speak": "123"
            # },
            # {
            #     "name": "couch_bottom", 
            #     "furniture": "couch",
            #     "center_coords": [(self.robot.get_bottom_coords_from_furniture("Couch")[0] + (self.robot.get_top_coords_from_furniture("Couch")[0] + self.robot.get_bottom_coords_from_furniture("Couch")[0])/2.0)/2.0, 
            #                       (self.robot.get_top_coords_from_furniture("Couch")[1] + self.robot.get_bottom_coords_from_furniture("Couch")[1])/2.0,
            #                       self.robot.get_height_from_furniture("Couch")[0]], # manual values for now
            #     "speak": "123"
            # },
            {
                "name": "left_lounge_chair",
                "furniture": "left_lounge_chair",
                "center_coords": self.robot.get_location_coords_from_furniture("left_lounge_chair"),
                "speak": "furniture/left_lounge_chair"
            },
            {
                "name": "right_lounge_chair", 
                "furniture": "right_lounge_chair",
                "center_coords": self.robot.get_location_coords_from_furniture("right_lounge_chair"),
                "speak": "furniture/right_lounge_chair"
            }
        ]

        print("SITTING PLACES:")
        for sf in self.SITTING_PLACES_IN_FURNITURE:
            print(" - ", sf["name"], " in ", sf["furniture"], " seats at ", sf["center_coords"])

        self.ENTRANCE_DOOR_FURNITURE = "exit"
        self.SITTING_AREA_ROOM = "living_room"
        self.SIDE_TO_LOOK = "right" # side where guest2 must stand next to the robot when introducing the guests ("right" or "left")

        # Which objects should be acquired
        self.OPEN_DOOR_GUEST1 = False
        self.OPEN_DOOR_GUEST2 = False
        self.HANDOVER_GUEST2_BAG = False
        
        # Initial Position
        self.initial_position = [2.0, 4.0, -45.0]
        # print(self.initial_position)
        
        # self.start_follow_position = self.initial_position
        self.start_follow_position = [2.0, 4.0, -90.0] # position to start following host after introducing guests
        # print(self.start_follow_position)

        self.guest_communication_position = self.robot.get_navigation_coords_from_furniture("couch")
        # self.guest_communication_position = [2.0, 2.5, 0.0] # position to communicate with guests at sitting area
        # print(self.guest_communication_position)

        self.min_dist_for_sitting_place_to_be_occupied = 0.4 # minimum distance from person to sitting place center coords to consider that place as occupied
        
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

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
        self.search_tetas = [[-60, -30], [0, -30], [60, -30]]

        self.state = self.task_states["Get_guest2_bag"]

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

                self.robot.wait_for_start_button()
                
                time.sleep(1.0) # wait time for robot to stop and do an audio calibration
                self.robot.calibrate_audio(wait_for_end_of=True)
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                time.sleep(5.0) # time for person to be in front of robot

                people_found = []
                while len(people_found) == 0:
                    # still need to check for timeout, and decide what to do in that case
                    people_found = self.robot.search_for_person(tetas=[self.look_forward], time_in_each_frame=10.0, break_if_detect=True, characteristics=True, only_detect_person_right_in_front=True)

                print("People found:", len(people_found))

                #################################################################################### DYNAMICALLY ADJUST NECK !!!!

                self.GUEST1 = people_found[0]
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

                self.robot.set_speech(filename="generic/thank_you", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/guide_to_sitting_area", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/please_follow_me", wait_for_end_of=True)

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

                ### TEMP
                # self.robot.set_initial_position(self.guest_communication_position)
                # print("SET INITIAL POSITION")
                # self.robot.set_face("charmie_face", wait_for_end_of=False)
                # self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                # self.robot.set_speech(filename="hri/start_hri_task", wait_for_end_of=True)
                # self.robot.wait_for_start_button()
                ### TEMP END

                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)
                    self.robot.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)      
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=True)

                # self.robot.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                # self.robot.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                people_found = self.robot.search_for_person(tetas=self.search_tetas)

                temp_min_dist_sitting_places_dict = {
                    place["name"]: 10.0
                    for place in self.SITTING_PLACES_IN_FURNITURE
                }

                # print("temp_min_dist_sitting_places_dict:", temp_min_dist_sitting_places_dict)
                # print("People found for seating:", len(people_found))

                p_considered = []

                for p in people_found:
                    print(p.room_location)
                    if p.room_location.replace(" ","_").lower() == self.SITTING_AREA_ROOM.replace(" ","_").lower():
                        p_considered.append(p)
                        p_coords = [p.position_absolute.x, p.position_absolute.y, p.position_absolute.z]
                        for place in self.SITTING_PLACES_IN_FURNITURE:
                            # print(name, center_coords)
                            name = place["name"]
                            center_coords = place["center_coords"]

                            dist = math.sqrt( (p_coords[0]-center_coords[0])**2 + (p_coords[1]-center_coords[1])**2 )

                            if dist < temp_min_dist_sitting_places_dict[name]:
                                temp_min_dist_sitting_places_dict[name] = dist
                 
                print("temp_min_dist_sitting_places_dict:", temp_min_dist_sitting_places_dict)   
                print("People found for seating:", len(p_considered))    
                
                if all(dist < self.min_dist_for_sitting_place_to_be_occupied for dist in temp_min_dist_sitting_places_dict.values()):
                    # SPECIAL CASE, if all seats are occupided, by default we say the person should sit in the center of the sofa 
                    # Might make sense to chang in the future
                    neck_position = self.robot.get_location_coords_from_furniture("couch")
                    speak_file = "hri/couch_center"

                else:
                    max_name = max(
                    temp_min_dist_sitting_places_dict,
                    key=temp_min_dist_sitting_places_dict.get
                    )
                    max_value = temp_min_dist_sitting_places_dict[max_name]

                    max_place_info = next(
                        place for place in self.SITTING_PLACES_IN_FURNITURE
                        if place["name"] == max_name
                    )
                    neck_position = [max_place_info["center_coords"][0], max_place_info["center_coords"][1], 0.9]
                    speak_file = max_place_info["speak"]

                # print("Closest sitting place:")
                # for k, v in max_place_info.items():
                #     print(f"{k}: {v}")

                self.robot.set_neck_coords(position=neck_position, wait_for_end_of=False)
                self.robot.set_speech(filename="hri/found_free_seat", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/i_am_looking_at", wait_for_end_of=True)
                self.robot.set_speech(filename=speak_file, wait_for_end_of=True)
                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)
                self.robot.set_speech(filename="receptionist/dear_guest", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/please_take_a_seat", wait_for_end_of=True)
                self.robot.set_speech(filename=speak_file, wait_for_end_of=True)
                
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
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                
                people_found = []
                while len(people_found) == 0:
                    # still need to check for timeout, and decide what to do in that case
                    people_found = self.robot.search_for_person(tetas=[self.look_forward], time_in_each_frame=10.0, break_if_detect=True, characteristics=True, only_detect_person_right_in_front=True)

                print("People found:", len(people_found))

                #################################################################################### DYNAMICALLY ADJUST NECK !!!!

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
                    bag_object = DetectedObject()
                    bag_object.object_name = "bag"
                        
                    self.robot.ask_help_pick_object_gripper(object_d=bag_object, show_detection=False, look_judge=self.look_forward, wait_time_show_detection=1.0, wait_time_show_help_face=1.0, attempts_at_receiving=2, bb_color=(0, 255, 0))
                        
                    self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                
                self.robot.set_speech(filename="generic/thank_you", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/guide_to_sitting_area", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/please_follow_me", wait_for_end_of=True)

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

                # self.robot.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                # self.robot.set_speech(filename="receptionist/keep_face_clear", wait_for_end_of=True)

                people_found = self.robot.search_for_person(tetas=self.search_tetas)

                temp_min_dist_sitting_places_dict = {
                    place["name"]: 10.0
                    for place in self.SITTING_PLACES_IN_FURNITURE
                }

                # print("temp_min_dist_sitting_places_dict:", temp_min_dist_sitting_places_dict)
                # print("People found for seating:", len(people_found))

                p_considered = []

                for p in people_found:
                    print(p.room_location)
                    if p.room_location.replace(" ","_").lower() == self.SITTING_AREA_ROOM.replace(" ","_").lower():
                        p_considered.append(p)
                        p_coords = [p.position_absolute.x, p.position_absolute.y, p.position_absolute.z]
                        for place in self.SITTING_PLACES_IN_FURNITURE:
                            # print(name, center_coords)
                            name = place["name"]
                            center_coords = place["center_coords"]

                            dist = math.sqrt( (p_coords[0]-center_coords[0])**2 + (p_coords[1]-center_coords[1])**2 )

                            if dist < temp_min_dist_sitting_places_dict[name]:
                                temp_min_dist_sitting_places_dict[name] = dist
                 
                print("temp_min_dist_sitting_places_dict:", temp_min_dist_sitting_places_dict)   
                print("People found for seating:", len(p_considered))    
                
                if all(dist < self.min_dist_for_sitting_place_to_be_occupied for dist in temp_min_dist_sitting_places_dict.values()):
                    # SPECIAL CASE, if all seats are occupided, by default we say the person should sit in the center of the sofa 
                    # Might make sense to chang in the future
                    neck_position = self.robot.get_location_coords_from_furniture("couch")
                    speak_file = "hri/couch_center"

                else:
                    max_name = max(
                    temp_min_dist_sitting_places_dict,
                    key=temp_min_dist_sitting_places_dict.get
                    )
                    max_value = temp_min_dist_sitting_places_dict[max_name]

                    max_place_info = next(
                        place for place in self.SITTING_PLACES_IN_FURNITURE
                        if place["name"] == max_name
                    )
                    neck_position = [max_place_info["center_coords"][0], max_place_info["center_coords"][1], 0.9]
                    speak_file = max_place_info["speak"]

                # print("Closest sitting place:")
                # for k, v in max_place_info.items():
                #     print(f"{k}: {v}")

                #################################################################################### COMPARE GUEST1 PICTURE !!!!
                #################################################################################### LOOK CORRECT PLACE !!!!
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.set_speech(filename="receptionist/second_guest_name_is", wait_for_end_of=True)
                self.robot.set_speech(filename="person_names/"+self.GUEST2_NAME.replace(" ","_").lower(), wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/favourite_drink_is", wait_for_end_of=True)
                self.robot.set_speech(filename="objects_names/"+self.GUEST2_DRINK, wait_for_end_of=True)
                
                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)

                self.robot.set_speech(filename="receptionist/first_guest_name_is", wait_for_end_of=True)
                self.robot.set_speech(filename="person_names/"+self.GUEST1_NAME.replace(" ","_").lower(), wait_for_end_of=True)
                self.robot.set_speech(filename="receptionist/favourite_drink_is", wait_for_end_of=True)
                self.robot.set_speech(filename="objects_names/"+self.GUEST1_DRINK, wait_for_end_of=True)

                self.robot.set_neck_coords(position=neck_position, wait_for_end_of=False)
                self.robot.set_speech(filename="hri/found_free_seat", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/i_am_looking_at", wait_for_end_of=True)
                self.robot.set_speech(filename=speak_file, wait_for_end_of=True)
                if self.SIDE_TO_LOOK.lower() == "right":
                    self.robot.set_neck(position=self.look_right, wait_for_end_of=False)
                elif self.SIDE_TO_LOOK.lower() == "left":
                    self.robot.set_neck(position=self.look_left, wait_for_end_of=False)
                self.robot.set_speech(filename="receptionist/dear_guest", wait_for_end_of=True)
                self.robot.set_speech(filename="hri/please_take_a_seat", wait_for_end_of=True)
                self.robot.set_speech(filename=speak_file, wait_for_end_of=True)

                self.state = self.task_states["Move_to_start_follow_position"]


            elif self.state == self.task_states["Move_to_start_follow_position"]:
                                        
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="rooms/"+self.SITTING_AREA_ROOM.replace(" ","_").lower(), wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.start_follow_position, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                self.robot.set_speech(filename="rooms/"+self.SITTING_AREA_ROOM.replace(" ","_").lower(), wait_for_end_of=False)
                
                self.state = self.task_states["Follow_host_to_bag_drop"]


            elif self.state == self.task_states["Follow_host_to_bag_drop"]:

                ### DETECT HOST IN FRONT OF ROBOT
                ### CREATE TRACK SYSTEM FOR HOST
                ### FOLLOW HOST TO BAG DROP LOCATION
                ### RECOVERY SYSTEMS
                ### USE TOUCHSCREEN TO CONFIRM ARRIVAL AT BAG DROP LOCATION
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
