#!/usr/bin/env python3
import rclpy
import threading
import time
import math
from charmie_interfaces.msg import DetectedObject
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  True,
    "charmie_audio":                False,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          True,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
    "charmie_neck":                 True,
    "charmie_radar":                True,
    "charmie_sound_classification": False,
    "charmie_speakers":             True,
    "charmie_speakers_save":        False,
    "charmie_tracking":             False,
    "charmie_tray_gripper":         False,
    "charmie_yolo_objects":         True,
    "charmie_yolo_pose":            True,
    "charmie_yolo_world":           True,
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
            "Waiting_for_task_start":           0,
            "State_selector":                   1,
            "Final_state":                      2,

            # "Solve_misplaced_objects_legacy_full_house":      30, # Legacy, but left here, if necessary in the future
            # "Solve_people_with_requests_legacy_full_house":   31, # Legacy, but left here, if necessary in the future
        }


    def configurables(self): # Variables that may change depending on the arena the robot does the task 

        # Configurables for Task Selection:
        self.SOLVE_DOOR_OPENING = True
        self.SOLVE_REQUEST_FROM_PERSON_BEHIND_DOOR = True
        self.SOLVE_MISPLACED_OBJECTS = True
        self.MAX_PROBLEM_SOLVING_MISPLACEDED_OBJECTS = 1
        self.SOLVE_PEOPLE_WITH_REQUESTS = True
        self.MAX_PROBLEM_SOLVING_PEOPLE_WITH_REQUESTS = 1
        self.SOLVE_TRASH_OBJECTS = True
        self.MAX_PROBLEM_SOLVING_TRASH_OBJECTS = 1

        # Overall Configurables:
        self.rooms_to_go = ["Kitchen", "Living room", "Hallway", "Office"]
        self.rooms_to_go = [s.replace(" ", "_").lower() for s in self.rooms_to_go]
        # Initial Position
        #self.initial_position = self.robot.get_navigation_coords_from_furniture("dishwasher")
        self.initial_position = [0.0, 0.0, 0.0]
        # self.initial_position = self.robot.get_navigation_coords_from_furniture(furniture="Dinner Table")
        # self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        print(self.initial_position)

        # Configurables for Door Opening and Request Getting:
        self.handle_side = "right"
        pass

        # Configurables for Misplaced Objects:
        self.FURNITURE_WE_WANT_TO_ANALYSE = ["Shelf", "Coffee Table", "Dishwasher", "Dinner Table", "Pantry", "Office Table"]
        # self.FURNITURE_WE_WANT_TO_ANALYSE = ["Office Table", "Office Counter", "Bench", "Shelf", "Coffee Table", "Dishwasher", "Dinner Table", "Kitchen Counter", "Kitchen Cabinet", "Pantry"]
        self.FURNITURE_WE_WANT_TO_ANALYSE = [s.replace(" ", "_").lower() for s in self.FURNITURE_WE_WANT_TO_ANALYSE]
        self.FURNITURE_WITH_ONLY_ONE_TETA = ["Dishwasher", "Coffee Table"]
        self.FURNITURE_WITH_ONLY_ONE_TETA = [s.replace(" ", "_").lower() for s in self.FURNITURE_WITH_ONLY_ONE_TETA]
        self.ONE_TETA_SEARCH_POSITION = [[0, -15]] 
        
        self.IGNORED_OBJECT               = ["Water", "Peach", "Apple"]
        self.IGNORED_OBJECT               = [s.replace(" ", "_").lower() for s in self.IGNORED_OBJECT]
        self.NON_PICKABLE_OBJECT          = ["Tuna", "Plate", "Bowl", "Dishwasher Tab"] 
        self.NON_PICKABLE_OBJECT          = [s.replace(" ", "_").lower() for s in self.NON_PICKABLE_OBJECT]
        # Configurables for People with Requests:
        self.tetas_for_rooms = {
            "Kitchen":          [[-20, -10], [40, -10]],
            "Living Room":      [[-30, -10], [30, -10]],
            "Hallway":          [[-30, -10], [30, -10]],
            "Office":           [[-40, -10], [20, -10]]
        }
        self.tetas_for_rooms = {key.replace(" ", "_").lower(): value for key, value in self.tetas_for_rooms.items()}

        # Configurables for Trash Objects:
        self.TRASH_SEARCH_CAMERA = "head"
        self.divisions          = ["kitchen"         ,"living room"         ,"hallway","office","bedroom","workshop"]
        self.divisions          = [s.replace(" ", "_").lower() for s in self.divisions]
        self.trashcans          = ["kitchen trashcan","living room trashcan",""       ,""      ,""       ,""        ]
        self.trashcans          = [s.replace(" ", "_").lower() for s in self.trashcans]
        self.MIN_OBJECT_DISTANCE_X = 0.05
        self.MAX_OBJECT_DISTANCE_X = 6
        self.MIN_OBJECT_DISTANCE_Y = -6
        self.MAX_OBJECT_DISTANCE_Y = 6
        

    def main(self):

        self.configurables() # set all the configuration variables
        
        self.robot.set_task_name_and_states(task_name=self.TASK_NAME, task_states=self.task_states) # Necessary to visualize states and task info in GUI
        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1 # state to be set by task_demo, so that the task can wait for new state to be set by task_demo

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        # self.search_tetas_horizontal = [[0, -45], [-40, -45], [40, -45]]
        # self.search_tetas_vertical = [[0, -15], [0, -35], [0, 15]]

        self.search_tetas_horizontal = [[-20, -20], [20, -20]]
        self.search_tetas_vertical = [[0, -15], [0, 15]]

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
                self.robot.set_speech(filename="finals/start_finals", wait_for_end_of=True)
                self.robot.wait_for_start_button()
                self.robot.wait_for_door_opening()
                self.robot.enter_house_after_door_opening()
                self.state = self.task_states["State_selector"]


            elif self.state == self.task_states["State_selector"]:

                misplaced_objects_problems_solved_ctr = 0
                peoples_with_requests_problems_solved_ctr = 0
                trash_objects_problems_solved_ctr = 0

                # Starts with door opening and getting the request fsolve_open_door_and_get_requestrom the person behind the door
                self.solve_open_door_and_get_request()
                
                while True:
                    for room in self.rooms_to_go:

                        # if they are all False, it means we will only do detections from now on, but we have decided to reset all flags
                        if not self.SOLVE_MISPLACED_OBJECTS and not self.SOLVE_TRASH_OBJECTS and not self.SOLVE_PEOPLE_WITH_REQUESTS:
                            print("CLEARED ALL FLAGS")
                            self.SOLVE_MISPLACED_OBJECTS = True
                            self.SOLVE_TRASH_OBJECTS = True
                            self.SOLVE_PEOPLE_WITH_REQUESTS = True

                        misplaced_objects_number_of_problems_solved = self.solve_misplaced_objects(room=room, requests_left=self.MAX_PROBLEM_SOLVING_MISPLACEDED_OBJECTS - misplaced_objects_problems_solved_ctr)
                        print("misplaced_objects_number_of_problems_solved:", misplaced_objects_number_of_problems_solved)
                        misplaced_objects_problems_solved_ctr += misplaced_objects_number_of_problems_solved
                        if misplaced_objects_problems_solved_ctr >= self.MAX_PROBLEM_SOLVING_MISPLACEDED_OBJECTS:
                            self.SOLVE_MISPLACED_OBJECTS = False
                            misplaced_objects_problems_solved_ctr = 0

                        trash_objects_number_of_problems_solved = self.solve_trash_objects(room=room, requests_left=self.MAX_PROBLEM_SOLVING_TRASH_OBJECTS - trash_objects_problems_solved_ctr, pick_to_trashcan=self.SOLVE_TRASH_OBJECTS, camera=self.TRASH_SEARCH_CAMERA)
                        print("trash_objects_number_of_problems_solved:", trash_objects_number_of_problems_solved)
                        trash_objects_problems_solved_ctr += trash_objects_number_of_problems_solved
                        if trash_objects_problems_solved_ctr >= self.MAX_PROBLEM_SOLVING_TRASH_OBJECTS:
                            self.SOLVE_TRASH_OBJECTS = False
                            trash_objects_problems_solved_ctr = 0

                        peoples_with_requests_number_of_problems_solved = self.solve_people_with_requests(room=room, requests_left=self.MAX_PROBLEM_SOLVING_PEOPLE_WITH_REQUESTS - peoples_with_requests_problems_solved_ctr)
                        print("peoples_with_requests_number_of_problems_solved:", peoples_with_requests_number_of_problems_solved)
                        peoples_with_requests_problems_solved_ctr += peoples_with_requests_number_of_problems_solved
                        if peoples_with_requests_problems_solved_ctr >= self.MAX_PROBLEM_SOLVING_PEOPLE_WITH_REQUESTS:
                            self.SOLVE_PEOPLE_WITH_REQUESTS = False
                            peoples_with_requests_problems_solved_ctr = 0

                # Not used in 2026 finals strategy, but left here for future use
                # self.solve_basket_misplacement()
                # self.solve_close_dishwasher()

                self.state = self.task_states["Final_state"]


            elif self.state == self.task_states["Final_state"]:
                
                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="finals/finished_finals", wait_for_end_of=False)
                self.robot.set_speech(filename="sound_effects/cr7_siuu", wait_for_end_of=False)
                
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











    def solve_open_door_and_get_request(self):
        print("\n>>> Current Task State: Solve_Open_Door_And_Get_Request <<<\n")

        # Move to door coordinates
        self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
        self.robot.set_speech(filename="furniture/exit", wait_for_end_of=False)
        self.robot.move_to_position(move_coords=[-0.1, -1.35, 0.0], wait_for_end_of=True)
        # self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(furniture="Exit"), wait_for_end_of=True)
        self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
        self.robot.set_speech(filename="furniture/exit", wait_for_end_of=False)

        # Open the door
        self.robot.open_door(push_pull="push", handle_side="right", wait_for_end_of=True)

        # Request GPSR command
        self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)

        self.robot.set_speech(filename="generic/please_wait", wait_for_end_of=True)
        time.sleep(0.5) # a little of robot not doing anythingbefore starting background noise calibration 
        # self.robot.calibrate_audio()

        ### llp = self.robot.receive_command_and_generate_low_level_planner()
        ### print("Low-level planner:", llp)
        self.robot.wait_for_start_button()

        # TODO: PLACEHOLDER: CHECK IF ORDER CAN BE EXECUTED...
        order_can_be_executed = True

        if order_can_be_executed:
            
            # TODO: PLACEHOLDER: ADD LOW LEVEL PLANNER EXECUTION HERE ...
            time.sleep(0.5)
            self.robot.set_speech(filename="sound_effects/cr7_siuu", wait_for_end_of=True)
            # self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)
            
        else: # WILL SEARCH FOR MORE PEOPLE IN THE SAME ROOM
            self.robot.set_speech(filename="finals/cannot_perform_task", wait_for_end_of=True)
            self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)


    def solve_misplaced_objects(self, room, requests_left):
        print("\n>>> Current Task State: Solve_Misplaced_Objects <<<\n")

        number_of_replaced_objects = 0

        for current_furniture in self.FURNITURE_WE_WANT_TO_ANALYSE:

            if self.robot.get_room_from_furniture(current_furniture) == room and number_of_replaced_objects < requests_left:

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+ current_furniture, wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(furniture=current_furniture), wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                # self.robot.set_speech(filename="furniture/"+ current_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                objects_in_correct_furniture = []
                objects_in_wrong_furniture = []
                ignored_objects = []

                if current_furniture in self.FURNITURE_WITH_ONLY_ONE_TETA:
                    search_misplaced_obj_tetas =self.ONE_TETA_SEARCH_POSITION
                elif self.robot.get_look_orientation_from_furniture(furniture=current_furniture) == "horizontal":
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

                    if obj.furniture_location.replace(" ","_").lower() == current_furniture \
                    and (obj.object_name.replace(" ","_").lower() not in self.IGNORED_OBJECT):
                        
                        print("Object:", obj.object_name, "List of ignored:", self.IGNORED_OBJECT)

                        if obj.furniture_location.replace(" ","_").lower() != self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(object_name=obj.object_name.replace(" ","_").lower())):
                            print("OBJECT IS NOT IN CORRECT FURNITURE")
                            # self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                            # self.robot.set_speech(filename="finals/object_in_the_wrong_furniture", wait_for_end_of=True)
                            objects_in_wrong_furniture.append(obj)
                        else:
                            print("OBJECT IN CORRECT FURNITURE")
                            # self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                            # self.robot.set_speech(filename="finals/object_in_the_correct_furniture", wait_for_end_of=True)
                            objects_in_correct_furniture.append(obj)

                    else:
                        print("I don't care about this object")
                        ignored_objects.append(obj)


                # print("Objects in correct furniture:", objects_in_correct_furniture)
                # print("Objects in wrong furniture:", objects_in_wrong_furniture)
                # print("Ignored objects:", ignored_objects)
                # correct_objects_counter = 0

                if len(objects_in_wrong_furniture) > 0:

                    self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                    for obj in objects_in_wrong_furniture:
                        self.robot.set_speech(filename="finals/encountered_a_problem", wait_for_end_of=True)
                        self.robot.detected_object_to_face_path(object=obj, send_to_face=True)
                        self.robot.set_speech(filename="generic/found_the", wait_for_end_of=True)
                        self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                        self.robot.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)
                        self.robot.set_speech(filename="finals/object_should_be_placed_on", wait_for_end_of=True)
                        self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(obj.object_name.replace(" ","_").lower())), wait_for_end_of=False)
                    self.robot.set_face("charmie_face")
                        
                if self.SOLVE_MISPLACED_OBJECTS:

                    if len(objects_in_wrong_furniture) == 0:
                        print("All objects are in the correct furniture")
                        self.robot.set_speech(filename="finals/check_the_next_furniture", wait_for_end_of=False)

                    else:

                        objects_in_wrong_furniture.sort(key=lambda p: math.hypot(p.position_absolute.x, p.position_absolute.y))

                        for wrong_obj in objects_in_wrong_furniture:

                            if wrong_obj.object_name.replace(" ","_").lower() not in self.NON_PICKABLE_OBJECT:

                                if current_furniture == "dinner_table":
                                    self.robot.move_to_pre_pick_position_after_search_for_objects(furniture=current_furniture, object=wrong_obj) # detected object

                                picked_height, asked_help = self.robot.pick_object(selected_object=wrong_obj.object_name.replace(" ","_").lower(),
                                                            pick_mode=self.robot.get_standard_pick_from_object(wrong_obj.object_name.replace(" ","_").lower()),
                                                            first_search_tetas=search_misplaced_obj_tetas, max_search_attempts=2, finals_flag=True)
                                
                                if picked_height != -1: # ASK FOR HELP WAS CORRECT
                                
                                    place_furniture = self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(wrong_obj.object_name.replace(" ","_").lower()))
                                
                                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                                    self.robot.set_speech(filename="furniture/" + place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(place_furniture.replace(" ","_").lower()), wait_for_end_of=True)


                                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                                    self.robot.set_speech(filename="furniture/" + place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                                    if len(self.robot.get_height_from_furniture(place_furniture)) > 1:
                                        if place_furniture == "pantry" or place_furniture == "shelf": # HARDCODED
                                            shelf_number_place = 1
                                        else:
                                            shelf_number_place = 2
                                    else:
                                        shelf_number_place = 0

                                    self.robot.place_object_in_furniture(selected_object=wrong_obj.object_name.replace(" ","_").lower(),
                                                                        place_mode=self.robot.get_standard_pick_from_object(wrong_obj.object_name.replace(" ","_").lower()),
                                                                        furniture=place_furniture.replace(" ","_").lower(),
                                                                        shelf_number=shelf_number_place, place_height=picked_height,
                                                                        return_to_initial_position=True)
                                    
                                    number_of_replaced_objects += 1
                                    if number_of_replaced_objects == requests_left: # breaks if we have solved the number of requests we wanted to solve
                                        break
                        
                        if number_of_replaced_objects < requests_left: # breaks if we have solved the number of requests we wanted to solve
                            self.robot.set_speech(filename="finals/check_the_next_furniture", wait_for_end_of=True)

        return number_of_replaced_objects
    

    def solve_people_with_requests(self, room, requests_left):
        print("\n>>> Current Task State: Solve_People_With_Requests <<<\n")

        print(room)
        number_of_solved_requests = 0
        
        no_people_left_with_requests_in_this_room = False
        while not no_people_left_with_requests_in_this_room:

            # NAVIGATION TO ROOM
            self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

            self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
            self.robot.set_speech(filename="rooms/"+room, wait_for_end_of=False)

            self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_room(room), wait_for_end_of=True)

            self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
            self.robot.set_speech(filename="finals/raise_your_hand", wait_for_end_of=True) # may be problematic becuase referee may place himself in front of the robot...
            # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
            # self.robot.set_speech(filename="rooms/"+room, wait_for_end_of=True)

            tetas = self.tetas_for_rooms[room]
            print("Tetas:", tetas)
            people_found = self.robot.search_for_person(tetas=tetas, only_detect_person_arm_raised=True)

            people_with_requests = []
            print("FOUND:", len(people_found)) 
            for p in people_found:
                if p.arm_raised and p.room_location.replace(" ", "_").lower() == room:
                    people_with_requests.append(p)
                    print("ID:", p.index, p.arm_raised, p.room_location, p.position_absolute.x, p.position_absolute.y, " - HAS REQUEST!")
                else:
                    print("ID:", p.index, p.arm_raised, p.room_location, p.position_absolute.x, p.position_absolute.y, " - NO REQUEST!")

            # There is a discussion to be had, whether this should be a for loop.
            # The reasoning behind this decision, is that after a long time and executing a GPSR task,
            # the people may have changed position, and therefore the robot would go to an old position.
            # If there are multiple people with requests, the robot does not continue to be next room
            if len(people_with_requests) == 0:
                no_people_left_with_requests_in_this_room = True
            else: # there are people with requests

                # REORDER BY DISTANCE TO THE ROBOT (NOT BY NAVIGATION DISTANCE)
                people_with_requests.sort(key=lambda p: math.hypot(p.position_absolute.x, p.position_absolute.y))

                if not self.SOLVE_PEOPLE_WITH_REQUESTS: # added this just because of the WFEO of the speaks, otherwise I would leave this task and maybe move to other place

                    for p in people_with_requests:

                        # INFORM THERE WAS AN ENCOUNTERED PROBLEM
                        self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                        self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute.z], wait_for_end_of=False)
                        # self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, 1.6], wait_for_end_of=True) # pre defined height for better looking at the face

                        self.robot.set_speech(filename="finals/encountered_a_problem", wait_for_end_of=True)
                        self.robot.set_speech(filename="finals/problem_person_with_request", wait_for_end_of=True)
                        self.robot.set_speech(filename="finals/check_face_detected_person", wait_for_end_of=True) # may be problematic becuase referee may place himself in front of the robot...
                        time.sleep(2.0) # wait a bit to let the speaks be said before going to next room

                    self.robot.set_face("charmie_face")
                    no_people_left_with_requests_in_this_room = True
                
                else: # if self.SOLVE_PEOPLE_WITH_REQUESTS:
                    person_with_request = people_with_requests[0]

                    # INFORM THERE WAS AN ENCOUNTERED PROBLEM
                    self.robot.detected_person_to_face_path(person=person_with_request, send_to_face=True)
                    self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, person_with_request.position_absolute.z], wait_for_end_of=False)
                    # self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, 1.6], wait_for_end_of=True) # pre defined height for better looking at the face

                    self.robot.set_speech(filename="finals/encountered_a_problem", wait_for_end_of=True)
                    self.robot.set_speech(filename="finals/problem_person_with_request", wait_for_end_of=False)
                    self.robot.set_speech(filename="finals/check_face_detected_person", wait_for_end_of=False) # may be problematic becuase referee may place himself in front of the robot...

                    # NAVIGATION TO PERSON
                    self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                    # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    # self.robot.set_speech(filename="generic/person", wait_for_end_of=False)
                    self.robot.move_to_person(person=person_with_request)

                    # CONFIRM PERSON HAS A REQUEST
                    self.robot.detected_person_to_face_path(person=person_with_request, send_to_face=True)
                    self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, person_with_request.position_absolute.z], wait_for_end_of=False)
                    self.robot.set_speech(filename="finals/charmie_here_to_help", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/press_correct_option_touchscreen", wait_for_end_of=True) # SAY: Please press the correct option on my face.
                    answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Do you have a new request for me?", speak_results=False, speak_timeout=False, start_speak_file="finals/do_you_have_a_new_request_for_me", wait_for_end_of=True)
                    self.robot.set_face("charmie_face")
                    
                    if answer != ["yes"]:
                        self.robot.set_speech(filename="finals/problem_with_accepting_request", wait_for_end_of=True)
                        no_people_left_with_requests_in_this_room = True # MOVE TO NEXT ROOM, THIS IS NOT IDEAL BUT AVOIDS TO ROBOT GETTING STUCK IN ENDLESS LOOP OF GOING BACK TO SEARCH AND CONTINUING TO FIND THE SAME PERSON WITH THE REQUEST
                    else:
                        # EXECUTE GPSR
                        self.robot.set_speech(filename="generic/please_wait", wait_for_end_of=True)
                        time.sleep(0.5) # a little of robot not doing anythingbefore starting background noise calibration 
                        # self.robot.calibrate_audio()

                        ### llp = self.robot.receive_command_and_generate_low_level_planner()
                        ### print("Low-level planner:", llp)
                        self.robot.wait_for_start_button()

                        # TODO: PLACEHOLDER: CHECK IF ORDER CAN BE EXECUTED...
                        order_can_be_executed = True

                        if order_can_be_executed:
                            
                            # TODO: PLACEHOLDER: ADD LOW LEVEL PLANNER EXECUTION HERE ...
                            time.sleep(0.5)
                            self.robot.set_speech(filename="sound_effects/cr7_siuu", wait_for_end_of=True)
                            
                            number_of_solved_requests+=1
                            self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)
                            
                        else: # WILL SEARCH FOR MORE PEOPLE IN THE SAME ROOM
                            self.robot.set_speech(filename="finals/cannot_perform_task", wait_for_end_of=True)
                            self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)

            if number_of_solved_requests == requests_left: # breaks if we have solved the number of requests we wanted to solve
                no_people_left_with_requests_in_this_room = True

        return number_of_solved_requests


    def solve_trash_objects(self, room="", requests_left=0, pick_to_trashcan=False, camera="head"):
        print("\n>>> Current Task State: Solve_Trash_Objects <<<\n")
        requests_solved = 0

        if requests_left > 0:
            self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
            self.robot.set_speech(filename="rooms/"+ room.replace(" ","_").lower(), wait_for_end_of=False)
            self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_room(room=room), wait_for_end_of=True)
            valid_detected_object = DetectedObject()
            validated = False

            MIN_OBJECT_DISTANCE_X = self.MIN_OBJECT_DISTANCE_X 
            MAX_OBJECT_DISTANCE_X = self.MAX_OBJECT_DISTANCE_X
            MIN_OBJECT_DISTANCE_Y = self.MIN_OBJECT_DISTANCE_Y 
            MAX_OBJECT_DISTANCE_Y = self.MAX_OBJECT_DISTANCE_Y 
            goal = ""

            if camera=="head":
                objects_found = self.robot.search_for_objects(tetas = [[0.0,-40.0]], time_in_each_frame=1.5, list_of_objects=[], detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
            else:
                objects_found = self.robot.search_for_objects(tetas = [[0.0,-40.0]], time_in_each_frame=1.5, list_of_objects=[], detect_objects=False, detect_objects_hand=False, detect_objects_base=True)

            objects_found_validated = []
            if objects_found:
                for obj in objects_found:
                    conf   = f"{obj.confidence * 100:.0f}%"
                    cam_x_ = f"{obj.position_relative.x:5.2f}"
                    cam_y_ = f"{obj.position_relative.y:5.2f}"
                    cam_z_ = f"{obj.position_relative.z:5.2f}"

                    print(f"{'ID:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({cam_x_},{cam_y_},{cam_z_} {obj.position_absolute.z}{obj.furniture_location})")

                    if MIN_OBJECT_DISTANCE_X < obj.position_relative.x < MAX_OBJECT_DISTANCE_X and \
                        MIN_OBJECT_DISTANCE_Y < obj.position_relative.y < MAX_OBJECT_DISTANCE_Y and \
                        (obj.position_absolute.z < self.robot.get_object_height_from_object(obj.object_name) * 2 or \
                         obj.position_absolute.z < self.robot.get_object_length_from_object(obj.object_name) * 2 or \
                         obj.position_absolute.z < self.robot.get_object_width_from_object(obj.object_name) * 2):
                        objects_found_validated.append(obj)

                    # if MIN_OBJECT_DISTANCE_X < obj.position_relative.x < MAX_OBJECT_DISTANCE_X and MIN_OBJECT_DISTANCE_Y < obj.position_relative.y < MAX_OBJECT_DISTANCE_Y and (obj.position_absolute.z < self.robot.get_object_height_from_object(obj.object_name) * 2 or obj.position_absolute.z < self.robot.get_object_length_from_object(obj.object_name) * 2 or obj.position_absolute.z < self.robot.get_object_width_from_object(obj.object_name) * 2) and not validated:
                    #     valid_detected_object = obj
                    #     validated = True
                    # if validated:
                    #     if MIN_OBJECT_DISTANCE_X < obj.position_relative.x < MAX_OBJECT_DISTANCE_X and MIN_OBJECT_DISTANCE_Y < obj.position_relative.y < MAX_OBJECT_DISTANCE_Y and (obj.position_absolute.z < self.robot.get_object_height_from_object(obj.object_name) * 2 or obj.position_absolute.z < self.robot.get_object_length_from_object(obj.object_name) * 2 or obj.position_absolute.z < self.robot.get_object_width_from_object(obj.object_name) * 2) and valid_detected_object.position_relative.x > obj.position_relative.x:
                    #         valid_detected_object = obj
                    
            # if validated == True:
            if len(objects_found_validated) > 0:

                # REORDER BY DISTANCE TO THE ROBOT (NOT BY NAVIGATION DISTANCE)
                objects_found_validated.sort(key=lambda p: math.hypot(p.position_absolute.x, p.position_absolute.y))
                valid_detected_object = objects_found_validated[0]

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                for o in objects_found_validated:
                    self.robot.set_speech(filename="finals/encountered_a_problem", wait_for_end_of=True)
                    self.robot.detected_object_to_face_path(object=o, send_to_face=True)
                    self.robot.set_speech(filename="generic/found_the", wait_for_end_of=True)
                    self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)
                    self.robot.set_speech(filename="finals/should_be_in_trashcan", wait_for_end_of=True)
                self.robot.set_face("charmie_face")
                    
                if pick_to_trashcan:

                    counter = 0
                    for d in self.divisions:
                        if d == room:
                            goal = self.trashcans[counter]
                        counter = counter + 1
                    
                    if goal != "":
                        self.robot.ask_help_pick_object_gripper(object_d = valid_detected_object, look_judge= [0,0], show_detection = False)
                        self.robot.set_arm(command="ask_for_objects_to_initial_position", wait_for_end_of=True)
                        self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/"+goal.replace(" ","_").lower(), wait_for_end_of=False)

                        self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(furniture=goal), wait_for_end_of=True)
                        self.robot.place_object_in_furniture(selected_object=valid_detected_object.object_name,place_mode = "front",furniture=goal)
                        requests_solved = 1

            return requests_solved


    def solve_basket_misplacement(self):
        print("\n>>> Current Task State: Solve_Basket_Misplacement <<<\n")
        # NOT PART OF ROBOCUP 2026 FINALS STRATEGY


    def solve_close_dishwasher(self):
        print("\n>>> Current Task State: Solve_Close_Dishwasher <<<\n")
        # NOT PART OF ROBOCUP 2026 FINALS STRATEGY


    """
    elif self.state == self.task_states["Solve_misplaced_objects_legacy_full_house"]:

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
                        self.robot.set_speech(filename="finals/object_in_the_wrong_furniture", wait_for_end_of=True)
                        objects_in_wrong_furniture.append(obj.object_name)
                    else:
                        print("OBJECT IN CORRECT FURNITURE")
                        # self.robot.set_speech(filename="objects_names/"+obj.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                        # self.robot.set_speech(filename="finals/object_in_the_correct_furniture", wait_for_end_of=True)
                        objects_in_correct_furniture.append(obj.object_name)

                else:
                    print("I don't care about this object")
                    ignored_objects.append(obj.object_name)


            print("Objects in correct furniture:", objects_in_correct_furniture)
            print("Objects in wrong furniture:", objects_in_wrong_furniture)
            print("Ignored objects:", ignored_objects)
            # correct_objects_counter = 0

            if len(objects_in_wrong_furniture) == 0:
                print("All objects are in the correct furniture")
                self.robot.set_speech(filename="finals/check_the_next_furniture", wait_for_end_of=False)

            elif len(objects_in_wrong_furniture) == 1:

                for wrong_obj in objects_in_wrong_furniture:

                    picked_height, asked_help = self.robot.pick_object_risky(selected_object=wrong_obj,
                                                pick_mode=self.robot.get_standard_pick_from_object(wrong_obj),
                                                first_search_tetas=search_misplaced_obj_tetas, return_arm_to_initial_position=False)
                    
                    place_furniture = self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(wrong_obj))
                
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/" + place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(place_furniture.replace(" ","_").lower()), wait_for_end_of=True)


                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/" + place_furniture.replace(" ","_").lower(), wait_for_end_of=False)

                    if len(place_furniture) > 1:
                        shelf_number_place = 2
                    else:
                        shelf_number_place = 0

                    self.robot.place_object_in_furniture(selected_object=wrong_obj,
                                                        place_mode=self.robot.get_standard_pick_from_object(wrong_obj),
                                                        furniture=place_furniture,
                                                        shelf_number=shelf_number_place, place_height=picked_height,
                                                        return_to_initial_position=True)
                    
                    self.robot.set_speech(filename="finals/check_the_next_furniture", wait_for_end_of=True)
                    
            elif len(objects_in_wrong_furniture) > 1:
                    
                    wrong_obj_1 = objects_in_wrong_furniture[0]

                    picked_height_1, asked_help_1 = self.robot.pick_object_risky(selected_object=wrong_obj_1,
                                                                                pick_mode=self.robot.get_standard_pick_from_object(wrong_obj_1),
                                                                                first_search_tetas=search_misplaced_obj_tetas)
                    
                    place_object_in_tray_height = self.robot.place_object_in_furniture(selected_object=wrong_obj_1,
                                                                                        place_mode=self.robot.get_standard_pick_from_object(object_name=wrong_obj_1),
                                                                                        furniture="Tray", place_height=picked_height_1)
                    
                    wrong_obj_2 = objects_in_wrong_furniture[1]

                    picked_height_2, asked_help_2 = self.robot.pick_object_risky(selected_object=wrong_obj_2,
                                                                                pick_mode=self.robot.get_standard_pick_from_object(wrong_obj_2),
                                                                                first_search_tetas=search_misplaced_obj_tetas)
        
                    place_furniture_2 = self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(wrong_obj_2))
                
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/" + place_furniture_2.replace(" ","_").lower(), wait_for_end_of=False)

                    self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(place_furniture_2.replace(" ","_").lower()), wait_for_end_of=True)


                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/" + place_furniture_2.replace(" ","_").lower(), wait_for_end_of=False)

                    if len(place_furniture_2) > 1:
                        shelf_number_place_2 = 2
                    else:
                        shelf_number_place_2 = 0

                    
                    self.robot.place_object_in_furniture(selected_object=wrong_obj_2,
                                                        place_mode=self.robot.get_standard_pick_from_object(wrong_obj_2),
                                                        furniture=place_furniture_2,
                                                        shelf_number=shelf_number_place_2, place_height=picked_height_2,
                                                        return_to_initial_position=True)
                    
                    picked_height_3 = self.robot.pick_object_risky(selected_object=wrong_obj_1,
                                                                    pick_mode=self.robot.get_standard_pick_from_object(wrong_obj_1),
                                                                    furniture="Tray", placed_in_tray_height = place_object_in_tray_height)
        
                    
                    place_furniture_1 = self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(wrong_obj_1))

                    if place_furniture_1 != place_furniture_2:

                        self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/" + place_furniture_1.replace(" ","_").lower(), wait_for_end_of=False)

                        self.robot.move_to_position(self.robot.get_navigation_coords_from_furniture(place_furniture_1.replace(" ","_").lower()), wait_for_end_of=True)

                        self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                        self.robot.set_speech(filename="furniture/" + place_furniture_2.replace(" ","_").lower(), wait_for_end_of=False)

                    else:
                        print("SAME PLACE FURNITURE")
                        pass

                    if len(place_furniture_1) > 1:
                        shelf_number_place_1 = 2
                    else:
                        shelf_number_place_1 = 0

                    self.robot.place_object_in_furniture(selected_object=wrong_obj_1,
                                                            place_mode=self.robot.get_standard_pick_from_object(wrong_obj_1),
                                                            furniture=place_furniture_1,
                                                            shelf_number=shelf_number_place_1, place_height=picked_height_3,
                                                            return_to_initial_position=True)

                    print("I have already moved 2 objects, I will stop looking for more objects in this furniture to save time")
                    self.robot.set_speech(filename="finals/check_the_next_furniture", wait_for_end_of=True)

                    # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    # self.robot.set_speech(filename="furniture/"+ current_furniture.replace(" ","_").lower(), wait_for_end_of=False)
                    # self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(furniture=current_furniture), wait_for_end_of=True)
                    # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    # self.robot.set_speech(filename="furniture/"+ current_furniture.replace(" ","_").lower(), wait_for_end_of=False)                                                    
            
        
        self.state = self.task_states["Final_state"]
        """

    """
    elif self.state == self.task_states["Solve_people_with_requests_legacy_full_house"]:

        number_of_solved_requests = 0
        print(self.rooms_to_go)
        for room in self.rooms_to_go:
            print(room)

            no_people_left_with_requests_in_this_room = False
            while not no_people_left_with_requests_in_this_room:

                # NAVIGATION TO ROOM
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="rooms/"+room, wait_for_end_of=False)

                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_room(room), wait_for_end_of=True)

                self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                self.robot.set_speech(filename="finals/raise_your_hand", wait_for_end_of=True) # may be problematic becuase referee may place himself in front of the robot...
                # self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                # self.robot.set_speech(filename="rooms/"+room, wait_for_end_of=True)

                people_found = self.robot.search_for_person(tetas=self.search_for_people_tetas, only_detect_person_arm_raised=True)

                people_with_requests = []
                print("FOUND:", len(people_found)) 
                for p in people_found:
                    if p.arm_raised and p.room_location.replace(" ", "_").lower() == room:
                        people_with_requests.append(p)
                        print("ID:", p.index, p.arm_raised, p.room_location, p.position_absolute.x, p.position_absolute.y, " - HAS REQUEST!")
                    else:
                        print("ID:", p.index, p.arm_raised, p.room_location, p.position_absolute.x, p.position_absolute.y, " - NO REQUEST!")

                # There is a discussion to be had, whether this should be a for loop.
                # The reasoning behind this decision, is that after a long time and executing a GPSR task,
                # the people may have changed position, and therefore the robot would go to an old position.
                # If there are multiple people with requests, the robot does not continue to be next room
                if len(people_with_requests) == 0:
                    no_people_left_with_requests_in_this_room = True
                else: # there are people with requests

                    # REORDER BY DISTANCE TO THE ROBOT (NOT BY NAVIGATION DISTANCE)
                    people_with_requests.sort(key=lambda p: math.hypot(p.position_absolute.x, p.position_absolute.y))
                    person_with_request = people_with_requests[0]

                    # INFORM THERE WAS AN ENCOUNTERED PROBLEM
                    self.robot.detected_person_to_face_path(person=person_with_request, send_to_face=True)
                    self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, person_with_request.position_absolute.z], wait_for_end_of=False)
                    # self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, 1.6], wait_for_end_of=True) # pre defined height for better looking at the face
                    self.robot.set_speech(filename="finals/encountered_a_problem", wait_for_end_of=True)
                    self.robot.set_speech(filename="finals/problem_person_with_request", wait_for_end_of=False)
                    self.robot.set_speech(filename="finals/check_face_detected_person", wait_for_end_of=False) # may be problematic becuase referee may place himself in front of the robot...

                    # NAVIGATION TO PERSON
                    self.robot.set_neck(position=self.look_forward, wait_for_end_of=False)
                    # self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    # self.robot.set_speech(filename="generic/person", wait_for_end_of=False)
                    self.robot.move_to_person(person=person_with_request)

                    # CONFIRM PERSON HAS A REQUEST
                    self.robot.detected_person_to_face_path(person=person_with_request, send_to_face=True)
                    self.robot.set_neck_coords(position=[person_with_request.position_absolute.x, person_with_request.position_absolute.y, person_with_request.position_absolute.z], wait_for_end_of=False)
                    self.robot.set_speech(filename="finals/charmie_here_to_help", wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/press_correct_option_touchscreen", wait_for_end_of=True) # SAY: Please press the correct option on my face.
                    answer = self.robot.set_face_touchscreen_menu(choice_category=["yes_or_no"], timeout=10, instruction="Do you have a new request for me?", speak_results=False, speak_timeout=False, start_speak_file="finals/do_you_have_a_new_request_for_me", wait_for_end_of=True)
                    self.robot.set_face("charmie_face")
                    
                    if answer != ["yes"]:
                        self.robot.set_speech(filename="finals/problem_with_accepting_request", wait_for_end_of=True)
                        no_people_left_with_requests_in_this_room = True # MOVE TO NEXT ROOM, THIS IS NOT IDEAL BUT AVOIDS TO ROBOT GETTING STUCK IN ENDLESS LOOP OF GOING BACK TO SEARCH AND CONTINUING TO FIND THE SAME PERSON WITH THE REQUEST
                    else:
                        # EXECUTE GPSR
                        self.robot.set_speech(filename="generic/please_wait", wait_for_end_of=True)
                        time.sleep(0.5) # a little of robot not doing anythingbefore starting background noise calibration 
                        # self.robot.calibrate_audio()

                        ### llp = self.robot.receive_command_and_generate_low_level_planner()
                        ### print("Low-level planner:", llp)
                        self.robot.wait_for_start_button()


                        # TODO: PLACEHOLDER: CHECK IF ORDER CAN BE EXECUTED...
                        order_can_be_executed = True

                        if order_can_be_executed:
                            
                            # TODO: PLACEHOLDER: ADD LOW LEVEL PLANNER EXECUTION HERE ...
                            time.sleep(0.5)
                            self.robot.set_speech(filename="sound_effects/cr7_siuu", wait_for_end_of=True)
                            
                            number_of_solved_requests+=1
                            self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)
                            
                        else: # WILL SEARCH FOR MORE PEOPLE IN THE SAME ROOM
                            self.robot.set_speech(filename="finals/cannot_perform_task", wait_for_end_of=True)
                            self.robot.set_speech(filename="finals/please_dont_raise_arm_anymore", wait_for_end_of=True)

                if self.number_of_requests_to_solve == number_of_solved_requests: # breaks if we have solved the number of requests we wanted to solve
                    no_people_left_with_requests_in_this_room = True

            if self.number_of_requests_to_solve == number_of_solved_requests: # breaks if we have solved the number of requests we wanted to solve
                break

        self.state = self.task_states["Final_state"]
        """
