#!/usr/bin/env python3
import rclpy
import threading
import time
from datetime import datetime
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          False,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                False,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         False,
    "charmie_low_level":            False,
    "charmie_navigation":           False,
    "charmie_nav2":                 False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": False,
    "charmie_speakers":             False,
    "charmie_tracking":             False,
    "charmie_yolo_objects":         False,
    "charmie_yolo_pose":            False,
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
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        self.state = Waiting_for_start_button

        print("IN NEW MAIN")
        # time.sleep(2)

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:

                o = "milk"
                c = self.robot.get_object_class_from_object(o)
                f = self.robot.get_furniture_from_object_class(c)
                r = self.robot.get_room_from_furniture(f)
                fh = self.robot.get_height_from_furniture(f)

                fnc = self.robot.get_navigation_coords_from_furniture(f)
                flc = self.robot.get_location_coords_from_furniture(f)
                rnc = self.robot.get_navigation_coords_from_room(r)
                print(o, "|", c, "|", f, "|", fh, "|", fnc, "|", flc, "|", r, "|", rnc)

                ow = self.robot.get_object_width_from_object(o)
                ol = self.robot.get_object_length_from_object(o)
                oh = self.robot.get_object_height_from_object(o)
                os = self.robot.get_object_shape_from_object(o)
                ocp = self.robot.get_how_object_can_be_picked_from_object(o)
                osp = self.robot.get_standard_pick_from_object(o)
                print(o, "|", ow, "|", ol, "|", oh, "|", os, "|", ocp, "|", osp)

                while True:
                    
                    # input("\nPress Enter to continue... (DEBUG)")
                    choice = input("\nPress 'a' to add or 'c' to compare. Press Enter to continue... (DEBUG)").strip().lower()[:1]

                    det_ppl = self.robot.node.detected_people.persons
                    print(len(det_ppl))
                    
                    if choice == 'a':
                        print("You pressed A")
                        for person in det_ppl:
                            print("ID:", person.index)
                            self.robot.add_face_to_face_recognition(person=person, name=f"Person_{person.index}")

                    elif choice == 'c':
                        print("You pressed C")
                    else:
                        print("Other key")
                    

                """ path_recognize = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_13-38-42_153i_.jpg"
                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path_recognize)
                print(pred, round(pred_perc, 2), "Should be ADAM")

                path1 = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_12-41-44_.jpg" # RENATA
                path2 = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_13-34-28_.jpg" # XIAOI
                path3 = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_13-35-55_.jpg" # ADAM

                path4 = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2025-07-11_14-36-49_150_spoon_head.jpg" # INVALID PATH
                path5 = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces/2025-07-11_14-36-49_150_spoon_head.jpg"        # NO FACE
                
                self.robot.add_face_to_face_recognition(image_path=path1, name="Renata")
                self.robot.add_face_to_face_recognition(image_path=path2, name="Xiaoi")
                self.robot.add_face_to_face_recognition(image_path=path3, name="Adam")
                self.robot.add_face_to_face_recognition(image_path=path4, name="T1")
                self.robot.add_face_to_face_recognition(image_path=path5, name="T2")

                path_recognize = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_13-38-42_153i_.jpg"
                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path_recognize)
                print(pred, round(pred_perc, 2), "Should be ADAM")

                path_recognize = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_13-38-42_154i_.jpg"
                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path_recognize)
                print(pred, round(pred_perc, 2), "Should be XIAOI")

                path_recognize = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-18_07-24-43_126i_.jpg"
                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path_recognize)
                print(pred, round(pred_perc, 2), "Should be RENATA")
                
                path_recognize = "charmie_ws/src/charmie_tasks/charmie_tasks/old/receptionist/images/2024-07-16_20-40-04_.jpg"
                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path_recognize)
                print(pred, round(pred_perc, 2), "Should be UNKNOWN")

                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path4)
                print(pred, round(pred_perc, 2), "Should be ERROR")

                pred, pred_perc = self.robot.recognize_face_from_face_recognition(image_path=path5)
                print(pred, round(pred_perc, 2), "Should be ERROR") """


                """ while True:

                    selected_option = self.robot.set_face_touchscreen_menu(timeout=10, mode="numpad", start_speak_file="face_touchscreen_menu/init_touchscreen_keyboard_menu", speak_results=True)
                    print(selected_option)
                    time.sleep(3.0)

                    selected_option = self.robot.set_face_touchscreen_menu(timeout=10, mode="keyboard", start_speak_file="face_touchscreen_menu/init_touchscreen_keyboard_menu", speak_results=True)
                    print(selected_option)
                    time.sleep(3.0)

                    selected_option = self.robot.set_face_touchscreen_menu(["object classes"], timeout=10, mode="single", instruction="Shelves are counter from bottom to up. Bottom is first shelf", speak_results=True)
                    time.sleep(3.0)
                    
                    selected_option = self.robot.set_face_touchscreen_menu([selected_option[0]], timeout=10, mode="single", speak_results=True)
                    time.sleep(3.0) """

                # selected_option_ = self.robot.set_face_touchscreen_menu(["rooms"], timeout=10, mode="single", speak_results=True)
                # selected_option_ = self.robot.set_face_touchscreen_menu([selected_option_[0]], timeout=10, mode="single", speak_results=True)

                # self.robot.set_neck_coords(flc, wait_for_end_of=True)
                # self.robot.set_neck_coords([0,0,0], wait_for_end_of=True)
                # self.robot.set_neck_coords([1.5, 0, 0], wait_for_end_of=True)

                while True:
                    pass

                # self.robot.set_face_touchscreen_menu(["dishes", "toys"])
                # self.robot.set_face_touchscreen_menu(["custom"], custom_options=["ines", "tiago", "charmie", "etc"])
                # self.robot.set_face_touchscreen_menu(["poker"])
                
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods", "fruits", "snacks"], timeout=10))
                # time.sleep(3.0)
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods", "fruits", "snacks"], timeout=10))
                # time.sleep(3.0)
                
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["drinks", "foods", "fruits", "snacks"], timeout=10, mode="single"))
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods", "drinks"], timeout=10, mode="multi"))
                # time.sleep(3.0)
                

                #print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["drinks"], timeout=10, mode="single"))
                # time.sleep(3.0)
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["drinks"], timeout=10, mode="single"))
                # time.sleep(3.0)
                #print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods"], timeout=10, mode="single"))
                # time.sleep(3.0)
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods", "drinks"], timeout=10, mode="single"))
                # time.sleep(3.0)
                # print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["custom"], custom_options=["ines", "tiago", "charmie", "etc"]))
                # time.sleep(3.0)
                #print("Selected option via face touchscreen menu:", self.robot.set_face_touchscreen_menu(["foods", "drinks"], timeout=10, mode="multi"))
                # time.sleep(3.0)
                # selected_option = self.robot.set_face_touchscreen_menu(["names", "drinks"], timeout=10, mode="single", speak_results=True)
                # print(selected_option[0])
                # time.sleep(3.0)

                selected_option = self.robot.set_face_touchscreen_menu(["object classes"], timeout=10, mode="single", speak_results=True)
                selected_option = self.robot.set_face_touchscreen_menu([selected_option[0]], timeout=10, mode="single", speak_results=True)
                
                selected_option_ = self.robot.set_face_touchscreen_menu(["rooms"], timeout=10, mode="single", speak_results=True)
                selected_option_ = self.robot.set_face_touchscreen_menu([selected_option_[0]], timeout=10, mode="single", speak_results=True)


                
                # self.robot.set_face("qr_code")
                # time.sleep(3.0)
                

                while True:
                    self.robot.set_face(camera="head", show_detections=True)
                    print("Stream head camera")
                    time.sleep(3.0)


                    self.robot.set_face(camera="head depth", show_detections=True)
                    print("Stream head camera")
                    time.sleep(3.0)


                    # self.robot.set_face(custom="2024-07-19 18-20-45 15", camera="hand", show_detections=False)
                    # print("help_pick_cornflakes")
                    # time.sleep(3.0)

                    self.robot.set_face(camera="hand", show_detections=True)
                    print("Stream hand camera")
                    time.sleep(3.0)


                    self.robot.set_face(camera="hand depth", show_detections=True)
                    print("Stream hand camera")
                    time.sleep(3.0)

                    # self.robot.set_face("help_pick_orange")
                    # print("help_pick_orange")
                    # time.sleep(3.0)

                    self.robot.set_face(camera="base", show_detections=True)
                    print("Stream base camera")
                    time.sleep(3.0)
                    # self.robot.set_face("help_pick_red_wine")
                    # print("help_pick_red_wine")
                    # time.sleep(3.0)


                    self.robot.set_face(camera="base depth", show_detections=True)
                    print("Stream base camera")
                    time.sleep(3.0)



                    # self.robot.set_face(custom="2024-07-19 18-20-45 15", camera="hand", show_detections=False)
                    # print("help_pick_milk")
                    # time.sleep(3.0)
                    
                    """ 


                    self.robot.set_face("help_pick_water")
                    print("help_pick_water")
                    time.sleep(3.0)

                    self.robot.set_face("help_pick_orange")
                    print("help_pick_orange")
                    time.sleep(3.0)

                    self.robot.set_face("help_pick_red_wine")
                    print("help_pick_red_wine")
                    time.sleep(3.0)

                    self.robot.set_face("help_pick_mustard")
                    print("help_pick_mustard")
                    time.sleep(3.0) """
                    


                    """
                    self.robot.set_face("charmie_face_green")
                    print("charmie_face_green")
                    self.robot.set_speech(filename="sound_effects/you_have_to_pick_renata", show_in_face=True, wait_for_end_of=True)
                    time.sleep(5.0)
                    self.robot.set_face("charmie_face_green_my_order")
                    print("charmie_face_green_my_order")
                    time.sleep(5.0)        
                    self.robot.set_speech(filename="generic/found_following_items", show_in_face=True)
                    self.robot.set_speech(filename="objects_names/"+o.replace(" ","_").lower(), show_in_face=True, long_pause_show_in_face=True, \
                                          wait_for_end_of=True)
                    time.sleep(5.0)
                    self.robot.set_face("charmie_face_green_receptionist")
                    print("charmie_face_green_receptionist")
                    time.sleep(5.0)
                    self.robot.set_speech(filename="serve_breakfast/sb_finished", show_in_face=True, wait_for_end_of=True)
                    time.sleep(5.0)
                    self.robot.set_face("charmie_face_green_yes_no")
                    print("charmie_face_green_yes_no")
                    time.sleep(5.0)
                    self.robot.set_face("charmie_face_mixed")
                    print("charmie_face_mixed")
                    time.sleep(5.0)
                    self.robot.set_speech(filename="gpsr/stand_front_of_me_egpsr", show_in_face=True, wait_for_end_of=True)
                    self.robot.set_face("charmie_face_angry")
                    time.sleep(5.0)
                    print("charmie_face_angry")
                    time.sleep(5.0)
                    self.robot.set_speech(filename="generic/arrived", show_in_face=True, wait_for_end_of=True)
                    self.robot.set_speech(filename="generic/moving", show_in_face=True, long_pause_show_in_face=True, wait_for_end_of=True)
                    time.sleep(1.0)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)), \
                                          show_in_face=True, long_pause_show_in_face=True, wait_for_end_of=True)
                    time.sleep(5.0)
                    self.robot.set_face("charmie_face")
                    print("charmie_face")
                    time.sleep(5.0)
                    """

                if self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)) is not None:
                    self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)), wait_for_end_of=True)
                    time.sleep(1.0)
                    self.robot.set_speech(filename="generic/arrived", wait_for_end_of=False)
                    self.robot.set_speech(filename="furniture/"+self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o)), wait_for_end_of=True)
                else:
                    print("Wrong object name! Skipping speaking and navigation...")
                
                # self.robot.set_neck_coords(self.robot.get_location_coords_from_furniture(self.robot.get_furniture_from_object_class(self.robot.get_object_class_from_object(o))), 
                #                            wait_for_end_of=True)
                # self.robot.set_neck_coords(self.robot.get_location_coords_from_furniture("dishwasher"), 
                #                            wait_for_end_of=True)
                
                self.robot.get_detected_person_characteristics(first_sentence="demonstration/demo_characteristics_first_sentence", shirt_color=True, age=True)

                
                ##### SAVE SPEAK
                # current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                # self.robot.save_speech(command="This is just a test with a play command", filename=current_datetime, quick_voice=True, play_command=True, show_in_face=True, wait_for_end_of=True)

                ##### SPEAK: (repeats the command)
                # self.robot.set_speech(filename="temp/"+current_datetime, wait_for_end_of=True)

                self.robot.set_face("help_pick_milk")
                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_milk", command="", wait_for_end_of=True)
                time.sleep(3)

                # self.robot.set_speech(filename="generic/introduction_ful", command="", wait_for_end_of=True)

                self.robot.set_face("help_pick_orange_juice")
                time.sleep(3)

                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_orange_juice", show_in_face=True, wait_for_end_of=True)



                self.robot.set_face("help_pick_red_wine")
                self.robot.set_speech(filename="receptionist/favourite_drink/recep_drink_red_wine", wait_for_end_of=True)
                time.sleep(3)

                self.robot.set_face("help_pick_spoon")
                time.sleep(3)

                # next state
                # self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                self.state += 1
                print("Finished task!!!")

            else:
                pass