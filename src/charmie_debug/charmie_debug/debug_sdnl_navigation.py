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
    "charmie_head_camera":          False,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         True,
    "charmie_low_level":            True,
    "charmie_navigation":           True,
    "charmie_nav2":                 True,
    "charmie_nav_sdnl":             False,
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

    def main(self):
        Waiting_for_start_button = 0
        Move_to_pre_pick_position_after_search_for_objects = 1
        Final_State = 2
        
        # self.initial_position = [0.0, 0.0, 0.0]
        self.initial_position = [2.0, -3.80, 90.0] # temp (near Tiago desk for testing)
        # self.initial_position = [2.5, -4.50, 0.0] # temp (near Tiago desk for testing)
        # self.initial_position = [2.6, -3.60, 45.0] # temp (near Tiago desk for testing)
        # self.initial_position = [0.0, 0.0, 0.0] # temp (near Tiago desk for testing)
        self.NAVIGATION_TARGET = "Dinner Table"
        self.NAVIGATION_TARGET = self.NAVIGATION_TARGET.replace(" ","_").lower()
        self.NAVIGATION_TARGET2 = "desk"
        self.NAVIGATION_TARGET3 = "Left Lounge Chair"
        self.NAVIGATION_TARGET4 = [ 2.0,  2.0,   0.0]
        self.NAVIGATION_TARGET5 = [ 2.0, -2.0,   0.0]
        self.NAVIGATION_TARGET6 = [ 0.0, -0.0,  90.0]

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [-90, 0]
        
        # VARS ...
        self.state = Move_to_pre_pick_position_after_search_for_objects

        self.robot.set_rgb(RED+BACK_AND_FORTH_8)


        while True:

            if self.state == Waiting_for_start_button:
                # your code here ...

                # self.robot.set_initial_position(self.initial_position)
                # self.robot.wait_for_start_button()

                """ self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET4, reached_radius=1, wait_for_end_of=False)
                # self.robot.sdnl_move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET), wait_for_end_of=False)
                time.sleep(3.0)

                self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET5, reached_radius=1, wait_for_end_of=False)
                time.sleep(3.0)
                self.robot.sdnl_move_to_position_cancel()
                time.sleep(5.0)

                self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET4, reached_radius=1, wait_for_end_of=False)
                # self.robot.sdnl_move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET), wait_for_end_of=False)
                time.sleep(2.0)

                while not self.robot.sdnl_move_to_position_is_done():
                    print("Waiting...")
                    time.sleep(0.5)
                    pass
                print("DONE")
                time.sleep(5.0) """

                while True:

                    self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET4, ignore_obstacles=True, first_rotate=False, orient_after_move=False, reached_radius=0.5, print_feedback=True, wait_for_end_of=True)
                    print("Moved to target 4")
                    # time.sleep(10.0)
                    self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET5, ignore_obstacles=True, first_rotate=False, orient_after_move=False, reached_radius=0.5, print_feedback=True, wait_for_end_of=True)
                    print("Moved to target 5")
                    # time.sleep(10.0)
                    self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET6, ignore_obstacles=True, first_rotate=False, orient_after_move=False, reached_radius=0.5, print_feedback=True, wait_for_end_of=True)
                    print("Moved to target 6")
                    # time.sleep(10.0)
               

                # to test move_to_pre_pick_position_after_search_for_objects std_function
                o = DetectedObject()
                o.position_absolute.x = 5.55 -0.2
                o.position_absolute.y = 1.78 -0.2
                print(self.robot.move_to_pre_pick_position_after_search_for_objects(
                    furniture="dinner table", object=o, approach_offset=0.5))

                while True:
                    pass

                # self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET5, first_rotate=False, reached_radius=0.8, print_feedback=True, wait_for_end_of=True)
                # self.robot.sdnl_move_to_position(move_coords=self.NAVIGATION_TARGET5, first_rotate=False, reached_radius=0.8, print_feedback=True, wait_for_end_of=True)
                
                # self.robot.sdnl_move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET), wait_for_end_of=False)
                # time.sleep(2.0)

                # next state
                # self.state = Move_to_location1

            elif self.state == Move_to_pre_pick_position_after_search_for_objects:
                print('State 1 = Move to pre pick position after search for objects')

                ### MOVE TO DINNER TABLE POSITION TO SEARCH FOR OBJECTS

                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAVIGATION_TARGET, wait_for_end_of=False)

                # must be removed after the update to minimize as much as possivle the final orientation error 
                move_coords = self.robot.get_navigation_coords_from_furniture(self.NAVIGATION_TARGET)                
                # move_coords = self.robot.add_rotation_to_pick_position(move_coords=move_coords)                
                self.robot.move_to_position(move_coords=move_coords, wait_for_end_of=True)

                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/"+self.NAVIGATION_TARGET, wait_for_end_of=True)

                ### SEARCH FOR OBJECTS EXAMPLE     
                
                # tetas = [[-120, -10], [-60, -10], [0, -10], [60, -10], [120, -10]]
                # tetas = [[-30, -45], [0, -45], [30, -45]]
                tetas = [[0, -30]]
                # objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=3.0, list_of_objects=["Milk", "Cornflakes"], list_of_objects_detected_as=[["cleanser"], ["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=2.0, use_arm=False, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                
                print("LIST OF DETECTED OBJECTS:")
                for o in objects_found:
                    conf = f"{o.confidence * 100:.0f}%"
                    x_ = f"{o.position_absolute.x:4.2f}"
                    y_ = f"{o.position_absolute.y:5.2f}"
                    z_ = f"{o.position_absolute.z:5.2f}"
                    print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.furniture_location} ({x_}, {y_}, {z_})")

                time.sleep(0.5)
                self.robot.set_face("charmie_face", wait_for_end_of=False)
                    
                if objects_found:
                    self.robot.set_neck(position=self.look_judge, wait_for_end_of=False)
                    self.robot.set_speech(filename="generic/found_the", wait_for_end_of=True)
                    for o in objects_found:
                        path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(0,255,255))
                        self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                        time.sleep(3)
                                

                    for o in objects_found:
                        if o.furniture_location.replace(" ","_").lower() == self.NAVIGATION_TARGET.replace(" ","_").lower():
                            
                            self.robot.set_face("charmie_face", wait_for_end_of=False)
                            self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)
                            self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                            self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=False)
                            
                            self.robot.move_to_pre_pick_position_after_search_for_objects(furniture=self.NAVIGATION_TARGET, object=o, approach_offset=0.5, wait_for_end_of=True)

                            self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                            self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                            path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(0,255,255))
                            self.robot.set_neck_coords(position=[o.position_absolute.x, o.position_absolute.y, o.position_absolute.z], wait_for_end_of=True)
                            time.sleep(3.0)

                
                # self.robot.set_neck(position=self.look_table_objects, wait_for_end_of=False)

                # while True:
                #     print(self.robot.get_robot_localization())
                #     time.sleep(0.5)
                #     pass    

                # next state
                self.state = Final_State
            
            elif self.state == Final_State:

                self.state += 1
                print("Finished task!!!")

            else:
                pass