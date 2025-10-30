#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import math

import cv2
import numpy as np
import time
from skimage.metrics import structural_similarity as ssim

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  True,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          True,
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
    "charmie_neck":                 True,
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

# OLD DEBUG ROUTINE, most likely will not have all current functions, make sure all is updated to have the features you want to debug before running !!!
class TaskMain():

    def __init__(self, robot: RobotStdFunctions):
        # create a robot instance so use all standard CHARMIE functions
        self.robot = robot


    def configurables(self):

        #CONFIGURABLES

        #DEFINE PICK MODE AND OBJECT
        self.mode = "pick_front"
        self.SELECTED_OBJECT = "Pringles"
        
        #DEFINE NECK ANGLES AND TIMER
        self.tetas = [[0, 20], [0, 0], [0, -35]] #-> SEARCH CABINET ANGLES
        #self.tetas = [[-30, -45], [0, -45], [30, -45]] #-> SEARCH TABLE ANGLES

        #CAMERA CALIBRATION VARIABLES
        self.threshold = 0.85
        self.timeout = 10
        self.check_interval = 0.1
        self.stable_duration = 0.3
        
        #EXTRA
        self.speech = True

    def main(self):
        
        self.configurables()

        Search_for_objects = 1
        Final_State = 2


        # VARS ...

        self.state = Search_for_objects

        print("IN NEW MAIN")

        while True:
            
            if self.state == Search_for_objects:
                
                ### PICK OBJECT STANDARD FUNCTION ###
                #self.robot.pick_obj(selected_object=self.SELECTED_OBJECT, mode=self.mode, first_tetas = self.tetas)

                ### SEARCH FOR OBJECTS EXAMPLE ###
                
                # self.set_face(command="charmie_face")
                self.robot.set_neck(position=[0.0, 0.0], wait_for_end_of=True)

                time.sleep(2.0)
                
                #BEGIN SEARCH FOR OBJECT
                objects_found = self.robot.search_for_objects(tetas=self.tetas, time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.SELECTED_OBJECT], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                
                print("LIST OF DETECTED OBJECTS:")
                for o in objects_found:
                    conf = f"{o.confidence * 100:.0f}%"

                    cam_x_ = f"{o.position_relative.x:5.2f}"
                    cam_y_ = f"{o.position_relative.y:5.2f}"
                    cam_z_ = f"{o.position_relative.z:5.2f}"

                    print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({cam_x_},{cam_y_},{cam_z_})")

                    if o.object_name == self.SELECTED_OBJECT:

                        #CONFIRM FOUND OBJECT IF SPEECH = TRUE
                        if self.speech:
                            self.robot.set_speech(filename="generic/found_following_items", wait_for_end_of=True)
                            self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)
                        print(f"Initial pose to search for objects")

                        

                        #ADJUST FOR DIFFERENT HEIGHTS PICK FRONT IF SELECTED
                        if self.mode == "pick_front":

                            #SEND TO INITIAL POSITION PICK FRONT
                            self.robot.set_arm(command="initial_pose_to_search_table_front", wait_for_end_of=True)

                            #HEIGHT CALCULATIONS
                            high_z = (o.position_relative.z - 1.28)*100
                            rise_z = [high_z, 0.0, 0.0, 0.0, 0.0, 0.0]
                            low_z = (o.position_relative.z - 0.73)*100
                            descend_z = [low_z, 0.0, 0.0, 0.0, 0.0, 0.0]

                            #CHANGE ARM HEIGHT DEPENDING ON FOUND OBJECT HEIGHT
                            if 0.55 <= o.position_relative.z <= 1.0:
                                self.robot.set_arm(command="search_front_min_z", wait_for_end_of=True)
                                self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = descend_z, wait_for_end_of=True)
                                self.search_table_objects_hand()
                                if self.speech:
                                    self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)

                            elif 1.0 < o.position_relative.z <=1.6:
                                self.robot.set_arm(command="search_front_max_z", wait_for_end_of=True)
                                self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = rise_z, wait_for_end_of=True)
                                self.search_table_objects_hand()
                                if self.speech:
                                    self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=True)

                            elif 0.55 > o.position_relative.z or o.position_relative.z > 1.6:
                                self.robot.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                                self.robot.set_speech(filename="storing_groceries/cannot_reach_shelf", wait_for_end_of=True)

                        #BEGIN PICK TOP IF SELECTED
                        elif self.mode == "pick_top":
                            self.robot.set_arm(command="initial_pose_to_search_table_top", wait_for_end_of=True)
                            self.search_table_objects_hand()

                    else:
                        self.robot.set_speech(filename="generic/could_not_find_any_objects", wait_for_end_of=True)


                self.robot.set_rgb(CYAN+HALF_ROTATE)
                time.sleep(0.5)

                for o in objects_found:
                    path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(0,255,255))
                    time.sleep(4)
                                
                #NEXT STATE
                self.state = Final_State  

            elif self.state == Final_State:

                if self.speech:
                    print("Finished task")

                while True:
                    pass

            else:
                pass

    def search_table_objects_hand(self):
        table_objects = self.robot.search_for_objects(tetas=[[0, 0]], time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.SELECTED_OBJECT], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False)
        # print("LIST OF DETECTED OBJECTS:")
        # print(len(table_objects))
        for o in table_objects:
            ow = self.robot.get_object_length_from_object(o.object_name)
            conf = f"{o.confidence * 100:.0f}%"

            #SAVE NEW X,Y,Z
            hand_x_ = f"{o.position_cam.x:5.2f}"
            hand_y_ = f"{o.position_cam.y:5.2f}"
            hand_z_ = f"{o.position_cam.z:5.2f}"

            tf_x = 0.13
            tf_y = -0.006
            tf_z = -0.075

            print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} {o.orientation} ({hand_x_},{hand_y_},{hand_z_})")

            #CHANGE------------------------------------------------------------------------------------------------------------------------------------------------- correct_x doesnt have method yet
            correct_x = ((o.position_cam.x + ow - tf_x)*1000) - 150
            correct_y = (o.position_cam.y - tf_y)*1000
            correct_z = (o.position_cam.z - tf_z)*1000

            #CORRECT ROTATION CALCULATIONS
            if o.orientation < 0.0:
                correct_rotation = o.orientation +90.0
            else:
                correct_rotation = o.orientation -90.0

            #DEFINE AND CALCULATE KEY ARM POSITIONS
            object_position = [correct_z, -correct_y, correct_x, 0.0, 0.0, correct_rotation]
            final_position = [0.0, 0.0, 150.0, 0.0, 0.0, 0.0]
            security_position_front = [100.0*math.cos(math.radians(correct_rotation)), -100.0*math.sin(math.radians(correct_rotation)), -250.0, 0.0, 0.0, 0.0] #Rise the gripper in table orientation
            security_position_top = [0.00, 0.0, -150.0, 0.0, 0.0, 0.0] #Rise the gripper in table orientation
            object_reajust = [0.0, 0.0, 0.0, 0.0, 0.0, -correct_rotation]
            
            search_table_top_joints =	[-160.1, 57.5, -123.8, -87.3, 109.1, 69.5]
            search_table_front_joints =	[-215.0, -70.0, -16.0, 80.0, 30.0, 182.0]
            
            #IF OBJECT FOUND
            if o.object_name == self.SELECTED_OBJECT:

                #OPEN GRIPPER
                self.robot.set_arm(command="open_gripper", wait_for_end_of=True)

                #MOVE ARM IN THAT DIRECTION
                self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position, wait_for_end_of=True)
                self.wait_until_stable()

                #CALIBRATE GRIPPER BEFORE GRABBING
                final_objects = self.robot.search_for_objects(tetas=[[0, 0]], time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.SELECTED_OBJECT], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False)
                for obj in final_objects:
                    conf = f"{obj.confidence * 100:.0f}%"
                    hand_y_grab = f"{obj.position_cam.y:5.2f}"
                    hand_z_grab = f"{obj.position_cam.z:5.2f}"
                    correct_y_grab = (obj.position_cam.y - tf_y)*1000
                    correct_z_grab = (obj.position_cam.z - tf_z)*1000
                    print(f"{'BEFORE GRIP ID AND ADJUST:'+str(obj.index):<7} {obj.object_name:<17} {conf:<3} {obj.camera} ({hand_y_grab}, {hand_z_grab})")
                    object_position_grab = [correct_z_grab, -correct_y_grab, 0.0, 0.0, 0.0, 0.0]

                #APPLY ADJUSTEMENT BEFORE GRABBING
                self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_position_grab, wait_for_end_of=True)

                #MOVE ARM TO FINAL POSITION
                self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = final_position, wait_for_end_of=True)

                #CLOSE GRIPPER
                self.robot.set_arm(command="close_gripper", wait_for_end_of=True)

                #MOVE TO SAFE POSITION DEPENDING ON MODE SELECTED

                if self.mode == "pick_front":
                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = security_position_front, wait_for_end_of=True)
                    #MOVE TO SEARCH TABLE
                    self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_front_joints, wait_for_end_of=True)

                elif self.mode == "pick_top":
                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = security_position_top, wait_for_end_of=True)
                    self.robot.set_arm(command="adjust_move_tool_line", move_tool_line_pose = object_reajust, wait_for_end_of=True)
                    self.robot.set_arm(command="adjust_joint_motion", joint_motion_values = search_table_top_joints, wait_for_end_of=True)

                #MOVE ARM TO INITIAL POSITION
                self.robot.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                print(f"Bring object to initial pose")

            #IF AN OBJECT WAS NOT FOUND
            else:
                self.robot.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                print(f"Could not bring object to initial pose")

    def is_stable(self):

        #CONVERT IMG TO GRAYSCALE TO REDUCE LIGHT EFFECT
        prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(self.curr_frame, cv2.COLOR_BGR2GRAY)

        #CALCULATE DIFFERENCE SCORE FROM LAST TO CURRENT FRAME
        score, _ = ssim(prev_gray, curr_gray, full=True)
        print("Score", score)

        #RETURN BOOLEAN ON IF SCORE IS ABOVE THRESHOLD
        return score >= self.threshold

    def wait_until_stable(self):

        #INITIATE VARIABLES REPRESENTING TIMER
        image_time_out = 0.0
        stable_image = 0.0

        #STAY IN WHILE LOOP UNTIL CALIBRATION
        while (stable_image <= self.stable_duration) and (image_time_out < self.timeout):

            #GET FIRST FRAME
            _, self.prev_frame = self.robot.get_hand_rgb_image()
            print("Waiting for camera to stabilize...", image_time_out, stable_image)

            #WAIT INTERVAL
            time.sleep(self.check_interval)

            #GET SECOND FRAME TO COMPARE
            _, self.curr_frame = self.robot.get_hand_rgb_image()

            #IF IMAGES ARE CLOSE BASED ON THRESHOLD ADD TO STABLE TIMER, IF NOT RESET STABLE TIMER 
            if (self.is_stable()) and (image_time_out >= 0.5) :
                stable_image += 0.1
            else:
                stable_image = 0.0
            image_time_out += 0.1
