#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              True,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      True,
    "charmie_base_camera":      False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_ps4_controller":   False,
<<<<<<< HEAD
    "charmie_speakers":         False,
=======
    "charmie_speakers":         True,
>>>>>>> main
    "charmie_tracking":         False,
    "charmie_yolo_objects":     False,
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

    def main(self):
        
        Search_for_person = 1
        Search_for_objects = 2
        Continuous_tracking = 3
        Final_State = 4

        # VARS ...
<<<<<<< HEAD
        self.state = Search_for_objects
=======
        self.state = Search_for_person
>>>>>>> main

        print("IN NEW MAIN")

        while True:

            if self.state == Search_for_person:

                ### SEARCH FOR PERSON EXAMPLE ###
                
                # self.set_face(command="charmie_face")
                self.robot.set_neck(position=[0.0, 0.0], wait_for_end_of=True)

                time.sleep(2.0)

                tetas = [[-120, -10], [-60, -10], [0, -10], [60, -10], [120, -10]]
                people_found = self.robot.search_for_person(tetas=tetas, delta_t=2.0)

                print("FOUND:", len(people_found)) 
                for p in people_found:
                    print("ID:", p.index)
                time.sleep(0.5)

                # for p in people_found:
                #     path = self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                #     time.sleep(4)

                self.robot.set_rgb(CYAN+HALF_ROTATE)
                time.sleep(0.5)

                for p in people_found:
                    print(p.position_absolute.x, p.position_absolute.y, p.position_absolute_head.z)
                    path = self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                    # self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                    self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute_head.z], wait_for_end_of=True)
                    time.sleep(4)
                                
                # next state
                self.state = Final_State

            
            elif self.state == Search_for_objects:
                
                ### SEARCH FOR OBJECTS EXAMPLE ###
                
                # self.set_face(command="charmie_face")
                self.robot.set_neck(position=[0.0, 0.0], wait_for_end_of=True)

                time.sleep(2.0)

                # tetas = [[-120, -10], [-60, -10], [0, -10], [60, -10], [120, -10]]
                tetas = [[-30, -45], [0, -45], [30, -45]]
                # objects_found = self.robot.search_for_objects(tetas=tetas, delta_t=3.0, list_of_objects=["Milk", "Cornflakes"], list_of_objects_detected_as=[["cleanser"], ["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                objects_found = self.robot.search_for_objects(tetas=tetas, delta_t=3.0, list_of_objects=["Milk"], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                
                print("LIST OF DETECTED OBJECTS:")
                for o in objects_found:
                    conf = f"{o.confidence * 100:.0f}%"
                    '''x_ = f"{o.position_absolute.x:4.2f}"
                    y_ = f"{o.position_absolute.y:5.2f}"
                    z_ = f"{o.position_absolute.z:5.2f}"'''
                    cam_x_ = f"{o.position_cam.x:5.2f}"
                    cam_y_ = f"{o.position_cam.y:5.2f}"
                    cam_z_ = f"{o.position_cam.z:5.2f}"

                    print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({cam_x_},{cam_y_},{cam_z_})")
                    # ({x_}, {y_}, {z_})
                    if o.object_name == "Milk":
                        #self.robot.set_speech(filename="sound_effects/cr7_siuu", wait_for_end_of=True)
                        self.robot.set_speech(filename="generic/found_following_items", wait_for_end_of=True)
                        self.robot.set_speech(filename="objects_names/milk", wait_for_end_of=True)
                        self.robot.set_arm(command="initial_pose_to_search_table_front", wait_for_end_of=True)
                        print(f"Initial pose to search for objects")

                        self.search_table_objects_hand()
                        self.robot.set_speech(filename="objects_names/milk", wait_for_end_of=True)
                    else:
                        self.robot.set_speech(filename="generic/could_not_find_any_objects", wait_for_end_of=True)


                self.robot.set_rgb(CYAN+HALF_ROTATE)
                time.sleep(0.5)

                for o in objects_found:
                    path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(0,255,255))
                    time.sleep(4)
                                
                # next state
                self.state = Final_State

            
            elif self.state == Continuous_tracking:
                
                ### CONTINUOUS TRACKING EXAMPLE ###
                
                # self.robot.set_continuous_tracking_with_coordinates()
                self.robot.set_follow_person()

                # next state
                self.state = Final_State
            

            elif self.state == Final_State:
                print("Finished task")

                while True:
                    pass

            else:
                pass

    def search_table_objects_hand(self):
        table_objects = self.robot.search_for_objects(tetas=[[0, 0]], delta_t=2.0, list_of_objects=["Milk"], use_arm=False, detect_objects=False, detect_objects_hand=True, detect_objects_base=False)
        # print("LIST OF DETECTED OBJECTS:")
        # print(len(table_objects))
        for o in table_objects:
            conf = f"{o.confidence * 100:.0f}%"
            #SAVE NEW X,Y,Z
            hand_x_ = f"{o.position_cam.x:5.2f}"
            hand_y_ = f"{o.position_cam.y:5.2f}"
            hand_z_ = f"{o.position_cam.z:5.2f}"

            print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({hand_x_},{hand_y_},{hand_z_})")
            if o.object_name == "Milk":
                #OPEN GRIPPER
                self.robot.set_arm(command="open_gripper", wait_for_end_of=True)
                #MOVE ARM IN THAT DIRECTION

                #CLOSE GRIPPER
                self.robot.set_arm(command="close_gripper", wait_for_end_of=True)
                #MOVE ARM TO INITIAL POSITION
                self.robot.set_arm(command="search_table_to_initial_pose", wait_for_end_of=True)
                print(f"Bring object to initial pose")
            else:
                # self.robot.set_arm(command="search_table_front_to_initial_pose", wait_for_end_of=True)
                print(f"Could not bring object to initial pose")

