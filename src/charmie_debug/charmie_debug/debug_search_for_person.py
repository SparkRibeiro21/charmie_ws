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
    "charmie_arm":              False,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      True,
    "charmie_hand_camera":      False,
    "charmie_base_camera":      False,
    "charmie_gamepad":          False,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_speakers":         True,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     False,
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

    def main(self):
        
        Search_for_person = 1
        Search_for_objects = 2
        Continuous_tracking = 3
        Final_State = 4

        # VARS ...
        self.state = Search_for_person

        print("IN NEW MAIN")

        # self.robot.set_initial_position([0.0, 0.0, 0.0])

        while True:

            if self.state == Search_for_person:

                ### SEARCH FOR PERSON EXAMPLE ###
                
                # self.set_face(command="charmie_face")
                self.robot.set_neck(position=[0.0, 0.0], wait_for_end_of=True)

                time.sleep(2.0)

                tetas = [[-120, -10], [-60, -10], [0, -10], [60, -10], [120, -10]]
                people_found = self.robot.search_for_person(tetas=tetas, time_in_each_frame=2.0)

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
                    print(p.position_absolute.x, p.position_absolute.y, p.position_absolute.z)
                    path = self.robot.detected_person_to_face_path(person=p, send_to_face=True)
                    # self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y], ang=-10, wait_for_end_of=True)
                    # self.robot.set_neck_coords(position=[p.position_absolute.x, p.position_absolute.y, p.position_absolute.z], wait_for_end_of=True)
                    self.robot.set_neck_coords(position=[p.position_absolute_head.x, p.position_absolute_head.y, p.position_absolute_head.z], wait_for_end_of=True)
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
                # objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=3.0, list_of_objects=["Milk", "Cornflakes"], list_of_objects_detected_as=[["cleanser"], ["strawberry_jello", "chocolate_jello"]], use_arm=False, detect_objects=True, detect_furniture=False)
                objects_found = self.robot.search_for_objects(tetas=tetas, time_in_each_frame=2.0, use_arm=False, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                
                print("LIST OF DETECTED OBJECTS:")
                for o in objects_found:
                    conf = f"{o.confidence * 100:.0f}%"
                    x_ = f"{o.position_absolute.x:4.2f}"
                    y_ = f"{o.position_absolute.y:5.2f}"
                    z_ = f"{o.position_absolute.z:5.2f}"
                    print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({x_}, {y_}, {z_})")
                    
                self.robot.set_rgb(CYAN+HALF_ROTATE)
                time.sleep(0.5)

                if objects_found:
                    self.robot.set_speech(filename="generic/found_the", wait_for_end_of=True)
                    for o in objects_found:
                        path = self.robot.detected_object_to_face_path(object=o, send_to_face=True, bb_color=(0,255,255))
                        self.robot.set_speech(filename="objects_names/"+o.object_name.replace(" ","_").lower(), wait_for_end_of=False)
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
                
                self.robot.set_face("charmie_face")
                self.robot.set_neck(position=[0.0, 0.0], wait_for_end_of=False)
                print("Finished task")

                while True:
                    pass

            else:
                pass