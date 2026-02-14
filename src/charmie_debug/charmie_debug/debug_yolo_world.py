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
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          True,
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

    def main(self):
        
        Prompt_free_yolo_world = 1
        # Search_for_objects = 2
        # Continuous_tracking = 3
        Final_State = 4

        # VARS ...
        self.state = Prompt_free_yolo_world

        print("IN NEW MAIN")

        # self.robot.set_initial_position([0.0, 0.0, 0.0])

        while True:

            if self.state == Prompt_free_yolo_world:

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_tv_prompt_head=True, text_prompts=["chair", "cabinet"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(2.0)

                # sends detected object to face
                world_obj_found = self.robot.node.detected_world_objects.objects
                while len(world_obj_found) < 1:
                    world_obj_found = self.robot.node.detected_world_objects.objects

                print("DONE")
                print(world_obj_found[0].object_name, 
                      world_obj_found[0].confidence, 
                      world_obj_found[0].position_absolute.x,
                      world_obj_found[0].position_absolute.y,
                      world_obj_found[0].position_absolute.z,
                      world_obj_found[0].room_location, 
                      world_obj_found[0].furniture_location, 
                      world_obj_found[0].object_class,
                      world_obj_found[0].orientation,
                      world_obj_found[0].cf_width,
                      world_obj_found[0].cf_length,
                      world_obj_found[0].cf_height,
                      world_obj_found[0].cf_shape,
                      world_obj_found[0].cf_can_pick,
                      world_obj_found[0].cf_std_pick
                )

                print("to face")
                self.robot.detected_object_to_face_path(object=world_obj_found[0], send_to_face=True)
                time.sleep(8.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_prompt_free_head=True, text_prompts=["chair", "cabinet"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_tv_prompt_head=True, text_prompts=["chair", "cabinet"], visual_prompts=["tomato_soup_dinner_table_head_cam", "mustard_dinning_table_lar_head_cam"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(2.0)

                # sends detected object to face
                world_obj_found = self.robot.node.detected_world_objects.objects
                while len(world_obj_found) < 1:
                    world_obj_found = self.robot.node.detected_world_objects.objects

                print("DONE")
                print(world_obj_found[0].object_name, 
                      world_obj_found[0].confidence, 
                      world_obj_found[0].position_absolute.x,
                      world_obj_found[0].position_absolute.y,
                      world_obj_found[0].position_absolute.z,
                      world_obj_found[0].room_location, 
                      world_obj_found[0].furniture_location, 
                      world_obj_found[0].object_class,
                      world_obj_found[0].orientation,
                      world_obj_found[0].cf_width,
                      world_obj_found[0].cf_length,
                      world_obj_found[0].cf_height,
                      world_obj_found[0].cf_shape,
                      world_obj_found[0].cf_can_pick,
                      world_obj_found[0].cf_std_pick
                )

                print("to face")
                # self.robot.detected_object_to_face_path(object=world_obj_found[0], send_to_face=True)
                time.sleep(8.0)

                """ ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_tv_prompt_head=True, text_prompts=["chair"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_tv_prompt_head=True, visual_prompts=["red_wine_dinner_table_head_cam", "bowl_dinner_table_head_cam"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_tv_prompt_hand=True, visual_prompts=["door_handle_lar_gripper_cam"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(activate_prompt_free_head=True, visual_prompts=["tomato_soup_dinner_table_head_cam", "mustard_dinning_table_lar_head_cam"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0)

                ### YOLO WORLD PROMPT FREE EXAMPLE ###
                t0 = time.perf_counter()
                print("PRE ACTIVATE")
                self.robot.activate_yolo_world(visual_prompts=["red_wine_dinner_table_head_cam", "bowl_dinner_table_head_cam"])
                print("ACTIVATE TRUE ", time.perf_counter()-t0)
                time.sleep(10.0) """
                
                """ # self.set_face(command="charmie_face")
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
                    time.sleep(4) """
                                
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