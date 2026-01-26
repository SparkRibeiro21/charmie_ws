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

    def configurables(self):
        self.tetas = [[0, 20], [0, 0], [0, -35]]
        self.SELECTED_OBJECT = "Cola"

    # main state-machine function
    def main(self):

        self.configurables(self)
        
        # States in DebugMoveit Task
        self.Waiting_for_task_start = 0
        self.test_state_1 = 1
        self.Final_State = 2

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start


        # debug print to know we are on the main start of the task
        print("In DebugMoveit Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")

                # front_pick_joints = [-3.7524, -1.2217, -0.2793, 1.3963, 0.5236, 3.1765]

                # self.robot.set_joint_target_arm(front_pick_joints, wait_for_end_of=True)

                objects_found = self.robot.search_for_objects(tetas=self.tetas, time_in_each_frame=3.0, time_wait_neck_move_pre_each_frame=0.5, list_of_objects=[self.SELECTED_OBJECT], use_arm=True, detect_objects=True, detect_objects_hand=False, detect_objects_base=False)
                
                print("LIST OF DETECTED OBJECTS:")
                for o in objects_found:
                    conf = f"{o.confidence * 100:.0f}%"

                    cam_x_ = f"{o.position_relative.x:5.2f}"
                    cam_y_ = f"{o.position_relative.y:5.2f}"
                    cam_z_ = f"{o.position_relative.z:5.2f}"

                    print(f"{'ID:'+str(o.index):<7} {o.object_name:<17} {conf:<3} {o.camera} ({cam_x_},{cam_y_},{cam_z_})")

                    if o.object_name == self.SELECTED_OBJECT:

                        self.robot.set_pose_target_arm(
                            o.position_relative.x,
                            o.position_relative.y,
                            o.position_relative.z,
                            0.0,
                            0.0,
                            0.0,
                            cartesian=False,
                        )

                    time.sleep(2.0)

                    self.robot.set_named_target_arm("home", wait_for_end_of=True)

                pass
                    
                    

            elif self.state == self.Final_State:
                
                print("State:", self.state, "- Final_State")

                # Lock after finishing task
                while True:
                    pass

            else:
                pass
