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
    "charmie_lidar":            True,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       True,
    "charmie_neck":             True,
    "charmie_obstacles":        True,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         False,
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
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7
        
        # self.initial_position = [-2.5, 1.5, 0]
        self.initial_position = [0.0, 0.1, 0.0]

        # navigation positions
        self.front_of_sofa = [-2.5, 1.5]
        self.sofa = [-2.5, 3.0]

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        
        # VARS ...
        self.state = Waiting_for_start_button

        while True:

            if self.state == Waiting_for_start_button:
                # your code here ...

                # If initial position is inside while loop you are telling the robot the wrong localisation.
                # This command must only be sent once, at the start of the task
                self.robot.set_initial_position(self.initial_position)

                self.robot.activate_obstacles(obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=True)

                self.robot.wait_for_start_button()
                
                self.robot.set_neck(position=self.look_navigation, wait_for_end_of=False)

                self.robot.wait_for_door_start()

                # time.sleep(5)
                self.robot.set_neck(position=[0-0, -50.0], wait_for_end_of=True)

                # next state
                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                print('State 1 = Hand Raising Detect')

                # time.sleep(5)
                # time.sleep(3)
                
                # self.set_navigation(movement="move", target=[-2.0, 2.1], max_speed=10, reached_radius=0.5, flag_not_obs=False, wait_for_end_of=True)
                self.robot.set_navigation(movement="rotate", target=[-0.5, 1.5], wait_for_end_of=True)
                self.robot.set_navigation(movement="move", target=[-0.5, 1.5], max_speed=15, reached_radius=0.5, flag_not_obs=False, wait_for_end_of=False)

                """
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=0.0, wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=180.0, wait_for_end_of=True)
                # time.sleep(1.0)
                self.set_navigation(movement="adjust_angle", absolute_angle=5.0, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                # time.sleep(1.0)
                
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=0.0, wait_for_end_of=True)
                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=180.0, wait_for_end_of=True)
                # time.sleep(1.0)
                self.set_navigation(movement="adjust_angle", absolute_angle=5.0, flag_not_obs=True, wait_for_end_of=True)
                # self.set_navigation(movement="adjust_angle", absolute_angle=0.0, flag_not_obs=True, wait_for_end_of=True)
                """

                while True:
                    print(self.robot.get_robot_localization())
                    time.sleep(0.5)
                    pass    

                # your code here ...
                                
                # next state
                # self.state = Final_State
            
            elif self.state == Final_State:

                self.state += 1
                print("Finished task!!!")

            else:
                pass