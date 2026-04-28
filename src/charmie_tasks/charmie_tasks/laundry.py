#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions


ros2_modules = {
    "charmie_arm":                  True,
    "charmie_audio":                False,
    "charmie_face":                 True,
    "charmie_head_camera":          True,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                True,
    "charmie_lidar_bottom":         True,
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
    "charmie_yolo_objects":         False,
    "charmie_yolo_pose":            False,
    "charmie_yolo_world":           False,
}


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

        self.robot = robot

        self.TASK_NAME = "Laundry"

        self.task_states = {

            "Waiting_for_task_start":          0,

            "Navigate_to_table":               1,
            "Place_CFM_on_table":              2,

            "Navigate_to_laundry_area":        3,
            "Approach_washing_machine":        4,

            "Open_washing_machine":            5,

            "Pick_clothes":                    6,
            "Place_clothes_in_basket":         7,

            "Close_washing_machine":           8,

            "Pick_basket":                     9,
            "Move_to_table":                   10,

            "Place_basket_near_table":         11,

            "Pick_from_basket":                12,
            "Place_clothes_on_CFM":            13,

            "Ask_arbitrator_placement_on_CFM": 14,
            "Confirm_clothes_on_CFM":          15,

            "Activate_CFM_Mode":               16,
            "Retrieve_folded_clothes":         17,

            "Stack_clothes":                   18,

            "Check_more_clothes":              19,

            "Final_State":                     20,
        }


    def configurables(self):

        self.initial_position = [0.0, 0.0, 0.0]

        # Name of the table where breakfast is served
        self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED = "Dinner Table"
        self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED = self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED.lower().replace(" ", "_")

    def main(self):

        self.configurables()

        self.robot.set_task_name_and_states(
            task_name=self.TASK_NAME,
            task_states=self.task_states
        )

        self.DEMO_MODE = self.robot.get_demo_mode()
        self.DEMO_STATE = -1

        self.state = self.task_states["Waiting_for_task_start"]

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.search_tetas = [[-45, -35], [-45+20, -35+10], [-45-20, -35+10]] # , [-45-10, -45-5], [-45+10, -45-5]]

        print("IN " + self.TASK_NAME.upper() + " MAIN")

        while True:

            self.robot.set_current_task_state_id(current_state=self.state)

            if self.state == self.task_states["Waiting_for_task_start"]:

                self.robot.set_initial_position(self.initial_position)
                self.robot.set_face("charmie_face", wait_for_end_of=False)
                self.robot.set_neck(position=[0, 0], wait_for_end_of=False)
                # self.robot.set_speech(filename="laundry/start_laundry_task", wait_for_end_of=True)
                self.robot.wait_for_start_button()

                self.state = self.task_states["Navigate_to_table"]


            elif self.state == self.task_states["Navigate_to_table"]:

                self.robot.set_neck(position=[0, -30], wait_for_end_of=False)
                self.robot.set_speech(filename="generic/moving", wait_for_end_of=False)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED, wait_for_end_of=False)
                self.robot.move_to_position(move_coords=self.robot.get_navigation_coords_from_furniture(self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED), wait_for_end_of=True)
                self.robot.set_speech(filename="generic/arrived", wait_for_end_of=True)
                self.robot.set_speech(filename="furniture/"+self.NAME_TABLE_WHERE_LAUNDRY_IS_FOLDED, wait_for_end_of=False)

                self.state = self.task_states["Place_CFM_on_table"]


            elif self.state == self.task_states["Place_CFM_on_table"]:

                _ , _ , furniture_distance = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45)
                self.adjust_x_      = furniture_distance - 0.04 
                s,m = self.adjust_omnidirectional_position(dx = self.adjust_x_, dy = self.adjust_y_, wait_for_end_of=False)
                self.robot.adjust_omnidirectional_position

                self.state = self.task_states["Navigate_to_laundry_area"]


            elif self.state == self.task_states["Navigate_to_laundry_area"]:

                self.state = self.task_states["Approach_washing_machine"]


            elif self.state == self.task_states["Approach_washing_machine"]:

                self.state = self.task_states["Open_washing_machine"]


            elif self.state == self.task_states["Open_washing_machine"]:

                self.state = self.task_states["Pick_clothes"]


            elif self.state == self.task_states["Pick_clothes"]:

                self.state = self.task_states["Place_clothes_in_basket"]


            elif self.state == self.task_states["Place_clothes_in_basket"]:

                self.state = self.task_states["Close_washing_machine"]


            elif self.state == self.task_states["Close_washing_machine"]:

                self.state = self.task_states["Pick_basket"]


            elif self.state == self.task_states["Pick_basket"]:

                self.state = self.task_states["Move_to_table"]


            elif self.state == self.task_states["Move_to_table"]:

                self.state = self.task_states["Place_basket_near_table"]


            elif self.state == self.task_states["Place_basket_near_table"]:

                self.state = self.task_states["Pick_from_basket"]


            elif self.state == self.task_states["Pick_from_basket"]:

                self.state = self.task_states["Place_clothes_on_CFM"]


            elif self.state == self.task_states["Place_clothes_on_CFM"]:

                self.state = self.task_states["Ask_arbitrator_placement_on_CFM"]


            elif self.state == self.task_states["Ask_arbitrator_placement_on_CFM"]:

                self.state = self.task_states["Confirm_clothes_on_CFM"]


            elif self.state == self.task_states["Confirm_clothes_on_CFM"]:

                self.state = self.task_states["Activate_CFM_Mode"]


            elif self.state == self.task_states["Activate_CFM_Mode"]:

                self.state = self.task_states["Retrieve_folded_clothes"]


            elif self.state == self.task_states["Retrieve_folded_clothes"]:

                self.state = self.task_states["Stack_clothes"]


            elif self.state == self.task_states["Stack_clothes"]:

                self.state = self.task_states["Check_more_clothes"]


            elif self.state == self.task_states["Check_more_clothes"]:

                self.state = self.task_states["Final_State"]


            elif self.state == self.task_states["Final_State"]:

                while True:
                    pass


            else:
                pass


            # DEMO MODE
            if self.state == self.DEMO_STATE:
                while not self.robot.get_received_new_demo_task_state():
                    time.sleep(1.0)
                self.state = self.robot.get_new_demo_task_state()

            elif self.DEMO_MODE:
                self.state = self.DEMO_STATE