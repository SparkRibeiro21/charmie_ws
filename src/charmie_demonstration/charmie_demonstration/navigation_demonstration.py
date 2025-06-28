#!/usr/bin/env python3
import rclpy
import threading
import time

from geometry_msgs.msg import Twist
from charmie_interfaces.msg import GamepadController
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              False,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      False,
    "charmie_base_camera":      False,
    "charmie_gamepad":          True,
    "charmie_lidar":            False,
    "charmie_lidar_bottom":     False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_speakers":         True,
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

    # main state-machine function
    def main(self):
        
        # Task States
        self.Waiting_for_task_start = 0
        self.Demo_actuators_with_tasks = 1
        self.Final_State = 10

        self.TIMEOUT_FLAG = True
        self.MOTORS_ACTIVE_FLAG = True

        self.MAX_LINEAR_SPEED = 0.40  # m/s
        self.MAX_ANGULAR_SPEED = 0.60  # rad/s

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start
        
        while True:

            if self.state == self.Waiting_for_task_start:
                # Initialization State

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_speakers"]:
                    self.robot.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                    # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                # to initially set TIMEOUT FLAGS and RGB 
                gamepad_controller, new_message = self.robot.update_gamepad_controller_state()
                if self.robot.get_gamepad_timeout(self.robot.OFF):
                    self.TIMEOUT_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        self.MOTORS_ACTIVE_FLAG = True
                        self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)
                else:
                    self.TIMEOUT_FLAG = True
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                        self.MOTORS_ACTIVE_FLAG = False
                        self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Demo_actuators_with_tasks:

                gamepad_controller, new_message = self.robot.update_gamepad_controller_state()
                
                # WATCHDOG VERIFICATIONS
                if self.robot.get_gamepad_timeout(self.robot.FALLING):
                    self.TIMEOUT_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(MAGENTA+HALF_ROTATE)
                        # only allow reactivation of motors via LOGO button (SAFETY)
                        # self.MOTORS_ACTIVE_FLAG = True
                        # self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                if self.robot.get_gamepad_timeout(self.robot.RISING):
                    self.TIMEOUT_FLAG = True
                    self.MOTORS_ACTIVE_FLAG = False
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                    
                    self.safety_stop_modules()
    
                    if ros2_modules["charmie_speakers"]:
                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)

                
                if new_message and not self.TIMEOUT_FLAG: # if we receive a new message and there is no timeout
                    
                    # Activate motors: only activates if gamepad controller messages are being received
                    if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_LOGO, self.robot.RISING):

                        if ros2_modules["charmie_low_level"]:

                            self.MOTORS_ACTIVE_FLAG = not self.MOTORS_ACTIVE_FLAG
                            self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)

                            if self.MOTORS_ACTIVE_FLAG:
                                self.robot.set_rgb(BLUE+HALF_ROTATE)
                                if ros2_modules["charmie_speakers"]:
                                    self.robot.set_speech(filename="demonstration/motors_unlocked", wait_for_end_of=False)
                            else:
                                self.robot.set_rgb(MAGENTA+HALF_ROTATE)
                                if ros2_modules["charmie_speakers"]:
                                    self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)
                            
                    if self.MOTORS_ACTIVE_FLAG: # robot is fully operational 

                        if ros2_modules["charmie_low_level"]: # Motors

                            cmd_vel = Twist()

                            if self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L1, self.robot.ON) and self.robot.get_gamepad_button_pressed(self.robot.BUTTON_R1, self.robot.ON):
                                ang_speed_percentage = 50
                                lin_speed_percentage = 100
                            elif self.robot.get_gamepad_button_pressed(self.robot.BUTTON_L1, self.robot.ON):
                                ang_speed_percentage = 50
                                lin_speed_percentage = 75
                            else:
                                ang_speed_percentage = 50
                                lin_speed_percentage = 50

                            cmd_vel.linear.x =  float(self.percentage_to_linear_speed(  self.robot.get_gamepad_axis(self.robot.AXIS_L3_YY), lin_speed_percentage))
                            cmd_vel.linear.y =  float(self.percentage_to_linear_speed( -self.robot.get_gamepad_axis(self.robot.AXIS_L3_XX), lin_speed_percentage))
                            cmd_vel.angular.z = float(self.percentage_to_angular_speed(-self.robot.get_gamepad_axis(self.robot.AXIS_R3_XX), ang_speed_percentage))
                            
                            self.robot.node.cmd_vel_publisher.publish(cmd_vel)

                time.sleep(0.05)
            

            else:
                pass

    def safety_stop_modules(self):
        # Add here, all safety restrictions for the safety stop
        
        if ros2_modules["charmie_low_level"]:

            cmd_vel = Twist()
            self.robot.node.cmd_vel_publisher.publish(cmd_vel)

            self.MOTORS_ACTIVE_FLAG = False
            self.robot.activate_motors(activate=self.MOTORS_ACTIVE_FLAG)
            
    def percentage_to_angular_speed(self, value, percentage):
        # Convert percentage to angular speed
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100

        return value * percentage/100 * self.MAX_ANGULAR_SPEED
    
    def percentage_to_linear_speed(self, value, percentage):
        # Convert percentage to angular speed
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100

        return value * percentage/100 * self.MAX_LINEAR_SPEED