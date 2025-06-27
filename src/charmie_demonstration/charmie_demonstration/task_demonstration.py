#!/usr/bin/env python3
import rclpy
import threading
import time

from geometry_msgs.msg import Vector3, Pose2D
from charmie_interfaces.msg import PS4Controller
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
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
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

    # main state-machine function
    def main(self):
        
        # Task States
        self.Waiting_for_task_start = 0
        self.Demo_actuators_with_tasks = 1
        self.Introduction_demonstration = 5
        self.LLM_demonstration = 8
        self.Final_State = 10
        
        self.OFF = 0     # LOW  -> LOW
        self.FALLING = 1 # HIGH -> LOW
        self.RISING = 2  # LOW  -> HIGH
        self.ON = 3      # HIGH -> HIGH

        self.ON_AND_RISING = 2   # used with <= 
        self.OFF_AND_FALLING = 1 # used with >=

        # self.previous_message = False
        self.PREVIOUS_WATCHDOG_SAFETY_FLAG = True # just for RGB debug
        self.WATCHDOG_SAFETY_FLAG = True
        self.WATCHDOG_CUT_TIME = 1.5
        self.iteration_time = 0.01
        self.watchdog_timer_ctr = self.WATCHDOG_CUT_TIME/self.iteration_time

        self.motors_active = False
        self.omni_move = Vector3()
        self.torso_pos = Pose2D()

        self.start_time = time.time()

        self.task_state_selection = 0

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start
        
        while True:

            if self.state == self.Waiting_for_task_start:
                # Initialization State

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(CLEAR)
                    self.motors_active = False
                    self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    self.robot.node.torso_movement_publisher.publish(self.torso_pos)

                time.sleep(0.5)

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_speakers"]:
                    self.robot.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                    # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                # to initially set WATCHDOG TIMER FLAGS and RGB 
                ps4_controller, new_message = self.robot.get_controller_state()
                self.controller_watchdog_timer(new_message)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        self.motors_active = True
                        self.robot.activate_motors(activate=self.motors_active)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                        self.motors_active = False
                        self.robot.activate_motors(activate=self.motors_active)

                self.robot.set_task_state_selection(0)

                self.state = self.Demo_actuators_with_tasks


            elif self.state == self.Demo_actuators_with_tasks:

                ps4_controller, new_message = self.robot.get_controller_state()
                self.controller_watchdog_timer(new_message)

                if self.WATCHDOG_SAFETY_FLAG:
                    ps4_controller = PS4Controller() # cleans ps4_controller -> sets everything to 0

                # print(self.WATCHDOG_SAFETY_FLAG, self.PREVIOUS_WATCHDOG_SAFETY_FLAG)
                
                # WATCHDOG VERIFICATIONS
                if not self.WATCHDOG_SAFETY_FLAG and self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                        # only allow reactivation via PS button (safety)
                        # self.motors_active = True
                        # self.robot.activate_motors(activate=self.motors_active)

                elif self.WATCHDOG_SAFETY_FLAG and not self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    if ros2_modules["charmie_low_level"]:
                        self.robot.set_rgb(RED+HALF_ROTATE)
                    
                    self.safety_stop_modules()
    
                    if ros2_modules["charmie_speakers"]:
                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)


                # print(time.time()-self.start_time)
                if new_message and time.time()-self.start_time > 0.05:

                    print(time.time()-self.start_time)        
                    self.start_time = time.time()


                    # Activate motors: only activates if ps4 controller messages are being received
                    if ros2_modules["charmie_low_level"]:

                        # Watchdog Verifications
                        if ps4_controller.ps == self.RISING:
                            if not self.WATCHDOG_SAFETY_FLAG:
                                self.motors_active = not self.motors_active
                                self.robot.activate_motors(activate=self.motors_active)

                                self.torso_pos.x = 0.0
                                self.torso_pos.y = 0.0
                                self.robot.node.torso_movement_publisher.publish(self.torso_pos)

                                if self.motors_active:
                                    if ros2_modules["charmie_speakers"]:
                                        self.robot.set_speech(filename="demonstration/motors_unlocked", wait_for_end_of=False)
                                else:
                                    if ros2_modules["charmie_speakers"]:
                                        self.robot.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)
                        
                    
                    if ros2_modules["charmie_low_level"]: # Motors
                        # Robot Omni Movement
                        # left joy stick to control x and y movement (direction and linear speed) 
                        if ps4_controller.l3_dist >= 0.1:
                            self.omni_move.x = ps4_controller.l3_ang
                            if ps4_controller.l1 >= self.ON_AND_RISING and ps4_controller.r1 >= self.ON_AND_RISING: # if R1 and L1 are pressed, enters in TURBO MODE (MAX possible speed)
                                self.omni_move.y = ps4_controller.l3_dist*100 # max is *100 is you use higher it will limit to *100
                            else:
                                self.omni_move.y = ps4_controller.l3_dist*50 # max is *100 is you use higher it will limit to *100
                        else:
                            self.omni_move.x = 0.0
                            self.omni_move.y = 0.0

                        # right joy stick to control angular speed
                        if ps4_controller.r3_dist >= 0.1:
                            if ps4_controller.l1 >= self.ON_AND_RISING and ps4_controller.r1 >= self.ON_AND_RISING: # if R1 and L1 are pressed, enters in TURBO MODE (MAX possible speed)
                                self.omni_move.z = 100 + ps4_controller.r3_xx*100 # max is *100 is you use higher it will limit to *100
                            else:
                                self.omni_move.z = 100 + ps4_controller.r3_xx*25
                        else:
                            self.omni_move.z = 100.0
                        
                        if self.motors_active:
                            self.robot.node.omni_move_publisher.publish(self.omni_move)

                    if ros2_modules["charmie_low_level"]: # Torso
                        
                        pass
                        ### HAVE TO REVIEW BECAUSE THIS IS SENDING DATA EVERY LOOP!!!!
                        """
                        if self.motors_active:

                            # Torso Movement
                            if ps4_controller.arrow_up >= 2:
                                self.torso_pos.x = 1.0
                            elif ps4_controller.arrow_down >= 2:
                                self.torso_pos.x = -1.0
                            else:
                                self.torso_pos.x = 0.0

                            if ps4_controller.arrow_right >= 2:
                                self.torso_pos.y = 1.0
                            elif ps4_controller.arrow_left >= 2:
                                self.torso_pos.y = -1.0
                            else:
                                self.torso_pos.y = 0.0

                            self.robot.node.torso_movement_publisher.publish(self.torso_pos)
                        """

                    if ros2_modules["charmie_speakers"]: # Speak Introduction
                        if ps4_controller.r3 == self.RISING:
                            self.state = self.Introduction_demonstration

                    if ros2_modules["charmie_llm"] and ros2_modules["charmie_speakers"]: # LLM demo
                        if ps4_controller.l3 == self.RISING:
                            self.state = self.LLM_demonstration
    
                    # Task State Selection
                    if ros2_modules["charmie_ps4_controller"]:
                        
                        if len(self.robot.node.received_demo_tsi.list_of_states) != 0:
                            to_send = False
                            if ps4_controller.arrow_up == self.RISING:
                                self.task_state_selection -= 1
                                to_send = True
                            if ps4_controller.arrow_down == self.RISING:
                                self.task_state_selection += 1
                                to_send = True

                            temp_state_selection = self.task_state_selection % len(self.robot.node.received_demo_tsi.list_of_states)

                            if to_send:
                                self.robot.set_task_state_selection(temp_state_selection)

                            if ps4_controller.arrow_right == self.RISING:
                                self.robot.set_task_state_demo(new_demo_state=temp_state_selection)
                            
                            if ps4_controller.arrow_left == self.RISING:
                                self.robot.set_task_state_demo(new_demo_state=self.robot.node.received_demo_tsi.current_task_state_id+1)


                time.sleep(self.iteration_time)


            elif self.state == self.Introduction_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                # self.robot.set_speech(filename="generic/welcome_roboparty", wait_for_end_of=False)
                self.robot.set_speech(filename="demonstration/introduction_demo", wait_for_end_of=True)
                self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks

            elif self.state == self.LLM_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                if ros2_modules["charmie_llm"]:
                    self.robot.get_llm_demonstration(wait_for_end_of=True)
                    
                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks


                """
            elif self.state == self.Example_demonstration:
                
                temp_active_motors = self.motors_active
                self.safety_stop_modules()

                ### your code here

                self.robot.set_neck(self.look_forward_down, wait_for_end_of=True)
                self.robot.set_speech(filename="generic/ready_new_task", wait_for_end_of=True)
                # self.robot.set_speech(filename="generic/how_can_i_help", wait_for_end_of=True)

                self.motors_active = temp_active_motors
                self.robot.activate_motors(activate=self.motors_active)

                if ros2_modules["charmie_low_level"]:
                    if not self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(BLUE+HALF_ROTATE)
                    elif self.WATCHDOG_SAFETY_FLAG:
                        self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators_with_tasks
                """
            
            else:
                pass


    def controller_watchdog_timer(self, new_message):

        if new_message:
            self.watchdog_timer_ctr = 0
        else:
            self.watchdog_timer_ctr += 1

        self.PREVIOUS_WATCHDOG_SAFETY_FLAG = self.WATCHDOG_SAFETY_FLAG

        if self.watchdog_timer_ctr > (self.WATCHDOG_CUT_TIME/self.iteration_time):
            self.WATCHDOG_SAFETY_FLAG = True
        else:
            self.WATCHDOG_SAFETY_FLAG = False

    def safety_stop_modules(self):
        
        if ros2_modules["charmie_low_level"]:
            
            self.omni_move.x = 0.0
            self.omni_move.y = 0.0
            self.omni_move.z = 100.0
            self.robot.node.omni_move_publisher.publish(self.omni_move)

            self.motors_active = False
            self.robot.activate_motors(activate=self.motors_active)

            self.torso_pos.x = 0.0
            self.torso_pos.y = 0.0
            self.robot.node.torso_movement_publisher.publish(self.torso_pos)
