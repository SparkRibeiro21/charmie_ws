#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject, DetectedPerson, PS4Controller
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
    "charmie_lidar":            False,
    "charmie_localisation":     False,
    "charmie_low_level":        True,
    "charmie_navigation":       False,
    "charmie_neck":             True,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
    "charmie_point_cloud":      False,
    "charmie_ps4_controller":   True,
    "charmie_speakers":         False,
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
        self.Demo_actuators = 1
        self.Final_State = 10
        
        # Neck Positions
        self.look_forward = [0, 0]
        # self.look_navigation = [0, -30]
        # self.look_judge = [45, 0]
        # self.look_table_objects = [-45, -45]
        # self.look_tray = [0, -60]

        self.OFF = 0     # LOW  -> LOW
        self.FALLING = 1 # HIGH -> LOW
        self.RISING = 2  # LOW  -> HIGH
        self.ON = 3      # HIGH -> HIGH

        self.ON_AND_RISING = 2   # used with <= 
        self.OFF_AND_FALLING = 1 # used with >=

        self.neck_pos_pan = self.look_forward[0]
        self.neck_pos_tilt = self.look_forward[1]

        # self.previous_message = False
        self.PREVIOUS_WATCHDOG_SAFETY_FLAG = True # just for RGB debug
        self.WATCHDOG_SAFETY_FLAG = True
        self.WATCHDOG_CUT_TIME = 1.0
        self.iteration_time = 0.025
        self.watchdog_timer_ctr = self.WATCHDOG_CUT_TIME/self.iteration_time

        # Start localisation position
        # self.initial_position = [-1.0, 1.5, -90.0]

        # navigation positions
        # self.front_of_sofa = [-2.5, 1.5]
        # self.sofa = [-2.5, 3.0]
        
        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start

        while True:

            if self.state == self.Waiting_for_task_start:
                # Initialization State

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(CLEAR)

                if ros2_modules["charmie_neck"]:
                    self.robot.set_neck(self.look_forward, wait_for_end_of=True)

                if ros2_modules["charmie_low_level"]:
                    self.robot.set_rgb(RAINBOW_ROT)

                time.sleep(1.0)

                # to initially set WATCHDOG TIMER FLAGS and RGB 
                ps4_controller, new_message = self.robot.get_controller_state()

                self.controller_watchdog_timer(new_message)

                if not self.WATCHDOG_SAFETY_FLAG:
                    self.robot.set_rgb(BLUE+HALF_ROTATE)
                elif self.WATCHDOG_SAFETY_FLAG:
                    self.robot.set_rgb(RED+HALF_ROTATE)

                self.state = self.Demo_actuators

            elif self.state == self.Demo_actuators:

                ps4_controller, new_message = self.robot.get_controller_state()

                self.controller_watchdog_timer(new_message)

                if self.WATCHDOG_SAFETY_FLAG:
                    ps4_controller = PS4Controller() # cleans ps4_controller -> sets everything to 0


                # just for RGB visual debug  
                if not self.WATCHDOG_SAFETY_FLAG and self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    self.robot.set_rgb(BLUE+HALF_ROTATE)
                elif self.WATCHDOG_SAFETY_FLAG and not self.PREVIOUS_WATCHDOG_SAFETY_FLAG:
                    self.robot.set_rgb(RED+HALF_ROTATE)

                # self.previous_message = new_message

                # print(ps4_controller)
                # print(self.neck_pos_pan, self.neck_pos_tilt, ps4_controller.triangle, ps4_controller.cross)

                # if ps4_controller.r1 >= self.ON_AND_RISING:
                #     self.robot.set_rgb(GREEN+HALF_ROTATE)
                # else:
                #     self.robot.set_rgb(MAGENTA+HALF_ROTATE)

                if ros2_modules["charmie_neck"]:
                    
                    # circle and square to move neck left and right
                    # triangle and cross to move the neck up and down
                    neck_inc_hor = 2
                    neck_inc_ver = 1
                    if ps4_controller.circle >= self.ON_AND_RISING:
                        self.neck_pos_pan -= neck_inc_hor
                        if self.neck_pos_pan < -180:
                            self.neck_pos_pan = -180
                        self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        
                    elif ps4_controller.square >= self.ON_AND_RISING:
                        self.neck_pos_pan += neck_inc_hor
                        if self.neck_pos_pan > 180:
                            self.neck_pos_pan = 180
                        self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        # print(self.neck_pos)

                    if ps4_controller.cross >= self.ON_AND_RISING:
                        self.neck_pos_tilt -= neck_inc_ver
                        print("DOWN")
                        if self.neck_pos_tilt < -60:
                            self.neck_pos_tilt = -60
                        self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        # print(self.neck_pos)

                    elif ps4_controller.triangle >= self.ON_AND_RISING:
                        self.neck_pos_tilt += neck_inc_ver
                        print("UP")
                        if self.neck_pos_tilt > 45:
                            self.neck_pos_tilt = 45
                        self.robot.set_neck([self.neck_pos_pan, self.neck_pos_tilt], wait_for_end_of=False)
                        # print(self.neck_pos)
                
                time.sleep(self.iteration_time)
                3


            elif self.state == self.Final_State:
                
                self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                # Lock after finishing task
                while True:
                    pass

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



    """


    def init(self):

        self.rgb_demo_index = 0
        self.face_demo_index = 0
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_arm = False 
        self.arm_ready = True

        # rgb leds used for demonstration, can be added any other necessary for demonstration
        rgb_demonstration = [100, 0, 13, 24, 35, 46, 57, 68, 79, 100, 101, 102, 103, 104, 105, 106, 255]

        # rgb leds used for demonstration, can be added any other necessary for demonstration
        face_demonstration = ["charmie_face", "charmie_face_green", "help_pick_spoon", "help_pick_milk", "help_pick_cornflakes", "help_pick_bowl"]


        self.i = 0

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.arm_success = True
        self.arm_message = ""
        self.rgb_success = True
        self.rgb_message = ""

        self.wfeon = Bool()
        self.torso_pos = Pose2D()
        self.select_movement = String()

        if self.CONTROL_SET_MOVEMENT:
            self.set_movement = Bool()
            self.set_movement.data = True
            self.set_movement_publisher.publish(self.set_movement)

        if self.CONTROL_MOTORS:
            self.omni_move = Vector3()
            self.omni_move.x = 0.0
            self.omni_move.y = 0.0
            self.omni_move.z = 100.0
            self.omni_move_publisher.publish(self.omni_move)

        if self.CONTROL_FACE:
            self.face_mode = String()
            self.face_mode.data = "charmie_face"
            self.image_to_face_publisher.publish(self.face_mode)

        self.watchdog_timer = 0
        self.watchdog_flag = False
    
    

def timer_callback(self):

        # create ps4 controller object
        ps_con = PS4Controller()

        if self.controller.values_updated == True:
            self.watchdog_timer = 0            
            self.watchdog_flag = False

            # attribute controller data to ps4 controller object   
            ps_con.arrow_up = int(self.controller.button_state(self.controller.ARROW_UP))
            ps_con.arrow_right = int(self.controller.button_state(self.controller.ARROW_RIGHT))
            ps_con.arrow_down = int(self.controller.button_state(self.controller.ARROW_DOWN))
            ps_con.arrow_left = int(self.controller.button_state(self.controller.ARROW_LEFT))

            ps_con.triangle = int(self.controller.button_state(self.controller.TRIANGLE))
            ps_con.circle = int(self.controller.button_state(self.controller.CIRCLE))
            ps_con.cross = int(self.controller.button_state(self.controller.CROSS))
            ps_con.square = int(self.controller.button_state(self.controller.SQUARE))

            ps_con.l1 = int(self.controller.button_state(self.controller.L1))
            ps_con.r1 = int(self.controller.button_state(self.controller.R1))
            ps_con.l3 = int(self.controller.button_state(self.controller.L3))
            ps_con.r3 = int(self.controller.button_state(self.controller.R3))

            ps_con.share = int(self.controller.button_state(self.controller.SHARE))
            ps_con.options = int(self.controller.button_state(self.controller.OPTIONS))
            ps_con.ps = int(self.controller.button_state(self.controller.PS))

            ps_con.l2 = float(self.controller.L2dist)
            ps_con.r2 = float(self.controller.R2dist)

            ps_con.l3_ang = float(self.controller.L3ang)
            ps_con.l3_dist = float(self.controller.L3dist)
            ps_con.l3_xx = float(self.controller.L3xx)
            ps_con.l3_yy = float(self.controller.L3yy)

            ps_con.r3_ang = float(self.controller.R3ang)
            ps_con.r3_dist = float(self.controller.R3dist)
            ps_con.r3_xx = float(self.controller.R3xx)
            ps_con.r3_yy = float(self.controller.R3yy)

            # prevents small noises in joy sticks
            if ps_con.l3_dist < 0.1:
                ps_con.l3_ang = 0.0

            if ps_con.r3_dist < 0.1:
                ps_con.r3_ang = 0.0
            
            # overall debug print
            print("\n", ps_con.arrow_up, ps_con.arrow_right, ps_con.arrow_down, ps_con.arrow_left, "|",
                  ps_con.triangle, ps_con.circle, ps_con.cross, ps_con.square, "|",
                  ps_con.l1, ps_con.r1, round(ps_con.l2, 1), round(ps_con.r2, 1), ps_con.l3, ps_con.r3, "|",
                  ps_con.share, ps_con.ps, ps_con.options, "|",
                  str(round(ps_con.l3_ang)).rjust(3), round(ps_con.l3_dist, 1), str(round(ps_con.l3_xx, 1)).rjust(4), str(round(ps_con.l3_yy, 1)).rjust(4), "|", 
                  str(round(ps_con.r3_ang)).rjust(3), round(ps_con.r3_dist, 1), str(round(ps_con.r3_xx, 1)).rjust(4), str(round(ps_con.r3_yy, 1)).rjust(4), "|",
                  end='')

            # publishes ps4 controller object so if any other needs it, can use controller data  
            self.ps4_controller_publisher.publish(ps_con)
            
            # control code to send commands to other nodes if CONTROL variables are set to true (ros2 params)
            self.control_robot(ps_con)

            # gets values ready for next iteration
            self.controller.every_button_update()
            self.controller.values_updated = False

        else:
            if not self.watchdog_flag:
                self.watchdog_timer += 1


        if self.watchdog_timer == 40: # since the ps4 controller checks every 50 ms. 20*50ms is 1 second. 40 is 2 seconds.
            # this is set if in any kind of emergency the controller stops communicating. 
            # If the system continues with the last received variables it may result in physical damages.
            # Therefore, to every moving part that may continue moving is set stop commands
            self.watchdog_flag = True
            self.watchdog_timer += 1

            print("WATCHDOG BLOCK - NO COMMUNICATIONS IN THE LAST 2 SECONDS")

            # only does this if not in locked motors mode, this prevents always sending the following commands continuously even when it is already locked (mainly stops speaking a lot of times)
            if self.set_movement.data == True:
                    
                # locks motors
                if self.CONTROL_SET_MOVEMENT:
                    self.set_movement.data = False
                    self.set_movement_publisher.publish(self.set_movement)

                # sends command to stop torso
                if self.CONTROL_TORSO:
                    self.torso_pos.x = 0.0
                    self.torso_pos.y = 0.0
                    self.torso_movement_publisher.publish(self.torso_pos)

                # changes to a red value to notify it entered in motors locked mode
                if self.CONTROL_RGB:
                    self.set_rgb(RED+HALF_ROTATE)

                # in case it is not intended for the robot to speak since it may disturb other packages
                if self.CONTROL_SPEAKERS:
                    self.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)

        # print(self.watchdog_timer, end='')
        print(".", end='')

    # control code to send commands to other nodes if CONTROL variables are set to true (ros2 params)
    def control_robot(self, ps4_controller):
        if self.CONTROL_TORSO:
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

            self.torso_movement_publisher.publish(self.torso_pos)

            # if self.CONTROL_WAIT_FOR_END_OF_NAVIGATION:
            # self.wfeon.data = True
            # if ps4_controller.options:
            #     self.flag_pos_reached_publisher.publish(self.wfeon)

        if self.CONTROL_MOTORS:
            # left joy stick to control x and y movement (direction and linear speed) 
            if ps4_controller.l3_dist >= 0.1:
                self.omni_move.x = ps4_controller.l3_ang
                self.omni_move.y = ps4_controller.l3_dist*100/5
            else:
                self.omni_move.x = 0.0
                self.omni_move.y = 0.0

            # right joy stick to control angular speed
            if ps4_controller.r3_dist >= 0.1:
                self.omni_move.z = 100 + ps4_controller.r3_xx*10
            else:
                self.omni_move.z = 100.0
                       
            self.omni_move_publisher.publish(self.omni_move)

        if self.CONTROL_RGB:
            # only does this if not in locked motors mode, otherwise it will change the rgb and being in this mode might be unoticed
            if self.set_movement.data == True:
                if ps4_controller.r1 == 2:
                    self.rgb_demo_index+=1
                    if self.rgb_demo_index >= len(rgb_demonstration):
                        self.rgb_demo_index-=len(rgb_demonstration)

                    # print(self.rgb_demo_index)
                    self.set_rgb(rgb_demonstration[self.rgb_demo_index])

        if self.CONTROL_SPEAKERS:
            # examples of two different speech commands
            if ps4_controller.r3 == 2:
                self.i += 1
                if self.i == 1:
                    success, message = self.set_speech(filename="generic/introduction_full", wait_for_end_of=False)
                    # success, message = self.set_speech(filename="generic/welcome_roboparty", wait_for_end_of=False)

                    print(success, message)
                    # elif self.i == 2:
                    # success, message = self.set_speech(filename="generic/welcome_roboparty", wait_for_end_of=False)
                    # print(success, message)
                else: 
                    success, message = self.set_speech(filename="receptionist/receptionist_question", wait_for_end_of=False)
                    # success, message = self.set_speech(filename="generic/welcome_roboparty", wait_for_end_of=False)
                    print(success, message)
                    self.i = 0
            # elif ps4_controller.l3 == 2:
                # success, message = self.set_speech(filename="receptionist/receptionist_question", wait_for_end_of=False)
                # print(success, message)
                

        if self.CONTROL_SET_MOVEMENT:
            if ps4_controller.ps == 2:
                if self.set_movement.data == False:
                    
                    # stops locked motor mode, motors can now run
                    self.set_movement.data = True
                    self.set_movement_publisher.publish(self.set_movement)

                    # returns to the value it was previously
                    if self.CONTROL_RGB:
                        self.set_rgb(rgb_demonstration[self.rgb_demo_index])

                    # in case it is not intended for the robot to speak since it may disturb other packages
                    if self.CONTROL_SPEAKERS:
                        self.set_speech(filename="demonstration/motors_unlocked", wait_for_end_of=False)

                else:

                    # locks motors
                    self.set_movement.data = False
                    self.set_movement_publisher.publish(self.set_movement)

                    # sends command to stop torso
                    if self.CONTROL_RGB:
                        self.torso_pos.x = 0.0
                        self.torso_pos.y = 0.0
                        self.torso_movement_publisher.publish(self.torso_pos)
                
                    # changes to a red value to notify it entered in motors locked mode    
                    if self.CONTROL_RGB:
                        self.set_rgb(RED+HALF_ROTATE)

                    # in case it is not intended for the robot to speak since it may disturb other packages
                    if self.CONTROL_SPEAKERS:
                        self.set_speech(filename="demonstration/motors_locked", wait_for_end_of=False)

        if self.CONTROL_FACE:
            if ps4_controller.l1 == 2:
                self.face_demo_index+=1
                if self.face_demo_index >= len(face_demonstration):
                    self.face_demo_index-=len(face_demonstration)

                # print(self.rgb_demo_index)
                self.face_mode.data = face_demonstration[self.face_demo_index]
                self.image_to_face_publisher.publish(self.face_mode)

        if self.CONTROL_ARM:
            if self.arm_ready: 
                if ps4_controller.share == 2:
                    # self.arm_ready = False
                    # Command to say hello 
                    # success, message = self.set_arm(command="hello", wait_for_end_of=False)
                    # success, message = self.set_arm(command="place_objects", wait_for_end_of=False)
                    success, message = self.set_arm(command="pick_objects", wait_for_end_of=False)
                    print(success, message)
                if ps4_controller.options == 2:
                    # Command to say hello 
                    success, message = self.set_arm(command="hello", wait_for_end_of=False)

                if ps4_controller.l3 == 2:
                    # self.arm_ready = False
                    success, message = self.set_arm(command="place_objects", wait_for_end_of=False)
"""