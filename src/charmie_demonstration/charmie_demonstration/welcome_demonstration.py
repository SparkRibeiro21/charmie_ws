#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_interfaces.msg import DetectedObject, DetectedPerson
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
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
    "charmie_point_cloud":      False,
    "charmie_ps4_controller":   False,
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
        self.Approach_kitchen_counter = 1
        self.Picking_up_spoon = 2
        self.Picking_up_milk = 3
        self.Picking_up_cereal = 4
        self.Picking_up_bowl = 5
        self.Approach_kitchen_table = 6
        self.Placing_bowl = 7
        self.Placing_cereal = 8
        self.Placing_milk = 9
        self.Placing_spoon = 10
        self.Final_State = 11
        
        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        
        # Start localisation position
        self.initial_position = [-1.0, 1.5, -90.0]

        # navigation positions
        self.front_of_sofa = [-2.5, 1.5]
        self.sofa = [-2.5, 3.0]
        
        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")

                # your code here ...
                
                # moves the neck to look down for navigation
                # self.set_neck(position=self.look_navigation, wait_for_end_of=False)
                """
                # send speech command to speakers voice, intrucing the robot 
                self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
                
                # sends RGB value for debug
                self.set_rgb(command=CYAN+HALF_ROTATE)

                # change face, to face picking up cup
                # self.set_face("help_pick_cup")
                
                # waiting for start button
                self.wait_for_door_start()

                self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)

                self.set_speech(filename="serve_breakfast/sb_moving_kitchen_counter", wait_for_end_of=True)

                print("DONE2 - ", self.node.start_button_state)
                # calibrate the background noise for better voice recognition
                # self.calibrate_audio(wait_for_end_of=True)

                # moves the neck to look to absolute cordinates in the map
                # self.set_neck_coords(position=[1.0, 1.0], ang=30, wait_for_end_of=True)
                """
                self.robot.set_initial_position(self.initial_position)

                time.sleep(3)
                
                # this gives an error because "orient" is a non-existing movement type and does not send anything to navigation 
                self.robot.set_navigation(movement="orient", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)

                print("2 move")

                self.robot.set_navigation(movement="orientate", absolute_angle=90.0, flag_not_obs=True, wait_for_end_of=True)

                print("3 move")

                self.robot.set_navigation(movement="move", target=self.front_of_sofa, flag_not_obs=True, wait_for_end_of=True)

                print("4 move")

                self.robot.set_navigation(movement="rotate", target=self.sofa, flag_not_obs=True, wait_for_end_of=True)

                print("5 move")

                while True:
                    pass

                ### RECEPTIONIST AUDIO EXAMPLE
                # command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                # print("Finished:", command)
                # keyword_list= command.split(" ")
                # print(keyword_list[0], keyword_list[1])
                # self.set_speech(filename="receptionist/recep_first_guest_"+keyword_list[0].lower(), wait_for_end_of=True)
                # self.set_speech(filename="receptionist/recep_drink_"+keyword_list[1].lower(), wait_for_end_of=True)  

                # change face, to standard face
                # self.set_face("charmie_face")

                # moves the neck to look forward
                # self.set_neck(position=self.look_forward, wait_for_end_of=False)

                """
                ### EXAMPLES TO ACTIVATE/DEACTIVATE AND CONFIGURE YOLO POSE AND TOLO OBJECTS 
                self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)
                print("activated yolo pose")
                time.sleep(5)
                # self.activate_yolo_pose(activate=True, only_detect_person_right_in_front=True)
                # time.sleep(0.5)
                while self.node.detected_people.num_person == 0:
                    pass
                self.track_person(self.node.detected_people.persons[0], body_part="Head", wait_for_end_of=True)
                time.sleep(3)
                self.activate_yolo_pose(activate=False)
                self.activate_yolo_objects(activate_objects=True)
                time.sleep(5)
                while self.node.detected_objects.num_objects == 0:
                    pass
                self.track_object(self.node.detected_objects.objects[0], wait_for_end_of=True)
                time.sleep(3)
                # self.activate_yolo_objects(activate_objects=True, minimum_objects_confidence=0.3)
                # print("deactivated yolo pose - 0.8")
                # time.sleep(5)
                # self.activate_yolo_pose(activate=True, minimum_keypoints_to_detect_person=10)
                # self.activate_yolo_objects(activate_objects=True, minimum_objects_confidence=0.8)
                # print("deactivated yolo pose - right in front")
                # time.sleep(5)
                """

                # example of arm function to say hello
                # self.set_arm(command="hello", wait_for_end_of=False)

                # next state
                # self.state = self.Approach_kitchen_counter

            elif self.state == self.Approach_kitchen_counter:
                print("State:", self.state, "- Approach_kitchen_counter")
                # your code here ...
                                
                # next state
                self.state = self.Picking_up_spoon

            elif self.state == self.Picking_up_spoon:
                print("State:", self.state, "- Picking_up_spoon")

                ##### NECK LOOKS AT TABLE
                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### MOVES ARM TO TOP OF TABLE POSITION

                ##### SPEAK: Searching for objects
                # self.set_speech(filename="generic/search_objects", wait_for_end_of=True)

                ##### YOLO OBJECTS SEARCH FOR SPOON, FOR BOTH CAMERAS
                # self.set_neck(position=self.look_judge, wait_for_end_of=True)
                
                ##### SPEAK: Found spoon
                # self.set_speech(filename="serve_breakfast/sb_found_spoon", show_in_face=True, wait_for_end_of=True)
                
                ##### SPEAK: Check face to see object detected
                # self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=True)

                ##### SHOW FACE DETECTED OBJECT (CUSTOM)

                ##### MOVE ARM TO PICK UP OBJECT 

                ##### IF AN ERROR IS DETECTED:

                    ##### SPEAK: There is a problem picking up the object
                    # self.set_speech(filename="generic/problem_pick_object", wait_for_end_of=True) # False

                    ##### MOVE ARM TO ERROR POSITION 
                
                    ##### NECK LOOK JUDGE
                    # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                    ##### SPEAK: Need help, put object on my hand as it is on my face
                    # self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                    ##### SHOW FACE GRIPPER SPOON 
                    # self.set_face("help_pick_spoon") 

                    ##### WHILE OBJECT IS NOT IN GRIPPER:

                        ##### SPEAK: Close gripper in 3 2 1 
                        # self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                        ##### ARM: CLOSE GRIPPER

                        ##### IF OBJECT NOT GRABBED: 

                            ##### SPEAK: There seems to be a problem, please retry.
                            # self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)

                            ##### ARM OPEN GRIPPER
                
                ##### SET FACE TO STANDARD FACE
                # self.set_face("charmie_face")
                        
                ##### NECK LOOK TRAY
                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                        
                ##### ARM PLACE OBJECT IN TRAY

                self.state = self.Picking_up_milk

            elif self.state == self.Picking_up_milk:
                print("State:", self.state, "- Picking_up_milk")
                # your code here ...
                                                
                # next state
                self.state = self.Picking_up_cereal
           
            elif self.state == self.Picking_up_cereal:
                print("State:", self.state, "- Picking_up_cereal")
                # your code here ...
                                
                # next state
                self.state = self.Picking_up_bowl

            elif self.state == self.Picking_up_bowl:
                print("State:", self.state, "- Picking_up_bowl")
                # your code here ...
                                
                # next state
                self.state = self.Approach_kitchen_table

            elif self.state == self.Approach_kitchen_table:
                print("State:", self.state, "- Approach_kitchen_table")
                # your code here ...
                                
                # next state
                self.state = self.Placing_bowl

            elif self.state == self.Placing_bowl:
                print("State:", self.state, "- Placing_bowl")
                # your code here ...
                                
                # next state
                self.state = self.Placing_cereal 

            elif self.state == self.Placing_cereal:
                print("State:", self.state, "- Placing_cereal")
                # your code here ...
                                
                # next state
                self.state = self.Placing_milk
           
            elif self.state == self.Placing_milk:
                print("State:", self.state, "- Placing_milk")
                # your code here ...
                                
                # next state
                self.state = self.Placing_spoon

            elif self.state == self.Placing_spoon:
                print("State:", self.state, "- Placing_spoon")
                # your code here ...
                                
                # next state
                self.state = self.Final_State 
                
            elif self.state == self.Final_State:
                
                self.robot.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                # Lock after finishing task
                while True:
                    pass

            else:
                pass





# class ControllerNode(Node):

    # def __init__(self):
        # super().__init__("PS4_Controller")
        # self.get_logger().info("Initialised CHARMIE PS4 Controller Node")

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        # self.declare_parameter("control_arm", True) 
        # self.declare_parameter("control_face", True)
        # self.declare_parameter("control_motors", True)
        # self.declare_parameter("control_neck", True) 
        # self.declare_parameter("control_rgb", True) 
        # self.declare_parameter("control_set_movement", True) 
        # self.declare_parameter("control_speakers", True)
        # self.declare_parameter("control_torso", True)
        # self.declare_parameter("control_wait_for_end_of_navigation", True)

        # Create Controller object
        # self.controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        # self.torso_movement_publisher = self.create_publisher(Pose2D, "torso_move" , 10)
        # self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        # self.set_movement_publisher = self.create_publisher(Bool, "set_movement", 10)

        # Navigation
        # self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_pos_reached", 10)

        # PS4 Controller
        # self.ps4_controller_publisher = self.create_publisher(PS4Controller, "controller_state", 10)

        # Face
        # self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        
        # Arm 
        # self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        # self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)

        ### Services (Clients) ###
        # Speakers
        # self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Neck
        # self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        # Low level
        # self.set_rgb_client = self.create_client(SetRGB, "rgb_mode")


        # CONTROL VARIABLES, this is what defines which modules will the ps4 controller control
        # self.CONTROL_ARM = self.get_parameter("control_arm").value
        # self.CONTROL_FACE = self.get_parameter("control_face").value
        # self.CONTROL_MOTORS = self.get_parameter("control_motors").value
        # self.CONTROL_NECK = self.get_parameter("control_neck").value
        # self.CONTROL_RGB = self.get_parameter("control_rgb").value
        # self.CONTROL_SET_MOVEMENT = self.get_parameter("control_set_movement").value
        # self.CONTROL_SPEAKERS = self.get_parameter("control_speakers").value
        # self.CONTROL_TORSO = self.get_parameter("control_torso").value
        # self.CONTROL_WAIT_FOR_END_OF_NAVIGATION = self.get_parameter("control_wait_for_end_of_navigation").value

        # timer that checks the controller every 50 ms 
        # self.create_timer(0.05, self.timer_callback)








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

        if self.CONTROL_RGB:
            self.set_rgb(RAINBOW_ROT)

        if self.CONTROL_NECK:
            self.neck_pos = SetNeckPosition.Request()
            self.neck_pos.pan = float(0)
            self.neck_pos.tilt = float(0)
            self.set_neck_position_client.call_async(self.neck_pos)

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
                
        if self.CONTROL_NECK:
            # circle and square to move neck left and right
            # triangle and cross to move the neck up and down
            neck_inc = 5.0
            if ps4_controller.circle >= 2:
                self.neck_pos.pan -= neck_inc
                if self.neck_pos.pan < -180.0:
                    self.neck_pos.pan = -180.0
                self.set_neck_position_client.call_async(self.neck_pos)
                # print(self.neck_pos)

            elif ps4_controller.square >= 2:
                self.neck_pos.pan += neck_inc
                if self.neck_pos.pan > 180.0:
                    self.neck_pos.pan = 180.0
                self.set_neck_position_client.call_async(self.neck_pos)
                # print(self.neck_pos)

            if ps4_controller.cross >= 2:
                self.neck_pos.tilt -= neck_inc
                if self.neck_pos.tilt < -60.0:
                    self.neck_pos.tilt = -60.0
                self.set_neck_position_client.call_async(self.neck_pos)
                # print(self.neck_pos)

            elif ps4_controller.triangle >= 2:
                self.neck_pos.tilt += neck_inc
                if self.neck_pos.tilt > 45.0:
                    self.neck_pos.tilt = 45.0
                self.set_neck_position_client.call_async(self.neck_pos)
                # print(self.neck_pos)

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