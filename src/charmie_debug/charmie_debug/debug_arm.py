#!/usr/bin/env python3

### JUST ARM, SPEAKERS AND LOW LEVEL ###

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger

import cv2 
import threading
import time
from cv_bridge import CvBridge

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class DebugArmNode(Node):

    def __init__(self):
        super().__init__("DebugArm")
        self.get_logger().info("Initialised CHARMIE DebugArm Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 

        # Arm CHARMIE
        self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        # Arm (CHARMIE)
        while not self.arm_trigger_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_arm = False

        self.start_button_state = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.arm_success = True
        self.arm_message = ""


    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        # self.arm_ready = True
        self.waited_for_end_of_arm = True
        self.arm_success = flag.data
        if flag.data:
            self.arm_message = "Arm successfully moved"
        else:
            self.arm_message = "Wrong Movement Received"

        self.get_logger().info("Received Arm Finished")

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

    #### SPEECH SERVER FUNCTIONS #####
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True, show_in_face=False):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.show_in_face = show_in_face
    
        future = self.speech_command_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"

    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = DebugArmNode()
    th_main = threading.Thread(target=ThreadMainDebugArm, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainDebugArm(node: DebugArmNode):
    main = DebugArmMain(node)
    main.main()

class DebugArmMain():

    def __init__(self, node: DebugArmNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message

    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message
 
    def wait_for_start_button(self):

        self.node.start_button_state = False

        t = Bool()
        t.data = True
        self.node.flag_start_button_publisher.publish(t)

        while not self.node.start_button_state:
            pass

        t.data = False 
        self.node.flag_start_button_publisher.publish(t)
        
    def set_arm(self, command="", wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False
        
        temp = String()
        temp.data = command
        self.node.arm_command_publisher.publish(temp)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_arm:
                pass
            self.node.waited_for_end_of_arm = False
        else:
            self.node.arm_success = True
            self.node.arm_message = "Wait for answer not needed"

        # self.node.get_logger().info("Set Arm Response: %s" %(str(self.arm_success) + " - " + str(self.arm_message)))
        return self.node.arm_success, self.node.arm_message
    
    # main state-machine function
    def main(self):
        
        # States in DebugArm Task
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

        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start

        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In DebugArm Main...")

        while True:

            if self.state == self.Waiting_for_task_start:
                print("State:", self.state, "- Waiting_for_task_start")
                

                # INITIAL STATE

                self.set_rgb(command=CYAN+HALF_ROTATE)

                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)

                self.wait_for_start_button()

                self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)



                # DETECTION

                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)
                
                self.set_arm(command="search_for_objects", wait_for_end_of=True)

                self.set_speech(filename="serve_breakfast/found_all_sb_objects", wait_for_end_of=True)

                self.set_arm(command="search_for_objects_to_ask_for_objects", wait_for_end_of=False)
                
                
                # GET SPOON 
                
                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                   
                self.set_speech(filename="serve_breakfast/found_the_milk", wait_for_end_of=False)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="collect_spoon_to_tray", wait_for_end_of=True)

                
                # GET MILK 
                
                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                   
                self.set_speech(filename="serve_breakfast/found_the_cornflakes", wait_for_end_of=False)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="collect_milk_to_tray", wait_for_end_of=True)


                # GET CORNFLAKES 
                
                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                   
                self.set_speech(filename="serve_breakfast/found_the_bowl", wait_for_end_of=False)  
                
                self.set_speech(filename="generic/check_face_object_detected", wait_for_end_of=False)  
                
                self.set_arm(command="collect_cornflakes_to_tray", wait_for_end_of=True)



                # GET BOWL 
                
                self.set_arm(command="open_gripper", wait_for_end_of=False)

                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)
                
                object_in_gripper = False
                while not object_in_gripper:
                
                    self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                    object_in_gripper, m = self.set_arm(command="close_gripper_with_check_object", wait_for_end_of=True)

                    # object_in_gripper, m = self.set_arm(command="verify_if_object_is_grabbed", wait_for_end_of=True)
                    
                    if not object_in_gripper:
                
                        self.set_speech(filename="arm/arm_error_receive_object_quick", wait_for_end_of=True)
                        
                        self.set_arm(command="open_gripper", wait_for_end_of=False)
                   
                self.set_arm(command="collect_bowl_to_initial_position", wait_for_end_of=True)
























                # example of arm function to say hello
                # self.set_arm(command="hello", wait_for_end_of=False)

                # next state
                self.state = self.Approach_kitchen_counter

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
                # self.set_face("demo5")
                        
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
                
                self.set_speech(filename="serve_breakfast/sb_finished", wait_for_end_of=True)

                # Lock after finishing task
                while True:
                    pass

            else:
                pass
