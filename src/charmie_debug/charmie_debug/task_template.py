#!/usr/bin/env python3

# initial instructions:
# this is an example of the layout code for a task in our workspace
# It is used a threading system so the ROS2 functionalities are not blocked when executing the state machine
# So we create a thread for the main state machine of the task and another for the ROS2 funcitonalities
# It is being used GPSR as an example, so when changing the code for a specific task Ctrl+F gpsr
# and replace everything fot the name of your task 

import rclpy
from rclpy.node import Node

import threading

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from charmie_interfaces.msg import SpeechType, RobotSpeech
from charmie_interfaces.srv import SpeechCommand

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class GpsrNode(Node):

    def __init__(self):
        super().__init__("Gpsr")
        self.get_logger().info("Initialised CHARMIE GPSR Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)    
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
    
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # while not self.speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.flag_navigation_done = False

        # Sucess and Message confirmations for all set_(something) CHARMIE functions
        self.speech_sucess = True
        self.speech_message = ""

    # Function to send commands to speaker module 
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
    
        future = self.speech_command_client.call_async(request)
        print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_sucess = True
            self.speech_message = "Wait for answer not needed"

    # Function that wait for a response from the service (int his case: wait for speech being said to be over)
    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucess = response.success
            self.speech_message = response.message
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    # Subscriber to get the start button status     
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = GpsrNode()
    th_main = threading.Thread(target=thread_main_gpsr, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_gpsr(node: GpsrNode):
    main = GpsrMain(node)
    main.main()

class GpsrMain():

    def __init__(self, node: GpsrNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node
        
        # create the necessary variables for the state machine
        self.state = 0
        self.rgb_mode = Int16()

    # wait_for_end_of functions ... 
    # functions where the state machine must wait or not (depending on the wait_for_end_of) for a command 
    # whether this is a variable or just finish a sub-task (speaking, hearing, navigating ...)
    def set_speech(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_sucess, self.node.speech_message
    

    # just an example of a wait for end of navigation
    def wait_for_end_of_navigation(self):
        self.node.flag_navigation_done = False
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False
        print("Finished Navigation")

    # main state-machine function
    def main(self):
        # examples of names of states
        # use the names of states rather than just numbers to ease 3rd person code analysis
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        # debug print
        print("IN NEW MAIN")

        while True:

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')

                # your code here ...

                # send speech command to speakers voice, intrucing the robot 
                self.set_speech(filename="introduction_full", wait_for_end_of=True)
                # if you want to have some information regarding the the set_speech you just sent you can try the following:
                success, message = self.set_speech(filename="introduction_full", wait_for_end_of=True)
                # THIS CAN BE DONE FOR ALL set_(something) functions
                print(success, message)
                
                # send rgb commands to low level
                self.rgb_mode.data = CYAN+ROTATE
                self.node.rgb_mode_publisher.publish(self.rgb_mode)

                                
                # next state
                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                
                print("Finished task!!!")

                # After finishing the task stays in this loop 
                while True:
                    pass

            else:
                pass