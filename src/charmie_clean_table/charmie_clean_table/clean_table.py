#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from std_msgs.msg import Bool, String, Int16
from charmie_interfaces.msg import SpeechType, RobotSpeech


class CleanTableNode(Node):

    def __init__(self):
        super().__init__("CleanTable")
        self.get_logger().info("Initialised CHARMIE CleanTable Node")

        ###         PUBs/SUBs       
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
   
    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.get_logger().info("Received Speech Flag")    


def main(args=None):
    rclpy.init(args=args)
    node = CleanTableNode()
    th_main = threading.Thread(target=thread_main_clean_table, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_clean_table(node: CleanTableNode):
    main = CleanTableMain(node)
    main.main()

class CleanTableMain():

    def __init__(self, node: CleanTableNode):
        self.node = node
        
        # VARS ...
        self.state = 0

        # wait_for_end_of functions ...

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        print("IN NEW MAIN")
        # time.sleep(1)

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:
                #print('State 0 = Initial')

                # your code here ...
                                
                # next state
                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
                #print('State 1 = Hand Raising Detect')

                # your code here ...
                                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my clean the table task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass