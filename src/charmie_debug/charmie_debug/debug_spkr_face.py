#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from std_msgs.msg import Bool, String, Int16
from charmie_interfaces.msg import SpeechType, RobotSpeech


class GpsrNode(Node):

    def __init__(self):
        super().__init__("Gpsr")
        self.get_logger().info("Initialised CHARMIE GPSR Node")

        ###         PUBs/SUBs       
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
   
    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.get_logger().info("Received Speech Flag")    


def main(args=None):
    rclpy.init(args=args)
    node = GpsrNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: GpsrNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: GpsrNode):
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
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass