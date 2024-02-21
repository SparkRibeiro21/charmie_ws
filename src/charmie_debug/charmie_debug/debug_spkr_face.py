#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

import threading
import time

from example_interfaces.msg import String
from charmie_interfaces.srv import SpeechCommand
from sensor_msgs.msg import Image


class TestNode(Node):

    def __init__(self):
        super().__init__("Test")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)

        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")

        self.waited_for_end_of_speaking = False
        self.test_image_face_str = String()
        self.test_custom_image_face_str = String()

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
    


    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success)+str(response.message))
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: TestNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: TestNode):
        self.node = node
        
        # VARS ...
        self.state = 0
    
    def speech_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False
    

    # def wait_for_end_of_speaking(self):
    #     while not self.node.waited_for_end_of_speaking:
    #         pass
    #     self.node.waited_for_end_of_speaking = False
        
        

        # wait_for_end_of functions ...

    def main(self):
        a = "a"
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        print("IN NEW MAIN")
        time.sleep(2)

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
                """
                # a += "a"
                self.node.call_speech_command_server("ready_serve_breakfast", "", wait_for_end_of=True)
                self.wait_for_end_of_speaking()
                print("Test Wait")

                start = time.time()
                while time.time() < start + 3: # in seconds
                    print(".", end='')
                print()
                
                self.node.call_speech_command_server("waiting_door_open", "", wait_for_end_of=False)
                # self.wait_for_end_of_speaking()
                print("Test Not Wait")

                start = time.time()
                while time.time() < start + 3: # in seconds
                    print(",", end='')
                print()
                """

                # self.node.call_speech_command_server("receptionist_second_guest", "", wait_for_end_of=True)
                # self.node.call_speech_command_server(command="Welcome to the serve the breakfast task", wait_for_end_of=True)#, quick_voice=Fa)

                # self.node.call_speech_command_server("introduction_quick", "", wait_for_end_of=True)
                # self.node.call_speech_command_server(command="I will start in a minute", wait_for_end_of=True)#, quick_voice=True)
                

                # self.node.call_speech_command_server(filename="receptionist_second_guest_john", command="hey", wait_for_end_of=True)#, quick_voice=Fa)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")

                # self.node.call_speech_command_server(filename="recep_drink_red_win", command="hey", wait_for_end_of=True)#, quick_voice=Fa)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")




                ### self.speech_server(filename="introduction_full", command="", wait_for_end_of=True)
                # self.speech_server(filename="recep", command="My favourite drink is pleno", wait_for_end_of=True, quick_voice=False)
                # self.speech_server(filename="receptionist2_1", wait_for_end_of=True)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")


                # start = time.time()
                # while time.time() < start + 4: # in seconds
                #     pass
                #     print(".", end='')
                # print()


                ### self.speech_server(filename="arm_close_gripper", command="", wait_for_end_of=True)#, quick_voice=Fa)
                # self.speech_server(filename="", command="The favourite drink of Leia is water.", wait_for_end_of=True, quick_voice=False)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")

                self.node.test_custom_image_face_str.data = "clients_temp"
                self.node.custom_image_to_face_publisher.publish(self.node.test_custom_image_face_str)
                time.sleep(5)

                self.speech_server(filename="introduction_full", command="", wait_for_end_of=True)

                self.node.test_image_face_str.data = "demo1"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo2"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo3"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo4"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.speech_server(filename="arm_close_gripper", command="", wait_for_end_of=True)

                self.node.test_image_face_str.data = "demo5"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo6"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo7"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                self.node.test_image_face_str.data = "demo8"
                self.node.image_to_face_publisher.publish(self.node.test_image_face_str)
                time.sleep(1)

                



                start = time.time()
                while time.time() < start + 3: # in seconds
                    pass
                    # print(",", end='')
                # print()



                #print('State 0 = Initial')

                # your code here ...
                                
                # next state
                # self.state = Searching_for_clients

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