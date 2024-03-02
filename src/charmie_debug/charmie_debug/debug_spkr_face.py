#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

import threading
import time

from example_interfaces.msg import String, Int16
from charmie_interfaces.srv import SpeechCommand
from sensor_msgs.msg import Image

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        ### Topics (Publisher and Subscribers) ###  
        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")

        # Variables
        self.waited_for_end_of_speaking = False
        self.test_image_face_str = String()
        self.test_custom_image_face_str = String()

        # Sucess and Message confirmations for all set_(something) CHARMIE functions
        self.speech_sucess = True
        self.speech_message = ""
        self.rgb_sucess = True
        self.rgb_message = ""
        self.face_sucess = True
        self.face_message = ""

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
            self.speech_sucess = True
            self.speech_message = "Wait for answer not needed"
    


    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucess = response.success
            self.speech_message = response.message
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
    
    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_sucess, self.node.speech_message
    
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_sucess = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_sucess, self.node.rgb_message
    
    
    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        if custom == "":
            temp = String()
            temp.data = command
            self.node.image_to_face_publisher.publish(temp)
        else:
            temp = String()
            temp.data = custom
            self.node.custom_image_to_face_publisher.publish(temp)

        self.node.face_sucess = True
        self.node.face_message = "Value Sucessfully Sent"

        return self.node.face_sucess, self.node.face_message


    # def wait_for_end_of_speaking(self):
    #     while not self.node.waited_for_end_of_speaking:
    #         pass
    #     self.node.waited_for_end_of_speaking = False
        
        

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




                ### self.set_speech(filename="introduction_full", command="", wait_for_end_of=True)
                # self.set_speech(filename="recep", command="My favourite drink is pleno", wait_for_end_of=True, quick_voice=False)
                # self.set_speech(filename="receptionist2_1", wait_for_end_of=True)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")


                # start = time.time()
                # while time.time() < start + 4: # in seconds
                #     pass
                #     print(".", end='')
                # print()


                ### self.set_speech(filename="arm_close_gripper", command="", wait_for_end_of=True)#, quick_voice=Fa)
                # self.set_speech(filename="", command="The favourite drink of Leia is water.", wait_for_end_of=True, quick_voice=False)
                # self.wait_for_end_of_speaking()
                # print("Test Wait")



                """
                s, m = self.set_rgb(RED+MOON)
                print(s, m)
                success, message = self.set_speech(filename="arm_close_gripper", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)
                self.set_rgb(BLUE+ROTATE)
                success, message = self.set_speech(filename="introduction_full", command="", wait_for_end_of=False)
                print(success, message)
                time.sleep(5)
                self.set_rgb(WHITE+ALTERNATE_QUARTERS)
                success, message = self.set_speech(filename="arm_close_grippe", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)

                """

                
                # self.set_speech(filename="introduction_full", command="", wait_for_end_of=True)
                # time.sleep(2)

                self.set_speech(filename="introduction_full", command="", wait_for_end_of=True)
                self.set_face("help_pick_cup")
                time.sleep(3)


                # self.set_speech(filename="introduction_ful", command="", wait_for_end_of=True)


                # self.set_speech(command="Hello motherfucker", wait_for_end_of=True)


                # self.node.test_custom_image_face_str.data = "clients_temp"
                # self.node.custom_image_to_face_publisher.publish(self.node.test_custom_image_face_str)
                # time.sleep(5)

                # self.set_speech(filename="introduction_full", wait_for_end_of=True)
                
                # self.set_face(custom="clients_temp")
                # time.sleep(3)


                self.set_face("help_pick_bowl")
                time.sleep(3)

                self.set_speech(filename="introduction_full", show_in_face=True, wait_for_end_of=True)


                self.set_face("demo8")
                time.sleep(3)

                self.set_face(custom="clients_tem")
                time.sleep(3)

                # self.set_speech(filename="arm_close_gripper", command="", wait_for_end_of=True)
                # time.sleep(2)


                self.set_speech(filename="introduction_full", wait_for_end_of=True)

                self.set_face("help_pick_milk")
                time.sleep(3)

                self.set_face("help_pick_spoon")
                time.sleep(3)


                self.set_speech(filename="introduction_full", show_in_face=True, wait_for_end_of=True)
                # start = time.time()
                # while time.time() < start + 3: # in seconds
                #     pass
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