#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

import threading
import time

from example_interfaces.msg import String, Int16
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, SetFace
from sensor_msgs.msg import Image

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        ### Topics (Publisher and Subscribers) ###  
        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        self.save_speech_command_client = self.create_client(SaveSpeechCommand, "save_speech_command")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        while not self.save_speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        while not self.face_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Face Command...")

        # Variables
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_face = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.save_speech_success = False
        self.save_speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.face_success = True
        self.face_message = ""


    #### FACE SERVER FUNCTIONS #####
    def call_face_command_server(self, command="", custom="", wait_for_end_of=True):
        request = SetFace.Request()
        request.command = command
        request.custom = custom
        
        future = self.face_command_client.call_async(request)
        
        if wait_for_end_of:
            future.add_done_callback(self.callback_call_face_command)
        else:
            self.face_success = True
            self.face_message = "Wait for answer not needed"
    
    def callback_call_face_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.face_success = response.success
            self.face_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_face = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

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
            

    def call_save_speech_command_server(self, filename, command):
        request = SaveSpeechCommand.Request()
        request.filename = filename
        request.command = command

        future = self.save_speech_command_client.call_async(request)
        # print("Sent Command")

        future.add_done_callback(self.callback_call_save_speech_command)


    def callback_call_save_speech_command(self, future): #, a, b):
        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.save_speech_success = response.success
            self.save_speech_message = response.message
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

        return self.node.speech_success, self.node.speech_message
    

    def save_speech(self, filename, command):
        self.node.call_save_speech_command_server(filename=filename, command=command)
    
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message
    
    
    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        self.node.call_face_command_server(command=command, custom=custom, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
            while not self.node.waited_for_end_of_face:
                pass
        self.node.waited_for_end_of_face = False

        return self.node.face_success, self.node.face_message

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
                s, m = self.set_rgb(RED+MOON)
                print(s, m)
                success, message = self.set_speech(filename="arm/arm_close_gripper", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)
                self.set_rgb(BLUE+ROTATE)
                success, message = self.set_speech(filename="arm/introduction_full", command="", wait_for_end_of=False)
                print(success, message)
                time.sleep(5)
                self.set_rgb(WHITE+ALTERNATE_QUARTERS)
                success, message = self.set_speech(filename="arm/arm_close_grippe", command="", wait_for_end_of=True)
                print(success, message)
                time.sleep(5)

                """


                files = ["recep_characteristic_1", "recep_characteristic_2", "recep_characteristic_3", "recep_characteristic_4"]
                commands = ["The first guest shirt is black", "Its age is between 23 and 32", "The guest is a bit taller than me", "and its ethnicity is white."]
                self.save_speech(files, commands)
                print("...")
                time.sleep(5)
                print("...")


                self.set_speech(filename="temp/recep_characteristic_1", wait_for_end_of=True)
                

                # self.set_speech(command="Hello brother", wait_for_end_of=True)
                
                # self.set_speech(filename="generic/introduction_full", command="", wait_for_end_of=True)
                # time.sleep(2)

                self.set_speech(filename="receptionist/recep_drink_milk", command="", wait_for_end_of=True)
                self.set_face("help_pick_cup")
                time.sleep(3)


                # self.set_speech(filename="generic/introduction_ful", command="", wait_for_end_of=True)



                # self.set_speech(filename="generic/introduction_full", wait_for_end_of=True)
                
                # self.set_face(custom="clients_temp")
                # time.sleep(3)


                self.set_face("help_pick_bowl")
                time.sleep(3)

                self.set_speech(filename="receptionist/recep_drink_orange_juice", show_in_face=True, wait_for_end_of=True)


                self.set_face("demo8")
                time.sleep(3)

                self.set_face(custom="clients_tem")
                time.sleep(3)

                # self.set_speech(filename="arm/arm_close_gripper", command="", wait_for_end_of=True)
                # time.sleep(2)


                self.set_speech(filename="receptionist/recep_drink_red_wine", wait_for_end_of=True)

                self.set_face("help_pick_milk")
                time.sleep(3)

                self.set_face("help_pick_spoon")
                time.sleep(3)


                self.set_speech(filename="generic/introduction_full", show_in_face=True, wait_for_end_of=True)
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