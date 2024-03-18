#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Pose2D
from charmie_interfaces.srv import SpeechCommand, SetNeckPosition, GetNeckPosition, SetNeckCoordinates

import time

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class OpenDoorsDemoNode(Node):

    def __init__(self):
        super().__init__("OpenDoorsDemo")
        self.get_logger().info("Initialised CHARMIE OpenDoorsDemo Node")

        ### Topics (Publisher and Subscribers) ###  
        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)

        # Neck
        # self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        self.neck_to_coords_publisher = self.create_publisher(Pose2D, "neck_to_coords", 10)


        ### Services (Clients) ###
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")

        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")


        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")

        # Variables
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_get_neck = False

        # Sucess and Message confirmations for all set_(something) CHARMIE functions
        self.speech_sucess = True
        self.speech_message = ""
        self.neck_sucess = True
        self.neck_message = ""
        self.rgb_sucess = True
        self.rgb_message = ""
        self.face_sucess = True
        self.face_message = ""

        self.get_neck_position = [1.0, 1.0]
        
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

    #### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, position=[0, 0], wait_for_end_of=True):
        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])
        
        future = self.set_neck_position_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_sucess = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucess = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    #### SET NECK COORDINATES SERVER FUNCTIONS #####
    def call_neck_coordinates_server(self, x, y, z, tilt, flag, wait_for_end_of=True):
        request = SetNeckCoordinates.Request()
        request.coords.x = float(x)
        request.coords.y = float(y)
        request.coords.z = float(z)
        request.is_tilt = flag
        request.tilt = float(tilt)
        
        future = self.set_neck_coordinates_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_coords_command)
        else:
            self.neck_sucess = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_coords_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucess = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_coords = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    #### GET NECK POSITION SERVER FUNCTIONS #####
    def call_get_neck_position_server(self):
        request = GetNeckPosition.Request()
        
        future = self.get_neck_position_client.call_async(request)
        # print("Sent Command")

        # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
        future.add_done_callback(self.callback_call_get_neck_command)
    
    def callback_call_get_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info("Received Neck Position: (%s" %(str(response.pan) + ", " + str(response.tilt)+")"))
            self.get_neck_position[0] = response.pan
            self.get_neck_position[1] = response.tilt
            # time.sleep(3)
            self.waited_for_end_of_get_neck = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))     


def main(args=None):
    rclpy.init(args=args)
    node = OpenDoorsDemoNode()
    th_main = threading.Thread(target=thread_main_open_doors_demo, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_open_doors_demo(node: OpenDoorsDemoNode):
    main = OpenDoorsDemoMain(node)
    main.main()

class OpenDoorsDemoMain():

    def __init__(self, node: OpenDoorsDemoNode):
        self.node = node
        
        # Task Related Variables
        self.Waiting_for_task_start = 0
        self.Open_door_start = 1
        self.Approaching_cabinet_1 = 2
        self.Approaching_table = 3
        self.Picking_first_object = 4
        self.Approaching_cabinet_2 = 5
        self.Placing_first_object = 6
        self.Approaching_dishwasher = 7
        self.Open_dishwasher = 8
        self.Final_state = 9

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]

        # to debug just a part of the task you can just change the initial state, example:
        # self.state = self.Approach_kitchen_table
        self.state = self.Waiting_for_task_start
        
    ##### SETS #####

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_sucess, self.node.speech_message

    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_sucess, self.node.neck_message
    
    def set_neck_coords(self, position=[], ang=0.0, wait_for_end_of=True):

        #  x, y, z, tilt, flag, wait_for_end_of=True):
        self.node.get_logger().info("LENGTH %d"%len(position))

        if len(position) == 2:
            self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=0.0, tilt=ang, flag=True, wait_for_end_of=wait_for_end_of)
        elif len(position) == 3:
            print("You tried neck to coordintes using (x,y,z) please switch to (x,y,theta)")
            pass
            # The following line is correct, however since the functionality is not implemented yet, should not be called
            # self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=position[2], tilt=0.0, flag=False, wait_for_end_of=wait_for_end_of)
        else:
            print("Something went wrong")


        # self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        # if wait_for_end_of:
        #   while not self.node.waited_for_end_of_neck_coords:
        #     pass
        # self.node.waited_for_end_of_neck_coords = False

        return self.node.neck_sucess, self.node.neck_message
    
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

    ##### GETS #####
    def get_neck(self, wait_for_end_of=True):
    
        self.node.call_get_neck_position_server()
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False


        return self.node.get_neck_position[0], self.node.get_neck_position[1] 


    def main(self):

        print("IN NEW MAIN")
        # time.sleep(1)

        while True:

            if self.state == self.Waiting_for_task_start:
                #print('State 0 = Initial')

                self.set_face("demo5")

                self.set_speech(filename="open_doors_demo/od_ready_start", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                self.state = self.Open_door_start
            
            elif self.state == self.Open_door_start:
                #print('State 1 = Open door start')

                	### IF DOOR AND HANDLER WERE IDENTIFIED:
                self.set_speech(filename="open_doors_demo/door_detection", show_in_face=True, wait_for_end_of=True)
                
                ### SHOW IMAGE WITH DETECTION IN FACE
                
                self.set_speech(filename="open_doors_demo/door_opening", show_in_face=True, wait_for_end_of=True)
                
                ### MOVE TO THE DOOR
                
                self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=True)
                
                ### ARM IN POSITION OF OPENING DOOR
                
                ### OPEN DOOR
                
                ### ELSE: KEEP MOVING NECK TO SEARCH DOOR
                
                ### I MUST FIND THE BEST PLACE TO VERIFY IF THE DOOR WAS REALLY OPENED
                
                # next state
                self.state = self.Approaching_cabinet_1
            
            elif self.state == self.Approaching_cabinet_1:
                #print('State 2 = Approaching cabinet for the first time')
                
                ###### NAVIGATE TO THE CABINET
                
                elf.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)
                
                ### NECK TO CABINET
                
                	### IF CABINET DOOR IS DETECTED:

                self.set_speech(filename="open_doors_demo/cabinet_detection", show_in_face=True, wait_for_end_of=True)
                
                ### SHOW IMAGE WITH DETECTION IN FACE
                
                self.set_speech(filename="open_doors_demo/cabinet_opening", show_in_face=True, wait_for_end_of=True)
                
                	### ELSE: KEEP MOVING NECK
                
                self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=True)
                
                ### ARM IN POSITION OF OPENING CABINET
                
                ### OPEN CABINET
                
					### IF DOOR IS OPENED:                         
                self.state = self.Approaching_table
                
					### ELSE: STATE = CURRENT
                
            elif self.state == self.Approaching_table:
                #print('State 3 = Approaching table')

                # self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                ###### MOVEMENT TO THE TABLE 

                self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)
                                
                # next state
                self.state = self.Picking_first_object
                
            elif self.state == self.Picking_first_object:
                #print('State 4 = Picking first object')

                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### MOVES ARM TO TOP OF TABLE POSITION

                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)

                ##### YOLO OBJECTS SEARCH FOR THE OBJECTS PREDETERMINED, FOR BOTH CAMERAS

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                self.set_speech(filename="open_doors_demo/found_object", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/check_face_objects_detected", wait_for_end_of=True)

                ##### SHOW FACE DETECTED OBJECTS

                ##### MOVE ARM TO PICK UP OBJECT 

                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### IF AN ERROR IS DETECTED:
                
                self.set_speech(filename="generic/problem_pick_object", wait_for_end_of=True) # False
                   
                    ##### MOVE ARM TO ERROR POSITION 
                
                # self.set_neck(position=self.look_judge, wait_for_end_of=True)
                
                self.set_speech(filename="generic/check_face_put_object_hand", wait_for_end_of=True)

                self.set_face("help_pick_spoon") #change to the object detected...

                time.sleep(2)
                
                    ##### WHILE OBJECT IS NOT IN GRIPPER:
                
                self.set_speech(filename="arm/arm_close_gripper", wait_for_end_of=True)

                        ##### ARM CLOSE GRIPPER

                        ##### IF OBJECT NOT GRABBED:
                
                self.set_speech(filename="arm/arm_error_receive_object", wait_for_end_of=True)
                        
                            ##### ARM OPEN GRIPPER
                
                self.set_face("demo5")
                        
                # self.set_neck(position=self.look_tray, wait_for_end_of=True)
                
                ##### ARM PLACE FIRST OBJECT IN TRAY
                                
                # next state
                self.state = self.Approaching_cabinet_2
                
            """ elif self.state == self.Approaching_cabinet_2:
                #print('State 0 = Initial')

                self.set_face("demo5")

                self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                self.state = self.Placing_first_object
                
            elif self.state == self.Placing_first_object:
                #print('State 0 = Initial')

                self.set_face("demo5")

                self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                self.state = self.Approaching_dishwasher
                
            elif self.state == self.Approaching_dishwasher:
                #print('State 0 = Initial')

                self.set_face("demo5")

                self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                self.state = self.Open_dishwasher
                
            elif self.state == self.Open_dishwasher:
                #print('State 0 = Initial')

                self.set_face("demo5")

                self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                self.state = self.Final_state
            
            elif self.state == self.Final_state:
                #print('State 15 = Finished task')
                # self.node.speech_str.command = "I have finished my storing groceries task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")
                
                # self.set_neck(position=self.look_judge) # , wait_for_end_of=True)

                self.set_speech(filename="open_doors_demo/od_finished", wait_for_end_of=True)

                while True:
                    pass 
            else:
                pass"""