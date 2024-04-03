#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import threading

from example_interfaces.msg import Bool, String, Int16
from geometry_msgs.msg import Pose2D
from charmie_interfaces.srv import SpeechCommand, SetNeckPosition, GetNeckPosition, SetNeckCoordinates
from charmie_interfaces.msg import Yolov8Objects
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import time

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106



class StoringGroceriesNode(Node):

    def __init__(self):
        super().__init__("StoringGroceries")
        self.get_logger().info("Initialised CHARMIE StoringGroceries Node")

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

        # Objects detected
        self.objects_filtered_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered', self.get_objects_callback, 10)

        ### CHECK IF ALL SERVICES ARE RESPONSIVE ###
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        # while not self.set_neck_coordinates_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        
        # Speakers
        """ while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...") """

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

    def get_objects_callback(self, objects: Yolov8Objects):
        #print(objects.objects)
        self.nr_objects = objects.num_objects
        self.objects = objects.objects
        self.image = objects.image_rgb
        
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
    node = StoringGroceriesNode()
    th_main = threading.Thread(target=thread_main_storing_groceries, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_storing_groceries(node: StoringGroceriesNode):
    main = StoringGroceriesMain(node)
    main.main()

class StoringGroceriesMain():

    def __init__(self, node: StoringGroceriesNode):
        self.node = node
        
        # Task Related Variables
        self.Waiting_for_task_start = 0
        # self.Open_cabinet_door = 1
        self.Approach_tables_first_time = 1
        self.Picking_first_object = 2
        self.Picking_second_object = 3
        self.Picking_third_object = 4
        self.Approach_cabinet_first_time = 5
        self.Placing_third_object = 6
        self.Placing_second_object = 7
        self.Placing_first_object = 8
        self.Approach_tables_second_time = 9
        self.Picking_fourth_object = 10
        self.Picking_fifth_object = 11
        self.Approach_cabinet_second_time = 12
        self.Placing_fifth_object = 13
        self.Placing_fourth_object = 14
        self.Final_State = 15

        # Neck Positions
        self.look_forward = [0, 0]
        self.look_navigation = [0, -30]
        self.look_judge = [45, 0]
        self.look_table_objects = [-45, -45]
        self.look_tray = [0, -60]
        self.look_cabinet_top = [-45, 45]
        self.look_cabinet_center = [-45, 0]
        self.look_cabinet_bottom = [-45, -45]

        self.shelf_1_height = 0.18 #0.97
        self.shelf_2_height = 0.66 #1.39
        self.shelf_3_height = 1.17 #1.81

        self.left_limit_shelf = -0.35
        self.right_limit_shelf = 0.65
        self.center_shelf = 0.2


        # to debug just a part of the task you can just change the initial state, example:
        # self.state = self.Approach_kitchen_table
        self.state = self.Waiting_for_task_start

        self.nr_objects_detected_previous = 0
        self.nr_max_objects_detected = 0
        self.image_most_obj_detected = Image()
        self.prev_time = 0.0
        self.new_time = 0.0

        self.names_counter = 0
        self.objects_names_list = [""]
        self.object_details = {}
        
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
    
    def get_caracteristics_image(self):
        i = 0
        if hasattr(self.node, 'objects') and self.node.objects:
            objects_stored = self.node.objects
            print(len(objects_stored))
            while i < self.node.nr_objects:
                    if objects_stored and i < len(objects_stored):
                        #print(self.node.objects[i])
                        #print('ATÉ AQUI NÃO CRASHOU')
                        detected_object = objects_stored[i]
                        object_name = detected_object.object_name
                        object_height = detected_object.position_relative.z
                        object_confidence = detected_object.confidence
                        print("Object Name: ", object_name)
                        print(f"Object height: {object_height:.2f}")
                        print(f"Object confidence: {object_confidence:.2f}")
                        #print(i)
                        #print(objects_stored[i])
                        print('\n')
                        i += 1

                    else:
                        break
                    
                    #time.sleep(2)
            print('Image read \n\n')

        else:
            print('No objects detected \n \n')    

    
    def store_frame_more_detections(self):
        i = 0
        #if hasattr(self.node, 'image') and self.node.image:
        if hasattr(self.node, 'image') and self.node.image:
            self.current_image = self.node.image
            if hasattr(self.node, 'objects') and self.node.objects:
                objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects
                if self.nr_objects_detected > self.nr_objects_detected_previous:
                    self.nr_max_objects_detected = self.nr_objects_detected
                    self.nr_objects_detected_previous = self.nr_objects_detected

                    print(self.nr_objects_detected)
                    
                    # Create a CvBridge object
                    bridge = CvBridge()
                    # Convert ROS Image to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                    self.image_most_obj_detected = cv_image

                    
                    while i < self.nr_objects_detected:
                        if objects_stored and i < len(objects_stored):
                            #print(self.node.objects[i])
                            #print('ATÉ AQUI NÃO CRASHOU')
                            detected_object = objects_stored[i]
                            object_name = detected_object.object_name
                            object_height = detected_object.position_relative.z
                            object_confidence = detected_object.confidence
                            print("Object Name: ", object_name)
                            print(f"Object height: {object_height:.2f}")
                            print(f"Object confidence: {object_confidence:.2f}")
                            #print(i)
                            #print(objects_stored[i])
                            print('\n')
                            i += 1

                        else:
                            break
                
    ##### ANALYSE CABINET #####

    def analysis_cabinet(self):

        i = 0
        if hasattr(self.node, 'image') and self.node.image:
            if hasattr(self.node, 'objects') and self.node.objects:
                objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects
                while i < self.nr_objects_detected:                    
                    detected_object = objects_stored[i]
                    object_name = detected_object.object_name
                    object_height = detected_object.position_relative.z
                    object_confidence = detected_object.confidence
                    object_x_position = detected_object.position_relative.x
                    box_top_left_x = detected_object.box_top_left_x
                    box_top_left_y = detected_object.box_top_left_y
                    box_width = detected_object.box_width
                    box_height = detected_object.box_height
                    #print(f"Object: {object_name}, Height: {object_height}, Confidence: {object_confidence}")
                    if object_name in self.object_details:
                        pass
                    else:
                        self.object_details[object_name] = {'height': object_height, 'confidence': object_confidence, 'object_height': object_height,
                                                            'x_position': object_x_position, 'box_top_left_x': box_top_left_x,
                                                            'box_top_left_y': box_top_left_y, 'box_width': box_width, 'box_height': box_height}

                        print("After updating self.object_details")
                        print(self.object_details)

                        start_point = (box_top_left_x, box_top_left_y)
                        end_point = (box_top_left_x + box_width, box_top_left_y + box_height)
                        cv2.rectangle(self.image_most_obj_detected, start_point, end_point, (56, 56, 255) , 4) 

                        if object_x_position < self.left_limit_shelf or object_x_position > self.right_limit_shelf:
                            print(object_name, 'is outside the wardrobe')
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Outside wardrobe',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        ) 
                        
                        elif object_height < self.shelf_1_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            print(object_name, 'is on the floor')
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Floor',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        ) 
                            
                        elif self.shelf_1_height < object_height < self.shelf_2_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            print(object_name, 'is in the first shelf')
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'First shelf',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        ) 

                        elif self.shelf_2_height < object_height < self.shelf_3_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            print(object_name, 'is in the second shelf')
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Second shelf',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        ) 

                        elif object_height > self.shelf_3_height and self.left_limit_shelf < object_x_position < self.right_limit_shelf :
                            print(object_name, 'is in the third shelf')
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Third shelf',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        ) 
                            
                        self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            f"{object_name}",
                            (box_top_left_x, box_top_left_y),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (255, 0, 0),
                            1,
                            cv2.LINE_AA
                        ) 


                    i += 1

                #self.objects_names_list.clear()

    def choose_priority(self):
        i = 0
        if hasattr(self.node, 'image') and self.node.image:
            if hasattr(self.node, 'objects') and self.node.objects:
                objects_stored = self.node.objects
                self.nr_objects_detected = self.node.nr_objects
                while i < self.nr_objects_detected:                    
                    detected_object = objects_stored[i]
                    object_name = detected_object.object_name
                    object_class = detected_object.object_class
                    object_height = detected_object.position_relative.z
                    object_confidence = detected_object.confidence
                    object_x_position = detected_object.position_relative.x
                    box_top_left_x = detected_object.box_top_left_x
                    box_top_left_y = detected_object.box_top_left_y
                    box_width = detected_object.box_width
                    box_height = detected_object.box_height
                    #print(f"Object: {object_name}, Height: {object_height}, Confidence: {object_confidence}")
                    if object_name in self.object_details:
                        pass
                    else:
                        if object_class == 'Drinks' or object_class == 'Foods' or object_class == 'Snacks':
                            object_priority = 'High'
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'High priority',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        )
                            
                        elif object_class == 'Toys' or object_class == 'Cleaning Supplies':
                            object_priority = 'Medium'
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Medium priority',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        )
                        
                        elif object_class == 'Dishes' or object_class == 'Fruits':
                            object_priority = 'Low'
                            self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            'Low priority',
                            (box_top_left_x, box_top_left_y + box_height),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA
                        )

                        self.object_details[object_name] = {'height': object_height, 'confidence': object_confidence, 'object_height': object_height,
                                                            'object_class': object_class,'x_position': object_x_position, 'box_top_left_x': box_top_left_x,
                                                            'box_top_left_y': box_top_left_y, 'box_width': box_width, 'box_height': box_height, 'priority': object_priority}

                        #print("After updating self.object_details")
                        #print(self.object_details)

                        start_point = (box_top_left_x, box_top_left_y)
                        end_point = (box_top_left_x + box_width, box_top_left_y + box_height)
                        cv2.rectangle(self.image_most_obj_detected, start_point, end_point, (56, 56, 255) , 4) 

                        self.image_most_obj_detected = cv2.putText(
                            self.image_most_obj_detected,
                            # f"{round(float(per.conf),2)}",
                            f"{object_confidence:.2f}",
                            (box_top_left_x, box_top_left_y),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (255, 0, 0),
                            1,
                            cv2.LINE_AA
                        ) 


                    i += 1

    def main(self):

        print("IN NEW MAIN")
        # time.sleep(1)

        while True:

            if self.state == self.Waiting_for_task_start:
                #print('State 0 = Initial')

                self.set_face("demo5")

                # self.set_speech(filename="storing_groceries/sg_ready_start", show_in_face=True, wait_for_end_of=True)

                # self.set_speech(filename="generic/waiting_start_button", show_in_face=True, wait_for_end_of=True) # must change to door open

                ###### WAITS FOR START BUTTON / DOOR OPEN

                time.sleep(2)
                                
                # next state
                # self.state = self.Approach_tables_first_time
                j = 0
                self.state = 16

            elif self.state == self.Approach_tables_first_time:
                #print('State 1 = Approaching table for the first time')

                # self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                ###### MOVEMENT TO THE KITCHEN COUNTER

                self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)
                
                # next state
                self.state = self.Picking_first_object
                
            elif self.state == self.Picking_first_object:
                #print('State 2 = Picking first object from table')

                # self.set_neck(position=self.look_table_objects, wait_for_end_of=True)

                ##### MOVES ARM TO TOP OF TABLE POSITION

                self.set_speech(filename="generic/search_objects", wait_for_end_of=True)

                ##### YOLO OBJECTS SEARCH FOR THE FIVE OBJECTS WITH HIGHER CONFIDENCE, FOR BOTH CAMERAS

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

                self.set_speech(filename="storing_groceries/sg_found_five_objects", show_in_face=True, wait_for_end_of=True)

                self.set_speech(filename="storing_groceries/check_face_objects_detected", wait_for_end_of=True)

                ##### SHOW FACE DETECTED OBJECTS
                # self.set_face("objects_detected")

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
                self.state = self.Picking_second_object
                
            elif self.state == self.Picking_second_object:
                #print('State 3 = Picking second object from table')

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

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
                
                ##### ARM PLACE SECOND OBJECT IN TRAY
                                
                # next state
                self.state = self.Picking_third_object
                
            elif self.state == self.Picking_third_object:
                #print('State 4 = Picking third object from table')

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

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
                
                ##### ARM PLACE THIRD OBJECT IN HAND - REST POSITION
                                
                # next state
                self.state = self.Approach_cabinet_first_time
                
            elif self.state == self.Approach_cabinet_first_time:
                #print('State 5 = Approaching cabinet for the first time')

                self.set_speech(filename="storing_groceries/sg_collected_objects_1st_round", wait_for_end_of=True)
                
                # self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ###### MOVEMENT TO THE CABINET

                self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)

                # self.set_neck(position=self.look_cabinet_top, wait_for_end_of=True)
                # self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)
                # self.set_neck(position=self.look_cabinet_bottom, wait_for_end_of=True)
                
                self.set_speech(filename="storing_groceries/sg_analysing_cabinet", wait_for_end_of=True)
                
                self.set_speech(filename="storing_groceries/sg_finished_analise_cabinet", wait_for_end_of=True) 

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)
                
                self.set_speech(filename="storing_groceries/sg_check_face_cabinet_distribution", wait_for_end_of=True) 
                
                ####### SHOW IMAGE OF CABINET DIVIDED INTO ZONES /  CLASSES
                
                self.set_speech(filename="generic/place_object_cabinet", wait_for_end_of=True)

                self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=True)
                
                # next state
                self.state = self.Placing_third_object
                
            elif self.state == self.Placing_third_object:
                #print('State 6 = Placing third object grabbed in the cabinet')

                self.set_speech(filename="storing_groceries/sg_place_object_shown_face", wait_for_end_of=True) 

                # self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)

                time.sleep(1)

                ##### ARM MOVE TO CABINET

                ##### ARM PLACE OBJECT

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=True)
                                
                # next state
                self.state = self.Placing_second_object
                
            elif self.state == self.Placing_second_object:
                #print('State 7 = Placing second object grabbed in the cabinet')

                self.set_speech(filename="storing_groceries/sg_place_object_shown_face", wait_for_end_of=True) 
                                
                time.sleep(1)

                ##### ARM MOVE TO CABINET

                ##### ARM PLACE OBJECT

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=True)
                                
                # next state
                self.state = self.Placing_first_object
                
            elif self.state == self.Placing_first_object:
                #print('State 8 = Placing first object grabbed in the cabinet')

                self.set_speech(filename="storing_groceries/sg_place_object_shown_face", wait_for_end_of=True) 
                                
                time.sleep(1)

                ##### ARM MOVE TO CABINET

                ##### ARM PLACE OBJECT

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=True)
                                
                # next state
                self.state = self.Approach_tables_second_time
                
            elif self.state == self.Approach_tables_second_time:
                #print('State 9 = Approaching table for the second time to grab rest of the objects')

                # self.set_neck(position=self.look_navigation) # , wait_for_end_of=True)

                self.set_speech(filename="generic/moving_table", wait_for_end_of=True)

                ###### MOVEMENT TO THE TABLE

                self.set_speech(filename="generic/arrived_table", wait_for_end_of=True)
                                
                # next state
                self.state = self.Picking_fourth_object
                
            elif self.state == self.Picking_fourth_object:
                #print('State 10 = Picking fourth object from table')

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

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
                
                ##### ARM PLACE FOURTH OBJECT IN TRAY
                                
                # next state
                self.state = self.Picking_fifth_object
                
            elif self.state == self.Picking_fifth_object:
                #print('State 11 = Picking fifth object from table')

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)

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
                
                ##### ARM PLACE FIFTH OBJECT IN TRAY
                                
                # next state
                self.state = self.Approach_cabinet_second_time
                
            elif self.state == self.Approach_cabinet_second_time:
                #print('State 12 = Approaching cabinet door for the second time')

                self.set_speech(filename="storing_groceries/sg_collected_objects_2nd_round", wait_for_end_of=True)
                
                # self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                ###### MOVEMENT TO THE CABINET

                self.set_speech(filename="generic/arrived_cabinet", wait_for_end_of=True)

                # self.set_neck(position=self.look_judge, wait_for_end_of=True)
                
                self.set_speech(filename="generic/place_object_cabinet", wait_for_end_of=True)

                self.set_speech(filename="generic/place_stay_clear", wait_for_end_of=True)
                                
                # next state
                self.state = self.Placing_fifth_object
                
            elif self.state == self.Placing_fifth_object:
                #print('State 13 = Placing fifth object grabbed')

                # self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)
                                
                time.sleep(1)

                ##### ARM MOVE TO CABINET

                ##### ARM PLACE OBJECT

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=True)
                                
                # next state
                self.state = self.Placing_fourth_object
                
            elif self.state == self.Placing_fourth_object:
                #print('State 14 = Placing forth object grabbed')

                # self.set_neck(position=self.look_cabinet_center, wait_for_end_of=True)
                                
                time.sleep(1)

                ##### ARM MOVE TO CABINET

                ##### ARM PLACE OBJECT

                self.set_speech(filename="generic/place_object_placed", wait_for_end_of=True)
                                
                # next state
                self.state = self.Final_State
            
            elif self.state == self.Final_State:
                #print('State 15 = Finished task')
                # self.node.speech_str.command = "I have finished my storing groceries task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")
                
                # self.set_neck(position=self.look_judge) # , wait_for_end_of=True)

                self.set_speech(filename="storing_groceries/sg_finished", wait_for_end_of=True)

                while True:
                    pass

            else:
                # self.get_caracteristics_image()
                start_time = time.time()

                self.current_image = self.node.image
                bridge = CvBridge()
                # Convert ROS Image to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding="bgr8")
                self.image_most_obj_detected = cv_image

                while time.time() - start_time < 1.0:
                    # self.store_frame_more_detections()
                    # self.analysis_cabinet()
                    self.choose_priority()

                print(' ---------------------- ')

                # Sort objects by confidence in descending order
                sorted_objects = sorted(self.object_details.items(), key=lambda x: x[1]['confidence'], reverse=True)
                print('Sorted: ',sorted_objects)

                # Filter objects with confidence higher than 0.5
                filtered_objects = [(name, details) for name, details in sorted_objects if details['confidence'] > 0.5]
                print('Filtered: ', filtered_objects)

                # Initialize selected objects list
                selected_objects = []

                # Select objects with higher confidence and higher priority
                for name, details in filtered_objects:
                    if len(selected_objects) == 5:
                        break
                    if details.get('priority') == 'High':
                        selected_objects.append((name, details))

                

                # If there are not enough high priority objects, select from remaining objects
                if len(selected_objects) < 5:
                    remaining_count = 5 - len(selected_objects)
                    selected_objects.extend(filtered_objects[len(selected_objects):len(selected_objects) + remaining_count])

                print('Selected: ', selected_objects)

                for name, details in selected_objects:
                    box_top_left_x = details['box_top_left_x']
                    box_top_left_y = details['box_top_left_y']
                    box_width = details['box_width']
                    box_height = details['box_height']
                    
                    # Calculate end point of the rectangle
                    end_point = (box_top_left_x + box_width, box_top_left_y + box_height)
                    
                    # Draw rectangle on the original image
                    cv2.rectangle(cv_image, (box_top_left_x, box_top_left_y), end_point, (0, 255, 0), 2)
                
                #print('\n\n\n\n\n\n\n\n\n')
                #print('\n\n\n\n\n\n\n\n\n')
                #print('\n\n\n\n\n\n\n\n\n')
                #print('\n\n\n\n\n\n\n\n\n')

                #print(self.object_details)

                # print(self.objects_names_list)


                if self.image_most_obj_detected is not None:
                    print('Mostra imagem')
                    cv2.imshow('Image with more objects detected', self.image_most_obj_detected)
                    cv2.waitKey(1)
                else:
                    print("Error: self.image_most_obj_detected is None")

                self.object_details.clear()


                #pass