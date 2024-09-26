#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from geometry_msgs.msg import Point, Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, BoundingBox, BoundingBoxAndPoints, ListOfDetectedObject
from charmie_interfaces.srv import GetPointCloud, ActivateYoloObjects
from cv_bridge import CvBridge
import cv2 
import json
import threading

from pathlib import Path

import math
import time

objects_filename = "segmentation_M_size_model_600_epochs.pt"
# objects_filename = "epoch20.pt"
# objects_filename = "new_best.pt"
# objects_filename = "new_best_2.pt"
# objects_filename = "detect_hands_2.pt"
# objects_filename = "50_epochs.pt"
# objects_filename = "slender_ycb_03_07_2024_v1.pt"
# objects_filename = "lar_dataset_post_fnr2024.pt"
shoes_filename = "shoes_socks_v1.pt"
doors_filename = "door_bruno_2.pt"
# doors_filename = "furniture_robocup.pt"

MIN_OBJECT_CONF_VALUE = 0.5
MIN_SHOES_CONF_VALUE = 0.5
MIN_DOORS_CONF_VALUE = 0.5

DRAW_OBJECT_CONF = True
DRAW_OBJECT_ID = True
DRAW_OBJECT_BOX = True
DRAW_OBJECT_NAME = True
DRAW_OBJECT_CLASS = True
DRAW_OBJECT_LOCATION_COORDS = True
DRAW_OBJECT_LOCATION_HOUSE_FURNITURE = False


class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")

         ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("debug_draw", False) 
        self.declare_parameter("activate_objects", False)
        self.declare_parameter("activate_shoes", False)
        self.declare_parameter("activate_doors", False)
        self.declare_parameter("activate_objects_hand", False)
        self.declare_parameter("activate_shoes_hand", False)
        self.declare_parameter("activate_doors_hand", False)
    
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects"
        self.complete_path = self.home+'/'+self.midpath+'/'

        self.midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+self.midpath_configuration_files+'/'

        # Opens files with objects names and categories
        try:
            with open(self.complete_path_configuration_files + 'objects_lar.json', encoding='utf-8') as json_file:
                self.objects_file = json.load(json_file)
            # print(self.objects_file) """
            # with open(self.complete_path_configuration_files + 'objects_robocup.json', encoding='utf-8') as json_file:
            #     self.objects_file = json.load(json_file)
            # print(self.objects_file)
            with open(self.complete_path_configuration_files + 'rooms_location.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.complete_path_configuration_files + 'furniture_location.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)
            self.get_logger().info("Successfully Imported data from json configuration files. (objects_list, house_rooms and house_furniture)")
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.DEBUG_DRAW = self.get_parameter("debug_draw").value
        
        # HEAD
        # whether the activate objects flag starts as ON or OFF 
        self.ACTIVATE_YOLO_OBJECTS = self.get_parameter("activate_objects").value
        # whether the activate shoes flag starts as ON or OFF 
        self.ACTIVATE_YOLO_SHOES = self.get_parameter("activate_shoes").value
        # whether the activate doors flag starts as ON or OFF 
        self.ACTIVATE_YOLO_DOORS = self.get_parameter("activate_doors").value

        # HAND
        # whether the activate objects flag starts as ON or OFF 
        self.ACTIVATE_YOLO_OBJECTS_HAND = self.get_parameter("activate_objects_hand").value
        # whether the activate shoes flag starts as ON or OFF 
        self.ACTIVATE_YOLO_SHOES_HAND = self.get_parameter("activate_shoes_hand").value
        # whether the activate doors flag starts as ON or OFF 
        self.ACTIVATE_YOLO_DOORS_HAND = self.get_parameter("activate_doors_hand").value

        print(self.home+'/'+objects_filename)

        # Import the models, one for each category
        self.object_model = YOLO(self.home+'/'+objects_filename)
        self.shoes_model = YOLO(self.complete_path + shoes_filename)
        self.doors_model = YOLO(self.complete_path + doors_filename)
        # it needs to have a different model for head and hand image because of the track parameter, otherwise it is always creating new track ids
        self.object_model_hand = YOLO(self.home+'/'+objects_filename)
        self.shoes_model_hand = YOLO(self.complete_path + shoes_filename)
        self.doors_model_hand = YOLO(self.complete_path + doors_filename)

        print("Sucessfully imported YOLO models")

        ### Topics ###
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
         
        # Publish Results
        self.objects_filtered_publisher = self.create_publisher(ListOfDetectedObject, 'objects_all_detected_filtered', 10)
        self.objects_filtered_hand_publisher = self.create_publisher(ListOfDetectedObject, 'objects_all_detected_filtered_hand', 10)
        
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        ### Services ###
        self.activate_yolo_objects_service = self.create_service(ActivateYoloObjects, "activate_yolo_objects", self.callback_activate_yolo_objects)

        ### Variables ###

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.hand_rgb = Image()
        self.new_head_rgb = False
        self.new_hand_rgb = False
        self.waiting_for_pcloud = False
        self.point_cloud_response = GetPointCloud.Response()

        self.objects_class_names = ['7up', 'Apple', 'Bag', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser',
                                   'Coffee_grounds', 'Cola', 'Cornflakes', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_Tea', 
                                   'Juice_pack', 'Knife', 'Lemon', 'Milk', 'Mustard', 'Orange', 'Orange_juice', 'Peach', 'Pear',                                  
                                   'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube', 'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 
                                   'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup', 'Tropical_juice', 'Tuna', 'Water']
        
        # Objects robocup 24
        # self.objects_class_names = ['Apple', 'Bag', 'Banana', 'Big_coke', 'Bowl', 'Candle', 'Candy', 'Cola', 'Cornflakes', 'Crisps', 'Cup', 
        #                             'Curry', 'Dishwasher_tab', 'Dubbelfris', 'Fanta', 'Fork', 'Hagelslag', 'Ice_tea', 'Knife', 'Lemon', 'Liquorice', 
        #                             'Mayonaise', 'Milk', 'Orange', 'Pancake_mix', 'Pea_soup', 'Peach', 'Pear', 'Plate', 'Plum', 'Pringles', 'Soap', 
        #                             'Sponge', 'Spoon', 'Strawberry', 'Stroopwafel', 'Suasages', 'Tictac', 'Washcloth', 'Water']
        
        # Secondary declaration used for debug
        # self.objects_class_names = ['Apple', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser', 'Coffee_grounds', 'Cola',
        #                              'Cornflakes', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_tea', 'Juice_pack', 'Knife', 'Lemon', 'Milk',
        #                                'Mustard', 'Orange', 'Orange_juice', 'Peach', 'Pear', 'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube',
        #                                  'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup',
        #                                    'Tropical_juice', 'Tuna']
        
        self.shoes_class_names = ['Shoe', 'Sock']
        
        self.doors_class_names = ['Cabinet', 'Dishwasher', 'Door', 'Drawer', 'LevelHandler', 'Wardrobe_Door']
        # self.doors_class_names = ['Cabinet', 'Dishwasher']

        self.objects_class_names_dict = {}
        self.objects_class_names_dict = {item["name"]: item["class"] for item in self.objects_file}
    

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req, camera):
        request = GetPointCloud.Request()
        request.data = req
        request.retrieve_bbox = False
        request.camera = camera
    
        future = self.point_cloud_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud)

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_response = future.result()
            self.waiting_for_pcloud = False
            # print("Received Back")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_activate_yolo_objects(self, request, response):
        
        # Type of service received: 
        # bool activate_objects                       # activate or deactivate yolo object detection
        # bool activate_shoes                         # activate or deactivate yolo shoes detection
        # bool activate_doors                         # activate or deactivate yolo doors detection (includes doors, drawers, washing machine door, closet with doors)
        # bool activate_objects_hand                  # activate or deactivate hand yolo object detection
        # bool activate_shoes_hand                    # activate or deactivate hand yolo shoes detection
        # bool activate_doors_hand                    # activate or deactivate hand yolo doors detection (includes doors, drawers, washing machine door, closet with doors)
        # float64 minimum_objects_confidence          # adjust the minimum accuracy to assume as an object
        # float64 minimum_shoes_confidence            # adjust the minimum accuracy to assume as a shoe
        # float64 minimum_doors_confidence            # adjust the minimum accuracy to assume as a door or handle
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        global MIN_OBJECT_CONF_VALUE, MIN_SHOES_CONF_VALUE, MIN_DOORS_CONF_VALUE

        self.get_logger().info("Received Activate Yolo Objects %s" %("("+str(request.activate_objects)+", "
                                                                        +str(request.activate_shoes)+", "
                                                                        +str(request.activate_doors)+", "
                                                                        +str(request.activate_objects_hand)+", "
                                                                        +str(request.activate_shoes_hand)+", "
                                                                        +str(request.activate_doors_hand)+", "
                                                                        +str(request.minimum_objects_confidence)+", "
                                                                        +str(request.minimum_shoes_confidence)+", "
                                                                        +str(request.minimum_doors_confidence)+")"))

        self.ACTIVATE_YOLO_OBJECTS = request.activate_objects
        self.ACTIVATE_YOLO_SHOES = request.activate_shoes
        self.ACTIVATE_YOLO_DOORS = request.activate_doors
        self.ACTIVATE_YOLO_OBJECTS_HAND = request.activate_objects_hand
        self.ACTIVATE_YOLO_SHOES_HAND = request.activate_shoes_hand
        self.ACTIVATE_YOLO_DOORS_HAND = request.activate_doors_hand
        MIN_OBJECT_CONF_VALUE = request.minimum_objects_confidence
        MIN_SHOES_CONF_VALUE = request.minimum_shoes_confidence
        MIN_DOORS_CONF_VALUE = request.minimum_doors_confidence
        
        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response

    def get_color_image_hand_callback(self, img: Image):
        self.hand_rgb = img
        self.new_hand_rgb = True

    def get_color_image_head_callback(self, img: Image):
        self.head_rgb = img
        self.new_head_rgb = True

    def add_object_to_detectedobject_msg(self, boxes_id, object_name, object_class, center_object_coordinates, camera):

        object_id = boxes_id.id
        if boxes_id.id == None:
            object_id = 0 

        new_object = DetectedObject()

        new_object.object_name = object_name
        new_object.object_class = object_class
        new_object.index = int(object_id)
        new_object.confidence = float(boxes_id.conf)

        # print(center_object_coordinates)

        # changes the axis of point cloud coordinates to fit with robot axis
        object_rel_pos = Point()
        object_rel_pos.x =  -center_object_coordinates.y/1000
        object_rel_pos.y =  center_object_coordinates.x/1000
        object_rel_pos.z =  center_object_coordinates.z/1000
        new_object.position_relative = object_rel_pos
        
        # calculate the absolute position according to the robot localisation
        angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
        dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_obj - self.robot_t)

        target_x = dist_obj * math.cos(theta_aux) + self.robot_x
        target_y = dist_obj * math.sin(theta_aux) + self.robot_y

        a_ref = (target_x, target_y)
        # print("Rel:", (object_rel_pos.x, object_rel_pos.y), "Abs:", a_ref)

        object_abs_pos = Point()
        object_abs_pos.x = target_x
        object_abs_pos.y = target_y
        object_abs_pos.z = center_object_coordinates.z/1000
        new_object.position_absolute = object_abs_pos

        new_object.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_object.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_object.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_object.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        new_object.room_location, new_object.furniture_location = self.position_to_house_rooms_and_furniture(object_abs_pos)

        new_object.box_center_x = new_object.box_top_left_x + new_object.box_width//2
        new_object.box_center_y = new_object.box_top_left_y + new_object.box_height//2

        new_object.camera = camera

        new_object.orientation = 0.0 # still missing... (says the object angle so the gripper can adjust to correctly pick up the object)
        
        if camera == "head":
            new_object.image_rgb_frame = self.head_rgb
        else:
            new_object.image_rgb_frame = self.hand_rgb
            
        return new_object

    def position_to_house_rooms_and_furniture(self, person_pos):
        
        room_location = "Outside"
        for room in self.house_rooms:
            min_x = room['top_left_coords'][0]
            max_x = room['bot_right_coords'][0]
            min_y = room['bot_right_coords'][1]
            max_y = room['top_left_coords'][1]
            # print(min_x, max_x, min_y, max_y)
            if min_x < person_pos.x < max_x and min_y < person_pos.y < max_y:
                room_location = room['name'] 

        furniture_location = "None"
        for furniture in self.house_furniture:
            min_x = furniture['top_left_coords'][0]
            max_x = furniture['bot_right_coords'][0]
            min_y = furniture['bot_right_coords'][1]
            max_y = furniture['top_left_coords'][1]
            # print(min_x, max_x, min_y, max_y)
            if min_x < person_pos.x < max_x and min_y < person_pos.y < max_y:
                furniture_location = furniture['name'] 

        return room_location, furniture_location
 
    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta

        
# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    th_main = threading.Thread(target=ThreadMainYoloObjects, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainYoloObjects(node: Yolo_obj):
    main = YoloObjectsMain(node)
    main.main()


class YoloObjectsMain():

    def __init__(self, node: Yolo_obj):
        # create a node instance so all variables ros related can be acessed
        self.node = node

        self.new_head_frame_time = time.time()
        self.prev_head_frame_time = time.time()
        self.new_hand_frame_time = time.time()
        self.prev_hand_frame_time = time.time()
        
    def detect_with_yolo_model(self, model, camera, current_frame_draw):

        # self.get_logger().info('Receiving color video frame head')
        self.tempo_total = time.perf_counter()
        
        model = model.lower()
        camera = camera.lower()
        
        # The persist=True argument tells the tracker that the current image or frame is the next in a sequence and to expect tracks from the previous image in the current image.
        # results = self.object_model(current_frame, stream = True)

        if camera == "head:":
            if model == "objects":  
                object_results = self.node.object_model.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            elif model == "shoes":  
                object_results = self.node.shoes_model.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            elif model == "doors":  
                object_results = self.node.doors_model.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            # else: # just so there is no error in case of wrong model name
            #     object_results = self.node.object_model.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
        else: # camera == "hand" 
            if model == "objects":  
                object_results = self.node.object_model_hand.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            elif model == "shoes":  
                object_results = self.node.shoes_model_hand.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            elif model == "doors":  
                object_results = self.node.doors_model_hand.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
            # else: # just so there is no error in case of wrong model name
            #     object_results = self.node.object_model.track(current_frame_draw, persist=True, tracker="bytetrack.yaml")
        

        num_obj = len(object_results[0])
        # self.get_logger().info(f"Objects detected: {num_obj}")

        requested_objects = []
        for object_idx in range(num_obj):

            boxes_id = object_results[0].boxes[object_idx]
            
            bb = BoundingBox()
            bb.box_top_left_x = int(boxes_id.xyxy[0][0])
            bb.box_top_left_y = int(boxes_id.xyxy[0][1])
            bb.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
            bb.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

            get_pc = BoundingBoxAndPoints()
            get_pc.bbox = bb

            requested_objects.append(get_pc)


        self.node.waiting_for_pcloud = True
        self.node.call_point_cloud_server(requested_objects, camera)

        while self.node.waiting_for_pcloud:
            pass

        new_pcloud = self.node.point_cloud_response.coords

        yolov8_obj_filtered = ListOfDetectedObject()
        
        # print(num_obj)
        # print(object_results[0])
        
        for object_idx in range(num_obj):
            boxes_id = object_results[0].boxes[object_idx]
            # print(object_results[0].boxes)

            if model == "objects":  
                object_name = self.node.objects_class_names[int(boxes_id.cls[0])].replace("_", " ").title()
                object_class = self.node.objects_class_names_dict[object_name]
            elif model == "shoes":  
                object_name = self.node.shoes_class_names[int(boxes_id.cls[0])].replace("_", " ").title()
                object_class = "Footwear"
            elif model == "doors":  
                object_name = self.node.doors_class_names[int(boxes_id.cls[0])].replace("_", " ").title()
                object_class = "Furniture"
        
            # adds object to "object_pose" without any restriction
            new_object = DetectedObject()
            self.node.get_logger().info(f"Objects detected: {new_pcloud[object_idx].center_coords}")
            new_object = self.node.add_object_to_detectedobject_msg(boxes_id, object_name, object_class, new_pcloud[object_idx].center_coords, camera)

            ALL_CONDITIONS_MET = 1

            if model == "objects":   
                # checks whether the object confidence is above a selected level
                if not boxes_id.conf >= MIN_OBJECT_CONF_VALUE:
                    ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                    # print("- Misses minimum object confidence level")

            elif model == "shoes":     
                # checks whether the shoes confidence is above a selected level
                if not boxes_id.conf >= MIN_SHOES_CONF_VALUE:
                    ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                    # print("- Misses minimum shoe confidence level")

            elif model == "doors":  
                # checks whether the doors confidence is above a selected level
                if not boxes_id.conf >= MIN_DOORS_CONF_VALUE:
                    ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                    # print("- Misses minimum door confidence level")

            # if the object detection passes all selected conditions, the detected object is added to the publishing list
            if ALL_CONDITIONS_MET:
                yolov8_obj_filtered.objects.append(new_object)
 
        self.node.get_logger().info(f"Objects detected: {num_obj}/{len(yolov8_obj_filtered.objects)}")
        self.node.get_logger().info(f"Time Yolo_Objects: {round(time.perf_counter() - self.tempo_total,2)}")

        if self.node.DEBUG_DRAW:
            self.draw_detected_objects(yolov8_obj_filtered, current_frame_draw)

        return yolov8_obj_filtered, num_obj

    def draw_detected_objects(self, yolov8_objects, current_frame_draw):
        
        red_yp =     ( 56,   56, 255)
        lblue_yp =   ( 255, 194,   0)
        blue_yp =    ( 255,   0,   0)
        green_yp =   (   0, 255,   0)
        dgreen_yp =  (  50, 204,  50)
        orange_yp =  (  51, 153, 255)
        yellow_yp =  (   0, 255, 255)
        magenta_yp = ( 255,  51, 255)
        purple_yp =  ( 255,  56, 132)
        white_yp =   ( 255, 255, 255)
        grey_yp =    ( 190, 190, 190)
        black_yp =   (   0,   0,   0)

        for object in yolov8_objects.objects:

            if object.object_class == "Cleaning Supplies":
                bb_color = yellow_yp
            elif object.object_class == "Drinks":
                bb_color = purple_yp
            elif object.object_class == "Foods":
                bb_color = lblue_yp
            elif object.object_class == "Fruits":
                bb_color = orange_yp
            elif object.object_class == "Toys":
                bb_color = blue_yp
            elif object.object_class == "Snacks":
                bb_color = magenta_yp
            elif object.object_class == "Dishes":
                bb_color = grey_yp
            elif object.object_class == "Footwear":
                bb_color = red_yp
            elif object.object_class == "Furniture":
                bb_color = green_yp
            else:
                bb_color = black_yp

            # creates the points for alternative TR visual representation 
            start_point = (object.box_top_left_x, object.box_top_left_y)
            end_point = (object.box_top_left_x+object.box_width, object.box_top_left_y+object.box_height)
            start_point_text_rect = (object.box_top_left_x-2, object.box_top_left_y)

            if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                start_point_text = object.box_top_left_x-2, object.box_top_left_y+25
                end_point_text_rect = object.box_top_left_x+50, object.box_top_left_y+30
            else:
                start_point_text = object.box_top_left_x-2, object.box_top_left_y-5
                end_point_text_rect = object.box_top_left_x+50, object.box_top_left_y-30

            ### CHANGE COLOR ACCORDING TO CLASS NAME
            if DRAW_OBJECT_BOX:
                # draws the bounding box around the detected object
                cv2.rectangle(current_frame_draw, start_point, end_point, bb_color , 4) 

            if DRAW_OBJECT_CONF and not DRAW_OBJECT_ID:
                # draws the background for the confidence of each detected object
                cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                
                # draws the confidence next to each detected object, without the initial '0' for easier visualization
                current_frame_draw = cv2.putText(
                    current_frame_draw,
                    # f"{round(float(per.conf),2)}",
                    f"{'.'+str(int((object.confidence+0.005)*100))}",
                    (start_point_text[0], start_point_text[1]),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                ) 
                
            elif not DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                if object.index != 0:

                    # draws the background for the confidence of each detected object
                    cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+10, end_point_text_rect[1]) , bb_color , -1) 
                    
                    current_frame_draw = cv2.putText(
                        current_frame_draw,
                        # f"{round(float(per.conf),2)}",
                        f"{str(int(object.index))}",
                        (start_point_text[0], start_point_text[1]),
                        cv2.FONT_HERSHEY_DUPLEX,
                        1,
                        (0, 0, 0),
                        1,
                        cv2.LINE_AA
                    ) 

            elif DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                if object.index != 0:

                    # draws the background for the confidence of each detected object
                    cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+70, end_point_text_rect[1]) , bb_color , -1) 
                    
                    current_frame_draw = cv2.putText(
                        current_frame_draw,
                        # f"{round(float(per.conf),2)}",
                        f"{str(int(object.index))}",
                        (start_point_text[0], start_point_text[1]),
                        cv2.FONT_HERSHEY_DUPLEX,
                        1,
                        (0, 0, 0),
                        1,
                        cv2.LINE_AA
                    ) 

                    # draws the confidence next to each detected object, without the initial '0' for easier visualization
                    current_frame_draw = cv2.putText(
                        current_frame_draw,
                        # f"{round(float(per.conf),2)}",
                        f"{'.'+str(int((object.confidence+0.005)*100))}",
                        (start_point_text[0]+70, start_point_text[1]),
                        cv2.FONT_HERSHEY_DUPLEX,
                        1,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA
                    ) 

                else:
                    # draws the background for the confidence of each detected object
                    cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                    
                    # draws the confidence next to each detected object, without the initial '0' for easier visualization
                    current_frame_draw = cv2.putText(
                        current_frame_draw,
                        # f"{round(float(per.conf),2)}",
                        f"{'.'+str(int((object.confidence+0.005)*100))}",
                        (start_point_text[0], start_point_text[1]),
                        cv2.FONT_HERSHEY_DUPLEX,
                        1,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA
                    ) 

            if DRAW_OBJECT_NAME:

                ### draws the name of the object
                current_frame_draw = cv2.putText(
                    current_frame_draw,
                    # f"{round(float(per.conf),2)}",
                    f"{object.object_name}",
                    (start_point[0], end_point[1]),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1,
                    (0,0,0),
                    1,
                    cv2.LINE_AA
                ) 
            
            if DRAW_OBJECT_LOCATION_COORDS:
                cv2.putText(current_frame_draw, '('+str(round(object.position_relative.x,2))+
                            ', '+str(round(object.position_relative.y,2))+
                            ', '+str(round(object.position_relative.z,2))+')',
                            (object.box_center_x, object.box_center_y), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                

            if DRAW_OBJECT_LOCATION_HOUSE_FURNITURE:
                    cv2.putText(current_frame_draw, object.room_location+" - "+object.furniture_location,
                        (object.box_center_x, object.box_center_y+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    # main state-machine function
    def main(self):
        
        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In YoloObjects Main...")

        while True:

            # type(results) = <class 'list'>
            # type(results[0]) = <class 'ultralytics.engine.results.Results'>
            # type(results[0].boxes) = <class 'ultralytics.engine.results.Boxes'>
            
            # /*** ultralytics.engine.results.Results ***/
            # A class for storing and manipulating inference results.
            # Attributes:
            # Name 	        Type 	    Description
            # orig_img 	    ndarray 	The original image as a numpy array.
            # orig_shape 	tuple 	    The original image shape in (height, width) format.
            # boxes 	    Boxes 	    A Boxes object containing the detection bounding boxes.
            # masks 	    Masks 	    A Masks object containing the detection masks.
            # probs 	    Probs 	    A Probs object containing probabilities of each class for classification task.
            # keypoints 	Keypoints 	A Keypoints object containing detected keypoints for each object.
            # speed 	    dict 	    A dictionary of preprocess, inference, and postprocess speeds in milliseconds per image.
            # names 	    dict 	    A dictionary of class names.
            # path 	        str 	    The path to the image file.
            # keys 	        tuple 	    A tuple of attribute names for non-empty attributes. 

            # /*** ultralytics.engine.results.Boxes ***/
            # A class for storing and manipulating detection boxes.
            # Attributes:
            # Name      Type                Description
            # xyxy 	    Tensor | ndarray 	The boxes in xyxy format.
            # conf 	    Tensor | ndarray 	The confidence values of the boxes.
            # cls 	    Tensor | ndarray 	The class values of the boxes.
            # id 	    Tensor | ndarray 	The track IDs of the boxes (if available).
            # xywh 	    Tensor | ndarray 	The boxes in xywh format.
            # xyxyn 	Tensor | ndarray 	The boxes in xyxy format normalized by original image size.
            # xywhn 	Tensor | ndarray 	The boxes in xywh format normalized by original image size.
            # data 	    Tensor 	            The raw bboxes tensor (alias for boxes). 

            if self.node.new_head_rgb:

                total_obj = 0
                current_frame = self.node.br.imgmsg_to_cv2(self.node.head_rgb, "bgr8")
                # current_frame = cv2.resize(current_frame, (1280, 720), interpolation=cv2.INTER_NEAREST)
                _height, _width, _ = current_frame.shape
                current_frame_draw = current_frame.copy()
                list_all_objects_detected = ListOfDetectedObject()
                    

                if self.node.ACTIVATE_YOLO_OBJECTS:
                    list_detected_objects, to = self.detect_with_yolo_model(model="objects", camera="head", current_frame_draw=current_frame_draw)
                    total_obj += to
                    for o in list_detected_objects.objects:
                        list_all_objects_detected.objects.append(o)
                    
                if self.node.ACTIVATE_YOLO_DOORS:
                    list_detected_doors, td = self.detect_with_yolo_model(model="doors", camera="head", current_frame_draw=current_frame_draw)
                    total_obj += td
                    for o in list_detected_doors.objects:
                        list_all_objects_detected.objects.append(o)
                
                if self.node.ACTIVATE_YOLO_SHOES:
                    list_detected_shoes, ts = self.detect_with_yolo_model(model="shoes", camera="head", current_frame_draw=current_frame_draw)
                    total_obj += ts
                    for o in list_detected_shoes.objects:
                        list_all_objects_detected.objects.append(o)

                if len(list_all_objects_detected.objects) > 0:
                    self.node.objects_filtered_publisher.publish(list_all_objects_detected)


                self.new_head_frame_time = time.time()
                self.head_fps = str(round(1/(self.new_head_frame_time-self.prev_head_frame_time), 2))
                self.prev_head_frame_time = self.new_head_frame_time

                if self.node.DEBUG_DRAW:
                    cv2.putText(current_frame_draw, 'fps:' + self.head_fps, (0, _height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(current_frame_draw, 'np:' + str(len(list_all_objects_detected.objects)) + '/' + str(total_obj), (180, _height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow("Yolo Objects TR Detection HEAD", current_frame_draw)
                    cv2.waitKey(1)

                self.node.new_head_rgb = False

            if self.node.new_hand_rgb:

                total_obj = 0
                current_frame = self.node.br.imgmsg_to_cv2(self.node.hand_rgb, "bgr8")
                current_frame = cv2.resize(current_frame, (1280, 720), interpolation=cv2.INTER_NEAREST)
                _height, _width, _ = current_frame.shape
                current_frame_draw = current_frame.copy()
                list_all_objects_detected_hand = ListOfDetectedObject()

                if self.node.ACTIVATE_YOLO_OBJECTS_HAND:
                    list_detected_objects_hand, to = self.detect_with_yolo_model(model="objects", camera="hand", current_frame_draw=current_frame_draw)
                    total_obj += to
                    for o in list_detected_objects_hand.objects:
                        list_all_objects_detected_hand.objects.append(o)
                    
                if self.node.ACTIVATE_YOLO_DOORS_HAND:
                    list_detected_doors_hand, td = self.detect_with_yolo_model(model="doors", camera="hand", current_frame_draw=current_frame_draw)
                    total_obj += td
                    for o in list_detected_doors_hand.objects:
                        list_all_objects_detected_hand.objects.append(o)
                
                if self.node.ACTIVATE_YOLO_SHOES_HAND:
                    list_detected_shoes_hand, ts = self.detect_with_yolo_model(model="shoes", camera="hand", current_frame_draw=current_frame_draw)
                    total_obj += ts
                    for o in list_detected_shoes_hand.objects:
                        list_all_objects_detected_hand.objects.append(o)

                if len(list_all_objects_detected_hand.objects) > 0:
                    self.node.objects_filtered_hand_publisher.publish(list_all_objects_detected_hand)


                self.new_hand_frame_time = time.time()
                self.hand_fps = str(round(1/(self.new_hand_frame_time-self.prev_hand_frame_time), 2))
                self.prev_hand_frame_time = self.new_hand_frame_time

                if self.node.DEBUG_DRAW:
                    cv2.putText(current_frame_draw, 'fps:' + self.hand_fps, (0, _height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(current_frame_draw, 'np:' + str(len(list_all_objects_detected_hand.objects)) + '/' + str(total_obj), (180, _height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow("Yolo Objects TR Detection HAND", current_frame_draw)
                    cv2.waitKey(1)

                self.node.new_hand_rgb = False
            