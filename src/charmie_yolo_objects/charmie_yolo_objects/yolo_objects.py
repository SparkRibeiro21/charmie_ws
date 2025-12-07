#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, ListOfDetectedObject, MaskDetection
from charmie_interfaces.srv import ActivateYoloObjects
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
import cv2 
import json
import threading
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from pathlib import Path
import time
import math

from charmie_point_cloud.point_cloud_class import PointCloud

MIN_OBJECT_CONF_VALUE = 0.5
MIN_SHOES_CONF_VALUE = 0.5
MIN_FURNITURE_CONF_VALUE = 0.5

DRAW_OBJECT_CONF = True
DRAW_OBJECT_ID = True
DRAW_OBJECT_BOX = True
DRAW_OBJECT_NAME = True
DRAW_OBJECT_CLASS = True
DRAW_OBJECT_LOCATION_COORDS = True
DRAW_OBJECT_LOCATION_HOUSE_FURNITURE = False

data_lock = threading.Lock()

# Just to check if everything is OK with CUDA
# import torch
# print("CUDA available:", torch.cuda.is_available())
# print("Device count:", torch.cuda.device_count())
# print("Device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No GPU")

class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")

         ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("debug_draw", False) 
        self.declare_parameter("activate_objects", False)
        # self.declare_parameter("activate_shoes", False)
        self.declare_parameter("activate_furniture", False)
        self.declare_parameter("activate_objects_hand", False)
        # self.declare_parameter("activate_shoes_hand", False)
        self.declare_parameter("activate_furniture_hand", False)
        self.declare_parameter("activate_objects_base", False)
        # self.declare_parameter("activate_shoes_base", False)
        self.declare_parameter("activate_furniture_base", False)
    
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        midpath_yolo_models = "charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects/yolo_models"
        self.complete_path_yolo_models = self.home+'/'+midpath_yolo_models+'/'

        midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+midpath_configuration_files+'/'

        midpath_select_yolo_models = "charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects"
        self.complete_path_select_yolo_models = self.home+'/'+midpath_select_yolo_models+'/'

        # Opens files with objects names and categories
        try:
            with open(self.complete_path_configuration_files + 'objects.json', encoding='utf-8') as json_file:
                self.objects_file = json.load(json_file)
            # print(self.objects_file)
            with open(self.complete_path_configuration_files + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.complete_path_configuration_files + 'furniture.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)
            with open(self.complete_path_configuration_files + 'detected_furniture.json', encoding='utf-8') as json_file:
                self.detected_furniture_file = json.load(json_file)
            # print(self.objects_file)
            self.get_logger().info("Successfully Imported data from json configuration files. (objects, rooms, furniture and detected_furniture)")
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects, rooms, furniture and detected_furniture)")

        try:
            with open(self.complete_path_select_yolo_models + 'select_models.json', encoding='utf-8') as json_file:
                
                temp_json = json.load(json_file)
                print(temp_json)

                self.objects_model_filename   = temp_json["objects_model_filename"]
                self.furniture_model_filename = temp_json["furniture_model_filename"]
                self.shoes_model_filename     = temp_json["shoes_model_filename"]
        except:
            print("")
            print("ERROR: Could NOT import name of models json files. (select_models)")
            print("I will continue with the default filenames for the yolo models.")
        
            self.objects_model_filename   = "24_25_october_v1_LAR_seg.pt"
            self.furniture_model_filename = "door_bruno_2.pt"
            self.shoes_model_filename     = "shoes_socks_v1.pt"
                    
        # gets list of detected objects from objects.json and alphabetically orders it to match YOLO detections 
        self.objects_class_names = [item["name"] for item in self.objects_file]
        self.objects_class_names.sort()
        
        # gets objects_classes from objetcs.json
        self.objects_class_names_dict = {}
        self.objects_class_names_dict = {item["name"]: item["class"] for item in self.objects_file}

        # gets list of detected furniture from detected_furniture.json and alphabetically orders it to match YOLO detections 
        self.furniture_class_names = [item["name"] for item in self.detected_furniture_file]
        self.furniture_class_names.sort()

        # list of detections from Stickler for the Rules: check if person is wearing shoes...
        self.shoes_class_names = ['Shoe', 'Sock']        
        

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.DEBUG_DRAW = self.get_parameter("debug_draw").value
        
        # HEAD
        # whether the activate objects flag starts as ON or OFF 
        self.ACTIVATE_YOLO_OBJECTS = self.get_parameter("activate_objects").value
        # whether the activate shoes flag starts as ON or OFF 
        self.ACTIVATE_YOLO_SHOES = False # self.get_parameter("activate_shoes").value
        # whether the activate furniture flag starts as ON or OFF 
        self.ACTIVATE_YOLO_FURNITURE = self.get_parameter("activate_furniture").value

        # HAND
        # whether the activate objects flag starts as ON or OFF 
        self.ACTIVATE_YOLO_OBJECTS_HAND = self.get_parameter("activate_objects_hand").value
        # whether the activate shoes flag starts as ON or OFF 
        self.ACTIVATE_YOLO_SHOES_HAND = False # self.get_parameter("activate_shoes_hand").value
        # whether the activate furniture flag starts as ON or OFF 
        self.ACTIVATE_YOLO_FURNITURE_HAND = self.get_parameter("activate_furniture_hand").value

        # BASE
        # whether the activate objects flag starts as ON or OFF 
        self.ACTIVATE_YOLO_OBJECTS_BASE = self.get_parameter("activate_objects_base").value
        # whether the activate shoes flag starts as ON or OFF 
        self.ACTIVATE_YOLO_SHOES_BASE = False # self.get_parameter("activate_shoes_base").value
        # whether the activate furniture flag starts as ON or OFF 
        self.ACTIVATE_YOLO_FURNITURE_BASE = self.get_parameter("activate_furniture_base").value

        # print(self.home+'/'+objects_model_filename)
        yolo_models_sucessful_imported = False

        while not yolo_models_sucessful_imported:
            
            try: 
                # Import the models, one for each category
                self.object_model = YOLO(self.complete_path_yolo_models + self.objects_model_filename, task="segment")
                # self.shoes_model = YOLO(self.complete_path_yolo_models + self.shoes_model_filename)
                self.furniture_model = YOLO(self.complete_path_yolo_models + self.furniture_model_filename, task="detect")

                # it needs to have a different model because of the track parameter, otherwise it is always creating new track ids
                self.object_model_hand = YOLO(self.complete_path_yolo_models + self.objects_model_filename, task="segment")
                # self.shoes_model_hand = YOLO(self.complete_path_yolo_models + self.shoes_model_filename)
                self.furniture_model_hand = YOLO(self.complete_path_yolo_models + self.furniture_model_filename, task="detect")

                # it needs to have a different model because of the track parameter, otherwise it is always creating new track ids
                self.object_model_base = YOLO(self.complete_path_yolo_models + self.objects_model_filename, task="segment")
                # self.shoes_model_base = YOLO(self.complete_path_yolo_models + self.shoes_model_filename)
                self.furniture_model_base = YOLO(self.complete_path_yolo_models + self.furniture_model_filename, task="detect")

                self.get_logger().info("Successfully imported YOLO models (objects, furniture, shoes)")

                yolo_models_sucessful_imported = True

            except:
                self.get_logger().error("Could NOT import YOLO models (objects, furniture, shoes)")
                time.sleep(1.0)

        ### Topics ###
        # Intel Realsense Subscribers (RGBD) Head and Hand Cameras
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        # Orbbec Camera (Base)
        self.color_image_base_subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.get_color_image_base_callback, 10)
        self.aligned_depth_image_base_subscriber = self.create_subscription(Image, "/camera/depth/image_raw", self.get_depth_base_image_callback, 10)
        # Publish Results
        self.objects_filtered_publisher = self.create_publisher(ListOfDetectedObject, 'objects_all_detected_filtered', 10)

        ### Services ###
        # This service is initialized on a function that is explained in comments on that timer function
        # self.activate_yolo_objects_service = self.create_service(ActivateYoloObjects, "activate_yolo_objects", self.callback_activate_yolo_objects)

        ### TF buffer and listener ###
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ### Class ###
        self.point_cloud = PointCloud()

        ### Variables ###        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.head_depth = Image()
        self.hand_rgb = Image()
        self.hand_depth = Image()
        self.base_rgb = Image()
        self.base_depth = Image()
        self.new_head_rgb = False
        self.new_head_depth = False
        self.new_hand_rgb = False
        self.new_hand_depth = False
        self.new_base_rgb = False
        self.new_base_depth = False

        self.CAM_IMAGE_WIDTH = 848
        self.CAM_BASE_IMAGE_WIDTH = 640
        self.CAM_IMAGE_HEIGHT = 480

        self.head_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH, 3), np.uint8)
        self.head_depth_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH), np.uint8)

        self.hand_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH, 3), np.uint8)
        self.hand_depth_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH), np.uint8)

        self.base_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_BASE_IMAGE_WIDTH, 3), np.uint8)
        self.base_depth_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_BASE_IMAGE_WIDTH), np.uint8)
        
        # this code forces the ROS2 component to wait for the models initialization with an empty frame, so that when turned ON does spend time with initializations and sends detections imediatly 
        # Allocates the memory necessary for each model, this takes some seconds, by doing this in the first frame, everytime one of the models is called instantly responde instead of loading the model
        self.yolo_models_initialized = False
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    # This type of structure was done to make sure the YOLO models were initializes and only after the service was created, 
    # Because other nodes use this service to make sure yolo is ready to work, and there were some conflicts with receiving commands
    # while initializing the models, this ways we have a timer that checks when the yolo models finished initializing and 
    # only then creates the service. Not common but works.
    def timer_callback(self):
        if self.yolo_models_initialized:
            self.temp_activate_yolo_service()
            self.get_logger().info('Condition met, destroying timer.')
            self.timer.cancel()  # Cancel the timer

    def temp_activate_yolo_service(self):
        ### Services ###
        self.activate_yolo_objects_service = self.create_service(ActivateYoloObjects, "activate_yolo_objects", self.callback_activate_yolo_objects)

    def callback_activate_yolo_objects(self, request, response):

        # bool activate_objects                                       # activate or deactivate yolo object detection
        # bool activate_furniture                                     # activate or deactivate yolo furniture detection (includes doors, drawers, washing machine, closet with doors)
        # bool activate_objects_hand                                  # activate or deactivate hand yolo object detection
        # bool activate_furniture_hand                                # activate or deactivate hand yolo furniture detection (includes doors, drawers, washing machine, closet with doors)
        # bool activate_objects_base                                  # activate or deactivate base yolo object detection
        # bool activate_furniture_base                                # activate or deactivate base yolo furniture detection (includes doors, drawers, washing machine, closet with doors)
        # float64 minimum_objects_confidence                          # adjust the minimum accuracy to assume as an object
        # float64 minimum_furniture_confidence                        # adjust the minimum accuracy to assume as a furniture
        # # bool activate_shoes               # NOT USED ANYMORE      # activate or deactivate yolo shoes detection
        # # bool activate_shoes_hand          # NOT USED ANYMORE      # activate or deactivate hand yolo shoes detection
        # # float64 minimum_shoes_confidence  # NOT USED ANYMORE      # adjust the minimum accuracy to assume as a shoe
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        global MIN_OBJECT_CONF_VALUE, MIN_SHOES_CONF_VALUE, MIN_FURNITURE_CONF_VALUE

        self.get_logger().info("Received Activate Yolo Objects %s" %("("+str(request.activate_objects)+", "
                                                                        +str(request.activate_furniture)+", "
                                                                        +str(request.activate_objects_hand)+", "
                                                                        +str(request.activate_furniture_hand)+", "
                                                                        +str(request.activate_objects_base)+", "
                                                                        +str(request.activate_furniture_base)+", "
                                                                        +str(request.minimum_objects_confidence)+", "
                                                                        +str(request.minimum_furniture_confidence)+")"))

        self.ACTIVATE_YOLO_OBJECTS          = request.activate_objects
        self.ACTIVATE_YOLO_SHOES            = False # NOT USED ANYMORE
        self.ACTIVATE_YOLO_FURNITURE        = request.activate_furniture
        self.ACTIVATE_YOLO_OBJECTS_HAND     = request.activate_objects_hand
        self.ACTIVATE_YOLO_SHOES_HAND       = False # NOT USED ANYMORE
        self.ACTIVATE_YOLO_FURNITURE_HAND   = request.activate_furniture_hand
        self.ACTIVATE_YOLO_OBJECTS_BASE     = request.activate_objects_base
        self.ACTIVATE_YOLO_SHOES_BASE       = False # NOT USED ANYMORE
        self.ACTIVATE_YOLO_FURNITURE_BASE   = request.activate_furniture_base

        MIN_OBJECT_CONF_VALUE       = request.minimum_objects_confidence
        MIN_SHOES_CONF_VALUE        = 0.5 # NOT USED ANYMORE
        MIN_FURNITURE_CONF_VALUE    = request.minimum_furniture_confidence
        
        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response
    
    def get_rgbd_head_callback(self, rgbd: RGBD):
        with data_lock: 
            self.head_rgb = rgbd.rgb
            self.head_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.head_depth = rgbd.depth
            self.head_depth_cv2_frame = self.br.imgmsg_to_cv2(rgbd.depth, "passthrough")
        self.new_head_rgb = True
        self.new_head_depth = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        with data_lock: 
            self.hand_rgb = rgbd.rgb
            self.hand_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.hand_depth = rgbd.depth
            self.hand_depth_cv2_frame = self.br.imgmsg_to_cv2(rgbd.depth, "passthrough")
        self.new_hand_rgb = True
        self.new_hand_depth = True
        # print("HAND:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)
    
    def get_color_image_base_callback(self, img: Image):
        with data_lock: 
            self.base_rgb = img
            self.base_rgb_cv2_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        self.new_base_rgb = True

    def get_depth_base_image_callback(self, img: Image):
        with data_lock: 
            self.base_depth = img
            self.base_depth_cv2_frame = self.br.imgmsg_to_cv2(img, "passthrough")
        self.new_base_depth = True

    def add_object_to_detectedobject_msg(self, boxes_id, object_name, object_class, object_coords_to_cam, object_coords_to_base, object_coords_to_map, object_2d_orientation, camera, current_img, mask=None):

        object_id = boxes_id.id
        if boxes_id.id == None:
            object_id = 0 

        new_object = DetectedObject()

        new_object.object_name = object_name
        new_object.object_class = object_class
        new_object.index = int(object_id)
        new_object.confidence = float(boxes_id.conf)

        # print(object_coords_to_cam)
        new_object.position_cam = object_coords_to_cam
        # print(object_coords_to_base)
        new_object.position_relative = object_coords_to_base
        # print(object_coords_to_map)
        new_object.position_absolute = object_coords_to_map

        # if there is a segmentation mask, it adds to DetectedObject 
        new_mask = MaskDetection()
        new_mask_norm = MaskDetection()
        
        if mask is not None:
            for p, p_n in zip(mask.xy[0], mask.xyn[0]):

                points_mask = Point()
                points_mask.x = float(p[0])
                points_mask.y = float(p[1])
                points_mask.z = 0.0
                new_mask.point.append(points_mask)

                points_mask_norm = Point()
                points_mask_norm.x = float(p_n[0])
                points_mask_norm.y = float(p_n[1])
                points_mask_norm.z = 0.0
                new_mask_norm.point.append(points_mask_norm)

        # print(new_mask)
        new_object.mask = new_mask
        new_object.mask_norm = new_mask_norm

        new_object.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_object.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_object.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_object.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        new_object.room_location, new_object.furniture_location = self.position_to_house_rooms_and_furniture(new_object.position_absolute)

        new_object.box_center_x = new_object.box_top_left_x + new_object.box_width//2
        new_object.box_center_y = new_object.box_top_left_y + new_object.box_height//2

        new_object.camera = camera
        new_object.orientation = object_2d_orientation # object 2D angle so gripper can adjust to correctly pick up the object
        new_object.image_rgb_frame = current_img
            
        return new_object

    def position_to_house_rooms_and_furniture(self, person_pos):
        
        room_location = "Outside"
        for room in self.house_rooms:
            min_x = room['bot_right_coords'][0]
            max_x = room['top_left_coords'][0]
            min_y = room['bot_right_coords'][1]
            max_y = room['top_left_coords'][1]
            # print(min_x, max_x, min_y, max_y)
            if min_x < person_pos.x < max_x and min_y < person_pos.y < max_y:
                room_location = room['name'] 

        furniture_location = "None"
        for furniture in self.house_furniture:
            min_x = furniture['bot_right_coords'][0]
            max_x = furniture['top_left_coords'][0]
            min_y = furniture['bot_right_coords'][1]
            max_y = furniture['top_left_coords'][1]
            # print(min_x, max_x, min_y, max_y)
            if min_x < person_pos.x < max_x and min_y < person_pos.y < max_y:
                furniture_location = furniture['name'] 

        return room_location, furniture_location
        
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
        self.new_base_frame_time = time.time()
        self.prev_base_frame_time = time.time()

    def get_transform(self, camera=""):

        match camera:
            case "head":
                child_link = 'D455_head_color_frame'
                parent_link = 'base_footprint'
            case "hand":
                child_link = 'D405_hand_color_frame'
                parent_link = 'base_footprint'
            case "base":
                child_link = 'camera_color_frame'
                parent_link = 'base_footprint'
            case "":
                child_link = 'base_footprint'
                parent_link = 'map'

        # proceed to lookup_transform
        if self.node.tf_buffer.can_transform(parent_link, child_link, rclpy.time.Time()):
            
            # print(parent_link, child_link, "GOOD")
            try:
                transform = self.node.tf_buffer.lookup_transform(
                    parent_link,        # target frame
                    child_link,         # source frame
                    rclpy.time.Time()   # latest available
                    # timeout=rclpy.duration.Duration(seconds=0.1) quero por isto???
                )
            except Exception as e:
                self.node.get_logger().warn(f"TF lookup failed: {e}")
                transform = None
                return  # or handle the error appropriately
        else:
            # print(parent_link, child_link, "BAD")
            transform = None
        
        return transform, child_link

    def detect_with_yolo_model(self, head_frame, hand_frame, base_frame, head_depth_frame, hand_depth_frame, base_depth_frame, head_image, hand_image, base_image):

        yolov8_obj_filtered = ListOfDetectedObject()
        objects_result_list = []
        num_obj = 0

        models_dict = {
            "head_objects": -1,
            "hand_objects": -1,
            "base_objects": -1,
            "head_furniture": -1,
            "hand_furniture": -1,
            "base_furniture": -1,
            "head_shoes": -1,
            "hand_shoes": -1,
            "base_shoes": -1
            }

        # self.get_logger().info('Receiving color video frame head')
        tempo_total = time.perf_counter()

        map_transform, _ = self.get_transform() # base_footprint -> map

        ### OBJECTS
        
        if self.node.ACTIVATE_YOLO_OBJECTS:
            object_results = self.node.object_model.track(head_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["head_objects"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_head, head_link = self.get_transform("head")

            if self.node.DEBUG_DRAW:
                cv2.imshow("HEAD OBJECTS DEBUG", object_results[0].plot())

        if self.node.ACTIVATE_YOLO_OBJECTS_HAND:
            object_results = self.node.object_model_hand.track(hand_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["hand_objects"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_hand, hand_link = self.get_transform("hand")

            if self.node.DEBUG_DRAW:
                cv2.imshow("HAND OBJECTS DEBUG", object_results[0].plot())

        if self.node.ACTIVATE_YOLO_OBJECTS_BASE:
            object_results = self.node.object_model_base.track(base_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["base_objects"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_base, base_link = self.get_transform("base")

            if self.node.DEBUG_DRAW:
                cv2.imshow("BASE OBJECTS DEBUG", object_results[0].plot())

        ### FURNITURE

        if self.node.ACTIVATE_YOLO_FURNITURE:
            object_results = self.node.furniture_model.track(head_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["head_furniture"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_head, head_link = self.get_transform("head")

            if self.node.DEBUG_DRAW:
                cv2.imshow("HEAD FURNITURE DEBUG", object_results[0].plot())
            
        if self.node.ACTIVATE_YOLO_FURNITURE_HAND:
            object_results = self.node.furniture_model_hand.track(hand_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["hand_furniture"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_hand, hand_link = self.get_transform("hand")

            if self.node.DEBUG_DRAW:
                cv2.imshow("HAND FURNITURE DEBUG", object_results[0].plot())
            
        if self.node.ACTIVATE_YOLO_FURNITURE_BASE:
            object_results = self.node.furniture_model_base.track(base_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            objects_result_list.append(object_results)
            models_dict["base_furniture"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])
            if num_obj > 0:
                transform_base, base_link = self.get_transform("base")

            if self.node.DEBUG_DRAW:
                cv2.imshow("BASE FURNITURE DEBUG", object_results[0].plot())
            
        ### SHOES (NOT USED BUT EVERYTHING IS READY IF NEEDED TO BE RESTORED)
        # 
        # if self.node.ACTIVATE_YOLO_SHOES:
        #     object_results = self.node.shoes_model.track(head_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
        #     objects_result_list.append(object_results)
        #     models_dict["head_shoes"] = len(objects_result_list) - 1
        #     num_obj += len(object_results[0])
        #     if num_obj > 0:
        #         transform_head, head_link = self.get_transform("head")
        # 
        #     if self.node.DEBUG_DRAW:
        #         cv2.imshow("HEAD SHOES DEBUG", object_results[0].plot())
        #    
        # if self.node.ACTIVATE_YOLO_SHOES_HAND:
        #     object_results = self.node.shoes_model_hand.track(hand_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
        #     objects_result_list.append(object_results)
        #     models_dict["hand_shoes"] = len(objects_result_list) - 1
        #     num_obj += len(object_results[0])
        #     if num_obj > 0:
        #         transform_hand, hand_link = self.get_transform("hand")
        # 
        #     if self.node.DEBUG_DRAW:
        #         cv2.imshow("HAND SHOES DEBUG", object_results[0].plot())
        #     
        # if self.node.ACTIVATE_YOLO_SHOES_BASE:
        #     object_results = self.node.shoes_model_base.track(base_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
        #     objects_result_list.append(object_results)
        #     models_dict["base_shoes"] = len(objects_result_list) - 1
        #     num_obj += len(object_results[0])
        #     if num_obj > 0:
        #         transform_base, base_link = self.get_transform("base")
        # 
        #     if self.node.DEBUG_DRAW:
        #         cv2.imshow("BASE SHOES DEBUG", object_results[0].plot())
        
        if self.node.DEBUG_DRAW:
            cv2.waitKey(1)

        print("TRACK TIME:", time.perf_counter()-tempo_total)

        reverse_models_dict = {v: k for k, v in models_dict.items() if v != -1}

        # print(num_obj)
        for idx, obj_res in enumerate(objects_result_list):

            if obj_res[0].boxes.id is not None:

                camera, model = reverse_models_dict[idx].split("_")
                # print(idx, "->", camera, model)
                
                rgb_img = Image()

                # specific camera settings
                match camera:
                    case "head":
                        rgb_img = head_image
                        depth_frame = head_depth_frame
                        transform = transform_head
                        camera_link = head_link
                    case "hand":
                        rgb_img = hand_image
                        depth_frame = hand_depth_frame
                        transform = transform_hand
                        camera_link = hand_link
                    case "base":
                        rgb_img = base_image
                        depth_frame = base_depth_frame
                        transform = transform_base
                        camera_link = base_link
                
                if map_transform is None:
                    print("MAP TF: OFF!", end='')
                else:
                    print("MAP TF:  ON!", end='')
                if transform is None:
                    print("\tROBOT TF: OFF!")
                else:
                    print("\tROBOT TF:  ON!")

                # specific model settings
                match model:
                    case "objects":
                        MIN_CONF_NALUE = MIN_OBJECT_CONF_VALUE
                    case "furniture":
                        MIN_CONF_NALUE = MIN_FURNITURE_CONF_VALUE
                    case "shoes":
                        MIN_CONF_NALUE = MIN_SHOES_CONF_VALUE
                        
                boxes = obj_res[0].boxes
                masks = obj_res[0].masks
                # track_ids = obj_res[0].boxes.id.int().cpu().tolist()

                if masks is not None: # if model has segmentation mask
                    
                    for box, mask in zip(boxes, masks):

                        # print(mask.xy[0])
                        # print(len(mask.xy[0]))
                        # mask.xy[0] = np.array([], dtype=np.float32) # used to test the bug prevented on the next line
                        if len(mask.xy[0]) >= 3: # this prevents a BUG where sometimes the mask had less than 3 points, which caused PC (if empty) and GUI (if less than 3 points) to crash

                            if model == "objects":
                                object_name = self.node.objects_class_names[int(box.cls[0])]
                                object_class = self.node.objects_class_names_dict[object_name]
                            elif model == "shoes":  
                                object_name = self.node.shoes_class_names[int(box.cls[0])]
                                object_class = "Footwear"
                            elif model == "furniture":  
                                object_name = self.node.furniture_class_names[int(box.cls[0])]
                                object_class = "Furniture"
                            
                            # aaa_ = time.time()
                            obj_3d_cam_coords = self.node.point_cloud.convert_mask_to_3dpoint(depth_img=depth_frame, camera=camera, mask=mask.xy[0])
                            object_2d_orientation = self.get_object_2D_orientation(depth_img=depth_frame, mask=mask.xy[0])
                            
                            # print("3D Coords Time", time.time() - aaa_)
                            # print(object_name, "3D Coords", obj_3d_cam_coords)

                            # bbb_ = time.time()

                            ALL_CONDITIONS_MET = 1
                            
                            # no mask depth points were available, so it was not possible to calculate x,y,z coordiantes
                            if obj_3d_cam_coords.x == 0 and obj_3d_cam_coords.y == 0 and obj_3d_cam_coords.z == 0:
                                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                                print ("REMOVED")

                            # checks whether the object confidence is above a selected level
                            if not box.conf >= MIN_CONF_NALUE:
                                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                                # print("- Misses minimum confidence level")

                            # if the object detection passes all selected conditions, the detected object is added to the publishing list
                            if ALL_CONDITIONS_MET:

                                point_cam = PointStamped()
                                point_cam.header.stamp = self.node.get_clock().now().to_msg()
                                point_cam.header.frame_id = camera_link
                                point_cam.point = obj_3d_cam_coords

                                transformed_point = PointStamped()
                                transformed_point_map = PointStamped()
                                if transform is not None:
                                    transformed_point = do_transform_point(point_cam, transform)
                                    self.node.get_logger().info(f"Object in base_footprint frame: {transformed_point.point}")

                                    if map_transform is not None:
                                        transformed_point_map = do_transform_point(transformed_point, map_transform)
                                        self.node.get_logger().info(f"Object in map frame: {transformed_point_map.point}")

                                new_object = DetectedObject()
                                new_object = self.node.add_object_to_detectedobject_msg(boxes_id=box, object_name=object_name, object_class=object_class, object_coords_to_cam=point_cam.point, \
                                                                                        object_coords_to_base=transformed_point.point, object_coords_to_map=transformed_point_map.point, \
                                                                                        object_2d_orientation=object_2d_orientation, camera=camera, current_img=rgb_img, mask=mask)
                                # print(new_object.object_name, "ID:", new_object.index, str(round(new_object.confidence*100,0)) + "%", round(new_object.position_cam.x, 2), round(new_object.position_cam.y, 2), round(new_object.position_cam.z, 2) )
                                
                                conf = f"{new_object.confidence * 100:.0f}%"
                                x_ = f"{new_object.position_cam.x:4.2f}"
                                y_ = f"{new_object.position_cam.y:5.2f}"
                                z_ = f"{new_object.position_cam.z:5.2f}"
                                print(f"{new_object.object_name:<17} {'ID:'+str(new_object.index):<6} {conf:<3} ({x_}, {y_}, {z_})")

                                yolov8_obj_filtered.objects.append(new_object)
                                # print("Create obj time:", time.time() - bbb_)

                else: # if for some reason, a used model does not have 'segmentation' masks

                    for box in boxes:

                        if model == "objects":
                            object_name = self.node.objects_class_names[int(box.cls[0])]
                            object_class = self.node.objects_class_names_dict[object_name]
                        elif model == "shoes":  
                            object_name = self.node.shoes_class_names[int(box.cls[0])]
                            object_class = "Footwear"
                        elif model == "furniture":  
                            object_name = self.node.furniture_class_names[int(box.cls[0])]
                            object_class = "Furniture"

                        obj_3d_cam_coords = self.node.point_cloud.convert_bbox_to_3d_point(depth_img=depth_frame, camera=camera, bbox=box)
                        object_2d_orientation = 0.0 # in bounding box mode, there is no 2D orientation calculation
                        
                        ALL_CONDITIONS_MET = 1

                        # no mask depth points were available, so it was not possible to calculate x,y,z coordiantes
                        if obj_3d_cam_coords.x == 0 and obj_3d_cam_coords.y == 0 and obj_3d_cam_coords.z == 0:
                            ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                            print ("REMOVED")

                        # checks whether the object confidence is above a selected level
                        if not box.conf >= MIN_CONF_NALUE:
                            ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                            # print("- Misses minimum confidence level")

                        # if the object detection passes all selected conditions, the detected object is added to the publishing list
                        if ALL_CONDITIONS_MET:

                            point_cam = PointStamped()
                            point_cam.header.stamp = self.node.get_clock().now().to_msg()
                            point_cam.header.frame_id = camera_link
                            point_cam.point = obj_3d_cam_coords

                            transformed_point = PointStamped()
                            transformed_point_map = PointStamped()
                            if transform is not None:
                                transformed_point = do_transform_point(point_cam, transform)
                                self.node.get_logger().info(f"Object in base_footprint frame: {transformed_point.point}")

                                if map_transform is not None:
                                    transformed_point_map = do_transform_point(transformed_point, map_transform)
                                    self.node.get_logger().info(f"Object in map frame: {transformed_point_map.point}")

                            new_object = DetectedObject()
                            new_object = self.node.add_object_to_detectedobject_msg(boxes_id=box, object_name=object_name, object_class=object_class, object_coords_to_cam=point_cam.point, \
                                                                                    object_coords_to_base=transformed_point.point, object_coords_to_map=transformed_point_map.point, \
                                                                                    object_2d_orientation=object_2d_orientation, camera=camera, current_img=rgb_img)
                            yolov8_obj_filtered.objects.append(new_object)

        # self.node.get_logger().info(f"Objects detected: {len(yolov8_obj_filtered.objects)}/{num_obj}")
        # self.node.get_logger().info(f"Time Yolo_Objects: {time.perf_counter() - tempo_total}")

        return yolov8_obj_filtered, num_obj

    def get_object_2D_orientation(self, depth_img, mask):

        # aaa = time.time()
        
        b_mask = np.zeros(depth_img.shape[:2], np.uint8) # creates new empty window
        contour = mask
        contour = contour.astype(np.int32)
        contour = contour.reshape(-1, 1, 2)
        cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED) # creates mask window with just the inside pixesl of the detected objects
        
        # Find non-zero points in the mask image
        points = cv2.findNonZero(b_mask)  # returns (N,1,2) array or None

        # Check if there are valid points to fit the line
        if points is None or len(points) < 2:
            return  0.0 # or handle it gracefully
        
        [vx,vy,x,y] = cv2.fitLine(points, cv2.DIST_L2,0,0.01,0.01)
        theta = math.degrees(math.atan2(vy,vx))

        # Drawings for debug

        # Define length of the line to draw
        # line_length = 1000  # Increase this if the line is too short
        # Compute two points along the line (in both directions from the center point)
        # x1 = int(x - vx * line_length)
        # y1 = int(y - vy * line_length)
        # x2 = int(x + vx * line_length)
        # y2 = int(y + vy * line_length)
        # b_mask_aux = b_mask.copy()
        # Draw the line on the image
        # cv2.line(b_mask_aux, (x1, y1), (x2, y2), (128), 2)  # grayscale image → color is a single value (0-255)
        # cv2.imwrite('output_image_orientation.jpg', b_mask_aux)
        
        # print(theta)
        # print("Orientation_time:", time.time() - aaa)

        return theta


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
                bb_color = black_yp
            elif object.object_class == "Furniture":
                bb_color = green_yp
            else:
                bb_color = red_yp

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
        time_till_done = time.time()
        
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

            # /*** ultralytics.engine.results.Masks ***/
            # A class for storing and manipulating detection masks.
            # Attributes:
            # Name	Type	Description
            # data	Tensor | ndarray	The raw tensor or array containing mask data.
            # orig_shape	tuple	Original image shape in (height, width) format.
            # xy	List[ndarray]	A list of segments in pixel coordinates.
            # xyn	List[ndarray]	A list of normalized segments.

            if not self.node.yolo_models_initialized:

                self.node.new_head_rgb = True
                self.node.new_hand_rgb = True 
                self.node.new_base_rgb = True

                self.node.ACTIVATE_YOLO_OBJECTS = True
                self.node.ACTIVATE_YOLO_OBJECTS_HAND = True
                self.node.ACTIVATE_YOLO_OBJECTS_BASE = True
                self.node.ACTIVATE_YOLO_FURNITURE = True
                self.node.ACTIVATE_YOLO_FURNITURE_HAND = True
                self.node.ACTIVATE_YOLO_FURNITURE_BASE = True


            if self.node.new_head_rgb or self.node.new_hand_rgb or self.node.new_base_rgb:

                time_till_done = time.time()
                
                with data_lock: 
                    head_image_frame = self.node.head_rgb_cv2_frame.copy()
                    hand_image_frame = self.node.hand_rgb_cv2_frame.copy()
                    base_image_frame = self.node.base_rgb_cv2_frame.copy()
                    head_depth_frame = self.node.head_depth_cv2_frame.copy()
                    hand_depth_frame = self.node.hand_depth_cv2_frame.copy()
                    base_depth_frame = self.node.base_depth_cv2_frame.copy()
                    head_image = self.node.head_rgb
                    hand_image = self.node.hand_rgb
                    base_image = self.node.base_rgb

                if self.node.ACTIVATE_YOLO_OBJECTS          and self.node.new_head_rgb or \
                    self.node.ACTIVATE_YOLO_OBJECTS_HAND    and self.node.new_hand_rgb or \
                    self.node.ACTIVATE_YOLO_OBJECTS_BASE    and self.node.new_base_rgb or \
                    self.node.ACTIVATE_YOLO_FURNITURE       and self.node.new_head_rgb or \
                    self.node.ACTIVATE_YOLO_FURNITURE_HAND  and self.node.new_hand_rgb or \
                    self.node.ACTIVATE_YOLO_FURNITURE_BASE  and self.node.new_base_rgb:

                    self.node.new_head_rgb = False
                    self.node.new_hand_rgb = False
                    self.node.new_base_rgb = False

                    list_detected_objects, total_obj = self.detect_with_yolo_model(head_frame=head_image_frame, hand_frame=hand_image_frame, base_frame=base_image_frame, head_depth_frame=head_depth_frame, hand_depth_frame=hand_depth_frame, base_depth_frame=base_depth_frame, head_image=head_image, hand_image=hand_image, base_image=base_image)
                    if self.node.yolo_models_initialized:
                        self.node.objects_filtered_publisher.publish(list_detected_objects)

                    print("TR Time Yolo_Objects: ", time.time() - time_till_done)

                    # if self.node.DEBUG_DRAW:
                    #     cv2.putText(current_frame_draw, 'fps:' + self.hand_fps, (0, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.putText(current_frame_draw, 'np:' + str(len(list_detected_objects.objects)) + '/' + str(total_obj), (180, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.imshow("Yolo Objects TR Detection HAND", current_frame_draw)
                    #     cv2.waitKey(1)

            if not self.node.yolo_models_initialized:
                
                self.node.new_head_rgb = False
                self.node.new_hand_rgb = False
                self.node.new_base_rgb = False

                self.node.ACTIVATE_YOLO_OBJECTS = False
                self.node.ACTIVATE_YOLO_OBJECTS_HAND = False
                self.node.ACTIVATE_YOLO_OBJECTS_BASE = False
                self.node.ACTIVATE_YOLO_FURNITURE = False
                self.node.ACTIVATE_YOLO_FURNITURE_HAND = False
                self.node.ACTIVATE_YOLO_FURNITURE_BASE = False

                self.node.yolo_models_initialized = True
