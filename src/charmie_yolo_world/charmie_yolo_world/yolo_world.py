#!/usr/bin/env python3
from ultralytics import YOLOE
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, ListOfDetectedObject, MaskDetection
from charmie_interfaces.srv import ActivateYoloWorld
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

MIN_PF_CONF_VALUE = 0.5
MIN_TV_CONF_VALUE = 0.5

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
        self.declare_parameter("load_prompt_free_model", False) 
        self.declare_parameter("activate_world_pf_head", False)
        self.declare_parameter("activate_world_pf_hand", False)
        self.declare_parameter("activate_world_pf_base", False)
        self.declare_parameter("activate_world_tv_head", False)
        self.declare_parameter("activate_world_tv_hand", False)
        self.declare_parameter("activate_world_tv_base", False)
    
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        midpath_yolo_models = "charmie_ws/src/charmie_yolo_world/charmie_yolo_world/yolo_models"
        self.complete_path_yolo_models = self.home+'/'+midpath_yolo_models+'/'

        midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+midpath_configuration_files+'/'

        # Opens files with objects names and categories
        try:
            with open(self.complete_path_configuration_files + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.complete_path_configuration_files + 'furniture.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)
            self.get_logger().info("Successfully Imported data from json configuration files. (rooms and furniture)")
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (rooms and furniture)")

        self.world_prompt_free_model_filename   = "yoloe-11l-seg-pf.pt"
        self.world_text_visual_prompt_model_filename   = "yoloe-11l-seg.pt"

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.DEBUG_DRAW = self.get_parameter("debug_draw").value

        self.LOAD_PF_MODEL = self.get_parameter("load_prompt_free_model").value
        
        self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = self.get_parameter("activate_world_pf_head").value
        self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = self.get_parameter("activate_world_pf_hand").value
        self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = self.get_parameter("activate_world_pf_base").value
        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD   = self.get_parameter("activate_world_tv_head").value
        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND   = self.get_parameter("activate_world_tv_hand").value
        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE   = self.get_parameter("activate_world_tv_base").value

        if not self.LOAD_PF_MODEL:
            self.get_logger().info("Prompt-free model disabled (load_prompt_free_model=false). Only TV model will run.")
        
        yolo_models_sucessful_imported = False
        while not yolo_models_sucessful_imported:
            
            try: 
                # Import the models
                if self.LOAD_PF_MODEL:
                    self.world_prompt_free_model = YOLOE(self.complete_path_yolo_models + self.world_prompt_free_model_filename)
                else:
                    self.world_prompt_free_model = None
                
                self.world_tv_prompt_model   = YOLOE(self.complete_path_yolo_models + self.world_text_visual_prompt_model_filename)
                
                self.get_logger().info("Successfully imported YOLO models (prompt free and text/visual prompt models)")

                yolo_models_sucessful_imported = True

            except:
                self.get_logger().error("Could NOT import YOLO models (prompt free and text/visual prompt models)")
                time.sleep(1.0)

        ### Topics ###
        # Intel Realsense Subscribers (RGBD) Head and Hand Cameras
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        # Orbbec Camera (Base)
        self.color_image_base_subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.get_color_image_base_callback, 10)
        self.aligned_depth_image_base_subscriber = self.create_subscription(Image, "/camera/depth/image_raw", self.get_depth_base_image_callback, 10)
        # Publish Results
        self.world_objects_filtered_publisher = self.create_publisher(ListOfDetectedObject, 'world_objects_all_detected_filtered', 10)

        ### Services ###
        # This service is initialized on a function that is explained in comments on that timer function
        # self.activate_yolo_world_service = self.create_service(ActivateYoloWorld, "activate_yolo_world", self.callback_activate_yolo_world)

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

        # Default prompts so TV model can warm up immediately
        self.TEXT_PROMPT_CLASSES = ["chair"]
        self._tv_prompts_last = None

        # Apply once at startup
        self.update_tv_text_prompts(self.TEXT_PROMPT_CLASSES)
        
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
        self.activate_yolo_world_service = self.create_service(ActivateYoloWorld, "activate_yolo_world", self.callback_activate_yolo_world)

    def callback_activate_yolo_world(self, request, response):

        # bool activate_prompt_free_head          # activate or deactivate prompt free detection for head camera
        # bool activate_tv_prompt_head            # activate or deactivate text and visual prompt detection for head camera
        # bool activate_prompt_free_hand          # activate or deactivate prompt free detection for gripper camera
        # bool activate_tv_prompt_hand            # activate or deactivate text and visual prompt detection for gripper camera
        # bool activate_prompt_free_base          # activate or deactivate prompt free detection for base camera
        # bool activate_tv_prompt_base            # activate or deactivate text and visual prompt detection for base camera
        # float64 minimum_prompt_free_confidence  # set minimum confidence value for prompt free detections
        # float64 minimum_tv_prompt_confidence    # set minimum confidence value for text and visual prompt detections
        # 
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"

        global MIN_PF_CONF_VALUE, MIN_TV_CONF_VALUE

        self.get_logger().info("Received ActivateYoloWorld("
            f"pf_head={request.activate_prompt_free_head}, "
            f"tv_head={request.activate_tv_prompt_head}, "
            f"pf_hand={request.activate_prompt_free_hand}, "
            f"tv_hand={request.activate_tv_prompt_hand}, "
            f"pf_base={request.activate_prompt_free_base}, "
            f"tv_base={request.activate_tv_prompt_base}, "
            f"min_pf={request.minimum_prompt_free_confidence:.2f}, "
            f"min_tv={request.minimum_tv_prompt_confidence:.2f}, "
            f"text_prompts={list(request.text_prompts)}), "
            f"visual_prompts={list(request.visual_prompts)})."
        )

        if not self.LOAD_PF_MODEL:
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = False
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = False
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = False
        else:
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = request.activate_prompt_free_head
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = request.activate_prompt_free_hand
            self.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = request.activate_prompt_free_base

        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD = request.activate_tv_prompt_head
        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND = request.activate_tv_prompt_hand
        self.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE = request.activate_tv_prompt_base

        if self.LOAD_PF_MODEL:
            MIN_PF_CONF_VALUE = request.minimum_prompt_free_confidence
        MIN_TV_CONF_VALUE = request.minimum_tv_prompt_confidence

        # --- reset prompts state every activation (your "clear cache/start over") ---
        self.TEXT_PROMPT_CLASSES = []
        self._tv_prompts_last = None

        # Apply prompts text only if user sent some
        if request.text_prompts and len(request.text_prompts) > 0:
            ok, msg = self.update_tv_text_prompts(request.text_prompts)
            response.message += " | " + msg
        else:
            response.message += " | No text_prompts provided: TV prompts remain unset."

        if not self.LOAD_PF_MODEL and (request.activate_prompt_free_head or request.activate_prompt_free_hand or request.activate_prompt_free_base):
            response.message += " | Text/Visual model activated. Prompt Free model ignored because load_prompt_free_model is false."
        
        return response
    
    def update_tv_text_prompts(self, prompts):
        cleaned = []
        seen = set()

        for s in prompts:
            s = s.strip()
            if not s or s in seen:
                continue
            seen.add(s)
            cleaned.append(s)

        if len(cleaned) == 0:
            return False, "Ignored empty/invalid text_prompts."

        # Compute text embeddings + set classes
        text_pe = self.world_tv_prompt_model.get_text_pe(cleaned)
        self.world_tv_prompt_model.set_classes(cleaned, text_pe)

        self.TEXT_PROMPT_CLASSES = cleaned
        self._tv_prompts_last = tuple(cleaned)

        self.get_logger().info(f"TV prompts set → {cleaned}")
        return True, f"TV prompts updated: {cleaned}"
    
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

    def add_object_to_detectedobject_msg(self, boxes_id, object_name, object_coords_to_cam, object_coords_to_base, object_coords_to_map, camera, current_img, mask=None):

        new_object = DetectedObject()

        new_object.object_name = object_name
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

            if furniture['shape'] == "square":
                min_x = furniture['bot_right_coords'][0]
                max_x = furniture['top_left_coords'][0]
                min_y = furniture['bot_right_coords'][1]
                max_y = furniture['top_left_coords'][1]
                # print(min_x, max_x, min_y, max_y)
                if min_x < person_pos.x < max_x and min_y < person_pos.y < max_y:
                    furniture_location = furniture['name'] 
            else: # if furniture['shape'] == "circle":
                furniture_x = (furniture['top_left_coords'][0] + furniture['bot_right_coords'][0])/2
                furniture_y = (furniture['top_left_coords'][1] + furniture['bot_right_coords'][1])/2
                person_dist_to_furniture = math.sqrt((person_pos.x-furniture_x)**2 + (person_pos.y-furniture_y)**2)

                if person_dist_to_furniture < furniture['diameter']/2:
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
                # return  # or handle the error appropriately
                return None, child_link # better way to handle error

        else:
            # print(parent_link, child_link, "BAD")
            transform = None
        
        return transform, child_link

    def detect_with_yoloe_model(self, head_frame, hand_frame, base_frame, head_depth_frame, hand_depth_frame, base_depth_frame, head_image, hand_image, base_image):

        yolov8_obj_filtered = ListOfDetectedObject()
        objects_result_list = []
        num_obj = 0

        models_dict = {
            "head_pf": -1,
            "hand_pf": -1,
            "base_pf": -1,
            "head_tv": -1,
            "hand_tv": -1,
            "base_tv": -1
            }

        # self.get_logger().info('Receiving color video frame head')
        tempo_total = time.perf_counter()
        map_transform, _ = self.get_transform() # base_footprint -> map
        
        # just for safety
        transform_head = transform_hand = transform_base = None
        head_link = "D455_head_color_frame"
        hand_link = "D405_hand_color_frame"
        base_link = "camera_color_frame"


        ### PROMPT FREE
        
        if self.node.LOAD_PF_MODEL and self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD:
            transform_head, head_link = self.get_transform("head")
            object_results = self.node.world_prompt_free_model.predict(source=head_frame, conf=MIN_PF_CONF_VALUE, verbose=False)   
            objects_result_list.append(object_results)
            models_dict["head_pf"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("HEAD OBJECTS DEBUG", object_results[0].plot())

        if self.node.LOAD_PF_MODEL and self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND:
            transform_hand, hand_link = self.get_transform("hand")
            object_results = self.node.world_prompt_free_model.predict(source=hand_frame, conf=MIN_PF_CONF_VALUE, verbose=False)   
            objects_result_list.append(object_results)
            models_dict["hand_pf"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("HAND OBJECTS DEBUG", object_results[0].plot())

        if self.node.LOAD_PF_MODEL and self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE:
            transform_base, base_link = self.get_transform("base")
            object_results = self.node.world_prompt_free_model.predict(source=base_frame, conf=MIN_PF_CONF_VALUE, verbose=False)   
            objects_result_list.append(object_results)
            models_dict["base_pf"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("BASE OBJECTS DEBUG", object_results[0].plot())

        ### TEXT AND VISUAL PROMPT
        # makes sure there are text prompts
        tv_ready = (self.node._tv_prompts_last is not None) and (len(self.node.TEXT_PROMPT_CLASSES) > 0)

        if self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD and tv_ready:
            transform_head, head_link = self.get_transform("head")
            object_results = self.node.world_tv_prompt_model.predict(source=head_frame, conf=MIN_TV_CONF_VALUE, verbose=False)
            objects_result_list.append(object_results)
            models_dict["head_tv"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("HEAD TV DEBUG", object_results[0].plot())

        if self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND and tv_ready:
            transform_hand, hand_link = self.get_transform("hand")
            object_results = self.node.world_tv_prompt_model.predict(source=hand_frame, conf=MIN_TV_CONF_VALUE, verbose=False)
            objects_result_list.append(object_results)
            models_dict["hand_tv"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("HAND TV DEBUG", object_results[0].plot())

        if self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE and tv_ready:
            transform_base, base_link = self.get_transform("base")
            object_results = self.node.world_tv_prompt_model.predict(source=base_frame, conf=MIN_TV_CONF_VALUE, verbose=False)
            objects_result_list.append(object_results)
            models_dict["base_tv"] = len(objects_result_list) - 1
            num_obj += len(object_results[0])

            if self.node.DEBUG_DRAW:
                cv2.imshow("BASE TV DEBUG", object_results[0].plot())


        if self.node.DEBUG_DRAW:
            cv2.waitKey(1)

        print("PREDICT TIME:", time.perf_counter()-tempo_total)

        reverse_models_dict = {v: k for k, v in models_dict.items() if v != -1}

        # print(num_obj)
        for idx, obj_res in enumerate(objects_result_list):

            # if obj_res[0].boxes.id is not None:

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
                case "pf":
                    MIN_CONF_VALUE = MIN_PF_CONF_VALUE
                case "tv":
                    MIN_CONF_VALUE = MIN_TV_CONF_VALUE
                    
            boxes = obj_res[0].boxes
            masks = obj_res[0].masks
            # track_ids = obj_res[0].boxes.id.int().cpu().tolist()

            if boxes is None or len(boxes) == 0 or masks is None: # safety measure
                continue
                
            for box, mask in zip(boxes, masks):

                # print(mask.xy[0])
                # print(len(mask.xy[0]))
                # mask.xy[0] = np.array([], dtype=np.float32) # used to test the bug prevented on the next line
                if len(mask.xy[0]) >= 3: # this prevents a BUG where sometimes the mask had less than 3 points, which caused PC (if empty) and GUI (if less than 3 points) to crash

                    # aaa_ = time.time()
                    obj_3d_cam_coords = self.node.point_cloud.convert_mask_to_3dpoint(depth_img=depth_frame, camera=camera, mask=mask.xy[0])
                    
                    names = obj_res[0].names
                    object_name = names[int(box.cls)]

                    # print("3D Coords Time", time.time() - aaa_)
                    # print(object_name, "3D Coords", obj_3d_cam_coords)

                    # bbb_ = time.time()

                    ALL_CONDITIONS_MET = 1
                    
                    # no mask depth points were available, so it was not possible to calculate x,y,z coordiantes
                    if obj_3d_cam_coords.x == 0 and obj_3d_cam_coords.y == 0 and obj_3d_cam_coords.z == 0:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        print ("REMOVED")

                    # checks whether the object confidence is above a selected level
                    if not box.conf >= MIN_CONF_VALUE:
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
                        new_object = self.node.add_object_to_detectedobject_msg(boxes_id=box, object_name=object_name, object_coords_to_cam=point_cam.point, \
                                                                                object_coords_to_base=transformed_point.point, object_coords_to_map=transformed_point_map.point, \
                                                                                camera=camera, current_img=rgb_img, mask=mask)
                        # print(new_object.object_name, "ID:", new_object.index, str(round(new_object.confidence*100,0)) + "%", round(new_object.position_cam.x, 2), round(new_object.position_cam.y, 2), round(new_object.position_cam.z, 2) )
                        
                        conf = f"{new_object.confidence * 100:.0f}%"
                        x_ = f"{new_object.position_cam.x:4.2f}"
                        y_ = f"{new_object.position_cam.y:5.2f}"
                        z_ = f"{new_object.position_cam.z:5.2f}"
                        print(f"{new_object.object_name:<17} {conf:<3} ({x_}, {y_}, {z_})")

                        yolov8_obj_filtered.objects.append(new_object)
                        # print("Create obj time:", time.time() - bbb_)

        # self.node.get_logger().info(f"Objects detected: {len(yolov8_obj_filtered.objects)}/{num_obj}")
        # self.node.get_logger().info(f"Time Yolo_Objects: {time.perf_counter() - tempo_total}")

        return yolov8_obj_filtered, num_obj

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

                # PF warmup only if PF model is loaded
                if self.node.LOAD_PF_MODEL:
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = True
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = True
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = True
                else:
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = False
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = False
                    self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = False

                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD   = True
                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND   = True
                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE   = True


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

                pf_should_run = self.node.LOAD_PF_MODEL and (
                    (self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD and self.node.new_head_rgb) or
                    (self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND and self.node.new_hand_rgb) or
                    (self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE and self.node.new_base_rgb)
                )

                tv_ready = (self.node._tv_prompts_last is not None) and (len(self.node.TEXT_PROMPT_CLASSES) > 0)

                tv_should_run = tv_ready and (
                    (self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD and self.node.new_head_rgb) or
                    (self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND and self.node.new_hand_rgb) or
                    (self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE and self.node.new_base_rgb)
                )

                if pf_should_run or tv_should_run:

                    self.node.new_head_rgb = False
                    self.node.new_hand_rgb = False
                    self.node.new_base_rgb = False

                    list_detected_objects, total_obj = self.detect_with_yoloe_model(head_frame=head_image_frame, hand_frame=hand_image_frame, base_frame=base_image_frame, head_depth_frame=head_depth_frame, hand_depth_frame=hand_depth_frame, base_depth_frame=base_depth_frame, head_image=head_image, hand_image=hand_image, base_image=base_image)
                    if self.node.yolo_models_initialized:
                        self.node.world_objects_filtered_publisher.publish(list_detected_objects)

                    print("TR Time Yolo_World: ", time.time() - time_till_done)

                    # if self.node.DEBUG_DRAW:
                    #     cv2.putText(current_frame_draw, 'fps:' + self.hand_fps, (0, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.putText(current_frame_draw, 'np:' + str(len(list_detected_objects.objects)) + '/' + str(total_obj), (180, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.imshow("Yolo Objects TR Detection HAND", current_frame_draw)
                    #     cv2.waitKey(1)

            if not self.node.yolo_models_initialized:
                
                self.node.new_head_rgb = False
                self.node.new_hand_rgb = False
                self.node.new_base_rgb = False

                self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HEAD = False
                self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_HAND = False
                self.node.ACTIVATE_YOLO_WORLD_PROMPT_FREE_BASE = False
                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HEAD   = False
                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_HAND   = False
                self.node.ACTIVATE_YOLO_WORLD_TV_PROMPT_BASE   = False

                self.node.yolo_models_initialized = True
