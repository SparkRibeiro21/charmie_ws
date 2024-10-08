#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String, Float32
from geometry_msgs.msg import Point, Pose2D
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import DetectedObject, Yolov8Objects, ListOfImages, ListOfStrings, PointCloudCoordinates, BoundingBox, BoundingBoxAndPoints
from charmie_interfaces.srv import GetPointCloudBB, ActivateYoloObjects
from cv_bridge import CvBridge
import cv2 
import cvzone
import json

from pathlib import Path

import math
import time

objects_filename = "m_size_model_300_epochs_after_nandinho.pt"
objects_filename = "segmentation_M_size_model_600_epochs.pt"
shoes_filename = "shoes_socks_v1.pt"
doors_filename = "door_bruno.pt"

MIN_OBJECT_CONF_VALUE = 0.5
MIN_SHOES_CONF_VALUE = 0.5
MIN_DOORS_CONF_VALUE = 0.5

DRAW_OBJECT_CONF = True
DRAW_OBJECT_ID = True
DRAW_OBJECT_BOX = True
DRAW_OBJECT_NAME = True
DRAW_OBJECT_CLASS = True
DRAW_OBJECT_LOCATION_COORDS = True
DRAW_OBJECT_LOCATION_HOUSE_FURNITURE = True


class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")

         ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("debug_draw", True) 
        self.declare_parameter("activate_objects", True)
        self.declare_parameter("activate_shoes", False)
        self.declare_parameter("activate_doors", False)
        self.declare_parameter("activate_objects_hand", True)
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

        # Import the models, one for each category
        self.object_model = YOLO(self.complete_path + objects_filename)
        self.shoes_model = YOLO(self.complete_path + shoes_filename)

        ### Topics ###
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)

        # For individual images
        # self.cropped_image_subscription = self.create_subscription(ListOfImages, '/cropped_image', self.cropped_image_callback, 10)
        # self.cropped_image_object_detected_publisher = self.create_publisher(ListOfStrings, '/cropped_image_object_detected', 10)

        # Subscriber (Yolov8_Objects TR Parameters)
        # self.minimum_person_confidence_subscriber = self.create_subscription(Float32, "min_obj_conf", self.get_minimum_object_confidence_callback, 10)
         
        # Publish Results
        # self.objects_publisher = self.create_publisher(Yolov8Objects, 'objects_detected', 10) # test removed person_pose (non-filtered)
        self.objects_filtered_publisher = self.create_publisher(Yolov8Objects, 'objects_detected_filtered', 10)
        self.objects_filtered_hand_publisher = self.create_publisher(Yolov8Objects, 'objects_detected_filtered_hand', 10)
        self.doors_filtered_publisher = self.create_publisher(Yolov8Objects, 'doors_detected_filtered', 10)
        self.shoes_filtered_publisher = self.create_publisher(Yolov8Objects, 'shoes_detected_filtered', 10)
        
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloudBB, "get_point_cloud_bb")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        ### Services ###
        self.activate_yolo_objects_service = self.create_service(ActivateYoloObjects, "activate_yolo_objects", self.callback_activate_yolo_objects)

        ### Variables ###

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0 # math.pi/2

        # to calculate the FPS
        self.prev_frame_time = 0 # used to record the time when we processed last frame
        self.prev_frame_time_hand = 0 # used to record the time when we processed last frame
        self.new_frame_time = 0 # used to record the time at which we processed current frame
        self.new_frame_time_hand = 0 # used to record the time at which we processed current frame

        self.object_yolo_threshold = 0.2
        self.shoes_yolo_threshold = 0.5

        # self.object_results = []
        self.object_list = []
        self.waiting_for_pcloud = False
        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.hand_rgb = Image()

        self.objects_class_names = ['7up', 'Apple', 'Bag', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser',
                                   'Coffee_grounds', 'Cola', 'Cornflakes', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_Tea', 
                                   'Juice_pack', 'Knife', 'Lemon', 'Milk', 'Mustard', 'Orange', 'Orange_juice', 'Peach', 'Pear',                                  
                                   'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube', 'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 
                                   'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup', 'Tropical_juice', 'Tuna', 'Water']
        
        # Secondary declaration used for debug
        # self.objects_class_names = ['7up', 'Strawberry_jello', 'Bag', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser',
        #                             'Coffe_grounds', 'Cola', 'Cheezit', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_Tea', 
        #                             'Juice_pack', 'Knife', 'Lemon', 'Milk', '7up', 'Orange', 'Orange_juice', 'Peach', 'Pear',                                  
        #                             'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube', 'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 
        #                             'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup', 'Tropical_juice', 'Tuna', 'Water']
        
        self.shoes_class_names = ['shoe', 'sock']    
        
        self.door_class_names = ['Dishwasher', 'Door', 'Drawer', 'LevelHandler', 'Wardrobe_Door']

        self.objects_class_names_dict = {}
        self.objects_class_names_dict = {item["name"]: item["class"] for item in self.objects_file}
        
        # depending on the filename selected, the class names change
        # if objects_filename == 'vfinal.pt' or objects_filename == 'M_300epochs.pt' or objects_filename == "m_size_model_300_epochs_after_nandinho.pt":
        #     self.objects_classNames = self.lar_v_final_classname
        # elif objects_filename == 'serve_breakfast_v1.pt':
        #     self.objects_classNames = self.serve_breakfast_classname
        # else:
        #     print('Something is wrong with your model name or directory. Please check if the variable filename fits the name of your model and if the loaded directory is the correct.')
        # print(self.objects_classNames_dict

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req):
        request = GetPointCloudBB.Request()
        request.data = req
        request.retrieve_bbox = False
    
        future = self.point_cloud_client.call_async(request)
        #print("Sent Command")

        future.add_done_callback(self.callback_call_point_cloud)

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.post_receiving_pcloud(response.coords)
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

    # def get_minimum_object_confidence_callback(self, state: Float32):
    #     global MIN_OBJECT_CONF_VALUE
    #     # print(state.data)
    #     if 0.0 <= state.data <= 1.0:
    #         MIN_OBJECT_CONF_VALUE = state.data
    #         self.get_logger().info('NEW MIN_OBJECT_CONF_VALUE RECEIVED')    
    #     else:
    #         self.get_logger().info('ERROR SETTING MIN_OBJECT_CONF_VALUE')   

    def get_color_image_hand_callback(self, img: Image):
        # only when activated via service, the model computes the person detection
        if self.ACTIVATE_YOLO_OBJECTS:

            # self.get_logger().info('Receiving color video frame head')
            self.tempo_total_hand = time.perf_counter()
            self.hand_rgb = img

            # ROS2 Image Bridge for OpenCV
            current_frame = self.br.imgmsg_to_cv2(self.hand_rgb, "bgr8")
            current_frame_draw = current_frame.copy()
            
            # Getting image dimensions
            self.img_width_hand = img.width
            self.img_height_hand = img.height

            # The persist=True argument tells the tracker that the current image or frame is the next in a sequence and to expect tracks from the previous image in the current image.
            # results = self.object_model(current_frame, stream = True)
            self.object_results_hand = self.object_model.track(current_frame, persist=True, tracker="bytetrack.yaml")

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

            """
            num_obj = len(self.object_results[0])
            # self.get_logger().info(f"Objects detected: {num_obj}")

            requested_objects = []
            for object_idx in range(num_obj):

                boxes_id = self.object_results[0].boxes[object_idx]
                
                bb = BoundingBox()
                bb.box_top_left_x = int(boxes_id.xyxy[0][0])
                bb.box_top_left_y = int(boxes_id.xyxy[0][1])
                bb.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
                bb.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

                get_pc = BoundingBoxAndPoints()
                get_pc.bbox = bb

                requested_objects.append(get_pc)

            self.waiting_for_pcloud = True
            self.call_point_cloud_server(requested_objects)
            """
        
            # current_frame = self.br.imgmsg_to_cv2(self.head_rgb, "bgr8")
            # annotated_frame = self.object_results[0].plot()

            # Calculate the number of persons detected
            num_obj = len(self.object_results_hand[0])

            # yolov8_obj = Yolov8Objects() # test removed person_pose (non-filtered)
            yolov8_obj_filtered = Yolov8Objects()
            num_objects_filtered = 0

            # print(num_obj)
            # print(self.object_results[0])
            # print(self.object_results[0].boxes)

            for object_idx in range(num_obj):
                boxes_id = self.object_results_hand[0].boxes[object_idx]
                # print(self.object_results[0].boxes)

                ALL_CONDITIONS_MET = 1

                object_name = self.objects_class_names[int(boxes_id.cls[0])].replace("_", " ").title()
                object_class = self.objects_class_names_dict[object_name]

                # adds object to "object_pose" without any restriction
                new_object = DetectedObject()
                temp_point_cloud = Point()
                new_object = self.add_object_to_detectedobject_msg(boxes_id, object_name, object_class, temp_point_cloud)
                # yolov8_obj.objects.append(new_object) # test removed person_pose (non-filtered)

                object_id = boxes_id.id
                if boxes_id.id == None:
                    object_id = 0 

                # checks whether the person confidence is above a defined level
                if not boxes_id.conf >= MIN_OBJECT_CONF_VALUE:
                    ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                    # print("- Misses minimum person confidence level")

                if ALL_CONDITIONS_MET:
                    num_objects_filtered+=1

                    yolov8_obj_filtered.objects.append(new_object)

                    if self.DEBUG_DRAW:

                        red_yp = (56, 56, 255)
                        lblue_yp = (255,194,0)
                        blue_yp = (255,0,0)
                        green_yp = (0,255,0)
                        dgreen_yp = (50,204,50)
                        orange_yp = (51,153,255)
                        magenta_yp = (255, 51, 255)
                        purple_yp = (255, 56, 132)
                        white_yp = (255, 255, 255)
                        grey_yp = (190,190,190)

                        if object_class == "Cleaning Supplies":
                            bb_color = dgreen_yp
                        elif object_class == "Drinks":
                            bb_color = purple_yp
                        elif object_class == "Foods":
                            bb_color = lblue_yp
                        elif object_class == "Fruits":
                            bb_color = orange_yp
                        elif object_class == "Toys":
                            bb_color = blue_yp
                        elif object_class == "Snacks":
                            bb_color = magenta_yp
                        elif object_class == "Dishes":
                            bb_color = grey_yp
                        else:
                            bb_color = red_yp
                        
                        # creates the points for alternative TR visual representation 
                        start_point = (int(boxes_id.xyxy[0][0]), int(boxes_id.xyxy[0][1]))
                        end_point = (int(boxes_id.xyxy[0][2]), int(boxes_id.xyxy[0][3]))
                        start_point_text_rect = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]))

                        if int(boxes_id.xyxy[0][1]) < 30: # depending on the height of the box, so it is either inside or outside
                            start_point_text = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]+25))
                            # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]+30)) # if '0.95'
                            end_point_text_rect = (int(boxes_id.xyxy[0][0]+50), int(boxes_id.xyxy[0][1]+30)) # if '.95'
                        else:
                            start_point_text = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]-5))
                            # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]-30)) # if '0.95'
                            end_point_text_rect = (int(boxes_id.xyxy[0][0]+50), int(boxes_id.xyxy[0][1]-30)) # if '.95'

                        ### CHANGE COLOR ACCORDING TO CLASS NAME
                        if DRAW_OBJECT_BOX:
                            # draws the bounding box around the person
                            cv2.rectangle(current_frame_draw, start_point, end_point, bb_color , 4) 
                        
                        if DRAW_OBJECT_CONF and not DRAW_OBJECT_ID:
                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                            
                            # draws the confidence next to each person, without the initial '0' for easier visualization
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
                                (start_point_text[0], start_point_text[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (255, 255, 255),
                                1,
                                cv2.LINE_AA
                            ) 
                            
                        elif not DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                            if object_id != 0:

                                # draws the background for the confidence of each person
                                cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+10, end_point_text_rect[1]) , bb_color , -1) 
                                
                                current_frame_draw = cv2.putText(
                                    current_frame_draw,
                                    # f"{round(float(per.conf),2)}",
                                    f"{str(int(object_id))}",
                                    (start_point_text[0], start_point_text[1]),
                                    cv2.FONT_HERSHEY_DUPLEX,
                                    1,
                                    (0, 0, 0),
                                    1,
                                    cv2.LINE_AA
                                ) 

                        elif DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                            if object_id != 0:

                                # draws the background for the confidence of each person
                                cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+70, end_point_text_rect[1]) , bb_color , -1) 
                                
                                current_frame_draw = cv2.putText(
                                    current_frame_draw,
                                    # f"{round(float(per.conf),2)}",
                                    f"{str(int(object_id))}",
                                    (start_point_text[0], start_point_text[1]),
                                    cv2.FONT_HERSHEY_DUPLEX,
                                    1,
                                    (0, 0, 0),
                                    1,
                                    cv2.LINE_AA
                                ) 

                                # draws the confidence next to each person, without the initial '0' for easier visualization
                                current_frame_draw = cv2.putText(
                                    current_frame_draw,
                                    # f"{round(float(per.conf),2)}",
                                    f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
                                    (start_point_text[0]+70, start_point_text[1]),
                                    cv2.FONT_HERSHEY_DUPLEX,
                                    1,
                                    (255, 255, 255),
                                    1,
                                    cv2.LINE_AA
                                ) 

                            else:
                                # draws the background for the confidence of each person
                                cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                                
                                # draws the confidence next to each person, without the initial '0' for easier visualization
                                current_frame_draw = cv2.putText(
                                    current_frame_draw,
                                    # f"{round(float(per.conf),2)}",
                                    f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
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
                                f"{object_name}",
                                (start_point[0], end_point[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (0,0,0),
                                1,
                                cv2.LINE_AA
                            ) 
                        
                        if DRAW_OBJECT_LOCATION_COORDS:
                            cv2.putText(current_frame_draw, '('+str(round(new_object.position_relative.x,2))+
                                        ', '+str(round(new_object.position_relative.y,2))+
                                        ', '+str(round(new_object.position_relative.z,2))+')',
                                        (new_object.box_center_x, new_object.box_center_y), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                            
                        # if DRAW_OBJECT_LOCATION_HOUSE_FURNITURE:
                        #     cv2.putText(current_frame_draw, new_object.room_location+" - "+new_object.furniture_location,
                        #                 (new_object.box_center_x, new_object.box_center_y+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
            
            # must add also for hand 
            # yolov8_obj.image_rgb = self.head_rgb # test removed person_pose (non-filtered)
            # yolov8_obj.num_objects = num_obj # test removed person_pose (non-filtered)
            # self.objects_publisher.publish(yolov8_obj) # test removed person_pose (non-filtered)

            # must add also for hand
            yolov8_obj_filtered.image_rgb = self.hand_rgb
            yolov8_obj_filtered.num_objects = num_objects_filtered
            self.objects_filtered_hand_publisher.publish(yolov8_obj_filtered)
            
            self.new_frame_time_hand = time.time()
            self.fps_hand = round(1/(self.new_frame_time_hand-self.prev_frame_time_hand), 2)
            self.prev_frame_time_hand = self.new_frame_time_hand

            self.fps_hand = str(self.fps_hand)

            if self.DEBUG_DRAW:
                # putting the FPS count on the frame
                cv2.putText(current_frame_draw, 'fps:' + self.fps_hand, (0, self.img_height_hand-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(current_frame_draw, 'np:' + str(num_objects_filtered) + '/' + str(num_obj), (180, self.img_height_hand-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow("Yolo Objects TR Detection HAND", current_frame_draw)
                # cv2.imshow("Yolo Object Detection", annotated_frame)
                # cv2.imshow("Camera Image", current_frame)
                cv2.waitKey(1)
            
            ### TEM QUE PASSAR PARA A FUNCAO do Point Cloud
            # self.waiting_for_pcloud = False

            self.get_logger().info(f"Objects detected Hand: {num_obj}/{num_objects_filtered}")
            self.get_logger().info(f"Time Yolo_Objects Hand: {round(time.perf_counter() - self.tempo_total_hand,2)}")

            
    def get_color_image_head_callback(self, img: Image):
        
        if self.ACTIVATE_YOLO_SHOES:
            print("Shoes Activated - Debug")
        if self.ACTIVATE_YOLO_DOORS:
            print("Doors Activated - Debug")
        
        # only when activated via service, the model computes the person detection
        if self.ACTIVATE_YOLO_OBJECTS:

            if not self.waiting_for_pcloud:
                # self.get_logger().info('Receiving color video frame head')
                self.tempo_total = time.perf_counter()
                self.head_rgb = img

                # ROS2 Image Bridge for OpenCV
                current_frame = self.br.imgmsg_to_cv2(self.head_rgb, "bgr8")
                
                # Getting image dimensions
                self.img_width = img.width
                self.img_height = img.height

                # The persist=True argument tells the tracker that the current image or frame is the next in a sequence and to expect tracks from the previous image in the current image.
                # results = self.object_model(current_frame, stream = True)
                self.object_results = self.object_model.track(current_frame, persist=True, tracker="bytetrack.yaml")

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

                num_obj = len(self.object_results[0])
                # self.get_logger().info(f"Objects detected: {num_obj}")

                requested_objects = []
                for object_idx in range(num_obj):

                    boxes_id = self.object_results[0].boxes[object_idx]
                    
                    bb = BoundingBox()
                    bb.box_top_left_x = int(boxes_id.xyxy[0][0])
                    bb.box_top_left_y = int(boxes_id.xyxy[0][1])
                    bb.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
                    bb.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

                    get_pc = BoundingBoxAndPoints()
                    get_pc.bbox = bb

                    requested_objects.append(get_pc)

                self.waiting_for_pcloud = True
                self.call_point_cloud_server(requested_objects)
            

    def post_receiving_pcloud(self, new_pcloud):

        current_frame = self.br.imgmsg_to_cv2(self.head_rgb, "bgr8")
        current_frame_draw = current_frame.copy()
        # annotated_frame = self.object_results[0].plot()

        # Calculate the number of persons detected
        num_obj = len(self.object_results[0])

        # yolov8_obj = Yolov8Objects() # test removed person_pose (non-filtered)
        yolov8_obj_filtered = Yolov8Objects()
        num_objects_filtered = 0

        # print(num_obj)
        # print(self.object_results[0])
        # print(self.object_results[0].boxes)

        for object_idx in range(num_obj):
            boxes_id = self.object_results[0].boxes[object_idx]
            # print(self.object_results[0].boxes)

            ALL_CONDITIONS_MET = 1

            object_name = self.objects_class_names[int(boxes_id.cls[0])].replace("_", " ").title()
            object_class = self.objects_class_names_dict[object_name]

            # adds object to "object_pose" without any restriction
            new_object = DetectedObject()
            self.get_logger().info(f"Objects detected: {new_pcloud[object_idx].center_coords}")
            new_object = self.add_object_to_detectedobject_msg(boxes_id, object_name, object_class, new_pcloud[object_idx].center_coords)
            # yolov8_obj.objects.append(new_object) # test removed person_pose (non-filtered)

            object_id = boxes_id.id
            if boxes_id.id == None:
                object_id = 0 

            # checks whether the person confidence is above a defined level
            if not boxes_id.conf >= MIN_OBJECT_CONF_VALUE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print("- Misses minimum person confidence level")

            if ALL_CONDITIONS_MET:
                num_objects_filtered+=1

                yolov8_obj_filtered.objects.append(new_object)

                if self.DEBUG_DRAW:

                    red_yp = (56, 56, 255)
                    lblue_yp = (255,194,0)
                    blue_yp = (255,0,0)
                    green_yp = (0,255,0)
                    dgreen_yp = (50,204,50)
                    orange_yp = (51,153,255)
                    magenta_yp = (255, 51, 255)
                    purple_yp = (255, 56, 132)
                    white_yp = (255, 255, 255)
                    grey_yp = (190,190,190)

                    if object_class == "Cleaning Supplies":
                        bb_color = dgreen_yp
                    elif object_class == "Drinks":
                        bb_color = purple_yp
                    elif object_class == "Foods":
                        bb_color = lblue_yp
                    elif object_class == "Fruits":
                        bb_color = orange_yp
                    elif object_class == "Toys":
                        bb_color = blue_yp
                    elif object_class == "Snacks":
                        bb_color = magenta_yp
                    elif object_class == "Dishes":
                        bb_color = grey_yp
                    else:
                        bb_color = red_yp
                    
                    # creates the points for alternative TR visual representation 
                    start_point = (int(boxes_id.xyxy[0][0]), int(boxes_id.xyxy[0][1]))
                    end_point = (int(boxes_id.xyxy[0][2]), int(boxes_id.xyxy[0][3]))
                    start_point_text_rect = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]))

                    if int(boxes_id.xyxy[0][1]) < 30: # depending on the height of the box, so it is either inside or outside
                        start_point_text = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]+25))
                        # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]+30)) # if '0.95'
                        end_point_text_rect = (int(boxes_id.xyxy[0][0]+50), int(boxes_id.xyxy[0][1]+30)) # if '.95'
                    else:
                        start_point_text = (int(boxes_id.xyxy[0][0]-2), int(boxes_id.xyxy[0][1]-5))
                        # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]-30)) # if '0.95'
                        end_point_text_rect = (int(boxes_id.xyxy[0][0]+50), int(boxes_id.xyxy[0][1]-30)) # if '.95'

                    ### CHANGE COLOR ACCORDING TO CLASS NAME
                    if DRAW_OBJECT_BOX:
                        # draws the bounding box around the person
                        cv2.rectangle(current_frame_draw, start_point, end_point, bb_color , 4) 
                    
                    if DRAW_OBJECT_CONF and not DRAW_OBJECT_ID:
                        # draws the background for the confidence of each person
                        cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                        
                        # draws the confidence next to each person, without the initial '0' for easier visualization
                        current_frame_draw = cv2.putText(
                            current_frame_draw,
                            # f"{round(float(per.conf),2)}",
                            f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
                            (start_point_text[0], start_point_text[1]),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA
                        ) 
                        
                    elif not DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                        if object_id != 0:

                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+10, end_point_text_rect[1]) , bb_color , -1) 
                            
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{str(int(object_id))}",
                                (start_point_text[0], start_point_text[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (0, 0, 0),
                                1,
                                cv2.LINE_AA
                            ) 

                    elif DRAW_OBJECT_CONF and DRAW_OBJECT_ID:
                        if object_id != 0:

                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+70, end_point_text_rect[1]) , bb_color , -1) 
                            
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{str(int(object_id))}",
                                (start_point_text[0], start_point_text[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (0, 0, 0),
                                1,
                                cv2.LINE_AA
                            ) 

                            # draws the confidence next to each person, without the initial '0' for easier visualization
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
                                (start_point_text[0]+70, start_point_text[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (255, 255, 255),
                                1,
                                cv2.LINE_AA
                            ) 

                        else:
                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, bb_color , -1) 
                            
                            # draws the confidence next to each person, without the initial '0' for easier visualization
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{'.'+str(int((boxes_id.conf+0.005)*100))}",
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
                            f"{object_name}",
                            (start_point[0], end_point[1]),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1,
                            (0,0,0),
                            1,
                            cv2.LINE_AA
                        ) 
                    
                    if DRAW_OBJECT_LOCATION_COORDS:
                        cv2.putText(current_frame_draw, '('+str(round(new_object.position_relative.x,2))+
                                    ', '+str(round(new_object.position_relative.y,2))+
                                    ', '+str(round(new_object.position_relative.z,2))+')',
                                    (new_object.box_center_x, new_object.box_center_y), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                        
        
                    if DRAW_OBJECT_LOCATION_HOUSE_FURNITURE:
                            cv2.putText(current_frame_draw, new_object.room_location+" - "+new_object.furniture_location,
                                (new_object.box_center_x, new_object.box_center_y+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
            
        # must add also for hand 
        # yolov8_obj.image_rgb = self.head_rgb # test removed person_pose (non-filtered)
        # yolov8_obj.num_objects = num_obj # test removed person_pose (non-filtered)
        # self.objects_publisher.publish(yolov8_obj) # test removed person_pose (non-filtered)

        # must add also for hand
        yolov8_obj_filtered.image_rgb = self.head_rgb
        yolov8_obj_filtered.num_objects = num_objects_filtered
        self.objects_filtered_publisher.publish(yolov8_obj_filtered)
        
        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time

        self.fps = str(self.fps)

        if self.DEBUG_DRAW:
            # putting the FPS count on the frame
            cv2.putText(current_frame_draw, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame_draw, 'np:' + str(num_objects_filtered) + '/' + str(num_obj), (180, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Yolo Objects TR Detection HEAD", current_frame_draw)
            # cv2.imshow("Yolo Object Detection", annotated_frame)
            # cv2.imshow("Camera Image", current_frame)
            cv2.waitKey(1)
        
        ### TEM QUE PASSAR PARA A FUNCAO do Point Cloud
        # self.waiting_for_pcloud = False

        self.get_logger().info(f"Objects detected: {num_obj}/{num_objects_filtered}")
        self.get_logger().info(f"Time Yolo_Objects: {round(time.perf_counter() - self.tempo_total,2)}")


    def add_object_to_detectedobject_msg(self, boxes_id, object_name, object_class, center_object_coordinates):

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
        angle_person = math.atan2(object_rel_pos.x, object_rel_pos.y)
        dist_person = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_person - self.robot_t)

        target_x = dist_person * math.cos(theta_aux) + self.robot_x
        target_y = dist_person * math.sin(theta_aux) + self.robot_y

        a_ref = (target_x, target_y)
        # print("Rel:", (person_rel_pos.x, person_rel_pos.y), "Abs:", a_ref)

        object_abs_pos = Point()
        object_abs_pos.x = target_x
        object_abs_pos.y = target_y
        object_abs_pos.z = center_object_coordinates.z/1000
        new_object.position_absolute = object_abs_pos

        new_object.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_object.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_object.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_object.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        new_object.room_location = "None"      # still missing... (says on which room a detected object is)
        new_object.furniture_location = "None" # still missing... (says if an object location is associated with some furniture, on a table, sofa, counter, ...)

        new_object.room_location, new_object.furniture_location = self.position_to_house_rooms_and_furniture(object_abs_pos)

        new_object.box_center_x = new_object.box_top_left_x + new_object.box_width//2
        new_object.box_center_y = new_object.box_top_left_y + new_object.box_height//2

        new_object.orientation = 0.0 # still missing... (says the object angle so the gripper can adjust to correctly pick up the object)
        new_object.camera = "Head"   # still missing... (says which camera is being used)

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
    


    """
    def get_color_image_head_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame head')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        
        self.img_width = img.width
        self.img_height = img.height
        # cv2.imshow("Intel RealSense Current Frame", current_frame)
        # cv2.waitKey(1)
        
        
        # self.obj.objects = []
        # self.obj.confidence = []
        # self.obj.distance = []
        # self.obj.position = []

        # minimum value of confidence for object to be accepted as true and sent via topic
        
        # results = self.object_model(current_frame, stream = True)
        results = self.object_model.track(current_frame, persist=True, tracker="bytetrack.yaml")

        annotated_frame = results[0].plot()


        threshold = self.object_threshold
        classNames = self.objects_classNames

        #print(results)

        # this for only does 1 time ...
        for r in results:
            boxes = r.boxes

            for box in boxes:
                cls = int(box.cls[0])
                conf = math.ceil(box.conf[0] * 100) / 100
                
                if self.DEBUG_DRAW:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    #center_point = round((x1 + x2) / 2), round((y1 + y2) / 2)

                    w, h = x2 - x1, y2 - y1
                    if conf >= threshold:
                        cvzone.cornerRect(current_frame, (x1, y1, w, h), l=15)    
                        cvzone.putTextRect(current_frame, f"{classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
                    
                        print(classNames[cls], 'confidence = ' + str(conf))
                
                else:
                    pass

        # # minimum value of confidence for object to be accepted as true and sent via topic
        # results = self.shoes_model(current_frame, stream = True)
        # 
        # threshold = self.shoes_threshold
        # classNames = self.shoes_socks_classname                
        # 
        # # this for only does 1 time ...
        # for r in results:
        #     boxes = r.boxes
        # 
        #     for box in boxes:
        #         cls = int(box.cls[0])
        #         conf = math.ceil(box.conf[0] * 100) / 100
        #         
        #         if self.DEBUG_DRAW:
        #             x1, y1, x2, y2 = box.xyxy[0]
        #             x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #             #center_point = round((x1 + x2) / 2), round((y1 + y2) / 2)
        # 
        #             w, h = x2 - x1, y2 - y1
        #             if conf >= threshold:
        #                 cvzone.cornerRect(current_frame, (x1, y1, w, h), l=15)    
        #                 cvzone.putTextRect(current_frame, f"{classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
        #    
        #                 print(classNames[cls], 'confidence = ' + str(conf))
        #         
        #         else:
        #             pass
        
        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time

        self.fps = str(self.fps)

        if self.DEBUG_DRAW:
            # putting the FPS count on the frame
            # cv2.putText(current_frame, 'fps = ' + self.fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow('Output_head', current_frame) # Exibir a imagem capturada
            cv2.imshow("Yolo Detection", annotated_frame)
            cv2.waitKey(1)

            # THIS CODE IS TO SAVE IMAGES TO TEST WITHOUT THE ROBOT, IT SABES JPG AND RAW with object detection
            # cv2.imshow("Intel RealSense Current Frame", current_frame)
            # cv2.waitKey(1)
            # with open("yolo_objects.raw", "wb") as f:
            #     f.write(current_frame.tobytes())
            # height, width, channels = current_frame.shape
            # print(height, width, channels)
            # cv2.imwrite("yolo_objects.jpg", current_frame) 
            # time.sleep(1)

    """
    """
    def get_color_image_hand_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame hand')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")

        # cv2.imshow("Intel RealSense Current Frame", current_frame)
        # cv2.waitKey(1)
        
        
        # self.obj.objects = []
        # self.obj.confidence = []
        # self.obj.distance = []
        # self.obj.position = []

        # minimum value of confidence for object to be accepted as true and sent via topic
        
        results = self.object_model(current_frame, stream = True)

        threshold = self.object_threshold
        classNames = self.objects_classNames

        #print(results)

        # this for only does 1 time ...

        for r in results:
            boxes = r.boxes

            for box in boxes:
                cls = int(box.cls[0])
                conf = math.ceil(box.conf[0] * 100) / 100
                
                if self.DEBUG_DRAW:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    #center_point = round((x1 + x2) / 2), round((y1 + y2) / 2)

                    w, h = x2 - x1, y2 - y1
                    if conf >= threshold:
                        cvzone.cornerRect(current_frame, (x1, y1, w, h), l=15)    
                        cvzone.putTextRect(current_frame, f"{classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
                    
                        print(classNames[cls], 'confidence = ' + str(conf))
                
                else:
                    pass
        
        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time

        self.fps = str(self.fps)

        if self.DEBUG_DRAW:
            # putting the FPS count on the frame
            # cv2.putText(current_frame, 'fps = ' + self.fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow('Output_hand', current_frame) # Exibir a imagem capturada
            cv2.waitKey(1)

            # THIS CODE IS TO SAVE IMAGES TO TEST WITHOUT THE ROBOT, IT SABES JPG AND RAW with object detection
            # cv2.imshow("Intel RealSense Current Frame", current_frame)
            # cv2.waitKey(1)
            # with open("yolo_objects.raw", "wb") as f:
            #     f.write(current_frame.tobytes())
            # height, width, channels = current_frame.shape
            # print(height, width, channels)
            # cv2.imwrite("yolo_objects.jpg", current_frame) 
            # time.sleep(1)
        """
    """
    def cropped_image_callback(self, msg_list_of_images: ListOfImages):
        #convert msg to useful image 

        list_of_strings = ListOfStrings()
        # object_detected = ""
        list_of_cv2_images = []
        best_object_detected = ""
        best_object_detected_confidence = 0.0
        img_ctr = 0
        for msg in msg_list_of_images.images:
            img_ctr += 1
            img = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            list_of_cv2_images.append(img)
            #execute detection

            if img_ctr > 0 and img_ctr < 4:
                results = self.object_model(img)
                classNames = self.objects_class_names
                threshold = self.object_threshold
            else:
                results = self.shoes_model(img)
                classNames = self.shoes_class_names
                threshold = self.shoes_threshold
            
            msg = String()
            
            best_object_detected = ""
            best_object_detected_confidence = 0.0
            # object_detected = ""
            if results:
                for r in results:
                    boxes = r.boxes
                    for box in boxes: 
                        cls = int(box.cls[0])
                        conf = math.ceil(box.conf[0]*100) / 100
                        
                        if self.DEBUG_DRAW:
                            x1, y1, x2, y2 = box.xyxy[0]
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            #center_point = round((x1 + x2) / 2), round((y1 + y2) / 2)

                            w, h = x2 - x1, y2 - y1
                            if conf >= threshold:
                                cvzone.cornerRect(img, (x1, y1, w, h), l=15)    
                                cvzone.putTextRect(img, f"{classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
                            
                                # print(self.classNames[cls], 'confidence = ' + str(conf))
                
                        if conf >= threshold:  # Threshold for detection confidence
                        
                            if conf > best_object_detected_confidence:
                                best_object_detected = classNames[int(cls)]
                                best_object_detected_confidence = conf

                        print("Possible:", classNames[int(cls)], conf)
                
            print("Selected:", best_object_detected, best_object_detected_confidence)
            list_of_strings.strings.append(best_object_detected)

        if self.DEBUG_DRAW:
            if len(list_of_cv2_images) > 1:
                cv2.imshow('Left Hand', list_of_cv2_images[0]) # Exibir a imagem capturada
                cv2.waitKey(5)
                cv2.imshow('Right Hand', list_of_cv2_images[1]) # Exibir a imagem capturada
                cv2.waitKey(5)
                cv2.imshow('Surrounding Floor', list_of_cv2_images[2]) # Exibir a imagem capturada
                cv2.waitKey(5)
                cv2.imshow('Left Foot', list_of_cv2_images[3]) # Exibir a imagem capturada
                cv2.waitKey(5)
                cv2.imshow('Right Foot', list_of_cv2_images[4]) # Exibir a imagem capturada
                cv2.waitKey(5)

        print("----------")
        self.cropped_image_object_detected_publisher.publish(list_of_strings)
    """
    
    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta

        
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    rclpy.spin(node)
    rclpy.shutdown()