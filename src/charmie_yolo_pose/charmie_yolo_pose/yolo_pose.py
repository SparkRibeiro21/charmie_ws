#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedPerson, BoundingBox, BoundingBoxAndPoints, RGB, ListOfDetectedPerson
from charmie_interfaces.srv import GetPointCloudBB, ActivateYoloPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math
import json
from keras.models import load_model

from pathlib import Path

# configurable parameters through ros topics
ONLY_DETECT_PERSON_LEGS_VISIBLE = False              # if True only detects people whose legs are visible 
MIN_PERSON_CONF_VALUE = 0.3                          # defines the minimum confidence value to be considered a person
MIN_KP_TO_DETECT_PERSON = 4                          # this parameter does not consider the four legs keypoints 
ONLY_DETECT_PERSON_RIGHT_IN_FRONT = False            # only detects person right in front of the robot both on the x and y axis 
ONLY_DETECT_PERSON_RIGHT_IN_FRONT_X_THRESHOLD = 0.6
ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD = 1.8
ONLY_DETECT_PERSON_ARM_RAISED = False                # if True only detects people with their arm raised or waving 
GET_CHARACTERISTICS = False

# must be adjusted if we want just to not detect the feet in cases where the walls are really low and we can see the knees
# 3 may be used in cases where it just does not detect on of the feet 
NUMBER_OF_LEG_KP_TO_BE_DETECTED = 4
MIN_KP_CONF_VALUE = 0.5


DRAW_PERSON_CONF = True
DRAW_PERSON_ID = True
DRAW_PERSON_BOX = True
DRAW_PERSON_KP = True
DRAW_LOW_CONF_KP = False
DRAW_PERSON_LOCATION_COORDS = True
DRAW_PERSON_LOCATION_HOUSE_FURNITURE = False
DRAW_PERSON_POINTING_INFO = False
DRAW_PERSON_HAND_RAISED = False
DRAW_PERSON_HEIGHT = True
DRAW_PERSON_CLOTHES_COLOR = True
DRAW_CHARACTERISTICS = True
DRAW_FACE_RECOGNITION = True


class YoloPoseNode(Node):
    def __init__(self):
        super().__init__("YoloPose")
        self.get_logger().info("Initialised YoloPose Node")

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("yolo_model", "s") 
        self.declare_parameter("debug_draw", False) 
        self.declare_parameter("activate", False)

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_yolo_pose/charmie_yolo_pose"
        self.complete_path = self.home+'/'+self.midpath+'/'

        self.midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+self.midpath_configuration_files+'/'

        self.complete_path_characteristics_models = self.home+'/'+self.midpath+'/characteristics_models/'

        self.house_rooms = {}
        self.house_furniture = {}

        # Open all configuration files
        try:
            with open(self.complete_path_configuration_files + 'rooms_location.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.complete_path_configuration_files + 'furniture_location.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)
            self.get_logger().info("Successfully Imported data from json configuration files. (house_rooms and house_furniture)")
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (house_rooms and house_furniture)")
        
        try:
            self.race_model = load_model(self.complete_path_characteristics_models+"modelo_raca.h5")
            self.gender_model = load_model(self.complete_path_characteristics_models+"modelo_gender.h5")
            ageModel = self.complete_path_characteristics_models+"age_net.caffemodel"
            ageProto = self.complete_path_characteristics_models+"deploy_age.prototxt"
            self.ageNet = cv2.dnn.readNet(ageModel, ageProto)
            self.get_logger().info("Successfully Imported race, gender and age models.")
        except:
            self.get_logger().error("Could NOT import race, gender and age keras models. Please check if the files are in the /characteristics_models folder")

        # choose the yolo pose model intended to be used (n,s,m,l,...) 
        self.YOLO_MODEL = self.get_parameter("yolo_model").value
        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.DEBUG_DRAW = self.get_parameter("debug_draw").value
        # whether the activate flag starts as ON or OFF 
        self.ACTIVATE_YOLO_POSE = self.get_parameter("activate").value

        yolo_model = "yolov8" + self.YOLO_MODEL.lower() + "-pose.pt"
        full_yolo_model = self.complete_path + yolo_model
        self.get_logger().info(f"Using YOLO pose model: {yolo_model}")

        ### Topics ###
        # Yolo Model - Yolov8 Pose:
        # If the PC used has lower frame rates switch in: self.declare_parameter("yolo_model", "s")
        self.model = YOLO(full_yolo_model)

        # Publisher (Pose of People Detected Filtered and Non Filtered)
        self.person_pose_filtered_publisher = self.create_publisher(ListOfDetectedPerson, "person_pose_filtered", 10)

        # Subscriber (Yolov8_Pose TR Parameters)
        # self.only_detect_person_legs_visible_subscriber = self.create_subscription(Bool, "only_det_per_legs_vis", self.get_only_detect_person_legs_visible_callback, 10)
        # self.minimum_person_confidence_subscriber = self.create_subscription(Float32, "min_per_conf", self.get_minimum_person_confidence_callback, 10)
        # self.minimum_keypoints_to_detect_person_subscriber = self.create_subscription(Int16, "min_kp_det_per", self.get_minimum_keypoints_to_detect_person_callback, 10)
        # self.only_detect_person_right_in_front_subscriber = self.create_subscription(Bool, "only_det_per_right_in_front", self.get_only_detect_person_right_in_front_callback, 10)
        # self.only_detect_person_arm_raised_subscriber = self.create_subscription(Bool, "only_det_per_arm_raised", self.get_only_detect_person_arm_raised_callback, 10)

        # Intel Realsense Subscribers
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)

        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloudBB, "get_point_cloud_bb")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        ### Services ###
        self.activate_yolo_pose_service = self.create_service(ActivateYoloPose, "activate_yolo_pose", self.callback_activate_yolo_pose)
        
        ### Variables ###
        # to calculate the FPS
        self.prev_frame_time = 0 # used to record the time when we processed last frame
        self.new_frame_time = 0 # used to record the time at which we processed current frame
        
        self.br = CvBridge()
        self.rgb_img = Image()
        self.detpth_img = Image()

        self.results = []
        self.waiting_for_pcloud = False
        self.tempo_total = time.perf_counter()
        self.center_torso_person_list = []
        self.center_head_person_list = []

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0 # math.pi/2

        self.N_KEYPOINTS = 17
        self.NUMBER_OF_LEGS_KP = 4
        self.NOSE_KP = 0
        self.EYE_LEFT_KP = 1                        
        self.EYE_RIGHT_KP = 2
        self.EAR_LEFT_KP = 3
        self.EAR_RIGHT_KP = 4
        self.SHOULDER_LEFT_KP = 5
        self.SHOULDER_RIGHT_KP = 6
        self.ELBOW_LEFT_KP = 7
        self.ELBOW_RIGHT_KP = 8
        self.WRIST_LEFT_KP = 9
        self.WRIST_RIGHT_KP = 10
        self.HIP_LEFT_KP = 11
        self.HIP_RIGHT_KP = 12
        self.KNEE_LEFT_KP = 13
        self.KNEE_RIGHT_KP = 14
        self.ANKLE_LEFT_KP = 15
        self.ANKLE_RIGHT_KP = 16


    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req):
        request = GetPointCloudBB.Request()
        request.data = req
        request.retrieve_bbox = False
        request.camera = "head"
    
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


    def callback_activate_yolo_pose(self, request, response):
        
        # Type of service received: 
        # bool activate                               # activate or deactivate yolo pose detection
        # bool only_detect_person_legs_visible        # only detects persons with visible legs (filters all people outside @home house)
        # float64 minimum_person_confidence           # adjust the minimum accuracy to assume as a person
        # int32 minimum_keypoints_to_detect_person    # minimum necessary keypoints to detect as a person
        # bool only_detect_person_right_in_front      # only detects people who are right in front of the robot (easier to interact)
        # bool only_detect_person_arm_raised          # only detects people who are asking for assistance (arm raised)
        # bool characteristics                        # whether the person characteristics should be calculated or not (arm pointing, shirt and pants colour, ethnicity, age_estimate, gender) 
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        global ONLY_DETECT_PERSON_LEGS_VISIBLE, MIN_PERSON_CONF_VALUE, MIN_KP_TO_DETECT_PERSON, ONLY_DETECT_PERSON_RIGHT_IN_FRONT, ONLY_DETECT_PERSON_ARM_RAISED, GET_CHARACTERISTICS

        if request.activate:
            self.get_logger().info("Activated Yolo Pose %s" %("("+str(request.activate)+", "
                                                                 +str(request.only_detect_person_legs_visible)+", "
                                                                 +str(request.minimum_person_confidence)+", "
                                                                 +str(request.minimum_keypoints_to_detect_person)+", "
                                                                 +str(request.only_detect_person_right_in_front)+", "
                                                                 +str(request.only_detect_person_arm_raised)+", "
                                                                 +str(request.characteristics)+")"))
        else: 
            self.get_logger().info("Deactivated Yolo Pose")

        self.ACTIVATE_YOLO_POSE = request.activate
        ONLY_DETECT_PERSON_LEGS_VISIBLE = request.only_detect_person_legs_visible
        # MIN_PERSON_CONF_VALUE = request.minimum_person_confidence
        # MIN_KP_TO_DETECT_PERSON = request.minimum_keypoints_to_detect_person
        ONLY_DETECT_PERSON_RIGHT_IN_FRONT = request.only_detect_person_right_in_front
        ONLY_DETECT_PERSON_ARM_RAISED = request.only_detect_person_arm_raised
        GET_CHARACTERISTICS = request.characteristics

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response

    """
    def get_only_detect_person_legs_visible_callback(self, state: Bool):
        global ONLY_DETECT_PERSON_LEGS_VISIBLE
        # print(state.data)
        ONLY_DETECT_PERSON_LEGS_VISIBLE = state.data
        if ONLY_DETECT_PERSON_LEGS_VISIBLE:
            self.get_logger().info('ONLY_DETECT_PERSON_LEGS_VISIBLE = True')
        else:
            self.get_logger().info('ONLY_DETECT_PERSON_LEGS_VISIBLE = False')        

    def get_minimum_person_confidence_callback(self, state: Float32):
        global MIN_PERSON_CONF_VALUE
        # print(state.data)
        if 0.0 <= state.data <= 1.0:
            MIN_PERSON_CONF_VALUE = state.data
            self.get_logger().info('NEW MIN_PERSON_CONF_VALUE RECEIVED')    
        else:
            self.get_logger().info('ERROR SETTING MIN_PERSON_CONF_VALUE')    

    def get_minimum_keypoints_to_detect_person_callback(self, state: Int16):
        global MIN_KP_TO_DETECT_PERSON
        # print(state.data)
        if 0 < state.data <= self.N_KEYPOINTS - self.NUMBER_OF_LEGS_KP:  # all keypoints without the legs
            MIN_KP_TO_DETECT_PERSON = state.data
            self.get_logger().info('NEW MIN_KP_TO_DETECT_PERSON RECEIVED')    
        else:
            self.get_logger().info('ERROR SETTING MIN_KP_TO_DETECT_PERSON')  

    def get_only_detect_person_right_in_front_callback(self, state: Bool):
        global ONLY_DETECT_PERSON_RIGHT_IN_FRONT
        # print(state.data)
        ONLY_DETECT_PERSON_RIGHT_IN_FRONT = state.data
        if ONLY_DETECT_PERSON_RIGHT_IN_FRONT:
            self.get_logger().info('ONLY_DETECT_PERSON_RIGHT_IN_FRONT = True')
        else:
            self.get_logger().info('ONLY_DETECT_PERSON_RIGHT_IN_FRONT = False')  

    def get_only_detect_person_arm_raised_callback(self, state: Bool):
        global ONLY_DETECT_PERSON_ARM_RAISED
        # print(state.data)
        ONLY_DETECT_PERSON_ARM_RAISED = state.data
        if ONLY_DETECT_PERSON_ARM_RAISED:
            self.get_logger().info('ONLY_DETECT_PERSON_ARM_RAISED = True')
        else:
            self.get_logger().info('ONLY_DETECT_PERSON_ARM_RAISED = False')  
    """

    def get_color_image_head_callback(self, img: Image):
        # print("Received rgb cam")

        # only when activated via service, the model computes the person detection
        if self.ACTIVATE_YOLO_POSE:

            if not self.waiting_for_pcloud:
                # self.get_logger().info('Receiving color video frame')
                self.tempo_total = time.perf_counter()
                self.rgb_img = img

                # ROS2 Image Bridge for OpenCV
                current_frame = self.br.imgmsg_to_cv2(self.rgb_img, "bgr8")
                # current_frame_draw = current_frame.copy()

                # Getting image dimensions
                self.img_width = self.rgb_img.width
                self.img_height = self.rgb_img.height
                # print(self.img_width)
                # print(self.img_height)

                # The persist=True argument tells the tracker that the current image or frame is the next in a sequence and to expect tracks from the previous image in the current image.
                # tempo_calculo = time.perf_counter()
                self.results = self.model.track(current_frame, persist=True, tracker="bytetrack.yaml")
                # print('tempo calculo = ', time.perf_counter() - tempo_calculo)   # imprime o tempo de calculo em segundos

                # tf = time.perf_counter()

                # type(results) = <class 'list'>
                # type(results[0]) = <class 'ultralytics.engine.results.Results'>
                # type(results[0].keypoints) = <class 'ultralytics.engine.results.Keypoints'>
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
                
                # /*** ultralytics.engine.results.Keypoints ***/
                # A class for storing and   manipulating detection keypoints.
                # Attributes:
                # Name 	Type 	Description
                # xy 	Tensor 	A collection of keypoints containing x, y coordinates for each detection.
                # xyn 	Tensor 	A normalized version of xy with coordinates in the range [0, 1].
                # conf 	Tensor 	Confidence values associated with keypoints if available, otherwise None.

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


                # Index     Keypoint
                # 0         Nose                              2   1
                # 1         Left Eye                         / \ / \ 
                # 2         Right Eye                       4   0   3 
                # 3         Left Ear                        
                # 4         Right Ear                              
                # 5         Left Shoulder                  6---------5 
                # 6         Right Shoulder                / |       | \  
                # 7         Left Elbow                   /  |       |  \  
                # 8         Right Elbow                8/   |       |   \7  
                # 9         Left Wrist                  \   |       |   /
                # 10        Right Wrist                10\  |       |  /9 
                # 11        Left Hip                        ---------
                # 12        Right Hip                     12|       |11  
                # 13        Left Knee                       |       |
                # 14        Right Knee                    14|       |13  
                # 15        Left Ankle                      |       |
                # 16        Right Ankle                   16|       |15  

                # Calculate the number of persons detected
                num_persons = len(self.results[0].keypoints)
                if not self.results[0].keypoints.has_visible:
                    num_persons = 0

                # print("Number of people detected =", num_persons)
                self.get_logger().info(f"People detected: {num_persons}")

                self.center_torso_person_list = []
                self.center_head_person_list = []

                req2 = []
                
                for person_idx in range(num_persons):

                    keypoints_id = self.results[0].keypoints[person_idx]
                    boxes_id = self.results[0].boxes[person_idx]
                    
                    bb = BoundingBox()
                    bb.box_top_left_x = int(boxes_id.xyxy[0][0])
                    bb.box_top_left_y = int(boxes_id.xyxy[0][1])
                    bb.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
                    bb.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

                    # Conditions to safely select the pixel to calculate the person location 
                    if keypoints_id.conf[0][self.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and \
                        keypoints_id.conf[0][self.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE and \
                        keypoints_id.conf[0][self.HIP_LEFT_KP] > MIN_KP_CONF_VALUE and \
                        keypoints_id.conf[0][self.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE:
                    
                        ### After the yolo update, i must check if the conf value of the keypoint
                        person_center_x = int((keypoints_id.xy[0][self.SHOULDER_LEFT_KP][0] + keypoints_id.xy[0][self.SHOULDER_RIGHT_KP][0] + keypoints_id.xy[0][self.HIP_LEFT_KP][0] + keypoints_id.xy[0][self.HIP_RIGHT_KP][0]) / 4)
                        person_center_y = int((keypoints_id.xy[0][self.SHOULDER_LEFT_KP][1] + keypoints_id.xy[0][self.SHOULDER_RIGHT_KP][1] + keypoints_id.xy[0][self.HIP_LEFT_KP][1] + keypoints_id.xy[0][self.HIP_RIGHT_KP][1]) / 4)

                    else:
                        person_center_x = int(boxes_id.xyxy[0][0]+boxes_id.xyxy[0][2])//2
                        person_center_y = int(boxes_id.xyxy[0][1]+boxes_id.xyxy[0][3])//2

                    self.center_torso_person_list.append((person_center_x, person_center_y))

                    # head center position 
                    head_ctr = 0
                    head_center_x = 0
                    head_center_y = 0
                    if keypoints_id.conf[0][self.NOSE_KP] > MIN_KP_CONF_VALUE:
                        head_center_x += int(keypoints_id.xy[0][self.NOSE_KP][0])
                        head_center_y += int(keypoints_id.xy[0][self.NOSE_KP][1])
                        head_ctr +=1
                    if keypoints_id.conf[0][self.EYE_LEFT_KP] > MIN_KP_CONF_VALUE:
                        head_center_x += int(keypoints_id.xy[0][self.EYE_LEFT_KP][0])
                        head_center_y += int(keypoints_id.xy[0][self.EYE_LEFT_KP][1])
                        head_ctr +=1
                    if keypoints_id.conf[0][self.EYE_RIGHT_KP] > MIN_KP_CONF_VALUE:
                        head_center_x += int(keypoints_id.xy[0][self.EYE_RIGHT_KP][0])
                        head_center_y += int(keypoints_id.xy[0][self.EYE_RIGHT_KP][1])
                        head_ctr +=1
                    if keypoints_id.conf[0][self.EAR_LEFT_KP] > MIN_KP_CONF_VALUE:
                        head_center_x += int(keypoints_id.xy[0][self.EAR_LEFT_KP][0])
                        head_center_y += int(keypoints_id.xy[0][self.EAR_LEFT_KP][1])
                        head_ctr +=1
                    if keypoints_id.conf[0][self.EAR_RIGHT_KP] > MIN_KP_CONF_VALUE:
                        head_center_x += int(keypoints_id.xy[0][self.EAR_RIGHT_KP][0])
                        head_center_y += int(keypoints_id.xy[0][self.EAR_RIGHT_KP][1])
                        head_ctr +=1

                    if head_ctr > 0:
                        head_center_x /= head_ctr
                        head_center_y /= head_ctr
                    else:
                        head_center_x = int(boxes_id.xyxy[0][0]+boxes_id.xyxy[0][2])//2
                        head_center_y = int(boxes_id.xyxy[0][1]*3+boxes_id.xyxy[0][3])//4


                    head_center_x = int(head_center_x)
                    head_center_y = int(head_center_y)

                    self.center_head_person_list.append((head_center_x, head_center_y))

                    head_center_point = Pose2D()
                    head_center_point.x = float(head_center_x)
                    head_center_point.y = float(head_center_y)

                    torso_center_point = Pose2D()
                    torso_center_point.x = float(person_center_x)
                    torso_center_point.y = float(person_center_y)

                    # more points can be added here...

                    aux = BoundingBoxAndPoints()
                    aux.bbox = bb
                    aux.requested_point_coords.append(head_center_point)
                    aux.requested_point_coords.append(torso_center_point)

                    req2.append(aux)

                self.waiting_for_pcloud = True
                self.call_point_cloud_server(req2)
        
        else:
            yolov8_pose_filtered = ListOfDetectedPerson()
            self.person_pose_filtered_publisher.publish(yolov8_pose_filtered)

        

    def post_receiving_pcloud(self, new_pcloud):

        global GET_CHARACTERISTICS

        # print("points")
        # print(new_pcloud)

        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(self.rgb_img, "bgr8")
        current_frame_draw = current_frame.copy()
        # annotated_frame = self.results[0].plot()

        # Calculate the number of persons detected
        num_persons = len(self.results[0].keypoints)
        if not self.results[0].keypoints.has_visible:
            num_persons = 0

        # yolov8_pose = ListOfDetectedPerson()  # test removed person_pose (non-filtered)
        yolov8_pose_filtered = ListOfDetectedPerson()
        # num_persons_filtered = 0

        for person_idx in range(num_persons):
            keypoints_id = self.results[0].keypoints[person_idx]
            boxes_id = self.results[0].boxes[person_idx]
            ALL_CONDITIONS_MET = 1
            
            # condition whether the person has their arm raised, or waiving
            # at the current time, we are using the wrist coordinates and somparing with the nose coordinate
            is_hand_raised = False
            hand_raised = "None"
            if int(keypoints_id.xy[0][self.NOSE_KP][1]) >= int(keypoints_id.xy[0][self.WRIST_RIGHT_KP][1]) and \
                int(keypoints_id.xy[0][self.NOSE_KP][1]) >= int(keypoints_id.xy[0][self.WRIST_LEFT_KP][1]) and \
                keypoints_id.conf[0][self.NOSE_KP] > MIN_KP_CONF_VALUE and \
                keypoints_id.conf[0][self.WRIST_RIGHT_KP] > MIN_KP_CONF_VALUE and \
                keypoints_id.conf[0][self.WRIST_LEFT_KP] > MIN_KP_CONF_VALUE:
                    
                # print("Both Arms Up")
                hand_raised = "Both"
                is_hand_raised = True
            else:
                if int(keypoints_id.xy[0][self.NOSE_KP][1]) >= int(keypoints_id.xy[0][self.WRIST_RIGHT_KP][1]) and \
                    keypoints_id.conf[0][self.NOSE_KP] > MIN_KP_CONF_VALUE and \
                    keypoints_id.conf[0][self.WRIST_RIGHT_KP] > MIN_KP_CONF_VALUE:
                    # print("Right Arm Up")
                    hand_raised = "Right"
                    is_hand_raised = True
                elif int(keypoints_id.xy[0][self.NOSE_KP][1]) >= int(keypoints_id.xy[0][self.WRIST_LEFT_KP][1]) and \
                    keypoints_id.conf[0][self.NOSE_KP] > MIN_KP_CONF_VALUE and \
                    keypoints_id.conf[0][self.WRIST_LEFT_KP] > MIN_KP_CONF_VALUE:
                    # print("Left Arm Up")
                    hand_raised = "Left"
                    is_hand_raised = True
                else: 
                    # print("Both Arms Down")
                    hand_raised = "None"
                    is_hand_raised = False
            # print("Hand Raised:", hand_raised, is_hand_raised)

            # print(new_pcloud)
            new_person = DetectedPerson()
            new_person = self.add_person_to_detectedperson_msg(current_frame, current_frame_draw, boxes_id, keypoints_id, new_pcloud[person_idx].center_coords, \
                                                               self.center_torso_person_list[person_idx], self.center_head_person_list[person_idx], \
                                                               new_pcloud[person_idx].requested_point_coords[1], new_pcloud[person_idx].requested_point_coords[0], \
                                                               hand_raised)
            
            # adds people to "person_pose" without any restriction
            # yolov8_pose.persons.append(new_person) # test removed person_pose (non-filtered)
            
            legs_ctr = 0
            if keypoints_id.conf[0][self.KNEE_LEFT_KP] > MIN_KP_CONF_VALUE:
                legs_ctr+=1
            if keypoints_id.conf[0][self.KNEE_RIGHT_KP] > MIN_KP_CONF_VALUE:
                legs_ctr+=1
            if keypoints_id.conf[0][self.ANKLE_LEFT_KP] > MIN_KP_CONF_VALUE:
                legs_ctr+=1
            if keypoints_id.conf[0][self.ANKLE_RIGHT_KP] > MIN_KP_CONF_VALUE:
                legs_ctr+=1

            person_id = boxes_id.id
            if boxes_id.id == None:
                person_id = 0 

            # print("id = ", person_id)
            # print("conf", boxes_id.conf)
            # print("legs_ctr = ", legs_ctr)

            body_kp_high_conf_counter = 0
            for kp in range(self.N_KEYPOINTS - self.NUMBER_OF_LEGS_KP): # all keypoints without the legs
                if keypoints_id.conf[0][kp] > MIN_KP_CONF_VALUE:
                    body_kp_high_conf_counter+=1

            # print("body_kp_high_conf_counter = ", body_kp_high_conf_counter)


            # changed this condition from:
            # 0 < new_person.position_relative.y < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD:
            # to:
            # -ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD < new_person.position_relative.y < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD:
            # this way, this condition can be used for both checking if the person is right in front and checking if the person is following the robot

            center_comm_position = False
            if -ONLY_DETECT_PERSON_RIGHT_IN_FRONT_X_THRESHOLD < new_person.position_relative.x < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_X_THRESHOLD and \
             -ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD < new_person.position_relative.y < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD:
               center_comm_position = True


            # checks whether the person confidence is above a defined level
            if not boxes_id.conf >= MIN_PERSON_CONF_VALUE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print("- Misses minimum person confidence level")

            # checks if flag to detect people whose legs are visible 
            if not legs_ctr >= NUMBER_OF_LEG_KP_TO_BE_DETECTED and ONLY_DETECT_PERSON_LEGS_VISIBLE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print("- Misses legs visible flag")
            
            # checks whether the minimum number if body keypoints (excluding legs) has high confidence
            if not body_kp_high_conf_counter >= MIN_KP_TO_DETECT_PERSON:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print("- Misses minimum number of body keypoints")

            # checks if flag to detect people right in front of the robot 
            if not center_comm_position and ONLY_DETECT_PERSON_RIGHT_IN_FRONT:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print(" - Misses not being right in front of the robot")
            
            # checks if flag to detect people with their arm raised or waving (requesting assistance)
            if not is_hand_raised and ONLY_DETECT_PERSON_ARM_RAISED:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print(" - Misses not being with their arm raised")

            if ALL_CONDITIONS_MET:
                # num_persons_filtered+=1

                # characteristics will only be updated after we confirm that the person is inside the filteredpersons
                # otherwise the large amount of time spent getting the characteristics from the models is applied to
                # every detected person and not only the filtered 
                is_cropped_face = False
                if GET_CHARACTERISTICS:
                    # in order to predict the ethnicity, age and gender, it is necessary to first cut out the face of the detected person
                    is_cropped_face, cropped_face = self.crop_face(current_frame, current_frame_draw, new_person)

                    if is_cropped_face:
                        new_person.ethnicity, new_person.ethnicity_probability = self.get_ethnicity_prediction(cropped_face) # says whether the person white, asian, african descendent, middle eastern, ...
                        new_person.age_estimate, new_person.age_estimate_probability = self.get_age_prediction(cropped_face) # says an approximate age gap like 25-35 ...
                        new_person.gender, new_person.gender_probability = self.get_gender_prediction(cropped_face) # says whether the person is male or female
                        print(new_person.ethnicity, new_person.age_estimate, new_person.gender)        
                # adds people to "person_pose_filtered" with selected filters
                yolov8_pose_filtered.persons.append(new_person)
              
                if self.DEBUG_DRAW:
                    
                    red_yp = (56, 56, 255)
                    lblue_yp = (255,128,0)
                    green_yp = (0,255,0)
                    orange_yp = (51,153,255)
                    magenta_yp = (255, 51, 255)
                    white_yp = (255, 255, 255)

                    # /*** BOXES ***/

                    # print(f"Person index BOXES: {person_idx}") # just the index of the person
                    # print(boxes_id)
                    # print(boxes_id.xyxy)
                    # print(int(boxes_id.xyxy[0][1]))

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

                    if DRAW_PERSON_BOX:
                        # draws the bounding box around the person
                        cv2.rectangle(current_frame_draw, start_point, end_point, red_yp , 4) 
                    
                    if DRAW_PERSON_CONF and not DRAW_PERSON_ID:
                        # draws the background for the confidence of each person
                        cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, red_yp , -1) 
                        
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
                    
                    elif not DRAW_PERSON_CONF and DRAW_PERSON_ID:
                        if person_id != 0:

                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+10, end_point_text_rect[1]) , red_yp , -1) 
                            
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{str(int(person_id))}",
                                (start_point_text[0], start_point_text[1]),
                                cv2.FONT_HERSHEY_DUPLEX,
                                1,
                                (0, 0, 0),
                                1,
                                cv2.LINE_AA
                            ) 

                    elif DRAW_PERSON_CONF and DRAW_PERSON_ID:

                        if person_id != 0:

                            # draws the background for the confidence of each person
                            cv2.rectangle(current_frame_draw, start_point_text_rect, (end_point_text_rect[0]+70, end_point_text_rect[1]) , red_yp , -1) 
                            
                            current_frame_draw = cv2.putText(
                                current_frame_draw,
                                # f"{round(float(per.conf),2)}",
                                f"{str(int(person_id))}",
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
                            cv2.rectangle(current_frame_draw, start_point_text_rect, end_point_text_rect, red_yp , -1) 
                            
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

                    # /*** KEYPOINTS ***/

                    # print(keypoints_id.xy)
                    # print(keypoints_id.xy[0][0])
                    # print(keypoints_id.conf)        
                    
                    if DRAW_PERSON_KP:


                    
                        self.line_between_two_keypoints(current_frame_draw, self.NOSE_KP, self.EYE_LEFT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.NOSE_KP, self.EYE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        # self.line_between_two_keypoints(current_frame_draw, self.EYE_LEFT_KP, self.EYE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.EYE_LEFT_KP, self.EAR_LEFT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.EYE_RIGHT_KP, self.EAR_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.EAR_LEFT_KP, self.SHOULDER_LEFT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.EAR_RIGHT_KP, self.SHOULDER_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, green_yp)

                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_LEFT_KP, self.SHOULDER_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, lblue_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_LEFT_KP, self.ELBOW_LEFT_KP, keypoints_id.xy, keypoints_id.conf, lblue_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_RIGHT_KP, self.ELBOW_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, lblue_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.ELBOW_LEFT_KP, self.WRIST_LEFT_KP, keypoints_id.xy, keypoints_id.conf, lblue_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.ELBOW_RIGHT_KP, self.WRIST_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, lblue_yp)


                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_RIGHT_KP, self.HIP_LEFT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_LEFT_KP, self.HIP_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)

                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_LEFT_KP, self.HIP_LEFT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_RIGHT_KP, self.HIP_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.HIP_LEFT_KP, self.HIP_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)

                        self.line_between_two_keypoints(current_frame_draw, self.HIP_LEFT_KP, self.KNEE_LEFT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.HIP_RIGHT_KP, self.KNEE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.KNEE_LEFT_KP, self.ANKLE_LEFT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                        self.line_between_two_keypoints(current_frame_draw, self.KNEE_RIGHT_KP, self.ANKLE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)

                        for kp in range(self.N_KEYPOINTS):
                            if keypoints_id.conf[0][kp] > MIN_KP_CONF_VALUE:
                                if kp >= 0 and kp < 5:
                                    c = green_yp # green
                                elif kp >= 5 and kp < 11:
                                    c = lblue_yp # something blue 
                                elif kp >= 11 and kp < 17:
                                    c = orange_yp # orange
                                center_p = (int(keypoints_id.xy[0][kp][0]), int(keypoints_id.xy[0][kp][1]))
                                cv2.circle(current_frame_draw, center_p, 5, c, -1)
                            else:
                                if DRAW_LOW_CONF_KP:
                                    c = (0,0,255)
                                    center_p = (int(keypoints_id.xy[0][kp][0]), int(keypoints_id.xy[0][kp][1]))
                                    cv2.circle(current_frame_draw, center_p, 5, c, -1)

                        center_p_ = (int(boxes_id.xyxy[0][0]+boxes_id.xyxy[0][2])//2), (int(boxes_id.xyxy[0][1]+boxes_id.xyxy[0][3])//2)
                        cv2.circle(current_frame_draw, center_p_, 5, (128, 128, 128), -1)
                        cv2.circle(current_frame_draw, self.center_head_person_list[person_idx], 5, (255, 255, 255), -1)
                        cv2.circle(current_frame_draw, self.center_torso_person_list[person_idx], 5, (255, 255, 255), -1)
                       
                    if DRAW_PERSON_LOCATION_COORDS:
                        cv2.putText(current_frame_draw, '('+str(round(new_person.position_relative.x,2))+
                                    ', '+str(round(new_person.position_relative.y,2))+
                                    ', '+str(round(new_person.position_relative.z,2))+')',
                                    self.center_torso_person_list[person_idx], cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                    
                    if DRAW_PERSON_LOCATION_HOUSE_FURNITURE:
                        cv2.putText(current_frame_draw, new_person.room_location+" - "+new_person.furniture_location,
                                    (self.center_torso_person_list[person_idx][0], self.center_torso_person_list[person_idx][1]+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    if DRAW_PERSON_POINTING_INFO:
                        if new_person.pointing_at != "None":
                            cv2.putText(current_frame_draw, new_person.pointing_at+" "+new_person.pointing_with_arm,
                                        (self.center_torso_person_list[person_idx][0], self.center_torso_person_list[person_idx][1]+60), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

                    if DRAW_PERSON_HAND_RAISED:
                        # if hand_raised != "None":
                        cv2.putText(current_frame_draw, "Hand Raised:"+hand_raised,
                                    (self.center_torso_person_list[person_idx][0], self.center_torso_person_list[person_idx][1]+90), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        
                    if DRAW_PERSON_HEIGHT:
                        cv2.putText(current_frame_draw, str(round(new_person.height,2)),
                                    (self.center_torso_person_list[person_idx][0], self.center_torso_person_list[person_idx][1]+120), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        
                    if DRAW_PERSON_CLOTHES_COLOR:
                        cv2.putText(current_frame_draw, new_person.shirt_color + "(" + str(new_person.shirt_rgb.red) + ", " + str(new_person.shirt_rgb.green) + ", " + str(new_person.shirt_rgb.blue) + ")",
                                    # (self.center_head_person_list[person_idx][0], self.center_head_person_list[person_idx][1]), cv2.FONT_HERSHEY_DUPLEX, 1, (new_person.shirt_rgb.blue, new_person.shirt_rgb.green, new_person.shirt_rgb.red), 1, cv2.LINE_AA)
                                    (self.center_head_person_list[person_idx][0], self.center_head_person_list[person_idx][1]), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        
                        cv2.putText(current_frame_draw, new_person.pants_color + "(" + str(new_person.pants_rgb.red) + ", " + str(new_person.pants_rgb.green) + ", " + str(new_person.pants_rgb.blue) + ")",
                                    # (self.center_head_person_list[person_idx][0], self.center_head_person_list[person_idx][1]+30), cv2.FONT_HERSHEY_DUPLEX, 1, (new_person.pants_rgb.blue, new_person.pants_rgb.green, new_person.pants_rgb.red), 1, cv2.LINE_AA)
                                    (self.center_head_person_list[person_idx][0], self.center_head_person_list[person_idx][1]+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    if DRAW_CHARACTERISTICS and GET_CHARACTERISTICS and is_cropped_face:
                        cv2.putText(current_frame_draw, str(round(new_person.height,2))+" / "+new_person.age_estimate+" / "+new_person.gender+" / "+new_person.ethnicity,
                                    (self.center_head_person_list[person_idx][0], self.center_head_person_list[person_idx][1]+60), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        

            # print("===")

        # self.person_pose_publisher.publish(yolov8_pose) # test removed person_pose (non-filtered)
        self.person_pose_filtered_publisher.publish(yolov8_pose_filtered)

        # print("____END____")

        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time
        self.fps = str(self.fps)
        # print("fps = " + self.fps)

        if self.DEBUG_DRAW:
            # putting the FPS count on the frame
            cv2.putText(current_frame_draw, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame_draw, 'np:' + str(len(yolov8_pose_filtered.persons)) + '/' + str(num_persons), (180, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
            # cv2.imshow("Yolo Pose Detection", annotated_frame)
            # cv2.imshow("Camera Image", current_frame)
            cv2.waitKey(1)
        
        # print('tempo parcial = ', tf - ti)
        # print('tempo total = ', time.perf_counter() - self.tempo_total)   # imprime o tempo de calculo em segundos
        self.get_logger().info(f"Time Yolo_Pose: {round(time.perf_counter() - self.tempo_total,2)}")


    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta


    def add_person_to_detectedperson_msg(self, current_frame, current_frame_draw, boxes_id, keypoints_id, center_person_filtered, center_torso_person, center_head_person, torso_localisation, head_localisation, arm_raised):
        # receives the box and keypoints of a specidic person and returns the detected person 
        # it can be done in a way that is only made once per person and both 'person_pose' and 'person_pose_filtered'

        global GET_CHARACTERISTICS

        person_id = boxes_id.id
        if boxes_id.id == None:
            person_id = 0 

        new_person = DetectedPerson()

        new_person.image_rgb_frame = self.rgb_img

        new_person.index = int(person_id)
        new_person.confidence = float(boxes_id.conf)
        new_person.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_person.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_person.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_person.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        new_person.arm_raised = arm_raised
        new_person.body_posture = "None" # still missing... (says whether the person is standing up, sitting, laying down, ...)

        new_person.kp_nose_x = int(keypoints_id.xy[0][self.NOSE_KP][0])
        new_person.kp_nose_y = int(keypoints_id.xy[0][self.NOSE_KP][1])
        new_person.kp_nose_conf = float(keypoints_id.conf[0][self.NOSE_KP])

        new_person.kp_eye_left_x = int(keypoints_id.xy[0][self.EYE_LEFT_KP][0])
        new_person.kp_eye_left_y = int(keypoints_id.xy[0][self.EYE_LEFT_KP][1])
        new_person.kp_eye_left_conf = float(keypoints_id.conf[0][self.EYE_LEFT_KP])

        new_person.kp_eye_right_x = int(keypoints_id.xy[0][self.EYE_RIGHT_KP][0])
        new_person.kp_eye_right_y = int(keypoints_id.xy[0][self.EYE_RIGHT_KP][1])
        new_person.kp_eye_right_conf = float(keypoints_id.conf[0][self.EYE_RIGHT_KP])

        new_person.kp_ear_left_x = int(keypoints_id.xy[0][self.EAR_LEFT_KP][0])
        new_person.kp_ear_left_y = int(keypoints_id.xy[0][self.EAR_LEFT_KP][1])
        new_person.kp_ear_left_conf = float(keypoints_id.conf[0][self.EAR_LEFT_KP])

        new_person.kp_ear_right_x = int(keypoints_id.xy[0][self.EAR_RIGHT_KP][0])
        new_person.kp_ear_right_y = int(keypoints_id.xy[0][self.EAR_RIGHT_KP][1])
        new_person.kp_ear_right_conf = float(keypoints_id.conf[0][self.EAR_RIGHT_KP])

        new_person.kp_shoulder_left_x = int(keypoints_id.xy[0][self.SHOULDER_LEFT_KP][0])
        new_person.kp_shoulder_left_y = int(keypoints_id.xy[0][self.SHOULDER_LEFT_KP][1])
        new_person.kp_shoulder_left_conf = float(keypoints_id.conf[0][self.SHOULDER_LEFT_KP])

        new_person.kp_shoulder_right_x = int(keypoints_id.xy[0][self.SHOULDER_RIGHT_KP][0])
        new_person.kp_shoulder_right_y = int(keypoints_id.xy[0][self.SHOULDER_RIGHT_KP][1])
        new_person.kp_shoulder_right_conf = float(keypoints_id.conf[0][self.SHOULDER_RIGHT_KP])

        new_person.kp_elbow_left_x = int(keypoints_id.xy[0][self.ELBOW_LEFT_KP][0])
        new_person.kp_elbow_left_y = int(keypoints_id.xy[0][self.ELBOW_LEFT_KP][1])
        new_person.kp_elbow_left_conf = float(keypoints_id.conf[0][self.ELBOW_LEFT_KP])

        new_person.kp_elbow_right_x = int(keypoints_id.xy[0][self.ELBOW_RIGHT_KP][0])
        new_person.kp_elbow_right_y = int(keypoints_id.xy[0][self.ELBOW_RIGHT_KP][1])
        new_person.kp_elbow_right_conf = float(keypoints_id.conf[0][self.ELBOW_RIGHT_KP])

        new_person.kp_wrist_left_x = int(keypoints_id.xy[0][self.WRIST_LEFT_KP][0])
        new_person.kp_wrist_left_y = int(keypoints_id.xy[0][self.WRIST_LEFT_KP][1])
        new_person.kp_wrist_left_conf = float(keypoints_id.conf[0][self.WRIST_LEFT_KP])

        new_person.kp_wrist_right_x = int(keypoints_id.xy[0][self.WRIST_RIGHT_KP][0])
        new_person.kp_wrist_right_y = int(keypoints_id.xy[0][self.WRIST_RIGHT_KP][1])
        new_person.kp_wrist_right_conf = float(keypoints_id.conf[0][self.WRIST_RIGHT_KP])

        new_person.kp_hip_left_x = int(keypoints_id.xy[0][self.HIP_LEFT_KP][0])
        new_person.kp_hip_left_y = int(keypoints_id.xy[0][self.HIP_LEFT_KP][1])
        new_person.kp_hip_left_conf = float(keypoints_id.conf[0][self.HIP_LEFT_KP])

        new_person.kp_hip_right_x = int(keypoints_id.xy[0][self.HIP_RIGHT_KP][0])
        new_person.kp_hip_right_y = int(keypoints_id.xy[0][self.HIP_RIGHT_KP][1])
        new_person.kp_hip_right_conf = float(keypoints_id.conf[0][self.HIP_RIGHT_KP])

        new_person.kp_knee_left_x = int(keypoints_id.xy[0][self.KNEE_LEFT_KP][0])
        new_person.kp_knee_left_y = int(keypoints_id.xy[0][self.KNEE_LEFT_KP][1])
        new_person.kp_knee_left_conf = float(keypoints_id.conf[0][self.KNEE_LEFT_KP])

        new_person.kp_knee_right_x = int(keypoints_id.xy[0][self.KNEE_RIGHT_KP][0])
        new_person.kp_knee_right_y = int(keypoints_id.xy[0][self.KNEE_RIGHT_KP][1])
        new_person.kp_knee_right_conf = float(keypoints_id.conf[0][self.KNEE_RIGHT_KP])

        new_person.kp_ankle_left_x = int(keypoints_id.xy[0][self.ANKLE_LEFT_KP][0])
        new_person.kp_ankle_left_y = int(keypoints_id.xy[0][self.ANKLE_LEFT_KP][1])
        new_person.kp_ankle_left_conf = float(keypoints_id.conf[0][self.ANKLE_LEFT_KP])

        new_person.kp_ankle_right_x = int(keypoints_id.xy[0][self.ANKLE_RIGHT_KP][0])
        new_person.kp_ankle_right_y = int(keypoints_id.xy[0][self.ANKLE_RIGHT_KP][1])
        new_person.kp_ankle_right_conf = float(keypoints_id.conf[0][self.ANKLE_RIGHT_KP])

        new_person.body_center_x = center_torso_person[0]
        new_person.body_center_y = center_torso_person[1]

        new_person.head_center_x = center_head_person[0]
        new_person.head_center_y = center_head_person[1]

        # changes the axis of point cloud coordinates to fit with robot axis
        person_rel_pos = Point()
        # person_rel_pos.x = -torso_localisation.y/1000
        # person_rel_pos.y =  torso_localisation.x/1000
        person_rel_pos.x =  -center_person_filtered.y/1000
        person_rel_pos.y =  center_person_filtered.x/1000
        person_rel_pos.z =  center_person_filtered.z/1000
        
        new_person.position_relative = person_rel_pos
        
        # calculate the absolute position according to the robot localisation
        angle_person = math.atan2(person_rel_pos.x, person_rel_pos.y)
        dist_person = math.sqrt(person_rel_pos.x**2 + person_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_person - self.robot_t)

        target_x = dist_person * math.cos(theta_aux) + self.robot_x
        target_y = dist_person * math.sin(theta_aux) + self.robot_y

        a_ref = (target_x, target_y)
        # print("Rel:", (person_rel_pos.x, person_rel_pos.y), "Abs:", a_ref)

        person_abs_pos = Point()
        person_abs_pos.x = target_x
        person_abs_pos.y = target_y
        person_abs_pos.z = center_person_filtered.z/1000
        
        new_person.position_absolute = person_abs_pos

        # changes the axis of point cloud coordinates to fit with robot axis
        head_rel_pos = Point()
        # head_rel_pos.x = -head_localisation.y/1000
        # head_rel_pos.y =  head_localisation.x/1000
        head_rel_pos.x =  -head_localisation.y/1000
        head_rel_pos.y =  head_localisation.x/1000
        head_rel_pos.z =  head_localisation.z/1000

        new_person.position_relative_head = head_rel_pos
        
        # calculate the absolute head position according to the robot localisation
        angle_head = math.atan2(head_rel_pos.x, head_rel_pos.y)
        dist_head = math.sqrt(head_rel_pos.x**2 + head_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_head - self.robot_t)

        target_x = dist_head * math.cos(theta_aux) + self.robot_x
        target_y = dist_head * math.sin(theta_aux) + self.robot_y

        a_ref = (target_x, target_y)
        # print("Rel:", (head_rel_pos.x, head_rel_pos.y), "Abs:", a_ref)

        head_abs_pos = Point()
        head_abs_pos.x = target_x
        head_abs_pos.y = target_y
        head_abs_pos.z = head_localisation.z/1000
        
        new_person.position_absolute_head = head_abs_pos

        new_person.height = head_localisation.z/1000 + 0.08 # average person middle of face to top of head distance

        new_person.room_location, new_person.furniture_location = self.position_to_house_rooms_and_furniture(person_abs_pos)

        new_person.pointing_at, new_person.pointing_with_arm = self.arm_pointing_at(new_person)

        new_person.shirt_color, new_person.shirt_rgb = self.get_shirt_color(new_person, current_frame, current_frame_draw) 
        new_person.pants_color, new_person.pants_rgb = self.get_pants_color(new_person, current_frame, current_frame_draw) 

        # characteristics will only be updated after we confirm that the person is inside the filteredpersons
        # otherwise the large amount of time spent getting the characteristics from the models is applied to
        # every detected person and not only the filtered 

        # new_person.pointing_at = "None"
        # new_person.pointing_with_arm = "None"
        # new_person.shirt_color = "None"
        # new_person.pants_color = "None"
        new_person.ethnicity = "None"
        new_person.ethnicity_probability = 0.0
        new_person.age_estimate = "None"
        new_person.age_estimate_probability = 0.0
        new_person.gender = "None"
        new_person.gender_probability = 0.0


        return new_person

    def crop_face(self, current_frame, current_frame_draw, new_person):

        global DRAW_FACE_RECOGNITION

        # y1 = top of bounding box y
        # y2 = y of lowest height shoulder
        # x1 = keypoint more to the left
        # x2 = keypoint more to the right
        
        # using all face and shoulders keypoints to make sure face is correctly detected
        if new_person.kp_shoulder_right_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_shoulder_left_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_eye_right_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_eye_left_conf > MIN_KP_CONF_VALUE and \
            new_person.kp_nose_conf > MIN_KP_CONF_VALUE:
            # new_person.kp_ear_right_conf > MIN_KP_CONF_VALUE and \
            # new_person.kp_ear_left_conf > MIN_KP_CONF_VALUE and \
            
            y1 = new_person.box_top_left_y
            y2 = max(new_person.kp_shoulder_right_y, new_person.kp_shoulder_left_y)

            x1 = min(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)
            x2 = max(new_person.kp_shoulder_right_x, new_person.kp_shoulder_left_x, new_person.kp_nose_x, new_person.kp_eye_right_x, new_person.kp_eye_left_x)

            if DRAW_FACE_RECOGNITION:
                cv2.rectangle(current_frame_draw, (x1, y1), (x2, y2), (0, 255, 255) , 4) 
                
            # cv2.imwrite("cropped_face_test.jpg", current_frame[y1:y2, x1:x2])
            
            return True, current_frame[y1:y2, x1:x2]

        else:
            return False, current_frame
        
    def get_age_prediction(self, cropped_face):
        
        blob = cv2.dnn.blobFromImage(cropped_face, 1.0, (227, 227), (78.4263377603, 87.7689143744, 114.895847746), swapRB=False)
        self.ageNet.setInput(blob)
        agePreds = self.ageNet.forward()

        age_index = agePreds[0].argmax()
        # age_ranges = ["(0-2)", "(4-6)", "(8-12)", "(15-20)", "(25-32)", "(38-43)", "(48-53)", "(60-100)"]
        age_ranges = ["Under 20", "Under 20", "Under 20", "Under 20", "Between 18 and 32", "Between 28 and 42", "Between 40 and 60", "Over 60"]
        age = age_ranges[age_index]

        # Filters for the people that are more likely to show up to the robot
        # if age_index < 2:
        #     age = "(15 and 22)"
        # elif age_index >= 4:
        #     age = "(23 and 32)"
        
        # Verificar se a previsão está disponível
        if age is not None:
            age = age
            age_prob = float(round(max(agePreds[0]),2))
        else:
            age = "None"
            age_prob = 0.0

        # print("age predictions:", age, age_prob, agePreds[0])

        return age, age_prob

    def get_gender_prediction(self, cropped_face):

        color_img = cv2.cvtColor(cropped_face, cv2.COLOR_BGR2RGB)
        img = cv2.resize(color_img, (224, 224))
        normalized_img = img / 255.0
        expanded_img = np.expand_dims(normalized_img, axis=0)

        # Realizar a previsão usando o modelo carregado
        predictions = self.gender_model.predict(expanded_img)

        # Obter a previsão do gênero
        gender_predominante_index = np.argmax(predictions, axis=1)
        gender_predominante = ['Female', 'Male'][gender_predominante_index[0]]

        # Verificar se a previsão está disponível
        if gender_predominante is not None:
            gender = gender_predominante
            gender_prob = float(round(max(predictions[0]),2))
        else:
            gender = "None"
            gender_prob = 0.0
        
        # print("gender predictions:", gender, gender_prob, predictions[0])

        return gender, gender_prob

    def get_ethnicity_prediction(self, cropped_face):

        color_img = cv2.cvtColor(cropped_face, cv2.COLOR_BGR2RGB)
        img = cv2.resize(color_img, (224, 224))
        normalized_img = img / 255.0
        expanded_img = np.expand_dims(normalized_img, axis=0)

        # Realizar a previsão usando o modelo carregado
        predictions = self.race_model.predict(expanded_img)

        # Obter a previsão da raça
        race_labels = ['Asian', 'Indian', 'Black', 'Caucasian', 'Middle Eastern', 'Hispanic']
        race_predominante_index = np.argmax(predictions, axis=1)
        race_predominante = race_labels[race_predominante_index[0]]

        # Verificar se a previsão está disponível
        if race_predominante is not None:
            race = race_predominante
            race_prob = float(round(max(predictions[0]),2))
        else:
            race = "None"
            race_prob = 0.0
        
        # print("ethnicity predictions:", race, race_prob, predictions[0])

        return race, race_prob
    
    def line_between_two_keypoints(self, current_frame_draw, KP_ONE, KP_TWO, xy, conf, colour):

        if conf[0][KP_ONE] > MIN_KP_CONF_VALUE and conf[0][KP_TWO] > MIN_KP_CONF_VALUE:    
            p1 = (int(xy[0][KP_ONE][0]), int(xy[0][KP_ONE][1]))
            p2 = (int(xy[0][KP_TWO][0]), int(xy[0][KP_TWO][1]))
            cv2.line(current_frame_draw, p1, p2, colour, 2) 
        else:
            if DRAW_LOW_CONF_KP: 
                p1 = (int(xy[0][KP_ONE][0]), int(xy[0][KP_ONE][1]))
                p2 = (int(xy[0][KP_TWO][0]), int(xy[0][KP_TWO][1]))
                cv2.line(current_frame_draw, p1, p2, (0,0,255), 2) 


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

     
    def arm_pointing_at(self, person):

        MIN_ANGLE_POINTING = 25

        right_shoulder = (person.kp_shoulder_right_x, person.kp_shoulder_right_y)
        right_wrist = (person.kp_wrist_right_x, person.kp_wrist_right_y)
        right_hip = (person.kp_hip_right_x, person.kp_hip_right_y)

        left_shoulder = (person.kp_shoulder_left_x, person.kp_shoulder_left_y)
        left_wrist = (person.kp_wrist_left_x, person.kp_wrist_left_y)
        left_hip = (person.kp_hip_left_x, person.kp_hip_left_y)

        # The sides are relative to the person, so the right side is linked with the person right arm!
        if person.kp_shoulder_right_conf > MIN_KP_CONF_VALUE and \
            person.kp_wrist_right_conf > MIN_KP_CONF_VALUE and \
            person.kp_hip_right_conf > MIN_KP_CONF_VALUE:
            theta_right = self.calculate_3angle(right_shoulder, right_wrist, right_hip)
        else:
            theta_right = 0.0

        # The sides are relative to the person, so the right side is linked with the person right arm!
        if person.kp_shoulder_left_conf > MIN_KP_CONF_VALUE and \
            person.kp_wrist_left_conf > MIN_KP_CONF_VALUE and \
            person.kp_hip_left_conf > MIN_KP_CONF_VALUE:
            theta_left = self.calculate_3angle(left_shoulder, left_wrist, left_hip)
        else:
            theta_left = 0.0
        
        side_pointed = "None"
        arm_pointed_with = "None"

        # print("Sides0:", left_wrist[0], left_hip[0], right_wrist[0], right_hip[0])    
        # print("Sides1:", left_wrist[1], left_hip[1], right_wrist[1], right_hip[1])      

        if theta_left > MIN_ANGLE_POINTING:
            if left_wrist[0] < left_hip[0]:
                arm_pointed_with = "Left Arm"
                side_pointed = "Right"
            else:
                arm_pointed_with = "Left Arm"
                side_pointed = "Left"
        
        elif theta_right > MIN_ANGLE_POINTING:
            if right_wrist[0] < right_hip[0]:
                arm_pointed_with = "Right Arm"
                side_pointed = "Right"
            else:
                arm_pointed_with = "Right Arm"
                side_pointed = "Left"
        
        return side_pointed, arm_pointed_with


    def calculate_3angle(self, p1, p2, p3):
        vector_1 = (p2[0] - p1[0], p2[1] - p1[1])
        vector_2 = (p3[0] - p1[0], p3[1] - p1[1])

        dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
        
        try:
            magnitude_1 = math.sqrt(vector_1[0]**2 + vector_1[1]**2)
            magnitude_2 = math.sqrt(vector_2[0]**2 + vector_2[1]**2)

            # try catch is here in case any of the magnitudes is = 0, in that case an anle of 0 is returned for safety
            theta = math.acos(dot_product / (magnitude_1 * magnitude_2))
            theta_degrees = math.degrees(theta)
            return theta_degrees
        
        except:
            return 0
        

    def get_shirt_color(self, new_person, current_frame, current_frame_draw):
        color_name = "None"
        color_rgb = RGB()

        if new_person.kp_shoulder_left_conf > MIN_KP_CONF_VALUE and new_person.kp_shoulder_right_conf > MIN_KP_CONF_VALUE:
            color_name, color_value_rgb, n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (new_person.kp_shoulder_left_x, new_person.kp_shoulder_left_y), (new_person.kp_shoulder_right_x, new_person.kp_shoulder_right_y))
    
            color_rgb.red = int(color_value_rgb[0])
            color_rgb.green = int(color_value_rgb[1])
            color_rgb.blue = int(color_value_rgb[2])

        return color_name, color_rgb


    def get_pants_color(self, new_person, current_frame, current_frame_draw):
        left_leg_color_name = "None"
        left_leg_color_rgb = RGB()
        left_leg_n_points = 0
        right_leg_color_name = "None"
        right_leg_color_rgb = RGB()
        right_leg_n_points = 0
        color_name = "None"
        color_rgb = RGB()

        if new_person.kp_hip_left_conf > MIN_KP_CONF_VALUE and new_person.kp_knee_left_conf > MIN_KP_CONF_VALUE:
            left_leg_color_name, left_leg_color_rgb, left_leg_n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (new_person.kp_hip_left_x, new_person.kp_hip_left_y), (new_person.kp_knee_left_x, new_person.kp_knee_left_y))

        if new_person.kp_hip_right_conf > MIN_KP_CONF_VALUE and new_person.kp_knee_right_conf > MIN_KP_CONF_VALUE:
            right_leg_color_name, right_leg_color_rgb, right_leg_n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (new_person.kp_hip_right_x, new_person.kp_hip_right_y), (new_person.kp_knee_right_x, new_person.kp_knee_right_y))

        if left_leg_n_points >= right_leg_n_points and left_leg_n_points > 0:
            color_name = left_leg_color_name
            color_rgb.red = int(left_leg_color_rgb[0])
            color_rgb.green = int(left_leg_color_rgb[1])
            color_rgb.blue = int(left_leg_color_rgb[2])

        elif left_leg_n_points < right_leg_n_points and right_leg_n_points > 0:
            color_name = right_leg_color_name
            color_rgb.red = int(right_leg_color_rgb[0])
            color_rgb.green = int(right_leg_color_rgb[1])
            color_rgb.blue = int(right_leg_color_rgb[2])
        
        return color_name, color_rgb
    

    def rgb_to_string_tr(self, color_value_rgb):

        # Convert the RGB to BGR
        color_value_bgr = (color_value_rgb[2], color_value_rgb[1], color_value_rgb[0])

        # Define the RGB pixel value
        bgr_pixel = np.array([[color_value_bgr]], dtype=np.uint8)

        # Convert the BGR pixel to HSV
        hsv_pixel = cv2.cvtColor(bgr_pixel, cv2.COLOR_BGR2HSV)

        # Extract the HSV values
        h, s, vv = hsv_pixel[0][0]
        hsv_info = (h, s, vv)
        # print("HSV values:", h, s, vv)

        ### Step 1: Define Color using HSV: Hue

        # Boundaries between colors
        red_orange_border     = 9
        orange_yellow_border  = 20
        yellow_green_border   = 31
        green_cyan_border     = 78
        cyan_blue_border      = 100
        blue_purple_border    = 131
        purple_magenta_border = 150
        magenta_red_border    = 172

        # Averages and Standard Deviations for White, Grey and Black colors
        MIN_AVG_FOR_BLACK = 30 
        MIN_AVG_FOR_WHITE = 130 # was 220 
        MIN_STD_DEV_FOR_GREY = 8.0 
        MIN_STD_DEV_FOR_BW = 15.0 

        color_ranges = {
            "Red":     (0,                      red_orange_border),
            "Orange":  (red_orange_border,      orange_yellow_border),
            "Yellow":  (orange_yellow_border,   yellow_green_border),
            "Green":   (yellow_green_border,    green_cyan_border),
            "Cyan":    (green_cyan_border,      cyan_blue_border),
            "Blue":    (cyan_blue_border,       blue_purple_border),
            "Purple":  (blue_purple_border,     purple_magenta_border),
            "Magenta": (purple_magenta_border,  magenta_red_border),
            "Red2":    (magenta_red_border,     180),
        }

        # color thresholds to choose whether it is a light color, a normal color or a dark color
        color_thresholds = {
            "Red":     (30,  138),
            "Orange":  (100, 170),
            "Yellow":  (120, 210),
            "Green":   (30,  138),
            "Cyan":    (120, 210),
            "Blue":    (30,  138),
            "Purple":  (100, 170),
            "Magenta": (120, 210),
            "Red2":    (30,  138),
        }

        # Function to get color name from HSV values
        color_name = "None"
        hsv_ = (h, s, vv) 
        hue = hsv_[0]
        for color, (lower, upper) in color_ranges.items():
            if lower <= hue < upper:
                color_name = color
        
        ### Step 2: Calculate RGB mean and std_dev to calaulate grey, white and black

        avg_rgb_values = sum(color_value_rgb)/len(color_value_rgb)
        std_dev = math.sqrt(sum((x - avg_rgb_values) ** 2 for x in color_value_rgb) / len(color_value_rgb))
        # print("Avg:", round(avg_rgb_values, 2), "Std_dev:", round(std_dev, 2))

        if avg_rgb_values >= MIN_AVG_FOR_WHITE and std_dev < MIN_STD_DEV_FOR_BW:
            color_name = "White" 
        elif avg_rgb_values <= MIN_AVG_FOR_BLACK and std_dev < MIN_STD_DEV_FOR_BW:
            color_name = "Black"
        elif std_dev< MIN_STD_DEV_FOR_GREY:
            color_name = "Grey"
        else:

        ### Step 3 - transform a colour in light, normal and dark (ie. Light Red, Red, Dark Red)
            
            if color_name != "None":
                if avg_rgb_values < color_thresholds[color_name][0]:
                    color_name = "Dark "+color_name
                elif avg_rgb_values > color_thresholds[color_name][1]:
                    color_name = "Light "+color_name
                    
        # removes the 2 from Red2 color, basically it is red but is on the other side of the spectrum
        color_name = color_name.replace("2", "")

        ### Step 4 - rename some colors for a better day-to-day name (i.e. Dark Orange -> Brown)

        if color_name == "Light Red":
            color_name = "Pink"
        elif color_name == "Dark Yellow":
            color_name = "Olive"
        elif color_name == "Light Yellow":
            color_name = "Beige"
        elif color_name == "Light Magenta":
            color_name = "Pink"
        elif color_name == "Dark Cyan":
            color_name = "Blue"
        elif color_name == "Dark Orange":
            color_name = "Brown"

        return color_name, hsv_info


    def get_color_of_line_between_two_points(self, image, image_draw, p1, p2):

        DEBUG_DRAW_COLOR = False

        image_h, image_w, image_c = image.shape
        # print(image.shape)

        color_name = "None"
        color_value_rgb = (0, 0, 0)
        color_value_bgr = (0, 0, 0)
        color_value_hsv = (0, 0, 0) 
        n_points = 0

        if 0 <= p1[0] < image_w and 0 <= p2[0] < image_w and 0 <= p1[1] < image_h and 0 <= p2[1] < image_h:

            # if DEBUG_DRAW_COLOR:
                # cv2.line(image_draw, p1, p2, (255, 255, 255), 1)

            if p2[0] != p1[0]:

                m = (p2[1] - p1[1])/(p2[0] - p1[0])
                b = p2[1] - m* p2[0]
                # print("m:", m)

                if -1.0 <= m <= 1.0 :

                    # print("CASE 1")

                    t__ = [0,0,0] 
                    ctr = 0
                    for x in range(abs(p1[0]-p2[0])+1):
                        n_points = abs(p1[0]-p2[0])+1
                        y = m*(min(p2[0],p1[0])+x) + b
                        ctr+=1
                        t__ += image[int(y+0.5), min(p2[0],p1[0])+x] 
                        # print(ctr, image[int(y+0.5), min(p2[0],p1[0])+x])
                        if DEBUG_DRAW_COLOR:
                            image_draw[int(y+0.5), min(p2[0],p1[0])+x] = (0, 255, 0)

                    if ctr > 0:

                        final_avg_color_bgr = (t__/ctr)+0.5
                        final_avg_color_int_bgr = final_avg_color_bgr.astype(int)
                        # print(final_avg_color_bgr)
                        # print(final_avg_color_int_bgr)

                        # convert from BGR to RGB
                        final_avg_color_int_rgb = (final_avg_color_int_bgr[2], final_avg_color_int_bgr[1], final_avg_color_int_bgr[0])

                        # calls the rgb_to_string function
                        color_name, color_value_hsv = self.rgb_to_string_tr(final_avg_color_int_rgb)
                        color_value_rgb = final_avg_color_int_rgb
                        color_value_bgr = final_avg_color_int_bgr

                    else:
                        color_name = "None"
                        color_value_rgb = (0, 0, 0)
                        color_value_bgr = (0, 0, 0)

                else: # by turning the axis system, what happens is that i can have more than one point in each yy coordinates od the line

                    m = (p2[0] - p1[0])/(p2[1] - p1[1])
                    b = p2[0] - m* p2[1]

                    # print("CASE 2")

                    t__ = [0,0,0] 
                    ctr = 0

                    for x in range(abs(p1[1]-p2[1])+1):
                        n_points = abs(p1[1]-p2[1])+1
                        y = m*(min(p2[1], p1[1])+x) + b
                        ctr+=1
                        t__ += image[min(p2[1], p1[1])+x, int(y)] 
                        # print(ctr, image[min(p2[1], p1[1])+x, int(y)])
                        if DEBUG_DRAW_COLOR:
                            image_draw[min(p2[1],p1[1])+x, int(y)] = (0, 255, 0)

                    if ctr > 0:

                        final_avg_color_bgr = (t__/ctr)+0.5
                        final_avg_color_int_bgr = final_avg_color_bgr.astype(int)
                        # print(final_avg_color_bgr)
                        # print(final_avg_color_int_bgr)

                        # convert from BGR to RGB
                        final_avg_color_int_rgb = (final_avg_color_int_bgr[2], final_avg_color_int_bgr[1], final_avg_color_int_bgr[0])

                        # calls the rgb_to_string function
                        color_name, color_value_hsv = self.rgb_to_string_tr(final_avg_color_int_rgb)
                        color_value_rgb = final_avg_color_int_rgb
                        color_value_bgr = final_avg_color_int_bgr


                    else:
                        color_name = "None"
                        color_value_rgb = (0, 0, 0)
                        color_value_bgr = (0, 0, 0)

            elif p2[1] != p1[1]: # division by zero error when calculating m 
                
                m = (p2[0] - p1[0])/(p2[1] - p1[1])
                b = p2[0] - m* p2[1]

                # print("CASE 3")
                
                t__ = [0,0,0] 
                ctr = 0

                for x in range(abs(p1[1]-p2[1])+1):
                    n_points = abs(p1[1]-p2[1])+1
                    y = m*(min(p2[1], p1[1])+x) + b
                    ctr+=1
                    t__ += image[min(p2[1],p1[1])+x, int(y+0.5)] 
                    # print(ctr, image[min(p2[1], p1[1])+x, int(y+0.5)])
                    if DEBUG_DRAW_COLOR:
                        image_draw[min(p2[1],p1[1])+x, int(y+0.5)] = (0, 255, 0)

                if ctr > 0:

                    final_avg_color_bgr = (t__/ctr)+0.5
                    final_avg_color_int_bgr = final_avg_color_bgr.astype(int)
                    # print(final_avg_color_bgr)
                    # print(final_avg_color_int_bgr)

                    # convert from BGR to RGB
                    final_avg_color_int_rgb = (final_avg_color_int_bgr[2], final_avg_color_int_bgr[1], final_avg_color_int_bgr[0])

                    # calls the rgb_to_string function
                    color_name, color_value_hsv = self.rgb_to_string_tr(final_avg_color_int_rgb)
                    color_value_rgb = final_avg_color_int_rgb
                    color_value_bgr = final_avg_color_int_bgr

                else:
                    color_name = "None"
                    color_value_rgb = (0, 0, 0)
                    color_value_bgr = (0, 0, 0)

            if DEBUG_DRAW_COLOR:
                image_draw[p2[1], p2[0]] = (0, 0, 255)
                image_draw[p1[1], p1[0]] = (0, 0, 255)

        # print("Color:", color_name, "[ RGB:", color_value_rgb, "HSV:", color_value_hsv,"N_P:", n_points, "]")

        # return color_name, color_value_rgb, color_value_bgr, color_value_hsv, n_points
        return color_name, color_value_rgb, n_points


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()