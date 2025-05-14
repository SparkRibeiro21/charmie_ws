#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, PointStamped
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedPerson, BoundingBox, BoundingBoxAndPoints, RGB, ListOfDetectedPerson
from charmie_interfaces.srv import ActivateYoloPose
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import cv2
import numpy as np
import time
import math
import json
from keras.models import load_model
import threading
from pathlib import Path

from charmie_point_cloud.point_cloud_class import PointCloud

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
DRAW_FACE_RECOGNITION = False

data_lock = threading.Lock()

# Just to check if everything is OK with CUDA
# import torch
# print("CUDA available:", torch.cuda.is_available())
# print("Device count:", torch.cuda.device_count())
# print("Device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No GPU")

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
            with open(self.complete_path_configuration_files + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)
            with open(self.complete_path_configuration_files + 'furniture.json', encoding='utf-8') as json_file:
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

        yolo_models_sucessful_imported = False

        while not yolo_models_sucessful_imported:
            
            try: 
                self.model = YOLO(full_yolo_model)
                self.get_logger().info("Successfully imported YOLO pose model.")
                yolo_models_sucessful_imported = True

            except:
                self.get_logger().error("Could NOT import YOLO pose model.")
                time.sleep(1.0)

        ### Topics ###
        # Intel Realsense Subscribers (RGBD) Head Camera
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        # Publishe Results
        self.person_pose_filtered_publisher = self.create_publisher(ListOfDetectedPerson, "person_pose_filtered", 10)

        ### Services ###
        # This service is initialized on a function that is explained in comments on that timer function
        # self.activate_yolo_pose_service = self.create_service(ActivateYoloPose, "activate_yolo_pose", self.callback_activate_yolo_pose)
        
        ### TF buffer and listener ###
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ### Class ###
        self.point_cloud = PointCloud()

        ### Variables ###        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.head_depth = Image()
        self.new_head_rgb = False
        self.new_head_depth = False

        self.CAM_IMAGE_WIDTH = 848
        self.CAM_IMAGE_HEIGHT = 480

        self.head_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH, 3), np.uint8)
        self.head_depth_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH), np.uint8)

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
        self.activate_yolo_pose_service = self.create_service(ActivateYoloPose, "activate_yolo_pose", self.callback_activate_yolo_pose)

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
    
    def get_rgbd_head_callback(self, rgbd: RGBD):
        with data_lock: 
            self.head_rgb = rgbd.rgb
            self.head_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.head_depth = rgbd.depth
            self.head_depth_cv2_frame = self.br.imgmsg_to_cv2(rgbd.depth, "passthrough")
        self.new_head_rgb = True
        self.new_head_depth = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    # def add_person_to_detectedperson_msg(self, current_frame, current_frame_draw, boxes_id, keypoints_id, center_person_filtered, center_torso_person, center_head_person, torso_localisation, head_localisation, arm_raised):
    def add_person_to_detectedperson_msg(self, boxes_id, keypoints_id, face_bounding_box, is_face_bounding_box, arm_raised, object_coords_to_cam, object_coords_to_base, object_coords_to_map, center_torso_person, center_head_person, torso_localisation, head_localisation, \
                                        ethnicity, ethnicity_probability, age_estimate, age_estimate_probability, gender, gender_probability, shirt_color, shirt_rgb, pants_color, pants_rgb, pointing_at, pointing_with_arm, \
                                        camera, current_img):
        # receives the box and keypoints of a specidic person and returns the detected person 
        # it can be done in a way that is only made once per person and both 'person_pose' and 'person_pose_filtered'

        global GET_CHARACTERISTICS

        person_id = boxes_id.id
        if boxes_id.id == None:
            person_id = 0 

        new_person = DetectedPerson()

        new_person.image_rgb_frame = self.head_rgb

        new_person.index = int(person_id)
        new_person.confidence = float(boxes_id.conf)
        new_person.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_person.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_person.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_person.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        new_person.is_box_head = is_face_bounding_box
        new_person.box_head_top_left_x = int(face_bounding_box[0])
        new_person.box_head_top_left_y = int(face_bounding_box[1])
        new_person.box_head_width = int(face_bounding_box[2]) - int(face_bounding_box[0])
        new_person.box_head_height = int(face_bounding_box[3]) - int(face_bounding_box[1])

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
        
        # print(object_coords_to_cam)
        new_person.position_cam = object_coords_to_cam
        # print(object_coords_to_base)
        # new_person.position_relative = object_coords_to_base
        new_person.position_relative = object_coords_to_cam
        # print(object_coords_to_map)
        # new_person.position_absolute = object_coords_to_map
        new_person.position_absolute = object_coords_to_cam

        

        """
        # changes the axis of point cloud coordinates to fit with robot axis
        person_rel_pos = Point()
        person_rel_pos.x =  center_person_filtered.x/1000
        person_rel_pos.y =  center_person_filtered.y/1000
        person_rel_pos.z =  center_person_filtered.z/1000
        
        new_person.position_relative = person_rel_pos
        
        # calculate the absolute position according to the robot localisation
        angle_person = math.atan2(person_rel_pos.x, person_rel_pos.y)
        dist_person = math.sqrt(person_rel_pos.x**2 + person_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_person - self.robot_pose.theta)

        target_x = dist_person * math.cos(theta_aux) + self.robot_pose.x
        target_y = dist_person * math.sin(theta_aux) + self.robot_pose.y

        a_ref = (target_x, target_y)
        # print("Rel:", (person_rel_pos.x, person_rel_pos.y), "Abs:", a_ref)

        person_abs_pos = Point()
        person_abs_pos.x = target_x
        person_abs_pos.y = target_y
        person_abs_pos.z = center_person_filtered.z/1000
        
        new_person.position_absolute = person_abs_pos

        # changes the axis of point cloud coordinates to fit with robot axis
        head_rel_pos = Point()
        head_rel_pos.x =  head_localisation.x/1000
        head_rel_pos.y =  head_localisation.y/1000
        head_rel_pos.z =  head_localisation.z/1000
        new_person.position_relative_head = head_rel_pos
        
        # calculate the absolute head position according to the robot localisation
        angle_head = math.atan2(head_rel_pos.x, head_rel_pos.y)
        dist_head = math.sqrt(head_rel_pos.x**2 + head_rel_pos.y**2)

        theta_aux = math.pi/2 - (angle_head - self.robot_pose.theta)

        target_x = dist_head * math.cos(theta_aux) + self.robot_pose.x
        target_y = dist_head * math.sin(theta_aux) + self.robot_pose.y

        a_ref = (target_x, target_y)
        # print("Rel:", (head_rel_pos.x, head_rel_pos.y), "Abs:", a_ref)

        head_abs_pos = Point()
        head_abs_pos.x = target_x
        head_abs_pos.y = target_y
        head_abs_pos.z = head_localisation.z/1000
        
        new_person.position_absolute_head = head_abs_pos
        




        new_person.height = head_localisation.z/1000 + 0.08 # average person middle of face to top of head distance
        new_person.room_location, new_person.furniture_location = self.position_to_house_rooms_and_furniture(person_abs_pos)
        """

        new_person.pointing_at = pointing_at
        new_person.pointing_with_arm = pointing_with_arm

        new_person.shirt_color = shirt_color
        new_person.shirt_rgb = shirt_rgb 
        new_person.pants_color = pants_color
        new_person.pants_rgb = pants_rgb

        new_person.ethnicity = ethnicity 
        new_person.ethnicity_probability = ethnicity_probability
        new_person.age_estimate = age_estimate
        new_person.age_estimate_probability = age_estimate_probability
        new_person.gender = gender
        new_person.gender_probability = gender_probability

        return new_person

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
    node = YoloPoseNode()
    th_main = threading.Thread(target=ThreadMainYoloPose, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainYoloPose(node: YoloPoseNode):
    main = YoloPoseMain(node)
    main.main()


class YoloPoseMain():

    def __init__(self, node: YoloPoseNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

        self.new_head_frame_time = time.time()
        self.prev_head_frame_time = time.time()

    def get_transform(self, camera=""):

        match camera:
            case "head":
                child_link = 'D455_head_link'
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

    def detect_with_yolo_model(self, head_frame, head_depth_frame, head_image):

        yolov8_person_filtered = ListOfDetectedPerson()
        person_results_list = []
        num_people = 0

        models_dict = {
            "head_pose": -1,
            "hand_pose": -1,
            "base_pose": -1
            }

        # self.get_logger().info('Receiving color video frame head')
        tempo_total = time.perf_counter()

        map_transform, _ = self.get_transform() # base_footprint -> map
        
        if self.node.ACTIVATE_YOLO_POSE:
            people_results = self.node.model.track(head_frame, persist=True, tracker="bytetrack.yaml", verbose=False)
            person_results_list.append(people_results)
            models_dict["head_pose"] = len(person_results_list) - 1
            num_people += len(people_results[0])
            if num_people > 0:
                transform_head, head_link = self.get_transform("head")

            if self.node.DEBUG_DRAW:
                cv2.imshow("HEAD POSE DEBUG", people_results[0].plot())
            
        
        if self.node.DEBUG_DRAW:
            cv2.waitKey(1)

        print("TRACK TIME:", time.perf_counter()-tempo_total)

        reverse_models_dict = {v: k for k, v in models_dict.items() if v != -1}

        # print(num_people)
        for idx, p_res in enumerate(person_results_list):

            if p_res[0].boxes.id is not None:

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
                
                if map_transform is None:
                    print("MAP TF: OFF!", end='')
                else:
                    print("MAP TF:  ON!", end='')
                if transform is None:
                    print("\tROBOT TF: OFF!")
                else:
                    print("\tROBOT TF:  ON!")

                # specific model settings
                MIN_CONF_NALUE = MIN_PERSON_CONF_VALUE                        
                boxes = p_res[0].boxes
                keypoints = p_res[0].keypoints

                for box, keypoint in zip(boxes, keypoints):

                    # print(keypoint)
                    # print(keypoint.conf[0][self.node.NOSE_KP])

                    person_center_x, person_center_y, head_center_x, head_center_y = self.get_person_and_head_pixel(keypoint=keypoint, box=box)
                    hand_raised, is_hand_raised = self.check_arm_raised(keypoint=keypoint)
                    legs_ctr, body_kp_high_conf_counter = self.keypoint_counter(keypoint=keypoint)

                    ########### MISSING HERE: POINT CLOUD CALCULATIONS ##########
                    obj_3d_cam_coords = self.node.point_cloud.convert_bbox_to_3d_point(depth_img=depth_frame, camera=camera, bbox=box)
                    print("3D Coords", obj_3d_cam_coords.x, obj_3d_cam_coords.y, obj_3d_cam_coords.z)
                    
                    torso_center = []
                    torso_center.append(person_center_y)
                    torso_center.append(person_center_x)
                    head_center = []
                    head_center.append(head_center_y)
                    head_center.append(head_center_x)
                    
                    # obj_3d_cam_coords = self.node.point_cloud.convert_pixel_to_3dpoint(depth_img=depth_frame, camera=camera, pixel=torso_center)
                    # obj_3d_cam_coords = self.node.point_cloud.convert_pixel_to_3dpoint(depth_img=depth_frame, camera=camera, pixel=head_center)
                    # print("3D Coords", obj_3d_cam_coords.x, obj_3d_cam_coords.y, obj_3d_cam_coords.z)
                    # obj_3d_cam_coords.x += 0.25
                    
                    
                    # obj_3d_cam_coords = Point()
                    # obj_3d_cam_coords.x = 1.0
                    # obj_3d_cam_coords.y = 0.0
                    # obj_3d_cam_coords.z = 0.0
                    
                    ALL_CONDITIONS_MET = 1

                    ########## MISSING HERE: CASE WHERE NO POINTS WERE AVALILABLE SO WE DONT KNOW HOW TO COMPUTE 3D ##########
                    # no mask depth points were available, so it was not possible to calculate x,y,z coordiantes
                    if obj_3d_cam_coords.x == 0 and obj_3d_cam_coords.y == 0 and obj_3d_cam_coords.z == 0:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        print ("REMOVED")

                    # checks whether the person confidence is above a selected level
                    if not box.conf >= MIN_CONF_NALUE:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        # print("- Misses minimum confidence level")

                    # checks if flag to detect people whose legs are visible 
                    if not legs_ctr >= NUMBER_OF_LEG_KP_TO_BE_DETECTED and ONLY_DETECT_PERSON_LEGS_VISIBLE:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        # print("- Misses legs visible flag")

                    # checks whether the minimum number if body keypoints (excluding legs) has high confidence
                    if not body_kp_high_conf_counter >= MIN_KP_TO_DETECT_PERSON:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        # print("- Misses minimum number of body keypoints")
                    
                    # checks if flag to detect people with their arm raised or waving (requesting assistance)
                    if not is_hand_raised and ONLY_DETECT_PERSON_ARM_RAISED:
                        ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                        # print(" - Misses not being with their arm raised")

                    # if the person detection passes all selected conditions, the detected person is added to the publishing list
                    if ALL_CONDITIONS_MET:

                        ########### MISSING HERE: APPLY LOCAL AND GLOBAL TRANSFORMS ########### Suppose each detection has x, y, z coordinates in the camera frame
                        point_cam = PointStamped()
                        point_cam.header.stamp = self.node.get_clock().now().to_msg()
                        point_cam.header.frame_id = camera_link
                        point_cam.point = obj_3d_cam_coords

                        transformed_point = PointStamped()
                        transformed_point_map = PointStamped()
                        if transform is not None:
                            transformed_point = do_transform_point(point_cam, transform)
                            self.node.get_logger().info(f"Person in base_footprint frame: {transformed_point.point}")

                            if map_transform is not None:
                                transformed_point_map = do_transform_point(transformed_point, map_transform)
                                self.node.get_logger().info(f"Person in map frame: {transformed_point_map.point}")

                        center_comm_position = False
                        if -ONLY_DETECT_PERSON_RIGHT_IN_FRONT_X_THRESHOLD < transformed_point.point.x < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_X_THRESHOLD and \
                            -ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD < transformed_point.point.y < ONLY_DETECT_PERSON_RIGHT_IN_FRONT_Y_THRESHOLD:
                            center_comm_position = True
                    
                        # checks if flag to detect people right in front of the robot 
                        if not center_comm_position and ONLY_DETECT_PERSON_RIGHT_IN_FRONT:
                            ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                            # print(" - Misses not being right in front of the robot")

                        if ALL_CONDITIONS_MET: # special case just for the center_comm_position

                            # These two characteristics are outside the characteristics if clause, because there are almost instantaneous and don't need a model to calculate.
                            # Therefore it was decided that this parameter is always calculated for every detected person
                            shirt_color, shirt_rgb = self.get_shirt_color(keypoint, head_frame, head_frame) 
                            pants_color, pants_rgb = self.get_pants_color(keypoint, head_frame, head_frame)
                            pointing_at, pointing_with_arm = self.arm_pointing_at(keypoint)
                            is_cropped_face, cropped_face, face_bounding_box = self.crop_face(head_frame, head_frame, keypoint=keypoint, box=box)

                            # characteristics will only be updated after we confirm that the person is inside the filteredpersons
                            # otherwise the large amount of time spent getting the characteristics from the models is applied to
                            # every detected person and not only the filtered 
                            ethnicity = "None"
                            ethnicity_probability = 0.0
                            age_estimate = "None"
                            age_estimate_probability = 0.0
                            gender = "None"
                            gender_probability = 0.0 
                            if GET_CHARACTERISTICS:
                                # in order to predict the ethnicity, age and gender, it is necessary to first cut out the face of the detected person

                                if is_cropped_face:
                                    ethnicity, ethnicity_probability = self.get_ethnicity_prediction(cropped_face) # says whether the person white, asian, african descendent, middle eastern, ...
                                    age_estimate, age_estimate_probability = self.get_age_prediction(cropped_face) # says an approximate age gap like 25-35 ...
                                    gender, gender_probability = self.get_gender_prediction(cropped_face) # says whether the person is male or female
                                    print(ethnicity, age_estimate, gender)        
                            
                            new_person = DetectedPerson()
                            new_person = self.node.add_person_to_detectedperson_msg(boxes_id=box, keypoints_id=keypoint, face_bounding_box=face_bounding_box, is_face_bounding_box=is_cropped_face, arm_raised=hand_raised, \
                                                                                    object_coords_to_cam=point_cam.point, object_coords_to_base=transformed_point.point, object_coords_to_map=transformed_point_map.point, \
                                                                                    center_torso_person=(person_center_x, person_center_y), center_head_person=(head_center_x, head_center_y), \
                                                                                    torso_localisation=None, head_localisation=None, \
                                                                                    ethnicity=ethnicity, ethnicity_probability=ethnicity_probability, age_estimate=age_estimate, age_estimate_probability=age_estimate_probability, gender=gender, gender_probability=gender_probability, \
                                                                                    shirt_color=shirt_color, shirt_rgb=shirt_rgb, pants_color=pants_color, pants_rgb=pants_rgb, pointing_at=pointing_at, pointing_with_arm=pointing_with_arm, \
                                                                                    camera=camera, current_img=rgb_img)
                            yolov8_person_filtered.persons.append(new_person)

        # self.node.get_logger().info(f"People detected: {len(yolov8_person_filtered.persions)}/{num_people}")
        # self.node.get_logger().info(f"Time Yolo_People: {time.perf_counter() - tempo_total}")

        return yolov8_person_filtered, num_people

    def check_arm_raised(self, keypoint):
        # condition whether the person has their arm raised, or waiving
        # at the current time, we are using the wrist coordinates and somparing with the nose coordinate
        is_hand_raised = False
        hand_raised = "None"
        if int(keypoint.xy[0][self.node.NOSE_KP][1]) >= int(keypoint.xy[0][self.node.WRIST_RIGHT_KP][1]) and \
            int(keypoint.xy[0][self.node.NOSE_KP][1]) >= int(keypoint.xy[0][self.node.WRIST_LEFT_KP][1]) and \
            keypoint.conf[0][self.node.NOSE_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.WRIST_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.WRIST_LEFT_KP] > MIN_KP_CONF_VALUE:
                
            # print("Both Arms Up")
            hand_raised = "Both"
            is_hand_raised = True
        else:
            if int(keypoint.xy[0][self.node.NOSE_KP][1]) >= int(keypoint.xy[0][self.node.WRIST_RIGHT_KP][1]) and \
                keypoint.conf[0][self.node.NOSE_KP] > MIN_KP_CONF_VALUE and \
                keypoint.conf[0][self.node.WRIST_RIGHT_KP] > MIN_KP_CONF_VALUE:
                # print("Right Arm Up")
                hand_raised = "Right"
                is_hand_raised = True
            elif int(keypoint.xy[0][self.node.NOSE_KP][1]) >= int(keypoint.xy[0][self.node.WRIST_LEFT_KP][1]) and \
                keypoint.conf[0][self.node.NOSE_KP] > MIN_KP_CONF_VALUE and \
                keypoint.conf[0][self.node.WRIST_LEFT_KP] > MIN_KP_CONF_VALUE:
                # print("Left Arm Up")
                hand_raised = "Left"
                is_hand_raised = True
            else: 
                # print("Both Arms Down")
                hand_raised = "None"
                is_hand_raised = False

        # print("Hand Raised:", hand_raised, is_hand_raised)
        return hand_raised, is_hand_raised
        
    def arm_pointing_at(self, keypoint):

        MIN_ANGLE_POINTING = 25

        right_shoulder = (int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0]), int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1]))
        right_wrist =    (int(keypoint.xy[0][self.node.WRIST_RIGHT_KP][0]),    int(keypoint.xy[0][self.node.WRIST_RIGHT_KP][1]))
        right_hip =      (int(keypoint.xy[0][self.node.HIP_RIGHT_KP][0]),      int(keypoint.xy[0][self.node.HIP_RIGHT_KP][1]))

        left_shoulder =  (int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0]),  int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1]))
        left_wrist =     (int(keypoint.xy[0][self.node.WRIST_LEFT_KP][0]),     int(keypoint.xy[0][self.node.WRIST_LEFT_KP][1]))
        left_hip =       (int(keypoint.xy[0][self.node.HIP_LEFT_KP][0]),       int(keypoint.xy[0][self.node.HIP_LEFT_KP][1]))

        # The sides are relative to the person, so the right side is linked with the person right arm!
        if keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.WRIST_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE:
            theta_right = self.calculate_3angle(right_shoulder, right_wrist, right_hip)
        else:
            theta_right = 0.0

        # The sides are relative to the person, so the right side is linked with the person right arm!
        if keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.WRIST_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.HIP_LEFT_KP] > MIN_KP_CONF_VALUE:
            theta_left = self.calculate_3angle(left_shoulder, left_wrist, left_hip)
        else:
            theta_left = 0.0
        
        side_pointed = "None"
        arm_pointed_with = "None"

        # print("Sides0:", left_wrist[0], left_hip[0], right_wrist[0], right_hip[0])    
        # print("Sides1:", left_wrist[1], left_hip[1], right_wrist[1], right_hip[1])      

        if theta_left > MIN_ANGLE_POINTING and theta_left>=theta_right: # added condition: now left side does not always get priority, now priority is for pointing side wither bigger angle
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
        
    def get_person_and_head_pixel(self, keypoint, box):

        """
        # gets the torso u,v location
        # Conditions to safely select the pixel to calculate the person location 
        if keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.HIP_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE:
        
            ### After the yolo update, i must check if the conf value of the keypoint
            person_center_x = int((keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0] + keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0] + keypoint.xy[0][self.node.HIP_LEFT_KP][0] + keypoint.xy[0][self.node.HIP_RIGHT_KP][0]) / 4)
            person_center_y = int((keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1] + keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1] + keypoint.xy[0][self.node.HIP_LEFT_KP][1] + keypoint.xy[0][self.node.HIP_RIGHT_KP][1]) / 4)

        elif keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE:

            ### After the yolo update, i must check if the conf value of the keypoint
            person_center_x = int((keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0] + keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0]) / 2)
            person_center_y = int((keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1] + keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1]) / 2)

        elif keypoint.conf[0][self.node.HIP_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE:

            ### After the yolo update, i must check if the conf value of the keypoint
            person_center_x = int((keypoint.xy[0][self.node.HIP_LEFT_KP][0] + keypoint.xy[0][self.node.HIP_RIGHT_KP][0]) / 2)
            person_center_y = int((keypoint.xy[0][self.node.HIP_LEFT_KP][1] + keypoint.xy[0][self.node.HIP_RIGHT_KP][1]) / 2)

        else:
            person_center_x = int(box.xyxy[0][0]+box.xyxy[0][2])//2
            person_center_y = int(box.xyxy[0][1]+box.xyxy[0][3])//2
        """

        # head center position 
        head_ctr = 0
        head_center_x = 0
        head_center_y = 0
        if keypoint.conf[0][self.node.NOSE_KP] > MIN_KP_CONF_VALUE:
            head_center_x += int(keypoint.xy[0][self.node.NOSE_KP][0])
            head_center_y += int(keypoint.xy[0][self.node.NOSE_KP][1])
            head_ctr +=1
        if keypoint.conf[0][self.node.EYE_LEFT_KP] > MIN_KP_CONF_VALUE:
            head_center_x += int(keypoint.xy[0][self.node.EYE_LEFT_KP][0])
            head_center_y += int(keypoint.xy[0][self.node.EYE_LEFT_KP][1])
            head_ctr +=1
        if keypoint.conf[0][self.node.EYE_RIGHT_KP] > MIN_KP_CONF_VALUE:
            head_center_x += int(keypoint.xy[0][self.node.EYE_RIGHT_KP][0])
            head_center_y += int(keypoint.xy[0][self.node.EYE_RIGHT_KP][1])
            head_ctr +=1
        if keypoint.conf[0][self.node.EAR_LEFT_KP] > MIN_KP_CONF_VALUE:
            head_center_x += int(keypoint.xy[0][self.node.EAR_LEFT_KP][0])
            head_center_y += int(keypoint.xy[0][self.node.EAR_LEFT_KP][1])
            head_ctr +=1
        if keypoint.conf[0][self.node.EAR_RIGHT_KP] > MIN_KP_CONF_VALUE:
            head_center_x += int(keypoint.xy[0][self.node.EAR_RIGHT_KP][0])
            head_center_y += int(keypoint.xy[0][self.node.EAR_RIGHT_KP][1])
            head_ctr +=1

        if head_ctr > 0:
            head_center_x = int(head_center_x/head_ctr)
            head_center_y = int(head_center_y/head_ctr)
        else:
            head_center_x = int(box.xyxy[0][0]+box.xyxy[0][2])//2
            head_center_y = int(box.xyxy[0][1]*3+box.xyxy[0][3])//4

        # torso center position 
        torso_ctr = 0
        torso_center_x = 0
        torso_center_y = 0
        if keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE:
            torso_center_x += int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0])
            torso_center_y += int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1])
            torso_ctr +=1
        if keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE:
            torso_center_x += int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0])
            torso_center_y += int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1])
            torso_ctr +=1
        if keypoint.conf[0][self.node.HIP_LEFT_KP] > MIN_KP_CONF_VALUE:
            torso_center_x += int(keypoint.xy[0][self.node.HIP_LEFT_KP][0])
            torso_center_y += int(keypoint.xy[0][self.node.HIP_LEFT_KP][1])
            torso_ctr +=1
        if keypoint.conf[0][self.node.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE:
            torso_center_x += int(keypoint.xy[0][self.node.HIP_RIGHT_KP][0])
            torso_center_y += int(keypoint.xy[0][self.node.HIP_RIGHT_KP][1])
            torso_ctr +=1

        if torso_ctr > 0:
            torso_center_x = int(torso_center_x/torso_ctr)
            torso_center_y = int(torso_center_y/torso_ctr)
        else:
            # this way there is an attemp of always having a keypoint as the torso, much more stable than center of bounding box
            torso_center_x = head_center_x
            torso_center_y = head_center_y

        return torso_center_x, torso_center_y, head_center_x, head_center_y
        
    def keypoint_counter(self, keypoint):
    
        legs_ctr = 0
        if keypoint.conf[0][self.node.KNEE_LEFT_KP] > MIN_KP_CONF_VALUE:
            legs_ctr+=1
        if keypoint.conf[0][self.node.KNEE_RIGHT_KP] > MIN_KP_CONF_VALUE:
            legs_ctr+=1
        if keypoint.conf[0][self.node.ANKLE_LEFT_KP] > MIN_KP_CONF_VALUE:
            legs_ctr+=1
        if keypoint.conf[0][self.node.ANKLE_RIGHT_KP] > MIN_KP_CONF_VALUE:
            legs_ctr+=1

        body_kp_high_conf_counter = 0
        for kp in range(self.node.N_KEYPOINTS - self.node.NUMBER_OF_LEGS_KP): # all keypoints without the legs
            if keypoint.conf[0][kp] > MIN_KP_CONF_VALUE:
                body_kp_high_conf_counter+=1

        return legs_ctr, body_kp_high_conf_counter

    # THE DRAW FUCNTION WAS NOT UPDATED WITH YOLO_POSE April 2025 UPDATE, SINCE NOW ALL DRAWINGS ARE DONE IN GUI. IN THE FUTURE SHOULD BE DELETED.
    def draw_detected_people(self, yolov8_people, current_frame_draw):


        if self.node.DEBUG_DRAW:
            
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
                

        if self.DEBUG_DRAW:
            # putting the FPS count on the frame
            cv2.putText(current_frame_draw, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame_draw, 'np:' + str(len(yolov8_pose_filtered.persons)) + '/' + str(num_persons), (180, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
            # cv2.imshow("Yolo Pose Detection", annotated_frame)
            # cv2.imshow("Camera Image", current_frame)
            cv2.waitKey(1)

    def crop_face(self, current_frame, current_frame_draw, keypoint, box):

        global DRAW_FACE_RECOGNITION

        # y1 = top of bounding box y
        # y2 = y of lowest height shoulder
        # x1 = keypoint more to the left
        # x2 = keypoint more to the right
        
        # using all face and shoulders keypoints to make sure face is correctly detected
        if keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.EYE_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.EYE_LEFT_KP] > MIN_KP_CONF_VALUE and \
            keypoint.conf[0][self.node.NOSE_KP] > MIN_KP_CONF_VALUE:
            # keypoint.conf[0][self.node.EAR_RIGHT_KP] > MIN_KP_CONF_VALUE and \
            # keypoint.conf[0][self.node.EAR_LEFT_KP] > MIN_KP_CONF_VALUE and \
            
            y1 = int(box.xyxy[0][1])
            y2 = max(int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1]), int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1]))

            x1 = min(int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0]), int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0]), int(keypoint.xy[0][self.node.NOSE_KP][0]), int(keypoint.xy[0][self.node.EYE_RIGHT_KP][0]), int(keypoint.xy[0][self.node.EYE_LEFT_KP][0]))
            x2 = max(int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0]), int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0]), int(keypoint.xy[0][self.node.NOSE_KP][0]), int(keypoint.xy[0][self.node.EYE_RIGHT_KP][0]), int(keypoint.xy[0][self.node.EYE_LEFT_KP][0]))

            if DRAW_FACE_RECOGNITION:
                cv2.rectangle(current_frame_draw, (x1, y1), (x2, y2), (0, 255, 255) , 4) 
                
            # cv2.imwrite("cropped_face_test.jpg", current_frame[y1:y2, x1:x2])
            
            return True, current_frame[y1:y2, x1:x2], [x1, y1, x2, y2]

        else:
            return False, current_frame, [0, 0, 0, 0]
        
    def get_age_prediction(self, cropped_face):
        
        blob = cv2.dnn.blobFromImage(cropped_face, 1.0, (227, 227), (78.4263377603, 87.7689143744, 114.895847746), swapRB=False)
        self.node.ageNet.setInput(blob)
        agePreds = self.node.ageNet.forward()

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
        predictions = self.node.gender_model.predict(expanded_img)

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
        predictions = self.node.race_model.predict(expanded_img)

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
    
    def get_shirt_color(self, keypoint, current_frame, current_frame_draw):
        color_name = "None"
        color_rgb = RGB()

        if keypoint.conf[0][self.node.SHOULDER_LEFT_KP] > MIN_KP_CONF_VALUE and keypoint.conf[0][self.node.SHOULDER_RIGHT_KP] > MIN_KP_CONF_VALUE:
            color_name, color_value_rgb, n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][0]), int(keypoint.xy[0][self.node.SHOULDER_LEFT_KP][1])), (int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][0]), int(keypoint.xy[0][self.node.SHOULDER_RIGHT_KP][1])))
    
            color_rgb.red = int(color_value_rgb[0])
            color_rgb.green = int(color_value_rgb[1])
            color_rgb.blue = int(color_value_rgb[2])

        return color_name, color_rgb

    def get_pants_color(self, keypoint, current_frame, current_frame_draw):
        left_leg_color_name = "None"
        left_leg_color_rgb = RGB()
        left_leg_n_points = 0
        right_leg_color_name = "None"
        right_leg_color_rgb = RGB()
        right_leg_n_points = 0
        color_name = "None"
        color_rgb = RGB()

        if keypoint.conf[0][self.node.HIP_LEFT_KP] > MIN_KP_CONF_VALUE and keypoint.conf[0][self.node.KNEE_LEFT_KP] > MIN_KP_CONF_VALUE:
            left_leg_color_name, left_leg_color_rgb, left_leg_n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (int(keypoint.xy[0][self.node.HIP_LEFT_KP][0]), int(keypoint.xy[0][self.node.HIP_LEFT_KP][1])), (int(keypoint.xy[0][self.node.KNEE_LEFT_KP][0]), int(keypoint.xy[0][self.node.KNEE_LEFT_KP][1])))

        if keypoint.conf[0][self.node.HIP_RIGHT_KP] > MIN_KP_CONF_VALUE and keypoint.conf[0][self.node.KNEE_RIGHT_KP] > MIN_KP_CONF_VALUE:
            right_leg_color_name, right_leg_color_rgb, right_leg_n_points = self.get_color_of_line_between_two_points(current_frame, current_frame_draw, (int(keypoint.xy[0][self.node.HIP_RIGHT_KP][0]), int(keypoint.xy[0][self.node.HIP_RIGHT_KP][1])), (int(keypoint.xy[0][self.node.KNEE_RIGHT_KP][0]), int(keypoint.xy[0][self.node.KNEE_RIGHT_KP][1])))

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

    # main state-machine function
    def main(self):
        
        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In YoloPose Main...")
        time_till_done = time.time()
        
        while True:

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

            if not self.node.yolo_models_initialized:

                self.node.new_head_rgb = True
                self.node.ACTIVATE_YOLO_POSE = True

            if self.node.new_head_rgb:

                time_till_done = time.time()
                
                with data_lock: 
                    head_image_frame = self.node.head_rgb_cv2_frame.copy()
                    head_depth_frame = self.node.head_depth_cv2_frame.copy()
                    head_image = self.node.head_rgb

                if self.node.ACTIVATE_YOLO_POSE and self.node.new_head_rgb:

                    self.node.new_head_rgb = False

                    ### temp:
                    list_detected_people = ListOfDetectedPerson()
                    list_detected_people, total_people = self.detect_with_yolo_model(head_frame=head_image_frame, head_depth_frame=head_depth_frame, head_image=head_image)
                    if self.node.yolo_models_initialized:
                        self.node.person_pose_filtered_publisher.publish(list_detected_people)

                    print("TR Time Yolo_Pose: ", time.time() - time_till_done)

                    # if self.node.DEBUG_DRAW:
                    #     cv2.putText(current_frame_draw, 'fps:' + self.hand_fps, (0, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.putText(current_frame_draw, 'np:' + str(len(list_detected_people.persons)) + '/' + str(total_people), (180, self.node.CAM_IMAGE_HEIGHT-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                    #     cv2.imshow("Yolo Pose TR Detection HAND", current_frame_draw)
                    #     cv2.waitKey(1)

            if not self.node.yolo_models_initialized:
                
                self.node.new_head_rgb = False
                self.node.ACTIVATE_YOLO_POSE = False
                self.node.yolo_models_initialized = True

### percorrer todas as variaveis do DetectedPErson e confirmar que está tudo ok.
    
    ### localizacao torso e cabeça
    # coordenadas point cloud cabeça
    # coordenadas point cloud torso
    # coordenadas point cloud pessoad

    # room_location
    # furniture_location
    # height

### verificar flags do gui (missing Front_Close)
