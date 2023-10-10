#!/usr/bin/env python3
from ultralytics import YOLO
# from ultralytics.yolo.engine.results import Results
# from ultralytics.yolo.utils import DEFAULT_CFG, ROOT, ops
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import Keypoints, Yolov8Pose, Yolov8PoseArray
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np
# import array
# import sys
# import math
import time

DRAW_LOW_CONF_KP = False
MIN_PERSON_CONF_VALUE = 0.5





class YoloPoseNode(Node):
    def __init__(self):
        super().__init__("YoloPose")
        self.get_logger().info("Initialised YoloPose Node")

        # Yolo Model - Yolov8 Pode Nano
        self.model = YOLO('yolov8n-pose.pt')
        self.counter = 0

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.debug_draw = True

        # to calculate the FPS
        # used to record the time when we processed last frame
        self.prev_frame_time = 0
        # used to record the time at which we processed current frame
        self.new_frame_time = 0

        # Publisher
        self.yolov8_pose_publisher = self.create_publisher(Yolov8Pose, 'yolov8_pose', 10)

        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)

        # Neck Subscriber
        # self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback, 10)

        self.br = CvBridge()
        self.yolov8_pose = Yolov8Pose()
        self.yolo_array = Yolov8PoseArray()
        self.person_coordinate = Pose2D()

        self.N_KEYPOINTS = 17
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


    def get_color_image_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame')

        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        current_frame_draw = current_frame.copy()
        # Getting image dimensions
        self.img_width = img.width
        self.img_height = img.height
        # print(self.img_width)
        # print(self.img_height)

        # Launch Yolov8n-pose
        results = self.model(current_frame)

        annotated_frame = results[0].plot()

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

        self.yolov8_pose.keypoints = []
        # Calculate the number of persons detected
        num_persons = len(results[0].keypoints)
        if not results[0].keypoints.has_visible:
            num_persons = 0



        print("___START___")
        ALL_CONDITIONS_MET = 1

        CONDITION_1 = True
        # CONDITION_2 = True
        
        for person_idx in range(num_persons):
            keypoints_id = results[0].keypoints[person_idx]
            boxes_id = results[0].boxes[person_idx]
            # print(keypoints_id.xy[0][0])

            print(boxes_id.conf)

            # condition to be added to the 
            if not boxes_id.conf > MIN_PERSON_CONF_VALUE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                pass
            
            # guardar na variavel criada do yolo pose 

            
            
            # if not CONDITION_2:
            #     ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
            #     pass
            # 
            # 


            if self.debug_draw and ALL_CONDITIONS_MET:
                
                red_yp = (56, 56, 255)
                lblue_yp = (255,128,0)
                green_yp = (0,255,0)
                orange_yp = (51,153,255)
                magenta_yp = (255, 51, 255)

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


                # draws the bounding box around the person and the background for the confidence of each person
                cv2.rectangle(current_frame_draw, start_point, end_point, red_yp , 4) 
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
                print(keypoints_id.conf)        

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

                self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_LEFT_KP, self.HIP_LEFT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)
                self.line_between_two_keypoints(current_frame_draw, self.SHOULDER_RIGHT_KP, self.HIP_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)
                self.line_between_two_keypoints(current_frame_draw, self.HIP_LEFT_KP, self.HIP_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, magenta_yp)

                self.line_between_two_keypoints(current_frame_draw, self.HIP_LEFT_KP, self.KNEE_LEFT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                self.line_between_two_keypoints(current_frame_draw, self.HIP_RIGHT_KP, self.KNEE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                self.line_between_two_keypoints(current_frame_draw, self.KNEE_LEFT_KP, self.ANKLE_LEFT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)
                self.line_between_two_keypoints(current_frame_draw, self.KNEE_RIGHT_KP, self.ANKLE_RIGHT_KP, keypoints_id.xy, keypoints_id.conf, orange_yp)


                for kp in range(self.N_KEYPOINTS):
                    if keypoints_id.conf[0][kp] > 0.5:
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

        print("____END____")

        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time
        self.fps = str(self.fps)
        print("fps = " + self.fps)

        if self.debug_draw:
            # putting the FPS count on the frame
            cv2.putText(current_frame_draw, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame_draw, 'np:' + str(num_persons), (180, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Yolo Pose Detection", annotated_frame)
            # cv2.imshow("Intel RealSense Current Frame", current_frame)
            cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
            cv2.waitKey(1) 


    def get_aligned_depth_image_callback(self, img: Image):
        pass


    def line_between_two_keypoints(self, current_frame_draw, KP_ONE, KP_TWO, xy, conf, colour):

        if conf[0][KP_ONE] > 0.5 and conf[0][KP_TWO] > 0.5:    
            p1 = (int(xy[0][KP_ONE][0]), int(xy[0][KP_ONE][1]))
            p2 = (int(xy[0][KP_TWO][0]), int(xy[0][KP_TWO][1]))
            cv2.line(current_frame_draw, p1, p2, colour, 2) 
        else:
            if DRAW_LOW_CONF_KP: 
                p1 = (int(xy[0][KP_ONE][0]), int(xy[0][KP_ONE][1]))
                p2 = (int(xy[0][KP_TWO][0]), int(xy[0][KP_TWO][1]))
                cv2.line(current_frame_draw, p1, p2, (0,0,255), 2) 


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

    """
    # goes through the number of persons in the frame 
    for person_idx, per in enumerate(results[0].keypoints):
        # if person_idx == test_number:
        if num_persons > 0:
            
            
            # print(per.xy)
            print(per.xy[0][0])
            # print(per.conf)


            center_p = (int(per.xy[0][0][0]), int(per.xy[0][0][1]))


            cv2.circle(current_frame, center_p, 3, (255,255,255), -1)
            # print(f"Person index: {person_idx}") # just the index of the person

            # print("oi oi oi ")

            # print(f"Person keypoints: {per}") # all info regarding the person

            # current_frame = cv2.putText(
            #     current_frame,
            #     f"{num_persons}", # self.average_distance
            #     (self.keyp0_coordinate_x + 25*(person_idx+4), self.keyp0_coordinate_y + 25*(person_idx+4)),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 0, 255),
            #     1,
            #     cv2.LINE_AA
            # ) 


            # # Keypoints index number and x,y coordinates
            # for keypoint_idx, kpt in enumerate(per):
            # 
            #     # Display the keypoints info in the image
            #     annotated_frame = cv2.putText(
            #         annotated_frame,
            #         f"{keypoint_idx}:({int(kpt[0])}, {int(kpt[1])})",
            #         (int(kpt[0]), int(kpt[1])),
            #         cv2.FONT_HERSHEY_SIMPLEX,
            #         0.5,
            #         (255, 0, 0),
            #         1,
            #         cv2.LINE_AA
            #     ) 

    """     


    # print('Persons Nr:', num_persons)  # Print the number of persons detected
    
    # print(results[0].keypoints)
    # print(results[0].boxes)
    # print("p-_-_-p")


    # current_frame = cv2.putText(
    #     current_frame,
    #     f"{num_persons}", # self.average_distance
    #     (self.keyp0_coordinate_x + 25, self.keyp0_coordinate_y + 25),
    #     cv2.FONT_HERSHEY_DUPLEX,
    #     1,
    #     (0, 0, 255),
    #     1,
    #     cv2.LINE_AA
    # ) 



    # current_frame = cv2.putText(
    #     current_frame,
    #     f"{results[0].keypoints.has_visible}", # self.average_distance
    #     (self.keyp0_coordinate_x + 25*2, self.keyp0_coordinate_y + 25*2),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     1,
    #     (0, 0, 255),
    #     1,
    #     cv2.LINE_AA
    # ) 

    # print(results[0].probs)

    # test_number = 1


    """
    for person_idx, per in enumerate(results[0].boxes):
        print(f"Person index BOXES: {person_idx}") # just the index of the person
        print(f"Person index BOXES: {per.conf}") # just the index of the person
                
        if person_idx == 1:
            if num_persons > 0:
                # print(f"Person index BOXES: {person_idx}") # just the index of the person
                # print(per)
                # print(per.xyxy)
                # print(int(per.xyxy[0][1]))

                start_point = (int(per.xyxy[0][0]), int(per.xyxy[0][1]))
                end_point = (int(per.xyxy[0][2]), int(per.xyxy[0][3]))

                start_point_text_rect = (int(per.xyxy[0][0]-2), int(per.xyxy[0][1]))

                if int(per.xyxy[0][1]) < 30:
                    start_point_text = (int(per.xyxy[0][0]-2), int(per.xyxy[0][1]+25))
                    # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]+30)) # if '0.95'
                    end_point_text_rect = (int(per.xyxy[0][0]+50), int(per.xyxy[0][1]+30)) # if '.95'
                else:
                    start_point_text = (int(per.xyxy[0][0]-2), int(per.xyxy[0][1]-5))
                    # end_point_text_rect = (int(per.xyxy[0][0]+75), int(per.xyxy[0][1]-30)) # if '0.95'
                    end_point_text_rect = (int(per.xyxy[0][0]+50), int(per.xyxy[0][1]-30)) # if '.95'


                # Blue color in BGR 
                color = (56, 56, 255) 
                # Line thickness of 2 px 
                thickness = 4

                # Using cv2.rectangle() method 
                # Draw a rectangle with blue line borders of thickness of 2 px 
                current_frame = cv2.rectangle(current_frame, start_point, end_point, color, thickness) 
                current_frame = cv2.rectangle(current_frame, start_point_text_rect, end_point_text_rect, color, -1) 
                


                current_frame = cv2.putText(
                    current_frame,
                    # f"{round(float(per.conf),2)}", # self.average_distance
                    f"{'.'+str(int(per.conf*100))}", # self.average_distance
                    (start_point_text[0], start_point_text[1]),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                ) 
    """


"""
        # Variables Initialization 
        self.distance = 0.0
        self.distance_meters = 0.0
        self.average_distance = 0.0
        self.standard_deviation = 0.0
        self.center_y = 0
        self.center_x = 0
        self.relative_x_angle_radius = 0.0
        self.relative_y_angle_radius = 0.0
        self.relative_x = 0.0
        self.relative_y = 0.0
        self.angle = 0
        self.angle_degrees = 0
        self.i = 0
        self.keypoints_dist = []

        # Image Variables Initialization
        self.img_width = 0.0
        self.img_height = 0.0
        self.img_width_angle_degrees = 90.0  # Intel RealSense D455 Standart Value in Degrees
        self.img_height_angle_degrees = 65.0    #Intel RealSense D455 Standart Value in Degrees
        self.img_width_angle_radius = 0.0
        self.img_height_angle_radius = 0.0
        self.cv_image = []
        self.cv_image_array = []


        # COCO Keypoints YOLOv8 FRONT VIEW
        self.keyp0_coordinate_x = 0 # Nose
        self.keyp0_coordinate_y = 0 # Nose
        self.keyp1_coordinate_x = 0 # Right Eye
        self.keyp1_coordinate_y = 0 # Right Eye
        self.keyp2_coordinate_x = 0 # Left Eye
        self.keyp2_coordinate_y = 0 # Left Eye
        self.keyp3_coordinate_x = 0 # Right Ear 
        self.keyp3_coordinate_y = 0 # Right Ear
        self.keyp4_coordinate_x = 0 # Left Ear
        self.keyp4_coordinate_y = 0 # Left Ear
        self.keyp5_coordinate_x = 0 # Right Shoulder
        self.keyp5_coordinate_y = 0 # Right Shoulder
        self.keyp6_coordinate_x = 0 # Left Shoulder
        self.keyp6_coordinate_y = 0 # Left Shoulder
        self.keyp7_coordinate_x = 0 # Right Elbow
        self.keyp7_coordinate_y = 0 # Right Elbow
        self.keyp8_coordinate_x = 0 # Left Elbow
        self.keyp8_coordinate_y = 0 # Left Elbow
        self.keyp9_coordinate_x = 0 # Right Hand
        self.keyp9_coordinate_y = 0 # Right Hand
        self.keyp10_coordinate_x = 0 # Left Hand
        self.keyp10_coordinate_y = 0 # Left Hand
        self.keyp11_coordinate_x = 0 # Right Hip
        self.keyp11_coordinate_y = 0 # Right Hip
        self.keyp12_coordinate_x = 0 # Left Hip
        self.keyp12_coordinate_y = 0 # Left Hip
        self.keyp13_coordinate_x = 0 # Right Knee
        self.keyp13_coordinate_y = 0 # Right Knee
        self.keyp14_coordinate_x = 0 # Left Knee
        self.keyp14_coordinate_y = 0 # Left Knee
        self.keyp15_coordinate_x = 0 # Right Foot
        self.keyp15_coordinate_y = 0 # Right Foot
        self.keyp16_coordinate_x = 0 # Left Foot
        self.keyp16_coordinate_y = 0 # Left Foot

        self.create_timer(0.1, self.timer_callback_trans)

        self.yolo_pose_diagnostic_publisher = self.create_publisher(Bool, "yolo_pose_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.yolo_pose_diagnostic_publisher.publish(flag_diagn)

    def get_color_image_callback(self, img: Image):
        print("---")
        self.get_logger().info('Receiving color video frame')

        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")

        # Getting image dimensions
        self.img_width = img.width
        self.img_height = img.height
        print(self.img_width)
        print(self.img_height)

        # Launch Yolov8n-pose
        results = self.model(current_frame)
        annotated_frame = results[0].plot()

        self.yolov8_pose.keypoints = []

        # Calculate the number of persons detected
        num_persons = len(results[0].keypoints)
        print('Persons Nr:', num_persons)  # Print the number of persons detected
        
        
        try:
            for person_idx, per in enumerate(results[0].keypoints):
                print(f"Person index: {person_idx}")

                #     current_frame = cv2.putText(
                #     current_frame,
                #     f"{self.average_distance}",
                #     (self.keyp0_coordinate_x + 25, self.keyp0_coordinate_y + 25),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     1,
                #     (0, 0, 255),
                #     1,
                #     cv2.LINE_AA
                # ) 

                person_keypoints = Keypoints()  # Create a new instance of the Keypoints class for each person

                # Keypoints index number and x,y coordinates
                for keypoint_idx, kpt in enumerate(per):

                    # Display the keypoints info in the image
                    #     annotated_frame = cv2.putText(
                    #     annotated_frame,
                    #     f"{keypoint_idx}:({int(kpt[0])}, {int(kpt[1])})",
                    #     (int(kpt[0]), int(kpt[1])),
                    #     cv2.FONT_HERSHEY_SIMPLEX,
                    #     0.5,
                    #     (255, 0, 0),
                    #     1,
                    #     cv2.LINE_AA
                    # ) 

                    # Get the x,y of Keypoints
                    # Update the specific keypoint coordinates for each person
                    keypoint_coordinate_x = int(kpt[0])
                    keypoint_coordinate_y = int(kpt[1])

                    print(keypoint_coordinate_x)
                    print(keypoint_coordinate_y)

                    person_keypoints.box_topx_left = int(results[0].boxes.data[person_idx][0])
                    person_keypoints.box_topy_left = int(results[0].boxes.data[person_idx][1])
                    person_keypoints.box_width = int(results[0].boxes.data[person_idx][2])
                    person_keypoints.box_height = int(results[0].boxes.data[person_idx][3])

                    if keypoint_idx == 0:
                        self.keyp0_coordinate_x = keypoint_coordinate_x
                        self.keyp0_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 0", self.keyp0_coordinate_x, self.keyp0_coordinate_y)
                        setattr(person_keypoints, "key_p0_x", self.keyp0_coordinate_x)
                        setattr(person_keypoints, "key_p0_y", self.keyp0_coordinate_y)

                    elif keypoint_idx == 1:
                        self.keyp1_coordinate_x = keypoint_coordinate_x
                        self.keyp1_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 1", self.keyp1_coordinate_x, self.keyp1_coordinate_y)
                        setattr(person_keypoints, "key_p1_x", self.keyp1_coordinate_x)
                        setattr(person_keypoints, "key_p1_y", self.keyp1_coordinate_y)

                    elif keypoint_idx == 2:
                        self.keyp2_coordinate_x = keypoint_coordinate_x
                        self.keyp2_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p2_x", self.keyp2_coordinate_x)
                        setattr(person_keypoints, "key_p2_y", self.keyp2_coordinate_y)

                    elif keypoint_idx == 3:
                        self.keyp3_coordinate_x = keypoint_coordinate_x
                        self.keyp3_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 3", self.keyp3_coordinate_x, self.keyp3_coordinate_y)
                        setattr(person_keypoints, "key_p3_x", self.keyp3_coordinate_x)
                        setattr(person_keypoints, "key_p3_y", self.keyp3_coordinate_y)

                    elif keypoint_idx == 4:
                        self.keyp4_coordinate_x = keypoint_coordinate_x
                        self.keyp4_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p4_x", self.keyp4_coordinate_x)
                        setattr(person_keypoints, "key_p4_y", self.keyp4_coordinate_y)

                    elif keypoint_idx == 5:
                        self.keyp5_coordinate_x = keypoint_coordinate_x
                        self.keyp5_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 5", self.keyp5_coordinate_x, self.keyp5_coordinate_y)
                        setattr(person_keypoints, "key_p5_x", self.keyp5_coordinate_x)
                        setattr(person_keypoints, "key_p5_y", self.keyp5_coordinate_y)

                    elif keypoint_idx == 6:
                        self.keyp6_coordinate_x = keypoint_coordinate_x
                        self.keyp6_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 6", self.keyp0_coordinate_x, self.keyp0_coordinate_y)
                        setattr(person_keypoints, "key_p6_x", self.keyp6_coordinate_x)
                        setattr(person_keypoints, "key_p6_y", self.keyp6_coordinate_y)

                    elif keypoint_idx == 7:
                        self.keyp7_coordinate_x = keypoint_coordinate_x
                        self.keyp7_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p7_x", self.keyp7_coordinate_x)
                        setattr(person_keypoints, "key_p7_y", self.keyp7_coordinate_y)

                    elif keypoint_idx == 8:
                        self.keyp8_coordinate_x = keypoint_coordinate_x
                        self.keyp8_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p8_x", self.keyp8_coordinate_x)
                        setattr(person_keypoints, "key_p8_y", self.keyp8_coordinate_y)

                    elif keypoint_idx == 9:
                        self.keyp9_coordinate_x = keypoint_coordinate_x
                        self.keyp9_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p9_x", self.keyp9_coordinate_x)
                        setattr(person_keypoints, "key_p9_y", self.keyp9_coordinate_y)

                    elif keypoint_idx == 10:
                        self.keyp10_coordinate_x = keypoint_coordinate_x
                        self.keyp10_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p10_x", self.keyp10_coordinate_x)
                        setattr(person_keypoints, "key_p10_y", self.keyp10_coordinate_y)

                    elif keypoint_idx == 11:
                        self.keyp11_coordinate_x = keypoint_coordinate_x
                        self.keyp11_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p11_x", self.keyp11_coordinate_x)
                        setattr(person_keypoints, "key_p11_y", self.keyp11_coordinate_y)

                    elif keypoint_idx == 12:
                        self.keyp12_coordinate_x = keypoint_coordinate_x
                        self.keyp12_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p12_x", self.keyp12_coordinate_x)
                        setattr(person_keypoints, "key_p12_y", self.keyp12_coordinate_y)

                    elif keypoint_idx == 13:
                        self.keyp13_coordinate_x = keypoint_coordinate_x
                        self.keyp13_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p13_x", self.keyp13_coordinate_x)
                        setattr(person_keypoints, "key_p13_y", self.keyp13_coordinate_y)

                    elif keypoint_idx == 14:
                        self.keyp14_coordinate_x = keypoint_coordinate_x
                        self.keyp14_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p14_x", self.keyp14_coordinate_x)
                        setattr(person_keypoints, "key_p14_y", self.keyp14_coordinate_y)

                    elif keypoint_idx == 15:
                        self.keyp15_coordinate_x = keypoint_coordinate_x
                        self.keyp15_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p15_x", self.keyp15_coordinate_x)
                        setattr(person_keypoints, "key_p15_y", self.keyp15_coordinate_y)

                    elif keypoint_idx == 16:
                        self.keyp16_coordinate_x = keypoint_coordinate_x
                        self.keyp16_coordinate_y = keypoint_coordinate_y
                        #print("keypoint 2", self.keyp2_coordinate_x, self.keyp2_coordinate_y)
                        setattr(person_keypoints, "key_p16_x", self.keyp16_coordinate_x)
                        setattr(person_keypoints, "key_p16_y", self.keyp16_coordinate_y)


                person_keypoints.index_person = person_idx

                keypoints = [(self.keyp0_coordinate_x, self.keyp0_coordinate_y),
                            (self.keyp1_coordinate_x, self.keyp1_coordinate_y),
                            (self.keyp2_coordinate_x, self.keyp2_coordinate_y),
                            (self.keyp3_coordinate_x, self.keyp3_coordinate_y),
                            (self.keyp4_coordinate_x, self.keyp4_coordinate_y),
                            (self.keyp5_coordinate_x, self.keyp5_coordinate_y),
                            (self.keyp6_coordinate_x, self.keyp6_coordinate_y),
                            (self.keyp7_coordinate_x, self.keyp7_coordinate_y),
                            (self.keyp8_coordinate_x, self.keyp8_coordinate_y),
                            (self.keyp9_coordinate_x, self.keyp9_coordinate_y),
                            (self.keyp10_coordinate_x, self.keyp10_coordinate_y),
                            (self.keyp11_coordinate_x, self.keyp11_coordinate_y),
                            (self.keyp12_coordinate_x, self.keyp12_coordinate_y),
                            (self.keyp13_coordinate_x, self.keyp13_coordinate_y),
                            (self.keyp14_coordinate_x, self.keyp14_coordinate_y),
                            (self.keyp15_coordinate_x, self.keyp15_coordinate_y),
                            (self.keyp16_coordinate_x, self.keyp16_coordinate_y)]

                keypoints_distances = []

                for keypoint_idx, keypoint in enumerate(keypoints):
                    x, y = keypoint
                    #print(x)
                    #print(y)
                    #print(self.cv_image_array.shape)

                    if 0 <= x <= 1279 and 0 <= y <= 719:
                        distance = self.cv_image_array[int(y)][int(x)] / 1000
                        #distance = self.cv_image_array(int(y), int(x)) / 1000  
                        if distance > 0.01 and distance < 10.0:
                            keypoints_distances.append(distance)

                if len(keypoints_distances) > 0:
                    person_keypoints.average_distance = (sum(keypoints_distances) / len(keypoints_distances))
                    print(keypoints_distances)
                    #self.average_distance = (sum(keypoints_distances) / len(keypoints_distances))
                    self.center_x = self.keyp0_coordinate_x - (self.img_width // 2)
                    self.img_width_angle_radius = self.img_width_angle_degrees * (math.pi / 180)
                    self.relative_x_angle_radius = (self.center_x * self.img_width_angle_radius) / self.img_width
                    person_keypoints.x_person_relative = math.tan(self.relative_x_angle_radius) * person_keypoints.average_distance

                    #person_keypoints.x_person_relative = self.relative_x
                    #person_keypoints.average_distance = (sum(keypoints_distances) / len(keypoints_distances))
                    #     person_keypoints.standard_deviation = np.std(keypoints_distances)
                    # if person_keypoints.average_distance > 6.0:
                    #     print(keypoints_distances)
                    # if person_keypoints.average_distance == 0.0:
                    #     print(keypoints_distances) 
                    
                    print(f"Person {person_idx}: Average distance (Y relative) {person_keypoints.average_distance}")
                    self.yolov8_pose.keypoints.append(person_keypoints)

        except Exception as e:
            print(e)

        self.yolov8_pose.num_person = num_persons
        self.yolov8_pose_publisher.publish(self.yolov8_pose)

       
        cv2.imshow("Pose Detection", annotated_frame)
        cv2.waitKey(1)

        # cv2.imshow("c_camera", current_frame)
        # cv2.waitKey(1) 

    def get_aligned_depth_image_callback(self, img: Image):
        print("---")
        self.get_logger().info('Receiving aligned depth video frame')

        cv_image = self.br.imgmsg_to_cv2(img, "32FC1")
        self.cv_image_array = np.array(cv_image, dtype=np.dtype('f8'))

        #self.cv_image_array = np.array(self.cv_image, dtype=np.float32)

        # cv2.imshow("aligned_depth_camera", self.cv_image_array)
        # cv2.waitKey(1)

    def get_neck_position_callback(self, pos: Pose2D):
        print("Received Neck Position: pan =", int(pos.x), " tilt = ", int(pos.y))
        self.pan_cam = pos.x

    def timer_callback_trans(self, ):
        #print("---")
        pass
        


    def postprocess(self, preds, img, orig_imgs):
        # Return detection results for a given input image or list of images.
        preds = ops.non_max_suppression(preds,
                                        self.args.conf,
                                        self.args.iou,
                                        agnostic=self.args.agnostic_nms,
                                        max_det=self.args.max_det,
                                        classes=self.args.classes,
                                        nc=len(self.model.names))

        results = []
        for i, pred in enumerate(preds):
            orig_img = orig_imgs[i] if isinstance(orig_imgs, list) else orig_imgs
            shape = orig_img.shape
            pred[:, :4] = ops.scale_boxes(img.shape[2:], pred[:, :4], shape).round()
            pred_kpts = pred[:, 6:].view(len(pred), *self.model.kpt_shape) if len(pred) else pred[:, 6:]
            pred_kpts = ops.scale_coords(img.shape[2:], pred_kpts, shape)
            path = self.batch[0]
            img_path = path[i] if isinstance(path, list) else path
            results.append(
                Results(orig_img=orig_img,
                        path=img_path,
                        names=self.model.names,
                        boxes=pred[:, :6],
                        keypoints=pred_kpts))
        return results
      


"""
