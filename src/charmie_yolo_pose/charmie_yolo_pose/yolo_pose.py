#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedPerson, Yolov8Pose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# import array  
# import sys
# import math
import time

# configurable parameters through ros topics
ONLY_DETECT_PERSON_LEGS_VISIBLE = False       # if False only detects people whose legs are visible 
MIN_PERSON_CONF_VALUE = 0.5                 # defines the minimum confidence value to be considered a person
MIN_KP_TO_DETECT_PERSON = 4                 # this parameter does not consider the four legs keypoints 
ONLY_DETECT_PERSON_RIGHT_IN_FRONT = False    # only detects person right in front of the robot both on the x and y axis 

# must be adjusted if we want just to not detect the feet in cases where the walls are really low and we can see the knees
# 3 may be used in cases where it just does not detect on of the feet 
NUMBER_OF_LEG_KP_TO_BE_DETECTED = 2
MIN_KP_CONF_VALUE = 0.5

# TO DO TIAGO RIBEIRO:
# - add x_rel and y_rel to detectedperson

DRAW_PERSON_CONF = True
DRAW_PERSON_ID = True
DRAW_PERSON_BOX = True
DRAW_PERSON_KP = True
DRAW_LOW_CONF_KP = False


class YoloPoseNode(Node):
    def __init__(self):
        super().__init__("YoloPose")
        self.get_logger().info("Initialised YoloPose Node")

        # Yolo Model - Yolov8 Pode Nano
        self.model = YOLO('yolov8n-pose.pt')

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.debug_draw = True

        # Publisher (Pose of People Detected Filtered and Non Filtered)
        self.person_pose_publisher = self.create_publisher(Yolov8Pose, "person_pose", 10)
        self.person_pose_filtered_publisher = self.create_publisher(Yolov8Pose, "person_pose_filtered", 10)

        # Subscriber (Yolov8_Pose TR Parameters)
        self.only_detect_person_legs_visible_subscriber = self.create_subscription(Bool, "only_det_per_legs_vis", self.get_only_detect_person_legs_visible_callback, 10)
        self.minimum_person_confidence_subscriber = self.create_subscription(Float32, "min_per_conf", self.get_minimum_person_confidence_callback, 10)
        self.minimum_keypoints_to_detect_person_subscriber = self.create_subscription(Int16, "min_kp_det_per", self.get_minimum_keypoints_to_detect_person_callback, 10)
        self.only_detect_person_right_in_front_subscriber = self.create_subscription(Bool, "only_det_per_right_in_front", self.get_only_detect_person_right_in_front_callback, 10)

        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        # self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)


        # to calculate the FPS
        self.prev_frame_time = 0 # used to record the time when we processed last frame
        self.new_frame_time = 0 # used to record the time at which we processed current frame
        
        self.br = CvBridge()
        # self.yolov8_pose = Yolov8Pose()
        # self.yolov8_pose_filtered = Yolov8Pose()

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
        # results = self.model(current_frame)

        # The persist=True argument tells the tracker that the current image or frame is the next in a sequence and to expect tracks from the previous image in the current image.
        results = self.model.track(current_frame, persist=True, tracker="bytetrack.yaml")
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

        # Calculate the number of persons detected
        num_persons = len(results[0].keypoints)
        if not results[0].keypoints.has_visible:
            num_persons = 0

        print("___START___")
        yolov8_pose = Yolov8Pose()
        yolov8_pose_filtered = Yolov8Pose()
        num_persons_filtered = 0

        for person_idx in range(num_persons):
            keypoints_id = results[0].keypoints[person_idx]
            boxes_id = results[0].boxes[person_idx]
            ALL_CONDITIONS_MET = 1
            
            # print(keypoints_id.xy[0][0])
            # print(keypoints_id.conf)   

            print(boxes_id.conf)
            
            # guardar na variavel criada do yolo pose 
            # print(keypoints_id.conf[0][self.KNEE_LEFT_KP], 
            #       keypoints_id.conf[0][self.KNEE_RIGHT_KP],
            #       keypoints_id.conf[0][self.ANKLE_LEFT_KP], 
            #       keypoints_id.conf[0][self.ANKLE_RIGHT_KP]
            #       )
            
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

            print("legs_ctr = ", legs_ctr)
            print("id = ", person_id)
            

            body_kp_high_conf_counter = 0
            for kp in range(self.N_KEYPOINTS - self.NUMBER_OF_LEGS_KP): # all keypoints without the legs
                if keypoints_id.conf[0][kp] > MIN_KP_CONF_VALUE:
                    body_kp_high_conf_counter+=1
            print("body_kp_high_conf_counter = ", body_kp_high_conf_counter)


            ########## afterwards what should be used to calculate center_comm_position should be the x and y person coordinates related to the robot 
            center_comm_position = False
            if self.img_width*1/3 < int(keypoints_id.xy[0][self.NOSE_KP][0]) < self.img_width*2/3:
                center_comm_position = True
              

            # checks whether the person confidence is above a defined level
            if not boxes_id.conf >= MIN_PERSON_CONF_VALUE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                print("- Misses minimum person confidence level")

            # checks if flag to detect people whose legs are visible 
            if not legs_ctr >= NUMBER_OF_LEG_KP_TO_BE_DETECTED and ONLY_DETECT_PERSON_LEGS_VISIBLE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                print("- Misses legs visible flag")
            
            # checks whether the minimum number if body keypoints (excluding legs) has high confidence
            if not body_kp_high_conf_counter >= MIN_KP_TO_DETECT_PERSON:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                print("- Misses minimum number of body keypoints")

            # checks if flag to detect people whose legs are visible 
            if not center_comm_position and ONLY_DETECT_PERSON_RIGHT_IN_FRONT:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                print("Misses minimum number of body keypoints")


            # adds people to "person_pose" without any restriction
            new_person = DetectedPerson()
            new_person = self.add_person_to_detectedperson_msg(boxes_id, keypoints_id)
            yolov8_pose.persons.append(new_person)

            
            if ALL_CONDITIONS_MET:
                num_persons_filtered+=1

                # adds people to "person_pose" without any restriction
                # code here to add to filtered topic
                yolov8_pose_filtered.persons.append(new_person)

                if self.debug_draw:
                    
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

                        # center_p = (int(keypoints_id.xy[0][self.EYE_LEFT_KP][0]), int(keypoints_id.xy[0][self.EYE_LEFT_KP][1]))
                        # cv2.circle(current_frame_draw, center_p, 7, (255,255,255), -1)

            print("===")

        yolov8_pose.num_person = num_persons
        self.person_pose_publisher.publish(yolov8_pose)

        yolov8_pose_filtered.num_person = num_persons_filtered
        self.person_pose_filtered_publisher.publish(yolov8_pose_filtered)

        print("____END____")

        self.new_frame_time = time.time()
        self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
        self.prev_frame_time = self.new_frame_time
        self.fps = str(self.fps)
        print("fps = " + self.fps)

        if self.debug_draw:
            # putting the FPS count on the frame
            cv2.putText(current_frame_draw, 'fps:' + self.fps, (0, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(current_frame_draw, 'np:' + str(num_persons_filtered) + '/' + str(num_persons), (180, self.img_height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Yolo Pose Detection", annotated_frame)
            # cv2.imshow("Yolo Track", annotated_frame2)
            cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
            cv2.waitKey(1)
        
        """
        cv2.imshow("Intel RealSense Current Frame", current_frame)
        cv2.waitKey(1)
        with open("charmie_color_yp.raw", "wb") as f:
            f.write(current_frame.tobytes())
        height, width, channels = current_frame.shape
        print(height, width, channels)
        cv2.imwrite("charmie_color_yp.jpg", current_frame) 
        time.sleep(1)
        """
        

    def get_aligned_depth_image_callback(self, img: Image):
        pass

        print(img.height, img.width)
        current_frame = self.br.imgmsg_to_cv2(img, desired_encoding="passthrough")
        depth_array = np.array(current_frame, dtype=np.float32)
        # center_idx = np.array(depth_array.shape) // 2
        # print ('center depth:', depth_array[center_idx[0], center_idx[1]])

        """
        if img.height == 720:
            file_name = "charmie_depth_yp.txt"
        
            # Save the NumPy array to a text file
            with open(file_name, 'w') as file:
                np.savetxt(file, depth_array, fmt='%d', delimiter='\t')
        
            # np.savetxt(file_name, depth_array, fmt='%d', delimiter='\t', mode='a')
        
            # Optional: You can also specify formatting options using 'fmt'.
            # In this example, '%d' specifies integer formatting and '\t' is used as the delimiter.

            print(f"Array saved to {file_name}")
            time.sleep(1)
        
        
        cv2.imshow("Intel RealSense Depth Alligned", current_frame)
        cv2.waitKey(1) 
        """       
        
    def add_person_to_detectedperson_msg(self, boxes_id, keypoints_id):
        # receives the box and keypoints of a specidic person and returns the detected person 
        # it can be done in a way that is only made once per person and both 'person_pose' and 'person_pose_filtered'

        person_id = boxes_id.id
        if boxes_id.id == None:
            person_id = 0 

        # adds people to "person_pose" without any restriction
        new_person = DetectedPerson()

        new_person.index_person = int(person_id)
        new_person.conf_person = float(boxes_id.conf)
        new_person.x_rel = 0.0 # MISSING!!!
        new_person.y_rel = 0.0 # MISSING!!!
        new_person.box_top_left_x = int(boxes_id.xyxy[0][0])
        new_person.box_top_left_y = int(boxes_id.xyxy[0][1])
        new_person.box_width = int(boxes_id.xyxy[0][2]) - int(boxes_id.xyxy[0][0])
        new_person.box_height = int(boxes_id.xyxy[0][3]) - int(boxes_id.xyxy[0][1])

        # print(int(keypoints_id.xy[0][self.NOSE_KP][0]), int(keypoints_id.xy[0][self.NOSE_KP][1]), float(keypoints_id.conf[0][self.NOSE_KP]))

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


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()