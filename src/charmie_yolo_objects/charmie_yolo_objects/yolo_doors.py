#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, Yolov8Objects, ListOfImages, ListOfStrings
from charmie_interfaces.srv import ActivateYoloObjects
from cv_bridge import CvBridge
import cv2 
import cvzone

from pathlib import Path

from pathlib import Path

# import numpy as np

import math
import time

objects_filename = "door_bruno.pt"


### --------------------------------------------- CODE EXPLANATION --------------------------------------------- ### 

# This code aims to identify objects known previously when the camera is on. It receives an image from the camera (IntelRS)
# and publishes the name of the object, the confidence, the distance of the object to the camera and the position of the object 
# in relation to the robot. To do so, it runs a pre trained yolov8 model of a neural network.

### --------------------------------------------- /////////////// --------------------------------------------- ###

class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.debug_draw = True

        # used to record the time when we processed last frame
        self.prev_frame_time = 0
        
        # used to record the time at which we processed current frame
        self.new_frame_time = 0

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # self.model = YOLO('/home/utilizador/charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects/' + filename)
        self.object_model = YOLO(self.complete_path + objects_filename)
        
        self.doors_filtered_publisher = self.create_publisher(Yolov8Objects, 'doors_detected_filtered', 10)
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        
        # Variables
        self.br = CvBridge()

        self.door_classname = ['Dishwasher', 'Door', 'Drawer', 'LevelHandler', 'Wardrobe door']
        
        # depending on the filename selected, the class names change
        if objects_filename == 'door_bruno.pt':
            self.objects_classNames = self.door_classname
        else:
            print('Something is wrong with your model name or directory. Please check if the variable filename fits the name of your model and if the loaded directory is the correct.')
            
        self.object_threshold = 0.5

        ### Services ###
        self.activate_yolo_objects_service = self.create_service(ActivateYoloObjects, "activate_yolo_objects", self.callback_activate_yolo_objects)


### --------------------------------------------- ROUTINE EXPLANATION --------------------------------------------- ### 

# The following callback runs the NN model each time the camera publishes an image. After running it, the model 
# analyzes the image to identify any objects. It then publishes this information on the topic 'objects_detected' 
# and displays it on the screen. Any object detected with less than self.object_threshold(%) of confidence is ignored.

### --------------------------------------------- /////////////// --------------------------------------------- ###

    def callback_activate_yolo_objects(self, request, response):
        
        # Type of service received: 
        # bool activate_objects                       # activate or deactivate yolo object detection
        # bool activate_shoes                         # activate or deactivate yolo shoes detection
        # bool activate_doors                         # activate or deactivate yolo doors detection (includes doors, drawers, washing machine door, closet with doors)
        # float64 minimum_objects_confidence          # adjust the minimum accuracy to assume as an object
        # float64 minimum_shoes_confidence            # adjust the minimum accuracy to assume as a shoe
        # float64 minimum_doors_confidence            # adjust the minimum accuracy to assume as a door or handle
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        global MIN_OBJECT_CONF_VALUE

        self.get_logger().info("Received Activate Yolo Objects %s" %("("+str(request.activate_objects)+", "
                                                                        +str(request.activate_shoes)+", "
                                                                        +str(request.activate_doors)+", "
                                                                        +str(request.minimum_objects_confidence)+")"))

        self.ACTIVATE_YOLO_OBJECTS = request.activate_objects
        self.ACTIVATE_YOLO_SHOES = request.activate_shoes
        self.ACTIVATE_YOLO_DOORS = request.activate_doors
        MIN_OBJECT_CONF_VALUE = request.minimum_objects_confidence

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response

    def get_color_image_head_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame head')
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
                
                if self.debug_draw:
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

        if self.debug_draw:
            # putting the FPS count on the frame
            # cv2.putText(current_frame, 'fps = ' + self.fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow('Output_head', current_frame) # Exibir a imagem capturada
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
                
                if self.debug_draw:
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

        if self.debug_draw:
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

        
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    rclpy.spin(node)
    rclpy.shutdown()