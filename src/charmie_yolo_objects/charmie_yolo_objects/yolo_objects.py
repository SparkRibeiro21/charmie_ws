#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, Yolov8Objects
from cv_bridge import CvBridge
import cv2 
import cvzone

import numpy as np

import math
import time

filename = "last.pt"


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

        self.model = YOLO('/home/utilizador/charmie_ws/src/charmie_yolo_objects/charmie_yolo_objects/' + filename)
        
        self.objects_publisher = self.create_publisher(Yolov8Objects, 'objects_detected', 10)
        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        
        # Variables
        self.br = CvBridge()

        Rui_className = ['7_up', 'bag', 'bowl', 'cheezit', 'chocolate_jello', 'cleanser', 'coffee_grounds', 
                         'cola', 'cornflakes', 'cup', 'dishwasher_tab', 'fork', 'iced_tea', 'juice_pack', 
                         'knife', 'milk', 'mustard', 'orange', 'orange_juice', 'plate', 'pringles', 'red_wine', 
                         'spam', 'sponge', 'spoon', 'strawberry_jello', 'sugar', 'tennis_ball', 'tomato_soup', 
                         'tropical_juice', 'tuna', 'water']
        
        door_classname = ['BallHandler', 'Door', 'Drawer', 
              'Fridge_Door', 'Hidden Handler', 'LevelHandler', 'PullHandler', 
              'WardrobeHandler', 'Wardrobe_Door']
        
        # depending on the filename selected, the class names change
        if filename=='doors.pt':
            self.classNames = door_classname
        elif filename=='best.pt' or filename=='last.pt':
            self.classNames = Rui_className
        else:
            print('Something is wrong with your model name or directory. Please check if the variable filename fits the name of your model and if the loaded directory is the correct.')
            

        #self.crockery = ["Bowl", "Cup", "Fork", "Knife", "Mug", "Plate", "Spoon"]

        # self.obj = Yolov8Objects()
        # self.obj.objects = []
        # self.obj.confidence = []
        # self.obj.distance = []
        # self.obj.position = []

        self.yolo_object_diagnostic_publisher = self.create_publisher(Bool, "yolo_object_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.yolo_object_diagnostic_publisher.publish(flag_diagn)

### --------------------------------------------- ROUTINE EXPLANATION --------------------------------------------- ### 

# The following callback runs the NN model each time the camera publishes an image. After running it, the model 
# analyzes the image to identify any objects. It then publishes this information on the topic 'objects_detected' 
# and displays it on the screen. Any object detected with less than 50% of confidence is ignored.

### --------------------------------------------- /////////////// --------------------------------------------- ###

    def get_color_image_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        
        # self.obj.objects = []
        # self.obj.confidence = []
        # self.obj.distance = []
        # self.obj.position = []

        # minimum value of confidence for object to be accepted as true and sent via topic
        self.threshold = 0.2

        results = self.model(current_frame, stream = True)

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
                    if conf > self.threshold:
                        cvzone.cornerRect(current_frame, (x1, y1, w, h), l=15)    
                        cvzone.putTextRect(current_frame, f"{self.classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
                    
                        print(self.classNames[cls], 'confidence = ' + str(conf))
                
                else:
                    pass
                
                # if conf > self.threshold:
                #     self.obj.objects.append(str(self.classNames[cls]))
                #     self.obj.confidence.append(conf)
            
            # self.objects_publisher.publish(self.obj)

            self.new_frame_time = time.time()
            self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
            self.prev_frame_time = self.new_frame_time

            self.fps = str(self.fps)

            if self.debug_draw:
                # putting the FPS count on the frame
                cv2.putText(current_frame, 'fps = ' + self.fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow('Output', current_frame) # Exibir a imagem capturada
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