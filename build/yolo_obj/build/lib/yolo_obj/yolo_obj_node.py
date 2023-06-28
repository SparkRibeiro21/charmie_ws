#!/usr/bin/env python3
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
from ultralytics.yolo.utils import DEFAULT_CFG, ROOT, ops
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from charmie_interfaces.msg import MultiObjects
from cv_bridge import CvBridge
import cv2 
import cvzone

import numpy as np

import math

class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")
        #self.debug_draw = Bool()
        self.debug_draw = False

        self.model = YOLO('best.pt')
        
        self.objects_publisher = self.create_publisher(MultiObjects, 'objects_detected', 10)
        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        
        # Variables
        self.br = CvBridge()

        self.classNames = ["Apple", "Bag", "Banana", "Bottle", "Bowl", "Chair",
              "Cup", "Fork", "Knife", "Manga", "Mug", "Pear", "Person",
              "Plastic-bag", "Plate", "Pringles", "Shelf", "Spoon",
              "Table", "Tin-can", "Trash-can"]

        self.crockery = ["Bowl", "Cup", "Fork", "Knife", "Mug", "Plate", "Spoon"]

        self.obj = MultiObjects()
        self.obj.objects = []
        self.obj.confidence = []



    def get_color_image_callback(self, img: Image):
        self.get_logger().info('Receiving color video frame')
        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        
        self.obj.objects = []
        self.obj.confidence = []

        results = self.model(current_frame, stream = True)

        for r in results:
            boxes = r.boxes

            for box in boxes:
                cls = int(box.cls[0])
                conf = math.ceil(box.conf[0] * 100) / 100
                
                if self.debug_draw:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    center_point = round((x1 + x2) / 2), round((y1 + y2) / 2)

                    w, h = x2 - x1, y2 - y1
                    
                    cvzone.cornerRect(current_frame, (x1, y1, w, h), l=15)
                    cvzone.putTextRect(current_frame, f"{self.classNames[cls]} {conf}", (max(0, x1), max(35, y1)), scale=1.5, thickness=1, offset=3)
                    
                    print('BBB')
                    print(self.classNames[cls])
                    print(type(self.classNames[cls]))

                    cv2.imshow('Output', current_frame) # Exibir a imagem capturada

                    if cv2.waitKey(1) == ord('q'): # Se a tecla 'q' for pressionada, saia
                        break
                
                else:
                    pass

                self.obj.objects.append(str(self.classNames[cls]))
                self.obj.confidence.append(conf)
            
            self.objects_publisher.publish(self.obj)

            

        
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    rclpy.spin(node)
    rclpy.shutdown()