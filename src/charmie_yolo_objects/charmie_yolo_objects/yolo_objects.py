#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String, Float32
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, Yolov8Objects, ListOfImages, ListOfStrings, PointCloudCoordinates, BoundingBox, BoundingBoxAndPoints
from charmie_interfaces.srv import GetPointCloud
from cv_bridge import CvBridge
import cv2 
import cvzone
import json

from pathlib import Path
# import numpy as np

import math
import time

objects_filename = "m_size_model_300_epochs_after_nandinho.pt"
shoes_filename = "shoes_socks_v1.pt"    


MIN_OBJECT_CONF_VALUE = 0.8
# LIST_OF_OBJECTS_OF_TASK = [""]


DRAW_OBJECT_CONF = True
DRAW_OBJECT_ID = True
DRAW_OBJECT_BOX = True
DRAW_OBJECT_CLASS = True
DRAW_OBJECT_LOCATION_COORDS = True
DRAW_OBJECT_LOCATION_HOUSE_FURNITURE = False


class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Yolo Object Node")

         ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("debug_draw", True) 
    
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
            self.get_logger().info("Successfully Imported data from json configuration files. (objects_list)")
        except:
            self.get_logger().error("Could NOT import data from json configuration files. (objects_list)")

        # This is the variable to change to True if you want to see the bounding boxes on the screen and to False if you don't
        self.DEBUG_DRAW = self.get_parameter("debug_draw").value

        # Import the models, one for each category
        self.object_model = YOLO(self.complete_path + objects_filename)
        self.shoes_model = YOLO(self.complete_path + shoes_filename)

        ### Topics ###
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        ### self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)

        # For individual images
        self.cropped_image_subscription = self.create_subscription(ListOfImages, '/cropped_image', self.cropped_image_callback, 10)
        self.cropped_image_object_detected_publisher = self.create_publisher(ListOfStrings, '/cropped_image_object_detected', 10)

        # Subscriber (Yolov8_Objects TR Parameters)
        self.minimum_person_confidence_subscriber = self.create_subscription(Float32, "min_obj_conf", self.get_minimum_object_confidence_callback, 10)
        
        # Diagnostics        
        self.yolo_object_diagnostic_publisher = self.create_publisher(Bool, "yolo_object_diagnostic", 10)
 
        # Publish Results
        self.objects_publisher = self.create_publisher(Yolov8Objects, 'objects_detected', 10)
        self.objects_filtered_publisher = self.create_publisher(Yolov8Objects, 'objects_detected_filtered', 10)
        
        ### Services (Clients) ###
        # Point Cloud
        ### self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")
        ###
        ### while not self.point_cloud_client.wait_for_service(1.0):
        ###     self.get_logger().warn("Waiting for Server Point Cloud...")
        
        ### Variables ###

        # to calculate the FPS
        self.prev_frame_time = 0 # used to record the time when we processed last frame
        self.new_frame_time = 0 # used to record the time at which we processed current frame

        self.object_yolo_threshold = 0.2
        self.shoes_yolo_threshold = 0.5

        # self.object_results = []
        self.object_list = []
        self.waiting_for_pcloud = False
        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.hand_rgb = Image()

        flag_diagn = Bool()
        flag_diagn.data = True
        self.yolo_object_diagnostic_publisher.publish(flag_diagn)

        self.lar_v_final_classname = ['7up', 'Apple', 'Bag', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser',
                                      'Coffe_grounds', 'Cola', 'Cornflakes', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_Tea', 
                                      'Juice_pack', 'Knife', 'Lemon', 'Milk', 'Mustard', 'Orange', 'Orange_juice', 'Peach', 'Pear',                                  
                                      'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube', 'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 
                                      'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup', 'Tropical_juice', 'Tuna', 'Water']
        
        self.lar_v_final_classname = ['7up', 'Strawberry_jello', 'Bag', 'Banana', 'Baseball', 'Bowl', 'Cheezit', 'Chocolate_jello', 'Cleanser',
                                      'Coffe_grounds', 'Cola', 'Cornflakes', 'Cup', 'Dice', 'Dishwasher_tab', 'Fork', 'Iced_Tea', 
                                      'Juice_pack', 'Knife', 'Lemon', 'Milk', 'Mustard', 'Orange', 'Orange_juice', 'Peach', 'Pear',                                  
                                      'Plate', 'Plum', 'Pringles', 'Red_wine', 'Rubiks_cube', 'Soccer_ball', 'Spam', 'Sponge', 'Spoon', 
                                      'Strawberry', 'Strawberry_jello', 'Sugar', 'Tennis_ball', 'Tomato_soup', 'Tropical_juice', 'Tuna', 'Water']
        

        self.objects_classNames_dict = {}
        # self.serve_breakfast_classname = ['bowl', 'spoon', "milk", "cereal"]

        self.shoes_socks_classname = ['shoe', 'sock']      
        
        # depending on the filename selected, the class names change
        if objects_filename == 'vfinal.pt' or objects_filename == 'M_300epochs.pt' or objects_filename == "m_size_model_300_epochs_after_nandinho.pt":
            self.objects_classNames = self.lar_v_final_classname


        # elif objects_filename == 'serve_breakfast_v1.pt':
        #     self.objects_classNames = self.serve_breakfast_classname
        else:
            print('Something is wrong with your model name or directory. Please check if the variable filename fits the name of your model and if the loaded directory is the correct.')
            
        # self.objects_classNames_dict = {}
        # for item in self.objects_classNames:
        #     self.objects_classNames_dict[item["name"]]=item["class"]
        
        self.objects_classNames_dict = {item["name"]: item["class"] for item in self.objects_file}
        
        print(self.objects_classNames_dict)



    def get_minimum_object_confidence_callback(self, state: Float32):
        global MIN_OBJECT_CONF_VALUE
        # print(state.data)
        if 0.0 <= state.data <= 1.0:
            MIN_OBJECT_CONF_VALUE = state.data
            self.get_logger().info('NEW MIN_OBJECT_CONF_VALUE RECEIVED')    
        else:
            self.get_logger().info('ERROR SETTING MIN_OBJECT_CONF_VALUE')   


    def get_color_image_head_callback(self, img: Image):

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
            ### self.call_point_cloud_server(req2)

            ### TEMP: MUST DELETE LATER
            new_pcloud = PointCloudCoordinates()
            self.post_receiving_pcloud(new_pcloud)
            
    def post_receiving_pcloud(self, new_pcloud):

        current_frame = self.br.imgmsg_to_cv2(self.head_rgb, "bgr8")
        current_frame_draw = current_frame.copy()
        annotated_frame = self.object_results[0].plot()

        # Calculate the number of persons detected
        num_obj = len(self.object_results[0])

        yolov8_obj = Yolov8Objects()
        yolov8_obj_filtered = Yolov8Objects()
        num_objects_filtered = 0

        # print(num_obj)
        # print(self.object_results[0])
        # print(self.object_results[0].boxes)

        for object_idx in range(num_obj):
            boxes_id = self.object_results[0].boxes[object_idx]

            # print(self.object_results[0].boxes)

            ALL_CONDITIONS_MET = 1

            # adds object to "object_pose" without any restriction
            new_person = DetectedObject()
            new_person = self.add_object_to_detectedobject_msg()
            yolov8_obj.objects.append(new_person)



            # cls = int(boxes_id.cls[0])
            # object_name = self.objects_classNames[cls]
            # object_name_adjusted = object_name.replace("_", " ").title()
            object_name = self.objects_classNames[int(boxes_id.cls[0])].replace("_", " ").title()



            object_class = self.objects_classNames_dict[object_name]
            # print("name", object_name, object_name_adjusted)
            
            
            # for item in self.objects_file:
            #     print("for", item["name"])
            #     if item["name"] == object_name:
            #         # print(item["class"])
            #         object_class = item["class"]
            #         break



            # object_class = 
            
            
            
            # threshold = self.object_threshold
            # classNames = self.objects_classNames
            """
            # this for only does 1 time ...
            for r in self.object_results:
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
            """
            object_id = boxes_id.id
            if boxes_id.id == None:
                object_id = 0 

            # checks whether the person confidence is above a defined level
            if not boxes_id.conf >= MIN_OBJECT_CONF_VALUE:
                ALL_CONDITIONS_MET = ALL_CONDITIONS_MET*0
                # print("- Misses minimum person confidence level")

            if ALL_CONDITIONS_MET:
                num_objects_filtered+=1

                yolov8_obj_filtered.objects.append(new_person)

                if self.DEBUG_DRAW:

                    red_yp = (56, 56, 255)
                    lblue_yp = (255,194,0)
                    blue_yp = (255,0,0)
                    green_yp = (0,255,0)
                    orange_yp = (51,153,255)
                    magenta_yp = (255, 51, 255)
                    white_yp = (255, 255, 255)


                    # MISS ADDING:
                    # Cleaning Supplies
                    # Drinks
                    # Foods
                    # Fruits
                    # Toys
                    # Snacks
                    # Dishes

                    if object_class == "Foods":
                        bb_color = lblue_yp
                    elif object_class == "Toys":
                        bb_color = orange_yp
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

                    cv2.putText(current_frame_draw, object_name+object_class, (start_point_text[0], start_point_text[1]), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                    
                    
                    
                    # if DRAW_OBJECT_LOCATION_COORDS:
                    #     cv2.putText(current_frame_draw, '('+str(round(new_person.position_relative.x,2))+
                    #                 ', '+str(round(new_person.position_relative.y,2))+
                    #                 ', '+str(round(new_person.position_relative.z,2))+')',
                    #                 self.center_torso_person_list[object_idx], cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)         
                    
                    # if DRAW_OBJECT_LOCATION_HOUSE_FURNITURE:
                    #     cv2.putText(current_frame_draw, new_person.room_location+" - "+new_person.furniture_location,
                    #                 (self.center_torso_person_list[object_idx][0], self.center_torso_person_list[object_idx][1]+30), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    
        yolov8_obj.num_objects = num_obj
        self.objects_publisher.publish(yolov8_obj)

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
            cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
            cv2.imshow("Yolo Object Detection", annotated_frame)
            # cv2.imshow("Camera Image", current_frame)
            cv2.waitKey(1)
        
        ### TEM QUE PASSAR PARA A FUNCAO do Point Cloud
        self.waiting_for_pcloud = False

        self.get_logger().info(f"Objects detected: {num_obj}/{num_objects_filtered}")
        self.get_logger().info(f"Time Yolo_Objects: {round(time.perf_counter() - self.tempo_total,2)}")


    def add_object_to_detectedobject_msg(self):



        # COMPUTE CLASS NAME
        pass
        p = DetectedObject()
        return p

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
                classNames = self.lar_v_final_classname
                threshold = self.object_threshold
            else:
                results = self.shoes_model(img)
                classNames = self.shoes_socks_classname
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
        
def main(args=None):
    rclpy.init(args=args)
    node = Yolo_obj()
    rclpy.spin(node)
    rclpy.shutdown()