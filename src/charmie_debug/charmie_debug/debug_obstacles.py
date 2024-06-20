#!/usr/bin/env python3
# from ultralytics import YOLO
import rclpy
from rclpy.node import Node
# from example_interfaces.msg import Bool
from geometry_msgs.msg import Point, Pose2D
from sensor_msgs.msg import Image
from charmie_interfaces.msg import DetectedObject, Yolov8Objects, BoundingBox, BoundingBoxAndPoints, PointCloudCoordinates, ListOfPoints
from charmie_interfaces.srv import GetPointCloud, ActivateYoloObjects
from cv_bridge import CvBridge
# import cv2 
# import json
import threading
# from pathlib import Path
# import math
import time


class Yolo_obj(Node):
    def __init__(self):
        super().__init__("Yolo_obj")
        self.get_logger().info("Initialised Debug Obstacles Node")

        ### Topics ###
        # Intel Realsense
        # Head
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        # Hand
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)
        
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        
        # Camera Obstacles
        self.temp_camera_obstacles_publisher = self.create_publisher(ListOfPoints, "camera_obstacles", 10)
       
        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        ### Variables ###

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
        
        self.br = CvBridge()
        self.head_rgb = Image()
        self.hand_rgb = Image()
        self.new_head_rgb = False
        self.new_hand_rgb = False
        self.waiting_for_pcloud = False
        self.point_cloud_response = GetPointCloud.Response()

    # request point cloud information from point cloud node
    def call_point_cloud_server(self, req, camera):
        request = GetPointCloud.Request()
        request.data = req
        request.retrieve_bbox = True
        request.camera = camera
    
        future = self.point_cloud_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud)

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_response = future.result()
            self.waiting_for_pcloud = False
            # print("Received Back")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def get_aligned_depth_image_head_callback(self, img: Image):
        self.head_depth_img = img
        # print("Received Head Depth Image")

    def get_aligned_depth_image_hand_callback(self, img: Image):
        self.hand_depth_img = img
        # print("Received Hand Depth Image")

    def get_color_image_hand_callback(self, img: Image):
        self.hand_rgb = img
        self.new_hand_rgb = True

    def get_color_image_head_callback(self, img: Image):
        self.head_rgb = img
        self.new_head_rgb = True

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta

        
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
        # self.new_pcloud = PointCloudCoordinates()

    def get_point_cloud(self, wait_for_end_of=True):

        requested_objects = []
            
        bb = BoundingBox()
        bb.box_top_left_x = 0
        bb.box_top_left_y = 0
        bb.box_width = 1280
        bb.box_height = 720

        get_pc = BoundingBoxAndPoints()
        get_pc.bbox = bb

        requested_objects.append(get_pc)

        self.node.waiting_for_pcloud = True
        self.node.call_point_cloud_server(requested_objects, "head")

        while self.node.waiting_for_pcloud:
            pass

        # self.new_pcloud = self.node.point_cloud_response.coords
        return self.node.point_cloud_response.coords

    # main state-machine function
    def main(self):
        
        # debug print to know we are on the main start of the task
        self.node.get_logger().info("In Debug Obstacles Main...")

        time.sleep(5)

        while True:
            ### change resp_todos to uteis in point_cloud to remove all (0.0, 0.0, 0.0) from pcloud points ###
            pc = self.get_point_cloud() 
            # print(type(pc[0].bbox_point_coords))
            # print((pc[0].bbox_point_coords[0]))

            pc_lp = ListOfPoints()

            # print(len(pc[0].bbox_point_coords))

            for p in pc[0].bbox_point_coords:
                # print(p)
                pc_lp.coords.append(p)
            
            self.node.temp_camera_obstacles_publisher.publish(pc_lp)
            time.sleep(0.1)