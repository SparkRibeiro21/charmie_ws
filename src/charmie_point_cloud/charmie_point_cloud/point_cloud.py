#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import NeckPosition, RequestPointCloud, RetrievePointCloud
from cv_bridge import CvBridge, CvBridgeError

import cv2


DEBUG_DRAW = True



class PointCloud():
    def __init__(self):
        print("New PointCloud Class Initialised")
        

    # your code here

class PointCloudNode(Node):

    def __init__(self):
        super().__init__("PointCloud")
        self.get_logger().info("Initialised CHARMIE PointCloud Node")
        
        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        
        # Neck Position
        self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)

        # RequestPointCloud
        # self.request_point_cloud_subscriber = self.create_subscription(RequestPointCloud, "ask_point_cloud", self.get_request_point_cloud_callback, 10)

        # RetrievePointCloud
        self.retrieve_point_cloud_publisher = self.create_publisher(RetrievePointCloud, "get_point_cloud", 10)

        self.create_timer(1, self.get_request_point_cloud_callback)

        # Point Cloud Instance
        self.br = CvBridge()
        self.pcloud = PointCloud()
        self.rgb_img = Image()
        self.depth_img = Image()
        self.neck_position = NeckPosition()

    def get_color_image_callback(self, img: Image):
        self.rgb_img = img
        print("Received RGB Image")

        if DEBUG_DRAW:
            current_frame = self.br.imgmsg_to_cv2(self.rgb_img, "bgr8")
            cv2.imshow("Yolo Pose TR Detection", current_frame)
            # cv2.imshow("Yolo Pose Detection", annotated_frame)
            cv2.waitKey(1)

    def get_aligned_depth_image_callback(self, img: Image):
        self.depth_img = img
        print("Received Depth Image")

    def get_neck_position_callback(self, neck_pos: NeckPosition):
        self.neck_position = neck_pos
        print("Received Neck Position")

    def get_request_point_cloud_callback(self, req: RequestPointCloud):
        pass
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()