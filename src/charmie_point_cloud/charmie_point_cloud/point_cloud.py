#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech, NeckPosition, RequestPointCloud, RetrievePointCloud

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
        self.request_point_cloud_subscriber = self.create_subscription(RequestPointCloud, "get_point_cloud", self.get_request_point_cloud_callback, 10)

        # RetrievePointCloud
        self.retrieve_point_cloud_publisher = self.create_publisher(RetrievePointCloud, "set_point_cloud", 10)


        self.pcloud = PointCloud()

    def get_color_image_callback(self, img: Image):
        pass

    def get_aligned_depth_image_callback(self, img: Image):
        pass

    def get_neck_position_callback(self, neck_pos: NeckPosition):
        pass

    def get_request_point_cloud_callback(self, req: RequestPointCloud):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()