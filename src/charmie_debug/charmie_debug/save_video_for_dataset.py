#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
from datetime import datetime
from pathlib import Path
import cv2

class SaveImagesDatasetNode(Node):

    def __init__(self):
        super().__init__("SaveImagesDataset")
        self.get_logger().info("Initialised CHARMIE SaveImagesDataset Node")

        # Declare ROS 2 parameters
        self.declare_parameter("head_cam", False)
        self.declare_parameter("hand_cam", False)
        self.declare_parameter("base_cam", False)

        # Read ROS 2 parameters
        self.head_cam_record = self.get_parameter("head_cam").get_parameter_value().bool_value
        self.hand_cam_record = self.get_parameter("hand_cam").get_parameter_value().bool_value
        self.base_cam_record = self.get_parameter("base_cam").get_parameter_value().bool_value

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_point_cloud/charmie_point_cloud"
        self.complete_path = self.home+'/'+self.midpath+'/test_images/'

        # Intel Realsense Subscribers (RGBD) Head and Hand Cameras and Orbec Base Camera
        if self.head_cam_record:
            self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        if self.hand_cam_record:
            self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        if self.base_cam_record:
            self.color_image_base_subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.get_color_image_base_callback, 10)

        if not self.head_cam_record and not self.hand_cam_record and not self.base_cam_record:
            self.get_logger().error("No camera selected for recording. Please set the parameters 'head_cam', 'hand_cam', or 'base_cam' to True to enable recording.")
            raise SystemExit(1)
    
        self.current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        self.CAM_WIDTH = 868
        self.CAM_WIDTH_BASE = 640
        self.CAM_HEIGHT = 480

        # Define the codec and create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if self.head_cam_record:
            self.out_head = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_head_charmie.avi', fourcc, 20.0, (self.CAM_WIDTH,      self.CAM_HEIGHT))
            print("RECORDING HEAD CAMERA")
        if self.hand_cam_record:
            self.out_hand = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_hand_charmie.avi', fourcc, 20.0, (self.CAM_WIDTH,      self.CAM_HEIGHT))
            print("RECORDING HAND CAMERA")
        if self.base_cam_record:
            self.out_base = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_base_charmie.avi', fourcc, 20.0, (self.CAM_WIDTH_BASE, self.CAM_HEIGHT))
            print("RECORDING BASE CAMERA")
                             
        self.br = CvBridge()
        self.rgb_head_img = Image()
        # self.depth_img_head = Image()
        self.rgb_hand_img = Image()
        # self.depth_img_hand = Image()
        self.rgb_base_img = Image()
        # self.depth_base_img = Image()

        self.first_rgb_head_image_received = False
        # self.first_depth_head_image_received = False
        self.first_rgb_hand_image_received = False
        # self.first_depth_hand_image_received = False
        self.first_rgb_base_image_received = False
        # self.first_depth_base_image_received       = False

    def get_rgbd_head_callback(self, rgbd: RGBD):
        self.rgb_head_img = rgbd.rgb
        self.first_rgb_head_image_received = True
        # self.depth_head_img = rgbd.depth
        # self.first_depth_head_image_received = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

        current_frame_rgb_head = self.br.imgmsg_to_cv2(self.rgb_head_img, "bgr8")
        height, width, channels = current_frame_rgb_head.shape   
                
        # Make sure all images are the same size (self.CAM_HEIGHT x self.CAM_WIDTH)
        if width != self.CAM_WIDTH or height != self.CAM_HEIGHT:
            current_frame_rgb_head = cv2.resize(current_frame_rgb_head, (self.CAM_WIDTH, self.CAM_HEIGHT), interpolation = cv2.INTER_AREA)
            height, width, channels = current_frame_rgb_head.shape   
            # print("Head Colour Image: (Resized)", height, width, channels)    
        else:
            pass
            # print("Head Colour Image: ", height, width, channels)     
        
        self.out_head.write(current_frame_rgb_head)
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+'_head_charmie.avi', current_frame_rgb_head)
        self.check_end_videos()

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        self.rgb_hand_img = rgbd.rgb
        self.first_rgb_hand_image_received = True
        # self.depth_hand_img = rgbd.depth
        # self.first_depth_hand_image_received = True
        # print("HAND:", rgbd.rgb_camera_info.height, 

        current_frame_rgb_hand = self.br.imgmsg_to_cv2(self.rgb_hand_img, "bgr8")
        height, width, channels = current_frame_rgb_hand.shape   
                
        # Make sure all images are the same size (self.CAM_HEIGHT x self.CAM_WIDTH)
        if width != self.CAM_WIDTH or height != self.CAM_HEIGHT:
            current_frame_rgb_hand = cv2.resize(current_frame_rgb_hand, (self.CAM_WIDTH, self.CAM_HEIGHT), interpolation = cv2.INTER_AREA)
            height, width, channels = current_frame_rgb_hand.shape   
            # print("Hand Colour Image: (Resized)", height, width, channels)    
        else:
            # print("Hand Colour Image: ", height, width, channels)     
            pass

        self.out_hand.write(current_frame_rgb_hand)
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+'_hand_charmie.avi', current_frame_rgb_hand)
        self.check_end_videos()

    def get_color_image_base_callback(self, img: Image):
        self.rgb_base_img = img
        self.first_rgb_base_image_received = True

        current_frame_rgb_base = self.br.imgmsg_to_cv2(self.rgb_base_img, "bgr8")
        height, width, channels = current_frame_rgb_base.shape   
                
        # Make sure all images are the same size (self.CAM_HEIGHT x self.CAM_WIDTH)
        if width != self.CAM_WIDTH_BASE or height != self.CAM_HEIGHT:
            current_frame_rgb_base = cv2.resize(current_frame_rgb_base, (self.CAM_WIDTH_BASE, self.CAM_HEIGHT), interpolation = cv2.INTER_AREA)
            height, width, channels = current_frame_rgb_base.shape   
            # print("Base Colour Image: (Resized)", height, width, channels)    
        else:
            # print("Base Colour Image: ", height, width, channels)     
            pass

        self.out_base.write(current_frame_rgb_base)
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+'_base_charmie.avi', current_frame_rgb_base)
        self.check_end_videos()

    def check_end_videos(self):

        # Press 'q' to exit the video window before it ends
        if cv2.waitKey(1) & 0xFF == ord('q'):
                        
            # Release the video capture and writer objects
            if self.head_cam_record:
                self.out_head.release()
                print("Video saved as"+self.current_datetime+"_head_charmie.avi")
            if self.hand_cam_record:
                self.out_hand.release()
                print("Video saved as"+self.current_datetime+"_hand_charmie.avi")
            if self.base_cam_record:
                self.out_base.release()
                print("Video saved as"+self.current_datetime+"_base_charmie.avi")

            # Close all OpenCV windows
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SaveImagesDatasetNode()
    rclpy.spin(node)
    rclpy.shutdown()