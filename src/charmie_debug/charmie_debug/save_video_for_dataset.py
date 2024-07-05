#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from pathlib import Path
import cv2

class SaveImagesDatasetNode(Node):

    def __init__(self):
        super().__init__("SaveImagesDataset")
        self.get_logger().info("Initialised CHARMIE SaveImagesDataset Node")
        
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_point_cloud/charmie_point_cloud"
        self.complete_path = self.home+'/'+self.midpath+'/test_images/'

        # Intel Realsense Subscribers
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        # self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        # self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)

        self.current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        # Define the codec and create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out_head = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_head_charmie.avi', fourcc, 20.0, (1280, 720))
        self.out_hand = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_hand_charmie.avi', fourcc, 20.0, (1280, 720))
                             
        self.br = CvBridge()
        self.rgb_img_head = Image()
        # self.depth_img_head = Image()
        self.rgb_img_hand = Image()
        # self.depth_img_hand = Image()

        self.is_head_rgb = False
        # self.is_head_depth = False
        self.is_hand_rgb = False
        # self.is_hand_depth = False


    def get_color_image_head_callback(self, img: Image):
        self.rgb_img_head = img
        self.is_head_rgb = True
        
        current_frame_rgb_head = self.br.imgmsg_to_cv2(self.rgb_img_head, "bgr8")
        height, width, channels = current_frame_rgb_head.shape   
                
        # Make sure all images are the same size (720x1280)
        if height != 720:
            current_frame_rgb_head = cv2.resize(current_frame_rgb_head, (1280, 720), interpolation = cv2.INTER_AREA)
            height, width, channels = current_frame_rgb_head.shape   
            print("Head Colour Image: (Resized)", height, width, channels)    
        else:
            print("Head Colour Image: ", height, width, channels)     
        
        self.out_head.write(current_frame_rgb_head)
        
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+'_head_charmie.avi', current_frame_rgb_head)

        self.check_end_videos()

    def get_color_image_hand_callback(self, img: Image):
        self.rgb_img_hand = img
        self.is_hand_rgb = True
        
        current_frame_rgb_hand = self.br.imgmsg_to_cv2(self.rgb_img_hand, "bgr8")
        height, width, channels = current_frame_rgb_hand.shape   
                
        # Make sure all images are the same size (720x1280)
        if height != 720:
            current_frame_rgb_hand = cv2.resize(current_frame_rgb_hand, (1280, 720), interpolation = cv2.INTER_AREA)
            height, width, channels = current_frame_rgb_hand.shape   
            print("Hand Colour Image: (Resized)", height, width, channels)    
        else:
            print("Hand Colour Image: ", height, width, channels)     
        
        self.out_hand.write(current_frame_rgb_hand)
        
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+'_hand_charmie.avi', current_frame_rgb_hand)

        self.check_end_videos()

    def check_end_videos(self):

        # Press 'q' to exit the video window before it ends
        if cv2.waitKey(1) & 0xFF == ord('q'):
                        
            # Release the video capture and writer objects
            self.out_head.release()
            self.out_hand.release()

            # Close all OpenCV windows
            cv2.destroyAllWindows()

            print("Video saved as"+self.current_datetime+"_head_charmie.avi")
            print("Video saved as"+self.current_datetime+"_hand_charmie.avi")

    # def get_aligned_depth_image_head_callback(self, img: Image):
    #     self.depth_img_head = img
    #     self.is_head_depth = True
        
    # def get_aligned_depth_image_hand_callback(self, img: Image):
    #     self.depth_img_hand = img
    #     self.is_hand_depth = True

def main(args=None):
    rclpy.init(args=args)
    node = SaveImagesDatasetNode()
    rclpy.spin(node)
    rclpy.shutdown()