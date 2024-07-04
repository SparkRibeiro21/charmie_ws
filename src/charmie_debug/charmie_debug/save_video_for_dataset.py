#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from pathlib import Path
import numpy as np
import cv2
import os

class PointCloudNode(Node):

    def __init__(self):
        super().__init__("PointCloud")
        self.get_logger().info("Initialised CHARMIE PointCloud Node")
        
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

        # timer that checks the images every X seconds 
        # self.create_timer(10, self.timer_callback)

        self.current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        # Define the codec and create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.home+'/'+self.current_datetime+'_charmie.avi', fourcc, 20.0, (1280, 720))
                             
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
        
        self.out.write(current_frame_rgb_head)
        
        # Display the frame (optional)
        cv2.imshow(self.current_datetime+' charmie.avi', current_frame_rgb_head)

        # Press 'q' to exit the video window before it ends
        if cv2.waitKey(1) & 0xFF == ord('q'):
                        
            # Release the video capture and writer objects
            self.out.release()

            # Close all OpenCV windows
            cv2.destroyAllWindows()

            print("Video saved as output_video.mp4")

    def get_aligned_depth_image_head_callback(self, img: Image):
        self.depth_img_head = img
        self.is_head_depth = True


    def get_color_image_hand_callback(self, img: Image):
        self.rgb_img_hand = img
        self.is_hand_rgb = True
        

    def get_aligned_depth_image_hand_callback(self, img: Image):
        self.depth_img_hand = img
        self.is_hand_depth = True


    """
    def timer_callback(self):
        self.save_images()

    def save_images(self):

        print(".")
        if self.is_head_rgb or self.is_head_depth or self.is_hand_rgb or self.is_hand_depth:
            current_datetime = str(datetime.now().strftime("%Y_%m_%d__%H_%M_%S"))
            # print("Current date & time : ", current_datetime)
            filename = current_datetime+"/"
            os.mkdir(self.complete_path + filename) 
            print("Filename:", filename)
            filename = current_datetime+"/charmie_"

            ### HEAD RGB:
            if self.is_head_rgb: # if first frame arrived 
                # ROS2 Image Bridge for OpenCV
                current_frame_rgb_head = self.br.imgmsg_to_cv2(self.rgb_img_head, "bgr8")
                height, width, channels = current_frame_rgb_head.shape   
                
                # Make sure all images are the same size (720x1280)
                if height != 720:
                    current_frame_rgb_head = cv2.resize(current_frame_rgb_head, (1280, 720), interpolation = cv2.INTER_AREA)
                    height, width, channels = current_frame_rgb_head.shape   
                    print("Head Colour Image: (Resized)", height, width, channels)    
                else:
                    print("Head Colour Image: ", height, width, channels)     
                
                # Saves data files:
                with open(self.complete_path + filename + "head_color.raw", "wb") as f:
                    f.write(current_frame_rgb_head.tobytes())

                cv2.imwrite(self.complete_path + filename + "head_color.png", current_frame_rgb_head)

                # Shows images in PC Screen:
                cv2.imshow("Head RGB (D455)", current_frame_rgb_head)

            ### HEAD DEPTH:
            if self.is_head_depth: # if first frame arrived 
                # ROS2 Image Bridge for OpenCV
                current_frame_depth_head = self.br.imgmsg_to_cv2(self.depth_img_head, desired_encoding="passthrough")
                height, width = current_frame_depth_head.shape

                # Make sure all images are the same size (720x1280)
                if height != 720:
                    current_frame_depth_head = cv2.resize(current_frame_depth_head, (1280, 720), interpolation = cv2.INTER_AREA)
                    height, width = current_frame_depth_head.shape   
                    print("Head Depth Image: (Resized)", height, width)    
                else:
                    print("Head Depth Image: ", height, width)     
                
                # Save the NumPy array to a text file
                depth_head_array = np.array(current_frame_depth_head, dtype=np.float32)
                with open(self.complete_path + filename + "head_depth.txt", 'w') as file:
                    np.savetxt(file, depth_head_array, fmt='%d', delimiter='\t')
                # np.savetxt(file_name, depth_array, fmt='%d', delimiter='\t', mode='a')

                # Conversion to improve debug visualization
                current_frame_depth_head *= 8
                cv2.imwrite(self.complete_path + filename + "head_depth.png", current_frame_depth_head)
            
                # Shows images in PC Screen:
                cv2.imshow("Head Depth (D455)", current_frame_depth_head)

            ### HAND RGB:
            if self.is_hand_rgb: # if first frame arrived 
                # ROS2 Image Bridge for OpenCV
                current_frame_rgb_hand = self.br.imgmsg_to_cv2(self.rgb_img_hand, "bgr8")
                height, width, channels = current_frame_rgb_hand.shape
                
                # Make sure all images are the same size (720x1280)
                if height != 720:
                    current_frame_rgb_hand = cv2.resize(current_frame_rgb_hand, (1280, 720), interpolation = cv2.INTER_AREA)
                    height, width, channels = current_frame_rgb_hand.shape   
                    print("Hand Colour Image: (Resized)", height, width, channels)    
                else:
                    print("Hand Colour Image: ", height, width, channels)     

                # Saves data files:
                with open(self.complete_path + filename + "hand_color.raw", "wb") as f:
                    f.write(current_frame_rgb_hand.tobytes())

                cv2.imwrite(self.complete_path + filename + "hand_color.png", current_frame_rgb_hand)

                # Shows images in PC Screen:
                cv2.imshow("Hand RGB (D405)", current_frame_rgb_hand)

            ### HAND DEPTH:
            if self.is_hand_depth: # if first frame arrived 
                # ROS2 Image Bridge for OpenCV
                current_frame_depth_hand = self.br.imgmsg_to_cv2(self.depth_img_hand, desired_encoding="passthrough")
                height, width = current_frame_depth_hand.shape

                # Make sure all images are the same size (720x1280)
                if height != 720:
                    current_frame_depth_hand = cv2.resize(current_frame_depth_hand, (1280, 720), interpolation = cv2.INTER_AREA)
                    height, width = current_frame_depth_hand.shape   
                    print("Hand Depth Image: (Resized)", height, width)    
                else:
                    print("Hand Depth Image: ", height, width)
                          
                # Save the NumPy array to a text file
                depth_hand_array = np.array(current_frame_depth_hand, dtype=np.float32)
                with open(self.complete_path + filename + "hand_depth.txt", 'w') as file:
                    np.savetxt(file, depth_hand_array, fmt='%d', delimiter='\t')
                # np.savetxt(file_name, depth_array, fmt='%d', delimiter='\t', mode='a')

                # Conversion to improve debug visualization
                current_frame_depth_hand *= 8
                cv2.imwrite(self.complete_path + filename + "hand_depth.png", current_frame_depth_hand)
            
                # Shows images in PC Screen:
                cv2.imshow("Hand Depth (D405)", current_frame_depth_hand)

            cv2.waitKey(10)
    """


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()