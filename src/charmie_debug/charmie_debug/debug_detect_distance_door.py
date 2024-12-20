#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Bool, Float32, Int16, String 
from geometry_msgs.msg import Point, Pose2D
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject, ListOfPoints, NeckPosition, ListOfFloats, BoundingBoxAndPoints, BoundingBox, TarNavSDNL, ArmController
from charmie_interfaces.srv import TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, GetPointCloudBB
from xarm_msgs.srv import GetFloat32List, PlanPose, PlanExec, PlanJoint
from sensor_msgs.msg import Image

import cv2 
from array import array
import threading
import time
from cv_bridge import CvBridge
import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import open3d as o3d

from pathlib import Path
from datetime import datetime

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

class TestNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Test Speakers and Face Node")

        # path to save detected people in search for person
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path_custom_face = home+'/'+midpath+'/'

        # Intel Realsense Subscribers
        # self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_hand_image_callback, 10)
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_callback, 10)
        ### Topics (Publisher and Subscribers) ###  
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered_hand", self.object_detected_filtered_hand_callback, 10)
        self.doors_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "doors_detected_filtered", self.doors_detected_filtered_callback, 10)
        self.doors_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, "doors_detected_filtered_hand", self.doors_detected_filtered_hand_callback, 10)

        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)
        
        # Search for Person debug publisher
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)

        # Low level
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.torso_test_publisher = self.create_publisher(Pose2D, "torso_move" , 10)

        # Navigation
        self.target_pos_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)          

        # Arm 
        self.arm_command_publisher = self.create_publisher(ArmController, "arm_command", 10)
        self.arm_value_publisher = self.create_publisher(Float32, 'arm_value', 10)
        self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)
        self.arm_pose_subscriber = self.create_subscription(ListOfFloats, 'arm_current_pose', self.get_arm_current_pose_callback, 10)
        self.arm_set_pose_publisher = self.create_publisher(ListOfFloats, 'arm_set_desired_pose', 10)
        self.arm_set_height_publisher = self.create_publisher(Float32, 'arm_set_desired_height', 10)

        # Point cloud
        self.point_cloud_client = self.create_client(GetPointCloudBB, "get_point_cloud_bb")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")


        ### Services (Clients) ###
        
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")

    
        self.plan_pose_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')

        # self.objects_filtered_hand_publisher = self.create_subscription(Yolov8Objects, 'objects_detected_filtered_hand', self.hand_objects_callback, 10)
        
        # Neck 
        # while not self.set_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # Yolos
        # while not self.activate_yolo_pose_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")

        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")
        # while not self.neck_track_person_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Neck Track Person ...")
        
        # Variables
        self.waited_for_end_of_track_person = False
        self.waited_for_end_of_track_object = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_arm = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.rgb_success = True
        self.rgb_message = ""
        self.face_success = True
        self.face_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        self.track_object_success = True
        self.track_object_message = ""
        self.arm_success = True
        self.arm_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.flag_navigation_reached = False
        self.navigation_success = True
        self.navigation_message = ""

        self.br = CvBridge()
        self.depth_img = Image()
        self.depth_img_hand = Image()
        self.first_depth_image_received = False
        self.first_depth_image_hand_received = False
        self.new_image_hand_flag = False
        self.detected_people = Yolov8Pose()
        self.detected_objects_hand = Yolov8Objects()
        self.detected_objects = Yolov8Objects()
        self.detected_doors = Yolov8Objects()
        self.detected_doors_hand= Yolov8Objects()
        self.waiting_for_pcloud = False
        self.point_cloud_response = GetPointCloudBB.Response()

        self.wardrobe_width = 0.9
        self.door_width = self.wardrobe_width // 2
        self.third_shelf_height = 0.95
        self.house_door_width = 0.9


    def get_arm_current_pose_callback(self, arm_pose: ListOfFloats):
        self.arm_current_pose = arm_pose.pose

    def call_point_cloud_server(self, req, camera):
        request = GetPointCloudBB.Request()
        request.data = req
        request.retrieve_bbox = False
        request.camera = camera    
        future = self.point_cloud_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud)

    ### NAVIGATION ###
    def flag_navigation_reached_callback(self, flag: Bool):
        self.flag_navigation_reached = flag

    def callback_call_point_cloud(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_response = future.result()
            # self.post_receiving_pcloud(response.coords)
            self.waiting_for_pcloud = False
            # print("Received Back")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def distance(p1, p2):
            return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    
    def get_color_image_callback(self, img: Image):

        self.rgb_img = img

        current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        current_frame_draw = current_frame.copy()

        self.rgb_img_2 = current_frame

        """ 
        
        TENTATIVA FALHADA DE COM IMAGEM DE CORES SACAR PLANOS

        # Convert the image to grayscale
        gray = cv2.cvtColor(current_frame_draw, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform edge detection
        edges = cv2.Canny(blurred, 50, 150)

        edge_image = np.zeros_like(current_frame_draw)

        # Draw edges in r_d on the blank image
        edge_image[:, :, 2] = edges

        cv2.imshow('Detected Edges', edge_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)
        # Create a blank image of the same size as the original image
        line_image = np.zeros_like(current_frame_draw)

        print("Number of lines detected:", len(lines))


        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Calculate slope
            slope = (y2 - y1) / (x2 - x1 + 1e-5)
            # Filter horizontal lines (slope close to 0)
            if abs(slope) < 0.2:
                # Draw lines on the blank image
                cv2.line(current_frame_draw, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Display the detected horizontal lines
        cv2.imshow('Horizontal Lines', current_frame_draw)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Find contours in the edge-detected image
        # contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(gray)

        print("Mask shape:", mask.shape)


        # Apply binary threshold to the mask
        binary_mask = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        # Print the minimum and maximum values in the mask array
        print("Minimum value in mask:", np.min(binary_mask))
        print("Maximum value in mask:", np.max(binary_mask))

        # Display the binary mask
        cv2.imshow('Binary Mask', binary_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Find contours in the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through detected contours
        for contour in contours:
            print('aaaaaa')
            # Approximate the contour to a polygon
            area = cv2.contourArea(contour)
            
            # If the contour has 4 vertices (possible rectangle)
            if area > 10:
                # Draw a green rectangle around the detected contour
                cv2.drawContours(current_frame_draw, [contour], -1, (0, 255, 0), 2)

        # Display the result
        cv2.imshow('Plane Detection', current_frame_draw)
        cv2.waitKey(0)
        cv2.destroyAllWindows() """

                
    def get_aligned_depth_image_callback(self, img: Image):

        self.depth_img = img
        self.first_depth_image_received = True

        


        """ 
        CÓDIGO QUE TENTA DETETAR PLANOS (QUASE QUASE CONSIGO)

        self.depth_img = img
        self.first_depth_image_received = True

        current_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        current_frame_draw = current_frame.copy()

        depth_image_filtered = cv2.medianBlur(current_frame_draw, 5)

        # Normalize the depth image for visualization
        depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Apply a median filter to reduce noise
        

        # # Debug: Display the raw depth image
        # plt.figure(figsize=(10, 8))
        # plt.imshow(current_frame_draw, cmap='gray')
        # plt.title('Raw Depth Image')
        # plt.axis('off')
        # plt.show()

        # # Debug: Display the filtered depth image
        # plt.figure(figsize=(10, 8))
        # plt.imshow(depth_image_filtered, cmap='gray')
        # plt.title('Filtered Depth Image')
        # plt.axis('off')
        # plt.show()

        # Step 2: Use RANSAC to Fit Planes
        # Prepare data for RANSAC
        height, width = depth_image_normalized.shape
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))
        X = np.vstack((xx.flatten(), yy.flatten())).T
        Z = depth_image_normalized.flatten()

        print('c')

        # Debug: Display the distribution of depth values
        plt.figure(figsize=(10, 8))
        plt.hist(Z, bins=50, color='blue', alpha=0.7)
        plt.title('Histogram of Depth Values')
        plt.xlabel('Depth Value')
        plt.ylabel('Frequency')
        plt.show()

        # Apply RANSAC to fit planes
        ransac = RANSACRegressor(residual_threshold=10, max_trials=1000)
        ransac.fit(X, Z)

        print('d')

        # Get the fitted plane parameters
        plane_coef = ransac.estimator_.coef_
        plane_intercept = ransac.estimator_.intercept_

        print(f"Plane Coefficients: {plane_coef}")
        print(f"Plane Intercept: {plane_intercept}")

        if np.all(plane_coef == 0) and plane_intercept == 0:
            print("RANSAC failed to fit a meaningful plane. Try adjusting the parameters or preprocessing steps.")
            return

        # Visualize the fitted plane by drawing it on the depth image
        a, b = plane_coef[0], plane_coef[1]
        c = plane_intercept
        zz = a * xx + b * yy + c

        print(zz)
        print(depth_image_normalized)

        # Create a mask for the plane region
        mask = ((depth_image_normalized - zz) < 30) & ((depth_image_normalized - zz) > 0)
        mask = np.abs(depth_image_normalized - zz) < 10

        # Debugging: Visualize the mask
        plt.figure(figsize=(10, 8))
        plt.imshow(mask, cmap='gray')
        plt.title('Plane Mask')
        plt.axis('off')
        plt.show()
        

        # Step 3: Highlight the Detected Planes
        # Convert depth image to a colored image
        colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # Paint the plane pixels white in the colored image
        colored_depth_image[mask] = [0, 255, 0]
        print(mask)

        # Display the colored image with the highlighted plane
        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(colored_depth_image, cv2.COLOR_BGR2RGB))
        plt.title('Depth Image with Detected Plane')
        plt.axis('off')  # Turn off axis
        plt.show() """

        
        """ 
        CÓDIGO QUE COLOCA LINHAS EM ZONAS QUE ESTÃO ALINHADAS!
        
        self.depth_img = img
        self.first_depth_image_received = True

        current_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        current_frame_draw = current_frame.copy()
        # cv2.imshow("Aligned Depth Head", current_frame_draw)
        # cv2.waitKey(10)

        depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Apply a median filter to reduce noise
        depth_image_filtered = cv2.medianBlur(depth_image_normalized, 5)

        # Apply Canny edge detection
        edges = cv2.Canny(depth_image_filtered, 25, 50)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)

        # Plot the detected lines on top of the edges
        plt.figure(figsize=(8, 6))

        # Plot the edges
        plt.imshow(edges, cmap='gray')

        # Plot the detected lines
        if lines is not None:
            for rho, theta in lines[:, 0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                plt.plot([x1, x2], [y1, y2], color='blue')

        plt.title('Edges with Detected Lines')
        plt.axis('off')  # Turn off axis
        plt.show() """

        """ 
        Tentativa falhada de detetar planos
        
        self.depth_img = img
        self.first_depth_image_received = True

        current_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        current_frame_draw = current_frame.copy()
        # cv2.imshow("Aligned Depth Head", current_frame_draw)
        # cv2.waitKey(10)

        depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Apply a median filter to reduce noise
        depth_image_filtered = cv2.medianBlur(depth_image_normalized, 5)

        # Apply Canny edge detection
        edges = cv2.Canny(depth_image_filtered, 25, 50)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)

        # Plot the detected lines on top of the edges
        plt.figure(figsize=(8, 6))

        # Plot the edges
        plt.imshow(edges, cmap='gray')

        # Plot the detected lines
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                
                # Convert theta to degrees
                theta_degrees = np.degrees(theta)
                
                # Exclude lines with theta angle greater than 30 degrees (π/6 radians)
                if abs(theta_degrees) <= 160 and abs(theta_degrees) >= 30:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    # Ensure endpoints of the line are within image boundaries
                    x1 = max(0, min(x1, edges.shape[1]-1))
                    y1 = max(0, min(y1, edges.shape[0]-1))
                    x2 = max(0, min(x2, edges.shape[1]-1))
                    y2 = max(0, min(y2, edges.shape[0]-1))

                    plt.plot([x1, x2], [y1, y2], color='blue')

        plt.title('Edges with Detected Lines')
        plt.axis('off')  # Turn off axis
        plt.show()

        
        current_frame_uint8 = cv2.convertScaleAbs(current_frame_draw)
        depth_colored = cv2.applyColorMap(current_frame_uint8, cv2.COLORMAP_JET)

        # Iterate over adjacent lines
        for i in range(len(lines) - 1):
            # Get endpoints of current and next lines
            rho1, theta1 = lines[i][0]
            rho2, theta2 = lines[i+1][0]
            a1, b1 = np.cos(theta1), np.sin(theta1)
            a2, b2 = np.cos(theta2), np.sin(theta2)
            x01, y01 = a1 * rho1, b1 * rho1
            x02, y02 = a2 * rho2, b2 * rho2
            p1 = (int(x01), int(y01))
            p2 = (int(x02), int(y02))

            # Check if the lines are approximately horizontal
            if abs(theta1 - np.pi / 2) < np.pi / 6 and abs(theta2 - np.pi / 2) < np.pi / 6:
                # Define ROI between the lines
                roi = edges[max(p1[1], p2[1]): min(p1[1], p2[1]), :]

                # Extract depth values from the ROI
                depth_values = current_frame_draw[max(p1[1], p2[1]): min(p1[1], p2[1]), :].flatten()

                # Analyze the consistency of depth values within the ROI
                # You can use statistical measures such as variance to determine if the region is planar
                # For simplicity, let's assume it's planar if the variance is below a threshold
                variance_threshold = 100  # Adjust this threshold as needed
                print(np.var(depth_values))
                if np.var(depth_values) < variance_threshold:
                    print('bbb')
                    # If the depth values within the ROI exhibit characteristics of a planar surface,
                    # consider it as part of a plane

                    # Visualize the planes in the depth image by drawing a rectangle
                    cv2.rectangle(depth_colored, (0, max(p1[1], p2[1])), (depth_colored.shape[1], min(p1[1], p2[1])), (255, 255, 255), -1)

        # Display the depth image with highlighted planes
        cv2.imshow('Depth Image with Highlighted Planes', depth_colored)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        while True:
            pass """


        # print('b')

        # # Normalize the depth image for better visualization
        # depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        # depth_image_normalized = np.uint8(depth_image_normalized)

        # print('c')

        # # Apply a median filter to reduce noise
        # depth_image_filtered = cv2.medianBlur(depth_image_normalized, 5)

        # print('d')


        # Apply Canny edge detection
        # You can adjust these thresholds to better suit your specific depth image
        """ lower_threshold = 50
        upper_threshold = 150
        edges = cv2.Canny(depth_image_filtered, lower_threshold, upper_threshold)

        # Calculate gradient magnitude
        sobelx = cv2.Sobel(depth_image_filtered, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_image_filtered, cv2.CV_64F, 0, 1, ksize=3)
        gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)

        # Normalize gradient magnitude for visualization
        gradient_magnitude_normalized = cv2.normalize(gradient_magnitude, None, 0, 255, cv2.NORM_MINMAX)
        gradient_magnitude_normalized = np.uint8(gradient_magnitude_normalized)

        # Display the gradient magnitude image
        plt.imshow(gradient_magnitude_normalized, cmap='gray')
        plt.title('Gradient Magnitude')
        plt.colorbar()
        plt.show() """

        # threshold_1 = 25
        # threshold_2 = 50

        # # Create binary edge map based on the thresholds
        # edges_of_interest = np.zeros_like(depth_image_normalized)
        # edges_of_interest[(depth_image_normalized >= threshold_1) & (depth_image_normalized <= threshold_2)] = 255

        # # Display the binary edge map
        # plt.imshow(edges_of_interest, cmap='gray')
        # plt.title('Edges of Interest')
        # plt.colorbar()
        # plt.show()





        """ 
        CÓDIGO QUE MOSTRA OS LIMITES QUANDO ROBÔ ESTÁ EM FRENTE A MÁQUINA DE LAVAR

        self.depth_img = img
        self.first_depth_image_received = True

        current_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        current_frame_draw = current_frame.copy()
        # cv2.imshow("Aligned Depth Head", current_frame_draw)
        # cv2.waitKey(10)

        # Normalize the depth image for better visualization
        depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Apply a median filter to reduce noise
        depth_image_filtered = cv2.medianBlur(depth_image_normalized, 5)
        # Display the normalized and filtered depth image
        plt.imshow(depth_image_filtered, cmap='gray')
        plt.title('Normalized and Filtered Depth Image')
        plt.colorbar()
        plt.show()

        # Step 2: Apply edge detection
        edges = cv2.Canny(depth_image_filtered, 25, 50)

        # Display the edges
        plt.imshow(edges, cmap='gray')
        plt.title('Edges Detected in Depth Image')
        plt.colorbar()
        plt.show()

        # Step 3: Overlay edges on the colored depth image
        colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # Overlay edges on the colored depth image
        colored_depth_with_edges = colored_depth_image.copy()
        colored_depth_with_edges[edges > 0] = [0, 0, 255]  # Red edges

        # Display the colored depth image with edges
        plt.imshow(cv2.cvtColor(colored_depth_with_edges, cv2.COLOR_BGR2RGB))
        plt.title('Colored Depth Image with Edges')
        plt.show() """



        # # Apply Canny edge detection
        # edges = cv2.Canny(depth_image_filtered, 25, 50)

        # print('e')

        # # Prepare data for RANSAC
        # points = np.column_stack(np.nonzero(depth_image_filtered))

        # print('f')

        # # Define the RANSAC model
        # model_robust, inliers = ransac(points, LineModelND, min_samples=2, residual_threshold=1, max_trials=1000)

        # print('g')

        # # Extract inlier points
        # inlier_points = points[inliers]
        # print("Number of inlier points:", len(inlier_points))

        # # Plot the edges
        # plt.figure(figsize=(10, 5))
        # plt.subplot(1, 2, 1)
        # plt.imshow(edges, cmap='gray')
        # plt.title('Edges in Depth Image')
        # plt.show()

        

        # # Plot the detected lines
        # plt.subplot(1, 2, 2)
        # plt.imshow(depth_image_filtered, cmap='gray')
        # plt.plot(inlier_points[:, 1], inlier_points[:, 0], 'r.', markersize=2)
        # for line in model_robust:
        #     p0, p1 = line
        #     print("Line:", p0, p1)
        #     plt.plot([p0[1], p1[1]], [p0[0], p1[0]], color='blue')
        # plt.title('Detected Lines')
        

        # plt.show()
        
        
        """ self.depth_img = img
        self.first_depth_image_received = True

        current_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
        current_frame_draw = current_frame.copy()
        # cv2.imshow("Aligned Depth Head", current_frame_draw)
        # cv2.waitKey(10)

        depth_image_normalized = cv2.normalize(current_frame_draw, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_normalized = np.uint8(depth_image_normalized)

        # Apply a colormap (e.g., COLORMAP_JET) to the normalized depth image
        colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

        # Display the image using OpenCV
        cv2.imshow('Colored Depth Image', colored_depth_image)
        cv2.waitKey(10)
        cv2.destroyAllWindows()

        # Display the depth image using Matplotlib
        plt.imshow(current_frame_draw, cmap='jet')
        plt.colorbar()
        plt.title('Depth Image')
        plt.show()
         """



        
        
        # print("Received Depth Image")

    def get_aligned_depth_hand_image_callback(self, img: Image):
        self.depth_img_hand = img
        self.first_depth_image_hand_received = True
        self.new_image_hand_flag = True
        # print("Received Depth Image")


    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection", current_frame_draw)
        # cv2.waitKey(10)

    def doors_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_doors = det_object

    def doors_detected_filtered_hand_callback(self, det_object_hand: Yolov8Objects):
        self.detected_doors_hand = det_object_hand

    def object_detected_filtered_hand_callback(self, det_object_hand: Yolov8Objects):
        self.detected_objects_hand = det_object_hand


    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object

        """
        current_frame = self.br.imgmsg_to_cv2(self.detected_objects.image_rgb, "bgr8")
        current_frame_draw = current_frame.copy()


        # img = [0:720, 0:1280]
        corr_image = False
        thresh_h = 50
        thresh_v = 200

        if self.detected_objects.num_objects > 0:

            x_min = 1280
            x_max = 0
            y_min = 720
            y_max = 0

            for object in self.detected_objects.objects:      
            
                if object.object_class == "Dishes":
                    corr_image = True

                    if object.box_top_left_x < x_min:
                        x_min = object.box_top_left_x
                    if object.box_top_left_x+object.box_width > x_max:
                        x_max = object.box_top_left_x+object.box_width

                    if object.box_top_left_y < y_min:
                        y_min = object.box_top_left_y
                    if object.box_top_left_y+object.box_height > y_max:
                        y_max = object.box_top_left_y+object.box_height

                    start_point = (object.box_top_left_x, object.box_top_left_y)
                    end_point = (object.box_top_left_x+object.box_width, object.box_top_left_y+object.box_height)
                    cv2.rectangle(current_frame_draw, start_point, end_point, (255,255,255) , 4) 

                    cv2.circle(current_frame_draw, (object.box_center_x, object.box_center_y), 5, (255, 255, 255), -1)
                    
            
            for object in self.detected_objects.objects:      
                
                if object.object_class == "Dishes":
                
                    if object.box_top_left_y < 30: # depending on the height of the box, so it is either inside or outside
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y+25)
                    else:
                        start_point_text = (object.box_top_left_x-2, object.box_top_left_y-22)
                        
                    # just to test for the "serve the breakfast" task...
                    aux_name = object.object_name
                    if object.object_name == "Fork" or object.object_name == "Knife":
                        aux_name = "Spoon"
                    elif object.object_name == "Plate" or object.object_name == "Cup":
                        aux_name = "Bowl"

                    text_size, _ = cv2.getTextSize(f"{aux_name}", cv2.FONT_HERSHEY_DUPLEX, 1, 1)
                    text_w, text_h = text_size
                    cv2.rectangle(current_frame_draw, (start_point_text[0], start_point_text[1]), (start_point_text[0] + text_w, start_point_text[1] + text_h), (255,255,255), -1)
                    cv2.putText(current_frame_draw, f"{aux_name}", (start_point_text[0], start_point_text[1]+text_h+1-1), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        if corr_image:
            # current_frame_draw = current_frame_draw[x_min:y_min, x_max,y_max]
            # img = current_frame_draw[y_min:y_max, x_min,x_max]
            cv2.imshow("c", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)])
            cv2.waitKey(1)
        # cv2.imshow("Yolo Objects TR Detection", current_frame_draw)
        # cv2.waitKey(10)

        # cv2.imwrite("object_detected_test4.jpg", current_frame_draw[max(y_min-thresh_v,0):min(y_max+thresh_v,720), max(x_min-thresh_h,0):min(x_max+thresh_h,1280)]) 
        # cv2.waitKey(10)
        """


    def arm_finished_movement_callback(self, flag: Bool):
        # self.get_logger().info("Received response from arm finishing movement")
        self.arm_ready = True
        self.waited_for_end_of_arm = True
        self.arm_success = flag.data
        if flag.data:
            self.arm_message = "Arm successfully moved"
        else:
            self.arm_message = "Wrong Movement Received"

        self.get_logger().info("Received Arm Finished")

    #### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, position=[0, 0], wait_for_end_of=True):
        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])
        
        future = self.set_neck_position_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    #### NECK SERVER FUNCTIONS #####
    def call_neck_track_person_server(self, person, body_part="Head", wait_for_end_of=True):
        request = TrackPerson.Request()
        request.person = person
        request.body_part = body_part

        future = self.neck_track_person_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_person)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_person(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_person_success = response.success
            self.track_person_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_person = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_neck_track_object_server(self, object, wait_for_end_of=True):
        request = TrackObject.Request()
        request.object = object

        future = self.neck_track_object_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_object)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_object(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_object_success = response.success
            self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_object = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    #### SET NECK COORDINATES SERVER FUNCTIONS #####
    def call_neck_coordinates_server(self, x, y, z, tilt, flag, wait_for_end_of=True):
        request = SetNeckCoordinates.Request()
        request.coords.x = float(x)
        request.coords.y = float(y)
        request.coords.z = float(z)
        request.is_tilt = flag
        request.tilt = float(tilt)
        
        future = self.set_neck_coordinates_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_coords_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_coords_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_coords = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### GET NECK POSITION SERVER FUNCTIONS #####
    def call_get_neck_position_server(self):
        request = GetNeckPosition.Request()
        
        future = self.get_neck_position_client.call_async(request)
        # print("Sent Command")

        # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
        future.add_done_callback(self.callback_call_get_neck_command)
    
    def callback_call_get_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info("Received Neck Position: (%s" %(str(response.pan) + ", " + str(response.tilt)+")"))
            self.get_neck_position[0] = response.pan
            self.get_neck_position[1] = response.tilt
            # time.sleep(3)
            self.waited_for_end_of_get_neck = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    def call_neck_track_object_server(self, object, wait_for_end_of=True):
        request = TrackObject.Request()
        request.object = object

        future = self.neck_track_object_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_object)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_object(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_object_success = response.success
            self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_object = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



    ### ACTIVATE YOLO POSE SERVER FUNCTIONS ###
    def call_activate_yolo_pose_server(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False):
        request = ActivateYoloPose.Request()
        request.activate = activate
        request.only_detect_person_legs_visible = only_detect_person_legs_visible
        request.minimum_person_confidence = minimum_person_confidence
        request.minimum_keypoints_to_detect_person = minimum_keypoints_to_detect_person
        request.only_detect_person_arm_raised = only_detect_person_arm_raised
        request.only_detect_person_right_in_front = only_detect_person_right_in_front
        request.characteristics = characteristics

        self.activate_yolo_pose_client.call_async(request)

    ### ACTIVATE YOLO OBJECTS SERVER FUNCTIONS ###
    def call_activate_yolo_objects_server(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5):
        request = ActivateYoloObjects.Request()
        request.activate_objects = activate_objects
        request.activate_shoes = activate_shoes
        request.activate_doors = activate_doors
        request.activate_objects_hand = activate_objects_hand
        request.activate_shoes_hand = activate_shoes_hand
        request.activate_doors_hand = activate_doors_hand
        request.minimum_objects_confidence = minimum_objects_confidence
        request.minimum_shoes_confidence = minimum_shoes_confidence
        request.minimum_doors_confidence = minimum_doors_confidence

        self.activate_yolo_objects_client.call_async(request)

    def pose_planner(self, obj):
        #This routine was an introduction on how to use the planner of the arm to plan movements
        request = PlanPose.Request()
        response = PlanPose.Response()
        """ request.target.position.x = obj[0] * 0.001 #para ficar em mm
        request.target.position.y = obj[1] * 0.001 #para ficar em mm
        request.target.position.z = obj[2] * 0.001 #para ficar em mm
        quaternions = self.euler_to_quaternion(obj[3], obj[4], obj[5])
        request.target.orientation.x = quaternion[0]
        request.target.orientation.y = quaternion[1]
        request.target.orientation.z = quaternion[2]
        request.target.orientation.w = quaternion[3] """
        request.target.position.x = -0.5210
        request.target.position.y = -0.2200
        request.target.position.z = 0.4800
        request.target.orientation.x = 0.276
        request.target.orientation.y = -0.242
        request.target.orientation.z = -0.646
        request.target.orientation.w = 0.669

        future = self.plan_pose_client.call_async(request)
        time.sleep(2)
        print(future.result())

        response = future.result()

        # print(response.success)

        return response.success

    def pose_exec(self):
        request = PlanExec.Request()
        request.wait = True
        self.exec_plan_client.call_async(request)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cz = math.cos(yaw * 0.5)
        sz = math.sin(yaw * 0.5)
        cy = math.cos(pitch * 0.5)
        sy = math.sin(pitch * 0.5)
        cx = math.cos(roll * 0.5)
        sx = math.sin(roll * 0.5)

        w = cz * cy * cx + sz * sy * sx
        x = cz * cy * sx - sz * sy * cx
        y = cz * sy * cx + sz * cy * sx
        z = sz * cy * cx - cz * sy * sx

        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_restaurant(node: TestNode):
    main = RestaurantMain(node)
    main.main()

class RestaurantMain():

    def __init__(self, node: TestNode):
        self.node = node
        
        # VARS ...
        self.state = 0
        self.look_right = [-60, -30]
        self.look_left = [60, -30]
        self.look_down = [0, -50]
        self.look_navigation= [0, -30]
        self.look_forward = [0, 0]
    
    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message

    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        if custom == "":
            temp = String()
            temp.data = command
            self.node.image_to_face_publisher.publish(temp)
        else:
            temp = String()
            temp.data = custom
            self.node.custom_image_to_face_publisher.publish(temp)

        self.node.face_success = True
        self.node.face_message = "Value Sucessfully Sent"

        return self.node.face_success, self.node.face_message
    
    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message
    
    def set_navigation(self, movement="", target=[0.0, 0.0], absolute_angle=0.0, flag_not_obs=False, reached_radius=0.6, adjust_distance=0.0, adjust_direction=0.0, adjust_min_dist=0.0, wait_for_end_of=True):

        if movement.lower() != "move" and movement.lower() != "rotate" and movement.lower() != "orientate" and movement.lower() != "adjust" and movement.lower() != "adjust_obstacle" :   
            self.node.get_logger().error("WRONG MOVEMENT NAME: PLEASE USE: MOVE, ROTATE OR ORIENTATE.")

            self.navigation_success = False
            self.navigation_message = "Wrong Movement Name"

        else:
            
            navigation = TarNavSDNL()

            # Pose2D target_coordinates
            # string move_or_rotate
            # float32 orientation_absolute
            # bool flag_not_obs
            # float32 reached_radius
            # bool avoid_people
            # float32 adjust_distance
            # float32 adjust_direction
            # float32 adjust_min_dist

            if adjust_direction < 0:
                adjust_direction += 360

            navigation.target_coordinates.x = target[0]
            navigation.target_coordinates.y = target[1]
            navigation.move_or_rotate = movement
            navigation.orientation_absolute = absolute_angle
            navigation.flag_not_obs = flag_not_obs
            navigation.reached_radius = reached_radius
            navigation.avoid_people = False
            navigation.adjust_distance = adjust_distance
            navigation.adjust_direction = adjust_direction
            navigation.adjust_min_dist = adjust_min_dist

            self.node.flag_navigation_reached = False
            
            self.node.target_pos_publisher.publish(navigation)

            if wait_for_end_of:
                while not self.node.flag_navigation_reached:
                    pass
                self.node.flag_navigation_reached = False

            self.navigation_success = True
            self.navigation_message = "Arrived at selected location"

        return self.node.navigation_success, self.node.navigation_message   
 

    def set_arm(self, command="", pose=[], adjust_position=0.0, wait_for_end_of=True):
        
        # this prevents some previous unwanted value that may be in the wait_for_end_of_ variable 
        self.node.waited_for_end_of_arm = False
        
        temp = ArmController()
        temp.command = command
        temp.adjust_position = adjust_position
        temp.pose = pose
        self.node.arm_command_publisher.publish(temp)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_arm:
                pass
            self.node.waited_for_end_of_arm = False
        else:
            self.node.arm_success = True
            self.node.arm_message = "Wait for answer not needed"

        # self.node.get_logger().info("Set Arm Response: %s" %(str(self.arm_success) + " - " + str(self.arm_message)))
        return self.node.arm_success, self.node.arm_message
    
    
    def set_neck_coords(self, position=[], ang=0.0, wait_for_end_of=True):

        if len(position) == 2:
            self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=0.0, tilt=ang, flag=True, wait_for_end_of=wait_for_end_of)
        elif len(position) == 3:
            print("You tried neck to coordintes using (x,y,z) please switch to (x,y,theta)")
            pass
            # The following line is correct, however since the functionality is not implemented yet, should not be called
            # self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=position[2], tilt=0.0, flag=False, wait_for_end_of=wait_for_end_of)
        else:
            print("Something went wrong")
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_coords:
            pass
        self.node.waited_for_end_of_neck_coords = False

        return self.node.neck_success, self.node.neck_message
    
    def get_neck(self, wait_for_end_of=True):
    
        self.node.call_get_neck_position_server()
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_get_neck:
            pass
        self.node.waited_for_end_of_get_neck = False


        return self.node.get_neck_position[0], self.node.get_neck_position[1] 
    
    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def activate_yolo_objects(self, activate_objects=False, activate_shoes=False, activate_doors=False, activate_objects_hand=False, activate_shoes_hand=False, activate_doors_hand=False, minimum_objects_confidence=0.5, minimum_shoes_confidence=0.5, minimum_doors_confidence=0.5, wait_for_end_of=True):
        
        self.node.call_activate_yolo_objects_server(activate_objects=activate_objects, activate_shoes=activate_shoes, activate_doors=activate_doors, activate_objects_hand=activate_objects_hand, activate_shoes_hand=activate_shoes_hand, activate_doors_hand=activate_doors_hand, minimum_objects_confidence=minimum_objects_confidence, minimum_shoes_confidence=minimum_shoes_confidence, minimum_doors_confidence=minimum_doors_confidence)

        self.node.activate_yolo_objects_success = True
        self.node.activate_yolo_objects_message = "Activated with selected parameters"

        return self.node.activate_yolo_objects_success, self.node.activate_yolo_objects_message

    def track_person(self, person, body_part="Head", wait_for_end_of=True):

        self.node.call_neck_track_person_server(person=person, body_part=body_part, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_person:
            pass
        self.node.waited_for_end_of_track_person = False

        return self.node.track_person_success, self.node.track_person_message
 
    def track_object(self, object, wait_for_end_of=True):

        self.node.call_neck_track_object_server(object=object, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_object:
            pass
        self.node.waited_for_end_of_track_object = False

        return self.node.track_object_success, self.node.track_object_message   

    def main(self):
        Waiting_for_start_button = 0
        Searching_for_clients = 1
        Navigation_to_person = 2
        Receiving_order_speach = 3
        Receiving_order_listen_and_confirm = 4
        Collect_order_from_barman = 5
        Delivering_order_to_client = 6
        Final_State = 7

        

        print("IN NEW MAIN")
        # self.set_neck(position=self.look_navigation, wait_for_end_of=False)
        # time.sleep(2.0)

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')

                # self.set_face(command="please_say_restaurant")
                # self.set_face(command="please_say_yes_or_no")
                # self.set_face(command="please_say_receptionist")
                # self.set_neck(position=[0.0, -60.0], wait_for_end_of=True)
                # self.set_neck(position=[0.0, -40.0], wait_for_end_of=True)
                self.set_neck(position=self.look_down, wait_for_end_of=True)
                self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)
            

                while True:
                    self.close_house_door_push_left_side()
                    # self.close_house_door_push_right_side()
                    # self.open_house_door_push()
                    # self.open_house_door_pull()
                    # self.close_dishwasher_door()
                    # self.open_washing_machine_door()
                    # self.open_cabinet_door()
                    # self.close_cabinet_door()
                    # self.check_door_depth_hand()
                    # self.open_drawer()
                    # self.close_drawer()
                    # pass
                    
                
                # time.sleep(2.0)

                # POSICIONAR BRAÇO 
                self.set_neck(position=self.look_right, wait_for_end_of=True)
                time.sleep(2.0)
                self.state = Searching_for_clients

            elif self.state == Searching_for_clients:
    
                #print('State 1 = Hand Raising Detect')
                # your code here ...

                # self.detected_people = det_people
                # if self.detected_people.num_person > 0:

                # self.check_door_depth_hand()

                # overall = self.align_hand_with_object_detected_head() 
                      
                

                """ if overall: 
                    # print("STOP", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("STOP") 
                else:
                    # print("GO", overall, zeros, round(zeros_err,2), near, round(near_err,2))
                    print("GO") """
                        
                
                
                # next state
                # self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                print("Finished task!!!")

                while True:
                    pass

            else:
                pass

    def track_object(self, half_image_zero_or_near_percentage=0.3, full_image_near_percentage=0.1, near_max_dist = 400, near_max_dist_1=350, near_max_dist_2 = 900):
        overall = False
        DEBUG = True

        if self.node.first_depth_image_received:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            height, width = current_frame_depth_hand.shape
            center_height = height / 2
            center_width = width / 2

            

            # print(height, width)
            # print(current_frame_depth_hand)

            tot_pixeis = height*width 
            blank_image = self.node.detected_objects.image_rgb
            """  mask_zero = (current_frame_depth_hand == 0)
            mask_near = (current_frame_depth_hand > 0) & (current_frame_depth_hand <= near_max_dist_1)
            mask_remaining = (current_frame_depth_hand > near_max_dist_1)
            blank_image = np.zeros((height,width,3), np.uint8)
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            full_image_near_err = 0.0
            
            full_image_near_err = pixel_count_near/tot_pixeis 
            # print('Percentage of image under ', near_max_dist_1, ' = ',full_image_near_err*100 - 0.7) """

            if hasattr(self.node.detected_objects_hand, 'image_rgb'):
                if hasattr(self.node.detected_objects_hand, 'objects'): 
                    if self.node.detected_objects_hand.objects:
                        # print(self.node.detected_objects_hand.objects)

                        for obj in self.node.detected_objects_hand.objects:
                            print(obj)
                            if obj.object_name == "Tropical Juice":
                                wanted_object = obj
                                print('é isto')

                        # object_detected_hand = self.node.detected_objects_hand.objects[0]
                        object_detected_hand = wanted_object
                        current_frame_rgb_hand = self.node.br.imgmsg_to_cv2(self.node.detected_objects_hand.image_rgb, desired_encoding="passthrough")
                    
                        # print(self.node.detected_objects_hand.objects)

                        error_x_bbox = width / 2 - object_detected_hand.box_center_x
                        error_y_bbox = height / 2 - object_detected_hand.box_center_y

                        print(error_x_bbox)
                        print(error_y_bbox)
                        cv2.circle(current_frame_rgb_hand, (object_detected_hand.box_center_x, object_detected_hand.box_center_y), 100, [255, 0, 0], 10)
                        print(center_width, center_height)
                        cv2.circle(current_frame_rgb_hand, (int(center_width), int(center_height)), 20, [0, 0, 255], 5)
                        
                        cv2.imshow("New Img Distance Inspection", current_frame_rgb_hand)
                        cv2.waitKey(10)

                        """ self.set_arm(command="go_left", wait_for_end_of=True)
                        # time.sleep(3)
                        self.set_arm(command="go_up", wait_for_end_of=True)
                        # time.sleep(3)

                        self.set_arm(command="go_right", wait_for_end_of=True)
                        # time.sleep(3)
                        self.set_arm(command="go_down", wait_for_end_of=True)
                        # time.sleep(3) """
                        arm_value = Float32()



                        if error_x_bbox < -20.0:
                            print('move right')
                            arm_value.data = abs(error_x_bbox)
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_right", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                            
                        elif error_x_bbox > 20.0: 
                            print('move left')
                            arm_value.data = -error_x_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox < -20.0:
                            print('move down')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_down", wait_for_end_of=True)

                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox > 20.0:
                            print('move up')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_up", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')


                            
                        else:
                            print('Cheguei')
                            while True:
                                pass
                       
                        """ if error_x_bbox < 30.0 and error_y_bbox < 30.0:
                            print('CHEGAMOS')
                            print(error_x_bbox, error_y_bbox)
                            time.sleep(3)
                        else:
                            print('segue jogo') """
                        
                        print('\n \n \n \n')
                        # time.sleep(5)
            
            """ i = 0
            j = 0

            for a in current_frame_depth_hand:
                i += 1
                # print(a)
                print('---')
                j = 0

                mask_remaining = (current_frame_depth_hand > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)

                for b in a:
                    j += 1
                    print(b)
                    print('---')

            print(i)
            print(j)
            
            print('\n\n -- \n\n')

            time.sleep(3)

            """


            

            

            """ current_frame_depth_head_half = current_frame_depth_head[height//2:height,:]
            
            # FOR THE FULL IMAGE

            tot_pixeis = height*width 
            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head > 0) & (current_frame_depth_head <= near_max_dist)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_head > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            # FOR THE BOTTOM HALF OF THE IMAGE

            mask_zero_half = (current_frame_depth_head_half == 0)
            mask_near_half = (current_frame_depth_head_half > 0) & (current_frame_depth_head_half <= near_max_dist)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_head_half > near_max_dist) # just for debug
                blank_image_half = np.zeros((height//2,width,3), np.uint8)
                blank_image_half[mask_zero_half] = [255,255,255]
                blank_image_half[mask_near_half] = [255,0,0]
                blank_image_half[mask_remaining_half] = [0,0,255]
                    
            pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
            pixel_count_near_half = np.count_nonzero(mask_near_half)
            
            if DEBUG:
                cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
                cv2.imshow("New Img Distance Inspection", blank_image)
                cv2.waitKey(10)

            half_image_zero_or_near = False
            half_image_zero_or_near_err = 0.0
            
            full_image_near = False
            full_image_near_err = 0.0


            half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
            if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
                half_image_zero_or_near = True
            
            full_image_near_err = pixel_count_near/tot_pixeis
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True
            
            
            if half_image_zero_or_near or full_image_near:
                overall = True """

            # just for debug
            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
            # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err
        
        return overall
        

    def Rot(self, eixo, angulo):
        ang_rad = angulo*math.pi/180.0
        c = math.cos(ang_rad)
        s = math.sin(ang_rad)
        M = np.identity(4)
        if (eixo == 'x' or eixo == 'X'):
            M[1][1] = M[2][2] = c
            M[1][2] = -s
            M[2][1] = s
        elif (eixo == 'y' or eixo == 'Y'):
            M[0][0] = M[2][2] = c
            M[0][2] = s
            M[2][0] = -s
        elif (eixo == 'z' or eixo == 'Z'):
            M[0][0] = M[1][1] = c
            M[0][1] = -s
            M[1][0] = s
        return M
        
    def Trans(self, tx, ty, tz):
        M = np.identity(4)
        M[0][3] = tx
        M[1][3] = ty
        M[2][3] = tz
        return M

    def transform_coordinates(self, x, y, z):
        # C representa o ponto no espaço para onde eu quero transformar a base do braço do robô
        # A matriz de transformação desde a  base do braço até ao centro do Robot pode ser representada por:
        # T = Rot(z, 180) * Rot (x, -90) * Trans (3, -6, -110)
        # a2 representa a translação desde a base do braço até ao centro do robô  (em cm)
        # a1 representa a rotação sobre o eixo coordenadas x em -90º para alinhar os eixos coordenados
        # a0 representa a rotação sobre o eixo coordenadas z em 180º para alinhar o eixo dos x 
        # c representa o ponto (x,y,z) em relação ao centro do braço
        
        

        ### nos numeros que chegam: x representa a frente do robô. y positivo vai para a esquerda do robô. z vai para cima no robô
        ### nos numeros que saem: x vai para trás do robô. y vai para baixo no robô. z vai para a direita do robô
        
        
        ### PARECE-ME QUE X E Z ESTÃO TROCADOS NO RESULTADO QUE TENHO EM RELAºÃO AO BRAÇO
        print('\n\n')
    
        c = np.dot(np.identity(4), [0, 0, 0, 1])
        # c = np.dot(np.identity(4), [90.0, -30.0, 105.0, 1])
        ### ESTAS TRANSFORMAÇÕES SEGUINTES SÃO NECESSÁRIAS PORQUE O ROBOT TEM EIXO COORDENADAS COM Y PARA A FRENTE E X PARA A DIREITA E AS TRANSFORMAÇÕES DA CAMARA SÃO FEITAS COM X PARA A FRENTE Y PARA A ESQUERDA
        new_x = x * 1000
        new_y = y * 1000
        new_z = z * 1000
        c = np.dot(np.identity(4), [new_x, new_y, new_z, 1])
        print(f'Posição em relação ao solo:[{new_x:.2f}, {new_y:.2f}, {new_z:.2f}]')
        a2 = self.Trans(30.0, -60.0, -1100.0)
        a1 = self.Rot('x', -90.0)
        a0 = self.Rot('z', 180.0)
        T = np.dot(a0, a1)
        T = np.dot(T, a2)
        
        #print('T', T)
        
        AA = np.dot(T, c)
        
        print('Ponto em relação ao braço:', AA)


        # aux = AA[0]
        # AA[0] = AA[2]
        # AA[2] = aux

        # AA[0] = AA[0] * 10
        # AA[1] = AA[1] * 10
        # AA[2] = AA[2] * 10
        # my_formatted_list = [ '%.2f' % elem for elem in AA ]
        ### VALOR DO Z ESTÀ INVERSO AO QUE EU DEVO PASSAR PARA O BRAÇO EM AA !!!
        
        # print('Ponto em relação ao braço:', AA)
        # print('y = ', AA[1]*10)

        print('\n\n')

        return AA

    def transform_temp(self, obj):
        new_x = obj.y * 1000
        new_y = -obj.x * 1000
        new_z = obj.z * 1000
        c = np.dot(np.identity(4), [new_x, new_y, new_z, 1])
        print(f'Posição em relação ao solo:[{new_x:.2f}, {new_y:.2f}, {new_z:.2f}]')
        a2 = self.Trans(30.0, -60.0, -1100.0)
        a1 = self.Rot('x', -90.0)
        a0 = self.Rot('z', 180.0)
        T = np.dot(a0, a1)
        T = np.dot(T, a2)
        
        #print('T', T)
        
        AA = np.dot(T, c)
        
        print('Ponto em relação ao braço:', AA)
        return AA

    def transform(self, obj):
        # C representa o ponto no espaço para onde eu quero transformar a base do braço do robô
        # A matriz de transformação desde a  base do braço até ao centro do Robot pode ser representada por:
        # T = Rot(z, 180) * Rot (x, -90) * Trans (3, -6, -110)
        # a2 representa a translação desde a base do braço até ao centro do robô  (em cm)
        # a1 representa a rotação sobre o eixo coordenadas x em -90º para alinhar os eixos coordenados
        # a0 representa a rotação sobre o eixo coordenadas z em 180º para alinhar o eixo dos x 
        # c representa o ponto (x,y,z) em relação ao centro do braço
        
        

        ### nos numeros que chegam: x representa a frente do robô. y positivo vai para a esquerda do robô. z vai para cima no robô
        ### nos numeros que saem: x vai para trás do robô. y vai para baixo no robô. z vai para a direita do robô
        
        
        ### PARECE-ME QUE X E Z ESTÃO TROCADOS NO RESULTADO QUE TENHO EM RELAºÃO AO BRAÇO
        print('\n\n')
    
        c = np.dot(np.identity(4), [0, 0, 0, 1])
        # c = np.dot(np.identity(4), [90.0, -30.0, 105.0, 1])
        ### ESTAS TRANSFORMAÇÕES SEGUINTES SÃO NECESSÁRIAS PORQUE O ROBOT TEM EIXO COORDENADAS COM Y PARA A FRENTE E X PARA A DIREITA E AS TRANSFORMAÇÕES DA CAMARA SÃO FEITAS COM X PARA A FRENTE Y PARA A ESQUERDA
        new_x = obj.position_relative.y * 1000
        new_y = -obj.position_relative.x * 1000
        new_z = obj.position_relative.z * 1000
        print(obj.object_name)
        c = np.dot(np.identity(4), [new_x, new_y, new_z, 1])
        print(f'Posição em relação ao solo:[{new_x:.2f}, {new_y:.2f}, {new_z:.2f}]')
        a2 = self.Trans(30.0, -60.0, -1100.0)
        a1 = self.Rot('x', -90.0)
        a0 = self.Rot('z', 180.0)
        T = np.dot(a0, a1)
        T = np.dot(T, a2)
        
        #print('T', T)
        
        AA = np.dot(T, c)
        
        print('Ponto em relação ao braço:', AA)


        # aux = AA[0]
        # AA[0] = AA[2]
        # AA[2] = aux

        # AA[0] = AA[0] * 10
        # AA[1] = AA[1] * 10
        # AA[2] = AA[2] * 10
        # my_formatted_list = [ '%.2f' % elem for elem in AA ]
        ### VALOR DO Z ESTÀ INVERSO AO QUE EU DEVO PASSAR PARA O BRAÇO EM AA !!!
        
        # print('Ponto em relação ao braço:', AA)
        # print('y = ', AA[1]*10)

        print('\n\n')

        return AA
    
    def align_hand_with_object_detected_head(self, half_image_zero_or_near_percentage=0.3, full_image_near_percentage=0.1, near_max_dist = 400, near_max_dist_1=350, near_max_dist_2 = 900):
        DEBUG = True

        # quero implementar contas de passar de euler para quaternion e quero testar a função pose planner com isso

        # self.node.pose_planner(1.0)
        # self.node.pose_exec()



        if self.node.first_depth_image_received:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            height, width = current_frame_depth_hand.shape
            center_height = height / 2
            center_width = width / 2

            

            # print(height, width)
            # print(current_frame_depth_hand)

            tot_pixeis = height*width 
  

            if hasattr(self.node.detected_objects, 'image_rgb'):
                head_image = self.node.detected_objects.image_rgb
                if hasattr(self.node.detected_objects_hand, 'objects'): 
                    if self.node.detected_objects.objects:
                        # print(self.node.detected_objects_hand.objects)

                        wanted_object = ''

                        for obj in self.node.detected_objects.objects:
                            print(obj)
                            if obj.object_name == "Cleanser":
                                wanted_object = obj
                                print('é isto')

                        if wanted_object != '':

                            # object_detected_hand = self.node.detected_objects_hand.objects[0]
                            object_detected_head = wanted_object
                            current_frame_rgb_head = self.node.br.imgmsg_to_cv2(self.node.detected_objects.image_rgb, desired_encoding="passthrough")

                            set_pose_arm = ListOfFloats()
                            object_location = self.transform(wanted_object)
                            
                            object_x = object_location[0]
                            object_y = object_location[1]
                            object_z = object_location[2]

                            print(object_x, object_y, object_z)   

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(3)
                            print(self.node.arm_current_pose)

                            # self.set_arm(command="open_gripper", wait_for_end_of=True)

                            """ set_pose_arm.pose[:] = array('f')

                                # set_pose_arm.pose.clear()

                            # set_pose_arm.pose.append(-650.0)
                            # set_pose_arm.pose.append(340.0)
                            # set_pose_arm.pose.append(165.0)
                            set_pose_arm.pose.append(object_x)
                            set_pose_arm.pose.append(object_y)
                            set_pose_arm.pose.append(object_z)
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])
                            
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            self.set_arm(command="debug", wait_for_end_of=True)

                            time.sleep(3) """

                            set_pose_arm.pose[:] = array('f')

                                # set_pose_arm.pose.clear()

                            new_x = object_x + 100.0
                            new_y = object_y + 100.0
                            set_pose_arm.pose.append(new_x)
                            set_pose_arm.pose.append(new_y)
                            set_pose_arm.pose.append(object_z)
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])
                            
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            self.set_arm(command="debug", wait_for_end_of=True)
                            # self.set_arm(command="close_gripper", wait_for_end_of=True)
                            
                            while True:
                                pass

                            response = self.node.pose_planner([object_x, object_y, object_z, self.node.arm_current_pose[3], self.node.arm_current_pose[4], self.node.arm_current_pose[5]])
                            # self.node.pose_planner([-521.9, -220.5, 480.5, math.radians(43.0), math.radians(1.9), math.radians(-87.2)])
                            # self.node.pose_exec()

                            

                            """ aux_h = object_x**2 + object_z**2

                            h = math.sqrt(aux_h)
                            print('hipotenusa', h)
                            if object_x > 0.0:
                                new_x = math.sin(math.radians(45))*(h - 200)
                                print('new_x: ',new_x, 'old_x:', object_x)
                            else:
                                new_x = - math.sin(math.radians(45))*(h - 200)
                                print('new_x: ',new_x, 'old_x:', object_x)
                            if object_z > 0.0:
                                new_z = math.cos(math.radians(45))*(h-200)
                                print('new_z: ',new_z, 'old_z:', object_z)
                            else:
                                new_z = - math.cos(math.radians(45))*(h-200)
                                print('new_z: ',new_z, 'old_z:', object_z)

                            a = math.sqrt(new_z**2 + new_x**2)
                            new_y = object_y + 100 # para o braço baixar 10 cm
                            print(a)
                        


                            print('a')

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(1)
                            print(self.node.arm_current_pose)



                           
                            set_pose_arm.pose.append(new_x)
                            set_pose_arm.pose.append(new_y)
                            set_pose_arm.pose.append(new_z)
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])
                            
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            self.set_arm(command="move_linear", wait_for_end_of=True)
                            time.sleep(1)

                            print('A')

                            self.set_arm(command="open_gripper", wait_for_end_of=True)
                            time.sleep(1)

                            print(set_pose_arm)
                            print(set_pose_arm.pose) """

                            if response == True:
                                print('YES')

                                set_pose_arm.pose[:] = array('f')

                                # set_pose_arm.pose.clear()

                                set_pose_arm.pose.append(object_x)
                                set_pose_arm.pose.append(object_y)
                                set_pose_arm.pose.append(object_z)
                                # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(self.node.arm_current_pose[3])
                                set_pose_arm.pose.append(self.node.arm_current_pose[4])
                                set_pose_arm.pose.append(self.node.arm_current_pose[5])
                                
                                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                                self.set_arm(command="move_linear", wait_for_end_of=True)

                                time.sleep(3)
                                self.set_arm(command="close_gripper", wait_for_end_of=True)

                            else:
                                print('NO')
                                print(response)

                            while True:
                                pass
                        
                        ### AGORA TENHO DE RETIRAR A ORIENTAÇÃO ATUAL DO BRAÇO. DEPOIS CHAMO A FUNÇÃO DE TRANSFORMAÇÃO QUE ME DIZ QUAL É O VALOR QUE O BRAÇO TEM DE ESTAR
                        ### PARA ESTAR ALINHADO COM O OBJETO. USO ESSE VALOR PARA ALINHAR O BRAÇO EM X E Y COM O OBJETO



                        arm_value = Float32()



                        """ if error_x_bbox < -20.0:
                            print('move right')
                            arm_value.data = abs(error_x_bbox)
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_right", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                            
                        elif error_x_bbox > 20.0: 
                            print('move left')
                            arm_value.data = -error_x_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox < -20.0:
                            print('move down')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_down", wait_for_end_of=True)

                            print('---------------------------- \n \n --------------------------')
                        elif error_y_bbox > 20.0:
                            print('move up')
                            arm_value.data = error_y_bbox
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)
                            self.set_arm(command="go_up", wait_for_end_of=True)
                            print('---------------------------- \n \n --------------------------') """

    def check_door_depth_hand(self, half_image_zero_or_near_percentage=0.3, full_image_near_percentage=0.1, near_max_dist=600):

        overall = False
        DEBUG = True

        if self.node.new_image_hand_flag:
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img_hand, desired_encoding="passthrough")
            height, width = current_frame_depth_hand.shape
            current_frame_depth_hand_half = current_frame_depth_hand[height//2:height,:]
            current_frame_depth_hand_center = current_frame_depth_hand[height//4:height-height//4, width//3:width-width//3]
            # FOR THE FULL IMAGE

            tot_pixeis = height*width 
            tot_pixeis = (height-height//4 -height//4) * (width-width//3 - width//3)
            mask_zero = (current_frame_depth_hand == 0)
            mask_near = (current_frame_depth_hand > 0) & (current_frame_depth_hand <= near_max_dist)
            mask_zero_center = (current_frame_depth_hand_center == 0)
            mask_near_center = (current_frame_depth_hand_center > 0) & (current_frame_depth_hand_center <= near_max_dist)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_hand > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)
            pixel_count_zeros_center = np.count_nonzero(mask_zero_center)

            # FOR THE BOTTOM HALF OF THE IMAGE

            mask_zero_half = (current_frame_depth_hand_half == 0)
            mask_near_half = (current_frame_depth_hand_half > 0) & (current_frame_depth_hand_half <= near_max_dist)
            mask_near_center = (current_frame_depth_hand_center > 0) & (current_frame_depth_hand_center <= near_max_dist)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_hand_half > near_max_dist) # just for debug
                blank_image_half = np.zeros((height//2,width,3), np.uint8)
                blank_image_half[mask_zero_half] = [255,255,255]
                blank_image_half[mask_near_half] = [255,0,0]
                blank_image_half[mask_remaining_half] = [0,0,255]
                    
            pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
            pixel_count_near_half = np.count_nonzero(mask_near_half)
            pixel_count_near_center = np.count_nonzero(mask_near_center)
            
            if DEBUG:
                cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
                cv2.rectangle(blank_image, (width//3, height//4), (width - width//3, height - height//4), (0, 255, 0), 3)
                cv2.imshow("New Img Distance Inspection", blank_image)
                cv2.waitKey(10)

            half_image_zero_or_near = False
            half_image_zero_or_near_err = 0.0
            
            full_image_near = False
            full_image_near_err = 0.0


            half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
            if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
                half_image_zero_or_near = True
            
            full_image_near_err = pixel_count_near/tot_pixeis
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True

            center_image_near_err = pixel_count_near_center / tot_pixeis
            print(center_image_near_err*100)
            center_image_zeros = pixel_count_zeros_center/tot_pixeis
            #print(center_image_zeros*100)
            if full_image_near_err >= full_image_near_percentage:
                center_image_near = True
            
            if half_image_zero_or_near or full_image_near:
                overall = True

            # just for debug
            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
            # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err
        
            return center_image_near_err
        else:
            return -1.0

    def open_washing_machine_door(self):

        if self.node.first_depth_image_received:
            print('b')
            current_frame = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            frame_2 = current_frame.copy()
            height, width = current_frame.shape[:2]
            current_frame_draw = current_frame[:, width //2 - width // 6 : width// 2 + width // 6]
            height, width = current_frame_draw.shape[:2]

            cv2.imshow("Aligned Depth Head", current_frame_draw)
            cv2.waitKey(0)

            cv2.imshow("Aligned Depth Head", current_frame)
            cv2.waitKey(0)

            depth_image_filtered = cv2.medianBlur(current_frame_draw, 5)
            depth_image_filtered_2 = cv2.medianBlur(frame_2, 5)


            # Normalize the depth image for visualization
            depth_image_normalized = cv2.normalize(depth_image_filtered, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = np.uint8(depth_image_normalized)

            depth_image_normalized_2 = cv2.normalize(depth_image_filtered_2, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized_2 = np.uint8(depth_image_normalized_2)

            colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            colored_depth_image_2 = cv2.applyColorMap(depth_image_normalized_2, cv2.COLORMAP_JET)

            cv2.imshow("Aligned Depth Head", colored_depth_image)
            cv2.waitKey(0)

            # Apply Canny edge detection
            edges = cv2.Canny(depth_image_normalized, 25, 50)

            # Detect lines using Hough Line Transform
            lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)

            avg_depths = []
            
            if lines is not None:
                # Sort lines based on their y-coordinate
                lines_sorted = sorted(lines, key=lambda line: np.sin(line[0][1]) * line[0][0])
                print('nr de linhas:', len(lines_sorted))
                print(range(len(lines_sorted)))
                print(len(range(len(lines_sorted))))

                # Iterate over each pair of consecutive lines
                for i in range(len(lines_sorted) - 1):
                    print(lines_sorted[i][0])
                    rho1, theta1 = lines_sorted[i][0]
                    a1 = np.cos(theta1)
                    b1 = np.sin(theta1)
                    x01 = a1 * rho1
                    y01 = b1 * rho1

                    rho2, theta2 = lines_sorted[i+1][0]
                    a2 = np.cos(theta2)
                    b2 = np.sin(theta2)
                    x02 = a2 * rho2
                    y02 = b2 * rho2

                    # Calculate the difference in y-coordinate between consecutive lines
                    y_diff = np.abs(y01 - y02)

                    # Draw lines
                    x1 = int(x01 + 1000 * (-b1))
                    y1 = int(y01 + 1000 * (a1))
                    x2 = int(x01 - 1000 * (-b1))
                    y2 = int(y01 - 1000 * (a1))
                    cv2.line(colored_depth_image, (x1, y1), (x2, y2), (0, 0, 255), 1)

                    print(y_diff)

                    roi = colored_depth_image[int(y01):int(y02), :]
                    roi_depth = current_frame_draw[int(y01):int(y02), :]

                    k = int(y01)
                    j = 0
                    a = 0
                    zeros_ = 0
                    while k < int(y02):
                        while j < width:
                            a += current_frame_draw[k, j]
                            if current_frame_draw[k, j] == 0:
                                zeros_ += 1
                            j += 1
                        k += 1
                    a = a / ((width * height) - zeros_)
                    # print(a)

                    if y_diff > 100.0:
                        if y_diff < 400.0:

                            avg_depths.append((a, roi, y_diff, int(y01), int(y02), roi_depth))

                    # print('\n\n')
      
                print('\n\n')
                # Sort the average depths in descending order
                avg_depths.sort(reverse=True)

                # Display the ROIs in the order of decreasing average depth
                aux_depth = 1.0
                std_dev_aux = 0.0
                lines_detected = False
                for avg_depth, roi, y_diff, y_01, y_02, roi_depth in avg_depths:
                    cv2.imshow(f"ROI with Average Depth {avg_depth}", roi_depth)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    cv2.imshow(f"ROI with Average Depth {avg_depth}", roi)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    std_dev = np.std(roi_depth)
                    print('standard deviation', std_dev)
                    print('depth', avg_depth)
                    if avg_depth < 1.15:
                        if std_dev > 300:
                            if std_dev > std_dev_aux:
                                aux_depth = avg_depth
                                std_dev_aux = std_dev
                                lines_detected = True
                                
                                aux_ = avg_depth, roi, y_diff, y_01, y_02, roi_depth
                                # print('depth', avg_depth)

                cv2.imshow("Aligned Depth Head with Lines", colored_depth_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()


                print(lines_detected)

                if lines_detected == True:
                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    roi_top_y = 0
                    roi_top_x = int(roi_width / 2)

                    # Calculate the center coordinates of the ROI
                    roi_center_y = int(roi_height / 2)
                    roi_center_x = int(roi_width / 2)

                    # Calculate the radius of the circle (you can adjust the radius according to your preference)
                    radius = 10

                    # Draw a circle at the center of the ROI
                    cv2.circle(aux_[1], (roi_center_x, roi_center_y), radius, (0, 255, 0), -1)

                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    height, width = current_frame.shape[:2]

                    print(height, width)

                    # Center coordinates of the ROI
                    roi_center_y = roi_height // 2
                    roi_center_x = roi_width // 2

                    # Coordinates of the ROI in colored_depth_image
                    y1 = aux_[3]
                    y2 = aux_[4]
                    
                    # Coordinates of the colored_depth_image in the current_frame
                    current_frame_start_y = y1
                    current_frame_end_y = y2
                    current_frame_start_x = width // 2 - width // 6

                    # Center coordinates in the original image
                    circle_center_x = current_frame_start_x + roi_center_x
                    circle_center_y = current_frame_start_y + roi_center_y
                    bottom_circle_center_y = current_frame_end_y + roi_center_y

                    top_circle_center_y = circle_center_y - roi_center_y - 40
                    bottom_circle_center_y = bottom_circle_center_y - roi_center_y + 40

                    if top_circle_center_y < 0:
                        top_circle_center_y = 0 

                    # Draw the circle in the original image
                    radius = 10
                    cv2.circle(colored_depth_image_2, (circle_center_x, circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, top_circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, bottom_circle_center_y), radius, (0, 255, 0), -1)

                    

                    points = Pose2D()
                    points.x = float(circle_center_x)
                    points.y = float(top_circle_center_y)

                    print(points)

                    # Create a list to hold the Pose2D objects

                    requested_objects = []

                    bb = BoundingBox()
                    bb.box_top_left_x = circle_center_x - 20
                    bb.box_top_left_y = top_circle_center_y - 20 
                    bb.box_width = 40
                    bb.box_height = 40

                    bb_2 = BoundingBox()
                    bb_2.box_top_left_x = circle_center_x - 20
                    bb_2.box_top_left_y = circle_center_y - 20 
                    bb_2.box_width = 40
                    bb_2.box_height = 40

                    bb_3 = BoundingBox()
                    bb_3.box_top_left_x = circle_center_x - 20
                    bb_3.box_top_left_y = bottom_circle_center_y - 20 
                    bb_3.box_width = 40
                    bb_3.box_height = 40

                    cv2.rectangle(colored_depth_image_2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x + bb.box_width, bb.box_top_left_y + bb.box_height), (255, 0, 0), 2)
                    cv2.rectangle(colored_depth_image_2, (bb_2.box_top_left_x, bb_2.box_top_left_y), (bb_2.box_top_left_x + bb_2.box_width, bb_2.box_top_left_y + bb_2.box_height), (255, 0, 0), 2)
                    cv2.rectangle(colored_depth_image_2, (bb_3.box_top_left_x, bb_3.box_top_left_y), (bb_3.box_top_left_x + bb_3.box_width, bb_3.box_top_left_y + bb_3.box_height), (255, 0, 0), 2)


                    cv2.imshow("Original Image with Circles", colored_depth_image_2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    get_pc = BoundingBoxAndPoints()
                    # get_pc.requested_point_coords = [points]
                    get_pc.bbox = bb

                    get_pc_2 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_2.bbox = bb_2

                    get_pc_3 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_3.bbox = bb_3

                    requested_objects.append(get_pc)
                    requested_objects.append(get_pc_2)
                    requested_objects.append(get_pc_3)
                    camera = "head"

                    self.node.waiting_for_pcloud = True
                    self.node.call_point_cloud_server(requested_objects, camera)

                    while self.node.waiting_for_pcloud:
                        pass

                    new_pcloud = self.node.point_cloud_response.coords

                    print('--', new_pcloud)

                    print('Initial', new_pcloud[0])
                    auxiliar = new_pcloud[0].center_coords.x
                    new_pcloud[0].center_coords.x = -new_pcloud[0].center_coords.y / 1000
                    new_pcloud[0].center_coords.y = auxiliar/ 1000
                    new_pcloud[0].center_coords.z = new_pcloud[0].center_coords.z / 1000

                    print('Final', new_pcloud[0])

                    object_location = self.transform_temp(new_pcloud[0].center_coords)
                    print('...', object_location)
                    
                    arm_height = object_location[1]
                    arm_centered_x = 130.0
                    # arm_depth retira 20 cm ao ponto acima da máq de lavar que é onde eu quero teoricamente ir para abrir
                    arm_depth = object_location[0] + 200.0


                    set_pose_arm = ListOfFloats()
                    """ new_depth = Float32()
                    new_depth.data = arm_depth """
                    
                    object_x = arm_depth
                    object_y = arm_height
                    object_z = arm_centered_x

                    print('x y e z do ponto que quero alcançar:',object_x, object_y, object_z)   

                    
                    # print(self.node.arm_current_pose)
                    
                    # self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
                    self.set_arm(command="arm_side_of_washing_machine", wait_for_end_of=True)

                    time.sleep(3)
                    self.set_arm(command="get_arm_position", wait_for_end_of=True)
                    time.sleep(3)
                
                    set_pose_arm.pose[:] = array('f')

                    # Set the pose values
                    set_pose_arm.pose.append(object_x)
                    set_pose_arm.pose.append(object_y)
                    set_pose_arm.pose.append(object_z)
                    set_pose_arm.pose.append(self.node.arm_current_pose[3])
                    set_pose_arm.pose.append(self.node.arm_current_pose[4])
                    set_pose_arm.pose.append(self.node.arm_current_pose[5])

                    # Publish the pose
                    self.node.arm_set_pose_publisher.publish(set_pose_arm)
                    print('Desired pose:', set_pose_arm)

                    self.set_arm(command="change_depth_to_open_washing_machine", wait_for_end_of=True)

                    # set_pose_arm.pose.append(object_x)
                    # set_pose_arm.pose.append(500.0)
                    # set_pose_arm.pose.append(object_z)
                    # set_pose_arm.pose.append(self.node.arm_current_pose[3])
                    # set_pose_arm.pose.append(self.node.arm_current_pose[4])
                    # set_pose_arm.pose.append(self.node.arm_current_pose[5])
                    
                    arm_value = Float32()
                    arm_value.data = 300.0
                    self.node.arm_value_publisher.publish(arm_value)
                    print(arm_value)                    
                    self.set_arm(command="go_front", wait_for_end_of=True)
                    # Desde aqui quero baixar linearmente o braço, andar com robô para trás enquanto baixo mais braço e baixo corpo
                    
                    print('mover-se')

                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=1.5, adjust_direction=180.0, wait_for_end_of=True)
                
                    print('moveu-se')
                    self.torso_pos = Pose2D()
                    self.torso_pos.x = -1.0
                    self.node.torso_test_publisher.publish(self.torso_pos)

                    time.sleep(15)
                
                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=1.0, adjust_direction=180.0, wait_for_end_of=True)

                    time.sleep(15)

                    self.torso_pos.x = 0.0
                    self.node.torso_test_publisher.publish(self.torso_pos)

                    self.set_arm(command="end_opening_washing_machine", wait_for_end_of=True)

                    # arm_value.data = -215.0 #valor que quero que a primeira junta tenha antes de eu andar para trás
                    # self.node.arm_value_publisher.publish(arm_value)
                    # print(arm_value)                    
                    # self.set_arm(command="lower_arm_close_washing_machine", wait_for_end_of=True)



                else:
                    print('\n\n Not enough lines, please move me a bit \n\n')

    def close_dishwasher_door(self):
        if self.node.first_depth_image_received:
            print('b')
            self.set_neck(position=[0.0, -40.0], wait_for_end_of=True)
            time.sleep(2)
            current_frame = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            frame_2 = current_frame.copy()
            height, width = current_frame.shape[:2]
   

            # cv2.imshow("Aligned Depth Head", frame_2)
            # cv2.waitKey(0)

            # cv2.imshow("Aligned Depth Head", current_frame)
            # cv2.waitKey(0)

            depth_image_filtered = cv2.medianBlur(frame_2, 5)
            depth_image_filtered_2 = cv2.medianBlur(frame_2, 5)


            # Normalize the depth image for visualization
            depth_image_normalized = cv2.normalize(depth_image_filtered, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = np.uint8(depth_image_normalized)

            depth_image_normalized_2 = cv2.normalize(depth_image_filtered_2, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized_2 = np.uint8(depth_image_normalized_2)

            colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            colored_depth_image_2 = cv2.applyColorMap(depth_image_normalized_2, cv2.COLORMAP_JET)

            # cv2.imshow("Aligned Depth Head", colored_depth_image)
            # cv2.waitKey(0)

            while True:
                if hasattr(self.node.detected_doors, 'image_rgb'):
                    head_image = self.node.detected_doors.image_rgb
                    
                    if hasattr(self.node.detected_doors, 'objects'): 
                        # print(self.node.detected_doors)
                        
                        if self.node.detected_doors.objects:
                            # print(self.node.detected_doors.objects)
                            print('ddd')
            
                            wanted_object = ''
                            requested_objects = []

                            for obj in self.node.detected_doors.objects:
                                print(obj.object_name)
                                if obj.object_name == "Dishwasher":
                                    wanted_object = obj
                                    print('é isto')
                                    print(wanted_object.position_relative)
                                    print(wanted_object.confidence)
                                    bb = BoundingBox()
                                    bb.box_height = int(wanted_object.box_height / 10)
                                    bb.box_width = wanted_object.box_width
                                    bb.box_top_left_y = wanted_object.box_top_left_y + wanted_object.box_height - bb.box_height
                                    bb.box_top_left_x = wanted_object.box_top_left_x
                                    print('Bounding box: ', bb)
                                    get_pc = BoundingBoxAndPoints()
                                    # get_pc.requested_point_coords = [points]
                                    get_pc.bbox = bb
                                    requested_objects.append(get_pc)
                                    camera = "head"
                                    self.node.waiting_for_pcloud = True
                                    self.node.call_point_cloud_server(requested_objects, camera)

                                    while self.node.waiting_for_pcloud:
                                        pass

                                    new_pcloud = self.node.point_cloud_response.coords

                                    print('Distância ao centro da bbox', new_pcloud)

                                    cv2.rectangle(colored_depth_image_2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x + bb.box_width, bb.box_top_left_y + bb.box_height), (255, 0, 0), 2)
                                    cv2.imshow("Aligned Depth Head", colored_depth_image_2)
                                    cv2.waitKey(10)

                                    print('Distância em y da bbox ao robô', new_pcloud[0])
                                    auxiliar = new_pcloud[0].center_coords.x
                                    new_pcloud[0].center_coords.x = -new_pcloud[0].center_coords.y / 1000
                                    new_pcloud[0].center_coords.y = auxiliar/ 1000
                                    new_pcloud[0].center_coords.z = new_pcloud[0].center_coords.z / 1000

                                    print('Distância final', new_pcloud[0])

                                    distance_to_navigate = new_pcloud[0].center_coords.y - 0.6
                                    print('I MUST NAVIGATE: ', distance_to_navigate)

                                    object_location = self.transform_temp(new_pcloud[0].center_coords)
                                    print('Distância da bbox ao braço: ', object_location)

                                    print('moveu-se')
                                    self.torso_pos = Pose2D()
                                    self.torso_pos.x = -1.0
                                    self.node.torso_test_publisher.publish(self.torso_pos)
                                    
                                    time.sleep(30)
                                    self.torso_pos.x = 0.0
                                    self.node.torso_test_publisher.publish(self.torso_pos)

                                    self.set_arm(command="prepare_to_close_dishwasher", wait_for_end_of=True)

                                    time.sleep(1)

                                    self.torso_pos.y = -1.0
                                    self.node.torso_test_publisher.publish(self.torso_pos)
                                    
                                    time.sleep(20.5)

                                    self.torso_pos.y = 0.0
                                    self.node.torso_test_publisher.publish(self.torso_pos)

                                    #NAVIGATION
                                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_to_navigate, adjust_direction=0.0, wait_for_end_of=True)
                                    time.sleep(1)

                                    self.torso_pos.y = 1.0
                                    self.node.torso_test_publisher.publish(self.torso_pos)

                                    time.sleep(20.5)

                                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.2, adjust_direction=0.0, wait_for_end_of=True)

                                    time.sleep(1)

                                    #Navigation
                                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_to_navigate, adjust_direction=180.0, wait_for_end_of=True)

                                    time.sleep(1)                                    

                                    self.set_arm(command="go_initial_position", wait_for_end_of=True)

                                    while True:
                                        pass


                     

                else:
                    print('else')
            

            bb = BoundingBox()

            get_pc = BoundingBoxAndPoints()
            # get_pc.requested_point_coords = [points]
            get_pc.bbox = bb

            get_pc_2 = BoundingBoxAndPoints()
            # get_pc_2.requested_point_coords = [points]
            get_pc_2.bbox = bb_2

            get_pc_3 = BoundingBoxAndPoints()
            # get_pc_2.requested_point_coords = [points]
            get_pc_3.bbox = bb_3

            requested_objects.append(get_pc)
            requested_objects.append(get_pc_2)
            requested_objects.append(get_pc_3)
            camera = "head"
            
            self.node.waiting_for_pcloud = True
            self.node.call_point_cloud_server(requested_objects, camera)

            while self.node.waiting_for_pcloud:
                pass

            new_pcloud = self.node.point_cloud_response.coords

            print('--', new_pcloud)

            print('Initial', new_pcloud[0])
            auxiliar = new_pcloud[0].center_coords.x
            new_pcloud[0].center_coords.x = -new_pcloud[0].center_coords.y / 1000
            new_pcloud[0].center_coords.y = auxiliar/ 1000
            new_pcloud[0].center_coords.z = new_pcloud[0].center_coords.z / 1000

            print('Final', new_pcloud[0])

            object_location = self.transform_temp(new_pcloud[0].center_coords)
            print('...', object_location)
            
            arm_height = object_location[1]
            arm_centered_x = 130.0
            # arm_depth retira 20 cm ao ponto acima da máq de lavar que é onde eu quero teoricamente ir para abrir
            arm_depth = object_location[0] + 200.0


            set_pose_arm = ListOfFloats()
            """ new_depth = Float32()
            new_depth.data = arm_depth """
            
            object_x = arm_depth
            object_y = arm_height
            object_z = arm_centered_x

            print('x y e z do ponto que quero alcançar:',object_x, object_y, object_z)   

            
            # print(self.node.arm_current_pose)
            
            # self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
            self.set_arm(command="arm_side_of_washing_machine", wait_for_end_of=True)

            time.sleep(3)
            self.set_arm(command="get_arm_position", wait_for_end_of=True)
            time.sleep(3)
        
            set_pose_arm.pose[:] = array('f')

            # Set the pose values
            set_pose_arm.pose.append(object_x)
            set_pose_arm.pose.append(object_y)
            set_pose_arm.pose.append(object_z)
            set_pose_arm.pose.append(self.node.arm_current_pose[3])
            set_pose_arm.pose.append(self.node.arm_current_pose[4])
            set_pose_arm.pose.append(self.node.arm_current_pose[5])

            # Publish the pose
            self.node.arm_set_pose_publisher.publish(set_pose_arm)
            print('Desired pose:', set_pose_arm)

            self.set_arm(command="change_depth_to_open_washing_machine", wait_for_end_of=True)

            # set_pose_arm.pose.append(object_x)
            # set_pose_arm.pose.append(500.0)
            # set_pose_arm.pose.append(object_z)
            # set_pose_arm.pose.append(self.node.arm_current_pose[3])
            # set_pose_arm.pose.append(self.node.arm_current_pose[4])
            # set_pose_arm.pose.append(self.node.arm_current_pose[5])
            
            arm_value = Float32()
            arm_value.data = 300.0
            self.node.arm_value_publisher.publish(arm_value)
            print(arm_value)                    
            self.set_arm(command="go_front", wait_for_end_of=True)
            # Desde aqui quero baixar linearmente o braço, andar com robô para trás enquanto baixo mais braço e baixo corpo
            
            print('mover-se')

            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=1.5, adjust_direction=180.0, wait_for_end_of=True)
        
            print('moveu-se')
            self.torso_pos = Pose2D()
            self.torso_pos.x = -1.0
            self.node.torso_test_publisher.publish(self.torso_pos)

            time.sleep(15)
        
            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_time=1.0, adjust_direction=180.0, wait_for_end_of=True)

            time.sleep(15)

            self.torso_pos.x = 0.0
            self.node.torso_test_publisher.publish(self.torso_pos)

            self.set_arm(command="end_opening_washing_machine", wait_for_end_of=True)

            # arm_value.data = -215.0 #valor que quero que a primeira junta tenha antes de eu andar para trás
            # self.node.arm_value_publisher.publish(arm_value)
            # print(arm_value)                    
            # self.set_arm(command="lower_arm_close_washing_machine", wait_for_end_of=True)

    def open_drawer(self):

        if self.node.first_depth_image_received:
            print('b')
            current_frame = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            frame_2 = current_frame.copy()
            height, width = current_frame.shape[:2]
            current_frame_draw = current_frame[:, width //2 - width // 8 : width// 2 + width // 8]
            height, width = current_frame_draw.shape[:2]

            cv2.imshow("Aligned Depth Head", current_frame_draw)
            cv2.waitKey(0)

            cv2.imshow("Aligned Depth Head", current_frame)
            cv2.waitKey(0)

            depth_image_filtered = cv2.medianBlur(current_frame_draw, 5)
            depth_image_filtered_2 = cv2.medianBlur(frame_2, 5)


            # Normalize the depth image for visualization
            depth_image_normalized = cv2.normalize(depth_image_filtered, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = np.uint8(depth_image_normalized)

            depth_image_normalized_2 = cv2.normalize(depth_image_filtered_2, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized_2 = np.uint8(depth_image_normalized_2)

            colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            colored_depth_image_2 = cv2.applyColorMap(depth_image_normalized_2, cv2.COLORMAP_JET)

            cv2.imshow("Aligned Depth Head", colored_depth_image)
            cv2.waitKey(0)

            # Apply Canny edge detection
            edges = cv2.Canny(depth_image_normalized, 25, 50)

            # Display the edges
            cv2.imshow("Edges", edges)
            cv2.waitKey(0)

            # Cleanup windows
            cv2.destroyAllWindows()

            # Detect lines using Hough Line Transform
            lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=70)

            avg_depths = []
            
            if lines is not None:
                # Sort lines based on their y-coordinate
                lines_sorted = sorted(lines, key=lambda line: np.sin(line[0][1]) * line[0][0])
                print('nr de linhas:', len(lines_sorted))
                print(range(len(lines_sorted)))
                print(len(range(len(lines_sorted))))

                # Iterate over each pair of consecutive lines
                for i in range(len(lines_sorted) - 1):
                    print(lines_sorted[i][0])
                    rho1, theta1 = lines_sorted[i][0]
                    a1 = np.cos(theta1)
                    b1 = np.sin(theta1)
                    x01 = a1 * rho1
                    y01 = b1 * rho1

                    rho2, theta2 = lines_sorted[i+1][0]
                    a2 = np.cos(theta2)
                    b2 = np.sin(theta2)
                    x02 = a2 * rho2
                    y02 = b2 * rho2

                    # Calculate the difference in y-coordinate between consecutive lines
                    y_diff = np.abs(y01 - y02)

                    # Draw lines
                    x1 = int(x01 + 1000 * (-b1))
                    y1 = int(y01 + 1000 * (a1))
                    x2 = int(x01 - 1000 * (-b1))
                    y2 = int(y01 - 1000 * (a1))
                    cv2.line(colored_depth_image, (x1, y1), (x2, y2), (0, 0, 255), 1)

                    print(y_diff)

                    roi = colored_depth_image[int(y01):int(y02), :]
                    roi_depth = current_frame_draw[int(y01):int(y02), :]

                    if y_diff > 40.0:
                        if y_diff < 250.0:

                            total_depth = 0
                            valid_pixel_count = 0


                            for k in range(int(y01), int(y02)):
                                for j in range(width):
                                    depth_value = current_frame_draw[k, j]
                                    if depth_value > 0:  # Ignore zero depth values
                                        total_depth += depth_value
                                        valid_pixel_count += 1

                            if valid_pixel_count > 0:
                                avg_depth = total_depth / valid_pixel_count
                            else:
                                avg_depth = 0

                            print(f"Average depth: {avg_depth}")
                            print('y2', int(y02))
                            print('y1', int(y01))
                            print('valid_pixel_count', valid_pixel_count)

                            # Display the ROI with average depth
                            cv2.imshow(f"ROI with Average Depth {avg_depth}", roi_depth)
                            cv2.waitKey(0)
                            cv2.destroyAllWindows()

                            cv2.imshow(f"ROI with Average Depth {avg_depth}", roi)
                            cv2.waitKey(0)
                            cv2.destroyAllWindows()
                            avg_depths.append((avg_depth, roi, y_diff, int(y01), int(y02), roi_depth))

                                    # print('\n\n')
      
                print('\n\n')
                # Sort the average depths in descending order
                avg_depths.sort(reverse=True)

                print(avg_depths)

                # Display the ROIs in the order of decreasing average depth
                aux_depth = 0.0
                std_dev_aux = 0.0
                lines_detected = False
                for avg_depth, roi, y_diff, y_01, y_02, roi_depth in avg_depths:
                    if roi_depth is not None and roi_depth.size > 0:
                        # cv2.imshow(f"ROI with Average Depth {avg_depth}", roi_depth)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        # cv2.imshow(f"ROI with Average Depth {avg_depth}", roi)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        std_dev = np.std(roi_depth)
                        print('standard deviation', std_dev)
                        print('depth', avg_depth)
                        if avg_depth < 1245:
                            if avg_depth > aux_depth:
                                aux_depth = avg_depth
                                std_dev_aux = std_dev
                                lines_detected = True
                                
                                aux_ = avg_depth, roi, y_diff, y_01, y_02, roi_depth
                                # print('depth', avg_depth)
                    else:
                        print(f"Skipping ROI with Average Depth {avg_depth}: roi_depth is empty or not properly defined")


                cv2.imshow("Aligned Depth Head with Lines", colored_depth_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()


                print(lines_detected)

                print('\n\n')

                if lines_detected == True:
                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    roi_top_y = 0
                    roi_top_x = int(roi_width / 2)

                    # Calculate the center coordinates of the ROI
                    roi_center_y = int(roi_height / 2)
                    roi_center_x = int(roi_width / 2)

                    # Calculate the radius of the circle (you can adjust the radius according to your preference)
                    radius = 10

                    # Draw a circle at the center of the ROI
                    cv2.circle(aux_[1], (roi_center_x, roi_center_y), radius, (0, 255, 0), -1)

                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    height, width = current_frame.shape[:2]

                    print(height, width)

                    # Center coordinates of the ROI
                    roi_center_y = roi_height // 2
                    roi_center_x = roi_width // 2

                    # Coordinates of the ROI in colored_depth_image
                    y1 = aux_[3]
                    y2 = aux_[4]
                    
                    # Coordinates of the colored_depth_image in the current_frame
                    current_frame_start_y = y1
                    current_frame_end_y = y2
                    current_frame_start_x = width // 2 - width // 6

                    # Center coordinates in the original image
                    circle_center_x = current_frame_start_x + roi_center_x
                    circle_center_y = current_frame_start_y + roi_center_y
                    bottom_circle_center_y = current_frame_end_y + roi_center_y

                    top_circle_center_y = circle_center_y - roi_center_y - 40
                    bottom_circle_center_y = bottom_circle_center_y - roi_center_y + 40

                    if top_circle_center_y < 0:
                        top_circle_center_y = 0 

                    # Draw the circle in the original image
                    radius = 10
                    cv2.circle(colored_depth_image_2, (circle_center_x, circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, top_circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, bottom_circle_center_y), radius, (0, 255, 0), -1)

                    

                    points = Pose2D()
                    points.x = float(circle_center_x)
                    points.y = float(top_circle_center_y)

                    print(points)

                    # Create a list to hold the Pose2D objects

                    requested_objects = []

                    bb = BoundingBox()
                    bb.box_top_left_x = circle_center_x - 20
                    bb.box_top_left_y = top_circle_center_y - 20 
                    bb.box_width = 40
                    bb.box_height = 40

                    bb_2 = BoundingBox()
                    bb_2.box_top_left_x = circle_center_x - 20
                    bb_2.box_top_left_y = circle_center_y - 20 
                    bb_2.box_width = 40
                    bb_2.box_height = 40

                    bb_3 = BoundingBox()
                    bb_3.box_top_left_x = circle_center_x - 20
                    bb_3.box_top_left_y = bottom_circle_center_y - 20 
                    bb_3.box_width = 40
                    bb_3.box_height = 40

                    # cv2.rectangle(colored_depth_image_2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x + bb.box_width, bb.box_top_left_y + bb.box_height), (255, 0, 0), 2)
                    cv2.rectangle(colored_depth_image_2, (bb_2.box_top_left_x, bb_2.box_top_left_y), (bb_2.box_top_left_x + bb_2.box_width, bb_2.box_top_left_y + bb_2.box_height), (255, 0, 0), 2)
                    # cv2.rectangle(colored_depth_image_2, (bb_3.box_top_left_x, bb_3.box_top_left_y), (bb_3.box_top_left_x + bb_3.box_width, bb_3.box_top_left_y + bb_3.box_height), (255, 0, 0), 2)


                    cv2.imshow("Original Image with Circles", colored_depth_image_2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    get_pc = BoundingBoxAndPoints()
                    # get_pc.requested_point_coords = [points]
                    get_pc.bbox = bb

                    get_pc_2 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_2.bbox = bb_2

                    get_pc_3 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_3.bbox = bb_3

                    requested_objects.append(get_pc)
                    requested_objects.append(get_pc_2)
                    requested_objects.append(get_pc_3)
                    camera = "head"

                    self.node.waiting_for_pcloud = True
                    self.node.call_point_cloud_server(requested_objects, camera)

                    while self.node.waiting_for_pcloud:
                        pass

                    new_pcloud = self.node.point_cloud_response.coords

                    print('--', new_pcloud)

                    print('Initial', new_pcloud[0])
                    auxiliar = new_pcloud[0].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[0].center_coords.x = -new_pcloud[0].center_coords.y / 1000
                    new_pcloud[0].center_coords.y = auxiliar/ 1000
                    new_pcloud[0].center_coords.z = new_pcloud[0].center_coords.z / 1000

                    auxiliar = new_pcloud[1].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[1].center_coords.x = -new_pcloud[1].center_coords.y / 1000
                    new_pcloud[1].center_coords.y = auxiliar/ 1000
                    new_pcloud[1].center_coords.z = new_pcloud[1].center_coords.z / 1000

                    auxiliar = new_pcloud[2].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[2].center_coords.x = -new_pcloud[2].center_coords.y / 1000
                    new_pcloud[2].center_coords.y = auxiliar/ 1000
                    new_pcloud[2].center_coords.z = new_pcloud[2].center_coords.z / 1000

                    print('Final', new_pcloud[0])

                    object_location = self.transform_temp(new_pcloud[0].center_coords)
                    object_location_1 = self.transform_temp(new_pcloud[1].center_coords)
                    object_location_2 = self.transform_temp(new_pcloud[2].center_coords)
                    print('...', object_location, object_location_1, object_location_2)
                    
                    # arm_height = object_location[1]
                    arm_height = (object_location_1[1] + object_location_2[1]) / 2 
                    arm_centered_x = 200.0
                    # arm_depth retira 20 cm ao ponto acima da máq de lavar que é onde eu quero teoricamente ir para abrir
                    print(object_location[0], object_location_2[0])
                    arm_depth = (object_location[0] + object_location_2[0]) / 2
                    arm_depth = object_location_2[0] + 20.0 


                    set_pose_arm = ListOfFloats()
                    """ new_depth = Float32()
                    new_depth.data = arm_depth """
                    
                    object_x = arm_depth
                    object_y = arm_height
                    object_z = arm_centered_x

                    print('x y e z do ponto que quero alcançar:',object_x, object_y, object_z)   

                    
                    # print(self.node.arm_current_pose)
                    
                    # self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
                    self.set_arm(command="arm_prepare_open_drawer", wait_for_end_of=True)
                    self.set_arm(command="open_gripper", wait_for_end_of=True)

                    self.set_arm(command="get_arm_position", wait_for_end_of=True)
                
                    set_pose_arm.pose[:] = array('f')

                    # Set the pose values
                    set_pose_arm.pose.append(object_x)
                    set_pose_arm.pose.append(object_y)
                    set_pose_arm.pose.append(object_z)
                    set_pose_arm.pose.append(self.node.arm_current_pose[3])
                    set_pose_arm.pose.append(self.node.arm_current_pose[4])
                    set_pose_arm.pose.append(self.node.arm_current_pose[5])

                    # Publish the pose
                    self.node.arm_set_pose_publisher.publish(set_pose_arm)
                    print('Desired pose:', set_pose_arm)

                    self.set_arm(command="change_height_open_drawer", wait_for_end_of=True)

                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=180.0, wait_for_end_of=True)

                    arm_value = Float32()
                    arm_value.data = -200.0
                    self.node.arm_value_publisher.publish(arm_value)
                    print(arm_value)

                    time.sleep(3.0)

                    self.set_arm(command="go_back", wait_for_end_of=True)
                    self.set_arm(command="go_initial_position", wait_for_end_of=True)                    

                    while True:
                        print('yes')


                # else:
                #     print('\n\n Not enough lines, please move me a bit \n\n')

    def close_drawer(self):

        if self.node.first_depth_image_received:
            print('b')
            current_frame = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")
            frame_2 = current_frame.copy()
            height, width = current_frame.shape[:2]
            current_frame_draw = current_frame[:, width //2 - width // 8 : width// 2 + width // 8]
            height, width = current_frame_draw.shape[:2]

            cv2.imshow("Aligned Depth Head", current_frame_draw)
            cv2.waitKey(0)

            cv2.imshow("Aligned Depth Head", current_frame)
            cv2.waitKey(0)

            depth_image_filtered = cv2.medianBlur(current_frame_draw, 5)
            depth_image_filtered_2 = cv2.medianBlur(frame_2, 5)


            # Normalize the depth image for visualization
            depth_image_normalized = cv2.normalize(depth_image_filtered, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = np.uint8(depth_image_normalized)

            depth_image_normalized_2 = cv2.normalize(depth_image_filtered_2, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized_2 = np.uint8(depth_image_normalized_2)

            colored_depth_image = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
            colored_depth_image_2 = cv2.applyColorMap(depth_image_normalized_2, cv2.COLORMAP_JET)

            cv2.imshow("Aligned Depth Head", colored_depth_image)
            cv2.waitKey(0)

            # Apply Canny edge detection
            edges = cv2.Canny(depth_image_normalized, 25, 50)

            # Display the edges
            cv2.imshow("Edges", edges)
            cv2.waitKey(0)

            # Cleanup windows
            cv2.destroyAllWindows()

            # Detect lines using Hough Line Transform
            lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=70)

            avg_depths = []
            
            if lines is not None:
                # Sort lines based on their y-coordinate
                lines_sorted = sorted(lines, key=lambda line: np.sin(line[0][1]) * line[0][0])
                print('nr de linhas:', len(lines_sorted))
                print(range(len(lines_sorted)))
                print(len(range(len(lines_sorted))))

                # Iterate over each pair of consecutive lines
                for i in range(len(lines_sorted) - 1):
                    print(lines_sorted[i][0])
                    rho1, theta1 = lines_sorted[i][0]
                    a1 = np.cos(theta1)
                    b1 = np.sin(theta1)
                    x01 = a1 * rho1
                    y01 = b1 * rho1

                    rho2, theta2 = lines_sorted[i+1][0]
                    a2 = np.cos(theta2)
                    b2 = np.sin(theta2)
                    x02 = a2 * rho2
                    y02 = b2 * rho2

                    # Calculate the difference in y-coordinate between consecutive lines
                    y_diff = np.abs(y01 - y02)

                    # Draw lines
                    x1 = int(x01 + 1000 * (-b1))
                    y1 = int(y01 + 1000 * (a1))
                    x2 = int(x01 - 1000 * (-b1))
                    y2 = int(y01 - 1000 * (a1))
                    cv2.line(colored_depth_image, (x1, y1), (x2, y2), (0, 0, 255), 1)

                    print(y_diff)

                    if y_diff > 100.0 and y_diff < 250.0:
                        y01_int = max(0, min(height, int(y01)))  # Ensure y01 is within bounds
                        y02_int = max(0, min(height, int(y02)))  # Ensure y02 is within bounds

                        if y01_int < y02_int:  # Ensure the ROI is valid
                            roi = colored_depth_image[y01_int:y02_int, :]
                            roi_depth = current_frame_draw[y01_int:y02_int, :]

                            total_depth = 0
                            valid_pixel_count = 0

                            for k in range(y01_int, y02_int):
                                for j in range(width):
                                    depth_value = current_frame_draw[k, j]
                                    if depth_value > 0:  # Ignore zero depth values
                                        total_depth += depth_value
                                        valid_pixel_count += 1

                            if valid_pixel_count > 0:
                                avg_depth = total_depth / valid_pixel_count
                            else:
                                avg_depth = 0

                            print(f"Average depth: {avg_depth}")
                            print('y2', y02_int)
                            print('y1', y01_int)
                            print('valid_pixel_count', valid_pixel_count)

                            # Check ROI dimensions before showing
                            if roi_depth.size > 0:
                                cv2.imshow(f"ROI with Average Depth {avg_depth}", roi_depth)
                                cv2.waitKey(0)
                                cv2.destroyAllWindows()
                            else:
                                print(f"Skipping ROI with Average Depth {avg_depth}: roi_depth has zero size")

                            cv2.imshow(f"ROI with Average Depth {avg_depth}", roi)
                            cv2.waitKey(0)
                            cv2.destroyAllWindows()
                            avg_depths.append((avg_depth, roi, y_diff, int(y01), int(y02), roi_depth))

                                    # print('\n\n')
      
                print('\n\n')
                # Sort the average depths in descending order
                avg_depths.sort(reverse=True)

                # print(avg_depths)

                # Display the ROIs in the order of decreasing average depth
                aux_depth = 0.0
                std_dev_aux = 0.0
                lines_detected = False
                for avg_depth, roi, y_diff, y_01, y_02, roi_depth in avg_depths:
                    if roi_depth is not None and roi_depth.size > 0:
                        # cv2.imshow(f"ROI with Average Depth {avg_depth}", roi_depth)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        # cv2.imshow(f"ROI with Average Depth {avg_depth}", roi)
                        # cv2.waitKey(0)
                        # cv2.destroyAllWindows()
                        std_dev = np.std(roi_depth)
                        print('standard deviation', std_dev)
                        print('depth', avg_depth)
                        if avg_depth < 1245:
                            if avg_depth > aux_depth:
                                aux_depth = avg_depth
                                std_dev_aux = std_dev
                                lines_detected = True
                                
                                aux_ = avg_depth, roi, y_diff, y_01, y_02, roi_depth
                                # print('depth', avg_depth)
                    else:
                        print(f"Skipping ROI with Average Depth {avg_depth}: roi_depth is empty or not properly defined")


                cv2.imshow("Aligned Depth Head with Lines", colored_depth_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()


                print(lines_detected)

                print('\n\n')

                if lines_detected == True:
                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    roi_top_y = 0
                    roi_top_x = int(roi_width / 2)

                    # Calculate the center coordinates of the ROI
                    roi_center_y = int(roi_height / 2)
                    roi_center_x = int(roi_width / 2)

                    # Calculate the radius of the circle (you can adjust the radius according to your preference)
                    radius = 10

                    # Draw a circle at the center of the ROI
                    cv2.circle(aux_[1], (roi_center_x, roi_center_y), radius, (0, 255, 0), -1)

                    cv2.imshow('Furthest plane: ', aux_[1])
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    roi_height, roi_width = aux_[1].shape[:2]

                    height, width = current_frame.shape[:2]

                    print(height, width)

                    # Center coordinates of the ROI
                    roi_center_y = roi_height // 2
                    roi_center_x = roi_width // 2

                    # Coordinates of the ROI in colored_depth_image
                    y1 = aux_[3]
                    y2 = aux_[4]
                    
                    # Coordinates of the colored_depth_image in the current_frame
                    current_frame_start_y = y1
                    current_frame_end_y = y2
                    current_frame_start_x = width // 2 - width // 6

                    # Center coordinates in the original image
                    circle_center_x = current_frame_start_x + roi_center_x
                    circle_center_y = current_frame_start_y + roi_center_y
                    bottom_circle_center_y = current_frame_end_y + roi_center_y

                    top_circle_center_y = circle_center_y - roi_center_y - 40
                    bottom_circle_center_y = bottom_circle_center_y - roi_center_y + 40

                    if top_circle_center_y < 0:
                        top_circle_center_y = 0 

                    # Draw the circle in the original image
                    radius = 10
                    cv2.circle(colored_depth_image_2, (circle_center_x, circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, top_circle_center_y), radius, (0, 255, 0), -1)
                    cv2.circle(colored_depth_image_2, (circle_center_x, bottom_circle_center_y), radius, (0, 255, 0), -1)

                    

                    points = Pose2D()
                    points.x = float(circle_center_x)
                    points.y = float(top_circle_center_y)

                    print(points)

                    # Create a list to hold the Pose2D objects

                    requested_objects = []

                    bb = BoundingBox()
                    bb.box_top_left_x = circle_center_x - 20
                    bb.box_top_left_y = top_circle_center_y - 20 
                    bb.box_width = 40
                    bb.box_height = 40

                    bb_2 = BoundingBox()
                    bb_2.box_top_left_x = circle_center_x - 20
                    bb_2.box_top_left_y = circle_center_y - 20 
                    bb_2.box_width = 40
                    bb_2.box_height = 40

                    bb_3 = BoundingBox()
                    bb_3.box_top_left_x = circle_center_x - 20
                    bb_3.box_top_left_y = bottom_circle_center_y - 20 
                    bb_3.box_width = 40
                    bb_3.box_height = 40

                    # cv2.rectangle(colored_depth_image_2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x + bb.box_width, bb.box_top_left_y + bb.box_height), (255, 0, 0), 2)
                    # cv2.rectangle(colored_depth_image_2, (bb_2.box_top_left_x, bb_2.box_top_left_y), (bb_2.box_top_left_x + bb_2.box_width, bb_2.box_top_left_y + bb_2.box_height), (255, 0, 0), 2)
                    cv2.rectangle(colored_depth_image_2, (bb_3.box_top_left_x, bb_3.box_top_left_y), (bb_3.box_top_left_x + bb_3.box_width, bb_3.box_top_left_y + bb_3.box_height), (255, 0, 0), 2)


                    cv2.imshow("Original Image with Circles", colored_depth_image_2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    get_pc = BoundingBoxAndPoints()
                    # get_pc.requested_point_coords = [points]
                    get_pc.bbox = bb

                    get_pc_2 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_2.bbox = bb_2

                    get_pc_3 = BoundingBoxAndPoints()
                    # get_pc_2.requested_point_coords = [points]
                    get_pc_3.bbox = bb_3

                    requested_objects.append(get_pc)
                    requested_objects.append(get_pc_2)
                    requested_objects.append(get_pc_3)
                    camera = "head"

                    self.node.waiting_for_pcloud = True
                    self.node.call_point_cloud_server(requested_objects, camera)

                    while self.node.waiting_for_pcloud:
                        pass

                    new_pcloud = self.node.point_cloud_response.coords

                    print('--', new_pcloud)

                    print('Initial', new_pcloud[0])
                    auxiliar = new_pcloud[0].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[0].center_coords.x = -new_pcloud[0].center_coords.y / 1000
                    new_pcloud[0].center_coords.y = auxiliar/ 1000
                    new_pcloud[0].center_coords.z = new_pcloud[0].center_coords.z / 1000

                    auxiliar = new_pcloud[1].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[1].center_coords.x = -new_pcloud[1].center_coords.y / 1000
                    new_pcloud[1].center_coords.y = auxiliar/ 1000
                    new_pcloud[1].center_coords.z = new_pcloud[1].center_coords.z / 1000

                    auxiliar = new_pcloud[2].center_coords.x
                    # auxiliar = (new_pcloud[2].center_coords.x + new_pcloud[0].center_coords.x) / 2
                    new_pcloud[2].center_coords.x = -new_pcloud[2].center_coords.y / 1000
                    new_pcloud[2].center_coords.y = auxiliar/ 1000
                    new_pcloud[2].center_coords.z = new_pcloud[2].center_coords.z / 1000

                    print('Final', new_pcloud[0])

                    object_location = self.transform_temp(new_pcloud[0].center_coords)
                    object_location_1 = self.transform_temp(new_pcloud[1].center_coords)
                    object_location_2 = self.transform_temp(new_pcloud[2].center_coords)
                    print('...', object_location_2)
                    
                    # arm_height = object_location[1]
                    arm_height = object_location_2[1] + 20.0
                    arm_centered_x = 150.0
                    # arm_depth retira 20 cm ao ponto acima da máq de lavar que é onde eu quero teoricamente ir para abrir
                    arm_depth = object_location_2[0]
                    arm_depth = object_location_2[0] + 50.0 

                    set_pose_arm = ListOfFloats()
                    """ new_depth = Float32()
                    new_depth.data = arm_depth """
                    
                    object_x = arm_depth
                    object_y = arm_height
                    object_z = arm_centered_x

                    print('x y e z do ponto que quero alcançar:',object_x, object_y, object_z)

                    # print(self.node.arm_current_pose)
                    
                    # self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
                    self.set_arm(command="prepare_to_close_drawer", wait_for_end_of=True)
                    self.set_arm(command="close_gripper", wait_for_end_of=True)

                    self.set_arm(command="get_arm_position", wait_for_end_of=True)
                    time.sleep(3)
                
                    set_pose_arm.pose[:] = array('f')

                    # Set the pose values
                    set_pose_arm.pose.append(object_x)
                    set_pose_arm.pose.append(object_y)
                    set_pose_arm.pose.append(object_z)
                    set_pose_arm.pose.append(self.node.arm_current_pose[3])
                    set_pose_arm.pose.append(self.node.arm_current_pose[4])
                    set_pose_arm.pose.append(self.node.arm_current_pose[5])

                    # Publish the pose
                    self.node.arm_set_pose_publisher.publish(set_pose_arm)
                    print('Desired pose:', set_pose_arm)

                    # self.set_arm(command="change_height_close_drawer", wait_for_end_of=True)
                    self.set_arm(command="change_height_close_drawer_javardo", wait_for_end_of=True)

                    self.set_arm(command="get_arm_position", wait_for_end_of=True)
                    radius_robot = 300.0
                    distance_to_close = object_location[0] - self.node.arm_current_pose[0] #10 cm de garantia que fecho bem
                    distance_to_close = (abs(distance_to_close) / 1000 ) - 0.1

                    print('meters to walk', distance_to_close)


                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_to_close, adjust_direction=0.0, wait_for_end_of=True)

                    time.sleep(3.0)

                    self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.25, adjust_direction=360.0, wait_for_end_of=True)
                    time.sleep(3.0)

                    # self.set_arm(command="go_back", wait_for_end_of=True)
                    # self.set_arm(command="go_initial_position", wait_for_end_of=True)                    

                    while True:
                        print('yes')


                # else:
                #     print('\n\n Not enough lines, please move me a bit \n\n')

    def open_cabinet_door(self):
        if self.node.first_depth_image_received:
            print('bbb')

                        
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'): 
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')
        
                        wanted_object = ''

                        for obj in self.node.detected_doors.objects:
                            print(obj.object_name)
                            if obj.object_name == "Wardrobe Door":
                                wanted_object = obj
                                print('é isto')
                                door_position = wanted_object.position_relative
                                print(wanted_object.position_relative)
                                print(wanted_object.confidence)
                                

                        if wanted_object != '':
                            set_pose_arm = ListOfFloats()
                            object_location = self.transform(wanted_object)
                            #Value of height I want the arm to go to not touch in shelfs:
                            desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            new_height = Float32()
                            new_height.data = desired_height
                            
                            object_x = object_location[0]
                            object_y = object_location[1]
                            object_y = object_location[1] + 100.0
                            object_y = 350.0
                            object_z = object_location[2]

                            print('x y e z do objeto:',object_x, object_y, object_z)   

                            distance_to_close = abs(object_x)/1000 
                            print('Distance I am from door', distance_to_close)

                            distance_x_to_center = door_position.x - self.node.wardrobe_width/4 
                            ### ISTO CENTRA QD PORTA FECHADA É A ESQUERDA. -> door_position.x + self.node.wardrobe_width/4 
                            ###  SE PORTA FECHADA FOR A DIREITA, TENHO DE TROCAR SINAL PARA -> door_position.x - self.node.wardrobe_width/4 
                            if distance_x_to_center < 0.0:
                                move_side = 90.0
                            else:
                                move_side = -90.0 + 360.0

                            print('distancia lateral:', distance_x_to_center)
                            distance_x_to_center = abs(abs(distance_x_to_center) - 0.1)


                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_x_to_center, adjust_direction=move_side, wait_for_end_of=True)
                            time.sleep(2)


                            
                            # print(self.node.arm_current_pose)

                            # self.set_arm(command="arm_side_of_washing_machine", wait_for_end_of=True)

                            self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)
                            time.sleep(2)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(1)



                            distance_y = object_x - self.node.arm_current_pose[0]
                            print('distancia em y a andar: ', distance_y)
                            distance_y = abs(distance_y)
                            if distance_y > 400.0:
                                navigation_orientation = 0.0
                            else:
                                navigation_orientation = 180.0
                            
                            distance_y = 400.0 - distance_y

                            print(distance_y)

                            distance_y = abs(distance_y) / 1000

                            self.set_neck(position=self.look_down, wait_for_end_of=True)

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_y, adjust_direction=navigation_orientation, wait_for_end_of=True)
                            time.sleep(1)
                            
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(object_x)
                            set_pose_arm.pose.append(object_y)
                            set_pose_arm.pose.append(object_z)
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])

                            # response = self.node.pose_planner(set_pose_arm)
                       
                            # if response == True:
                            #     print('REACHABLE')
                            #     time.sleep(2)
                            # else:
                            #     print('NOT REACH')
                            #     object_y = 350.0
                            #     set_pose_arm.pose[:] = array('f')

                            #     # Set the pose values
                            #     set_pose_arm.pose.append(object_x)
                            #     set_pose_arm.pose.append(object_y)
                            #     set_pose_arm.pose.append(object_z)
                            #     set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            #     set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            #     set_pose_arm.pose.append(self.node.arm_current_pose[5])

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print(set_pose_arm)

                            # Set arm and check door depth
                            # self.set_arm(command="change_height_front_robot", wait_for_end_of=True)
                            self.set_arm(command="change_height_front_robot", wait_for_end_of=True)
                            time.sleep(1)
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass
                            near_percentage = -1.0
                            while near_percentage == -1.0:
                                near_percentage = self.check_door_depth_hand()

                            print(near_percentage * 100)

                            left_door = False
                            right_door = False

                            # Check if the door is near
                            if near_percentage < 0.5:
                                right_door = True
                                print('Porta direita aberta creio eu')

                            else:
                                right_door = False
                                print('Porta direita fechada creio eu')

                            # self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)

                            # self.set_arm(command="open_gripper", wait_for_end_of=True)

                            set_pose_arm.pose[:] = array('f')

                            # set_pose_arm.pose.clear()
                            set_pose_arm.pose.append(object_x)
                            set_pose_arm.pose.append(object_y)
                            set_pose_arm.pose.append(object_z)
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])
                            
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print(set_pose_arm)
                            # self.set_arm(command="change_height_front_left_robot", wait_for_end_of=True)
                            self.set_arm(command="change_height_front_left_robot", wait_for_end_of=True)
                            time.sleep(1)
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass

                            near_percentage = -1.0
                            while near_percentage == -1.0:
                                near_percentage = self.check_door_depth_hand()

                            print(near_percentage * 100)
                            if near_percentage < 0.5:
                                left_door = True
                                print('Porta esquerda aberta creio eu')
                            else:
                                left_door = False
                                print('Porta esquerda fechada creio eu')

                            
                            # print(right_door, right_door_pose)
                            arm_value = Float32()

                            if right_door == True:
                                print('a')
                                print('desired height', new_height)
                                
                                self.node.arm_set_height_publisher.publish(new_height)
                                time.sleep(2)

                                # self.set_arm(command="change_height_front_robot_value", wait_for_end_of=True)
                                self.set_arm(command="change_height_front_robot_value", wait_for_end_of=True)
                                time.sleep(2)

                                arm_value.data = 100.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)
                                self.set_arm(command="go_front", wait_for_end_of=True)
                                time.sleep(2)

                                self.set_arm(command="get_arm_position", wait_for_end_of=True)
                                time.sleep(1)

                                print('x do braço', self.node.arm_current_pose[0])
                                distance_to_close = object_x/1000 - self.node.arm_current_pose[0]/1000
                                distance_to_close = abs(distance_to_close) + 0.15
                                print('Distance I am from door', distance_to_close)
                                
                                
                                # print('move forward')
                                # arm_value.data = 100.0
                                # self.node.arm_value_publisher.publish(arm_value)
                                # print(arm_value)
                                # self.set_arm(command="go_front", wait_for_end_of=True)


                                ### Ver onde tenho braço em comparação ao ponto inicial detetado como sendo a porta. Quando o braço estiver para lá da porta,
                                ### inclino o braço com ângulo que a consiga pegar desde trás
                                
                                ### O VALOR DO TEMPO AQUI DEVERIA DEPENDER DA DISTÂNCIA A QUE ESTOU DO ARMÁRIO - OU SEJA EU SEI 
                                ### a DISTANCIA DO ARMÁRIO EM RELAÇÃO AO CENTRO DO ROBÔ, ENTÃO DEVERIA IR SEMPRE EM FRENTE TENDO ISSO EM CONTA
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_to_close, adjust_direction=0.0, wait_for_end_of=True)
                                time.sleep(2)
                                
                                self.set_arm(command="open_left_door_from_inside", wait_for_end_of=True)
                                time.sleep(1)
                                
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.2, adjust_direction=180.0, wait_for_end_of=True)
                                time.sleep(2)
                                

                                self.set_arm(command="finish_open_left_door_from_inside", wait_for_end_of=True)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.4, adjust_direction=180.0, wait_for_end_of=True)
                                time.sleep(2)

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)
                                
                                
                                
                                while True:
                                    pass

                                near_percentage = -1.0
                                while near_percentage < 0.5:
                                    near_percentage = self.check_door_depth_hand(near_max_dist=350)

                                    print(near_percentage * 100)
                                    print('move forward')
                                    arm_value.data = 50.0
                                    self.node.arm_value_publisher.publish(arm_value)
                                    print(arm_value)
                                    self.set_arm(command="go_front", wait_for_end_of=True)

                                    time.sleep(2)

                                print('hey')

                                # self.set_arm(command="open_left_door", wait_for_end_of=True)
                                self.set_arm(command="open_left_door_from_front", wait_for_end_of=True)

                                while True:
                                    pass

                            elif left_door == True:

                                print('a')
                                print('desired height', new_height)
                                
                                self.node.arm_set_height_publisher.publish(new_height)
                                time.sleep(2)
                                # self.set_arm(command="change_height_front_left_robot_value", wait_for_end_of=True)
                                self.set_arm(command="change_height_front_left_robot_value", wait_for_end_of=True)

                                

                                self.set_arm(command="get_arm_position", wait_for_end_of=True)
                                time.sleep(1)

                                print('x do braço', self.node.arm_current_pose[0])
                                print('object_x', object_x)
                                print('distance_y', distance_y)
                                distance_to_close = abs(object_x/1000) - abs(distance_y) - abs(self.node.arm_current_pose[0]/1000) + 0.15
                                distance_to_close = abs(distance_to_close)
                                print('Distance I am from door', distance_to_close)



                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_to_close, adjust_direction=0.0, wait_for_end_of=True)
                                time.sleep(2)
                                
                                self.set_arm(command="open_right_door_from_inside", wait_for_end_of=True)
                                
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.2, adjust_direction=180.0, wait_for_end_of=True)
                                time.sleep(1)

                                self.set_arm(command="finish_open_right_door_from_inside", wait_for_end_of=True)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.4, adjust_direction=180.0, wait_for_end_of=True)

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)

                                while True:
                                    pass

                                near_percentage = -1.0
                                while near_percentage < 0.5:
                                    near_percentage = self.check_door_depth_hand(near_max_dist=450)

                                    print(near_percentage * 100)
                                    print('move forward')
                                    arm_value.data = 50.0
                                    self.node.arm_value_publisher.publish(arm_value)
                                    print(arm_value)
                                    self.set_arm(command="go_front", wait_for_end_of=True)

                                    time.sleep(2)

                                print('hey')
                                while True:
                                    pass
                                

                            """ time.sleep(2)
                            arm_value = Float32()
                            # print('move forward')
                            # arm_value.data = 100.0
                            # self.node.arm_value_publisher.publish(arm_value)
                            # print(arm_value)
                            # self.set_arm(command="go_front", wait_for_end_of=True)


                            ### Ver onde tenho braço em comparação ao ponto inicial detetado como sendo a porta. Quando o braço estiver para lá da porta,
                            ### inclino o braço com ângulo que a consiga pegar desde trás

                            near_percentage = -1.0
                            while near_percentage < 0.5:
                                near_percentage = self.check_door_depth_hand(near_max_dist=350)

                                print(near_percentage * 100)
                                print('move forward')
                                arm_value.data = 50.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)
                                self.set_arm(command="go_front", wait_for_end_of=True)

                                time.sleep(2)

                            print('hey')

                            self.set_arm(command="open_left_door", wait_for_end_of=True) """


                            while True:
                                pass

                            time.sleep(3)


                            response = self.node.pose_planner([object_x, object_y, object_z, self.node.arm_current_pose[3], self.node.arm_current_pose[4], self.node.arm_current_pose[5]])
                       
                            if response == True:
                                print('YES')

                                set_pose_arm.pose[:] = array('f')

                                # set_pose_arm.pose.clear()

                                set_pose_arm.pose.append(object_x)
                                set_pose_arm.pose.append(object_y)
                                set_pose_arm.pose.append(object_z)
                                # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(self.node.arm_current_pose[3])
                                set_pose_arm.pose.append(self.node.arm_current_pose[4])
                                set_pose_arm.pose.append(self.node.arm_current_pose[5])
                                
                                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                                self.set_arm(command="move_linear", wait_for_end_of=True)

                                time.sleep(3)
                                self.set_arm(command="close_gripper", wait_for_end_of=True)

                            else:
                                print('NO')
                                print(response)

                            while True:
                                pass

    def close_cabinet_door(self):
        if self.node.first_depth_image_received:
            print('bbb')                 
            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'): 
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')

                        
        
                        wanted_object = ''

                        for obj in self.node.detected_doors.objects:
                            print(obj.object_name)
                            if obj.object_name == "Wardrobe Door":
                                wanted_object = obj
                                print('é isto')
                                print(wanted_object.position_relative)
                                door_position = wanted_object.position_relative
                                print(wanted_object.confidence)
                                

                        if wanted_object != '':
                            set_pose_arm = ListOfFloats()
                            object_location = self.transform(wanted_object)
                            #Value of height I want the arm to go to not touch in shelfs:
                            desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            new_height = Float32()
                            new_height.data = desired_height
                            
                            object_x = object_location[0]
                            object_y = object_location[1]
                            object_y = object_location[1] + 100.0
                            object_y = 350.0
                            object_z = object_location[2]

                            print('x y e z do objeto:', object_x, object_y, object_z)   

                            distance_to_close = abs(object_x)/1000 
                            print('Distance I am from door', distance_to_close)
                            print(self.node.wardrobe_width)
                            print(self.node.wardrobe_width/4)

                            distance_x_to_center = door_position.x + self.node.wardrobe_width/4 
                            ### ISTO CENTRA QD PORTA FECHADA É A ESQUERDA. -> door_position.x + self.node.wardrobe_width/4 
                            ###  SE PORTA FECHADA FOR A DIREITA, TENHO DE TROCAR SINAL PARA -> door_position.x - self.node.wardrobe_width/4 
                            if distance_x_to_center < 0.0:
                                move_side = 90.0
                            else:
                                move_side = -90.0 + 360.0

                            print('distancia lateral:', distance_x_to_center)
                            distance_x_to_center = abs(distance_x_to_center)


                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_x_to_center, adjust_direction=move_side, wait_for_end_of=True)
                            time.sleep(2)
                           
                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(2)
  
                            self.set_arm(command="front_robot_oriented_front", wait_for_end_of=True)

                        
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(object_x)
                            set_pose_arm.pose.append(object_y)
                            set_pose_arm.pose.append(object_z)
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print(set_pose_arm)

                            # Set arm and check door depth
                            # self.set_arm(command="change_height_front_robot", wait_for_end_of=True)
                            self.set_arm(command="change_height_front_robot", wait_for_end_of=True)
                            time.sleep(1)
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass
                            near_percentage = -1.0
                            while near_percentage == -1.0:
                                near_percentage = self.check_door_depth_hand()

                            print(near_percentage * 100)

                            left_door = False
                            right_door = False

                            # Check if the door is near
                            if near_percentage < 0.5:
                                right_door = True
                                print('Porta direita aberta creio eu')

                            else:
                                right_door = False
                                print('Porta direita fechada creio eu')

                            set_pose_arm.pose[:] = array('f')

                            # set_pose_arm.pose.clear()
                            set_pose_arm.pose.append(object_x)
                            set_pose_arm.pose.append(object_y)
                            set_pose_arm.pose.append(object_z)
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])
                            
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print(set_pose_arm)
                            # self.set_arm(command="change_height_front_left_robot", wait_for_end_of=True)
                            self.set_arm(command="change_height_front_left_robot", wait_for_end_of=True)
                            time.sleep(1)
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass

                            near_percentage = -1.0
                            while near_percentage == -1.0:
                                near_percentage = self.check_door_depth_hand()

                            print(near_percentage * 100)
                            if near_percentage < 0.5:
                                left_door = True
                                print('Porta esquerda aberta creio eu')
                            else:
                                left_door = False
                                print('Porta esquerda fechada creio eu')

                            
                            # print(right_door, right_door_pose)
                            arm_value = Float32()

                            if right_door == True:
                                print('a')
                                # print('desired height', new_height)
                                
                                # self.node.arm_set_height_publisher.publish(new_height)
                                # time.sleep(2)

                                # # self.set_arm(command="change_height_front_robot_value", wait_for_end_of=True)
                                # self.set_arm(command="change_height_front_robot_value", wait_for_end_of=True)
                                # time.sleep(2)

                                # arm_value.data = 100.0
                                # self.node.arm_value_publisher.publish(arm_value)
                                # print(arm_value)
                                # self.set_arm(command="go_front", wait_for_end_of=True)
                                # time.sleep(2)

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)

                                self.set_neck(position=[-45.0, -50.0], wait_for_end_of=True)
                                time.sleep(2)

                                wanted_object_2 = ''
                                while wanted_object_2 == '':
                                    for obj in self.node.detected_doors.objects:
                                        print(obj.object_name)
                                        if obj.object_name == "Wardrobe Door":
                                            wanted_object_2 = obj
                                            print('é isto')
                                            print(wanted_object_2.position_relative)
                                            second_door_position = wanted_object_2.position_relative
                                            print(wanted_object_2.confidence)
                                
                                distance_to_move_to_close = abs(second_door_position.x) + 0.3

                                print('Distancia a andar para estar do lado de lá da porta', distance_to_move_to_close)
                                print('Distancia a andar para estar do lado de lá da porta javardo', self.node.wardrobe_width)

                                print('distancia da porta ao robot: ', second_door_position.y - self.node.door_width/4 - 0.3)
                                distance_y = door_position.y - self.node.wardrobe_width/2 - 0.5
                                print('distancia em y a andar: ', distance_y)
                                if distance_y > 0.0:
                                    navigation_orientation = 0.0
                                else:
                                    navigation_orientation = 180.0
                                    distance_y = abs(distance_y)

                                self.set_neck(position=self.look_down, wait_for_end_of=True)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_y, adjust_direction=navigation_orientation, wait_for_end_of=True)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=self.node.wardrobe_width/2, adjust_direction=-90.0 + 360.0, wait_for_end_of=True)
                                time.sleep(1)

                                self.set_arm(command="close_right_door_from_outside", wait_for_end_of=True)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance= 0.2 + self.node.wardrobe_width/2, adjust_direction=90.0, wait_for_end_of=True)
                                time.sleep(1)  

                                self.set_arm(command="finish_close_right_door_from_outside", wait_for_end_of=True)
                                time.sleep(1)    

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)                            

                                while True:
                                    pass



                            elif left_door == True:

                                print('a')
                                # print('desired height', new_height)
                                
                                # self.node.arm_set_height_publisher.publish(new_height)
                                # time.sleep(2)
                                # self.set_arm(command="change_height_front_left_robot_value", wait_for_end_of=True)
                                # self.set_arm(command="change_height_front_left_robot_value", wait_for_end_of=True)

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)

                                self.set_neck(position=[45.0, -50.0], wait_for_end_of=True)
                                time.sleep(2)

                                wanted_object_2 = ''
                                while wanted_object_2 == '':
                                    for obj in self.node.detected_doors.objects:
                                        print(obj.object_name)
                                        if obj.object_name == "Wardrobe Door":
                                            wanted_object_2 = obj
                                            print('é isto')
                                            print(wanted_object_2.position_relative)
                                            second_door_position = wanted_object_2.position_relative
                                            print(wanted_object_2.confidence)
                                
                                distance_to_move_to_close = abs(second_door_position.x) + 0.3

                                print('Distancia a andar para estar do lado de lá da porta', distance_to_move_to_close)
                                print('Distancia a andar para estar do lado de lá da porta javardo', self.node.wardrobe_width)

                                print('distancia da porta ao robot: ', second_door_position.y - self.node.door_width/4 - 0.3)
                                distance_y = door_position.y - self.node.wardrobe_width/2 - 0.5
                                print('distancia em y a andar: ', distance_y)
                                if distance_y > 0.0:
                                    navigation_orientation = 0.0
                                else:
                                    navigation_orientation = 180.0
                                    distance_y = abs(distance_y)

                                self.set_neck(position=self.look_down, wait_for_end_of=True)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=distance_y, adjust_direction=navigation_orientation, wait_for_end_of=True)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=self.node.wardrobe_width/2, adjust_direction=90.0, wait_for_end_of=True)
                                time.sleep(1)

                                self.set_arm(command="close_left_door_from_outside", wait_for_end_of=True)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance= 0.2 + self.node.wardrobe_width/2, adjust_direction=-90.0 + 360.0, wait_for_end_of=True)
                                time.sleep(1)  

                                self.set_arm(command="finish_close_left_door_from_outside", wait_for_end_of=True)
                                time.sleep(1)    

                                self.set_arm(command="go_initial_position", wait_for_end_of=True)
                                time.sleep(1)                            

                                while True:
                                    pass


                                while True:
                                    pass

                        
                            while True:
                                pass

                            time.sleep(3)


                            response = self.node.pose_planner([object_x, object_y, object_z, self.node.arm_current_pose[3], self.node.arm_current_pose[4], self.node.arm_current_pose[5]])
                       
                            if response == True:
                                print('YES')

                                set_pose_arm.pose[:] = array('f')

                                # set_pose_arm.pose.clear()

                                set_pose_arm.pose.append(object_x)
                                set_pose_arm.pose.append(object_y)
                                set_pose_arm.pose.append(object_z)
                                # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(self.node.arm_current_pose[3])
                                set_pose_arm.pose.append(self.node.arm_current_pose[4])
                                set_pose_arm.pose.append(self.node.arm_current_pose[5])
                                
                                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                                self.set_arm(command="move_linear", wait_for_end_of=True)

                                time.sleep(3)
                                self.set_arm(command="close_gripper", wait_for_end_of=True)

                            else:
                                print('NO')
                                print(response)

                            while True:
                                pass

    def open_house_door_pull(self):
        if self.node.first_depth_image_received:
            print('bbb')

            self.set_neck(position=self.look_forward, wait_for_end_of=True)

            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'):
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')
                        # set_pose_arm = ListOfFloats()


                        objects = []
                        seen_names = set()

                        finished_detections = False

                        list_of_neck_position_search = [[0, 0], [10, 0], [-10, 0], [0, -10], [0, 10]]
                        self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)
                        while not finished_detections:
                            
                            for pos in list_of_neck_position_search:

                                print(pos)
                                new_neck_pos = [self.look_forward[0] + pos[0], self.look_forward[1] + pos[1]]
                                #new_neck_pos = [ pos[0],  pos[1]]
                                print('Neck: ', new_neck_pos)
                                self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                                time.sleep(1)
                                for obj in self.node.detected_doors.objects:
                                    print(obj.object_name)
                                    if obj.object_name == "Level Handler" or obj.object_name == "Door":
                                        if obj.object_name not in seen_names:
                                            objects.append(obj)
                                            seen_names.add(obj.object_name)
                                            if obj.object_name == "Level Handler":
                                                handler = obj
                                                print(handler.object_name)
                                            elif obj.object_name == "Door":
                                                door = obj
                                                print(door.object_name)
                                            print('é isto')
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                                    
                                        if len(objects) == 2:
                                            finished_detections = True
                                    if finished_detections == True:
                                        break
                                if finished_detections == True:
                                        break
                                

                        print('\n\n\n')
                        print('posição abs:', objects[0].position_absolute, 'da ', objects[0].object_name)
                        print('posição abs:', objects[1].position_absolute, 'da ', objects[1].object_name)
                        print('\n\n\n')
                        

                        if objects != []:
                            set_pose_arm = ListOfFloats()
                            door_location = self.transform(door)
                            handler_location = self.transform(handler)
                            #Value of height I want the arm to go to not touch in shelfs:
                            
                            # desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            # new_height = Float32()
                            # new_height.data = desired_height
                            
                            door_x = door_location[0]
                            door_y = door_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            door_z = door_location[2]

                            print('x y e z da porta:',door_x, door_y, door_z)   

                            handler_x = handler_location[0]
                            handler_y = handler_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            handler_z = handler_location[2]

                            print('x y e z do handler:',handler_x, handler_y, handler_z)
                            right_side = False
                            left_side = False

                            if handler.box_center_x > door.box_top_left_x and handler.box_center_y > door.box_top_left_y:
                                if handler.box_center_x < door.box_top_left_x + door.box_width and handler.box_center_y < door.box_top_left_y + door.box_height:
                                    print('handler inside door')
                                    if handler.box_center_x > door.box_center_x:
                                        print('right side of door')
                                        right_side = True
                                        left_side = False
                                    else:
                                        print('left side of door')
                                        right_side = False
                                        left_side = True

                            if handler_z < 0.0: # handler está à minha esquerda
                                adjust_x = abs(handler_z)/1000
                                print('Adjusting to the left in ', adjust_x, 'meters')
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_x, adjust_direction=90.0, wait_for_end_of=True)
                                time.sleep(1)
                            elif handler_z > 0.0: # handler está à minha drt
                                adjust_x = abs(handler_z)/1000
                                print('Adjusting to the right in ', adjust_x, 'meters')
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_x - 0.2, adjust_direction=-90.0 + 360.0, wait_for_end_of=True)
                                time.sleep(1)
                            
                            ### NAVEGAR PARA O HANDLER

                            navigate_x = ( abs(handler_x) - 850.0 ) / 1000

                            print('I must navigate for: ', navigate_x, 'm to be at 0.7m from door')
                            self.set_rgb(command=GREEN+BLINK_QUICK)

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=navigate_x, adjust_direction=0.0, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)





                            ### centrar com handler
                            self.set_arm(command="orient_to_door_front", wait_for_end_of=True)
                            time.sleep(1)
                            
                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(self.node.arm_current_pose[0])
                            set_pose_arm.pose.append(handler_y + 150.0)
                            set_pose_arm.pose.append(90.0)
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print('Desired pose:', set_pose_arm)
                            time.sleep(1)

                            self.activate_yolo_objects(activate_doors=False, activate_doors_hand = False)

                            self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            time.sleep(1)

                            
                            
                            





                            self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)

                            self.set_neck(position=self.look_down, wait_for_end_of=True)
                            time.sleep(1)

                            self.set_rgb(command=WHITE+HALF_ROTATE)

                            while not self.node.first_depth_image_hand_received:
                                pass
                            
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                                
                        finished_detections = False

                        time.sleep(1)

                        if hasattr(self.node.detected_doors, 'objects'):
                            print('ddd')
                            if self.node.detected_doors_hand.objects:
                                while not finished_detections:
                                    for obj in self.node.detected_doors_hand.objects:
                                        print(obj.object_name)
                                        if obj.object_name == "Level Handler":
                                            handler = obj
                                            print(handler.object_name)
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                            
                                            finished_detections = True
                                        if finished_detections == True:
                                            break
                                    if finished_detections == True:
                                        break
                        
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            self.activate_yolo_objects(activate_doors=False, activate_doors_hand = False)
                            arm_value = Float32()
                            arm_value.data = handler.position_relative.x * 1000
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go right: ', arm_value)                    
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            # arm_value = Float32()
                            arm_value.data = (handler.position_relative.z * 1000) + 65.0
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go down: ', arm_value)                   
                            self.set_arm(command="go_down", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(self.node.arm_current_pose[0])
                            set_pose_arm.pose.append(self.node.arm_current_pose[1])
                            set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(math.radians(-29.6)) #### COLOCAR GARRA DE LADO
                            set_pose_arm.pose.append(math.radians(87.7))
                            set_pose_arm.pose.append(math.radians(150.7))

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print('Desired pose:', set_pose_arm)
                            time.sleep(1)

                            self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="open_gripper", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            # arm_value = Float32()
                            arm_value.data = handler.position_relative.y * 1000 - 130
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go front: ', arm_value)  
                            self.set_arm(command="go_front", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(self.node.arm_current_pose[0])
                            set_pose_arm.pose.append(self.node.arm_current_pose[1])
                            set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            set_pose_arm.pose.append(math.radians(-67.0)) #### COLOCAR GARRA DE LADO
                            set_pose_arm.pose.append(math.radians(81.9))
                            set_pose_arm.pose.append(math.radians(105.3))

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print('Desired pose:', set_pose_arm)

                            self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            arm_value = Float32()
                            arm_value.data = -55.0
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)                    
                            self.set_arm(command="go_right", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.35, adjust_direction=-180.0, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            arm_value.data = 55.0
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)                    
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)
                        
                            # set_pose_arm.pose[:] = array('f')

                            # # Set the pose values
                            # set_pose_arm.pose.append(self.node.arm_current_pose[0])
                            # set_pose_arm.pose.append(self.node.arm_current_pose[1])
                            # set_pose_arm.pose.append(self.node.arm_current_pose[2])
                            # set_pose_arm.pose.append(math.radians(-29.6)) #### COLOCAR GARRA DE LADO
                            # set_pose_arm.pose.append(math.radians(87.7))
                            # set_pose_arm.pose.append(math.radians(150.7))

                            # # Publish the pose
                            # self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            # print('Desired pose:', set_pose_arm)

                            # self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            # self.set_rgb(command=GREEN+BLINK_QUICK)
                            # time.sleep(1)

                            arm_value.data = -150.0
                            self.node.arm_value_publisher.publish(arm_value)
                            print(arm_value)                    
                            self.set_arm(command="go_back", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)






                            

                            print('left_side:', left_side)
                            print('right_side:', right_side)

                            if right_side == True:
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.1, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_arm(command="open_door_pull_handler_right", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)
                            
                            elif left_side == True:
                                self.set_arm(command="open_door_pull_handler_left", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.1, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_arm(command="open_door_pull_handler_left_2", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_navigation(movement="orientate", absolute_angle=-90.0, flag_not_obs=True, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)


                            self.set_arm(command="go_initial_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="close_gripper", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)



                        while True:
                            pass
                                
    def open_house_door_push(self):
        if self.node.first_depth_image_received:
            print('bbb')

            self.set_neck(position=self.look_forward, wait_for_end_of=True)

            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'):
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')
                        # set_pose_arm = ListOfFloats()


                        objects = []
                        seen_names = set()

                        finished_detections = False

                        list_of_neck_position_search = [[0, 0], [10, 0], [-10, 0], [0, -10], [0, 10]]
                        self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)
                        while not finished_detections:
                            
                            for pos in list_of_neck_position_search:

                                print(pos)
                                new_neck_pos = [self.look_forward[0] + pos[0], self.look_forward[1] + pos[1]]
                                #new_neck_pos = [ pos[0],  pos[1]]
                                print('Neck: ', new_neck_pos)
                                self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                                time.sleep(1)
                                for obj in self.node.detected_doors.objects:
                                    print(obj.object_name)
                                    if obj.object_name == "Level Handler" or obj.object_name == "Door":
                                        if obj.object_name not in seen_names:
                                            objects.append(obj)
                                            seen_names.add(obj.object_name)
                                            if obj.object_name == "Level Handler":
                                                handler = obj
                                                print(handler.object_name)
                                            elif obj.object_name == "Door":
                                                door = obj
                                                print(door.object_name)
                                            print('é isto')
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                                    
                                        if len(objects) == 2:
                                            finished_detections = True
                                    if finished_detections == True:
                                        break
                                if finished_detections == True:
                                        break
                                

                        print('\n\n\n')
                        print('posição abs:', objects[0].position_absolute, 'da ', objects[0].object_name)
                        print('posição abs:', objects[1].position_absolute, 'da ', objects[1].object_name)
                        print('\n\n\n')
                        

                        if objects != []:
                            set_pose_arm = ListOfFloats()
                            door_location = self.transform(door)
                            handler_location = self.transform(handler)
                            #Value of height I want the arm to go to not touch in shelfs:
                            
                            # desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            # new_height = Float32()
                            # new_height.data = desired_height
                            
                            door_x = door_location[0]
                            door_y = door_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            door_z = door_location[2]

                            print('x y e z da porta:',door_x, door_y, door_z)   

                            handler_x = handler_location[0]
                            handler_y = handler_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            handler_z = handler_location[2]

                            print('x y e z do handler:',handler_x, handler_y, handler_z)
                            right_side = False
                            left_side = False

                            if handler.box_center_x > door.box_top_left_x and handler.box_center_y > door.box_top_left_y:
                                if handler.box_center_x < door.box_top_left_x + door.box_width and handler.box_center_y < door.box_top_left_y + door.box_height:
                                    print('handler inside door')
                                    if handler.box_center_x > door.box_center_x:
                                        print('right side of door')
                                        right_side = True
                                        left_side = False
                                    else:
                                        print('left side of door')
                                        right_side = False
                                        left_side = True

                            if handler_z < 0.0: # handler está à minha esquerda
                                adjust_x = abs(handler_z)/1000
                                print('Adjusting to the left in ', adjust_x, 'meters')
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_x, adjust_direction=90.0, wait_for_end_of=True)
                                time.sleep(1)
                            elif handler_z > 0.0: # handler está à minha drt
                                adjust_x = abs(handler_z)/1000
                                print('Adjusting to the right in ', adjust_x, 'meters')
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=adjust_x - 0.2, adjust_direction=-90.0 + 360.0, wait_for_end_of=True)
                                time.sleep(1)
                            
                            ### NAVEGAR PARA O HANDLER

                            if door_z > 0.0 and handler_z > 0.0:

                                center_door = abs(door_z)/1000 - abs(handler_z)/1000
                                center_door = abs(center_door) - 0.15
                            
                            elif handler_z > 0.0 and door_z < 0.0:

                                center_door = abs(door_z)/1000 + abs(handler_z)/1000
                                center_door = abs(center_door) - 0.15

                            elif handler_z < 0.0 and door_z > 0.0:

                                center_door = abs(door_z)/1000 + abs(handler_z)/1000
                                center_door = abs(center_door) - 0.15

                            elif door_z < 0.0 and handler_z < 0.0:
                                center_door = abs(door_z)/1000 - abs(handler_z)/1000
                                center_door = abs(center_door) - 0.15

                            print('Mais tarde tenho de me mover em: ', center_door)

                            navigate_x = ( abs(handler_x) - 850.0 ) / 1000

                            print('I must navigate for: ', navigate_x, 'm to be at 0.7m from door')
                            self.set_rgb(command=GREEN+BLINK_QUICK)

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=navigate_x, adjust_direction=0.0, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)





                            ### centrar com handler
                            self.set_arm(command="orient_to_door_front", wait_for_end_of=True)
                            time.sleep(1)
                            
                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            # Set the pose values
                            set_pose_arm.pose.append(self.node.arm_current_pose[0])
                            set_pose_arm.pose.append(handler_y + 150.0)
                            set_pose_arm.pose.append(90.0)
                            set_pose_arm.pose.append(self.node.arm_current_pose[3])
                            set_pose_arm.pose.append(self.node.arm_current_pose[4])
                            set_pose_arm.pose.append(self.node.arm_current_pose[5])

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print('Desired pose:', set_pose_arm)
                            time.sleep(1)

                            self.activate_yolo_objects(activate_doors=False, activate_doors_hand = False)

                            self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            time.sleep(1)

                            self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)

                            self.set_neck(position=self.look_down, wait_for_end_of=True)
                            time.sleep(1)

                            self.set_rgb(command=WHITE+HALF_ROTATE)

                            while not self.node.first_depth_image_hand_received:
                                pass
                            
                            self.node.new_image_hand_flag = False
                            while self.node.new_image_hand_flag == False:
                                pass
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                                
                        finished_detections = False

                        time.sleep(1)

                        if hasattr(self.node.detected_doors, 'objects'):
                            print('ddd')
                            if self.node.detected_doors_hand.objects:
                                while not finished_detections:
                                    for obj in self.node.detected_doors_hand.objects:
                                        print(obj.object_name)
                                        if obj.object_name == "Level Handler":
                                            handler = obj
                                            print(handler.object_name)
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                            
                                            finished_detections = True
                                        if finished_detections == True:
                                            break
                                    if finished_detections == True:
                                        break
                        
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            self.activate_yolo_objects(activate_doors=False, activate_doors_hand = False)
                            arm_value = Float32()
                            arm_value.data = handler.position_relative.x * 1000
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go right: ', arm_value)                    
                            self.set_arm(command="go_left", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            # arm_value = Float32()
                            arm_value.data = (handler.position_relative.z * 1000) + 65.0
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go down: ', arm_value)                   
                            self.set_arm(command="go_down", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            if right_side == True:

                                # Set the pose values
                                set_pose_arm.pose.append(self.node.arm_current_pose[0])
                                set_pose_arm.pose.append(self.node.arm_current_pose[1])
                                set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(math.radians(-29.6)) #### COLOCAR GARRA DE LADO
                                set_pose_arm.pose.append(math.radians(87.7))
                                set_pose_arm.pose.append(math.radians(150.7))

                            elif left_side == True:
                                # Set the pose values
                                set_pose_arm.pose.append(self.node.arm_current_pose[0])
                                set_pose_arm.pose.append(self.node.arm_current_pose[1])
                                set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(math.radians(42.2)) #### COLOCAR GARRA DE LADO SEM BATER NA PORTA
                                set_pose_arm.pose.append(math.radians(-87.3))
                                set_pose_arm.pose.append(math.radians(-41.8))

                            # Publish the pose
                            self.node.arm_set_pose_publisher.publish(set_pose_arm)
                            print('Desired pose:', set_pose_arm)
                            time.sleep(1)

                            self.set_arm(command="change_position_arm", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="open_gripper", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            # arm_value = Float32()
                            arm_value.data = handler.position_relative.y * 1000 - 130
                            self.node.arm_value_publisher.publish(arm_value)
                            print('Go front: ', arm_value)  
                            self.set_arm(command="go_front", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="get_arm_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)
                        
                            set_pose_arm.pose[:] = array('f')

                            if right_side == True:

                                # Set the pose values
                                set_pose_arm.pose.append(self.node.arm_current_pose[0])
                                set_pose_arm.pose.append(self.node.arm_current_pose[1])
                                set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(math.radians(-67.0)) #### COLOCAR GARRA DE LADO
                                set_pose_arm.pose.append(math.radians(81.9))
                                set_pose_arm.pose.append(math.radians(105.3))

                                # Publish the pose
                                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                                print('Desired pose:', set_pose_arm)

                                self.set_arm(command="change_position_arm", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                arm_value = Float32()
                                arm_value.data = -60.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)                    
                                self.set_arm(command="go_right", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.15, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                # self.set_arm(command="get_arm_position", wait_for_end_of=True)
                                # self.set_rgb(command=GREEN+BLINK_QUICK)
                                # time.sleep(1)

                                arm_value.data = 60.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)                    
                                self.set_arm(command="go_left", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_arm(command="open_door_push", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)


                            elif left_side == True:
                                # Set the pose values
                                set_pose_arm.pose.append(self.node.arm_current_pose[0])
                                set_pose_arm.pose.append(self.node.arm_current_pose[1])
                                set_pose_arm.pose.append(self.node.arm_current_pose[2])
                                set_pose_arm.pose.append(math.radians(66.6)) #### COLOCAR GARRA DE LADO SEM BATER NA PORTA
                                set_pose_arm.pose.append(math.radians(-81.9))
                                set_pose_arm.pose.append(math.radians(-74.4))

                                # Publish the pose
                                self.node.arm_set_pose_publisher.publish(set_pose_arm)
                                print('Desired pose:', set_pose_arm)

                                self.set_arm(command="change_position_arm", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                arm_value = Float32()
                                arm_value.data = 50.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)                    
                                self.set_arm(command="go_right", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.15, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                                # self.set_arm(command="get_arm_position", wait_for_end_of=True)
                                # self.set_rgb(command=GREEN+BLINK_QUICK)
                                # time.sleep(1)

                                arm_value.data = -50.0
                                self.node.arm_value_publisher.publish(arm_value)
                                print(arm_value)                    
                                self.set_arm(command="go_left", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                            # arm_value.data = -150.0
                            # self.node.arm_value_publisher.publish(arm_value)
                            # print(arm_value)                    
                            # self.set_arm(command="go_back", wait_for_end_of=True)
                            # self.set_rgb(command=GREEN+BLINK_QUICK)
                            # time.sleep(1)

                                self.set_arm(command="open_door_push_left", wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                            print('left_side:', left_side)
                            print('right_side:', right_side)

                            if right_side == True:
                                print('I must navigate to the left in: ', center_door)
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=center_door, adjust_direction=90.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)


                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.5, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)
                            
                            elif left_side == True:
                                print('I must navigate to the right in: ', center_door)
                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=center_door, adjust_direction=-90.0 + 360.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)


                                self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.5, adjust_direction=0.0, wait_for_end_of=True)
                                self.set_rgb(command=GREEN+BLINK_QUICK)
                                time.sleep(1)

                            self.set_arm(command="go_initial_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="close_gripper", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)



                        while True:
                            pass
                                
    def close_house_door_push_right_side(self):
        if self.node.first_depth_image_received:
            print('bbb')

            self.set_neck(position=self.look_right, wait_for_end_of=True)

            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'):
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')
                        # set_pose_arm = ListOfFloats()


                        objects = []
                        seen_names = set()

                        finished_detections = False

                        list_of_neck_position_search = [[0, 0], [10, 0], [-10, 0], [0, -10], [0, 10]]
                        self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)
                        while not finished_detections:
                            
                            for pos in list_of_neck_position_search:

                                print(pos)
                                new_neck_pos = [self.look_right[0] + pos[0], self.look_right[1] + pos[1]]
                                #new_neck_pos = [ pos[0],  pos[1]]
                                print('Neck: ', new_neck_pos)
                                self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                                time.sleep(1)
                                for obj in self.node.detected_doors.objects:
                                    print(obj.object_name)
                                    if obj.object_name == "Level Handler":
                                        if obj.object_name not in seen_names:
                                            objects.append(obj)
                                            seen_names.add(obj.object_name)
                                            if obj.object_name == "Level Handler":
                                                handler = obj
                                                print(handler.object_name)
                                            print('é isto')
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                                    
                                        if len(objects) == 1:
                                            finished_detections = True
                                    if finished_detections == True:
                                        break
                                if finished_detections == True:
                                        break
                                

                        print('\n\n\n')
                        print('posição abs:', objects[0].position_absolute, 'da ', objects[0].object_name)
                        print('\n\n\n')
                        

                        if objects != []:
                            set_pose_arm = ListOfFloats()
                            # door_location = self.transform(door)
                            handler_location = self.transform(handler)
                            #Value of height I want the arm to go to not touch in shelfs:
                            
                            # desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            # new_height = Float32()
                            # new_height.data = desired_height
                            
                            # door_x = door_location[0]
                            # door_y = door_location[1]
                            # door_z = door_location[2]

                            # print('x y e z da porta:',door_x, door_y, door_z)   

                            handler_x = handler_location[0]
                            handler_y = handler_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            handler_z = handler_location[2]

                            print('x y e z do handler:',handler_x, handler_y, handler_z)
                            right_side = False
                            left_side = False
                    
                            ### centrar com handler
                            self.set_arm(command="orient_to_door_front", wait_for_end_of=True)
                            time.sleep(1)

                            self.set_arm(command="close_door_push_right_side", wait_for_end_of=True)
                            time.sleep(1)

                            self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.35, adjust_direction=0.0, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="go_initial_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="close_gripper", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)



                        while True:
                            pass

    def close_house_door_push_left_side(self):
        if self.node.first_depth_image_received:
            print('bbb')

            self.set_neck(position=self.look_left, wait_for_end_of=True)

            current_frame_depth_hand = self.node.br.imgmsg_to_cv2(self.node.depth_img, desired_encoding="passthrough")

            if hasattr(self.node.detected_doors, 'image_rgb'):
                head_image = self.node.detected_doors.image_rgb
                print('ccc')
                
                if hasattr(self.node.detected_doors, 'objects'):
                    print('ddd')
                    
                    if self.node.detected_doors.objects:
                        # print(self.node.detected_doors.objects)
                        print('eee')
                        # set_pose_arm = ListOfFloats()


                        objects = []
                        seen_names = set()

                        finished_detections = False

                        list_of_neck_position_search = [[0, 0], [10, 0], [-10, 0], [0, -10], [0, 10]]
                        self.activate_yolo_objects(activate_doors=True, activate_doors_hand = True)
                        while not finished_detections:
                            
                            for pos in list_of_neck_position_search:

                                print(pos)
                                new_neck_pos = [self.look_left[0] + pos[0], self.look_left[1] + pos[1]]
                                #new_neck_pos = [ pos[0],  pos[1]]
                                print('Neck: ', new_neck_pos)
                                self.set_neck(position=new_neck_pos, wait_for_end_of=True)

                                time.sleep(1)
                                for obj in self.node.detected_doors.objects:
                                    print(obj.object_name)
                                    if obj.object_name == "Level Handler":
                                        if obj.object_name not in seen_names:
                                            objects.append(obj)
                                            seen_names.add(obj.object_name)
                                            if obj.object_name == "Level Handler":
                                                handler = obj
                                                print(handler.object_name)
                                            print('é isto')
                                            print('posição absoluta', obj.position_absolute)
                                            print('posição relativa', obj.position_relative)
                                            print(obj.confidence)
                                                    
                                        if len(objects) == 1:
                                            finished_detections = True
                                    if finished_detections == True:
                                        break
                                if finished_detections == True:
                                        break
                                

                        print('\n\n\n')
                        print('posição abs:', objects[0].position_absolute, 'da ', objects[0].object_name)
                        print('\n\n\n')
                        

                        if objects != []:
                            set_pose_arm = ListOfFloats()
                            # door_location = self.transform(door)
                            handler_location = self.transform(handler)
                            #Value of height I want the arm to go to not touch in shelfs:
                            
                            # desired_height = 1100.0 - (self.node.third_shelf_height - 0.2) * 1000
                            # new_height = Float32()
                            # new_height.data = desired_height
                            
                            # door_x = door_location[0]
                            # door_y = door_location[1]
                            # door_z = door_location[2]

                            # print('x y e z da porta:',door_x, door_y, door_z)   

                            handler_x = handler_location[0]
                            handler_y = handler_location[1]
                            # door_y = object_location[1] + 100.0
                            # door_y = 350.0
                            handler_z = handler_location[2]

                            print('x y e z do handler:',handler_x, handler_y, handler_z)
                            right_side = False
                            left_side = False
                    
                            ### centrar com handler
                            self.set_arm(command="orient_to_door_front", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            self.set_arm(command="close_door_push_left_side", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            print('movement door done')

                            self.set_navigation(movement="orientate", absolute_angle=-90.0, flag_not_obs=True, wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)

                            print('oriented')

                            # self.set_navigation(movement="adjust", flag_not_obs=True, adjust_distance=0.35, adjust_direction=0.0, wait_for_end_of=True)
                            # self.set_rgb(command=GREEN+BLINK_QUICK)
                            # time.sleep(1)

                            self.set_arm(command="go_initial_position", wait_for_end_of=True)
                            self.set_rgb(command=GREEN+BLINK_QUICK)
                            time.sleep(1)


                        while True:
                            pass
     

    def search_for_person(self, tetas, delta_t=3.0):

        self.activate_yolo_pose(activate=True, characteristics=False, only_detect_person_arm_raised=False, only_detect_person_legs_visible=False)                
        self.set_rgb(WHITE+ALTERNATE_QUARTERS)
        time.sleep(0.5)
        
        total_person_detected = []
        person_detected = []
        people_ctr = 0

        ### MOVES NECK AND SAVES DETECTED PEOPLE ###
        
        for t in tetas:
            self.set_rgb(RED+SET_COLOUR)
            self.set_neck(position=t, wait_for_end_of=True)
            time.sleep(1.0) # 0.5
            self.set_rgb(WHITE+SET_COLOUR)

            start_time = time.time()
            while (time.time() - start_time) < delta_t:        
                local_detected_people = self.node.detected_people
                for temp_people in local_detected_people.persons:
                    
                    is_already_in_list = False
                    person_already_in_list = DetectedPerson()
                    for people in person_detected:

                        if temp_people.index == people.index:
                            is_already_in_list = True
                            person_already_in_list = people

                    if is_already_in_list:
                        person_detected.remove(person_already_in_list)
                    elif temp_people.index > 0: # debug
                        # print("added_first_time", temp_people.index, temp_people.position_absolute.x, temp_people.position_absolute.y)
                        self.set_rgb(GREEN+SET_COLOUR)
                    
                    if temp_people.index > 0:
                        person_detected.append(temp_people)
                        people_ctr+=1

            # DEBUG
            # print("people in this neck pos:")
            # for people in person_detected:
            #     print(people.index, people.position_absolute.x, people.position_absolute.y)
        
            total_person_detected.append(person_detected.copy())
            # print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()          

        self.activate_yolo_pose(activate=False)
        # print(total_person_detected)

        # DEBUG
        # print("TOTAL people in this neck pos:")
        # for frame in total_person_detected:
        #     for people in frame:    
        #         print(people.index, people.position_absolute.x, people.position_absolute.y)
        #     print("-")

        ### DETECTS ALL THE PEOPLE SHOW IN EVERY FRAME ###
        
        filtered_persons = []

        for frame in range(len(total_person_detected)):

            to_append = []
            to_remove = []

            if not len(filtered_persons):
                # print("NO PEOPLE", frame)
                for person in range(len(total_person_detected[frame])):
                    to_append.append(total_person_detected[frame][person])
            else:
                # print("YES PEOPLE", frame)

                MIN_DIST = 1.0 # maximum distance for the robot to assume it is the same person

                for person in range(len(total_person_detected[frame])):
                    same_person_ctr = 0

                    for filtered in range(len(filtered_persons)):

                        dist = math.dist((total_person_detected[frame][person].position_absolute.x, total_person_detected[frame][person].position_absolute.y), (filtered_persons[filtered].position_absolute.x, filtered_persons[filtered].position_absolute.y))
                        # print("new:", total_person_detected[frame][person].index, "old:", filtered_persons[filtered].index, dist)
                        
                        if dist < MIN_DIST:
                            same_person_ctr+=1
                            same_person_old = filtered_persons[filtered]
                            same_person_new = total_person_detected[frame][person]
                            # print("SAME PERSON")                        
                    
                    if same_person_ctr > 0:

                        same_person_old_distance_center = abs(1280/2 - same_person_old.body_center_x) 
                        same_person_new_distance_center = abs(1280/2 - same_person_new.body_center_x) 

                        # print("OLD (pixel):", same_person_old.body_center_x, same_person_old_distance_center)
                        # print("NEW (pixel):", same_person_new.body_center_x, same_person_new_distance_center)

                        if same_person_new_distance_center < same_person_old_distance_center: # person from newer frame is more centered with camera center
                            to_remove.append(same_person_old)
                            to_append.append(same_person_new)
                        else: # person from older frame is more centered with camera center
                            pass # that person is already in the filtered list so we do not have to do anything, this is here just for explanation purposes 

                    else:
                        to_append.append(total_person_detected[frame][person])

            for p in to_remove:
                if p in filtered_persons:
                    # print("REMOVED: ", p.index)
                    filtered_persons.remove(p)
                # else:
                    # print("TRIED TO REMOVE TWICE THE SAME PERSON")
            to_remove.clear()  

            for p in to_append:
                # print("ADDED: ", p.index)
                filtered_persons.append(p)
            to_append.clear()

        # print("FILTERED:")
        # for p in filtered_persons:
        #     print(p.index)

        return filtered_persons


    def detected_person_to_face_path(self, person, send_to_face):

        current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S "))
        
        cf = self.node.br.imgmsg_to_cv2(person.image_rgb_frame, "bgr8")
        just_person_image = cf[person.box_top_left_y:person.box_top_left_y+person.box_height, person.box_top_left_x:person.box_top_left_x+person.box_width]
        # cv2.imshow("Search for Person", just_person_image)
        # cv2.waitKey(100)
        cv2.imwrite(self.node.complete_path_custom_face + current_datetime + str(person.index) + ".jpg", just_person_image) 
        time.sleep(0.5)
        
        if send_to_face:
            self.set_face(custom=current_datetime + str(person.index))
        
        return current_datetime + str(person.index)
