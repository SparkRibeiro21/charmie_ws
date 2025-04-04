#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Float32
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import LaserScan
from charmie_interfaces.msg import NeckPosition, ListOfPoints, Obstacles, ObstacleInfo, PointCloudCoordinates
from charmie_interfaces.srv import ActivateObstacles

import cv2
import numpy as np
import math
from pathlib import Path
import json
import time


class Robot():
    def __init__(self):
        print("New Robot Class Initialised")

        self.DEBUG_DRAW_IMAGE = False # debug drawing opencv
        self.DEBUG_DRAW_IMAGE_OVERALL = False
        self.DEBUG_DRAW_JUST_CALCULATION_POINTS = True
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.test_image_ = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.scale = 0.120*1000
        self.xx_shift = -110
        self.yy_shift = -370

        self.xc_adj = self.xc - self.xx_shift
        self.yc_adj = self.yc - self.yy_shift

        self.robot_radius = 0.560/2 # meter
        self.lidar_radius = 0.050/2 # meter
        self.lidar_to_robot_center = 0.255
        self.neck_visual_lines_length = 1.0

        self.robot_pose = Pose2D()

        self.MAX_OBS_DISTANCE = 2.5
        self.OBSTACLE_RADIUS_THRESHOLD = 0.05

        n_lines = 70
        start_angle = 0
        end_angle = 180
        lateral_corr_angle = 20

        step_size = (math.radians(end_angle-lateral_corr_angle) - math.radians(start_angle+lateral_corr_angle)) / (n_lines - 1)  # Calculate the step size
        self.values = [math.radians(start_angle+lateral_corr_angle) + i * step_size for i in range(n_lines)]
        self.D_TETA = 0.0

        # shifts in displacement so camera poitns meet lidar points
        self.X_SHIFT = -100
        self.Y_SHIFT = 50
        self.Z_SHIFT = 0
        
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/configuration_files"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # Open all configuration files
        try:
            with open(self.complete_path + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)

            with open(self.complete_path + 'furniture.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)

        except:
            print("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")

        self.neck_pan = 0.0
        self.neck_tilt = 0.0
        
        self.ACTIVATE_LIDAR_UP = True
        self.ACTIVATE_LIDAR_BOTTOM = False
        self.ACTIVATE_CAMERA_HEAD = False

        self.scan = LaserScan()
        self.valores_dict = {}
        self.lidar_obstacle_points = []
        self.lidar_obstacle_points_rel = []
        self.lidar_obstacle_points_rel_draw = []

        self.camera_obstacle_points = []
        self.camera_obstacle_points_rel = []
        self.camera_obstacle_points_rel_draw = []

        self.final_obstacle_points = []
        self.final_obstacle_just_lidar_points = [[self.MAX_OBS_DISTANCE * math.cos(v)*1.1, self.MAX_OBS_DISTANCE * math.sin(v)*1.1] for v in self.values]
        self.final_obstacle_just_camera_points = [[self.MAX_OBS_DISTANCE * math.cos(v)*1.05, self.MAX_OBS_DISTANCE * math.sin(v)*1.05] for v in self.values]
        
        self.NORTE = 338.0
        self.imu_orientation_norm_rad = 0.0

        self.linhas = 720
        self.colunas = 1280
        self.current_frame = np.zeros((self.linhas, self.colunas,3), dtype=np.uint8)


    # def pose2d_msg_to_position(self, pose: Pose2D):
        
        # self.robot_pose.x = pose.x
        # self.robot_pose.y = pose.y
        # self.robot_pose.theta = pose.theta

    
    def update_debug_drawings(self):
            
        if self.DEBUG_DRAW_IMAGE_OVERALL:

            ### DRAWS REFERENCE 1 METER LINES ###
            for i in range(20):
                # 1 meter lines horizontal and vertical
                if i == 0:
                    cv2.line(self.test_image, (int(self.xc_adj - self.scale*i), 0), (int(self.xc_adj - self.scale*i), self.xc*2), (0, 0, 255), 1)
                    cv2.line(self.test_image, (0, int(self.yc_adj - self.scale*i)), (self.yc*2, int(self.yc_adj - self.scale*i)), (0, 0, 255), 1)
                else:
                    cv2.line(self.test_image, (int(self.xc_adj + self.scale*i), 0), (int(self.xc_adj + self.scale*i), self.xc*2), (255, 255, 255), 1)
                    cv2.line(self.test_image, (int(self.xc_adj - self.scale*i), 0), (int(self.xc_adj - self.scale*i), self.xc*2), (255, 255, 255), 1)
                    cv2.line(self.test_image, (0, int(self.yc_adj - self.scale*i)), (self.yc*2, int(self.yc_adj - self.scale*i)), (255, 255, 255), 1)
                    cv2.line(self.test_image, (0, int(self.yc_adj + self.scale*i)), (self.yc*2, int(self.yc_adj + self.scale*i)), (255, 255, 255), 1)

            ### DRAWS THE HOUSE FURNITURE ###
            for furniture in self.house_furniture:
                cv2.rectangle(self.test_image, 
                            (int(self.xc_adj + self.scale*furniture['top_left_coords'][0]) , int(self.yc_adj - self.scale*furniture['top_left_coords'][1])),
                            (int(self.xc_adj + self.scale*furniture['bot_right_coords'][0]), int(self.yc_adj - self.scale*furniture['bot_right_coords'][1])),
                            (120,0,120), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*furniture['top_left_coords'][0]) , int(self.yc_adj - self.scale*furniture['top_left_coords'][1])), 6, (255,0,0), -1)
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*furniture['bot_right_coords'][0]), int(self.yc_adj - self.scale*furniture['bot_right_coords'][1])), 6, (0,0,255), -1)

                            
            ### DRAWS THE HOUSE WALLS ###
            for room in self.house_rooms:
                cv2.rectangle(self.test_image, 
                            (int(self.xc_adj + self.scale*room['top_left_coords'][0]) , int(self.yc_adj - self.scale*room['top_left_coords'][1])),
                            (int(self.xc_adj + self.scale*room['bot_right_coords'][0]), int(self.yc_adj - self.scale*room['bot_right_coords'][1])),
                            (255,0,255), 3)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*room['top_left_coords'][0]) , int(self.yc_adj - self.scale*room['top_left_coords'][1])), 6, (255,0,0), -1)
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*room['bot_right_coords'][0]), int(self.yc_adj - self.scale*room['bot_right_coords'][1])), 6, (0,0,255), -1)
            
            ### ROBOT
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_pose.x), int(self.yc_adj - self.scale * self.robot_pose.y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_pose.x), int(self.yc_adj - self.scale * self.robot_pose.y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_pose.x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                         int(self.yc_adj - self.scale*self.robot_pose.y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
            
            # NECK DIRECTION, CAMERA FOV
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_pose.x), int(self.yc_adj - self.scale * self.robot_pose.y)), 
                     (int(self.xc_adj + self.scale*self.robot_pose.x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_pose.theta + self.neck_pan + math.pi/2 - math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_pose.y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_pose.theta + self.neck_pan + math.pi/2 - math.pi/4))), (0,255,255), 1)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_pose.x), int(self.yc_adj - self.scale * self.robot_pose.y)), 
                     (int(self.xc_adj + self.scale*self.robot_pose.x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_pose.theta + self.neck_pan + math.pi/2 + math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_pose.y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_pose.theta + self.neck_pan + math.pi/2 + math.pi/4))), (0,255,255), 1)
                       
            for points in self.lidar_obstacle_points:

                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                        int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                        3, (0, 0, 255), -1)


            for points in self.camera_obstacle_points:

                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                        int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                        3, (255, 0, 0), -1)

            cv2.imshow("Person Localization", self.test_image)
            # cv2.imshow("SDNL", self.image_plt)
            
            k = cv2.waitKey(1)
            if k == ord('+'):
                self.scale /= 0.8
            if k == ord('-'):
                self.scale *= 0.8

            self.test_image[:, :] = 0

    def update_debug_drawings2(self):

        # corpo lidar
        cv2.circle(self.test_image_, (self.xc, int(self.yc-self.lidar_to_robot_center*self.scale)), (int)(self.lidar_radius*self.scale), (0, 0, 255), 1)
        # centro robo
        cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.robot_radius*self.scale/10), (0, 0, 255), 1)
        # corpo robo
        cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.robot_radius*self.scale), (0, 0, 255), 1)
        

        if self.DEBUG_DRAW_JUST_CALCULATION_POINTS:
            for points in self.lidar_obstacle_points_rel:
                cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                            int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                            int(self.OBSTACLE_RADIUS_THRESHOLD*self.scale), (0, 0, 255), 1)


            for points in self.camera_obstacle_points_rel:
                cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                            int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                            int(self.OBSTACLE_RADIUS_THRESHOLD*self.scale), (255, 0, 0), 1)

        else:
            for points in self.lidar_obstacle_points_rel_draw:
                cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                            int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                            int(self.OBSTACLE_RADIUS_THRESHOLD*self.scale), (0, 0, 255), 1)


            for points in self.camera_obstacle_points_rel_draw:
                cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_pose.theta + math.pi/2)),
                                            int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_pose.theta + math.pi/2))),
                                            int(self.OBSTACLE_RADIUS_THRESHOLD*self.scale), (255, 0, 0), 1)

    def calculate_obstacle_points(self, sensor="none"):

        # init_time = time.time()
        # print(self.values)

        self.final_obstacle_points.clear()

        for v in self.values:
            if self.DEBUG_DRAW_IMAGE:
                cv2.line(self.test_image_, (self.xc, self.yc), 
                                            (int(self.xc + self.scale * self.MAX_OBS_DISTANCE * math.cos(v)),
                                            int(self.yc - self.scale * self.MAX_OBS_DISTANCE * math.sin(v))),
                                            (255, 255, 255))
            
            m ,c = self.get_line_from_points(0.0, 0.0, self.MAX_OBS_DISTANCE * math.cos(v), self.MAX_OBS_DISTANCE * math.sin(v))
            # print("line", m, c)
                
            max_dist = self.MAX_OBS_DISTANCE

            if self.ACTIVATE_LIDAR_UP and sensor == "lidar_up":

                self.final_obstacle_just_lidar_points[self.values.index(v)] = [self.MAX_OBS_DISTANCE * math.cos(v)*1.1, self.MAX_OBS_DISTANCE * math.sin(v)*1.1]

                for p in self.lidar_obstacle_points_rel:
                    # get line from two points
                    # print("circle", p.x, p.y)
                    inter_p = self.find_intersection_circle_line(p.x, p.y, self.OBSTACLE_RADIUS_THRESHOLD, m, c)
                    # print("inter_p", inter_p)

                    t = np.sqrt(inter_p[0]**2 + inter_p[1]**2)
                    if t < max_dist and t > 0 and inter_p[1] > 0:
                        max_dist = t
                        closes_inter = inter_p
                
                max_dist_camera_head = np.sqrt(self.final_obstacle_just_camera_points[self.values.index(v)][0]**2 + self.final_obstacle_just_camera_points[self.values.index(v)][1]**2)
                if max_dist < self.MAX_OBS_DISTANCE or max_dist_camera_head < self.MAX_OBS_DISTANCE:
                    if max_dist < max_dist_camera_head:
                        self.final_obstacle_points.append(closes_inter)
                    else:
                        self.final_obstacle_points.append(self.final_obstacle_just_camera_points[self.values.index(v)])

                if max_dist < self.MAX_OBS_DISTANCE:
                    self.final_obstacle_just_lidar_points[self.values.index(v)] = closes_inter
                else:
                    self.final_obstacle_just_lidar_points[self.values.index(v)] =[self.MAX_OBS_DISTANCE * math.cos(v)*1.1, self.MAX_OBS_DISTANCE * math.sin(v)*1.1]
            

            elif self.ACTIVATE_CAMERA_HEAD and sensor == "camera_head":

                self.final_obstacle_just_camera_points[self.values.index(v)] = [self.MAX_OBS_DISTANCE * math.cos(v)*1.05, self.MAX_OBS_DISTANCE * math.sin(v)*1.05]

                for p in self.camera_obstacle_points_rel:
                    # get line from two points
                    # print("circle", p.x, p.y)
                    inter_p = self.find_intersection_circle_line(p.x, p.y, self.OBSTACLE_RADIUS_THRESHOLD, m, c)
                    # print("inter_p", inter_p)

                    t = np.sqrt(inter_p[0]**2 + inter_p[1]**2)
                    if t < max_dist and t > 0 and inter_p[1] > 0:
                        max_dist = t
                        closes_inter = inter_p

                max_dist_lidar_up = np.sqrt(self.final_obstacle_just_lidar_points[self.values.index(v)][0]**2 + self.final_obstacle_just_lidar_points[self.values.index(v)][1]**2)
                if max_dist < self.MAX_OBS_DISTANCE or max_dist_lidar_up < self.MAX_OBS_DISTANCE:
                    if max_dist < max_dist_lidar_up:
                        self.final_obstacle_points.append(closes_inter)
                    else:
                        self.final_obstacle_points.append(self.final_obstacle_just_lidar_points[self.values.index(v)])
                        
                if max_dist < self.MAX_OBS_DISTANCE:
                    self.final_obstacle_just_camera_points[self.values.index(v)] = closes_inter
                else:
                    self.final_obstacle_just_camera_points[self.values.index(v)] =[self.MAX_OBS_DISTANCE * math.cos(v)*1.05, self.MAX_OBS_DISTANCE * math.sin(v)*1.05]

        if self.DEBUG_DRAW_IMAGE:    
            for p in self.final_obstacle_points:
                cv2.circle(self.test_image_, (int(self.xc + self.scale * p[0]),
                                              int(self.yc - self.scale * p[1])),
                                              3, (0, 255, 0), -1)
        # print(time.time()-init_time)
        return self.final_obstacle_points

    def update_image_shown(self):

        cv2.imshow("Person Localization 2", self.test_image_)
        
        k = cv2.waitKey(1)
        if k == ord('+'):
            self.scale /= 0.8
        if k == ord('-'):
            self.scale *= 0.8
        
        self.test_image_[:, :] = 0


    def get_line_from_points(self, x1, y1, x2, y2):
        # Check if the line is vertical to avoid division by zero
        if x1 == x2:
            raise ValueError("The line is vertical, slope is undefined.")
        
        # Calculate the slope (m)
        m = (y2 - y1) / (x2 - x1)
        
        # Calculate the y-intercept (c)
        c = y1 - m * x1
        
        return m, c
    
    def find_intersection_circle_line(self, h, k, r, m, y_intercept):
        # Coefficients of the quadratic equation ax^2 + bx + c = 0
        a = 1 + m**2
        b = -2*h + 2*m*(y_intercept - k)
        c = h**2 + (y_intercept - k)**2 - r**2
      
        # Calculate the discriminant
        discriminant = (b**2) - (4*a*c)
        
        if discriminant < 0:
            return [0.0, 0.0]
        elif discriminant == 0:
            # One intersection point
            x = -b / (2*a)
            y = m*x + y_intercept
            return [x, y]
        else:
            # Two intersection points
            x1 = (-b + np.sqrt(discriminant)) / (2*a)
            y1 = m*x1 + y_intercept
            x2 = (-b - np.sqrt(discriminant)) / (2*a)
            y2 = m*x2 + y_intercept

            # Calculate distances to the origin
            dist1 = np.sqrt(x1**2 + y1**2)
            dist2 = np.sqrt(x2**2 + y2**2)
            
            # Return the point closer to the origin
            if dist1 < dist2:
                return [x1, y1]
            else:
                return [x2, y2]


class DebugVisualNode(Node):

    def __init__(self):
        super().__init__("Robot")
        self.get_logger().info("Initialised CHARMIE Debug Visual Node")

        # get neck position
        self.get_neck_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)
        
        # lidar
        self.lidar_subscriber = self.create_subscription(LaserScan, "scan", self.lidar_callback , 10)

        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        # IMU
        self.get_orientation_subscriber = self.create_subscription(Float32, "get_orientation", self.get_orientation_callback, 10)
       
        # Obstacles
        self.obstacles_publisher = self.create_publisher(Obstacles, "obs_lidar", 10)
        # Just for debug_visual draws
        self.camera_head_obstacles_publisher = self.create_publisher(ListOfPoints, "camera_head_obstacles", 10)
        self.final_obstacles_publisher = self.create_publisher(ListOfPoints, "final_obstacles", 10)
        
        # Obstacles Head Camera Depth
        self.obstacles_head_depth_subscriber = self.create_subscription(PointCloudCoordinates, "obstacles_head_depth", self.get_obstacles_head_depth_callback , 10)
    
        ### Services (Clients) ###
        # Point Cloud
        self.activate_obstacles_head_depth_client = self.create_client(ActivateObstacles, "activate_obstacles_head_depth")

        while not self.activate_obstacles_head_depth_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        ### Services (Server) ###
        # Activates Obstacles
        self.server_activate_obstacles = self.create_service(ActivateObstacles, "activate_obstacles", self.callback_activate_obstacles) 

        self.robot = Robot()
        self.prev_time = time.time()

        self.call_activate_head_depth_obstacles_server(obstacles_camera_head=self.robot.ACTIVATE_CAMERA_HEAD)


    def callback_activate_obstacles(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate_lidar_up     # activate lidar from robot body
        # bool activate_lidar_bottom # activate lidar to see floor objects
        # bool activate_camera_head  # activate head camera for 3D obstacles  
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        self.get_logger().info("Received Activate Obstacles %s" %("("+str(request.activate_lidar_up)+", "
                                                                     +str(request.activate_lidar_bottom)+", "
                                                                     +str(request.activate_camera_head)+")"))

        self.robot.ACTIVATE_LIDAR_UP = request.activate_lidar_up
        self.robot.ACTIVATE_LIDAR_BOTTOM = request.activate_lidar_bottom
        self.robot.ACTIVATE_CAMERA_HEAD = request.activate_camera_head

        self.call_activate_head_depth_obstacles_server(obstacles_camera_head=self.robot.ACTIVATE_CAMERA_HEAD)
        
        if not self.robot.ACTIVATE_LIDAR_UP:
            self.robot.final_obstacle_just_lidar_points = [[self.robot.MAX_OBS_DISTANCE * math.cos(v)*1.1, self.robot.MAX_OBS_DISTANCE * math.sin(v)*1.1] for v in self.robot.values]
        if not self.robot.ACTIVATE_CAMERA_HEAD:
            self.robot.final_obstacle_just_camera_points = [[self.robot.MAX_OBS_DISTANCE * math.cos(v)*1.05, self.robot.MAX_OBS_DISTANCE * math.sin(v)*1.05] for v in self.robot.values]

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response
    
    ### ACTIVATE OBSTACLES SERVER FUNCTIONS ###
    def call_activate_head_depth_obstacles_server(self, obstacles_camera_head=False):
        request = ActivateObstacles.Request()
        request.activate_camera_head = obstacles_camera_head
        self.activate_obstacles_head_depth_client.call_async(request)


    def get_orientation_callback(self, orientation: Float32):
        # self.robot.imu_orientation = orientation.data
        imu_orientation_norm = orientation.data - self.robot.NORTE
        if imu_orientation_norm > 180.0:
            imu_orientation_norm -= 360.0
        if imu_orientation_norm < -180.0:
            imu_orientation_norm += 360.0

        self.robot.imu_orientation_norm_rad = math.radians(imu_orientation_norm)
        self.robot.robot_pose.theta = -self.robot.imu_orientation_norm_rad


    def get_obstacles_head_depth_callback(self, pc: PointCloudCoordinates):
        
        # print(pc)
        # init_time = time.time()
        
        self.robot.camera_obstacle_points_rel.clear()
        if self.robot.DEBUG_DRAW_IMAGE:
            self.robot.camera_obstacle_points_rel_draw.clear()
        # if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
        self.robot.camera_obstacle_points.clear()
        
        temp_cam_head_obs = ListOfPoints()
        for p in pc.bbox_point_coords:

            object_rel_pos = Point()
            object_rel_pos.x =  -(p.y+self.robot.Y_SHIFT)/1000
            object_rel_pos.y =   (p.x+self.robot.X_SHIFT)/1000
            object_rel_pos.z =   (p.z+self.robot.Z_SHIFT)/1000

            if object_rel_pos.z >= 0.3 and object_rel_pos.z <= 1.7: # filters the floor and the ceiling
                
                # calculate the absolute position according to the robot localisation
                dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                if dist_obj < self.robot.MAX_OBS_DISTANCE and object_rel_pos.y > 0.0: 

                    self.robot.camera_obstacle_points_rel.append(object_rel_pos)

                    # if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
                    angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                    theta_aux = math.pi/2 - (angle_obj - self.robot.robot_pose.theta)

                    target = Point()
                    target.x = dist_obj * math.cos(theta_aux) + self.robot.robot_pose.x
                    target.y = dist_obj * math.sin(theta_aux) + self.robot.robot_pose.y
                    target.z = object_rel_pos.z

                    self.robot.camera_obstacle_points.append(target)

                if self.robot.DEBUG_DRAW_IMAGE:
                    self.robot.camera_obstacle_points_rel_draw.append(object_rel_pos)

        # remove points without neighbours, too heavy. Must find more efficient solution
        """
        neighbour_filter_distance = 0.2
        to_remove = []
        to_remove_rel = []
        for p in range(len(self.robot.camera_obstacle_points_rel)):
            p_ctr = 0
            for i in range(len(self.robot.camera_obstacle_points_rel)):
                # dist = math.dist((p.x, p.y, p.z),(i.x, i.y, i.z))  
                dist = math.dist((self.robot.camera_obstacle_points_rel[p].x, self.robot.camera_obstacle_points_rel[p].y),
                                    (self.robot.camera_obstacle_points_rel[i].x, self.robot.camera_obstacle_points_rel[i].y))  
                if dist < neighbour_filter_distance:
                    p_ctr +=1
            # print(p_ctr, end='\t')
            if p_ctr < 5:
                to_remove_rel.append(self.robot.camera_obstacle_points_rel[p])
                # if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
                to_remove.append(self.robot.camera_obstacle_points[p])
            
        for p in to_remove_rel:
            self.robot.camera_obstacle_points_rel.remove(p)
        # if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
        for p in to_remove:
            self.robot.camera_obstacle_points.remove(p)
        """

        for p in self.robot.camera_obstacle_points:
            temp_cam_head_obs.coords.append(p)
        self.camera_head_obstacles_publisher.publish(temp_cam_head_obs)

        if self.robot.ACTIVATE_CAMERA_HEAD:
            self.publish_obstacles(sensor="camera_head")
        
        # print("cam len:", len(self.robot.camera_obstacle_points))
        # print("cam:", time.time()-init_time)

    
    def lidar_callback(self, scan: LaserScan):
        self.robot.scan = scan
        
        # print(scan)
        # init_time = time.time()

        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.robot.lidar_obstacle_points_rel.clear()
        if self.robot.DEBUG_DRAW_IMAGE:
            self.robot.lidar_obstacle_points_rel_draw.clear()
        if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
            self.robot.lidar_obstacle_points.clear()

        # calculates list of lidar obstacle points
        for i in range(len(scan.ranges)):
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if value > self.min_dist_error and value < self.max_dist_error:

                object_rel_pos = Point()
                object_rel_pos.x =  -value * math.cos(-key + math.pi/2)
                object_rel_pos.y =  self.robot.lidar_to_robot_center + value * math.sin(-key + math.pi/2)
                object_rel_pos.z =  0.35 # lidar height on the robot
                
                # calculate the absolute position according to the robot localisation
                dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                if dist_obj < self.robot.MAX_OBS_DISTANCE and object_rel_pos.y > 0.0:
                    self.robot.lidar_obstacle_points_rel.append(object_rel_pos)

                if self.robot.DEBUG_DRAW_IMAGE:
                    self.robot.lidar_obstacle_points_rel_draw.append(object_rel_pos)

                if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
                    obs_x = value * math.cos(key + self.robot.robot_pose.theta + math.pi/2)
                    obs_y = value * math.sin(key + self.robot.robot_pose.theta + math.pi/2)

                    adj_x = (self.robot.robot_radius - self.robot.lidar_radius)*math.cos(self.robot.robot_pose.theta + math.pi/2)
                    adj_y = (self.robot.robot_radius - self.robot.lidar_radius)*math.sin(self.robot.robot_pose.theta + math.pi/2)

                    target = Point()
                    target.x = self.robot.robot_pose.x + obs_x + adj_x
                    target.y = self.robot.robot_pose.y + obs_y + adj_y
                    target.z = 0.35 # lidar height on the robot

                    self.robot.lidar_obstacle_points.append(target)

        # self.robot.lidar_obstacle_points_rel = [value for index, value in enumerate(self.robot.lidar_obstacle_points_rel) if (index+1) % 2 == 0]

        if self.robot.ACTIVATE_LIDAR_UP:
            self.publish_obstacles(sensor="lidar_up")

        # print(len(self.robot.lidar_obstacle_points_rel))
        # print("lidar:", time.time()-init_time)
        

    def get_neck_position_callback(self, pose: NeckPosition):
        # print("Received new neck position. PAN = ", pose.pan, " TILT = ", pose.tilt)
        self.robot.neck_pan = -math.radians(- pose.pan)
        self.robot.neck_tilt = -math.radians(- pose.tilt)

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot.robot_pose = pose
        
    def publish_obstacles(self, sensor="none"):       

        if self.robot.DEBUG_DRAW_IMAGE:
            self.robot.update_debug_drawings()
            self.robot.update_debug_drawings2()

        f_p = self.robot.calculate_obstacle_points(sensor=sensor)
        tot_obs = Obstacles()
        tot_obs.no_obstacles = len(f_p)
        temp_lp = ListOfPoints()
        for p in f_p:

            obs_info = ObstacleInfo()
            obs_info.alfa = math.atan2(p[0], p[1])
            obs_info.dist = math.sqrt(p[0]**2 + p[1]**2) - self.robot.robot_radius
            obs_info.length_angular = self.robot.D_TETA
            tot_obs.obstacles.append(obs_info)

            t = Point()
            t.x = p[0]
            t.y = p[1]
            temp_lp.coords.append(t)

        self.obstacles_publisher.publish(tot_obs)
        self.final_obstacles_publisher.publish(temp_lp)

        if self.robot.DEBUG_DRAW_IMAGE:
            self.robot.update_image_shown()

        print("elapsed time =", time.time()-self.prev_time)
        self.prev_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualNode()
    rclpy.spin(node)
    rclpy.shutdown()