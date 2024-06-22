#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, String, Float32
from geometry_msgs.msg import Pose2D, Point
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from charmie_interfaces.msg import Yolov8Pose, Yolov8Objects, DetectedPerson, NeckPosition, ListOfPoints, TarNavSDNL, BoundingBox, BoundingBoxAndPoints, PointCloudCoordinates, ListOfPoints
from charmie_interfaces.srv import GetPointCloud
# from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import math
import threading
from pathlib import Path
import json

DEBUG_DRAW = False


class Robot():
    def __init__(self):
        print("New Robot Class Initialised")

        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.test_image_ = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.scale = 0.063*1000
        self.xx_shift = -110
        self.yy_shift = -370

        self.xc_adj = self.xc - self.xx_shift
        self.yc_adj = self.yc - self.yy_shift

        self.robot_radius = 0.560/2 # meter
        self.lidar_radius = 0.050/2 # meter
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0 # math.pi/4
        # self.neck_hor_angle = math.radians(30)
        # self.neck_ver_angle = 0.0 # NOT USED ...
        self.all_pos_x_val = []
        self.all_pos_y_val = []
        self.all_pos_t_val = []
        self.neck_visual_lines_length = 1.0
        
        self.flag_get_person = False
        self.t_ctr = 0.0
        self.t_ctr2 = 100+1

        self.x_ant = 0.0
        self.y_ant = 0.0

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/configuration_files"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # Open all configuration files
        try:
            with open(self.complete_path + 'rooms_location.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)

            with open(self.complete_path + 'furniture_location.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)

            with open(self.complete_path + 'doors_location.json', encoding='utf-8') as json_file:
                self.house_doors = json.load(json_file)
            # print(self.house_doors)
        except:
            print("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")

        self.neck_pan = 0.0
        self.neck_tilt = 0.0

        self.person_pose = Yolov8Pose()
        self.object_detected = Yolov8Objects()
        self.search_for_person = ListOfPoints()

        self.navigation = TarNavSDNL()
        self.is_navigating = False

        self.scan = LaserScan()
        self.valores_dict = {}

        self.camera_obstacle_points = []
        self.camera_obstacle_points_adjusted = []
        self.camera_obstacle_points_adjusted_relative = []

        self.lidar_obstacle_points = []

        self.NORTE = 319.1 ### ????
        self.imu_orientation_norm_rad = 0.0

        self.linhas = 720
        self.colunas = 1280
        self.current_frame = np.zeros((self.linhas, self.colunas,3), dtype=np.uint8)


    def pose2d_msg_to_position(self, pose: Pose2D):
        
        self.robot_x = pose.x
        self.robot_y = pose.y
        # self.robot_t = pose.theta


    def update_debug_drawings(self):
            
        if self.DEBUG_DRAW_IMAGE:

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
            
            ### DRAWS THE HOUSE DOORS ###
            for door in self.house_doors:
                cv2.line(self.test_image, 
                            (int(self.xc_adj + self.scale*door['top_left_coords'][0]) , int(self.yc_adj - self.scale*door['top_left_coords'][1])),
                            (int(self.xc_adj + self.scale*door['bot_right_coords'][0]), int(self.yc_adj - self.scale*door['bot_right_coords'][1])),
                            (50,0,50), 3)


            ### PRESENT AND PAST LOCATIONS OF ROBOT
            # self.all_pos_x_val.append(self.robot_x)
            # self.all_pos_y_val.append(self.robot_y)
            # self.all_pos_t_val.append(self.robot_t)
            # for i in range(len(self.all_pos_x_val)):
            #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.all_pos_x_val[i]), int(self.yc_adj - self.scale * self.all_pos_y_val[i])), 1, (255, 255, 0), -1)

            ### ROBOT
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                         int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
            
            # NECK DIRECTION, CAMERA FOV
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), 
                     (int(self.xc_adj + self.scale*self.robot_x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_t + self.neck_pan + math.pi/2 - math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_t + self.neck_pan + math.pi/2 - math.pi/4))), (0,255,255), 1)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), 
                     (int(self.xc_adj + self.scale*self.robot_x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_t + self.neck_pan + math.pi/2 + math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_t + self.neck_pan + math.pi/2 + math.pi/4))), (0,255,255), 1)
                       
            # if self.is_navigating:
            #     pass
            
            # if self.navigation.move_or_rotate == "move" or self.navigation.move_or_rotate == "rotate":
            #     cv2.circle(self.test_image, (int(self.xc_adj + self.navigation.target_coordinates.x*self.scale),
            #         int(self.yc_adj - self.navigation.target_coordinates.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (0, 255, 0), -1)
            #     cv2.circle(self.test_image, (int(self.xc_adj + self.navigation.target_coordinates.x*self.scale),
            #         int(self.yc_adj - self.navigation.target_coordinates.y*self.scale)), (int)(self.scale*self.navigation.reached_radius), (0, 255, 0), 1)
            
            for points in self.lidar_obstacle_points:

                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        4, (0, 0, 255), -1)


            for points in self.camera_obstacle_points_adjusted:

                # if points.z > 0.3 and points.z < 1.7:
                    cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                            4, (255, 0, 0), -1)

                # elif points.z >= 1.7:
                #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             4, (255, 255, 255), -1)

            """
            for points in self.camera_obstacle_points:

                object_rel_pos = Point()
                object_rel_pos.x =  -points.y/1000
                object_rel_pos.y =  points.x/1000
                object_rel_pos.z =  points.z/1000
                
                # calculate the absolute position according to the robot localisation
                angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                theta_aux = math.pi/2 - (angle_obj - self.robot_t)

                target_x = dist_obj * math.cos(theta_aux) + self.robot_x
                target_y = dist_obj * math.sin(theta_aux) + self.robot_y
                target_z = object_rel_pos.z



                print(math.degrees(self.robot_t))

                if target_z > 0.3:
                    cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target_x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc_adj - self.scale * target_y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                            4, (255, 0, 0), -1)
                # else:
                #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target_x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * target_y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             2, (255, 100, 100), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
                
            """


            """
            for person in self.search_for_person.coords:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.circle(self.test_image, (int(self.xc_adj + person.x*self.scale),
                    int(self.yc_adj - person.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)
           

            
            for person in self.person_pose.persons:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.circle(self.test_image, (int(self.xc_adj + person.position_absolute.x*self.scale),
                    int(self.yc_adj - person.position_absolute.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 255, 255), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)

            for object in self.object_detected.objects:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.rectangle(self.test_image, 
                              (int(self.xc_adj + object.position_absolute.x*self.scale + self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_absolute.y*self.scale + self.lidar_radius*2*self.scale)),
                              (int(self.xc_adj + object.position_absolute.x*self.scale - self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_absolute.y*self.scale - self.lidar_radius*2*self.scale)),
                              (255, 255, 255), -1)
                               
                # cv2.circle(self.test_image, (int(self.xc_adj + object.position_relative.x*self.scale),
                #     int(self.yc_adj - object.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)
            """

            if DEBUG_DRAW:
                cv2.imshow("Debug Visual - RGB", self.current_frame)

            cv2.imshow("Person Localization", self.test_image)
            # cv2.imshow("SDNL", self.image_plt)
            
            k = cv2.waitKey(1)
            if k == ord('+'):
                self.scale /= 0.8
            if k == ord('-'):
                self.scale *= 0.8
            if k == ord('0'):
                self.all_pos_x_val.clear()
                self.all_pos_y_val.clear()

            self.test_image[:, :] = 0

    def update_debug_drawings2(self):
        
        
        self.lidar_to_robot_center = 0.255
        self.OBS_THRESH = 1.0
            
        """
        # margem obstaculos
        # cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.OBS_THRESH*self.scale), (0, 255, 255), 1)
        # corpo lidar
        cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.lidar_radius*self.scale), (255, 255, 255), 1)
        # centro robo
        cv2.circle(self.test_image_, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale/10), (255, 255, 255), 1)
        # corpo robo
        cv2.circle(self.test_image_, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale), (255, 255, 255), 1)
        



        for key, value in self.valores_dict.items():
            print(value, ":", key, ":", self.valores_dict[key])

            # ### key is the reading ID
            # ### value is the reading angle
            # ### valores_dict[value] is the reading distance
    
            # draws two LIDAR graphs, the circular one and the linear one
            # from right to left
            cv2.line(self.test_image_, (self.xc, self.yc), (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
                                                    int(self.yc - self.scale * value * math.sin(math.radians(math.degrees(-key) + 90)))),
                             (255, 0, 0))
                   
        
        """

        # margem obstaculos
        # cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.OBS_THRESH*self.scale), (0, 255, 255), 1)
        # corpo lidar
        cv2.circle(self.test_image_, (self.xc, int(self.yc-self.lidar_to_robot_center*self.scale)), (int)(self.lidar_radius*self.scale), (0, 0, 255), 1)
        # centro robo
        cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.robot_radius*self.scale/10), (0, 0, 255), 1)
        # corpo robo
        cv2.circle(self.test_image_, (self.xc, self.yc), (int)(self.robot_radius*self.scale), (0, 0, 255), 1)
        

        
        """
        for key, value in self.valores_dict.items():
            print(value, ":", key, ":", self.valores_dict[key])

            # ### key is the reading ID
            # ### value is the reading angle
            # ### valores_dict[value] is the reading distance
    
            # draws two LIDAR graphs, the circular one and the linear one
            # from right to left
            cv2.line(self.test_image_, (self.xc, int(self.yc-self.lidar_to_robot_center*self.scale)), (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
                                                    int(int(self.yc-self.lidar_to_robot_center*self.scale) - self.scale * value * math.sin(math.radians(math.degrees(-key) + 90)))),
                             (0, 255, 0))
                   
        
        for key, value in self.valores_dict.items():
            print(value, ":", key, ":", self.valores_dict[key])

            # ### key is the reading ID
            # ### value is the reading angle
            # ### valores_dict[value] is the reading distance
    
            # draws two LIDAR graphs, the circular one and the linear one
            # from right to left
            cv2.line(self.test_image_, (self.xc, self.yc), (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
                                                    int(int(self.yc-self.lidar_to_robot_center*self.scale) - self.scale * value * math.sin(math.radians(math.degrees(-key) + 90)))),
                             (0, 255, 255))
            
                             
    """
        # for key, value in self.valores_dict.items(): 
        #     cv2.circle(self.test_image_, (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
        #                                             int(int(self.yc-self.lidar_to_robot_center*self.scale) - self.scale * value * math.sin(math.radians(math.degrees(-key) + 90)))),
        #                      4, (255, 255, 255), -1)
        
        for key, value in self.valores_dict.items(): 
            cv2.circle(self.test_image_, (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
                                                    int(self.yc - self.scale * (self.lidar_to_robot_center + value * math.sin(math.radians(math.degrees(-key) + 90))))),
                             5, (0, 0, 255), 1)
        
        

        for points in self.camera_obstacle_points_adjusted_relative:

            # if points.z > 0.3 and points.z < 1.7:
            cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        5, (255, 0, 0), 1)

        
        
        
        cv2.imshow("Person Localization 2", self.test_image_)
        self.test_image_[:, :] = 0







class DebugVisualNode(Node):

    def __init__(self):
        super().__init__("Robot")
        self.get_logger().info("Initialised CHARMIE Debug Visual Node")

        # get neck position
        self.get_neck_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)
        
        # get yolo pose person detection filtered
        self.person_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.get_person_pose_callback, 10)
        self.object_detected_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.get_object_detected_callback, 10)

        # lidar
        self.lidar_subscriber = self.create_subscription(LaserScan, "scan", self.lidar_callback , 10)

        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        # Navigation
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_navigation_reached_callback, 10)  

        # search for person, person localisation 
        self.search_for_person_subscriber = self.create_subscription(ListOfPoints, "search_for_person_points", self.search_for_person_callback, 10)
        
        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)

        # IMU
        self.get_orientation_subscriber = self.create_subscription(Float32, "get_orientation", self.get_orientation_callback, 10)
       
        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        # Camera Obstacles
        # self.temp_camera_obstacles_subscriber = self.create_subscription(ListOfPoints, "camera_obstacles", self.get_camera_obstacles_callback, 10)
       
        self.robot = Robot()

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


    def get_orientation_callback(self, orientation: Float32):
        # self.robot.imu_orientation = orientation.data
        imu_orientation_norm = orientation.data - self.robot.NORTE
        if imu_orientation_norm > 180.0:
            imu_orientation_norm -= 360.0
        if imu_orientation_norm < -180.0:
            imu_orientation_norm += 360.0

        self.robot.imu_orientation_norm_rad = math.radians(imu_orientation_norm)
        self.robot.robot_t = -self.robot.imu_orientation_norm_rad

        
    # def get_camera_obstacles_callback(self, points: ListOfPoints):
    #     self.robot.camera_obstacle_points = points
    #     # print("Received Points")
    #     print
        

    def lidar_callback(self, scan: LaserScan):
        self.robot.scan = scan
        # print(scan)

        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.robot.lidar_obstacle_points.clear()

        for i in range(len(scan.ranges)):
            # print(x)
            # i = i + 1
            # self.valores_id[START_RAD+i*STEP_RAD] = i
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if scan.ranges[i] < self.min_dist_error or scan.ranges[i] > self.max_dist_error:
                scan.ranges[i] = 8.0
            self.robot.valores_dict[START_RAD+i*STEP_RAD] = scan.ranges[i]
            
            
            if value > 0.1: 
                obs_x = value * math.cos(key + self.robot.robot_t + math.pi/2)
                obs_y = value * math.sin(key + self.robot.robot_t + math.pi/2)

                adj_x = (self.robot.robot_radius - self.robot.lidar_radius)*math.cos(self.robot.robot_t + math.pi/2)
                adj_y = (self.robot.robot_radius - self.robot.lidar_radius)*math.sin(self.robot.robot_t + math.pi/2)

                target = Point()
                target.x = self.robot.robot_x + obs_x + adj_x
                target.y = self.robot.robot_y + obs_y + adj_y
                target.z = 0.35 # lidar height on the robot

                self.robot.lidar_obstacle_points.append(target)

        # print(self.robot.valores_dict, "\n")


    def target_pos_callback(self, nav: TarNavSDNL):
        self.robot.navigation = nav
        self.robot.is_navigating = True
        # print(nav)


    def flag_navigation_reached_callback(self, flag: Bool):
        self.robot.is_navigating = False


    def get_neck_position_callback(self, pose: NeckPosition):
        # print("Received new neck position. PAN = ", pose.pan, " TILT = ", pose.tilt)
        self.robot.neck_pan = -math.radians(- pose.pan)
        self.robot.neck_tilt = -math.radians(- pose.tilt)


    def get_person_pose_callback(self, pose: Yolov8Pose):
        # print("Received new yolo pose. Number of people = ", pose.num_person)
        self.robot.person_pose = pose

    def get_object_detected_callback(self, obj: Yolov8Objects):
        # print("Received new yolo objects. Number of objects = ", obj.num_person)
        self.robot.object_detected = obj

    def robot_localisation_callback(self, pose: Pose2D):
        self.robot.robot_x = pose.x
        self.robot.robot_y = pose.y
        # self.robot.robot_t = pose.theta
        
    def search_for_person_callback(self, points: ListOfPoints):
        self.robot.search_for_person = points

    def get_color_image_callback(self, img: Image):
        # self.get_logger().info('Receiving color video frame')
        # ROS2 Image Bridge for OpenCV
        br = CvBridge()
        self.robot.current_frame = br.imgmsg_to_cv2(img, "bgr8")
        
    
def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualNode()
    th_main = threading.Thread(target=thread_main_debug_visual, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_debug_visual(node: DebugVisualNode):
    main = DebugVisualMain(node)
    main.main()

class DebugVisualMain():

    def __init__(self, node: DebugVisualNode):
        self.node = node
        self.state = 0
        self.hand_raised = 0
        self.person_coordinates = Pose2D()
        self.person_coordinates.x = 0.0
        self.person_coordinates.y = 0.0

        self.neck_pose = Pose2D()
        self.neck_pose.x = 180.0
        self.neck_pose.y = 193.0

        self.target_x = 0.0
        self.target_y = 0.0

        self.pedido = ''

        self.i = 0

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

    def main(self):

        while True:
            # pass
            pc = self.get_point_cloud() 

            # self.node.robot.camera_obstacle_points = pc[0].bbox_point_coords

            self.node.robot.camera_obstacle_points_adjusted.clear()
            self.node.robot.camera_obstacle_points_adjusted_relative.clear()
                        
            for p in pc[0].bbox_point_coords:

                object_rel_pos = Point()
                object_rel_pos.x =  -p.y/1000
                object_rel_pos.y =  p.x/1000
                object_rel_pos.z =  p.z/1000

                if object_rel_pos.z >= 0.3 and object_rel_pos.z <= 1.7: # filters the floor and the ceiling
                    
                    # calculate the absolute position according to the robot localisation
                    dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                    if dist_obj < 2.5:

                        angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                        theta_aux = math.pi/2 - (angle_obj - self.node.robot.robot_t)

                        target = Point()
                        target.x = dist_obj * math.cos(theta_aux) + self.node.robot.robot_x
                        target.y = dist_obj * math.sin(theta_aux) + self.node.robot.robot_y
                        target.z = object_rel_pos.z

                        self.node.robot.camera_obstacle_points_adjusted.append(target)
                        self.node.robot.camera_obstacle_points_adjusted_relative.append(object_rel_pos)


            
            # print(len(self.node.robot.camera_obstacle_points_adjusted))

            neighbour_filter_distance = 0.2

            to_remove = []
            to_remove_rel = []
            for p in range(len(self.node.robot.camera_obstacle_points_adjusted)):
                p_ctr = 0
                for i in range(len(self.node.robot.camera_obstacle_points_adjusted)):
                    # dist = math.dist((p.x, p.y, p.z),(i.x, i.y, i.z))  
                    dist = math.dist((self.node.robot.camera_obstacle_points_adjusted[p].x, self.node.robot.camera_obstacle_points_adjusted[p].y),
                                     (self.node.robot.camera_obstacle_points_adjusted[i].x, self.node.robot.camera_obstacle_points_adjusted[i].y))  
                    if dist < neighbour_filter_distance:
                        p_ctr +=1
                # print(p_ctr, end='\t')
                if p_ctr < 5:
                    to_remove.append(self.node.robot.camera_obstacle_points_adjusted[p])
                    to_remove_rel.append(self.node.robot.camera_obstacle_points_adjusted_relative[p])
            
            for p in to_remove:
                self.node.robot.camera_obstacle_points_adjusted.remove(p)
            for p in to_remove_rel:
                self.node.robot.camera_obstacle_points_adjusted_relative.remove(p)
            
            for p in to_remove:
                cv2.circle(self.node.robot.test_image, (int(self.node.robot.xc_adj + self.node.robot.scale * p.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.node.robot.yc_adj - self.node.robot.scale * p.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        5, (255, 255, 255), -1)

                cv2.circle(self.node.robot.test_image, (int(self.node.robot.xc_adj + self.node.robot.scale * p.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.node.robot.yc_adj - self.node.robot.scale * p.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        int(self.node.robot.scale*neighbour_filter_distance), (255, 255, 255), 1)

            for p in to_remove_rel:
            
                cv2.circle(self.node.robot.test_image_, (int(self.node.robot.xc + self.node.robot.scale * p.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.node.robot.yc - self.node.robot.scale * p.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                            5, (255, 255, 255), 1)

                cv2.circle(self.node.robot.test_image_, (int(self.node.robot.xc + self.node.robot.scale * p.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.node.robot.yc - self.node.robot.scale * p.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                            int(self.node.robot.scale*neighbour_filter_distance), (255, 255, 255), 1)


            print(len(self.node.robot.camera_obstacle_points_adjusted)+len(to_remove), "-", len(to_remove), "=", len(self.node.robot.camera_obstacle_points_adjusted))


            """
            to_remove = []
            for p in self.node.robot.camera_obstacle_points_adjusted:
                p_ctr = 0
                for i in self.node.robot.camera_obstacle_points_adjusted:
                    # dist = math.dist((p.x, p.y, p.z),(i.x, i.y, i.z))  
                    dist = math.dist((p.x, p.y),(i.x, i.y))  
                    if dist < 0.3:
                        p_ctr +=1
                # print(p_ctr, end='\t')
                if p_ctr < 5:
                    to_remove.append(p)
            
            for p in to_remove:
                self.node.robot.camera_obstacle_points_adjusted.remove(p)
            # print("\n")
            """

            self.node.robot.update_debug_drawings()
            self.node.robot.update_debug_drawings2()

            # for points in to_remove:
            #     cv2.circle(self.node.robot.test_image, (int(self.node.robot.xc_adj + self.node.robot.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
            #                             int(self.node.robot.yc_adj - self.node.robot.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
            #                             4, (255, 255, 255), -1)



































"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
# from sensor_msgs.msg import Image
from charmie_interfaces.msg import ObstacleInfo, Obstacles, BoundingBox, BoundingBoxAndPoints, PointCloudCoordinates, ListOfPoints
from charmie_interfaces.srv import GetPointCloud
from example_interfaces.msg import Bool

import cv2
import numpy as np
import math
import time
import threading

class ObstaclesLIDAR:

    def __init__(self):
        print("Executing Main Code")
        
        self.START_DEG = -119.885
        self.STEP_DEG = 0.35208516886930985
        self.scale = 0.12*1000
        self.NumberOfLasers = 681
        
        self.valores_dict = {}
        self.valores_id = {}

        self.obstacles_pub = Obstacles()
        # self.obstacles_pub_ant = Obstacles()

        self.xc = 400
        self.yc = 400

        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.test_image2 = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.test_image3 = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)

        self.DEBUG_DRAW_IMAGE = False
        self.DEBUG_PRINT = True
        self.is_TRFilter = True         
        self.is_dummy_points = True
        self.is_obs = True
        self.error_lidar_reading = False
        
        #dists
        self.OBS_THRESH = 1.0
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0
        self.tr_aux = 0.01
        self.DIST_MIN_THRESH = 0.1
        self.lidar_to_robot_center = 0.255
        self.robot_radius = 0.560/2
        self.lidar_radius = 0.050/2


        self.stop_image_for_debug = False # variable so that we do a 1 second delay in a frame when we are debuging

    def lidar_readings_to_obstacles(self, ls: LaserScan):
        # readings = scan.ranges
        self.lidar_dicts(ls.ranges)

        if self.valores_dict is not None:
            self.error_lidar_reading = False

            if self.DEBUG_DRAW_IMAGE:
                self.test_image[:, :] = 0  # limpa a imagem
                self.test_image2[:, :] = 0  # limpa a imagem
                # self.test_image3[:, :] = 0  # limpa a imagem

            if self.is_TRFilter:
                self.TRFilter()

            obst_list = self.draw_points_and_obstacle_detection() 
            obs_dict_v = self.obstacle_detection(obst_list) 


            # print(self.valores_id)
            # print(self.valores_dict)
            if self.DEBUG_DRAW_IMAGE:
                cv2.circle(self.test_image, (self.xc, self.yc), (int)(self.OBS_THRESH*self.scale), (0, 255, 255), 1)
                cv2.line(self.test_image2, (60, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (800-60+1, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (0, 255, 255), 1)
                
                # corpo lidar
                cv2.circle(self.test_image, (self.xc, self.yc), (int)(self.lidar_radius*self.scale), (255, 255, 255), 1)
                # cv2.circle(self.test_image3, (self.xc, self.yc), (int)(self.lidar_radius*self.scale), (255, 255, 255), 1)
                # centro robo
                cv2.circle(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale/10), (255, 255, 255), 1)
                # cv2.circle(self.test_image3, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale/10), (255, 255, 255), 1)
                # corpo robo
                cv2.circle(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale), (255, 255, 255), 1)
                # cv2.circle(self.test_image3, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale), (255, 255, 255), 1)
                
                # cv2.imshow("LIDAR Linear", self.test_image2)
                cv2.imshow("LIDAR Circular", self.test_image)
                # cv2.imshow("Obstacles Fusion LIDAR", self.test_image3)

                # if self.stop_image_for_debug:
                #     self.stop_image_for_debug = False
                #     time.sleep(5)
                
                # k = cv2.waitKey(1)
                # if k == ord('+'):
                #     self.scale /= 0.8
                # if k == ord('-'):
                #     self.scale *= 0.8

            # return self.data_ready_to_publish(obs_dict_v)
            # self.obstacles_pub_ant = self.obstacles_pub
            self.obstacles_pub = self.data_ready_to_publish(obs_dict_v)
            # return self.obstacles_pub
        else:
            self.error_lidar_reading = True
            print("ERRO DO LIDAR AO ENVIAR TRAMA!!!!!!!!!! Segui com a minha vida...")
            
            # if it does not have new obstacle values to be updated than it does not update  

    def lidar_dicts(self, readings):
        # print(readings)
        # print(len(readings))
        # print("GO")
        # print(self.START_DEG)

        # do 0 and valor que esta no range -1 
        # self.valores_dict_ant = self.valores_dict
        for i in range(len(readings)):
            # print(x)
            # i = i + 1
            self.valores_id[self.START_DEG+i*self.STEP_DEG] = i
            self.valores_dict[self.START_DEG+i*self.STEP_DEG] = readings[i]

        # print(self.valores_id)
        # print(self.valores_dict)

        self.centre_data = (int)((self.test_image.shape[1] - 682) / 2)

    def TRFilter(self):


        # aux = self.OBS_THRESH + self.tr_aux 
        # for key, value in self.valores_id.items():
        #     if self.valores_dict[key] < self.min_dist_error or self.valores_dict[key] > self.max_dist_error:
        #         self.valores_dict[key] = aux
        #     else:
        #         aux = self.valores_dict[key]

        # aux = self.OBS_THRESH + self.tr_aux 
        for key, value in self.valores_id.items():
            if self.valores_dict[key] < self.min_dist_error or self.valores_dict[key] > self.max_dist_error:
                self.valores_dict[key] = 8
            # else:
            #     aux = self.valores_dict[key]

    def draw_points_and_obstacle_detection(self):  

        obst_dict = {}
        obst_list = []
        dist_ant = 0
        value_ant = 0
        key_ant = 0
        is_obstacle = False
        dist_counter = 0
        dist_min = self.OBS_THRESH
        dist_sum = 0

        # line used to know the zero value of the derivatives
        if self.DEBUG_DRAW_IMAGE:
            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - 0, self.yc + 250), (self.test_image.shape[1] - self.centre_data - self.NumberOfLasers, int(self.yc + 250 + self.scale * 0.5 * 0)), (255, 0, 0))


        for key, value in self.valores_id.items():
            # print(value, ":", key, ":", self.valores_dict[key])

            # ### key is the reading ID
            # ### value is the reading angle
            # ### valores_dict[value] is the reading distance
    
            # draws two LIDAR graphs, the circular one and the linear one
            # from right to left
            if self.DEBUG_DRAW_IMAGE:
                if self.valores_dict[key] > self.OBS_THRESH:
                    cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                                                    int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                             (255, 0, 0))
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 100),
                             (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 100 - self.scale * self.valores_dict[key])),
                             (255, 0, 0))
                else:
                    cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                                                    int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                             (0, 0, 255))
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 100),
                             (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 100 - self.scale * self.valores_dict[key])),
                             (0, 0, 255))
                # cv2.line(self.test_image3, (self.xc, self.yc), (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                #                                 int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                #             (255, 0, 0))
                # cv2.circle(self.test_image3, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                #                                 int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                #             2, (255, 0, 0), -1)
                
            
        
            # #### V2.0 #### #
            if value == 0:  # first angle case, since there is no previous value
                if self.valores_dict[key] < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * + (500))), (255, 0, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (0, 100, 255), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (0, 100, 255), 1)
                        # print("Start Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_i'] = key
                    obst_dict['d_i'] = self.valores_dict[key]
                    dist_min = self.valores_dict[key]
                    # key
                    is_obstacle = True
    
            if value > 0:
                if self.valores_dict[key] < self.OBS_THRESH and dist_ant < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
    
                    if self.valores_dict[key] - dist_ant > self.DIST_MIN_THRESH:  # the new obstacle is further
                        if self.DEBUG_DRAW_IMAGE:
                            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(700))), (0, 255, 255))
                            if self.is_dummy_points:
                                cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (255, 255, 0), 1)
                                cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (255, 255, 0), 1)
                            # print("Middle Obstacle: (", key, "-", valores_dict[key], ")")
                        obst_dict['alfa_f'] = key_ant
                        obst_dict['d_f'] = self.valores_dict[key_ant] 
                        if dist_counter == 0:
                            obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                        else:
                            obst_dict['d_avg'] = dist_sum/dist_counter
                        obst_dict['d_min'] = dist_min
                        obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                        obst_dict.clear()
                        dist_sum = 0
                        dist_counter = 0
                        dist_min = self.OBS_THRESH
                        obst_dict['alfa_i'] = key
                        obst_dict['d_i'] = self.valores_dict[key]
    
                    if self.valores_dict[key] - dist_ant < -self.DIST_MIN_THRESH:  # the new obstacle is closer
                        if self.DEBUG_DRAW_IMAGE:
                            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * +(300))), (0, 255, 255))
                            if self.is_dummy_points:
                                cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (255, 255, 0), 1)
                                cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (255, 255, 0), 1)
                            # print("Middle Obstacle: (", key, "-", valores_dict[key], ")")
                        obst_dict['alfa_f'] = key_ant
                        obst_dict['d_f'] = self.valores_dict[key_ant]
                        if dist_counter == 0:
                            obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                        else:
                            obst_dict['d_avg'] = dist_sum/dist_counter
                        obst_dict['d_min'] = dist_min
                        obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                        obst_dict.clear()
                        dist_sum = 0
                        dist_counter = 0
                        dist_min = self.OBS_THRESH
                        obst_dict['alfa_i'] = key
                        obst_dict['d_i'] = self.valores_dict[key]

                if self.valores_dict[key] > self.OBS_THRESH >= dist_ant and is_obstacle:  # from right to left, end of obstacle
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value_ant, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value_ant, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(500))), (255, 255, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key_ant] * math.cos(math.radians(-key_ant + 90))), int(self.yc - self.scale * self.valores_dict[key_ant] * math.sin(math.radians(-key_ant + 90)))), (int)(3), (255, 180, 180), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key_ant], int(self.yc + 100 - self.scale * self.valores_dict[key_ant])), (int)(3), (255, 180, 180), 1)
                        # print("End Obstacle: (", key_ant, "-", valores_dict[key_ant], ")")
                    obst_dict['alfa_f'] = key_ant
                    obst_dict['d_f'] = self.valores_dict[key_ant]
                    if dist_counter == 0:
                        obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                    else:
                        obst_dict['d_avg'] = dist_sum/dist_counter
                    obst_dict['d_min'] = dist_min
                    obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                    obst_dict.clear()
                    is_obstacle = False
    
                if self.valores_dict[key] <= self.OBS_THRESH < dist_ant:  # from right to left, start of obstacle
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * +(500))), (255, 0, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (0, 100, 255), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (0, 100, 255), 1)
                        # print("Start Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_i'] = key
                    obst_dict['d_i'] = self.valores_dict[key]
                    dist_min = self.valores_dict[key]
                    is_obstacle = True
                    # key
    
            if value == self.NumberOfLasers:  # last angle case, since there is no next value
                if self.valores_dict[key] < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(500))), (255, 255, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(3), (255, 180, 180), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(3), (255, 180, 180), 1)
                        # print("End Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_f'] = key
                    obst_dict['d_f'] = self.valores_dict[key]
                    dist_sum += self.valores_dict[key]
                    dist_counter += 1
                    obst_dict['d_avg'] = dist_sum / dist_counter
                    obst_dict['d_min'] = dist_min
                    obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                    obst_dict.clear()
                    is_obstacle = False
                    # key_ant

            # defines when a new obstacle starts and helps computing the distance average and minimum value
            if is_obstacle:
                dist_sum += self.valores_dict[key]
                dist_counter += 1
                if self.valores_dict[key] < dist_min:
                    dist_min = self.valores_dict[key]
            else:
                dist_sum = 0
                dist_counter = 0
                dist_min = self.OBS_THRESH
    
            value_ant = value
            dist_ant = self.valores_dict[key]
            key_ant = key
    
        # for obs in obst_list:
        #     print(obs)
    
        # print(obst_list)
        return obst_list
        
    def obstacle_detection(self, obst_list):

        # # # obst_dict is a list of dicts, each dict is an obstacle with the following parameters:
        # alfa_i: starting angle of the obstacle
        # d_i:    distance of the starting angle of the obstacle
        # alfa_i: final angle of the obstacle
        # d_i:    distance of the final angle of the obstacle
        # d_avg:  average of the obstacle distances
        # d_min:  minimum of the obstacle distances

        obs_list_v = []
        obstacle = {}
        obstacle_ = {}
        # obstacle2 = {}

        print("START OF CODE BEING TESTED:")
        print(obst_list)

        ########## ORA BEM, ESTÁ NA HORA DE FAZER MILAGRES ANTES DO ROBOCUP 23 ;) DG STYLE
        # função para dividir obstaculos grande em obstaculos pequenos. A maneira como o SDNL precisa dos dados para reconhecer obstaculos,
        # faz com que paredes laterais ao robô sejam vistas como 45º para a frente do robô, o que estraga tudo...
        # ou seja, está na hora de numa direta resolver isto, convém depois do RoboCup ver se realmente isto ficou mesmo direitinho...
         
        cut_obst_list = []

        obst_list_aux = obst_list

        cutting_obs_dist_cm = 0.1
        cutting_obs_dist_deg = 8 # was 5
        # idealmente isto teria que ser feito com o len_cm mas é muito mais rapido de implementar com diferencças de graus, para já vou tentar assim...

        for obs in obst_list_aux:
            x1 = obs['d_i'] * math.cos(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            y1 = obs['d_i'] * math.sin(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            x2 = obs['d_f'] * math.cos(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            y2 = obs['d_f'] * math.sin(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            l = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            aux_length_deg = - (obs['alfa_i'] - obs['alfa_f'])
            print("len_deg:", aux_length_deg)
            

            a_i = obs['alfa_i']
            d_i = obs['d_i']
            a_f = obs['alfa_f']
            d_f = obs['d_f']

            closer_obs = {}

            if aux_length_deg > cutting_obs_dist_deg:
                
                while a_i !=  obs['alfa_f']:
                    
                    temp_cut_obs_list = []
                    print("INSIDE")

                    # teta_entre_extremos1 = math.atan2(y2-y1, x2-x1)
                    # teta_entre_extremos2 = math.atan2(y1-y2, x1-x2)

                    teta_novo = a_i + cutting_obs_dist_deg

                    print(teta_novo)

                    # procura o valor no dicionario mais perto e devolve esse valor, esse valor é o novo a_f desta iteração e o a_i da proxima


                    aux_erro = 1000
                    next_key = 0.0
                    next_val = 0.0
                    for key, value in self.valores_dict.items():
                        err = abs(teta_novo - key)
                        if err < aux_erro:
                            aux_erro = err
                            next_key = key
                            next_val = value

                    if next_key > obs['alfa_f']:
                        a_f = obs['alfa_f']
                        d_f = obs['d_f']
                    else:
                        a_f = next_key
                        d_f = next_val

                    # print(next_key, next_val)
                    # print("OUT")

                    new_obs = {}
                    new_obs['alfa_i'] = a_i
                    new_obs['d_i'] = d_i
                    new_obs['alfa_f'] = a_f
                    new_obs['d_f'] = d_f
                    # fiz isto para ser rapido, é possivel que seja fazer melhor!!!
                    new_obs['d_avg'] = (d_i+d_f)/2
                    new_obs['d_min'] = d_i

                    
                    # cut_obst_list.append(new_obs)
                    if not closer_obs:
                        closer_obs = new_obs
                    else:
                        if new_obs['d_avg'] < closer_obs['d_avg']:
                            closer_obs = new_obs
                    

                    a_i = a_f
                    d_i = d_f
                    # a_f = obs['alfa_f']
                    # d_f = obs['d_f']
                    aux_length_deg = - (a_i - a_f)

                

                cut_obst_list.append(closer_obs)

            else:
                cut_obst_list.append(obs)
                

        print("CUT_VERSION:", cut_obst_list)


        for obs in cut_obst_list:
        # for obs in obst_list:
            # obstacle2['alfa'] = obs['alfa_i'] + (obs['alfa_f'] - obs['alfa_i']) / 2

            # 2*STEP_DEG to guarantee that the blind stop between rays is included in the object width
            # this method uses the distance values of the first and last angle to calculate the length of the obstacle
            # however some materials (i.e: metal) when detected have high variance on the first and last distances
            # thus this method has high variance and intrinsic error
            # _x1 = valores_dict[obs['alfa_i']] * math.cos(math.radians(-obs['alfa_i'] + 90 - STEP_DEG))
            # _y1 = valores_dict[obs['alfa_i']] * math.sin(math.radians(-obs['alfa_i'] + 90 - STEP_DEG))
            # _x2 = valores_dict[obs['alfa_f']] * math.cos(math.radians(-obs['alfa_f'] + 90 + STEP_DEG))
            # _y2 = valores_dict[obs['alfa_f']] * math.sin(math.radians(-obs['alfa_f'] + 90 + STEP_DEG))
            # _l = math.sqrt((_x2 - _x1) ** 2 + (_y2 - _y1) ** 2)

            # 2*STEP_DEG to guarantee that the blind stop between rays is included in the object width
            # this method uses the average distance of the obstacle to calculate its length,
            # thus it has less variance in error
            # however the length value always tends to be a bit shy of the true value
            x1 = obs['d_avg'] * math.cos(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            y1 = obs['d_avg'] * math.sin(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            x2 = obs['d_avg'] * math.cos(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            y2 = obs['d_avg'] * math.sin(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            l = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            obstacle['alfa'] = (obs['alfa_i'] + obs['alfa_f']) / 2  # center angle, just the average of the obstacle start and end angle
            obstacle['dist'] = obs['d_avg']  # it can also be used the minimum distance
            obstacle['length_cm'] = l
            obstacle['length_deg'] = - (obs['alfa_i'] - obs['alfa_f'])
            
            
            
            
            
            # print(obs, obstacle)

            #                  .(OBS)
            #                //
            #              /A/ 
            #          b /  /
            #          /   /
            #  (LID)./    /  
            #       |C   / c
            #       |   /
            #     a |  /
            #       |B/
            #       |/
            #  (ROB)'
            # 
            # Use the Law of Cosines to find one of the remaining angles. 
            # The Law of Cosines states that for any triangle with sides a, b, and c, and the opposite angle C, 
            # the following equation holds:
            # c^2 = a^2 + b^2 - 2ab * cos(C)          


            #                  .(OBS)
            #                //
            #              /A/ 
            #       dl_o /  /
            #          /   /
            #  (LID)./    /  
            #       |a   / dr_o
            #       |   /
            #  dl_r |  /
            #       |b/
            #       |/
            #  (ROB)'
            # 
            # c = sqrt(a^2 + b^2 - 2abcosC)   
            # dr_o = sqrt(dl_r^2 + dl_o^2 - 2*dl_r*dl_o*cos(alfa))  
            # 
            # C = cos-1((a^2+b^2-c^2)/2ab)   
            # beta = acos((dl_r^2+dr_o^2-dl_o^2)/2*dl_r*dr_o)   


            # obstacle['dist'] = 0.884190471399398
            # obstacle['alfa'] = 1.26280683878509

            obstacle_['dist'] = math.sqrt(self.lidar_to_robot_center*self.lidar_to_robot_center + obstacle['dist']*obstacle['dist'] -
                                          2*self.lidar_to_robot_center*obstacle['dist']*math.cos(math.pi-math.radians(obstacle['alfa'])))
            
            # prevents crashes that happened previously due to roundings fo 1.0 to 1.0000000000000004 (pasted from terminal) which are
            # outside the scope of acos which is between -1 and 1, inclusive
            aux = (self.lidar_to_robot_center*self.lidar_to_robot_center + obstacle_['dist']*obstacle_['dist'] -
                                                obstacle['dist']*obstacle['dist'])/(2*self.lidar_to_robot_center*obstacle_['dist'])
            
            
            # print("old =", obstacle['alfa'])
            # with this function I force the value to 1.0 if it rounds up to bigger than 1.0
            # it is not necessary to do this for -1, since i cannot detect obstacles in the back of the robot
            # The 1.0 input to the arcos is for angle 0 (front of the robot), the -1 is for angle 180 (deg) so the back of the robot
            if aux > 1.0:
                if self.DEBUG_PRINT:
                    print("Input of acos > 1.0 prevented!")
                aux = 1.0
                # self.stop_image_for_debug = True

            # however there is still a visual bug, due to roundings, check rosbag. (/charmie_ws/rosbags/erro_acos)
            # what can be done is just check the distance of both points in the circle and just draw in the image the one that is nearer.
            # OU DESENHAR DE MANEIRA DIFERENTE porque se o angulo e a distancia estão certos porque é que preciso de calcular os pontos, 
            # nao posso so desenhar a partir do centro e "apagar o resto" devo estar a fazer algo que não é a maneira mais eficiente

            if obstacle['alfa'] > 0:
                obstacle_['alfa'] = +math.acos(aux)
            else:
                obstacle_['alfa'] = -math.acos(aux)
            

            obstacle_['length_cm'] = obstacle['length_cm']
            obstacle_['length_deg'] = obstacle['length_deg']
            

            obstacle_['alfa'] = math.degrees(obstacle_['alfa'])
            # print("new =", obstacle_['alfa'])
            # print("dist =", obstacle_['dist'])
            

            if self.DEBUG_DRAW_IMAGE:
                if self.is_obs:
                    
                    # OLD CALCULATION GREEN LINES
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #            (int)(2), (0, 255, 0), 1)
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90)) + self.scale * (obstacle['length_cm']/2) * math.cos(math.radians(-(90 - obstacle['alfa']) + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)) - self.scale * (obstacle['length_cm']/2) * math.sin(math.radians(-(90 - obstacle['alfa']) + 90)))),
                    #            (int)(2), (0, 255, 0), 1)
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90)) - self.scale * (obstacle['length_cm']/2) * math.cos(math.radians(-(90 - obstacle['alfa']) + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)) + self.scale * (obstacle['length_cm']/2) * math.sin(math.radians(-(90 - obstacle['alfa']) + 90)))),
                    #           (int)(2), (0, 255, 0), 1)
                    # line lidar center to obstacle center
                    # cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                                                int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))
                    
                    
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.line(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                               int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                              (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                               int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                              (255, 255, 255))
                    
                                    
                    id_alfa = int((self.valores_id[obs['alfa_i']] + self.valores_id[obs['alfa_f']]) / 2)

                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - id_alfa, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_i']] + 1, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_f']] - 1, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_i']] + 1, int(self.yc + 100 - self.scale * obstacle['dist'])),
                                          (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_f']] - 1, int(self.yc + 100 - self.scale * obstacle['dist'])),
                             (0, 255, 0))
                    

                    # y = mx + b
                    (x1, y1) = (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale))
                    (x2, y2) = (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                                int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90))))

                    # to prevent divisions by zero
                    
                    if x1 == x2:
                        x2+=1
                        if self.DEBUG_PRINT:
                            print("Division by 0 prevented!")
                    
                    m = (y2-y1)/(x2-x1)
                    b = y1 - (m*x1)

                    # (x - h)^2 + (y - k)^2 = r^2
                    (h, k) = (x1, y1)
                    r = (int)(self.robot_radius*self.scale)

                    # intersection
                    # (x - h)^2 + (mx + c - k)^2 = r^2
                    # simplified
                    # (1 + m^2)x^2 + (2mc - 2hk)x + (h^2 + c^2 - 2mkx + 2kcx + k^2 - r^2) = 0

                    # (1 + m^2)x^2 + (2bm - 2h)x + (h^2 + b^2 - 2bk + k^2 - r^2) = 0
                    A = 1 + m*m
                    B = -2*h + 2*m*b - 2*m*k
                    # B = -2*h + 2*m*b - 2*m*k
                    C = h*h + b*b - 2*k*b + k*k - r*r


                    x3 = (-B + math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)
                    x4 = (-B - math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)

                    y3 = m*x3 + b
                    y4 = m*x4 + b

                    # print(x3, y3)
                    # print(x4, y4)

                    if obstacle_['alfa'] < 0:
                        pi_x = x3
                        pi_y = y3
                    else:
                        pi_x = x4
                        pi_y = y4
                
                    # line lidar center to obstacle center
                    # cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                                                int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))
                    
                    #line robot center to obstacle center
                    # cv2.line(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)),
                    #           (int(self.xc - self.scale * (obstacle_['dist']+self.robot_radius) * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                    #            int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * (obstacle_['dist']+self.robot_radius) * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                    #           (255, 255, 255))
                    
                    
                    # Visual Bug: Does not influence the working of the robot, because of angle 0 acos > 1.0
                    # explained when calculating acos

                    
                    # ponto de interseção da linha distancia do centro do robo ao obstaculo com o raio do robo
                    cv2.circle(self.test_image, (int(pi_x), int(pi_y)), (int)(4), (255, 0, 255), -1)
 
                    #line robot platform to obstacle center
                    cv2.line(self.test_image, (int(pi_x), int(pi_y)),
                              (int(pi_x - self.scale * (obstacle_['dist']-self.robot_radius) * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                               int(pi_y - self.scale * (obstacle_['dist']-self.robot_radius) * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                              (255, 255, 255))
                    



                    # erro antigo em que:
                    # 1) valores estao centradas no lidar e não no robo
                    # 2) é assumido que os valores estão todos a sair da margem do robo e nao do centro 

                    # y = mx + b
                    (x1, y1) = (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale))
                    (x2, y2) = (int(self.xc                                       - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                                int(self.yc+self.lidar_to_robot_center*self.scale - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90))))
                    
                    if x1 == x2:
                        x2+=1
                        if self.DEBUG_PRINT:
                            print("Division by 0 prevented!")
                    
                    m = (y2-y1)/(x2-x1)
                    b = y1 - (m*x1)

                    # (x - h)^2 + (y - k)^2 = r^2
                    (h, k) = (x1, y1)
                    r = (int)(self.robot_radius*self.scale)

                    # intersection
                    # (x - h)^2 + (mx + c - k)^2 = r^2
                    # simplified
                    # (1 + m^2)x^2 + (2mc - 2hk)x + (h^2 + c^2 - 2mkx + 2kcx + k^2 - r^2) = 0

                    # (1 + m^2)x^2 + (2bm - 2h)x + (h^2 + b^2 - 2bk + k^2 - r^2) = 0
                    A = 1 + m*m
                    B = -2*h + 2*m*b - 2*m*k
                    # B = -2*h + 2*m*b - 2*m*k
                    C = h*h + b*b - 2*k*b + k*k - r*r


                    x3 = (-B + math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)
                    x4 = (-B - math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)

                    y3 = m*x3 + b
                    y4 = m*x4 + b

                    # print(x3, y3)
                    # print(x4, y4)

                    if obstacle_['alfa'] < 0:
                        pi_x = x3
                        pi_y = y3
                    else:
                        pi_x = x4
                        pi_y = y4


                    # cv2.line(self.test_image, (int(pi_x), int(pi_y)), 
                    #          (int(pi_x - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #           int(pi_y - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))


            obstacle['length_deg'] = math.radians(obstacle['length_deg'])
            obstacle['alfa'] = -math.radians(obstacle['alfa'])
            

            # CORRECOES (pos impressoes no ambiente grafico)
            obstacle_['dist'] -= self.robot_radius
            obstacle_['length_deg'] = math.radians(obstacle_['length_deg'])
            obstacle_['alfa'] = -math.radians(obstacle_['alfa'])


            # print(obstacle_)

            obs_list_v.append(obstacle_.copy())

            obstacle_.clear()

        if self.DEBUG_PRINT:
            # print(obs_list_v)
            for obs in obs_list_v:
                pass
                print(obs)
            print()

        return obs_list_v
    
    def data_ready_to_publish(self, obs_dict_v):
        tot_obs = Obstacles()

        tot_obs.no_obstacles = len(obs_dict_v)
        # print(len(obs_dict_v))
        for o in range(len(obs_dict_v)):
            # print(obs_dict_v[o])
            # print(obs_dict_v[o]['alfa'])
            obs = ObstacleInfo()
            
            obs.alfa = obs_dict_v[o]['alfa']
            obs.dist = obs_dict_v[o]['dist']
            obs.length_cm = obs_dict_v[o]['length_cm']
            obs.length_degrees = obs_dict_v[o]['length_deg']

            tot_obs.obstacles.append(obs)
        
        # print(tot_obs)
        return tot_obs
    
    def draw_obstacles_fusion_lidar(self):

        for key, value in self.valores_id.items():
            cv2.circle(self.test_image3, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                                            int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                                            2, (255, 0, 0), -1)
            
            # corpo lidar
            cv2.circle(self.test_image3, (self.xc, self.yc), (int)(self.lidar_radius*self.scale), (255, 255, 255), 1)
            # centro robo
            cv2.circle(self.test_image3, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale/10), (255, 255, 255), 1)
            # corpo robo
            cv2.circle(self.test_image3, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale), (255, 255, 255), 1)
            

class ObstaclesNode(Node):

    def __init__(self):
        super().__init__("Obstacles")
        self.get_logger().info("Initialised CHARMIE Obstacles Node")

        # Create Code Class Instance
        self.obs_detect = ObstaclesLIDAR() 

        # Create PUBs/SUBs
        self.obstacles_publisher = self.create_publisher(Obstacles, "obs_lidar", 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "scan", self.lidar_callback , 10)
        self.obstacles_diagnostic_publisher = self.create_publisher(Bool, "obstacles_diagnostic", 10)
        
        # Camera Obstacles
        self.temp_camera_obstacles_publisher = self.create_publisher(ListOfPoints, "camera_obstacles", 10)
       
        flag_diagn = Bool()
        flag_diagn.data = True
        self.obstacles_diagnostic_publisher.publish(flag_diagn)

        # Create Timers
        # self.create_timer(1, self.timer_callback)

        ### Services (Clients) ###
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")

        while not self.point_cloud_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud...")

        self.waiting_for_pcloud = False
        self.point_cloud_response = GetPointCloud.Response()

    def lidar_callback(self, scan:LaserScan):
        self.obs_detect.lidar_readings_to_obstacles(scan)

        if not self.obs_detect.error_lidar_reading:
            self.obstacles_publisher.publish(self.obs_detect.obstacles_pub)

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


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    th_main = threading.Thread(target=ThreadMainObstaclesFusion, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def ThreadMainObstaclesFusion(node: ObstaclesNode):
    main = ObstaclesFusionMain(node)
    main.main()


class ObstaclesFusionMain():

    def __init__(self, node: ObstaclesNode):
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
            # time.sleep(0.1)



            self.node.obs_detect.test_image3[:, :] = 0
            
            for points in self.pc_lp.coords:

                object_rel_pos = Point()
                object_rel_pos.x =  -points.y/1000
                object_rel_pos.y =  points.x/1000
                object_rel_pos.z =  points.z/1000
                
                # calculate the absolute position according to the robot localisation
                angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                theta_aux = math.pi/2 - (angle_obj - self.robot_t)

                target_x = dist_obj * math.cos(theta_aux) + self.robot_x
                target_y = dist_obj * math.sin(theta_aux) + self.robot_y
                target_z = object_rel_pos.z

                # print(math.degrees(self.robot_t))

                # if target_z > 0.3:
                #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target_x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * target_y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             2, (255, 0, 0), -1)
                
                # else:
                #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target_x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * target_y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             2, (255, 100, 100), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
                

            # if self.node.obs_detect.DEBUG_DRAW_IMAGE:
                
            self.node.obs_detect.draw_obstacles_fusion_lidar()
            cv2.imshow("Obstacles Fusion LIDAR", self.node.obs_detect.test_image3)

            k = cv2.waitKey(1)
            if k == ord('+'):
                self.node.obs_detect.scale /= 0.8
            if k == ord('-'):
                self.node.obs_detect.scale *= 0.8
"""
