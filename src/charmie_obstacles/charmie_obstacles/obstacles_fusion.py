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
        self.lidar_to_robot_center = 0.255
        
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
        self.lidar_obstacle_points_adjusted_relative = []

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
        
        # for key, value in self.valores_dict.items(): 
        #     cv2.circle(self.test_image_, (int(self.xc - self.scale * value * math.cos(math.radians(math.degrees(-key) + 90))),
        #                                             int(self.yc - self.scale * (self.lidar_to_robot_center + value * math.sin(math.radians(math.degrees(-key) + 90))))),
        #                      5, (0, 0, 255), 1)
         
        # for key, value in self.valores_dict.items(): 
        #     cv2.circle(self.test_image_, (int(self.xc - self.scale * value * math.cos(-key + math.pi/2)),
        #                                   int(self.yc - self.scale * (self.lidar_to_robot_center + value * math.sin(-key + math.pi/2)))),
        #                      5, (0, 255, 0), 1)
        

        for points in self.lidar_obstacle_points_adjusted_relative:
            cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        int(0.05*self.scale), (0, 0, 255), 1)


        for points in self.camera_obstacle_points_adjusted_relative:
            cv2.circle(self.test_image_, (int(self.xc + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        int(0.05*self.scale), (255, 0, 0), 1)

        

        n_lines = 70
        max_lines = 2.5

        start_angle = 20

        start = math.radians(0+start_angle)
        end = math.radians(180-start_angle)
        step_size = (end - start) / (n_lines - 1)  # Calculate the step size
    

        values = [start + i * step_size for i in range(n_lines)]
        # values = [1.0471975511965976]
        # print(values)

        # print("===")
        # print(values)
        for v in values:
            cv2.line(self.test_image_, (self.xc, self.yc), 
                                       (int(self.xc + self.scale * max_lines * math.cos(v)),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc - self.scale * max_lines * math.sin(v))),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                       (255, 255, 255))
            
            m ,c = self.get_line_from_points(0.0, 0.0, max_lines * math.cos(v), max_lines * math.sin(v))
            # print("line", m, c)
                
            max_dist = 2.5
            for p in self.lidar_obstacle_points_adjusted_relative:
                # get line from two points
                # print("circle", p.x, p.y)
                inter_p = self.find_intersection_circle_line(p.x, p.y, 0.05, m, c)
                # print("inter_p", inter_p)

                t = np.sqrt(inter_p[0]**2 + inter_p[1]**2)
                if t < max_dist and t > 0 and inter_p[1] > 0:
                    max_dist = t
                    closes_inter = inter_p
            

            for p in self.camera_obstacle_points_adjusted_relative:
                # get line from two points
                # print("circle", p.x, p.y)
                inter_p = self.find_intersection_circle_line(p.x, p.y, 0.05, m, c)
                # print("inter_p", inter_p)

                t = np.sqrt(inter_p[0]**2 + inter_p[1]**2)
                if t < max_dist and t > 0 and inter_p[1] > 0:
                    max_dist = t
                    closes_inter = inter_p

            if max_dist < 2.5:
            
                cv2.circle(self.test_image_, (int(self.xc + self.scale * closes_inter[0]),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc - self.scale * closes_inter[1])),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        3, (0, 255, 0), -1)
                
        
        # all imshow cv2.circle and line in an if debug
        # all obstacle related variables as "global" and not 0.1 everywhere
        # add code to create sdnl obstacles 
        # criar um activate obstacles que permite escolher os obstaculos (lidar ou camera) em qql situação, assim só ligamos câmara quando o robô sair de casa


        cv2.imshow("Person Localization 2", self.test_image_)
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
        

    def lidar_callback(self, scan: LaserScan):
        self.robot.scan = scan
        # print(scan)

        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.robot.lidar_obstacle_points.clear()
        self.robot.lidar_obstacle_points_adjusted_relative.clear()

        for i in range(len(scan.ranges)):
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if value > self.min_dist_error and value < self.max_dist_error:

                object_rel_pos = Point()
                object_rel_pos.x =  -value * math.cos(-key + math.pi/2)
                object_rel_pos.y =  self.robot.lidar_to_robot_center + value * math.sin(-key + math.pi/2)
                object_rel_pos.z =  0.35 # lidar height on the robot
                
                self.robot.lidar_obstacle_points_adjusted_relative.append(object_rel_pos)
            
                obs_x = value * math.cos(key + self.robot.robot_t + math.pi/2)
                obs_y = value * math.sin(key + self.robot.robot_t + math.pi/2)

                adj_x = (self.robot.robot_radius - self.robot.lidar_radius)*math.cos(self.robot.robot_t + math.pi/2)
                adj_y = (self.robot.robot_radius - self.robot.lidar_radius)*math.sin(self.robot.robot_t + math.pi/2)

                target = Point()
                target.x = self.robot.robot_x + obs_x + adj_x
                target.y = self.robot.robot_y + obs_y + adj_y
                target.z = 0.35 # lidar height on the robot

                self.robot.lidar_obstacle_points.append(target)


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
            pc = self.get_point_cloud() 

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
            
            # this is a debug display of all the points without neighbours that are removed from the lisr
            """
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
            """

            print(len(self.node.robot.camera_obstacle_points_adjusted)+len(to_remove), "-", len(to_remove), "=", len(self.node.robot.camera_obstacle_points_adjusted))

            self.node.robot.update_debug_drawings()
            self.node.robot.update_debug_drawings2()