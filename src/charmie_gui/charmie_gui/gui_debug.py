#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, String, Float32
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import Image, LaserScan
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from charmie_interfaces.msg import  Yolov8Pose, Yolov8Objects, NeckPosition, ListOfPoints, TarNavSDNL, ListOfDetectedObject, ListOfDetectedPerson, PS4Controller, DetectedPerson, DetectedObject
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger, NavTrigger, SetFace, ActivateObstacles, GetPointCloud, SetAcceleration
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import math
import threading
from pathlib import Path
import json
import os
import time
from datetime import datetime

import pygame_widgets
import pygame
from pygame_widgets.toggle import Toggle
from pygame_widgets.button import Button
from pygame_widgets.textbox import TextBox

DEBUG_DRAW = False


class Robot():
    def __init__(self):
        print("New Robot Class Initialised")

        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
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
        self.search_for_person = ListOfDetectedPerson()
        self.search_for_object = ListOfDetectedObject()

        self.navigation = TarNavSDNL()
        self.is_navigating = False

        self.scan = LaserScan()
        self.valores_dict = {}

        self.camera_obstacle_points = []
        self.final_obstacle_points = []
        self.lidar_obstacle_points = []

        self.NORTE = 6.0
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
            self.all_pos_x_val.append(self.robot_x)
            self.all_pos_y_val.append(self.robot_y)
            self.all_pos_t_val.append(self.robot_t)
            for i in range(len(self.all_pos_x_val)):
                cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.all_pos_x_val[i]), int(self.yc_adj - self.scale * self.all_pos_y_val[i])), 1, (255, 255, 0), -1)

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
                       

            if self.is_navigating:
                pass
            
            if self.navigation.move_or_rotate == "move" or self.navigation.move_or_rotate == "rotate":
                cv2.circle(self.test_image, (int(self.xc_adj + self.navigation.target_coordinates.x*self.scale),
                    int(self.yc_adj - self.navigation.target_coordinates.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (0, 255, 0), -1)
                cv2.circle(self.test_image, (int(self.xc_adj + self.navigation.target_coordinates.x*self.scale),
                    int(self.yc_adj - self.navigation.target_coordinates.y*self.scale)), (int)(self.scale*self.navigation.reached_radius), (0, 255, 0), 1)
            



            for points in self.lidar_obstacle_points:

                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        3, (0, 0, 255), -1)


            for points in self.camera_obstacle_points:
            
                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * points.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                        int(self.yc_adj - self.scale * points.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                        3, (0, 165, 255), -1)
                       
            for points in self.final_obstacle_points:

                # calculate the absolute position according to the robot localisation
                dist_obj = math.sqrt(points.x**2 + points.y**2)

                # if self.robot.DEBUG_DRAW_IMAGE_OVERALL:
                angle_obj = math.atan2(points.x, points.y)
                theta_aux = math.pi/2 - (angle_obj - self.robot_t)

                target = Point()
                target.x = dist_obj * math.cos(theta_aux) + self.robot_x
                target.y = dist_obj * math.sin(theta_aux) + self.robot_y
                target.z = points.z

                cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target.x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                    int(self.yc_adj - self.scale * target.y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                    3, (255, 255, 255), -1)
            
            # print(self.robot_t, self.imu_orientation_norm_rad)

            # self.robot_t = -self.imu_orientation_norm_rad
            """
            for key, value in self.valores_dict.items():
                # print(f"Ang: {key}, Dist: {value}")



                if value > 0.1: 
                    obs_x = value * math.cos(key + self.robot_t + math.pi/2)
                    obs_y = value * math.sin(key + self.robot_t + math.pi/2)

                    ### ROBOT

                    cv2.circle(self.test_image, (int(self.xc_adj + self.scale * (self.robot_x + obs_x) + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                                int(self.yc_adj - self.scale * (self.robot_y + obs_y) - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                                                2, (0, 0, 255), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
                


            for points in self.camera_obstacle_points.coords:
                # print(f"Ang: {key}, Dist: {value}")



                # if value > 0.1: 
                #     obs_x = value * math.cos(key + self.robot_t + math.pi/2)
                #     obs_y = value * math.sin(key + self.robot_t + math.pi/2)

                ### ROBOT

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale * (self.robot_x - (points.y/1000))),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * (self.robot_y + (points.x/1000)))),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             2, (0, 255, 255), -1)

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
                                            2, (255, 0, 0), -1)
                # else:
                #     cv2.circle(self.test_image, (int(self.xc_adj + self.scale * target_x),# + (self.robot_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale * target_y)),# - (self.robot_radius)*self.scale*math.sin(self.robot_t + math.pi/2))),
                #                             2, (255, 100, 100), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #                             int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
                



            """
            for person in self.person_pose.persons:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           
                cv2.circle(self.test_image, (int(self.xc_adj + person.position_absolute.x*self.scale),
                    int(self.yc_adj - person.position_absolute.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 255, 255), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)


            for person in self.search_for_person.persons:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.circle(self.test_image, (int(self.xc_adj + person.position_absolute.x*self.scale),
                    int(self.yc_adj - person.position_absolute.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
                
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

            
            for object in self.search_for_object.objects:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.rectangle(self.test_image, 
                              (int(self.xc_adj + object.position_absolute.x*self.scale + self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_absolute.y*self.scale + self.lidar_radius*2*self.scale)),
                              (int(self.xc_adj + object.position_absolute.x*self.scale - self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_absolute.y*self.scale - self.lidar_radius*2*self.scale)),
                              (255, 0, 0), -1)
                               
                # cv2.circle(self.test_image, (int(self.xc_adj + object.position_relative.x*self.scale),
                #     int(self.yc_adj - object.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)


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

class DebugVisualNode(Node):

    def __init__(self):
        super().__init__("Robot")
        self.get_logger().info("Initialised CHARMIE Debug Visual Node")

        ### Topics ###
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)
        
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

        # search for person and object 
        self.search_for_person_subscriber = self.create_subscription(ListOfDetectedPerson, "search_for_person_detections", self.search_for_person_detections_callback, 10)
        self.search_for_object_subscriber = self.create_subscription(ListOfDetectedObject, "search_for_object_detections", self.search_for_object_detections_callback, 10)
        
        # IMU
        self.get_orientation_subscriber = self.create_subscription(Float32, "get_orientation", self.get_orientation_callback, 10)
       
        # Camera Obstacles
        self.temp_camera_obstacles_subscriber = self.create_subscription(ListOfPoints, "camera_head_obstacles", self.get_camera_obstacles_callback, 10)

        # Obstacles
        self.final_obstacles_subscriber = self.create_subscription(ListOfPoints, "final_obstacles", self.get_final_obstacles_callback, 10)

        # PS4 Controller
        self.controller_subscriber = self.create_subscription(PS4Controller, "controller_state", self.ps4_controller_callback, 10)

        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        self.object_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'objects_detected_filtered_hand', self.object_detected_filtered_hand_callback, 10)
        self.doors_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "doors_detected_filtered", self.doors_detected_filtered_callback, 10)
        self.doors_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'doors_detected_filtered_hand', self.doors_detected_filtered_hand_callback, 10)
        self.shoes_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "shoes_detected_filtered", self.shoes_detected_filtered_callback, 10)
        self.shoes_detected_filtered_hand_subscriber = self.create_subscription(Yolov8Objects, 'shoes_detected_filtered_hand', self.shoes_detected_filtered_hand_callback, 10)

        ### Services (Clients) ###
		# Arm (Ufactory)
        self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        self.save_speech_command_client = self.create_client(SaveSpeechCommand, "save_speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Face
        self.face_command_client = self.create_client(SetFace, "face_command")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolo Pose
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # Yolo Objects
        self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")
        # Navigation
        self.nav_trigger_client = self.create_client(NavTrigger, "nav_trigger")
        # Obstacles
        self.activate_obstacles_client = self.create_client(ActivateObstacles, "activate_obstacles")
        # Point Cloud
        self.point_cloud_client = self.create_client(GetPointCloud, "get_point_cloud")
        # Low level
        self.set_acceleration_ramp_client = self.create_client(SetAcceleration, "set_acceleration_ramp")

        self.create_timer(1.0, self.check_yolos_timer)
        self.is_yolo_pose_comm = False

        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        self.activate_yolo_objects_success = True
        self.activate_yolo_objects_message = ""
        self.activate_obstacles_success = True
        self.activate_obstacles_message = ""

        self.head_rgb = Image()
        self.hand_rgb = Image()
        self.head_depth = Image()
        self.hand_depth = Image()
        self.new_head_rgb = False
        self.new_hand_rgb = False
        self.new_head_depth = False
        self.new_hand_depth = False
        self.robot = Robot()

        self.detected_people = Yolov8Pose()
        self.detected_objects = Yolov8Objects()
        self.detected_objects_hand = Yolov8Objects()
        self.detected_doors = Yolov8Objects()
        self.detected_doors_hand = Yolov8Objects()
        self.detected_shoes = Yolov8Objects()
        self.detected_shoes_hand = Yolov8Objects()
        self.new_detected_people = False
        self.new_detected_objects = False
        self.new_detected_objects_hand = False
        self.new_detected_doors = False
        self.new_detected_doors_hand = False
        self.new_detected_shoes = False
        self.new_detected_shoes_hand = False

        self.lidar_time = 0.0
        self.odometry_time = 0.0
        self.ps4_controller_time = 0.0

        self.head_camera_time = 0.0
        self.head_depth_camera_time = 0.0
        self.hand_camera_time = 0.0
        self.hand_depth_camera_time = 0.0

        self.last_head_camera_time = 0.0
        self.last_head_depth_camera_time = 0.0
        self.last_hand_camera_time = 0.0
        self.last_hand_depth_camera_time = 0.0

        self.head_rgb_fps = 0.0
        self.head_depth_fps = 0.0
        self.hand_rgb_fps = 0.0
        self.hand_depth_fps = 0.0

    def check_yolos_timer(self):
        
        if self.new_detected_people:
            self.new_detected_people = False
            self.is_yolo_pose_comm = True
        else:
            self.is_yolo_pose_comm = False

        

    def get_color_image_hand_callback(self, img: Image):
        self.hand_rgb = img
        self.new_hand_rgb = True

        self.last_hand_camera_time = self.hand_camera_time
        self.hand_camera_time = time.time()

        self.hand_rgb_fps = str(round(1/(self.hand_camera_time-self.last_hand_camera_time), 2))

    def get_color_image_head_callback(self, img: Image):
        self.head_rgb = img
        self.new_head_rgb = True

        self.last_head_camera_time = self.head_camera_time
        self.head_camera_time = time.time()
        
        self.head_rgb_fps = str(round(1/(self.head_camera_time-self.last_head_camera_time), 2))

    def get_aligned_depth_image_hand_callback(self, img: Image):
        self.hand_depth = img
        self.new_hand_depth = True

        self.last_hand_depth_camera_time = self.hand_depth_camera_time
        self.hand_depth_camera_time = time.time()
        
        self.head_depth_fps = str(round(1/(self.hand_depth_camera_time-self.last_hand_depth_camera_time), 2))


    def get_aligned_depth_image_head_callback(self, img: Image):
        self.head_depth = img
        self.new_head_depth = True
        
        self.last_head_depth_camera_time = self.head_depth_camera_time
        self.head_depth_camera_time = time.time()
        
        self.hand_depth_fps = str(round(1/(self.head_depth_camera_time-self.last_head_depth_camera_time), 2))


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

    ### ACTIVATE OBSTACLES SERVER FUNCTIONS ###
    def call_activate_obstacles_server(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False):
        request = ActivateObstacles.Request()
        request.activate_lidar_up = obstacles_lidar_up
        request.activate_lidar_bottom = obstacles_lidar_bottom
        request.activate_camera_head = obstacles_camera_head

        self.activate_obstacles_client.call_async(request)

    
    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people
        self.new_detected_people = True
    
    def object_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_objects = det_object
        self.new_detected_objects = True

    def object_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_objects_hand = det_object
        self.new_detected_objects_hand = True

    def doors_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_doors = det_object
        self.new_detected_doors = True

    def doors_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_doors_hand = det_object
        self.new_detected_doors_hand = True

    def shoes_detected_filtered_callback(self, det_object: Yolov8Objects):
        self.detected_shoes = det_object
        self.new_detected_shoes = True

    def shoes_detected_filtered_hand_callback(self, det_object: Yolov8Objects):
        self.detected_shoes_hand = det_object
        self.new_detected_shoes_hand = True

    def ps4_controller_callback(self, controller: PS4Controller):
        self.ps4_controller_time = time.time()

    def get_orientation_callback(self, orientation: Float32):
        # self.robot.imu_orientation = orientation.data
        imu_orientation_norm = orientation.data - self.robot.NORTE
        if imu_orientation_norm > 180.0:
            imu_orientation_norm -= 360.0
        if imu_orientation_norm < -180.0:
            imu_orientation_norm += 360.0

        self.robot.imu_orientation_norm_rad = math.radians(imu_orientation_norm)
        self.robot.robot_t = -self.robot.imu_orientation_norm_rad

        
    def get_camera_obstacles_callback(self, points: ListOfPoints):
        self.robot.camera_obstacle_points = points.coords
        # print("Received Points")
        # print
        
    def get_final_obstacles_callback(self, points: ListOfPoints):
        self.robot.final_obstacle_points = points.coords
        # print("Received Points")
        # print(self.robot.final_obstacle_points)

    def lidar_callback(self, scan: LaserScan):
        self.robot.scan = scan

        self.lidar_time = time.time()

        # print(scan)
        """
        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment


        for i in range(len(scan.ranges)):
            # print(x)
            # i = i + 1
            # self.valores_id[START_RAD+i*STEP_RAD] = i
            self.robot.valores_dict[START_RAD+i*STEP_RAD] = scan.ranges[i]

        # print(self.robot.valores_dict, "\n")
        """
        START_RAD = scan.angle_min
        STEP_RAD = scan.angle_increment
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0

        self.robot.lidar_obstacle_points.clear()

        # calculates list of lidar obstacle points
        for i in range(len(scan.ranges)):
            
            value = scan.ranges[i]
            key = START_RAD+i*STEP_RAD
            
            if value > self.min_dist_error: # and value < self.max_dist_error:

                # object_rel_pos = Point()
                # object_rel_pos.x =  -value * math.cos(-key + math.pi/2)
                # object_rel_pos.y =  self.robot.lidar_to_robot_center + value * math.sin(-key + math.pi/2)
                # object_rel_pos.z =  0.35 # lidar height on the robot
                
                # calculate the absolute position according to the robot localisation
                # dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

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
        self.odometry_time = time.time()
        # self.robot.robot_t = pose.theta
        
    def search_for_person_detections_callback(self, points: ListOfDetectedPerson):
        self.robot.search_for_person = points
        
    def search_for_object_detections_callback(self, points: ListOfDetectedObject):
        self.robot.search_for_object = points

    def get_color_image_callback(self, img: Image):
        # self.get_logger().info('Receiving color video frame')
        # ROS2 Image Bridge for OpenCV
        br = CvBridge()
        self.robot.current_frame = br.imgmsg_to_cv2(img, "bgr8")

class CheckNodesMain():

    def __init__(self, node: DebugVisualNode):
        self.node = node

        self.CHECK_ARM_UFACTORY_NODE = False
        self.CHECK_ARM_NODE = False
        self.CHECK_AUDIO_NODE = False
        self.CHECK_FACE_NODE = False
        self.CHECK_HEAD_CAMERA_NODE = False
        self.CHECK_HAND_CAMERA_NODE = False
        self.CHECK_LIDAR_NODE = False
        self.CHECK_LOW_LEVEL_NODE = False
        self.CHECK_NAVIGATION_NODE = False
        self.CHECK_NECK_NODE = False
        self.CHECK_OBSTACLES_NODE = False
        self.CHECK_ODOMETRY_NODE = False
        self.CHECK_POINT_CLOUD_NODE = False
        self.CHECK_PS4_CONTROLLER_NODE = False
        self.CHECK_SPEAKERS_NODE = False
        self.CHECK_YOLO_OBJECTS_NODE = False
        self.CHECK_YOLO_POSE_NODE = False

        self.WAIT_TIME_CHECK_NODE = 0.0
        self.MIN_TIMEOUT_FOR_CHECK_NODE = 1.0

    def main(self):

        while True:

            current_time = time.time()

            # ARM_UFACTORY
            if not self.node.set_position_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.get_logger().warn("Waiting for Arm (uFactory) ...")
                self.CHECK_ARM_UFACTORY_NODE = False
            else:
                self.CHECK_ARM_UFACTORY_NODE = True

            # ARM_CHARMIE
            if not self.node.arm_trigger_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Arm (CHARMIE) ...")
                self.CHECK_ARM_NODE = False
            else:
                self.CHECK_ARM_NODE = True

            # AUDIO
            if not self.node.get_audio_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Audio ...")
                self.CHECK_AUDIO_NODE = False
            else:
                self.CHECK_AUDIO_NODE = True

            # FACE
            if not self.node.face_command_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Face ...")
                self.CHECK_FACE_NODE = False
            else:
                self.CHECK_FACE_NODE = True

            # HEAD CAMERA
            if current_time - self.node.head_camera_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_HEAD_CAMERA_NODE = False
            else:
                self.CHECK_HEAD_CAMERA_NODE = True

            # HAND CAMERA
            if current_time - self.node.hand_camera_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_HAND_CAMERA_NODE = False
            else:
                self.CHECK_HAND_CAMERA_NODE = True

            # LIDAR
            if current_time - self.node.lidar_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_LIDAR_NODE = False
            else:
                self.CHECK_LIDAR_NODE = True

            # LOW LEVEL
            if not self.node.set_acceleration_ramp_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Navigation ...")
                self.CHECK_LOW_LEVEL_NODE = False
            else:
                self.CHECK_LOW_LEVEL_NODE = True

            # NAVIGATION
            if not self.node.nav_trigger_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Navigation ...")
                self.CHECK_NAVIGATION_NODE = False
            else:
                self.CHECK_NAVIGATION_NODE = True

            # NECK
            if not self.node.set_neck_position_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Neck ...")
                self.CHECK_NECK_NODE = False
            else:
                self.CHECK_NECK_NODE = True
            
            # OBSTACLES
            if not self.node.activate_obstacles_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Obstacles ...")
                self.CHECK_OBSTACLES_NODE = False
            else:
                self.CHECK_OBSTACLES_NODE = True

            # ODOMETRY
            if current_time - self.node.odometry_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_ODOMETRY_NODE = False
            else:
                self.CHECK_ODOMETRY_NODE = True

            # POINT CLOUD
            if not self.node.point_cloud_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Speech ...")
                self.CHECK_POINT_CLOUD_NODE = False
            else:
                self.CHECK_POINT_CLOUD_NODE = True

            # PS4 CONTROLLER
            if current_time - self.node.ps4_controller_time > self.MIN_TIMEOUT_FOR_CHECK_NODE:
                # self.node.get_logger().warn("Waiting for Topic Lidar ...")
                self.CHECK_PS4_CONTROLLER_NODE = False
            else:
                self.CHECK_PS4_CONTROLLER_NODE = True

            # SPEAKERS
            if not self.node.speech_command_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Speech ...")
                self.CHECK_SPEAKERS_NODE = False
            else:
                self.CHECK_SPEAKERS_NODE = True
                
            # YOLO OBJECTS
            if not self.node.activate_yolo_objects_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Yolo Objects ...")
                self.CHECK_YOLO_OBJECTS_NODE = False
            else:
                self.CHECK_YOLO_OBJECTS_NODE = True
            
            # YOLO POSE
            if not self.node.activate_yolo_pose_client.wait_for_service(self.WAIT_TIME_CHECK_NODE):
                # self.node.get_logger().warn("Waiting for Server Yolo Pose ...")
                self.CHECK_YOLO_POSE_NODE = False
            else:
                self.CHECK_YOLO_POSE_NODE = True


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualNode()
    check_nodes = CheckNodesMain(node)
    th_nodes = threading.Thread(target=thread_check_nodes, args=(node,check_nodes,), daemon=True)
    th_main = threading.Thread(target=thread_main_debug_visual, args=(node,check_nodes,), daemon=True)
    th_main.start()
    th_nodes.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_debug_visual(node: DebugVisualNode, check_nodes: CheckNodesMain):
    main = DebugVisualMain(node, check_nodes)
    main.main()

def thread_check_nodes(node: DebugVisualNode, check_nodes: CheckNodesMain):
    check_nodes.main()

class DebugVisualMain():

    def __init__(self, node: DebugVisualNode, check_nodes: CheckNodesMain):
        
        self.node = node
        self.check_nodes = check_nodes

        self.RED = (255,0,0)
        self.GREEN = (0,255,0)
        self.BLUE = (0,0,255)
        self.BLUE_L = (0,128,255)
        self.WHITE = (255,255,255)
        self.GREY = (128,128,128)
        self.BLACK = (0,0,0)
        self.ORANGE = (255,153,51)
        self.MAGENTA = (255, 51, 255)
        self.YELLOW = (255,255,0)

        self.cam_width_ = 640
        self.cam_height_ = 360
        self.cams_initial_height = 10
        self.cams_initial_width = 165

        # self.pause_button = False
        
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        logo_midpath = "/charmie_ws/src/configuration_files/logos/"
        self.save_recordings_midpath = "/charmie_ws/src/charmie_gui/charmie_gui/saved_gui_recordings/"

        self.br = CvBridge()

        pygame.init()

        self.WIDTH, self.HEIGHT = 1387, 752
        self.FPS = 20

        os.environ['SDL_VIDEO_CENTERED'] = '1'
        info = pygame.display.Info()
        screen_width, screen_height = info.current_w, info.current_h
        print(screen_width, screen_height)
        # self.WIN = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
        self.WIN = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)

        # self.text_font = pygame.font.SysFont("Arial", 30)
        self.text_font_t = pygame.font.SysFont(None, 30)
        self.text_font = pygame.font.SysFont(None,24)

        
        # self.WIN = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
        
        # self.button = Button(self.WIN, 640+200+10, 10+360-50, 100, 100,
        # Optional Parameters
        # text='Pause',  # Text to display
        # fontSize=20,  # Size of font
        # textColour=self.WHITE,
        # margin=0,  # Minimum distance between text/image and edge of button
        # inactiveColour=(200, 50, 0),  # Colour of button when not being interacted with
        # hoverColour=(150, 0, 0),  # Colour of button when being hovered over
        # pressedColour=(255, 75, 0),  # Colour of button when being clicked
        # radius=10,  # Radius of border corners (leave empty for not curved)
        # onClick=lambda: self.test_button_function()  # Function to call when clicked on  
        # )


        # self.textbox = TextBox(self.WIN, 500, 500, 800, 80, fontSize=50,
        #           borderColour=(255, 0, 0), textColour=(0, 200, 0),
        #           onSubmit=self.output, radius=10, borderThickness=5)
        

        icon = pygame.image.load(self.home+logo_midpath+"logo_light_cropped_squared.png")
        pygame.display.set_icon(icon)
        pygame.display.set_caption("CHARMIE Debug Node")

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.current_datetime = str(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
        
        self.init_pos_w_rect_check_nodes = 15
        self.init_pos_h_rect_check_nodes = 50
        self.deviation_pos_h_rect_check_nodes = 25
        self.square_size_rect_check_nodes = 10

        self.ARM_UFACTORY_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*0, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_ARM_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*1, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_AUDIO_NODE_RECT            = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*2, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_FACE_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*3, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.HEAD_CAMERA_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*4, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.HAND_CAMERA_NODE_RECT              = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*5, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LIDAR_NODE_RECT            = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*6, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_LOW_LEVEL_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*7, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_NAVIGATION_NODE_RECT       = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*8, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_NECK_NODE_RECT             = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*9, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_OBSTACLES_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*10, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_ODOMETRY_NODE_RECT         = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*11, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_POINT_CLOUD_NODE_RECT      = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*12, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_PS4_CONTROLLER_NODE_RECT   = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*13, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_SPEAKERS_NODE_RECT         = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*14, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_YOLO_OBJECTS_NODE_RECT     = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*15, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)
        self.CHARMIE_YOLO_POSE_NODE_RECT        = pygame.Rect(self.init_pos_w_rect_check_nodes, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*16, self.square_size_rect_check_nodes, self.square_size_rect_check_nodes)

        self.toggle_record = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(18.75)), 40, 16)
        self.toggle_pause_cams = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*21.25), 40, 16)
        self.toggle_head_rgb_depth = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*23.75), 40, 16)
        self.toggle_hand_rgb_depth = Toggle(self.WIN, int(3.5*self.init_pos_w_rect_check_nodes), int(self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*26.25), 40, 16)

        self.toggle_activate_objects_head =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+50, 40, 16)
        self.toggle_activate_furniture_head = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+90,  self.cams_initial_height+50, 40, 16)
        self.toggle_activate_shoes_head =     Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+192, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_objects_hand =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+298, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_furniture_hand = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+390, self.cams_initial_height+50, 40, 16)
        self.toggle_activate_shoes_hand =     Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+492, self.cams_initial_height+50, 40, 16)

        self.toggle_pose_activate =       Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_waving =         Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+93,  self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_front_close =    Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+182, self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_legs_visible =   Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+302, self.cams_initial_height+80+50, 40, 16)
        self.toggle_pose_characteristcs = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+427, self.cams_initial_height+80+50, 40, 16)
        
        self.toggle_obstacles_lidar_top =    Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height,     self.cams_initial_height+160+50, 40, 16)
        self.toggle_obstacles_lidar_bottom = Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+100,  self.cams_initial_height+160+50, 40, 16)
        self.toggle_obstacles_head_camera =  Toggle(self.WIN, self.cams_initial_width+self.cam_width_+2*self.cams_initial_height+233, self.cams_initial_height+160+50, 40, 16)
        
        self.last_toggle_record = False

        self.last_toggle_activate_objects_head =   False
        self.last_toggle_activate_furniture_head = False
        self.last_toggle_activate_shoes_head =     False
        self.last_toggle_activate_objects_hand =   False
        self.last_toggle_activate_furniture_hand = False
        self.last_toggle_activate_shoes_hand =     False

        self.last_toggle_pose_activate =       False
        self.last_toggle_pose_waving =         False
        self.last_toggle_pose_front_close =    False
        self.last_toggle_pose_legs_visible =   False
        self.last_toggle_pose_characteristcs = False
        
        self.last_toggle_obstacles_lidar_top =    False
        self.last_toggle_obstacles_lidar_bottom = False
        self.last_toggle_obstacles_head_camera =  False

        self.curr_head_rgb = Image()
        self.last_head_rgb = Image()
        self.curr_head_depth = Image()
        self.last_head_depth = Image()
        self.curr_hand_rgb = Image()
        self.last_hand_rgb = Image()
        self.curr_hand_depth = Image()
        self.last_hand_depth = Image()

        self.curr_detected_people = Yolov8Pose()
        self.last_detected_people = Yolov8Pose()
    

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
    
    def activate_obstacles(self, obstacles_lidar_up=True, obstacles_lidar_bottom=False, obstacles_camera_head=False, wait_for_end_of=True):
        
        self.node.call_activate_obstacles_server(obstacles_lidar_up=obstacles_lidar_up, obstacles_lidar_bottom=obstacles_lidar_bottom, obstacles_camera_head=obstacles_camera_head)

        self.node.activate_obstacles_success = True
        self.node.activate_obstacles_message = "Activated with selected parameters"

        return self.node.activate_obstacles_success, self.node.activate_obstacles_message

    # def test_button_function(self):

        # self.pause_button = not self.pause_button

        # self.button.text = 'Play'

        # if self.pause_button:
        #     print("UP UP UP")
        #     self.pause_button = False
        # else:
        #     print("DOWN DOWN DOWN")
        #     self.pause_button = True

    # def output(self):
        # Get text in the textbox
        # print(self.textbox.getText())

    
    def draw_text(self, text, font, text_col, x, y):
        img = font.render(text, True, text_col)
        self.WIN.blit(img, (x, y))

    def draw_transparent_rect(self, x, y, width, height, color, alpha):
        temp_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        temp_surface.fill((*color, alpha))
        self.WIN.blit(temp_surface, (x, y))

    def draw_nodes_check(self):

        self.draw_text("Check Nodes:", self.text_font_t, self.WHITE, 10, 10)

        # ARM_UFACTORY
        self.draw_text("Arm uFactory", self.text_font, self.WHITE, self.ARM_UFACTORY_NODE_RECT.x+2*self.ARM_UFACTORY_NODE_RECT.width, self.ARM_UFACTORY_NODE_RECT.y-2)
        if self.check_nodes.CHECK_ARM_UFACTORY_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.ARM_UFACTORY_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.ARM_UFACTORY_NODE_RECT)
            
        # ARM_CHARMIE
        self.draw_text("Arm", self.text_font, self.WHITE, self.CHARMIE_ARM_NODE_RECT.x+2*self.CHARMIE_ARM_NODE_RECT.width, self.CHARMIE_ARM_NODE_RECT.y-2)
        if self.check_nodes.CHECK_ARM_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_ARM_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_ARM_NODE_RECT)

        # AUDIO
        self.draw_text("Audio", self.text_font, self.WHITE, self.CHARMIE_AUDIO_NODE_RECT.x+2*self.CHARMIE_AUDIO_NODE_RECT.width, self.CHARMIE_AUDIO_NODE_RECT.y-2)
        if self.check_nodes.CHECK_AUDIO_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_AUDIO_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_AUDIO_NODE_RECT)

        # FACE
        self.draw_text("Face", self.text_font, self.WHITE, self.CHARMIE_FACE_NODE_RECT.x+2*self.CHARMIE_FACE_NODE_RECT.width, self.CHARMIE_FACE_NODE_RECT.y-2)
        if self.check_nodes.CHECK_FACE_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_FACE_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_FACE_NODE_RECT)

        # HEAD CAMERA
        self.draw_text("Head Camera", self.text_font, self.WHITE, self.HEAD_CAMERA_NODE_RECT.x+2*self.HEAD_CAMERA_NODE_RECT.width, self.HEAD_CAMERA_NODE_RECT.y-2)
        if self.check_nodes.CHECK_HEAD_CAMERA_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.HEAD_CAMERA_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.HEAD_CAMERA_NODE_RECT)
        
        # HAND CAMERA
        self.draw_text("Hand Camera", self.text_font, self.WHITE, self.HAND_CAMERA_NODE_RECT.x+2*self.HAND_CAMERA_NODE_RECT.width, self.HAND_CAMERA_NODE_RECT.y-2)
        if self.check_nodes.CHECK_HAND_CAMERA_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.HAND_CAMERA_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.HAND_CAMERA_NODE_RECT)
        
        # LIDAR
        self.draw_text("Lidar", self.text_font, self.WHITE, self.CHARMIE_LIDAR_NODE_RECT.x+2*self.CHARMIE_LIDAR_NODE_RECT.width, self.CHARMIE_LIDAR_NODE_RECT.y-2)
        if self.check_nodes.CHECK_LIDAR_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_LIDAR_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_LIDAR_NODE_RECT)

        # LOW LEVEL
        self.draw_text("Low Level", self.text_font, self.WHITE, self.CHARMIE_LOW_LEVEL_NODE_RECT.x+2*self.CHARMIE_LOW_LEVEL_NODE_RECT.width, self.CHARMIE_LOW_LEVEL_NODE_RECT.y-2)
        if self.check_nodes.CHECK_LOW_LEVEL_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_LOW_LEVEL_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_LOW_LEVEL_NODE_RECT)
        
        # NAVIGATION
        self.draw_text("Navigation", self.text_font, self.WHITE, self.CHARMIE_NAVIGATION_NODE_RECT.x+2*self.CHARMIE_NAVIGATION_NODE_RECT.width, self.CHARMIE_NAVIGATION_NODE_RECT.y-2)
        if self.check_nodes.CHECK_NAVIGATION_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_NAVIGATION_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_NAVIGATION_NODE_RECT)

        # NECK
        self.draw_text("Neck", self.text_font, self.WHITE, self.CHARMIE_NECK_NODE_RECT.x+2*self.CHARMIE_NECK_NODE_RECT.width, self.CHARMIE_NECK_NODE_RECT.y-2)
        if self.check_nodes.CHECK_NECK_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_NECK_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_NECK_NODE_RECT)

        # OBSTACLES
        self.draw_text("Obstacles", self.text_font, self.WHITE, self.CHARMIE_OBSTACLES_NODE_RECT.x+2*self.CHARMIE_OBSTACLES_NODE_RECT.width, self.CHARMIE_OBSTACLES_NODE_RECT.y-2)
        if self.check_nodes.CHECK_OBSTACLES_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_OBSTACLES_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_OBSTACLES_NODE_RECT)

        # ODOMETRY
        self.draw_text("Odometry", self.text_font, self.WHITE, self.CHARMIE_ODOMETRY_NODE_RECT.x+2*self.CHARMIE_ODOMETRY_NODE_RECT.width, self.CHARMIE_ODOMETRY_NODE_RECT.y-2)
        if self.check_nodes.CHECK_ODOMETRY_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_ODOMETRY_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_ODOMETRY_NODE_RECT)

        # POINT CLOUD
        self.draw_text("Point Cloud", self.text_font, self.WHITE, self.CHARMIE_POINT_CLOUD_NODE_RECT.x+2*self.CHARMIE_POINT_CLOUD_NODE_RECT.width, self.CHARMIE_POINT_CLOUD_NODE_RECT.y-2)
        if self.check_nodes.CHECK_POINT_CLOUD_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_POINT_CLOUD_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_POINT_CLOUD_NODE_RECT)

        # PS4 CONTROLLER
        self.draw_text("PS4 Controller", self.text_font, self.WHITE, self.CHARMIE_PS4_CONTROLLER_NODE_RECT.x+2*self.CHARMIE_PS4_CONTROLLER_NODE_RECT.width, self.CHARMIE_PS4_CONTROLLER_NODE_RECT.y-2)
        if self.check_nodes.CHECK_PS4_CONTROLLER_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_PS4_CONTROLLER_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_PS4_CONTROLLER_NODE_RECT)

        # SPEAKERS
        self.draw_text("Speakers", self.text_font, self.WHITE, self.CHARMIE_SPEAKERS_NODE_RECT.x+2*self.CHARMIE_SPEAKERS_NODE_RECT.width, self.CHARMIE_SPEAKERS_NODE_RECT.y-2)
        if self.check_nodes.CHECK_SPEAKERS_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_SPEAKERS_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_SPEAKERS_NODE_RECT)

        # YOLO OBJECTS
        self.draw_text("YOLO Objects", self.text_font, self.WHITE, self.CHARMIE_YOLO_OBJECTS_NODE_RECT.x+2*self.CHARMIE_YOLO_OBJECTS_NODE_RECT.width, self.CHARMIE_YOLO_OBJECTS_NODE_RECT.y-2)
        if self.check_nodes.CHECK_YOLO_OBJECTS_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_YOLO_OBJECTS_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_YOLO_OBJECTS_NODE_RECT)

        # YOLO POSE
        self.draw_text("YOLO Pose", self.text_font, self.WHITE, self.CHARMIE_YOLO_POSE_NODE_RECT.x+2*self.CHARMIE_YOLO_POSE_NODE_RECT.width, self.CHARMIE_YOLO_POSE_NODE_RECT.y-2)
        if self.check_nodes.CHECK_YOLO_POSE_NODE:
            pygame.draw.rect(self.WIN, self.GREEN, self.CHARMIE_YOLO_POSE_NODE_RECT)
        else:
            pygame.draw.rect(self.WIN, self.RED, self.CHARMIE_YOLO_POSE_NODE_RECT)

    def draw_cameras(self):
        

        self.curr_head_rgb = self.node.head_rgb
        self.curr_head_depth = self.node.head_depth
        self.curr_hand_rgb = self.node.hand_rgb
        self.curr_hand_depth = self.node.hand_depth

        if self.toggle_pause_cams.getValue():
            used_img_head_rgb = self.last_head_rgb
            used_img_head_depth = self.last_head_depth
            used_img_hand_rgb = self.last_hand_rgb
            used_img_hand_depth = self.last_hand_depth

        else:
            used_img_head_rgb = self.curr_head_rgb 
            used_img_head_depth = self.curr_head_depth 
            used_img_hand_rgb = self.curr_hand_rgb
            used_img_hand_depth = self.curr_hand_depth

        self.last_head_rgb = used_img_head_rgb 
        self.last_head_depth = used_img_head_depth 
        self.last_hand_rgb = used_img_hand_rgb
        self.last_hand_depth = used_img_hand_depth


        if not self.toggle_head_rgb_depth.getValue():

            if self.node.new_head_rgb:
                
                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_head_rgb, "bgr8")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD RGB): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_, 3), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cams_initial_height, 55, 4*self.cams_initial_height, self.BLACK, 85)
                self.draw_text(str(self.node.head_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cams_initial_height)
                self.draw_text(str(self.node.head_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, 3*self.cams_initial_height)

            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cams_initial_height+(self.cam_height_//2))
        else:
            if self.node.new_head_depth:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_head_depth, "passthrough")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD Depth): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                
                """
                # Get the minimum and maximum values in the depth image
                min_val, max_val, _, _ = cv2.minMaxLoc(opencv_image)

                # Avoid zero values (invalid measurements) in the depth image
                if min_val == 0:
                    min_val = np.min(opencv_image[opencv_image > 0])
                if max_val == 0:
                    max_val = np.max(opencv_image[opencv_image > 0])

                print(min_val, max_val)
                """

                min_val = 0
                max_val = 6000

                # Normalize the depth image to fall between 0 and 1
                # depth_normalized = cv2.normalize(opencv_image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                # Normalize the depth image to fall between 0 and 1
                depth_normalized = (opencv_image - min_val) / (max_val - min_val)
                depth_normalized = np.clip(depth_normalized, 0, 1)
                
                # Convert the normalized depth image to an 8-bit image (0-255)
                depth_8bit = (depth_normalized * 255).astype(np.uint8)

                # Apply a colormap to the 8-bit depth image
                opencv_image = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cams_initial_height, 55, 4*self.cams_initial_height, self.BLACK, 85)
                self.draw_text(str(self.node.head_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cams_initial_height)
                self.draw_text(str(self.node.head_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, 3*self.cams_initial_height)

            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cams_initial_height+(self.cam_height_//2))



        if not self.toggle_hand_rgb_depth.getValue():

            if self.node.new_hand_rgb:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_hand_rgb, "bgr8")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HAND RGB): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_, 3), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cam_height_+2*self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, 55, 4*self.cams_initial_height, self.BLACK, 85)
                self.draw_text(str(self.node.hand_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+2*self.cams_initial_height)
                self.draw_text(str(self.node.hand_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+3*self.cams_initial_height+self.cams_initial_height)
                
            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cam_height_+2*self.cams_initial_height+(self.cam_height_//2))

        else:

            if self.node.new_hand_depth:

                try:
                    opencv_image = self.br.imgmsg_to_cv2(used_img_hand_depth, "passthrough")
                except CvBridgeError as e:
                    self.node.get_logger().error(f"Conversion error (HEAD Depth): {e}")
                    opencv_image = np.zeros((self.cam_height_, self.cam_width_), np.uint8)
                
                opencv_image = cv2.resize(opencv_image, (self.cam_width_, self.cam_height_), interpolation=cv2.INTER_NEAREST)
                
                """
                # Get the minimum and maximum values in the depth image
                min_val, max_val, _, _ = cv2.minMaxLoc(opencv_image)

                # Avoid zero values (invalid measurements) in the depth image
                if min_val == 0:
                    min_val = np.min(opencv_image[opencv_image > 0])
                if max_val == 0:
                    max_val = np.max(opencv_image[opencv_image > 0])

                print(min_val, max_val)
                """

                min_val = 0
                max_val = 3000

                # Normalize the depth image to fall between 0 and 1
                # depth_normalized = cv2.normalize(opencv_image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                # Normalize the depth image to fall between 0 and 1
                depth_normalized = (opencv_image - min_val) / (max_val - min_val)
                depth_normalized = np.clip(depth_normalized, 0, 1)
                
                # Convert the normalized depth image to an 8-bit image (0-255)
                depth_8bit = (depth_normalized * 255).astype(np.uint8)

                # Apply a colormap to the 8-bit depth image
                opencv_image = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                
                # Convert the image to RGB (OpenCV loads as BGR by default)
                opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB)
                
                # Convert the image to a banded surface (Pygame compatible format)
                height, width, channels = opencv_image.shape
                # print(height, width)
                image_surface = pygame.image.frombuffer(opencv_image.tobytes(), (width, height), 'RGB')
                self.WIN.blit(image_surface, (self.cams_initial_width, self.cam_height_+2*self.cams_initial_height))
                self.draw_transparent_rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, 55, 4*self.cams_initial_height, self.BLACK, 85)
                self.draw_text(str(self.node.hand_rgb_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+2*self.cams_initial_height)
                self.draw_text(str(self.node.hand_depth_fps), self.text_font, self.WHITE, self.cams_initial_width, self.cam_height_+3*self.cams_initial_height+self.cams_initial_height)
                
            else:
                temp_rect = pygame.Rect(self.cams_initial_width, self.cam_height_+2*self.cams_initial_height, self.cam_width_, self.cam_height_)
                pygame.draw.rect(self.WIN, self.GREY, temp_rect)
                self.draw_text("No image available ...", self.text_font_t, self.WHITE, self.cams_initial_width+(self.cam_width_//3), self.cam_height_+2*self.cams_initial_height+(self.cam_height_//2))


        first_pos_h = 17.5
        self.draw_text("Record Data:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*first_pos_h)
        self.draw_text("Pause Cams:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(first_pos_h+2.5))
        self.draw_text("Depth Head:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(first_pos_h+5))
        self.draw_text("Depth Hand:", self.text_font_t, self.WHITE, 10, self.init_pos_h_rect_check_nodes+self.deviation_pos_h_rect_check_nodes*(first_pos_h+7.5))

    def draw_activates(self):


        self.draw_text("Activate YOLO Objects: (Head/Hand)", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, self.cams_initial_height)
        self.draw_text("Activate YOLO Pose:", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 80+self.cams_initial_height)
        self.draw_text("Activate Obstacles:", self.text_font_t, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 160+self.cams_initial_height)
        
        self.draw_text("Objects:      Furniture:      Shoes:      /      Objects:      Furniture:      Shoes:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 25+self.cams_initial_height)
        self.draw_text("Activate:      Waving:      Front Close:      Legs Visible:      Characteristics:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 80+25+self.cams_initial_height)
        self.draw_text("Lidar Top:      Lidar Bottom:      Head Camera:", self.text_font, self.WHITE, self.cams_initial_width+self.cam_width_+self.cams_initial_height, 160+25+self.cams_initial_height)
        

        # this is done to make sure that the same value used for checking pressed is the same used to attribute to last toggle
        # YOLO OBJECTS ACTIVATE
        toggle_activate_objects_head   = self.toggle_activate_objects_head.getValue()
        toggle_activate_furniture_head = self.toggle_activate_furniture_head.getValue()
        toggle_activate_shoes_head     = self.toggle_activate_shoes_head.getValue()
        toggle_activate_objects_hand   = self.toggle_activate_objects_hand.getValue()
        toggle_activate_furniture_hand = self.toggle_activate_furniture_hand.getValue()
        toggle_activate_shoes_hand     = self.toggle_activate_shoes_hand.getValue()

        # YOLO POSE ACTIVATE
        toggle_pose_activate        = self.toggle_pose_activate.getValue()
        toggle_pose_waving          = self.toggle_pose_waving.getValue()
        toggle_pose_front_close     = self.toggle_pose_front_close.getValue()
        toggle_pose_legs_visible    = self.toggle_pose_legs_visible.getValue()
        toggle_pose_characteristcs  = self.toggle_pose_characteristcs.getValue()

        # OBSTACLES ACTIVATE
        toggle_obstacles_lidar_top      = self.toggle_obstacles_lidar_top.getValue()
        toggle_obstacles_lidar_bottom   = self.toggle_obstacles_lidar_bottom.getValue()
        toggle_obstacles_head_camera    = self.toggle_obstacles_head_camera.getValue()

        if toggle_activate_objects_head     != self.last_toggle_activate_objects_head or \
            toggle_activate_furniture_head  != self.last_toggle_activate_furniture_head or \
            toggle_activate_shoes_head      != self.last_toggle_activate_shoes_head or \
            toggle_activate_objects_hand    != self.last_toggle_activate_objects_hand or \
            toggle_activate_furniture_hand  != self.last_toggle_activate_furniture_hand or \
            toggle_activate_shoes_hand      != self.last_toggle_activate_shoes_hand:
            
            print("YOLO OBJECTS - CHANGED STATUS.")

            self.activate_yolo_objects(activate_objects=toggle_activate_objects_head, activate_shoes=toggle_activate_shoes_head, \
                                       activate_doors=toggle_activate_furniture_head, activate_objects_hand=toggle_activate_objects_hand, \
                                       activate_shoes_hand=toggle_activate_shoes_hand, activate_doors_hand=toggle_activate_furniture_hand)


        if toggle_pose_activate         != self.last_toggle_pose_activate or \
            toggle_pose_waving          != self.last_toggle_pose_waving or \
            toggle_pose_front_close     != self.last_toggle_pose_front_close or \
            toggle_pose_legs_visible    != self.last_toggle_pose_legs_visible or \
            toggle_pose_characteristcs  != self.last_toggle_pose_characteristcs:
            
            print("YOLO POSE - CHANGED STATUS.")

            self.activate_yolo_pose(activate=toggle_pose_activate, only_detect_person_legs_visible=toggle_pose_legs_visible, \
                                    only_detect_person_right_in_front=toggle_pose_front_close, only_detect_person_arm_raised=toggle_pose_waving, \
                                    characteristics=toggle_pose_characteristcs)
        

        if toggle_obstacles_lidar_top     != self.last_toggle_obstacles_lidar_top or \
            toggle_obstacles_lidar_bottom != self.last_toggle_obstacles_lidar_bottom or \
            toggle_obstacles_head_camera  != self.last_toggle_obstacles_head_camera:
            
            print("OBSTACLES - CHANGED STATUS.")

            self.activate_obstacles(obstacles_lidar_up=toggle_obstacles_lidar_top, obstacles_lidar_bottom=toggle_obstacles_lidar_bottom, \
                                    obstacles_camera_head=toggle_obstacles_head_camera)


        self.last_toggle_activate_objects_head =   toggle_activate_objects_head 
        self.last_toggle_activate_furniture_head = toggle_activate_furniture_head
        self.last_toggle_activate_shoes_head =     toggle_activate_shoes_head
        self.last_toggle_activate_objects_hand =   toggle_activate_objects_hand
        self.last_toggle_activate_furniture_hand = toggle_activate_furniture_hand
        self.last_toggle_activate_shoes_hand =     toggle_activate_shoes_hand

        self.last_toggle_pose_activate =       toggle_pose_activate
        self.last_toggle_pose_waving =         toggle_pose_waving
        self.last_toggle_pose_front_close =    toggle_pose_front_close
        self.last_toggle_pose_legs_visible =   toggle_pose_legs_visible
        self.last_toggle_pose_characteristcs = toggle_pose_characteristcs
        
        self.last_toggle_obstacles_lidar_top =    toggle_obstacles_lidar_top
        self.last_toggle_obstacles_lidar_bottom = toggle_obstacles_lidar_bottom
        self.last_toggle_obstacles_head_camera =  toggle_obstacles_head_camera

    def draw_pose_detections(self):

        MIN_DRAW_CONF = 0.5
        CIRCLE_RADIUS = 4
        BB_WIDTH = 3
        MIN_KP_LINE_WIDTH = 3

        self.curr_detected_people = self.node.detected_people

        if self.toggle_pause_cams.getValue():
            used_detected_people = self.last_detected_people

        else:
            used_detected_people = self.curr_detected_people 

        self.last_detected_people = used_detected_people 

        # print(len(self.node.detected_people.persons))

        # if self.node.is_yolo_pose_comm:
           
        if len(used_detected_people.persons) > 0:
            print("DETECTED PEOPLE:")

        for p in used_detected_people.persons:

            print("id:", p.index_person, "acc:", round(p.conf_person,2), "loc:", p.room_location, "wave:", p.arm_raised, "point:", p.pointing_at)

            PERSON_BB = pygame.Rect(int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2), int(p.box_width/2), int(p.box_height/2))
            pygame.draw.rect(self.WIN, self.RED, PERSON_BB, width=BB_WIDTH)

            if int(p.box_top_left_y) < 30: # depending on the height of the box, so it is either inside or outside
                self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2), int(p.box_width/2), 30/2, self.RED, 85)
                self.draw_text("id:"+str(p.index_person)+" "+str(int(round(p.conf_person,2)*100))+"%", self.text_font_t, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2))
            else:
                self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2-30/2), int(p.box_width/2), 30/2, self.RED, 85)
                self.draw_text("id:"+str(p.index_person)+" "+str(int(round(p.conf_person,2)*100))+"%", self.text_font_t, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2-30/2))
            
            self.draw_line_between_two_keypoints(p.kp_nose_conf, p.kp_nose_x, p.kp_nose_y, p.kp_eye_left_conf, p.kp_eye_left_x, p.kp_eye_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_nose_conf, p.kp_nose_x, p.kp_nose_y, p.kp_eye_right_conf, p.kp_eye_right_x, p.kp_eye_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_eye_left_conf, p.kp_eye_left_x, p.kp_eye_left_y, p.kp_ear_left_conf, p.kp_ear_left_x, p.kp_ear_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_eye_right_conf, p.kp_eye_right_x, p.kp_eye_right_y, p.kp_ear_right_conf, p.kp_ear_right_x, p.kp_ear_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_ear_left_conf, p.kp_ear_left_x, p.kp_ear_left_y, p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_ear_right_conf, p.kp_ear_right_x, p.kp_ear_right_y, p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, self.GREEN, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            
            self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_elbow_left_conf, p.kp_elbow_left_x, p.kp_elbow_left_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_elbow_right_conf, p.kp_elbow_right_x, p.kp_elbow_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_elbow_left_conf, p.kp_elbow_left_x, p.kp_elbow_left_y, p.kp_wrist_left_conf, p.kp_wrist_left_x, p.kp_wrist_left_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_elbow_right_conf, p.kp_elbow_right_x, p.kp_elbow_right_y, p.kp_wrist_right_conf, p.kp_wrist_right_x, p.kp_wrist_right_y, self.BLUE_L, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            
            self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            # self.draw_line_between_two_keypoints(p.kp_shoulder_left_conf, p.kp_shoulder_left_x, p.kp_shoulder_left_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            # self.draw_line_between_two_keypoints(p.kp_shoulder_right_conf, p.kp_shoulder_right_x, p.kp_shoulder_right_y, p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            
            self.draw_line_between_two_keypoints(p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_hip_left_conf, p.kp_hip_left_x, p.kp_hip_left_y, p.kp_knee_left_conf, p.kp_knee_left_x, p.kp_knee_left_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_hip_right_conf, p.kp_hip_right_x, p.kp_hip_right_y, p.kp_knee_right_conf, p.kp_knee_right_x, p.kp_knee_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_knee_left_conf, p.kp_knee_left_x, p.kp_knee_left_y, p.kp_ankle_left_conf, p.kp_ankle_left_x, p.kp_ankle_left_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            self.draw_line_between_two_keypoints(p.kp_knee_right_conf, p.kp_knee_right_x, p.kp_knee_right_y, p.kp_ankle_right_conf, p.kp_ankle_right_x, p.kp_ankle_right_y, self.ORANGE, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
            
            self.draw_circle_keypoint(p.kp_nose_conf,           p.kp_nose_x,            p.kp_nose_y,            self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_eye_left_conf,       p.kp_eye_left_x,        p.kp_eye_left_y,        self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_eye_right_conf,      p.kp_eye_right_x,       p.kp_eye_right_y,       self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_ear_left_conf,       p.kp_ear_left_x,        p.kp_ear_left_y,        self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_ear_right_conf,      p.kp_ear_right_x,       p.kp_ear_right_y,       self.GREEN, MIN_DRAW_CONF, CIRCLE_RADIUS)
            
            self.draw_circle_keypoint(p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_elbow_left_conf,     p.kp_elbow_left_x,      p.kp_elbow_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_elbow_right_conf,    p.kp_elbow_right_x,     p.kp_elbow_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_wrist_left_conf,     p.kp_wrist_left_x,      p.kp_wrist_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_wrist_right_conf,    p.kp_wrist_right_x,     p.kp_wrist_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
            
            self.draw_circle_keypoint(p.kp_hip_left_conf,       p.kp_hip_left_x,        p.kp_hip_left_y,        self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_hip_right_conf,      p.kp_hip_right_x,       p.kp_hip_right_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_knee_left_conf,      p.kp_knee_left_x,       p.kp_knee_left_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_knee_right_conf,     p.kp_knee_right_x,      p.kp_knee_right_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_ankle_left_conf,     p.kp_ankle_left_x,      p.kp_ankle_left_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            self.draw_circle_keypoint(p.kp_ankle_right_conf,    p.kp_ankle_right_x,     p.kp_ankle_right_y,     self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
            
            self.check_face_for_characteristics(p, MIN_DRAW_CONF, BB_WIDTH)

    def draw_circle_keypoint(self, conf, x, y, color, min_draw_conf, circle_radius):
        if conf > min_draw_conf:
            pygame.draw.circle(self.WIN, color, (self.cams_initial_width+(x)/2, self.cams_initial_height+(y)/2), radius=circle_radius, width=0)
                
    def draw_line_between_two_keypoints(self, conf1, x1, y1, conf2, x2, y2, color, min_draw_conf, min_kp_line_width):
        if conf1 > min_draw_conf and conf2 > min_draw_conf:  
            pygame.draw.line(self.WIN, color, (self.cams_initial_width+(x1)/2, self.cams_initial_height+(y1)/2), (self.cams_initial_width+(x2)/2, self.cams_initial_height+(y2)/2), min_kp_line_width)
    
    def check_face_for_characteristics(self, p: DetectedPerson, min_draw_conf, bb_width):

        if p.gender != "None" or p.age_estimate != "None" or p.ethnicity != "None":

            if p.kp_shoulder_right_conf > min_draw_conf and p.kp_shoulder_left_conf > min_draw_conf and \
                p.kp_eye_right_conf > min_draw_conf and p.kp_eye_left_conf > min_draw_conf and p.kp_nose_conf > min_draw_conf:
            
                y1 = p.box_top_left_y
                y2 = max(p.kp_shoulder_right_y, p.kp_shoulder_left_y)
                y_height = y2-y1

                x1 = min(p.kp_shoulder_right_x, p.kp_shoulder_left_x, p.kp_nose_x, p.kp_eye_right_x, p.kp_eye_left_x)
                x2 = max(p.kp_shoulder_right_x, p.kp_shoulder_left_x, p.kp_nose_x, p.kp_eye_right_x, p.kp_eye_left_x)
                x_width = x2-x1
            
                # CHARS_BB = pygame.Rect(int(self.cams_initial_width+(x1)/2), int(self.cams_initial_height+(y1)/2), int(x_width/2), int(y_height/2))
                # pygame.draw.rect(self.WIN, self.YELLOW, CHARS_BB, width=bb_width)
                
                if int(p.box_top_left_y) < 30: # depending on the height of the box, so it is either inside or outside
                    self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2+30/2), int(p.box_width/2), 6*(30/2), self.RED, 85)
                    self.draw_text(str(p.gender), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),            int(self.cams_initial_height+(p.box_top_left_y)/2+1*(30/2)))
                    self.draw_text(str(p.ethnicity), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),         int(self.cams_initial_height+(p.box_top_left_y)/2+2*(30/2)))
                    self.draw_text(str(p.age_estimate), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),      int(self.cams_initial_height+(p.box_top_left_y)/2+3*(30/2)))
                    self.draw_text(str(round(p.height,2)), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),   int(self.cams_initial_height+(p.box_top_left_y)/2+4*(30/2)))
                    self.draw_text(str(p.shirt_color), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),       int(self.cams_initial_height+(p.box_top_left_y)/2+5*(30/2)))
                    self.draw_text(str(p.pants_color), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),       int(self.cams_initial_height+(p.box_top_left_y)/2+6*(30/2)))
                else:
                    self.draw_transparent_rect(int(self.cams_initial_width+(p.box_top_left_x)/2), int(self.cams_initial_height+(p.box_top_left_y)/2), int(p.box_width/2), 6*(30/2), self.RED, 85)
                    self.draw_text(str(p.gender), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),            int(self.cams_initial_height+(p.box_top_left_y)/2+0*(30/2)))
                    self.draw_text(str(p.ethnicity), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),         int(self.cams_initial_height+(p.box_top_left_y)/2+1*(30/2)))
                    self.draw_text(str(p.age_estimate), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),      int(self.cams_initial_height+(p.box_top_left_y)/2+2*(30/2)))
                    self.draw_text(str(round(p.height,2)), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),   int(self.cams_initial_height+(p.box_top_left_y)/2+3*(30/2)))
                    self.draw_text(str(p.shirt_color), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),       int(self.cams_initial_height+(p.box_top_left_y)/2+4*(30/2)))
                    self.draw_text(str(p.pants_color), self.text_font, self.BLACK, int(self.cams_initial_width+(p.box_top_left_x)/2),       int(self.cams_initial_height+(p.box_top_left_y)/2+5*(30/2)))
             
    def draw_object_detections(self):
        pass

    def check_record_data(self):
        
        if self.toggle_record.getValue() and not self.last_toggle_record:
            print("STARTED RECORDING")
            self.current_datetime = str(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))       
            self.video = cv2.VideoWriter(self.home+self.save_recordings_midpath+self.current_datetime+".avi", self.fourcc, self.FPS, (self.WIDTH, self.HEIGHT))

        if not self.toggle_record.getValue() and self.last_toggle_record:
            print("STOPPED RECORDING")
            self.video.release()

        if self.toggle_record.getValue():
            # transform the pixels to the format used by open-cv
            pixels = cv2.rotate(pygame.surfarray.pixels3d(self.WIN), cv2.ROTATE_90_CLOCKWISE)
            pixels = cv2.flip(pixels, 1)
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)

            # write the frame
            self.video.write(pixels)

        self.last_toggle_record = self.toggle_record.getValue()


    def main(self):


        clock = pygame.time.Clock()
        run = True
        while run:
            clock.tick(self.FPS)
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    pygame.quit()
                    run = False
                    quit()
            
            self.WIN.fill((0, 0, 0))
            self.draw_nodes_check()
            self.draw_cameras()
            self.draw_activates()
            self.draw_pose_detections()
            self.draw_object_detections()
            
            pygame_widgets.update(events)
            pygame.display.update()

            self.check_record_data()


# CHANGE ORDER OF HOW SERVICES ARE INITIALISED SO THAT WHEN I GET GREEN IT IS NOT WAITING FOR ANY OTHER SERVICE
# EXAMPLE (OBSTACLES WAITING FOR POINT CLOUD)

    # testar se arm ufactory se liga se nao tiver braço

    # NOT # por tudo percentual ao ecrã

    # NOT # pôr FPS a cada segundo (ou a cada d_t definido) e não quando há uma imagem nova, porque senão está sempre a oscilar ...

    # sistema para deixar de imprimir quando nao está a obter respostas dos yolos

    # quando se faz pause, pausar também as detecoes

    # pose:
    # corte cara caracteristicas

    # save video ao lado dos toggles da depth
    # nome da data
    # perceber o tamanho 

    # novo package gui
    # criar path para guardar os videos

    # criar copia do restaurante do robocup24
    # print people

# yolo objects head
# yolo objects hand

# MAPA