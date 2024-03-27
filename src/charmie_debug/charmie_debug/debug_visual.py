#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, String, Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from charmie_interfaces.msg import  Yolov8Pose, Yolov8Objects, DetectedPerson, NeckPosition, ListOfPoints
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
        self.scale = 0.057*1000
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
        
        with open(self.complete_path + 'rooms_location.json', encoding='utf-8') as json_file:
            self.house_rooms = json.load(json_file)
        print(self.house_rooms)

        with open(self.complete_path + 'furniture_location.json', encoding='utf-8') as json_file:
            self.house_furniture = json.load(json_file)
        print(self.house_furniture)

        with open(self.complete_path + 'doors_location.json', encoding='utf-8') as json_file:
            self.house_doors = json.load(json_file)
        print(self.house_doors)

        self.neck_pan = 0.0
        self.neck_tilt = 0.0

        self.person_pose = Yolov8Pose()
        self.object_detected = Yolov8Objects()
        self.search_for_person = ListOfPoints()

        self.linhas = 720
        self.colunas = 1280
        self.current_frame = np.zeros((self.linhas, self.colunas,3), dtype=np.uint8)


    def odometry_msg_to_position(self, odom: Odometry):
        
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(self.robot_x, self.robot_y, self.robot_t)


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
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                         int(self.yc_adj - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius)+2, (0, 255, 255), -1)
            
            # NECK DIRECTION, CAMERA FOV
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), 
                     (int(self.xc_adj + self.scale*self.robot_x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_t + self.neck_pan + math.pi/2 - math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_t + self.neck_pan + math.pi/2 - math.pi/4))), (0,255,255), 1)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.robot_x), int(self.yc_adj - self.scale * self.robot_y)), 
                     (int(self.xc_adj + self.scale*self.robot_x + (self.neck_visual_lines_length)*self.scale*math.cos(self.robot_t + self.neck_pan + math.pi/2 + math.pi/4)),
                      int(self.yc_adj - self.scale*self.robot_y - (self.neck_visual_lines_length)*self.scale*math.sin(self.robot_t + self.neck_pan + math.pi/2 + math.pi/4))), (0,255,255), 1)
                       

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
           

                cv2.circle(self.test_image, (int(self.xc_adj + person.position_relative.x*self.scale),
                    int(self.yc_adj - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 255, 255), -1)
                
                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person.position_relative.x*self.scale),
                #     int(self.yc_adj - self.scale*self.robot_y - person.position_relative.y*self.scale)), (int)(self.scale*self.lidar_radius*3), (0, 255, 255), -1)

            for object in self.object_detected.objects:
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           

                cv2.rectangle(self.test_image, 
                              (int(self.xc_adj + object.position_relative.x*self.scale + self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_relative.y*self.scale + self.lidar_radius*2*self.scale)),
                              (int(self.xc_adj + object.position_relative.x*self.scale - self.lidar_radius*2*self.scale), int(self.yc_adj - object.position_relative.y*self.scale - self.lidar_radius*2*self.scale)),
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

        # get neck position
        self.get_neck_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)
        
        # get yolo pose person detection filtered
        self.person_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.get_person_pose_callback, 10)

        self.object_detected_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.get_object_detected_callback, 10)

        # get robot_localisation
        self.localisation_robot_subscriber = self.create_subscription(Odometry, "odom_a", self.odom_robot_callback, 10)

        # search for person, person localisation 
        self.search_for_person_subscriber = self.create_subscription(ListOfPoints, "search_for_person_points", self.search_for_person_callback, 10)
        
        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        
        self.robot = Robot()
        

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



    def odom_robot_callback(self, loc: Odometry):
        self.robot.odometry_msg_to_position(loc)


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


    def main(self):

        while True:
            # pass
            self.node.robot.update_debug_drawings()