#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import  Yolov8Pose, DetectedPerson, NeckPosition

import cv2
import numpy as np
import math
import threading


class Robot():
    def __init__(self):
        print("New Robot Class Initialised")

        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.scale = 0.072*1000
        self.xx_shift = 0
        self.yy_shift = -350

        self.xc_adj = self.xc - self.xx_shift
        self.yc_adj = self.yc - self.yy_shift


        self.robot_radius = 0.560/2 # meter
        self.lidar_radius = 0.050/2 # meter
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = math.pi/4
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

        self.house_center_coordinates = (1.45, 4.95)
        self.house_left_bot_coordinates = (-4.05, 0.45)
        self.house_left_upp_coordinates = (-4.05, 9.45)
        self.house_right_bot_coordinates = (4.95, 0.45)
        self.house_right_upp_coordinates = (4.95, 9.45)

        self.house_left_bot_name = "Living Room"
        self.house_left_upp_name = "Kitchen"
        self.house_right_bot_name = "Office"
        self.house_right_upp_name = "Bedroom"

        self.house_divisions = []
        # self.people_in_frame = []
        # self.people_in_frame_filtered = []
        
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_left_bot_coordinates, self.house_left_bot_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_left_upp_coordinates, self.house_left_upp_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_right_bot_coordinates, self.house_right_bot_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_right_upp_coordinates, self.house_right_upp_name)

        self.neck_pan = 0.0
        self.neck_tilt = 0.0

        self.person_pose = Yolov8Pose()
        


    def coordinates_to_divisions(self, p1, p2, name):
        
        min_x = min(p1[0], p2[0])
        max_x = max(p1[0], p2[0])
        min_y = min(p1[1], p2[1])
        max_y = max(p1[1], p2[1])

        aux_dict = {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y, "name": name}

        self.house_divisions.append(aux_dict)


    def locate_divisions(self):
        
        location = "Outside"

        for loc in self.house_divisions:
            if loc['min_x'] < self.robot_x < loc['max_x'] and loc['min_y'] < self.robot_y < loc['max_y']:
                location = loc['name']

        # print(location)


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
            
            ### DRAWS THE HOUSE WALLS ###
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_bot_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_left_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_upp_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_left_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_left_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)

            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_bot_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_right_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)

            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_upp_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc_adj + self.scale*self.house_right_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int(self.xc_adj + self.scale*self.house_right_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)

            # cantos da casa + central
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.house_center_coordinates[0]), int(self.yc_adj - self.scale * self.house_center_coordinates[1])), (int)(self.scale*self.lidar_radius*4), (255, 0, 0), -1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.house_left_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_bot_coordinates[1])), (int)(self.scale*self.lidar_radius*4), (255, 0, 0), -1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.house_left_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_left_upp_coordinates[1])), (int)(self.scale*self.lidar_radius*4), (255, 0, 0), -1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.house_right_bot_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_bot_coordinates[1])), (int)(self.scale*self.lidar_radius*4), (255, 0, 0), -1)
            # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.house_right_upp_coordinates[0]), int(self.yc_adj - self.scale * self.house_right_upp_coordinates[1])), (int)(self.scale*self.lidar_radius*4), (255, 0, 0), -1)
           
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
            
            
            print("Test")
            for person in self.person_pose.persons:
                person_x = -person.position_relative.y/1000
                person_y =  person.position_relative.x/1000
                person_z =  person.position_relative.z/1000
                # print(person.position_relative.x/1000, person.position_relative.y/1000)

                # cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + (person.position_relative.y/1000)*self.scale*math.cos(self.robot_t + math.pi/2)),
                #     int(self.yc_adj - self.scale*self.robot_y - (person.position_relative.x/1000)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius*2), (0, 255, 255), -1)
           
                cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + person_x*self.scale),
                    int(self.yc_adj - self.scale*self.robot_y - person_y*self.scale)), (int)(self.scale*self.lidar_radius*5), (0, 255, 255), -1)

                angle_person = math.atan2(person_x, person_y)
                dist_person = math.sqrt(person_x**2 + person_y**2)

                theta_aux = math.pi/2 - (angle_person - self.robot_t)

                target_x = dist_person * math.cos(theta_aux) + self.robot_x
                target_y = dist_person * math.sin(theta_aux) + self.robot_y

                a_ref = (target_x, target_y)
                print("Rel:", (person_x, person_y), "Abs:", a_ref)
           
                cv2.circle(self.test_image, (int(self.xc_adj + self.scale*self.robot_x + target_x*self.scale),
                    int(self.yc_adj - self.scale*self.robot_y - target_y*self.scale)), (int)(self.scale*self.lidar_radius*5), (255, 255, 255), -1)

            # people
            # for people in self.people_in_frame_filtered: 
            #     cv2.circle(self.test_image, (int(self.xc + self.scale*people[0]), int(self.yc - self.scale * people[1])), (int)(self.scale*self.lidar_radius*5), (203, 192, 255), -1)
           


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
        self.get_neck_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)
        
        # get yolo pose person detection filtered
        self.person_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.get_person_pose_callback, 10)
        # self.person_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose", self.get_person_pose_callback, 10)




        self.robot = Robot()




    def get_neck_position_callback(self, pose: NeckPosition):
        
        print("Received new neck position. PAN = ", pose.pan, " TILT = ", pose.tilt)
        self.robot.neck_pan = -math.radians(180 - pose.pan)
        self.robot.neck_tilt = -math.radians(180 - pose.tilt)


    def get_person_pose_callback(self, pose: Yolov8Pose):
        
        print("Received new yolo pose. Number of people = ", pose.num_person)
        self.robot.person_pose = pose

    
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