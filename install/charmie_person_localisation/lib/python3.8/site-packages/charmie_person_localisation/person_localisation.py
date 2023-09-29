#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import  Yolov8Pose, Keypoints

import cv2
import numpy as np
import math

class PersonLocalisationClass:

    def __init__(self):

        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.scale = 0.12*1000
        self.robot_radius = 0.560/2 # meter
        self.lidar_radius = 0.050/2 # meter
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
        self.all_pos_x_val = []
        self.all_pos_y_val = []
        self.all_pos_t_val = []
        
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
        self.house_right_bot_name = "Office or Study"
        self.house_right_upp_name = "Bedroom"

        self.house_divisions = []
        self.people_in_frame = []
        self.people_in_frame_filtered = []
        
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_left_bot_coordinates, self.house_left_bot_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_left_upp_coordinates, self.house_left_upp_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_right_bot_coordinates, self.house_right_bot_name)
        self.coordinates_to_divisions(self.house_center_coordinates, self.house_right_upp_coordinates, self.house_right_upp_name)



        # 
        # print(self.house_divisions)
        

    def person_inside_forbidden_room(self):
        
        x = 0.0
        y = 0.0
        t = 0.0



        if len(self.people_in_frame_filtered) == 0:
            self.t_ctr2 += 1
        else:
            self.t_ctr2 = 0

            x = self.people_in_frame_filtered[0][0]
            y = self.people_in_frame_filtered[0][1]

            # print(x, self.house_divisions[3]['min_x']-0.5)
            if x > self.house_divisions[3]['min_x']-0.5:  # and y > self.house_divisions[3]['min_y']:
                self.t_ctr = 0.0
            else:
                self.t_ctr += 1
                # t = 1.0


            if self.t_ctr > 5:
                t = 1.0
            else:
                t = 0.0




        if self.t_ctr2 > 100:
            t = -1.0


        if x == 0 and y == 0:
            x = self.x_ant
            y = self.y_ant


        self.x_ant = x
        self.y_ant = y

        return x, y, t

        





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

            # 1 meter lines horizontal and vertical
            for i in range(20):
                cv2.line(self.test_image, (int(self.xc + self.scale*i), 0), (int(self.xc + self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (int(self.xc - self.scale*i), 0), (int(self.xc - self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc - self.scale*i)), (self.yc*2, int(self.yc - self.scale*i)), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc + self.scale*i)), (self.yc*2, int(self.yc + self.scale*i)), (255, 255, 255), 1)
            
            # present and past localization positions
            self.all_pos_x_val.append(self.robot_x)
            self.all_pos_y_val.append(self.robot_y)
            self.all_pos_t_val.append(self.robot_t)
            for i in range(len(self.all_pos_x_val)):
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[i]), int(self.yc - self.scale * self.all_pos_y_val[i])), 1, (255, 255, 0), -1)

            # robot
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius), (0, 255, 255), -1)
            
            
            
            
            # HOUSE (made in robocup, can make a function to improve, did not have tome for that)


            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_left_bot_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_left_bot_coordinates[1])), (int(self.xc + self.scale*self.house_left_bot_coordinates[0]), int(self.yc - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_left_bot_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_left_bot_coordinates[0]), int(self.yc - self.scale * self.house_left_bot_coordinates[1])), (255,0,255), 2)
            
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_left_upp_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_left_upp_coordinates[1])), (int(self.xc + self.scale*self.house_left_upp_coordinates[0]), int(self.yc - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_left_upp_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_left_upp_coordinates[0]), int(self.yc - self.scale * self.house_left_upp_coordinates[1])), (255,0,255), 2)

            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_right_bot_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_right_bot_coordinates[1])), (int(self.xc + self.scale*self.house_right_bot_coordinates[0]), int(self.yc - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_right_bot_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_right_bot_coordinates[0]), int(self.yc - self.scale * self.house_right_bot_coordinates[1])), (255,0,255), 2)

            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_right_upp_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_right_upp_coordinates[1])), (int(self.xc + self.scale*self.house_right_upp_coordinates[0]), int(self.yc - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)
            cv2.line(self.test_image, (int(self.xc + self.scale*self.house_right_upp_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int(self.xc + self.scale*self.house_right_upp_coordinates[0]), int(self.yc - self.scale * self.house_right_upp_coordinates[1])), (255,0,255), 2)



            # cantos da casa + central
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.house_center_coordinates[0]), int(self.yc - self.scale * self.house_center_coordinates[1])), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.house_left_bot_coordinates[0]), int(self.yc - self.scale * self.house_left_bot_coordinates[1])), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.house_left_upp_coordinates[0]), int(self.yc - self.scale * self.house_left_upp_coordinates[1])), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.house_right_bot_coordinates[0]), int(self.yc - self.scale * self.house_right_bot_coordinates[1])), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.house_right_upp_coordinates[0]), int(self.yc - self.scale * self.house_right_upp_coordinates[1])), (int)(self.scale*self.lidar_radius*5), (255, 0, 0), -1)
           


            # print(self.people_in_frame)



            # people
            for people in self.people_in_frame_filtered: 
                cv2.circle(self.test_image, (int(self.xc + self.scale*people[0]), int(self.yc - self.scale * people[1])), (int)(self.scale*self.lidar_radius*5), (203, 192, 255), -1)
           


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





class PersonLocalisationNode(Node):

    def __init__(self):
        super().__init__("NavigationSDNL")
        self.get_logger().info("Initialised CHARMIE Person Localisation Node")

        # Create Code Class Instance
        self.per_loc = PersonLocalisationClass() 

        # Create PUBs/SUBs
        self.localisation_robot_subscriber = self.create_subscription(Odometry, "odom_a", self.odom_robot_callback, 10)
        
        #YOLOv8Pose
        self.yolo_pose_subscriber = self.create_subscription(Yolov8Pose, "yolov8_pose", self.yolo_pose_callback, 10)
        
        #Comms with Main
        self.get_person_subscriber = self.create_subscription(Bool, "get_person", self.get_person_callback, 10)
        self.person_info_publisher = self.create_publisher(Pose2D, "person_info", 10)
        
        self.flag_get_person = False
        self.first_time = False

        
    def get_person_callback(self, flag: Bool):
        self.flag_get_person = flag
        print("Received Flag Get Person")
        self.first_time = True

    def odom_robot_callback(self, loc: Odometry):

        self.per_loc.odometry_msg_to_position(loc)


    def yolo_pose_callback(self, pose: Yolov8Pose):
        # aux_people_in_frame = []
        aux_people_in_frame_ref_robot = []
        aux_people_in_frame_ref_robot_filtered = []
        # print(pose.keypoints)

        for people in pose.keypoints:
            x = people.x_person_relative
            y = people.average_distance
            h = people.box_height

            # a = (x,y,h)
            # aux_people_in_frame.append(a)

            angle_person = math.atan2(x, y)
            dist_person = math.sqrt(x**2 + y**2)

            theta_aux = math.pi/2 - (angle_person - self.per_loc.robot_t)

            target_x = dist_person * math.cos(theta_aux) + self.per_loc.robot_x
            target_y = dist_person * math.sin(theta_aux) + self.per_loc.robot_y


            # filters por person detection
            if h > 650:
                a_ref_filtered = (target_x, target_y)
                aux_people_in_frame_ref_robot_filtered.append(a_ref_filtered)


            a_ref = (target_x, target_y)
            aux_people_in_frame_ref_robot.append(a_ref) 



        self.per_loc.people_in_frame = aux_people_in_frame_ref_robot
        
        
        self.per_loc.people_in_frame_filtered = aux_people_in_frame_ref_robot_filtered
        self.per_loc.update_debug_drawings()
        self.per_loc.locate_divisions()



        if self.flag_get_person:
            if self.first_time:
                self.per_loc.t_ctr = 0.0
                self.per_loc.t_ctr2 = 100+1
                self.first_time = False
            x_, y_, t_ = self.per_loc.person_inside_forbidden_room()
            # print(x_, y_, t_)
            print(t_)
            aaa = Pose2D()
            aaa.x = x_
            aaa.y = y_
            aaa.theta = t_
            self.person_info_publisher.publish(aaa)









def main(args=None):
    rclpy.init(args=args)
    node = PersonLocalisationNode()
    rclpy.spin(node)
    rclpy.shutdown()
