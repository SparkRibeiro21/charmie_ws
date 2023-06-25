#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from charmie_interfaces.msg import TarNavSDNL, Obstacles

import cv2
import numpy as np
import math

class NavigationSDNLClass:

    def __init__(self):

        self.lambda_target = 10
        self.beta1 = 300
        self.beta2 = 400

        self.obstacles = Obstacles()
        # self.position = Odometry()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0

        # visual debug
        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.scale = 0.12*1000
        self.robot_radius = 0.560/2
        self.lidar_radius = 0.050/2
        self.all_pos_x_val = []
        self.all_pos_y_val = []
        self.all_pos_t_val = []
        self.all_obs_val = []
    

    def update_debug_drawings(self):
            
        if self.DEBUG_DRAW_IMAGE:

            # 1 meter lines horizontal and vertical
            for i in range(10):
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

            # obstacles
            self.all_obs_val.append(self.obstacles)
            
            for j in range(len(self.all_obs_val)):

                for i in range(self.all_obs_val[j].no_obstacles):

                    # aux variables
                    aux_ang = self.all_obs_val[j].obstacles[i].alfa
                    aux_dist = self.all_obs_val[j].obstacles[i].dist + self.robot_radius
                    aux_len_cm = self.all_obs_val[j].obstacles[i].length_cm
                    thickness = 20

                    # line length of obstacle at dist and alfa detected
                    cv2.line(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (0, 165, 255), int(1.0 + thickness*self.scale/1000))
             

            
            for i in range(self.obstacles.no_obstacles):

                # aux variables
                aux_ang = self.obstacles.obstacles[i].alfa
                aux_dist = self.obstacles.obstacles[i].dist + self.robot_radius
                aux_len_cm = self.obstacles.obstacles[i].length_cm
                thickness = 20

                #line robot center to obstacle center
                # cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)),
                #                         (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                #                         int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                #                         (255, 255, 255))

                # line robot platform to obstacle center
                cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x - self.scale * (self.robot_radius) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                                           int(self.yc - self.scale*self.robot_y - self.scale * (self.robot_radius) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                                          (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                                           int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                                          (255, 255, 255))
                
                # line length of obstacle at dist and alfa detected
                cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (0, 0, 255), int(1.0 + thickness*self.scale/1000))
                  

            # robot
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius), (0, 255, 255), -1)
            



            cv2.imshow("Navigation SDNL", self.test_image)
            
            k = cv2.waitKey(1)
            if k == ord('+'):
                self.scale /= 0.8
            if k == ord('-'):
                self.scale *= 0.8
            if k == ord('0'):
                self.all_pos_x_val.clear()
                self.all_pos_y_val.clear()

            self.test_image[:, :] = 0

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
        print(self.robot_x, self.robot_y, self.robot_t)

    def obstacles_msg_to_position(self, obs: Obstacles):
        self.obstacles = obs
        print(obs)

        
class NavSDNLNode(Node):

    def __init__(self):
        super().__init__("NavigationSDNL")
        self.get_logger().info("Initialised CHARMIE NavigationSDNL Node")

        # Create Code Class Instance
        self.nav = NavigationSDNLClass() 

        # Create PUBs/SUBs
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obs_lidar_callback, 10)
        self.odom_robot_subscriber = self.create_subscription(Odometry, "odom_robot", self.odom_robot_callback, 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_start_button", 10)

        # Create Timers
        # self.create_timer(1, self.timer_callback)

    def obs_lidar_callback(self, obs: Obstacles):
        # updates the obstacles variable
        self.nav.obstacles_msg_to_position(obs)
        self.nav.update_debug_drawings()

    def odom_robot_callback(self, odom: Odometry):
        # updates the position variable
        self.nav.odometry_msg_to_position(odom)
        self.nav.update_debug_drawings()

    def target_pos_callback(self, flag: TarNavSDNL):
        # calculates the velocities and sends them to the motors considering the latest obstacles and odometry position
        pass

    # def timer_callback(self):
    #     aux = TarNavSDNL()
    #     print(aux)
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node = NavSDNLNode()
    rclpy.spin(node)
    rclpy.shutdown()
