#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Quaternion
# from charmie_interfaces.msg import Encoders

import numpy as np
import math


class EKFLocalisation:

    def __init__(self):
        print("Executing Main Code")
        # Initialize the EKF state and covariance matrices
        self.state = None  # EKF state
        self.covariance = None  # Covariance matrix


class LocalisationNode(Node):

    def __init__(self):
        super().__init__("Localisation")
        self.get_logger().info("Initialised CHARMIE Localisation Node")

        # Create Code Class Instance
        self.ekf = EKFLocalisation() 

        self.odom_robot_subscriber = self.create_subscription(Odometry, "odom", self.odom_robot_callback, 10)
        self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_pose_callback, 10)
        # falta o subscriber do debug main para o nosso init pose
        self.init_loc_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "init_loc_pose", self.init_loc_pose_callback, 10)

        # self.initial_pose_amcl_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.initial_pose_amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.initial_pose_amcl_callbak, 10) # aux temp
        self.localisation_publisher = self.create_publisher(Odometry, "amcl_tr", 10)
        # publisher_auxiliar só para testes:
        self.amcl_alone_localisation_publisher = self.create_publisher(Odometry, "odom_a", 10)

        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)


        self.odom_robot = Odometry()
        self.conv_amcl = Odometry()
        self.amcl_loc = PoseWithCovarianceStamped()

        self.robot_x_fused = 0.0
        self.robot_y_fused = 0.0
        self.robot_t_fused = 0.0

        self.temp_robot_x_for_odom = 0.0
        self.temp_robot_y_for_odom = 0.0
        self.temp_robot_t_for_odom = 0.0

        self.previous_odom_x = 0.0
        self.previous_odom_y = 0.0
        self.previous_odom_t = 0.0

        self.received_init_pos = False
        self.updated_amcl_loc = False

        self.last_fixed_location_x = 0.0
        self.last_fixed_location_y = 0.0
        self.last_fixed_location_t = 0.0

        # self.fixed_odometry_orientation = 0.0


        # temp
        self.init_pose_test = True
        self.stop = False
        self.amcl_correction_ctr = 0


        self.a = 0

    def odom_robot_callback(self, odom:Odometry):

        self.odom_robot = odom
        # self.amcl_alone_localisation_publisher.publish(self.odom_robot)

        # calculate variation values since last update 

        # print(self.odom_robot)
        new_orientation = self.quat_to_rad(self.odom_robot.pose.pose.orientation)
        # self.fixed_odometry_orientation = new_orientation # ???

        # if not self.updated_amcl_loc:
        curr_x = self.odom_robot.pose.pose.position.x - self.previous_odom_x
        curr_y = self.odom_robot.pose.pose.position.y - self.previous_odom_y
        curr_t = new_orientation - self.previous_odom_t
        # else:
        # curr_x = 0.0
        # curr_y = 0.0
        # curr_t = 0.0
          #   self.updated_amcl_loc = False



        # print("CURR:", curr_x, curr_y, curr_t)
        
        # self.odom_robot.pose.pose.position.x 
        # self.odom_robot.pose.pose.position.y 
        # self.odom_robot.pose.pose.orientation 

        # calculate stuff and finally conver back to rad to publish

        # self.robot_t_fused += curr_t



        # summed_x = self.robot_x_fused + curr_x
        # summed_y = self.robot_y_fused + curr_y
        # summed_t = self.robot_t_fused + curr_t

        # self.robot_t_fused += curr_t
        # self.robot_x_fused += (curr_x*math.cos(math.pi/2+self.robot_t_fused))
        # self.robot_y_fused += (curr_y*math.sin(math.pi/2+self.robot_t_fused))
        # print("FUSED:", self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)


        # curr_x = 0

        # self.robot_t_fused += curr_t
        # self.robot_x_fused += (curr_x*math.cos(math.pi/2+self.robot_t_fused) - curr_y*math.sin(math.pi/2+self.robot_t_fused))
        # self.robot_y_fused -= (curr_x*math.sin(math.pi/2+self.robot_t_fused) + curr_y*math.cos(math.pi/2+self.robot_t_fused))


        odom_to_p = Odometry()
        if self.updated_amcl_loc:
            self.updated_amcl_loc = False
            amcl_temp = self.PoseWithCovarianceStamped_to_Odometry(self.amcl_loc)


            self.robot_x_fused = amcl_temp.pose.pose.position.x
            self.robot_y_fused = amcl_temp.pose.pose.position.y
            aux= self.quat_to_rad(amcl_temp.pose.pose.orientation) # + self.fixed_odometry_orientation
            
            
            print("POS_FIN_AMCL", self.robot_x_fused, self.robot_y_fused, aux)


            # print(self.last_fixed_location_t)
            # print(aux)

            # self.amcl_alone_localisation_publisher.publish(amcl_temp)


            # print(self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)


        self.a += curr_t
        theta = self.last_fixed_location_t
        self.robot_x_fused += (curr_x*math.cos(theta) - curr_y*math.sin(theta))
        self.robot_y_fused += (curr_x*math.sin(theta) + curr_y*math.cos(theta))
            
        odom_to_p.pose.pose.position.x = self.robot_x_fused # x corrdinates
        odom_to_p.pose.pose.position.y = self.robot_y_fused # y corrdinates 
        
        # quaternion = self.rad_to_quat(self.last_fixed_location_t+self.a)
        quaternion = self.rad_to_quat(self.last_fixed_location_t+self.a)
        odom_to_p.pose.pose.orientation.x = quaternion[0]
        odom_to_p.pose.pose.orientation.y = quaternion[1]
        odom_to_p.pose.pose.orientation.z = quaternion[2]
        odom_to_p.pose.pose.orientation.w = quaternion[3]



        # print("POS_FIN_ENC", self.robot_x_fused, self.robot_y_fused, self.last_fixed_location_t+self.a)


            
            # if self.received_init_pos:
            #     pass
            
            # if self.amcl_correction_ctr >= 20:
            #     pass
            # else:
        self.amcl_alone_localisation_publisher.publish(odom_to_p)




        self.previous_odom_x = self.odom_robot.pose.pose.position.x 
        self.previous_odom_y = self.odom_robot.pose.pose.position.y 
        self.previous_odom_t = new_orientation


    def amcl_pose_callback(self, amcl_p:PoseWithCovarianceStamped):
        self.amcl_loc = amcl_p
        covariance = amcl_p.pose.covariance
        std_deviations = np.sqrt(np.diag(covariance))
        # root mean square error
        rmse = np.sqrt(np.sum(np.diag(covariance)))
        # print(std_deviations)
        # print(rmse)



        if rmse < 0.8: # 1.0: # 0.6:
            # self.conv_amcl = self.PoseWithCovarianceStamped_to_Odometry(amcl_p)
            # self.amcl_alone_localisation_publisher.publish(self.conv_amcl)
                      
            self.amcl_correction_ctr += 1 
            self.updated_amcl_loc = True
            print(rmse)
            
            # print("UPDATED AMCL", self.amcl_correction_ctr)
            
            # self.last_fixed_location_x = -amcl_p.pose.pose.position.y
            # self.last_fixed_location_y = amcl_p.pose.pose.position.x
            # self.last_fixed_location_t = 
            # aux = self.quat_to_rad(amcl_p.pose.pose.orientation) # + self.fixed_odometry_orientation

            # self.robot_x_fused = self.last_fixed_location_x
            # self.robot_y_fused = self.last_fixed_location_y
            # self.robot_t_fused = self.last_fixed_location_t
        else:

            print("REJECTED AMCL:",rmse)


            
            

    def init_loc_pose_callback(self, p:PoseWithCovarianceStamped):
        pass


    def initial_pose_amcl_callbak(self, init: PoseWithCovarianceStamped):
        # receive initial pose from rviz, this is just temporary code, must be commented and changed to publisher
        pass
        if self.init_pose_test:
            self.init_pose_test = False
            # self.received_init_pos = True
            # print(init)
            self.last_fixed_location_x = -init.pose.pose.position.y
            self.last_fixed_location_y = init.pose.pose.position.x
            self.last_fixed_location_t = self.quat_to_rad(init.pose.pose.orientation)#  + self.fixed_odometry_orientation

            self.robot_x_fused = self.last_fixed_location_x
            self.robot_y_fused = self.last_fixed_location_y
            self.robot_t_fused = self.last_fixed_location_t
            
            # self.updated_amcl_loc = True
            # print("UPDATED INIT")

            print("POS_INIT", self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)
            
        
            # print(self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)
        










    def PoseWithCovarianceStamped_to_Odometry(self, pose_c: PoseWithCovarianceStamped):
        od = Odometry()
                
        od.pose.pose.position.x = -pose_c.pose.pose.position.y
        od.pose.pose.position.y = pose_c.pose.pose.position.x
        od.pose.pose.orientation = pose_c.pose.pose.orientation
        
        return od
    
    def quat_to_rad(self, q: Quaternion):

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)

        return math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
        
    def rad_to_quat(self, yaw):
        roll = 0
        pitch = 0

        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    rclpy.spin(node)
    rclpy.shutdown()
