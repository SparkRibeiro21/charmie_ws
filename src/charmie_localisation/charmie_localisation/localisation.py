#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int16, Bool
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
        
        
        # self.is_first_odom = True 
        self.is_first_odom = False


        self.rot_inc = 0


        self.erro_ini_t = 0.0

        self.xxx = 0.0
        self.yyy = 0.0


        self.loacalisation_diagnostic_publisher = self.create_publisher(Bool, "localisation_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.loacalisation_diagnostic_publisher.publish(flag_diagn)



    def odom_robot_callback(self, odom:Odometry):

        
        # print("###")
        # tr = self.rad_to_quat(math.pi/2)
        # print(tr)
        # qqq = Quaternion()
        # qqq.x = tr[0]
        # qqq.y = tr[1]
        # qqq.z = tr[2]
        # qqq.w = tr[3]
        # tr_q = self.quat_to_rad(qqq)
        # print(tr_q)
        # print("---")



        # mesmo  inicial : 2.150255047240044
        # antes da transição inicial: 1.9765148120108569


        self.odom_robot = odom
        # print("ini_t_enc:", self.quat_to_rad(self.odom_robot.pose.pose.orientation))
        # print("erro_ini_t:", self.erro_ini_t)
        
        
        
        # Temporary for when amcl is not implemtned and want the localisation to be precisely the odometry
        # self.amcl_alone_localisation_publisher.publish(self.odom_robot)


       
        if not self.is_first_odom:
            # self.amcl_alone_localisation_publisher.publish(self.odom_robot)

            # calculate variation values since last update 


            # print(self.odom_robot)
            new_orientation = self.quat_to_rad(self.odom_robot.pose.pose.orientation)
            # self.fixed_odometry_orientation = new_orientation # ???

            # if not self.updated_amcl_loc:
            curr_x = self.odom_robot.pose.pose.position.x - self.previous_odom_x
            curr_y = self.odom_robot.pose.pose.position.y - self.previous_odom_y
            curr_t = new_orientation - self.previous_odom_t


            if self.updated_amcl_loc:
                self.updated_amcl_loc = False
                amcl_temp = self.PoseWithCovarianceStamped_to_Odometry(self.amcl_loc)
                # self.rot_inc = 0


                self.robot_x_fused = amcl_temp.pose.pose.position.x
                self.robot_y_fused = amcl_temp.pose.pose.position.y
                self.rot_inc= self.quat_to_rad(amcl_temp.pose.pose.orientation) # + self.fixed_odometry_orientation
                
                
                # print("POS_FIN_AMCL", self.robot_x_fused, self.robot_y_fused, aux)


                # print(self.last_fixed_location_t)
                # print(aux)

                # self.amcl_alone_localisation_publisher.publish(amcl_temp)


                # print(self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)


            self.rot_inc += curr_t
            self.robot_y_fused += curr_y # tested in zero initial angle
            self.robot_x_fused += curr_x # tested in zero initial angle



            odom_to_p = Odometry()
            odom_to_p.pose.pose.position.x = self.robot_x_fused # x corrdinates
            odom_to_p.pose.pose.position.y = self.robot_y_fused # y corrdinates --
            
            # quaternion = self.rad_to_quat(self.last_fixed_location_t+self.a)
            quaternion = self.rad_to_quat(self.rot_inc)

            # print(self.last_fixed_location_t+self.rot_inc)


            odom_to_p.pose.pose.orientation.x = quaternion[0]
            odom_to_p.pose.pose.orientation.y = quaternion[1]
            odom_to_p.pose.pose.orientation.z = quaternion[2]
            odom_to_p.pose.pose.orientation.w = quaternion[3]


            # print("POS_FIN_ENC", self.robot_x_fused, self.robot_y_fused, self.last_fixed_location_t+self.a)
            self.amcl_alone_localisation_publisher.publish(odom_to_p)


            self.previous_odom_x = self.odom_robot.pose.pose.position.x 
            self.previous_odom_y = self.odom_robot.pose.pose.position.y 
            self.previous_odom_t = new_orientation




        """
            # print("CURR:", curr_x, curr_y, curr_t)
            
            # theta_aux = math.pi/2 - 2.150255047240044
            # self.xxx = (curr_x*math.cos(theta_aux) - curr_y*math.sin(theta_aux))
            # self.yyy = (curr_x*math.sin(theta_aux) + curr_y*math.cos(theta_aux))

            # print("xx:", self.xxx, "yy:", self.yyy)


            # xx = (self.xxx*math.cos(theta_aux) - self.yyy*math.sin(theta_aux))
            # yy = (self.xxx*math.sin(theta_aux) + self.yyy*math.cos(theta_aux))

            # print("xx:", xx, "yy:", yy)


            # self.robot_x_fused += xx
            # self.robot_y_fused += yy
            

            # print(curr_t)
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
                self.rot_inc = 0


                # self.robot_x_fused = amcl_temp.pose.pose.position.x
                # self.robot_y_fused = amcl_temp.pose.pose.position.y
                # aux= self.quat_to_rad(amcl_temp.pose.pose.orientation) # + self.fixed_odometry_orientation
                # print("POS_FIN_AMCL", self.robot_x_fused, self.robot_y_fused, aux)


                # print(self.last_fixed_location_t)
                # print(aux)

                self.amcl_alone_localisation_publisher.publish(amcl_temp)


                # print(self.robot_x_fused, self.robot_y_fused, self.robot_t_fused)


            self.rot_inc += curr_t # checked
            # theta = math.pi/2 + self.last_fixed_location_t # com este angulo, o yy quando etste o sempre em frente no inicio vai nesta direçção, mas nao anda muito
            theta = self.last_fixed_location_t
            # print("theta:", theta)
            # curr_x = 0
            
            # self.robot_x_fused += curr_y # tested in zero initial angle
            # self.robot_y_fused -= curr_x # tested in zero initial angle

            # self.robot_x_fused += (curr_x*math.cos(theta) - curr_y*math.sin(theta))
            # self.robot_y_fused += (curr_x*math.sin(theta) + curr_y*math.cos(theta))
            
            # self.robot_y_fused -= (curr_x*math.cos(theta) - curr_y*math.sin(theta))
            # self.robot_x_fused += (curr_x*math.sin(theta) + curr_y*math.cos(theta))
            
            # self.robot_x_fused += self.xxx
            # self.robot_y_fused += self.yyy
            

            

            odom_to_p.pose.pose.position.x = self.robot_x_fused # x corrdinates
            odom_to_p.pose.pose.position.y = self.robot_y_fused # y corrdinates --
            
            # quaternion = self.rad_to_quat(self.last_fixed_location_t+self.a)
            quaternion = self.rad_to_quat(self.last_fixed_location_t+self.rot_inc)

            print(self.last_fixed_location_t+self.rot_inc)


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
            # self.amcl_alone_localisation_publisher.publish(odom_to_p)




            self.previous_odom_x = self.odom_robot.pose.pose.position.x 
            self.previous_odom_y = self.odom_robot.pose.pose.position.y 
            self.previous_odom_t = new_orientation
        
        else: # first time que faz odom, para os valores anteriores da primiera vez nao darem saltos
            self.previous_odom_x = self.odom_robot.pose.pose.position.x 
            self.previous_odom_y = self.odom_robot.pose.pose.position.y 
            self.previous_odom_t = self.quat_to_rad(self.odom_robot.pose.pose.orientation)
            self.erro_ini_t = self.quat_to_rad(self.odom_robot.pose.pose.orientation)
            self.is_first_odom = False

        """

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
            print("rmse:", rmse)
            
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
        # pass
        # if self.init_pose_test:
        #     self.init_pose_test = False
            # self.received_init_pos = True
            # print(init)

        self.last_fixed_location_x = -init.pose.pose.position.y
        self.last_fixed_location_y = init.pose.pose.position.x
        self.last_fixed_location_t = self.quat_to_rad(init.pose.pose.orientation)#  + self.fixed_odometry_orientation
        
        # self.last_fixed_location_x = 1 # -init.pose.pose.position.y
        # self.last_fixed_location_y = 1 # init.pose.pose.position.x
        # self.last_fixed_location_t = 0

        self.robot_x_fused = self.last_fixed_location_x
        self.robot_y_fused = self.last_fixed_location_y
        self.robot_t_fused = self.last_fixed_location_t

        # self.xxx = self.last_fixed_location_x
        # self.yyy = self.last_fixed_location_y
        
        # self.rot_inc = 0
        
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

        # yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        # pitch = math.asin(-2.0*(q.x*q.z - q.w*q.y))
        # print(q)
        roll = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
        # print(yaw, pitch, roll)
        # print("roll:", roll)

        return roll
        
    def rad_to_quat(self, yaw):
        roll = 0
        pitch = 0
        # print(yaw)

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
