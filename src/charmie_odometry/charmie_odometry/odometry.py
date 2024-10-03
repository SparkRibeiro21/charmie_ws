#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from geometry_msgs.msg import Twist, TransformStamped, Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Encoders
from charmie_interfaces.srv import SetAcceleration, ActivateBool
import rclpy.time
import tf2_geometry_msgs

import tf2_py
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv2
import math
import numpy as np
import time
import tf2_ros

class RobotOdometry():
    def __init__(self):
        self.MAX_ENCODERS = 4294967295
        self.MOT_ENC = [0, 0, 0, 0]
        self.MOT_ENC_ANT = [0, 0, 0, 0]
        self.pulse_per_rotation = 980  # encoder pulses for a full wheel rotation
        self.wheel_diameter = 203  # mm
        self.robot_radius = 265  # this value is yet to be confimed by the 3D modulation of the robot !!!!!!!!!!
        
        self.DEBUG_DRAW_IMAGE = False # debug drawing opencv
        self.scale = 0.12*1000
        
        self.xc = 400
        self.yc = 400

        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)

        # debug dists
        self.robot_radius_d = 0.560/2
        self.lidar_radius_d = 0.050/2

        self.all_pos_x_val = []
        self.all_pos_y_val = []

        self.coord_rel_t = 0
        self.coord_rel_x_ = 0 
        self.coord_rel_y_ = 0 

        self.emergency_value = (100 << 24) + (100 << 16) + (100 << 8) + 100 # 1684300900
        self.emergency_flag = False
        self.emergency_ant_flag = False
        self.emergency_status = 0

    
    def localization(self, enc: Encoders):

        self.MOT_ENC[0] = enc.enc_m1
        self.MOT_ENC[1] = enc.enc_m2
        self.MOT_ENC[2] = enc.enc_m3
        self.MOT_ENC[3] = enc.enc_m4

        ### START OF EMERGENCY STOP CODE ###

        # print("---")
        # print(self.MOT_ENC)

        self.emergency_ant_flag = self.emergency_flag

        # ctr_ant = self.ctr
        self.ctr = 0
        for i in range(len(self.MOT_ENC)):
            if self.MOT_ENC[i] == self.emergency_value:
                self.ctr += 1
        if self.ctr == len(self.MOT_ENC):
            # print("EMERGENCY")
            self.emergency_flag = True
        else:
            self.emergency_flag = False

        self.emergency_status = self.emergency_flag*2 + self.emergency_ant_flag
        # print(self.emergency_status)
        # 0 LOW  -> LOW  = OFF
        # 1 LOW  -> HIGH = FALLING
        # 2 HIGH -> LOW  = RISING
        # 3 HIGH -> HIGH = ON
        
        # everytime it detects it is in emergency stop
        if self.emergency_status == 2 or self.emergency_status == 3:
            self.MOT_ENC = self.MOT_ENC_ANT

        # first frame emergency stops
        if self.emergency_status == 1:
            self.MOT_ENC_ANT = self.MOT_ENC

        # print(self.MOT_ENC, self.MOT_ENC_ANT)

        ### END OF EMERGENCY STOP CODE ###

        DIFF_MOT_ENC = [0, 0, 0, 0]

        # print(val_enc)

        for i in range(len(self.MOT_ENC)):
            DIFF_MOT_ENC[i] = self.MOT_ENC[i] - self.MOT_ENC_ANT[i]
            if DIFF_MOT_ENC[i] > self.MAX_ENCODERS/2:
                DIFF_MOT_ENC[i] -= self.MAX_ENCODERS + 1  # +1 because numerically the 0 to MAX_ENCODERS transition must be considered
            if DIFF_MOT_ENC[i] < -self.MAX_ENCODERS/2:
                DIFF_MOT_ENC[i] += self.MAX_ENCODERS + 1  # +1 because numerically the 0 to MAX_ENCODERS transition must be considered
            DIFF_MOT_ENC[i] *= -1

        # print("CURRENT:    ", self.MOT_ENC)
        # print("PREVIOUS:   ", self.MOT_ENC_ANT)
        # print("DIFFERENCE: ", DIFF_MOT_ENC)

        # acording to the Report of Inês Garcia and Tiago Ribeiro (P2 - 2017)
        # check report, named: C.H.A.R.M.I.E. - PLATAFORMA MÓVEL PARA ROBÔ DE SERVIÇOS EM CASA
        # the following equations calculate the odometry through encoders for a four wheeled omnidirectional platform

        # calculate the linear speed of each wheel

        # d = [0, 0, 0, 0]
        v = [0, 0, 0, 0]
        for i in range(len(self.MOT_ENC)):
            # in the report this variable is calculated, however it is not used for anything, thus is commented
            # d[i] = (((math.pi*self.wheel_diameter)/self.pulse_per_rotation)*self.MOT_ENC[i])
            v[i] = (((math.pi*self.wheel_diameter)/self.pulse_per_rotation)*DIFF_MOT_ENC[i])/0.05

        # print("v:", v)

        G_ = [0, 0]
        G = [0, 0]

        G_[0] = (v[1] + v[3]) / 2
        G_[1] = (v[0] + v[2]) / 2

        G[0] = ((math.sqrt(2) / 2) * (-G_[0] + G_[1]))  # + ((math.sqrt(2) / 2) * G_[1])
        G[1] = ((math.sqrt(2) / 2) * (G_[0] + G_[1]))  # + ((math.sqrt(2) / 2) * G_[1])

        # print("G_:", G_)
        # print("G:", G)

        vel_lin_enc = math.sqrt(G[0]**2 + G[1]**2)
        alfa_enc = math.pi/2 + math.pi/2 - math.atan2(G[1], -G[0])  # summed math.pi/2
        ### alfa_enc_deg = math.degrees(alfa_enc)
        # vel_ang_enc = (((G_[1] - v[0])+(G_[0] - v[3])) / 2) / self.robot_radius
        # vel_ang_enc = (((G_[1] - v[0]) + (G_[0] - v[1])) / 2) / self.robot_radius
        vel_ang_enc = (((G_[1] - v[0]) + (G_[0] - v[1]) + (-G_[1] + v[2]) + (-G_[0] + v[3])) / 4) / self.robot_radius

        # print(vel_ang_enc, vel_ang_enc2)

        # ##################### #
        #         90            #
        #     /---------\       #
        #     |         |       #
        # 180 |         | 360/0 #
        #     |         |       #
        #     \---------/       #
        #         270           #
        # ##################### #

        # print(round(alfa_enc_deg, 2))
        ### alfa_enc_deg = self.normalize_angles(alfa_enc_deg)

        # if alfa_enc_deg < 0:
        #     alfa_enc_deg += 360
        # print(vel_lin_enc, alfa_enc_deg, vel_ang_enc)
        # print(alfa_enc)

        ### xx = G[0]*0.05
        ### yy = G[1]*0.05
        ### theta = math.degrees(vel_ang_enc*0.05)
        theta = vel_ang_enc*0.05


        # print(xxx, yyy)

        ### self.coord_rel_x += xx
        ### self.coord_rel_y += yy
        self.coord_rel_t += theta

        ### self.coord_rel_t = self.normalize_angles(self.coord_rel_t)
        # tema que se normalizar o theta aqui!

        ### fi = alfa_enc_deg + self.coord_rel_t  # + self.coord_rel_t + (alfa_enc_deg)
        fi = alfa_enc + self.coord_rel_t ### + math.pi/2
        # print(math.degrees(fi), math.degrees(alfa_enc), math.degrees(self.coord_rel_t))

        ### fi = self.normalize_angles(fi)
        # while fi < 0:
        #     fi+=360
        # while fi >= 360:
        #     fi-=360

        # xx_ = xx * math.cos(math.radians(90 - self.coord_rel_t - alfa_enc_deg))
        # yy_ = yy * math.sin(math.radians(90 - self.coord_rel_t - alfa_enc_deg))
        ### xx_ = vel_lin_enc*0.05 * math.cos(math.radians(fi))
        ### yy_ = vel_lin_enc*0.05 * math.sin(math.radians(fi))
        xx_ = vel_lin_enc*0.05 * math.cos(fi)
        yy_ = vel_lin_enc*0.05 * math.sin(fi)


        #transforms from mm to m
        xx_ /= 1000
        yy_ /= 1000

        self.coord_rel_x_ += xx_
        self.coord_rel_y_ += yy_

        # print(round(xx_, 2), round(yy_, 2), round(fi, 2), end='\t')

        """        
        print("XX:\t", round(self.coord_rel_x / 10, 2), "\tYY:\t", round(self.coord_rel_y / 10, 2), end='\t')
        print("X_:\t", round(self.coord_rel_x_ / 10, 2), "\tY_:\t", round(self.coord_rel_y_ / 10, 2),
              "\tTHETA(deg/rad):\t", round(self.normalize_angles(math.degrees(self.coord_rel_t)), 2),
               "\t", round(self.coord_rel_t, 2), end='\t|\t')

        print("ALFA(deg/rad):\t", round(self.normalize_angles(math.degrees(alfa_enc)), 2),  "\t", round(alfa_enc, 2),
              "\tVEL_LIN:\t", round(vel_lin_enc/10, 2), "\tVEL_ANG:\t", round(vel_ang_enc, 2))
        """

        self.MOT_ENC_ANT = self.MOT_ENC.copy()


        # end = time.time()
        # print(end - start, end=' ')
        # time_to_50ms = 0.05 - (end - start)
        # print(time_to_50ms)

        if self.DEBUG_DRAW_IMAGE:
            # cv2.circle(self.test_image, (self.xc, self.yc), (int)(self.OBS_THRESH*self.scale), (0, 255, 255), 1)
            # cv2.line(self.test_image2, (60, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (800-60+1, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (0, 255, 255), 1)
            for i in range(10):
                cv2.line(self.test_image, (int(self.xc + self.scale*i), 0), (int(self.xc + self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (int(self.xc - self.scale*i), 0), (int(self.xc - self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc - self.scale*i)), (self.yc*2, int(self.yc - self.scale*i)), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc + self.scale*i)), (self.yc*2, int(self.yc + self.scale*i)), (255, 255, 255), 1)
            
            
            self.all_pos_x_val.append(self.coord_rel_x_)
            self.all_pos_y_val.append(self.coord_rel_y_)
            for i in range(len(self.all_pos_x_val)):
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[i]), int(self.yc - self.scale * self.all_pos_y_val[i])), 1, (255, 255, 0), -1)


            cv2.circle(self.test_image, (int(self.xc + self.scale*self.coord_rel_x_), int(self.yc - self.scale * self.coord_rel_y_)), (int)(self.scale*self.robot_radius_d), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.coord_rel_x_), int(self.yc - self.scale * self.coord_rel_y_)), (int)(self.scale*self.robot_radius_d/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.coord_rel_x_ + (self.robot_radius_d - self.lidar_radius_d)*self.scale*math.cos(self.coord_rel_t + math.pi/2)),
                                         int(self.yc - self.scale*self.coord_rel_y_ - (self.robot_radius_d - self.lidar_radius_d)*self.scale*math.sin(self.coord_rel_t + math.pi/2))), (int)(self.scale*self.lidar_radius_d), (0, 255, 255), -1)
            



            cv2.imshow("Odometry", self.test_image)
            
            k = cv2.waitKey(1)
            if k == ord('+'):
                self.scale /= 0.8
            if k == ord('-'):
                self.scale *= 0.8
            if k == ord('0'):
                self.all_pos_x_val.clear()
                self.all_pos_y_val.clear()

            self.test_image[:, :] = 0
        
        
        return self.coord_rel_x_, self.coord_rel_y_, self.coord_rel_t, G[0]/1000, G[1]/1000, vel_ang_enc
    

    def normalize_angles(self, ang):

        while ang < 0:
            ang += 360
        while ang >= 360:
            ang -= 360

        return ang
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
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


class OdometryNode(Node):

    def __init__(self):
        super().__init__("Odometry")
        self.get_logger().info("Initialised CHARMIE Odometry Node")

        ### TOPICS ###
        # Low level
        self.encoders_subscriber = self.create_subscription(Encoders, "get_encoders", self.get_encoders_callback, 10)
        self.odometry_publisher = self.create_publisher(Odometry, "odom", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Initial Pose
        self.initialpose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.get_initialpose_callback, 10)
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        # Localisation
        self.robot_localisation_publisher = self.create_publisher(Pose2D, "robot_localisation", 10)

        ### SERVICES ###
        # Low level
        self.activate_encoders_client = self.create_client(ActivateBool, "activate_encoders")

        while not self.activate_encoders_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Low Level...")

        self.robot_odom = RobotOdometry()
        self.robot_localisation = Pose2D()
        self.initialpose = Pose2D()
        self.is_map_odom_link = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tf_timer)

        self.tf_broadcaster_odom_base_link = tf2_ros.TransformBroadcaster(self)

        # time.sleep(0.100)
        
        request = ActivateBool.Request()
        request.activate = True
        self.activate_encoders_client.call_async(request)
    
        # self.create_timer(1.0, self.timer_callback)

        self.set_initial_position([0.0, 0.0, 0.0])


    def tf_timer(self):

        print(self.is_map_odom_link)
        # this was created to add initialpose the same way we add to when there is a map->base_link, because, since this connection does not exist
        # initial_pose_adjustement = Pose2D()

        # is there is a map tf, than we use the global localisation with odom and amcl 
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.is_map_odom_link = True
            # initial_pose_adjustement = 0, 0, 0
            
        except TransformException as ex:
            self.get_logger().warning(f'Could not transform {"map"} to {"base_link"}: {ex}')

            # else: we use the local localisation with just odom 
            try:
                transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
                        
            except TransformException as ex:
                self.get_logger().warning(f'Could not transform {"odom"} to {"base_link"}: {ex}')
                return

        position = transform.transform.translation
        orientation = transform.transform.rotation

        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        map_odom = Pose2D()
        map_odom.x = -position.y # - initial_pose_adjustement.y
        map_odom.y =  position.x # + initial_pose_adjustement.x
        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)
        map_odom.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) # + initial_pose_adjustement.theta
        
        self.robot_localisation.x = float(map_odom.x) 
        self.robot_localisation.y = float(map_odom.y)
        self.robot_localisation.theta = float(map_odom.theta)
        
        self.get_logger().info("Robot localisation: ({}, {}, {})".format(self.robot_localisation.x, self.robot_localisation.y, self.robot_localisation.theta))

        self.robot_localisation_publisher.publish(self.robot_localisation)

    def get_initialpose_callback(self, pose:PoseWithCovarianceStamped):
        
        self.initialpose.x = pose.pose.pose.position.x
        self.initialpose.y = pose.pose.pose.position.y
        
        qx = pose.pose.pose.orientation.x
        qy = pose.pose.pose.orientation.y
        qz = pose.pose.pose.orientation.z
        qw = pose.pose.pose.orientation.w

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)
        self.initialpose.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) 


        if self.is_map_odom_link == False:
            print("INSIDE")
            self.robot_odom.coord_rel_x_ = -self.initialpose.y
            self.robot_odom.coord_rel_y_ = self.initialpose.x
            self.robot_odom.coord_rel_t  = self.initialpose.theta


    def get_encoders_callback(self, enc: Encoders):
        coord_x, coord_y, coord_theta, vel_x, vel_y, vel_theta = self.robot_odom.localization(enc) 
        # print(coord_x, coord_y, coord_theta)

        quaternion = self.robot_odom.get_quaternion_from_euler(0,0,coord_theta)

        odom = Odometry()
        odom.pose.pose.position.x = coord_x # x corrdinates
        odom.pose.pose.position.y = coord_y # y corrdinates 
        
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3] 

        # print(odom.pose.pose.orientation)

        odom.twist.twist.linear.x = vel_x 
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.angular.z = vel_theta
        self.odometry_publisher.publish(odom)


        # print(coord_theta)

        quaternion_rviz = self.robot_odom.get_quaternion_from_euler(0,0,-coord_theta)
        odom_rviz = Odometry()
        odom_rviz.pose.pose.orientation.x = quaternion[0]
        odom_rviz.pose.pose.orientation.y = quaternion[1]
        odom_rviz.pose.pose.orientation.z = quaternion[2]
        odom_rviz.pose.pose.orientation.w = quaternion[3] 

        # creates a connection betweeen odom and base link, broadcast all joints of the tf transform,  
        transform = TransformStamped()
        # self.get_logger().info('Transform topic')
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odom.pose.pose.position.y
        transform.transform.translation.y = -odom.pose.pose.position.x
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom_rviz.pose.pose.orientation

        self.tf_broadcaster_odom_base_link.sendTransform(transform)
        
        twist = Twist()
        twist.angular.z = vel_theta
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.cmd_vel_publisher.publish(twist)
    
    def set_initial_position(self, initial_position):

        task_initialpose = PoseWithCovarianceStamped()

        task_initialpose.header.frame_id = "map"
        task_initialpose.header.stamp = self.get_clock().now().to_msg()

        task_initialpose.pose.pose.position.x = initial_position[1]
        task_initialpose.pose.pose.position.y = -initial_position[0]
        task_initialpose.pose.pose.position.z = 0.0

        # quaternion = self.get_quaternion_from_euler(0,0,math.radians(initial_position[2]))

        # Convert an Euler angle to a quaternion.
        # Input
        #     :param roll: The roll (rotation around x-axis) angle in radians.
        #     :param pitch: The pitch (rotation around y-axis) angle in radians.
        #     :param yaw: The yaw (rotation around z-axis) angle in radians.
        # 
        # Output
        #     :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format

        roll = 0.0
        pitch = 0.0
        yaw = math.radians(initial_position[2])

        task_initialpose.pose.pose.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        task_initialpose.pose.pose.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        task_initialpose.pose.pose.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        task_initialpose.pose.pose.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        self.initialpose_publisher.publish(task_initialpose)

    def normalize_angles(self, ang):

        while ang < 0:
            ang += 360
        while ang >= 360:
            ang -= 360

        return ang

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
