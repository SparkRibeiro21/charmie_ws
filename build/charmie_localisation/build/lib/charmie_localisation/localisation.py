#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
# from charmie_interfaces.msg import Encoders

import numpy as np


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
        # self.task_start_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_pose_callback, 10)

        self.initial_pose_amcl_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.localisation_publisher = self.create_publisher(Odometry, "amcl_tr", 10)
        # publisher_auxiliar só para testes:
        self.amcl_alone_localisation_publisher = self.create_publisher(Odometry, "odom_a", 10)

        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)


        self.odom_robot = Odometry()
        self.conv_amcl = Odometry()
        self.amcl_loc = PoseWithCovarianceStamped()
        

    def odom_robot_callback(self, odom:Odometry):
        self.odom_robot = odom


    def amcl_pose_callback(self, amcl_p:PoseWithCovarianceStamped):
        self.amcl_loc = amcl_p
        covariance = amcl_p.pose.covariance
        std_deviations = np.sqrt(np.diag(covariance))
        # root mean square error
        rmse = np.sqrt(np.sum(np.diag(covariance)))
        print(std_deviations)
        print(rmse)

        self.conv_amcl = self.PoseWithCovarianceStamped_to_Odometry(amcl_p)
        self.amcl_alone_localisation_publisher.publish(self.conv_amcl)


    def PoseWithCovarianceStamped_to_Odometry(self, pose_c: PoseWithCovarianceStamped):
        od = Odometry()
                
        od.pose.pose.position.x = -pose_c.pose.pose.position.y
        od.pose.pose.position.y = pose_c.pose.pose.position.x
        od.pose.pose.orientation = pose_c.pose.pose.orientation
        
        return od


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    rclpy.spin(node)
    rclpy.shutdown()
