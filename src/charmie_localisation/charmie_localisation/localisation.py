#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Pose2D

import numpy as np
import math


class LocalisationNode(Node):

    def __init__(self):
        super().__init__("Localisation")
        self.get_logger().info("Initialised CHARMIE Localisation Node")

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the robot pose
        self.pose_publisher = self.create_publisher(Pose2D, "robot_localisation", 10)

        # Timer to update the pose at 10Hz
        self.timer = self.create_timer(0.1, self.publish_robot_pose)


    def publish_robot_pose(self):
        try:
            # Get robot pose in map frame
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract orientation (convert quaternion to yaw)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = self.get_yaw_from_quaternion(qx, qy, qz, qw)

            # Publish Pose2D message
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = yaw  # In radians
            self.pose_publisher.publish(pose_msg)

            self.get_logger().info(f"Published Robot Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°")

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {str(e)}")

    def get_yaw_from_quaternion(self, x, y, z, w):
        """ Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis). """
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    rclpy.spin(node)
    rclpy.shutdown()
