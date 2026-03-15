#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
import math


class LocalisationNode(Node):

    def __init__(self):
        super().__init__("localisation")
        self.get_logger().info("Initialised CHARMIE Localisation Node")

        # -----------------------
        # Parameters
        # -----------------------
        # Feature flags
        self.declare_parameter('publish_pose', True)
        self.declare_parameter('publish_gripper_map', True)
        self.declare_parameter('publish_gripper_base', True)

        self.publish_pose_enabled = bool(self.get_parameter('publish_pose').value)
        self.publish_gripper_map_enabled = bool(self.get_parameter('publish_gripper_map').value)
        self.publish_gripper_base_enabled = bool(self.get_parameter('publish_gripper_base').value)

        # -----------------------
        # TF Buffers / Listeners
        # -----------------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_buffer_gripper = Buffer(cache_time=Duration(seconds=1.0))
        self.tf_listener_gripper = TransformListener(self.tf_buffer_gripper, self)

        self.tf_buffer_base_gripper = Buffer(cache_time=Duration(seconds=1.0))
        self.tf_listener_base_gripper = TransformListener(self.tf_buffer_base_gripper, self)

        # -----------------------
        # Publishers + Timers (only if enabled)
        # -----------------------
        self.pose_publisher = None
        self.point_publisher = None
        self.point_base_publisher = None

        self.timer_pose = None
        self.timer_gripper_map = None
        self.timer_gripper_base = None

        if self.publish_pose_enabled:
            self.pose_publisher = self.create_publisher(Pose2D, "robot_localisation", 10)
            self.timer_pose = self.create_timer(0.05, self.publish_robot_pose)

        if self.publish_gripper_map_enabled:
            self.point_publisher = self.create_publisher(Point, "robot_gripper_localisation", 10)
            self.timer_gripper_map = self.create_timer(0.05, self.publish_robot_gripper)

        if self.publish_gripper_base_enabled:
            self.point_base_publisher = self.create_publisher(Point, "robot_base_gripper_localisation", 10)
            self.timer_gripper_base = self.create_timer(0.05, self.publish_robot_base_gripper)

        self.get_logger().info(
            f"publish_pose={self.publish_pose_enabled}, "
            f"publish_gripper_map={self.publish_gripper_map_enabled}, "
            f"publish_gripper_base={self.publish_gripper_base_enabled}"
        )

    # -----------------------
    # Pose2D from TF: map -> base_link
    # -----------------------
    def publish_robot_pose(self):
        try:

            requested_time = rclpy.time.Time()

            if self.tf_buffer.can_transform("map", "base_link", requested_time):
                # Get robot pose in map frame
                transform = self.tf_buffer.lookup_transform("map", "base_link", requested_time)
                
                # Check if the transform is recent (less than 0.5s old)
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                now = self.get_clock().now()
                age = now - tf_time

                if age.nanoseconds > 500_000_000:  # 0.5 seconds
                    print(f"⚠️ Transform map->base_link too old: {age.nanoseconds/1e9:.2f}s")
                    self.get_logger().warn(f"⚠️ Transform map->base_link too old: {age.nanoseconds/1e9:.2f}s")
                    return  # Skip publishing
                    
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

                # print(f"Published Robot Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°")
            
            else:
                print("Could not get transform: map -> base_link")
                self.get_logger().debug("Could not get transform: map -> base_link")

        except Exception as e:
            print(f"Pose TF error: {str(e)}")
            self.get_logger().warn(f"Pose TF error: {str(e)}")

    # -----------------------
    # Point from TF: map -> xarm_link6
    # -----------------------
    def publish_robot_gripper(self):
        try:

            requested_time = rclpy.time.Time()

            if self.tf_buffer_gripper.can_transform("map", "xarm_link6", requested_time):
                # Get robot point in map frame
                transform = self.tf_buffer_gripper.lookup_transform("map", "xarm_link6", requested_time)
                
                # Check if the transform is recent (less than 0.5s old)
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                now = self.get_clock().now()
                age = now - tf_time

                if age.nanoseconds > 500_000_000:  # 0.5 seconds
                    print(f"⚠️ Transform map->xarm_link6 too old: {age.nanoseconds/1e9:.2f}s")
                    self.get_logger().warn(f"⚠️ Transform map->xarm_link6 too old: {age.nanoseconds/1e9:.2f}s")
                    return  # Skip publishing

                # Extract position
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                # Publish Point message
                point_msg = Point()
                point_msg.x = x
                point_msg.y = y
                point_msg.z = z
                self.point_publisher.publish(point_msg)

                # print(f"Published Gripper Point: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            
            else:
                print("Could not get transform: map -> xarm_link6")
                self.get_logger().debug("Could not get transform: map -> xarm_link6")

        except Exception as e:
            print(f"Gripper(map) TF error: {str(e)}")
            self.get_logger().warn(f"Gripper(map) TF error: {str(e)}")

    # -----------------------
    # Point from TF: base_link -> xarm_link6
    # -----------------------
    def publish_robot_base_gripper(self):
        try:
            requested_time = rclpy.time.Time()
            if self.tf_buffer_base_gripper.can_transform("base_link", "xarm_link6", requested_time):
                # Get robot point in map frame
                transform = self.tf_buffer_base_gripper.lookup_transform("base_link", "xarm_link6", requested_time)
                
                # Check if the transform is recent (less than 0.5s old)
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                now = self.get_clock().now()
                age = now - tf_time
                if age.nanoseconds > 500_000_000:  # 0.5 seconds
                    print(f"⚠️ Transform base_link->xarm_link6 too old: {age.nanoseconds/1e9:.2f}s")
                    self.get_logger().warn(f"⚠️ Transform base_link->xarm_link6 too old: {age.nanoseconds/1e9:.2f}s")
                    return  # Skip publishing
                
                # Extract position
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                # Publish Point message
                point_base_msg = Point()
                point_base_msg.x = x
                point_base_msg.y = y
                point_base_msg.z = z
                self.point_base_publisher.publish(point_base_msg)

                # print(f"Published Gripper to Base Point: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            
            else:
                print("Could not get transform: base_link -> xarm_link6")
                self.get_logger().debug("Could not get transform: base_link -> xarm_link6")

        except Exception as e:
            print(f"Gripper(base) TF error: {str(e)}")
            self.get_logger().warn(f"Gripper(base) TF error: {str(e)}")

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
