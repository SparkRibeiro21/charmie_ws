#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
import time

class TRNode(Node):

    def __init__(self):
        super().__init__("Debug")
        self.get_logger().info("Initialised CHARMIE Debug Node")
        self.neck_position_subscriber = self.create_subscription(Pose2D, "neck_pos", self.neck_position_callback , 10)
        self.neck_coordinates_publisher = self.create_publisher(Pose2D, "neck_coord", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.counter = 0

        self.create_timer(5, self.timer_callback)

    def neck_position_callback(self, pos: Pose2D):
        print("Received: pan =", int(pos.x), " tilt = ", int(pos.y))

    def timer_callback(self):
        cmd = Pose2D()

        cmd.x = 180.0
        cmd.y = 180.0 
        self.neck_coordinates_publisher.publish(cmd)
        time.sleep(2)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        cmd.x = 60.0
        cmd.y = 60.0 
        self.neck_error_publisher.publish(cmd)
        time.sleep(0.05)

        """
        if self.counter == 0:
            cmd.x = 180.0
            cmd.y = 180.0 
        if self.counter == 1:
            cmd.x = 300.0
            cmd.y = 180.0 
        if self.counter == 2:
            cmd.x = 180.0
            cmd.y = 180.0 
        if self.counter == 3:
            cmd.x = 60.0
            cmd.y = 180.0 
        if self.counter == 4:
            cmd.x = 180.0
            cmd.y = 180.0 
        if self.counter == 5:
            cmd.x = 180.0
            cmd.y = 120.0 
        if self.counter == 6:
            cmd.x = 180.0
            cmd.y = 180.0 
        if self.counter == 7:
            cmd.x = 180.0
            cmd.y = 270.0 
            self.counter = -1
        
        # k = cv2.waitKey(1)
        # if k == ord('a'):
        # if k == ord('s'):
        # cmd.x = 3000
        # cmd.y = 3000
        
        # cmd.x = 2000.0
        # cmd.y = 2000.0

        self.neck_coordinates_publisher.publish(cmd)
        self.counter+=1
        """

def main(args=None):
    rclpy.init(args=args)
    node = TRNode()
    rclpy.spin(node)
    rclpy.shutdown()
