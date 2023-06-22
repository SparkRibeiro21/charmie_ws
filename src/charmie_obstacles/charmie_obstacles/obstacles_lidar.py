#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from charmie_interfaces.msg import ObstacleInfo, Obstacles

class ObstaclesLIDAR:

    def __init__(self):
        print("Executing Main Code")


class ObstaclesNode(Node):

    def __init__(self):
        super().__init__("Obstacles")
        self.get_logger().info("Initialised CHARMIE Obstacles Node")

        # Create Code Class Instance
        self.obs = ObstaclesLIDAR() 

        # Create PUBs/SUBs
        self.obstacles_publisher = self.create_publisher(Obstacles, "obs_lidar", 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "lidar_scan", self.lidar_callback , 10)

        # Create Timers
        self.create_timer(1, self.timer_callback)
        

    def lidar_callback(self, scan:LaserScan):
        readings = scan.ranges
        pass

    def timer_callback(self):
        print("Pubed Obstacles")
        a = Obstacles()

        b1 = ObstacleInfo()
        b2 = ObstacleInfo()
        b3 = ObstacleInfo()

        b1.alfa = 1.0
        b1.dist = 2.0
        b1.length_cm = 3.0
        b1.length_degrees = 4.0
        
        b2.alfa = 5.0
        b2.dist = 6.0
        b2.length_cm = 7.0
        b2.length_degrees = 8.0
        
        a.obstacles.append(b1)
        a.obstacles.append(b2)
        a.obstacles.append(b3)
        a.no_obstacles=len(a.obstacles)

        self.obstacles_publisher.publish(a)
        pass



def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    rclpy.spin(node)
    rclpy.shutdown()
