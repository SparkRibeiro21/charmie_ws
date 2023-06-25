#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from charmie_interfaces.msg import TarNavSDNL, Obstacles

class NavigationSDNLClass:

    def __init__(self):

        self.lambda_target = 10
        self.beta1 = 300
        self.beta2 = 400

        self.obstacles = Obstacles()
        self.position = Odometry()
    
    def update_debug_drawings(self):
        pass


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
        pass

    def odom_robot_callback(self, obs: Odometry):
        # updates the position variable
        pass

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
