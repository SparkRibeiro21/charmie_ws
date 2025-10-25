#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, Float32, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, Vector3, Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import RadarData
from charmie_interfaces.srv import SetRGB

import time
import threading
import math

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class ROS2TaskNode(Node):

    def __init__(self, ros2_modules):
        super().__init__("ROS2TaskCHARMIE")
        self.get_logger().info("Initialised CHARMIE ROS2Task Node")

        ### TOPICS ###
        # Low level
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Odom
        self.odom_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.odom_wheels_subscriber = self.create_subscription(Odometry, "/wheel_encoders", self.odom_wheels_callback, 10)
        # Radar
        self.radar_dara_subscriber = self.create_subscription(RadarData, "radar/data", self.radar_data_callback, 10)
        
        
        
        
        
        # Localisation
        ###self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        ###self.amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.amcl_pose_callback, 10)
        ###self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        # Low level
        ###self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        
        ### Services (Clients) ###
        # Low level
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode") # just for low_level initialization checking 
        
        # Navigation
        ###self.nav_trigger_client = self.create_client(Trigger, "nav_trigger")
        # Low level
        ###self.internal_set_initial_position_define_north_client = self.create_client(SetPoseWithCovarianceStamped, "internal_initial_pose_for_north")
        ##self.activate_motors_client = self.create_client(ActivateBool, "activate_motors")
        
        ## CHECKS DE RADAR E LOW LEVEL ???? ###
        ## localisation

        while not self.set_rgb_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Low Level ...")


        ### INTERNAL SERVICES






        self.current_odom_pose = None
        self.current_odom_wheels_pose = None

        self.radar = RadarData()
        self.is_radar_initialized = False






        # Variables 
        ###self.flag_navigation_reached = False
        ###self.amcl_pose = PoseWithCovarianceStamped()
        ###self.new_amcl_pose_msg = False
        
        # robot localization
        ###self.robot_pose = Pose2D()
        ###self.gripper_point = Point()

        # Success and Message confirmations for all set_(something) CHARMIE functions
        ###self.navigation_success = True
        ###self.navigation_message = ""
        ###self.activate_motors_success = True
        ###self.activate_motors_message = ""
        
        ###self.orientation_yaw = 0.0
        ###self.cmd_vel = Twist()


    ###def robot_localisation_callback(self, pose: Pose2D):
    ###    self.robot_pose = pose

    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose

    def odom_wheels_callback(self, msg):
        self.current_odom_wheels_pose = msg.pose
    
    def radar_data_callback(self, radar: RadarData):
        self.radar = radar
        self.is_radar_initialized = True

        # DEBUG PRINTS
        # nos     = self.radar.number_of_sectors
        # sar     = self.radar.sector_ang_range
        # sectors = self.radar.sectors
        # print(f"Radar Data: Number of Sectors: {nos}, Sector Angle Range (deg): {round(math.degrees(sar),1)}")
        # i = 0
        # for s in sectors:
        #     sa = s.start_angle
        #     ea = s.end_angle
        #     md = s.min_distance
        #     p  = s.point
        #     hp = s.has_point
        #     print(f"Sector {i}: Start Angle: {round(math.degrees(sa),1)}, End Angle: {round(math.degrees(ea),1)}, Min Distance: {round(md,2)}, Has Point: {hp}, Point: ({round(p.x,2)}, {round(p.y,2)}, {round(p.z,2)})")
        #     i += 1    


    ### SERVICES ###



# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ROS2TaskNode()
    th_main = threading.Thread(target=ThreadMainTask, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainTask(node: ROS2TaskNode):
    main = TaskMain(node)
    main.main()

class TaskMain():

    def __init__(self, node: ROS2TaskNode):

        self.node = node

    def main(self):
        pass

    def adjust_omnidirectional_position(self, dx, dy, ang_obstacle_check=45, safety=True, max_speed=0.05, tolerance=0.01, kp=1.5, enter_house_special_case=False, use_wheel_odometry=False, wait_for_end_of=True):

        ### FOR NOW WE ARE USING THER MERGED ODOMETRY WITH ALL THE SENSORS, 
        ### BUT IN THE FUTURE WE MAY WANT TO USE JUST THE WHEEL ODOMETRY INSTEAD
        ### ALL THE CODE IS READY. JUST REPLACE:
        ### self.node.current_odom_pose -> self.node.current_odom_wheels_pose
        ### THE FUNCTION PARAMETER use_wheel_odometry SHOULD BE USED
        ### if use_wheel_odometry:
        ###     USE: self.node.current_odom_wheels_pose
        ### else:
        ###     USE: self.node.current_odom_pose

        success = False
        message = ""

        if safety:
            SAFETY_DISTANCE_FROM_ROBOT_EDGE = 0.02 # from robot edge to obstacle

            s, m, min_radar_distance_to_robot_edge = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=ang_obstacle_check)
            if not s:
                success = False
                message = m
                return success, message
            elif min_radar_distance_to_robot_edge - SAFETY_DISTANCE_FROM_ROBOT_EDGE < dx and dx > 0:
                success = False
                message = "Not enough space in front of the robot to perform the adjustment"
                self.node.get_logger().warn("Not enough space in front of the robot to perform the adjustment")
                print("Wanted to move forward:", round(dx,2), "meters. But minimum distance to obstacle in front of robot is:", round(min_radar_distance_to_robot_edge- SAFETY_DISTANCE_FROM_ROBOT_EDGE,2), "meters.")
                return success, message
            
            print("GOOD! Want to move forward:", round(dx,2), "meters. Minimum distance to obstacle in front is:", round(min_radar_distance_to_robot_edge- SAFETY_DISTANCE_FROM_ROBOT_EDGE,2), "meters.")
        
        # Wait until odom is received
        while self.node.current_odom_pose is None:
            self.node.get_logger().warning("Waiting for odom pose...") 
            time.sleep(0.01)

        # Initial pose and orientation
        pose = self.node.current_odom_pose.pose
        start_x = pose.position.x
        start_y = pose.position.y
        q = pose.orientation
        yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

        print("Adjusting Omnidirectional Position:", round(dx,2), round(dy,2), "meters")

        # Update the robot's distance to match the tolerance, this way the robot will actually aim for the correct spot
        # fixed bug where dx = 0 or dy = 0 still moved the robot
        if dx > 0:
            dx = dx + tolerance
        elif dx < 0:
            dx = dx - tolerance
        
        if dy > 0:
            dy = dy + tolerance
        elif dy < 0:
            dy = dy - tolerance

        # Compute target in odom frame
        target_x = start_x + math.cos(yaw) * dx - math.sin(yaw) * dy
        target_y = start_y + math.sin(yaw) * dx + math.cos(yaw) * dy
        # print("TARGETS", target_x, target_y)

        rate_hz = 20  # Hz
        rate = 1.0 / rate_hz

        while True:
            pose = self.node.current_odom_pose.pose
            curr_x = pose.position.x
            curr_y = pose.position.y

            error_x = target_x - curr_x
            error_y = target_y - curr_y
            dist_error = math.sqrt(error_x**2 + error_y**2)
            # print("ERRORS", error_x, error_y, dist_error)
            # print("TOLERANCE", tolerance)

            if dist_error < tolerance:
                break

            # Convert error from odom frame to base_footprint frame
            vx = math.cos(-yaw) * error_x - math.sin(-yaw) * error_y
            vy = math.sin(-yaw) * error_x + math.cos(-yaw) * error_y
            # print("X Speed", max(-max_speed, min(max_speed, vx * kp)))
            # print("Y Speed", max(-max_speed, min(max_speed, vy * kp)))

            twist = Twist()
            twist.linear.x = max(-max_speed, min(max_speed, vx * kp))
            twist.linear.y = max(-max_speed, min(max_speed, vy * kp))
            twist.angular.z = 0.0

            self.node.cmd_vel_publisher.publish(twist)
            time.sleep(rate)

        if not enter_house_special_case:
            # Stop the robot
            # Dirty, but had to do this way because of some commands to low_level being lost
            self.node.cmd_vel_publisher.publish(Twist())
            time.sleep(0.1)  # wait for the cmd_vel to be published
            self.node.cmd_vel_publisher.publish(Twist())
            time.sleep(0.1)  # wait for the cmd_vel to be published
            self.node.cmd_vel_publisher.publish(Twist())  

        self.node.get_logger().info("Omnidirectional Adjustment Complete.")

        success = True
        message = "Omnidirectional Adjustment Complete."
        return success, message

    def adjust_obstacles(self, distance=0.0, direction=0.0, ang_obstacle_check=45, max_speed=0.05, tolerance=0.01, kp=1.5, wait_for_end_of=True):

        success = False
        message = ""

        distance_to_adjust = 0.0

        # normalize direction to be between -180 and 180 (how radar handles angles)
        while direction > 180:
            direction -= 360
        while direction < -180:
            direction += 360

        s, m, min_radar_distance_to_robot_edge = self.get_minimum_radar_distance(direction=direction, ang_obstacle_check=ang_obstacle_check)

        if not s:
            success = False
            message = m
            return success, message
        else:
            distance_to_adjust = min_radar_distance_to_robot_edge - distance
            print("DISTANCE TO ADJUST (positive means move forward):", round(distance_to_adjust,2))

            # Copilot suggestion
            dx = distance_to_adjust * math.cos(math.radians(direction))
            dy = distance_to_adjust * math.sin(math.radians(direction))
            
            self.adjust_omnidirectional_position(dx=dx, dy=dy, max_speed=max_speed, safety=False, tolerance=tolerance, kp=kp)

            success = True
            message = "Obstacle Adjustment Complete."
            return success, message
    
    def adjust_angle(self, angle=0.0, max_angular_speed=0.25, tolerance=1, kp=1.3, use_wheel_odometry=False, wait_for_end_of=True):

        ### FOR NOW WE ARE USING THER MERGED ODOMETRY WITH ALL THE SENSORS, 
        ### BUT IN THE FUTURE WE MAY WANT TO USE JUST THE WHEEL ODOMETRY INSTEAD
        ### ALL THE CODE IS READY. JUST REPLACE:
        ### self.node.current_odom_pose -> self.node.current_odom_wheels_pose
        ### THE FUNCTION PARAMETER use_wheel_odometry SHOULD BE USED
        ### if use_wheel_odometry:
        ###     USE: self.node.current_odom_wheels_pose
        ### else:
        ###     USE: self.node.current_odom_pose

        success = False
        message = ""

        # normalize direction to be between -180 and 180
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360

        # Wait until odom is received
        while self.node.current_odom_pose is None:
            self.node.get_logger().warning("Waiting for odom pose...") 
            time.sleep(0.01)

        # Initial pose and orientation
        pose = self.node.current_odom_pose.pose
        q = pose.orientation
        yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # print("Adjusting Angle Position:", round(angle,2), "degrees")

        # Update the robot's distance to match the tolerance, this way the robot will actually aim for the correct spot
        # fixed bug where dx = 0 or dy = 0 still moved the robot
        if angle > 0:
            angle = angle + tolerance
        elif angle < 0:
            angle = angle - tolerance

        # Compute target in odom frame
        target_angle = yaw + math.radians(angle)
        tolerance_rad = math.radians(tolerance)
        # print("TARGETS", math.degrees(target_angle))

        rate_hz = 20  # Hz
        rate = 1.0 / rate_hz

        while True:
            pose = self.node.current_odom_pose.pose
            q = pose.orientation
            curr_yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)
            # print("CURR YAW:", math.degrees(curr_yaw))

            error_angle = target_angle - curr_yaw
            # error circular correction due to angle circularity
            if error_angle >= 2*math.pi:
                error_angle -= 2*math.pi
            elif error_angle <= -2*math.pi:
                error_angle += 2*math.pi

            # print("ERROR:", math.degrees(error_angle))
            # print("TOLERANCE", math.degrees(tolerance_rad))

            # print("target", math.degrees(target_angle), "curr_yaw", math.degrees(curr_yaw), "error", math.degrees(error_angle), "tol", math.degrees(tolerance_rad))

            if abs(error_angle) < tolerance_rad:
                break

            twist = Twist()
            twist.linear.x  = 0.0 
            twist.linear.y  = 0.0
            twist.angular.z = max(-max_angular_speed, min(max_angular_speed, error_angle * kp))

            self.node.cmd_vel_publisher.publish(twist)
            time.sleep(rate)

        # Stop the robot
        # Dirty, but had to do this way because of some commands to low_level being lost
        self.node.cmd_vel_publisher.publish(Twist())
        time.sleep(0.1)  # wait for the cmd_vel to be published
        self.node.cmd_vel_publisher.publish(Twist())
        time.sleep(0.1)  # wait for the cmd_vel to be published
        self.node.cmd_vel_publisher.publish(Twist())  
        self.node.get_logger().info("Angle Adjustment Complete.")

        success = True
        message = "Angle Adjustment Complete."
        return success, message

    def get_minimum_radar_distance(self, direction=0.0, ang_obstacle_check=45):

        success = False
        message = ""
        min_radar_distance_to_robot_edge = None

        if self.node.is_radar_initialized:
            if -100 <= direction <= 100 and 0 < ang_obstacle_check <= 360:
                radar = self.node.radar
                used_sectors = []
                min_distance = None

                # DEBUG PRINTS
                # nos     = radar.number_of_sectors
                # sar     = radar.sector_ang_range
                # sectors = radar.sectors
                # print(f"Radar Data: Number of Sectors: {nos}, Sector Angle Range (deg): {round(math.degrees(sar),1)}")
                # i = 0
                # for s in sectors:
                #     sa = s.start_angle
                #     ea = s.end_angle
                #     md = s.min_distance
                #     p  = s.point
                #     hp = s.has_point
                #     print(f"Sector {i}: Start Angle: {round(math.degrees(sa),1)}, End Angle: {round(math.degrees(ea),1)}, Min Distance: {round(md,2)}, Has Point: {hp}")
                #     i += 1    

                # print("USED SECTORS:")
                for s in radar.sectors:
                    if  -math.radians(ang_obstacle_check/2) <= s.start_angle - math.radians(direction) <= math.radians(ang_obstacle_check/2) and \
                        -math.radians(ang_obstacle_check/2) <= s.end_angle   - math.radians(direction) <= math.radians(ang_obstacle_check/2):
                        used_sectors.append(s)
                        # print(f"Start Angle: {round(math.degrees(s.start_angle),1)}, End Angle: {round(math.degrees(s.end_angle),1)}, Min Distance: {round(s.min_distance,2)}, Has Point: {s.has_point}")
            
                        if s.has_point:
                            if min_distance is None:
                                min_distance = s.min_distance
                            elif s.min_distance < min_distance:
                                min_distance = s.min_distance
                
                if min_distance is not None:
                    # print("MIN DISTANCE IN USED SECTORS:", round(min_distance,2))
                    min_radar_distance_to_robot_edge = min_distance - self.ROBOT_RADIUS
                    # print("MIN DISTANCE TO ROBOT EDGE IN USED SECTORS:", round(min_radar_distance_to_robot_edge,2))
                    success = True
                    message = ""
                    return success, message, min_radar_distance_to_robot_edge
                else:
                    success = False
                    message = "No obstacles detected in the selected direction"
                    self.node.get_logger().warn("No obstacles detected in the selected direction")
                    return success, message, min_radar_distance_to_robot_edge
            else:
                success = False
                message = "Wrong parameter definition"
                self.node.get_logger().warn("Wrong parameter definition")
                return success, message, min_radar_distance_to_robot_edge
        else:
            success = False
            message = "Radar not initialized"
            self.node.get_logger().warn("Radar not initialized")
            return success, message, min_radar_distance_to_robot_edge
        

    def get_yaw_from_quaternion(self, x, y, z, w):
        """ Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis). """
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians