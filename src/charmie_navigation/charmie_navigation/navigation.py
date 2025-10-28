#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
from charmie_interfaces.msg import RadarData
from charmie_interfaces.srv import SetRGB, Trigger, GetMinRadarDistance
from charmie_interfaces.action import AdjustNavigationAngle

import time
import threading
import math

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

### PASSAR UM DOS ADJUSTS, PODE SER O DO ANGULO QUE E O MAIS SIMPLES PARA AQUI COMO ACAO SERVER E TESTAR

### Do adjusts structures in std_functions
### inspection nav here
### Do the same with the nav2 related stuff ???

class ROS2NavigationNode(Node):

    def __init__(self):
        super().__init__("ROS2CHARMIENavigation")
        self.get_logger().info("Initialised CHARMIE Navigation Node")

        # Reentrant group so action + subs can overlap
        self.cb_group = ReentrantCallbackGroup()

        # Lock
        self._lock = threading.Lock()
        self._goal_lock = threading.Lock()
        
        # Variables
        self._active_goal = None
        self.current_odom_pose = None
        self.current_odom_wheels_pose = None
        self.radar = RadarData()
        self.is_radar_initialized = False

        # Error codes for AdjustNavigationAngle.Result
        self.ERR_SUCCESS  = 0
        self.ERR_TIMEOUT  = 1
        self.ERR_CANCELED = 2
        self.ERR_NO_ODOM  = 3
        self.ERR_EXCEPTION= 4

        ### TOPICS ###

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscriptions (attach to the reentrant group)
        self.odom_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10, callback_group=self.cb_group)
        self.odom_wheels_subscriber = self.create_subscription(Odometry, "/wheel_encoders", self.odom_wheels_callback, 10, callback_group=self.cb_group)
        self.radar_data_subscriber = self.create_subscription(RadarData, "radar/data", self.radar_data_callback, 10, callback_group=self.cb_group)
        
        # Clients/Servers (optional)
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode")
        self.get_minimum_radar_distance_server = self.create_service(GetMinRadarDistance, "get_min_radar_distance", self.get_min_radar_dist,  callback_group=self.cb_group)
        
        # 3) Action server
        self.adjust_navigation_angle_server = ActionServer(
            self,
            AdjustNavigationAngle,
            "adjust_navigation_angle",
            goal_callback=self._adjust_nav_angle_goal_cb,
            cancel_callback=self._adjust_nav_angle_cancel_cb,
            execute_callback=self._adjust_nav_angle_execute_cb,
            callback_group=self.cb_group,
        )

    def odom_callback(self, msg: Odometry):
        with self._lock:
            self.current_odom_pose = msg.pose

    def odom_wheels_callback(self, msg: Odometry):
        with self._lock:
            self.current_odom_wheels_pose = msg.pose

    def radar_data_callback(self, radar: RadarData):
        with self._lock:
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
    

    def get_min_radar_dist(self, request, response):
        # Type of service received: 
        # float32 direction
        # float32 ang_obstacle_check
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.
        # float32 min_radar_distance_to_robot_edge  # minimum distance from radar to robot edge in meters

        response.success = True
        response.message = "Nav Trigger"
        response.min_radar_distance_to_robot_edge = 0.0
        return response


    def _adjust_nav_angle_goal_cb(self, goal: AdjustNavigationAngle.Goal):
        angle_deg       = float(goal.angle)
        max_ang_speed   = float(goal.max_angular_speed)
        tol_deg         = float(goal.tolerance)
        kp              = float(goal.kp)
        timeout_s       = float(goal.timeout)

        # Variable Validation
        if max_ang_speed <= 0.0:
            self.get_logger().warn("Reject: max_angular_speed must be > 0 (rad/s)")
            return GoalResponse.REJECT
        if kp <= 0.0:
            self.get_logger().warn("Reject: kp must be > 0")
            return GoalResponse.REJECT
        if tol_deg <= 0.0:
            self.get_logger().warn("Reject: tolerance must higher than 0 deg")
            return GoalResponse.REJECT
        if timeout_s < 0.0:
            self.get_logger().warn("Reject: timeout must be >= 0 seconds (0 = no timeout)")
            return GoalResponse.REJECT

        # Enforce single active goal
        with self._goal_lock:
            if self._active_goal is not None and self._active_goal.is_active:
                self.get_logger().warn("Reject: another AdjustNavigationAngle goal is active.")
                return GoalResponse.REJECT
        
        
        self.get_logger().info(
            f"Accept AdjustNavigationAngle: angle={angle_deg:.2f}°, max={max_ang_speed:.2f} rad/s, "
            f"tol={tol_deg:.2f}°, kp={kp:.2f}, wheel_odom={goal.use_wheel_odometry}, timeout={timeout_s:.2f}s"
        )
        return GoalResponse.ACCEPT
           
    def _adjust_nav_angle_cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested for AdjustNavigationAngle.")
        return CancelResponse.ACCEPT

    def _adjust_nav_angle_execute_cb(self, goal_handle):
        # track as active
        with self._goal_lock:
            self._active_goal = goal_handle

        fb = AdjustNavigationAngle.Feedback()
        res = AdjustNavigationAngle.Result()

        g = goal_handle.request
        angle_deg       = float(g.angle)
        max_ang_speed   = float(g.max_angular_speed)
        tol_deg         = float(g.tolerance)
        kp              = float(g.kp)
        use_wheel_odom  = bool(g.use_wheel_odometry)
        timeout_s       = float(g.timeout)

        start_time = self.get_clock().now()

        # closures for adjust_angle
        def should_stop_fn():
            # stop if: 
            # node is shutting down or goal is inactive
            if not rclpy.ok() or not goal_handle.is_active:
                return True
            # cancel is requested
            if goal_handle.is_cancel_requested:
                return True
            # timeout reached
            if timeout_s > 0.0:
                elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
                if elapsed >= timeout_s:
                    return True
            return False

        def feedback_fn(elapsed_s: float, remaining_deg: float):
            fb.navigation_time = Duration()
            fb.navigation_time.sec = int(elapsed_s)
            fb.navigation_time.nanosec = int((elapsed_s - int(elapsed_s)) * 1e9)
            fb.distance_remaining = float(remaining_deg)
            goal_handle.publish_feedback(fb)

        try:
            # call your loop with cooperative hooks
            err_code, msg = self.adjust_angle(
                angle=angle_deg,
                max_angular_speed=max_ang_speed,
                tolerance=tol_deg,
                kp=kp,
                use_wheel_odometry=use_wheel_odom,
                should_stop_fn=should_stop_fn,
                feedback_fn=feedback_fn,
                timeout_s=timeout_s,
            )

            # map outcome to action state/result
            if err_code == self.ERR_SUCCESS:
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.succeed()
                res.error_code = self.ERR_SUCCESS
            elif err_code == self.ERR_CANCELED:
                self._stop_robot()
                goal_handle.canceled()
                res.error_code = self.ERR_CANCELED
            elif err_code == self.ERR_TIMEOUT:
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_TIMEOUT
            elif err_code == self.ERR_NO_ODOM:
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_NO_ODOM
            else:  # ERR_EXCEPTION or unknown
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_EXCEPTION

            return res

        except Exception:
            self.get_logger().exception("Exception in _adjust_nav_angle_execute_cb")
            self._stop_robot()
            if goal_handle.is_active:
                goal_handle.abort()
            res.error_code = self.ERR_EXCEPTION
            return res

        finally:
            self._clear_active_goal(goal_handle)

    def adjust_angle(self, angle=0.0, max_angular_speed=0.25, tolerance=1.0, kp=1.3, use_wheel_odometry=False, \
                     should_stop_fn=None, feedback_fn=None, timeout_s=0.0):

        # normalize direction to be between -180 and 180
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360

        # Wait for odom (cooperatively)
        start_time_monotonic = time.monotonic()
        src = None
        while src is None:
            if should_stop_fn and should_stop_fn():
                return self.ERR_CANCELED, "Canceled before start"
            with self._lock:
                src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
            time.sleep(0.01)

        try:
            # Initial yaw
            with self._lock:
                src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
                if src is None:
                    return self.ERR_NO_ODOM, "No odometry"
                q = src.pose.orientation
            yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

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
                # cooperative cancel/timeout
                if should_stop_fn and should_stop_fn():
                    # self._stop_robot()
                    # disambiguate timeout if provided
                    if timeout_s > 0.0 and (time.monotonic() - start_time_monotonic) >= timeout_s:
                        return self.ERR_TIMEOUT, "Timeout"
                    else:
                        return self.ERR_CANCELED, "Canceled"

                with self._lock:
                    src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
                    if src is None:
                        # self._stop_robot()
                        return self.ERR_NO_ODOM, "Lost odometry"
                    q = src.pose.orientation
                curr_yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)
                # print("CURR YAW:", math.degrees(curr_yaw))

                # shortest error
                error_angle = target_angle - curr_yaw
                
                # error circular correction due to angle circularity
                if error_angle >= 2*math.pi:
                    error_angle -= 2*math.pi
                elif error_angle <= -2*math.pi:
                    error_angle += 2*math.pi


                # feedback: remaining angle in deg
                if feedback_fn:
                    elapsed = time.monotonic() - start_time_monotonic
                    remaining_deg = abs(math.degrees(error_angle))
                    feedback_fn(elapsed, remaining_deg)

                # DONE
                if abs(error_angle) < tolerance_rad:
                    break

                twist = Twist()
                twist.linear.x  = 0.0 
                twist.linear.y  = 0.0
                twist.angular.z = max(-max_angular_speed, min(max_angular_speed, error_angle * kp))

                self.cmd_vel_publisher.publish(twist)
                time.sleep(rate)

            # Stop the robot (send a few zeros)
            # self._stop_robot()
            self.get_logger().info("Angle Adjustment Complete.")
            return self.ERR_SUCCESS, "Angle Adjustment Complete."

        except Exception:
            self.get_logger().exception("Exception in adjust_angle")
            # self._stop_robot()
            return self.ERR_EXCEPTION, "Exception during angle adjustment"

    def _stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())
        time.sleep(0.05)
        self.cmd_vel_publisher.publish(Twist())
        time.sleep(0.05)
        self.cmd_vel_publisher.publish(Twist())

    def _clear_active_goal(self, goal_handle):
        with self._goal_lock:
            if self._active_goal is goal_handle:
                self._active_goal = None

    def get_yaw_from_quaternion(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis).
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians """
    
def main(args=None):
    rclpy.init(args=args)
    node = ROS2NavigationNode()
    executor = MultiThreadedExecutor(num_threads=3)  # 2 is enough; bump to 3–4 if needed later
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


""" # main function that already creates the thread for the navigation state machine
def main(args=None):
    rclpy.init(args=args)
    node = ROS2NavigationNode()
    # th_main = threading.Thread(target=ThreadMainNav, args=(node,), daemon=True)
    # th_main.start()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown() """

""" def ThreadMainNav(node: ROS2NavigationNode):
    main = NavigationMain(node)
    main.main()

class NavigationMain():

    def __init__(self, node: ROS2NavigationNode):

        self.node = node

    def main(self):
        while True:
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
        # Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis).
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians """