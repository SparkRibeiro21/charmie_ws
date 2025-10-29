#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle

from geometry_msgs.msg import Twist
from realsense2_camera_msgs.msg import RGBD
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from charmie_interfaces.msg import RadarData
from charmie_interfaces.srv import SetRGB, Trigger, GetMinRadarDistance
from charmie_interfaces.action import AdjustNavigationAngle, AdjustNavigationObstacles, AdjustNavigationOmnidirectional

import time
import threading
import math

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

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

        self.goal_handle_ = None                 # active Nav2 ClientGoalHandle (or None)
        self._active_charmie_goal = None         # active ServerGoalHandle for our wrapper
        self.nav2_feedback = None
        self.nav2_goal_accepted = False
        self.nav2_status = GoalStatus.STATUS_UNKNOWN

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
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10, callback_group=self.cb_group)
        
        # Clients/Servers (optional)
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode", callback_group=self.cb_group)
        self.get_minimum_radar_distance_client = self.create_client(GetMinRadarDistance, "get_min_radar_distance", callback_group=self.cb_group)
        #self.get_minimum_radar_distance_server = self.create_service(GetMinRadarDistance, "get_min_radar_distance", self.get_min_radar_dist,  callback_group=self.cb_group)
                # Nav2 costmap clear clients
        self.clear_entire_local_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap", callback_group=self.cb_group
        )
        self.clear_entire_global_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap", callback_group=self.cb_group
        )

        self.nav2_client_ = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav2_client_follow_waypoints_ = ActionClient(self, FollowWaypoints, "follow_waypoints")

        while not self.set_rgb_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Low Level ...")
        while not self.get_minimum_radar_distance_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Radar ...")
        while not self.clear_entire_local_costmap_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /local_costmap/clear_entirely_local_costmap ...")
        while not self.clear_entire_global_costmap_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /global_costmap/clear_entirely_global_costmap ...")

        # Service to clear both nav2 costmaps
        self.clear_nav_costmaps_server = self.create_service(Trigger, "clear_nav_costmaps", self._clear_nav_costmaps_cb, callback_group=self.cb_group)

        # 3) Action servers
        self.adjust_navigation_angle_server = ActionServer(
            self,
            AdjustNavigationAngle,
            "adjust_navigation_angle",
            goal_callback=self._adjust_nav_angle_goal_cb,
            cancel_callback=self._adjust_nav_angle_cancel_cb,
            execute_callback=self._adjust_nav_angle_execute_cb,
            callback_group=self.cb_group,
        )

        self.adjust_navigation_omni_server = ActionServer(
            self,
            AdjustNavigationOmnidirectional,
            "adjust_navigation_omni",
            goal_callback=self._adjust_nav_omni_goal_cb,
            cancel_callback=self._adjust_nav_omni_cancel_cb,
            execute_callback=self._adjust_nav_omni_execute_cb,
            callback_group=self.cb_group,
        )

        self.adjust_navigation_obstacles_server = ActionServer(
            self,
            AdjustNavigationObstacles,
            "adjust_navigation_obstacle",
            goal_callback=self._adjust_nav_obst_goal_cb,
            cancel_callback=self._adjust_nav_obst_cancel_cb,
            execute_callback=self._adjust_nav_obst_execute_cb,
            callback_group=self.cb_group,
        )

        self.charmie_nav_server = ActionServer(
            self,
            NavigateToPose,
            "charmie_navigate_to_pose",
            goal_callback=self._charmie_nav_goal_cb,
            cancel_callback=self._charmie_nav_cancel_cb,
            execute_callback=self._charmie_nav_execute_cb,
            callback_group=self.cb_group,
        )

        # self.timer = self.create_timer(5.0, self.timer_callback)

    # def timer_callback(self):
    #     ok, msg, d = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=45.0, timeout_s=2.0)
    #     self.get_logger().info(f"RadarMinEdge ok={ok} msg='{msg}' dist={d}")

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

    def get_rgbd_head_callback(self, rgbd: RGBD):
        self.rgb_head_img = rgbd.rgb
        self.first_rgb_head_image_received = True
        self.depth_head_img = rgbd.depth
        self.first_depth_head_image_received = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    """ 
    def get_head_depth_image(self):

        if self.node.first_depth_head_image_received:
            current_frame_depth_head = self.node.br.imgmsg_to_cv2(self.node.depth_head_img, desired_encoding="passthrough")
        else:
            current_frame_depth_head = np.zeros((self.node.CAM_IMAGE_HEIGHT, self.node.CAM_IMAGE_WIDTH), dtype=np.uint8)
        
        return self.node.first_depth_head_image_received, current_frame_depth_head
  """

    ### SERVICES ###
    
    def set_rgb(self, command=0, wait_for_end_of=True):
        request = SetRGB.Request()
        request.colour = int(command)
        self.set_rgb_client.call_async(request)
        return True, ""

    """ def get_min_radar_dist(self, request, response):
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
 """
    
    def _clear_nav_costmaps_cb(self, request, response):
        try:
            self.clear_costmaps()
            self.get_logger().info("Sent costmap clear requests (local + global).")
            response.success = True
            response.message = "Clear requests sent (local + global)."
        except Exception as e:
            self.get_logger().error(f"Failed to send costmap clear requests: {e}")
            response.success = False
            response.message = f"Failed to send clear requests: {e}"
        return response
    
    def clear_costmaps(self):
        self.clear_entire_local_costmap_client.call_async(ClearEntireCostmap.Request())
        self.clear_entire_global_costmap_client.call_async(ClearEntireCostmap.Request())
    
    def get_minimum_radar_distance(self, direction: float = 0.0, ang_obstacle_check: float = 45.0, timeout_s: float = 1.0):
        
        # quick availability check (non-blocking if already up)
        if not self.get_minimum_radar_distance_client.wait_for_service(timeout_sec=0.0):
            msg = "get_min_radar_distance service not available"
            self.get_logger().warn(msg)
            return False, msg, None

        # build request
        req = GetMinRadarDistance.Request()
        req.direction = float(direction)
        req.ang_obstacle_check = float(ang_obstacle_check)

        # async call + cooperative wait with timeout
        future = self.get_minimum_radar_distance_client.call_async(req)
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if timeout_s > 0.0 and (time.monotonic() - start) >= timeout_s:
                self.get_logger().warn("get_min_radar_distance timed out")
                return False, "Timeout", None
            time.sleep(0.01)

        if not future.done():
            # node shutting down, etc.
            return False, "Canceled", None

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"get_min_radar_distance call failed: {e}")
            return False, "Exception", None

        if not resp.success:
            # Service returned a logical failure (e.g., no obstacles)
            return False, resp.message, None

        return True, resp.message, float(resp.min_radar_distance_to_robot_edge)

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
                self.set_rgb(GREEN+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.succeed()
                res.error_code = self.ERR_SUCCESS
            elif err_code == self.ERR_CANCELED:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                goal_handle.canceled()
                res.error_code = self.ERR_CANCELED
            elif err_code == self.ERR_TIMEOUT:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_TIMEOUT
            elif err_code == self.ERR_NO_ODOM:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_NO_ODOM
            else:  # ERR_EXCEPTION or unknown
                self.set_rgb(RED+BACK_AND_FORTH_8)
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
        
        self.set_rgb(BLUE+BACK_AND_FORTH_8)

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

            self.set_rgb(CYAN+BACK_AND_FORTH_8)

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
        




    def _adjust_nav_omni_goal_cb(self, goal: AdjustNavigationOmnidirectional.Goal):
        dx                 = float(goal.dx)
        dy                 = float(goal.dy)
        ang_obstacle_check = float(goal.ang_obstacle_check)
        safety             = bool(goal.safety)
        max_speed          = float(goal.max_speed)
        tolerance          = float(goal.tolerance)
        kp                 = float(goal.kp)
        enter_special      = bool(goal.enter_house_special_case)
        use_wheel_odom     = bool(goal.use_wheel_odometry)
        timeout_s          = float(goal.timeout)

        # Validation
        if not (0.0 < ang_obstacle_check <= 360.0):
            self.get_logger().warn("Reject: ang_obstacle_check must be in (0, 360]")
            return GoalResponse.REJECT
        if max_speed <= 0.0:
            self.get_logger().warn("Reject: max_speed must be > 0 (m/s)")
            return GoalResponse.REJECT
        if tolerance <= 0.0:
            self.get_logger().warn("Reject: tolerance must be > 0 (m)")
            return GoalResponse.REJECT
        if kp <= 0.0:
            self.get_logger().warn("Reject: kp must be > 0")
            return GoalResponse.REJECT
        if timeout_s < 0.0:
            self.get_logger().warn("Reject: timeout must be >= 0 seconds (0 = no timeout)")
            return GoalResponse.REJECT
        # You may allow dx=dy=0 (no-op), or reject. Keeping it allowed is fine.

        # Single-active-goal policy (shared with other actions)
        with self._goal_lock:
            if self._active_goal is not None and self._active_goal.is_active:
                self.get_logger().warn("Reject: another navigation goal is active.")
                return GoalResponse.REJECT

        self.get_logger().info(
            f"Accept AdjustNavigationOmnidirectional: dx={dx:.3f} m, dy={dy:.3f} m, "
            f"win={ang_obstacle_check:.1f}°, safety={safety}, vmax={max_speed:.2f} m/s, "
            f"tol={tolerance:.3f} m, kp={kp:.2f}, house={enter_special}, "
            f"wheel_odom={use_wheel_odom}, timeout={timeout_s:.2f}s"
        )
        return GoalResponse.ACCEPT


    def _adjust_nav_omni_cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested for AdjustNavigationOmnidirectional.")
        return CancelResponse.ACCEPT


    def _adjust_nav_omni_execute_cb(self, goal_handle):
        with self._goal_lock:
            self._active_goal = goal_handle

        fb = AdjustNavigationOmnidirectional.Feedback()
        res = AdjustNavigationOmnidirectional.Result()

        g = goal_handle.request
        dx                 = float(g.dx)
        dy                 = float(g.dy)
        ang_obstacle_check = float(g.ang_obstacle_check)
        safety             = bool(g.safety)
        max_speed          = float(g.max_speed)
        tolerance          = float(g.tolerance)
        kp                 = float(g.kp)
        enter_special      = bool(g.enter_house_special_case)
        use_wheel_odom     = bool(g.use_wheel_odometry)
        timeout_s          = float(g.timeout)

        start_clock = self.get_clock().now()
        t0 = time.monotonic()

        def should_stop_fn():
            if not rclpy.ok() or not goal_handle.is_active:
                return True
            if goal_handle.is_cancel_requested:
                return True
            if timeout_s > 0.0:
                elapsed = (self.get_clock().now() - start_clock).nanoseconds * 1e-9
                if elapsed >= timeout_s:
                    return True
            return False

        def feedback_fn(elapsed_s: float, remaining_m: float):
            fb.navigation_time = Duration()
            fb.navigation_time.sec = int(elapsed_s)
            fb.navigation_time.nanosec = int((elapsed_s - int(elapsed_s)) * 1e9)
            fb.distance_remaining = float(remaining_m)
            goal_handle.publish_feedback(fb)

        try:
            err_code, msg = self.adjust_omnidirectional(
                dx=dx, dy=dy,
                ang_obstacle_check=ang_obstacle_check,
                safety=safety,
                max_speed=max_speed,
                tolerance=tolerance,
                kp=kp,
                enter_house_special_case=enter_special,
                use_wheel_odometry=use_wheel_odom,
                should_stop_fn=should_stop_fn,
                feedback_fn=feedback_fn,
                timeout_s=timeout_s,
            )

            if err_code == self.ERR_SUCCESS:
                self.set_rgb(GREEN+BACK_AND_FORTH_8)
                if not enter_special:
                    self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.succeed()
                res.error_code = self.ERR_SUCCESS
            elif err_code == self.ERR_CANCELED:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                if not enter_special:
                    self._stop_robot()
                goal_handle.canceled()
                res.error_code = self.ERR_CANCELED
            elif err_code == self.ERR_TIMEOUT:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                if not enter_special:
                    self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_TIMEOUT
            elif err_code == self.ERR_NO_ODOM:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                if not enter_special:
                    self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_NO_ODOM
            else:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                if not enter_special:
                    self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_EXCEPTION

            return res

        except Exception:
            self.get_logger().exception("Exception in _adjust_nav_omni_execute_cb")
            self._stop_robot()
            if goal_handle.is_active:
                goal_handle.abort()
            res.error_code = self.ERR_EXCEPTION
            return res

        finally:
            self._clear_active_goal(goal_handle)


    def adjust_omnidirectional(self, dx, dy, ang_obstacle_check=45.0, safety=True, max_speed=0.05, tolerance=0.01, kp=1.5, enter_house_special_case=False, use_wheel_odometry=False, should_stop_fn=None, feedback_fn=None, timeout_s=0.0):

        self.set_rgb(BLUE+BACK_AND_FORTH_8)

        SAFETY_DISTANCE_FROM_ROBOT_EDGE = 0.02  # m

        # Wait for odom (cooperatively)
        t0 = time.monotonic()
        src = None
        while src is None:
            if should_stop_fn and should_stop_fn():
                if timeout_s > 0.0 and (time.monotonic() - t0) >= timeout_s:
                    return self.ERR_TIMEOUT, "Timeout before odometry available"
                return self.ERR_CANCELED, "Canceled before start"
            with self._lock:
                src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
            time.sleep(0.01)

        # Optional safety check (front-only, as in your legacy code)
        if safety and dx > 0.0:
            ok, msg, min_edge = self.get_minimum_radar_distance(direction=0.0, ang_obstacle_check=ang_obstacle_check, timeout_s=0.5)
            if not ok:
                self.get_logger().warn(f"Safety check failed: {msg}")
                return self.ERR_EXCEPTION, msg if msg else "Radar safety query failed"
            if (min_edge - SAFETY_DISTANCE_FROM_ROBOT_EDGE) < dx:
                warn = ("Not enough space in front of the robot to perform the adjustment "
                        f"(wanted dx={dx:.2f} m, free ~{(min_edge - SAFETY_DISTANCE_FROM_ROBOT_EDGE):.2f} m)")
                self.get_logger().warn(warn)
                return self.ERR_EXCEPTION, warn

        # Snapshot initial pose & yaw
        with self._lock:
            src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
            q = src.pose.orientation
            start_x = src.pose.position.x
            start_y = src.pose.position.y
        yaw0 = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Nudge toward the target by tolerance (your legacy behavior)
        dx_adj = dx + (tolerance if dx > 0 else (-tolerance if dx < 0 else 0.0))
        dy_adj = dy + (tolerance if dy > 0 else (-tolerance if dy < 0 else 0.0))

        # Compute target in odom frame using current yaw
        target_x = start_x + math.cos(yaw0) * dx_adj - math.sin(yaw0) * dy_adj
        target_y = start_y + math.sin(yaw0) * dx_adj + math.cos(yaw0) * dy_adj

        rate_hz = 20.0
        dt = 1.0 / rate_hz
        tstart = time.monotonic()

        self.set_rgb(CYAN+BACK_AND_FORTH_8)

        while True:
            if should_stop_fn and should_stop_fn():
                if timeout_s > 0.0 and (time.monotonic() - tstart) >= timeout_s:
                    return self.ERR_TIMEOUT, "Timeout"
                return self.ERR_CANCELED, "Canceled"

            with self._lock:
                src = self.current_odom_wheels_pose if use_wheel_odometry else self.current_odom_pose
                if src is None:
                    return self.ERR_NO_ODOM, "Lost odometry"
                pose = src.pose
                q = pose.orientation
                curr_x = pose.position.x
                curr_y = pose.position.y
                # Optionally keep updating yaw if you want body-axis alignment to be exact
                yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

            # Position error in odom
            ex = target_x - curr_x
            ey = target_y - curr_y
            dist_error = math.hypot(ex, ey)

            # feedback
            if feedback_fn:
                feedback_fn(time.monotonic() - tstart, dist_error)

            # Done?
            if dist_error < tolerance:
                break

            # Convert error from odom to base frame using current yaw
            vx =  math.cos(-yaw) * ex - math.sin(-yaw) * ey
            vy =  math.sin(-yaw) * ex + math.cos(-yaw) * ey

            twist = Twist()
            twist.linear.x = max(-max_speed, min(max_speed, kp * vx))
            twist.linear.y = max(-max_speed, min(max_speed, kp * vy))
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)

            time.sleep(dt)

        self.get_logger().info("Omnidirectional Adjustment Complete.")
        return self.ERR_SUCCESS, "Omnidirectional Adjustment Complete."



    def _adjust_nav_obst_goal_cb(self, goal: AdjustNavigationObstacles.Goal):
        distance            = float(goal.distance)
        direction_deg       = float(goal.direction)
        ang_obstacle_check  = float(goal.ang_obstacle_check)
        max_speed           = float(goal.max_speed)
        tolerance           = float(goal.tolerance)
        kp                  = float(goal.kp)
        timeout_s           = float(goal.timeout)

        # Basic validation (mirror your earlier style)
        if distance <= 0.0:
            self.get_logger().warn("Reject: distance must be > 0 (m)")
            return GoalResponse.REJECT
        if not (-100.0 <= direction_deg <= 100.0):
            self.get_logger().warn("Reject: direction must be in [-100, 100] deg")
            return GoalResponse.REJECT
        if not (0.0 < ang_obstacle_check <= 360.0):
            self.get_logger().warn("Reject: ang_obstacle_check must be in (0, 360]")
            return GoalResponse.REJECT
        if max_speed <= 0.0:
            self.get_logger().warn("Reject: max_speed must be > 0 (m/s)")
            return GoalResponse.REJECT
        if tolerance <= 0.0:
            self.get_logger().warn("Reject: tolerance must be > 0 (m)")
            return GoalResponse.REJECT
        if kp <= 0.0:
            self.get_logger().warn("Reject: kp must be > 0")
            return GoalResponse.REJECT
        if timeout_s < 0.0:
            self.get_logger().warn("Reject: timeout must be >= 0 seconds (0 = no timeout)")
            return GoalResponse.REJECT

        # Enforce single active goal (share same guard with angle action)
        with self._goal_lock:
            if self._active_goal is not None and self._active_goal.is_active:
                self.get_logger().warn("Reject: another navigation goal is active.")
                return GoalResponse.REJECT

        self.get_logger().info(
            f"Accept AdjustNavigationObstacles: dist={distance:.3f} m, dir={direction_deg:.1f}°, "
            f"win={ang_obstacle_check:.1f}°, vmax={max_speed:.2f} m/s, tol={tolerance:.3f} m, "
            f"kp={kp:.2f}, timeout={timeout_s:.2f}s"
        )
        return GoalResponse.ACCEPT


    def _adjust_nav_obst_cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested for AdjustNavigationObstacles.")
        return CancelResponse.ACCEPT


    def _adjust_nav_obst_execute_cb(self, goal_handle):
        # mark active
        with self._goal_lock:
            self._active_goal = goal_handle

        fb = AdjustNavigationObstacles.Feedback()
        res = AdjustNavigationObstacles.Result()

        g = goal_handle.request
        distance           = float(g.distance)
        direction_deg      = float(g.direction)
        ang_check_deg      = float(g.ang_obstacle_check)
        max_speed          = float(g.max_speed)
        tolerance          = float(g.tolerance)
        kp                 = float(g.kp)
        timeout_s          = float(g.timeout)

        start_clock = self.get_clock().now()
        start_mono  = time.monotonic()

        def should_stop_fn():
            if not rclpy.ok() or not goal_handle.is_active:
                return True
            if goal_handle.is_cancel_requested:
                return True
            if timeout_s > 0.0:
                elapsed = (self.get_clock().now() - start_clock).nanoseconds * 1e-9
                if elapsed >= timeout_s:
                    return True
            return False

        def feedback_fn(elapsed_s: float, remaining_m: float):
            fb.navigation_time = Duration()
            fb.navigation_time.sec = int(elapsed_s)
            fb.navigation_time.nanosec = int((elapsed_s - int(elapsed_s)) * 1e9)
            fb.distance_remaining = float(remaining_m)
            goal_handle.publish_feedback(fb)

        try:
            err_code, msg = self.adjust_obstacles(
                distance=distance,
                direction=direction_deg,
                ang_obstacle_check=ang_check_deg,
                max_speed=max_speed,
                tolerance=tolerance,
                kp=kp,
                should_stop_fn=should_stop_fn,
                feedback_fn=feedback_fn,
                timeout_s=timeout_s,
            )

            if err_code == self.ERR_SUCCESS:
                self.set_rgb(GREEN+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.succeed()
                res.error_code = self.ERR_SUCCESS
            elif err_code == self.ERR_CANCELED:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                goal_handle.canceled()
                res.error_code = self.ERR_CANCELED
            elif err_code == self.ERR_TIMEOUT:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_TIMEOUT
            elif err_code == self.ERR_NO_ODOM:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                # not used here, but keep mapping consistent
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_NO_ODOM
            else:
                self.set_rgb(RED+BACK_AND_FORTH_8)
                self._stop_robot()
                if goal_handle.is_active:
                    goal_handle.abort()
                res.error_code = self.ERR_EXCEPTION

            return res

        except Exception:
            self.get_logger().exception("Exception in _adjust_nav_obst_execute_cb")
            self._stop_robot()
            if goal_handle.is_active:
                goal_handle.abort()
            res.error_code = self.ERR_EXCEPTION
            return res

        finally:
            self._clear_active_goal(goal_handle)


    def adjust_obstacles(self, distance=0.0, direction=0.0, ang_obstacle_check=45, max_speed=0.05, tolerance=0.01, kp=1.5, \
                         should_stop_fn=None, feedback_fn=None, timeout_s=0.0):

        self.set_rgb(BLUE+BACK_AND_FORTH_8)

        # Cooperative pre-check
        if should_stop_fn and should_stop_fn():
            if timeout_s > 0.0:
                return self.ERR_TIMEOUT, "Timeout"
            return self.ERR_CANCELED, "Canceled"

        # Normalize direction to [-180, 180]
        while direction > 180.0:
            direction -= 360.0
        while direction < -180.0:
            direction += 360.0

        # Query radar (give it a bounded slice of our timeout if provided)
        srv_timeout = 0.5 if timeout_s == 0.0 else min(max(0.2, 0.5 * timeout_s), 2.0)
        ok, msg, min_edge = self.get_minimum_radar_distance(
            direction=direction,
            ang_obstacle_check=ang_obstacle_check,
            timeout_s=srv_timeout,
        )

        if not ok:
            # Map service outcomes to error codes
            if msg and msg.lower().startswith("timeout"):
                return self.ERR_TIMEOUT, "Radar query timeout"
            if msg and "No obstacles detected" in msg:
                # If there are no obstacles, the distance is effectively "infinite"; in this case,
                # move forward until we reach the target 'distance' (i.e., distance_to_adjust = +inf).
                # But we’ll be conservative and just say nothing to do.
                return self.ERR_SUCCESS, "No obstacles in window; nothing to adjust"
            return self.ERR_EXCEPTION, (msg if msg else "Radar query failed")

        # How much we need to change the clearance (positive => move toward obstacle)
        distance_to_adjust = float(min_edge) - float(distance)

        # Early exit if within tolerance
        if abs(distance_to_adjust) < float(tolerance):
            self.get_logger().info(
                f"Obstacle clearance already within tolerance (err={distance_to_adjust:.3f} m)."
            )
            return self.ERR_SUCCESS, "Already within tolerance"

        self.get_logger().info(
            f"Adjusting obstacle clearance by {distance_to_adjust:.3f} m at dir {direction:.1f}° "
            f"(min_edge={min_edge:.3f} m → target={distance:.3f} m)."
        )

        # Convert scalar motion along 'direction' (robot frame) into (dx, dy) in robot frame
        rad = math.radians(direction)
        dx = distance_to_adjust * math.cos(rad)
        dy = distance_to_adjust * math.sin(rad)

        # Delegate motion to omnidirectional adjust (no safety here—it’s a deliberate approach)
        return self.adjust_omnidirectional(
            dx=dx,
            dy=dy,
            ang_obstacle_check=ang_obstacle_check,  # unused when safety=False, but keep interface consistent
            safety=False,
            max_speed=max_speed,
            tolerance=tolerance,
            kp=kp,
            enter_house_special_case=False,
            use_wheel_odometry=False,
            should_stop_fn=should_stop_fn,
            feedback_fn=feedback_fn,  # passes through distance_remaining feedback
            timeout_s=timeout_s,
        )

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










    def _charmie_nav_goal_cb(self, goal: NavigateToPose.Goal):
        # If we already have a Charmie goal running, mark it as CANCELED (preemption)
        if self._active_charmie_goal and self._active_charmie_goal.is_active:
            try:
                self.get_logger().info("Preempt: canceling previous Charmie goal (marking CANCELED).")
                self._active_charmie_goal.canceled()
            except Exception as e:
                self.get_logger().warn(f"Could not mark previous Charmie goal canceled: {e}")

        # Also cancel the underlying Nav2 goal so the robot stops moving
        if self.goal_handle_ is not None:
            self.get_logger().info("Preempt: canceling current Nav2 NavigateToPose.")
            try:
                self.nav2_client_cancel_goal()
            except Exception as e:
                self.get_logger().warn(f"Preempt: Nav2 cancel request failed/ignored: {e}")

        return GoalResponse.ACCEPT


    def _charmie_nav_cancel_cb(self, goal_handle):
        # We’ll forward a cancel to Nav2 when we detect it in execute()
        return CancelResponse.ACCEPT

    def _charmie_nav_execute_cb(self, goal_handle):
        # ---- ADD: record this as the active Charmie goal
        self._active_charmie_goal = goal_handle
        try:
            goal: NavigateToPose.Goal = goal_handle.request
            pose = goal.pose.pose

            x = float(pose.position.x)
            y = float(pose.position.y)
            theta_deg = math.degrees(self.get_yaw_from_quaternion(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
            move_coords = [x, y, theta_deg]

            ok, msg = self.move_to_position(
                move_coords=move_coords,
                print_feedback=True,
                feedback_freq=10.0,
                clear_costmaps=True,
                inspection_safety_nav=False,
                wait_for_end_of=True,
                goal_handle=goal_handle,
            )

            result = NavigateToPose.Result()

            # ---- ADD: if this goal was already marked canceled (by preemption), exit quietly
            if not goal_handle.is_active:
                return result

            # Your existing logic:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

        finally:
            # ---- ADD: clear the pointer when we leave execute
            self._active_charmie_goal = None



    # FUNCTIONS TO CALL NAV2 ACTION SERVER
    def nav2_client_goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.nav2_client_goal_result_callback)
            self.nav2_goal_accepted = True
        else:
            self.nav2_goal_accepted = False
            self.get_logger().warn("Goal rejected.")

    def nav2_client_goal_result_callback(self, future):
        status = future.result().status
        # result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_status = GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav2_status = GoalStatus.STATUS_ABORTED
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.nav2_status = GoalStatus.STATUS_CANCELED
            self.get_logger().warn("CANCELED.")
        
        self.goal_handle_ = None
            
        # self.get_logger().info(f"Result: {result.reached_number}")
    def nav2_client_cancel_goal(self):
        if self.goal_handle_ is None:
            self.get_logger().warn("No active NavigateToPose goal handle to cancel.")
            return

        self.get_logger().info("Sending cancel request to Nav2...")
        self.goal_handle_.cancel_goal_async()
        self.get_logger().info("Cancel request sent.")
        self.goal_handle_ = None

    def nav2_client_goal_feedback_callback(self, feedback_msg):
        self.nav2_feedback = feedback_msg.feedback
        # print(type(feedback))   
        # current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        # current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        # current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        # no_recoveries = str(feedback.number_of_recoveries)
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")




    def move_to_position(self, move_coords, print_feedback=True, feedback_freq=10.0, clear_costmaps=True, inspection_safety_nav=False, wait_for_end_of=True, goal_handle=None):

        # Whether the nav2 goal has been successfully completed until the end
        nav2_goal_completed = False

        # Create a goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(move_coords[0])
        goal_msg.pose.pose.position.y = float(move_coords[1])
        goal_msg.pose.pose.position.z = float(0.0)
        q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(move_coords[2]))        
        goal_msg.pose.pose.orientation.x = q_x
        goal_msg.pose.pose.orientation.y = q_y
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w

        success = True
        message = ""
        
        self.set_rgb(BLUE+BACK_AND_FORTH_8)

        while not nav2_goal_completed:

            # Clear costmaps before sending a new goal
            # Helps clearing cluttered costmaps that may cause navigation problems
            if clear_costmaps:
                self.clear_costmaps()
                time.sleep(0.5) # wait a bit for costmaps to be cleared
                
            self.nav2_goal_accepted = False
            self.nav2_status = GoalStatus.STATUS_UNKNOWN

            # Makes sure goal is accepted, and if not attempts to resend it
            while not self.nav2_goal_accepted:
                
                self.get_logger().info("Waiting for nav2 server...")
                self.nav2_client_.wait_for_server()
                self.get_logger().info("Nav2 server is ON...")

                # Send the goal
                self.get_logger().info("Sending goal...")
                self.nav2_client_.send_goal_async(goal_msg, feedback_callback=self.nav2_client_goal_feedback_callback).add_done_callback(self.nav2_client_goal_response_callback)
                self.get_logger().info("Goal Sent")

                time.sleep(0.5)


            if wait_for_end_of:

                feedback_timer_period = 1.0 / feedback_freq  # Convert Hz to seconds
                feedback_start_time = time.time()

                # is_canceled = False

                self.set_rgb(CYAN+BACK_AND_FORTH_8)

                while self.nav2_status == GoalStatus.STATUS_UNKNOWN:
                    
                    if goal_handle is not None and goal_handle.is_cancel_requested:
                        self.nav2_client_cancel_goal()
                        self.set_rgb(RED + BACK_AND_FORTH_8)
                        return False, "Canceled by client"

                    # Checks conditions to cancel safety navigation (used in inspection task)
                    # if inspection_safety_nav and not self.check_conditions_to_stop_safety_navigation(move_coords) and not is_canceled:
                    #     self.nav2_client_cancel_goal()
                    #     is_canceled = True
                    
                    if print_feedback:

                        if time.time() - feedback_start_time > feedback_timer_period:

                            # prints de feedback
                            feedback = self.nav2_feedback
                            current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
                            current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
                            current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
                            navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
                            estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
                            no_recoveries = str(feedback.number_of_recoveries)
                            distance_remaining = str(round(feedback.distance_remaining, 2))
                            print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
                            # self.get_logger().info(f"Feedback: {feedback}")
                            
                            if goal_handle is not None:
                                # nav2_feedback is already NavigateToPose.Feedback – publish directly
                                goal_handle.publish_feedback(self.nav2_feedback)
                            
                            feedback_start_time = time.time()

                
                if self.nav2_status == GoalStatus.STATUS_SUCCEEDED:
                    self.set_rgb(GREEN+BACK_AND_FORTH_8)
                    self.get_logger().info("NAV2 RESULT: SUCCEEDED.")
                    nav2_goal_completed = True
                    success = True
                    message = "Successfully moved to position"
                    return success, message
                elif self.nav2_status == GoalStatus.STATUS_ABORTED:
                    self.set_rgb(RED+BACK_AND_FORTH_8)
                    self.get_logger().info("NAV2 RESULT: ABORTED.")
                    self.get_logger().info("ATTEMPING TO RETRY MOVEMENT TO GOAL POSE.")
                elif self.nav2_status == GoalStatus.STATUS_CANCELED:
                    self.set_rgb(RED+BACK_AND_FORTH_8)
                    self.get_logger().info("NAV2 RESULT: CANCELED.")
                    success = False
                    message = "Aborted moved to position due to safety measures"
                    return success, message


       
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
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		
        #print(qx,qy,qz,qw)
  
        return [qx, qy, qz, qw]

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
