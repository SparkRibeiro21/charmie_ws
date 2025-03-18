#!/usr/bin/env python3
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped

import time
import math


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("nav2_client_TR")
        self.nav2_client_TR_ = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav2_client_follow_waypoints_TR_ = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.get_logger().info("Nav2 action client has been started.")

    def send_goal_follow_waypoints(self):

        self.get_logger().info("Waiting for nav2 through poses server...")
        self.nav2_client_follow_waypoints_TR_.wait_for_server()
        self.get_logger().info("Nav2 through poses server is ON...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = []
        
        waypoints = [
        (2.0, -3.0, 1.5707),
        (3.0, -1.0, 1.5707),
        (2.0,  1.0, 1.5707),
        ]

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            
            
            q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, yaw) # math.radians(initial_position[2]))        
            pose.pose.orientation.x = q_x
            pose.pose.orientation.y = q_y
            pose.pose.orientation.z = q_z
            pose.pose.orientation.w = q_w

            # Convert yaw (Euler) to quaternion
            # q = quaternion_from_euler(0, 0, yaw)  # roll=0, pitch=0, yaw=desired_rotation
            # pose.pose.orientation.x = q[0]
            # pose.pose.orientation.y = q[1]
            # pose.pose.orientation.z = q[2]
            # pose.pose.orientation.w = q[3]

            goal_msg.poses.append(pose)

        # Send the goal
        self.get_logger().info("Sending goal through poses...")
        
        ### send_goal blocks the ros2 structure whereas the send_goal_async does not
        self.nav2_client_follow_waypoints_TR_.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_follow_waypoints_callback).add_done_callback(self.goal_response_follow_waypoints_callback)


    def goal_response_follow_waypoints_callback(self, future):
        
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_follow_waypoints_callback)
        else:
            self.get_logger().warn("Goal rejected.")


    def goal_result_follow_waypoints_callback(self, future):
        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("CANCELED.")
            
        # self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_follow_waypoints_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        # current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        # current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        # navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        # estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        # no_recoveries = str(feedback.number_of_recoveries)
        # distance_remaining = str(round(feedback.distance_remaining, 2))
        # number_of_poses_remaining = str(feedback.number_of_poses_remaining)
        current_waypoint = str(feedback.current_waypoint)

        # print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left: " + distance_remaining + " Poses Left: " + number_of_poses_remaining)
        self.get_logger().info(f"🔄 Current Waypoint: {current_waypoint}")
        self.get_logger().info(f"🔄 Feedback: {feedback}")









    def send_goal(self):

        self.get_logger().info("Waiting for nav2 server...")
        self.nav2_client_TR_.wait_for_server()
        self.get_logger().info("Nav2 server is ON...")

        # Create a goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0 # float(initial_position[0])
        # goal_msg.pose.pose.position.y = 1.0 # float(initial_position[1])
        goal_msg.pose.pose.position.y = -2.0 # float(initial_position[1])
        goal_msg.pose.pose.position.z = float(0.0)
        # q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, 0.0) #math.radians(90.0)) # math.radians(initial_position[2]))        
        q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, math.radians(90.0)) # math.radians(initial_position[2]))        
        goal_msg.pose.pose.orientation.x = q_x
        goal_msg.pose.pose.orientation.y = q_y
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w

        # Send the goal
        self.get_logger().info("Sending goal...")
        
        ### send_goal blocks the ros2 structure whereas the send_goal_async does not
        self.nav2_client_TR_.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(5.0, self.cancel_goal)

    # def cancel_goal(self):
    #     self.get_logger().info("Canceling goal...")
    #     self.goal_handle_.cancel_goal_async() # Not ideal, but just to test, goal_handle_ is only defined in goal_response_callback
    #     self.timer_.cancel()

    def goal_response_callback(self, future):
        
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal rejected.")


    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("SUCCEEDED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("CANCELED.")
            
        # self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        current_pose_x = str(round(feedback.current_pose.pose.position.x, 2))
        current_pose_y = str(round(feedback.current_pose.pose.position.y, 2))
        current_pose_theta = str(round(math.degrees(self.get_yaw_from_quaternion(feedback.current_pose.pose.orientation.x, feedback.current_pose.pose.orientation.y, feedback.current_pose.pose.orientation.z, feedback.current_pose.pose.orientation.w)),2))
        navigation_time = str(round(feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9, 2))
        estimated_time_remaining = str(round(feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9, 2))
        no_recoveries = str(feedback.number_of_recoveries)
        distance_remaining = str(round(feedback.distance_remaining, 2))


        print("Current Pose: (" + current_pose_x + ", " + current_pose_y + ", " + current_pose_theta + ")" + " Times (nav, remain): (" + navigation_time + ", " + estimated_time_remaining + ")" + " Recoveries: " + no_recoveries + " Distance Left:" + distance_remaining)
        # self.get_logger().info(f"Feedback: {feedback}")



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
        """ Convert quaternion (x, y, z, w) to Yaw (rotation around Z-axis). """
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)  # Yaw angle in radians
    


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    # node.send_goal()
    node.send_goal_follow_waypoints()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    # ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

    """ #goal definition
    geometry_msgs/PoseStamped pose
    string behavior_tree
    ---
    #result definition

    # Error codes
    # Note: The expected priority order of the errors should match the message order
    uint16 NONE=0
    uint16 UNKNOWN=9000
    uint16 FAILED_TO_LOAD_BEHAVIOR_TREE=9001
    uint16 TF_ERROR=9002

    uint16 error_code
    string error_msg
    ---
    #feedback definition
    geometry_msgs/PoseStamped current_pose
    builtin_interfaces/Duration navigation_time
    builtin_interfaces/Duration estimated_time_remaining
    int16 number_of_recoveries
    float32 distance_remaining """