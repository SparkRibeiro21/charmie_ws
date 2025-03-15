#!/usr/bin/env python3
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from charmie_interfaces.action import CountUntil

import time


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        self.get_logger().info("Action Client has been started.")
        

    def send_goal(self, target_number, period):

        self.count_until_client_.wait_for_server()

        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending goal...")
        
        ### send_goal blocks the ros2 structure whereas the send_goal_async does not
        # self.count_until_client_.send_goal(goal)
        self.count_until_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info("Canceling goal...")
        self.goal_handle_.cancel_goal_async() # Not ideal, but just to test, goal_handle_ is only defined in goal_response_callback
        self.timer_.cancel()

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
            
        self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Feedback: {number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(20, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()