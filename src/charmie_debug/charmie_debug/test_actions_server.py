#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from charmie_interfaces.action import CountUntil

import time


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(
            self, 
            CountUntil, 
            "count_until",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info("Action Server has been started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received new goal request.")

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().warn("Invalid target number.")
            return GoalResponse.REJECT
        self.get_logger().info("Accepted the goal.")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing goal...")
        counter = 0
        for i in range(target_number):
            counter += 1
            self.get_logger().info(f"Current number: {counter}")
            time.sleep(period)

        # Once done, set the goal final state
        goal_handle.succeed()

        # And send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result 



        """
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("Goal has been canceled.")
            return CountUntil.Result()
        counter += 1
        goal_handle.publish_feedback(CountUntil.Feedback(current_number=counter))
        self.get_logger().info(f"Current number: {counter}")
        self.get_logger().info(f"Period: {period}")
        self.get_logger().info(f"Target number: {target_number}")
        self.get_logger().info("")
        """


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()