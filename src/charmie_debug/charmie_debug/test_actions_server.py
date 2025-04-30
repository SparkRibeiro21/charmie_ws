#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from charmie_interfaces.action import CountUntil

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import threading


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock = threading.Lock()
        self.goal_queue = []
        self.count_until_server_ = ActionServer(
            self, 
            CountUntil, 
            "count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
            )
        self.get_logger().info("Action Server has been started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received new goal request.")

        # Policy: refuse new goal if current goal still active
        # with self.goal_lock:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Another goal is already active. Rejecting new goal")
        #         return GoalResponse.REJECT

        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().warn("Invalid target number.")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        # with self.goal_lock:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abborting current goal and accepting new goal.")
        #         self.goal_handle_.abort()

        self.get_logger().info("Accepted the goal.")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:
            if self.goal_handle_ is not None:
                self.goal_queue.append(goal_handle)

            else:
                goal_handle.execute()



    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("Received cancel request.")
        return CancelResponse.ACCEPT
        # return CancelResponse.REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        with self.goal_lock:
            self.goal_handle_ = goal_handle
        
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing goal...")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                self.get_logger().warn("Goal has been abborted.")
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Goal has been canceled.")
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            counter += 1
            self.get_logger().info(f"Current number: {counter}")
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # Once done, set the goal final state
        goal_handle.succeed()
        # goal_handle.abort()

        # And send the result
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result 
    
    def process_next_goal_in_queue(self):
        with self.goal_lock:
            if len(self.goal_queue) > 0:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()