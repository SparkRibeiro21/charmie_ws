#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from charmie_interfaces.srv import NamedTarget, JointTarget, PoseTarget

class CommanderClient(Node):
    def __init__(self):
        super().__init__('commander_client_node')
        
        # Create service clients
        self.named_target_client = self.create_client(NamedTarget, 'set_named_target')
        self.joint_target_client = self.create_client(JointTarget, 'set_joint_target')
        self.pose_target_client = self.create_client(PoseTarget, 'set_pose_target')
        
        self.get_logger().info('Commander Client Node initialized')
    
    def send_named_target(self, target_name):
        """
        Send a named target request (e.g., 'home', 'rest')
        
        Args:
            target_name (str): Name of the target pose
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.named_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Named target service not available')
            return False
        
        request = NamedTarget.Request()
        request.target_name = target_name
        
        self.get_logger().info(f'Sending named target request: {target_name}')
        
        future = self.named_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            status = 'SUCCESS' if response.success else 'FAILED'
            self.get_logger().info(f'Response: {status} - {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call named target service')
            return False
    
    def send_joint_target(self, joint_positions):
        """
        Send a joint target request
        
        Args:
            joint_positions (list): List of joint angles in radians
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.joint_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Joint target service not available')
            return False
        
        request = JointTarget.Request()
        request.joint_positions = joint_positions
        
        self.get_logger().info(f'Sending joint target request with {len(joint_positions)} positions')
        
        future = self.joint_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            status = 'SUCCESS' if response.success else 'FAILED'
            self.get_logger().info(f'Response: {status} - {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call joint target service')
            return False
    
    def send_pose_target(self, x, y, z, roll, pitch, yaw, cartesian=False):
        """
        Send a pose target request
        
        Args:
            x, y, z (float): Position in meters
            roll, pitch, yaw (float): Orientation in radians
            cartesian (bool): If True, use cartesian path planning
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.pose_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Pose target service not available')
            return False
        
        request = PoseTarget.Request()
        request.x = x
        request.y = y
        request.z = z
        request.roll = roll
        request.pitch = pitch
        request.yaw = yaw
        request.cartesian = cartesian
        
        self.get_logger().info(
            f'Sending pose target: pos({x:f}, {y:f}, {z:f}), '
            f'orient({roll:f}, {pitch:f}, {yaw:f}), '
            f'cartesian={cartesian}'
        )
        
        future = self.pose_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            status = 'SUCCESS' if response.success else 'FAILED'
            self.get_logger().info(f'Response: {status} - {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call pose target service')
            return False


def main(args=None):
    rclpy.init(args=args)
    client_node = CommanderClient()
        
    client_node.send_named_target('pick_front')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()