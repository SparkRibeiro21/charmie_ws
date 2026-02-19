#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from charmie_interfaces.srv import SetFloat

from xarm.wrapper import XArmAPI


class XArmGripperService(Node):

    def __init__(self):
        super().__init__('xarm_gripper_service')

        # Parameters
        self.declare_parameter('robot_ip', '192.168.1.219')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # Connect to robot
        self.get_logger().info(f'Connecting to xArm at {self.robot_ip}...')
        self.arm = XArmAPI(self.robot_ip)
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        # Enable gripper
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_mode(0)

        self.get_logger().info('Gripper ready.')

        # Service: set gripper position in mm
        self.srv = self.create_service(
            SetFloat,
            'set_gripper_mm',
            self.set_gripper_callback
        )

    # -----------------------------
    # Service callback
    # -----------------------------
    def set_gripper_callback(self, request, response):
        pos = float(request.data)

        # Safety clamp (adjust to your gripper limits)
        pos = max(0.0, min(pos, 850.0))

        self.get_logger().info(f'Moving gripper to {pos:.1f} mm')

        code = self.arm.set_gripper_position(pos, wait=True)

        if code == 0:
            response.success = True
            response.message = f'Gripper moved to {pos:.1f} mm'
        else:
            response.success = False
            response.message = f'Gripper command failed (code {code})'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = XArmGripperService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()