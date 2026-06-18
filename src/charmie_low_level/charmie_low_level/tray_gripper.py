#!/usr/bin/env python3
"""
tray_gripper.py
ROS 2 Python node that sends UDP commands to the W6100-EVB-Pico.

Topics
------
  Server : /tray_gripper_command        (charmie_interfaces/SetInt)   - command to send to Pico

The Pico listens on UDP port 5005 and expects a plain integer payload such as
"0", "90", or "180".
"""

import socket

import rclpy
from rclpy.node import Node
from charmie_interfaces.srv import SetInt


class TrayGripperNode(Node):
    def __init__(self):
        super().__init__("tray_gripper")

        self.declare_parameter("pico_ip", "192.168.1.177")
        self.declare_parameter("pico_port", 5005)

        self._pico_ip = str(self.get_parameter("pico_ip").value)
        self._pico_port = int(self.get_parameter("pico_port").value or 5005)

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pico_endpoint = (self._pico_ip, self._pico_port)

        # self._cmd_sub = self.create_subscription(Int32, "pico_cmd", self._cmd_callback, 10)
        self.server_tray_gripper_command = self.create_service(SetInt, "tray_gripper_command", self.callback_tray_gripper_command)

        self.get_logger().info(f"tray_gripper sending UDP to {self._pico_ip}:{self._pico_port}")

    def callback_tray_gripper_command(self, request, response):

        # Type of service received: 
        # int32 data   # generic int value sent 
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        pct = max(0, min(100, int(100-request.data)))
        value = pct * 170 // 100
        payload = str(value).encode("utf-8")
        
        self._udp_socket.sendto(payload, self._pico_endpoint)
        message = f"[pico_cmd] sent {pct}% = {value} (0-170 scale) to {self._pico_ip}:{self._pico_port}"
        self.get_logger().info(message)

        response.success = True
        response.message = message

        return response

    def send_command(self, value: int) -> None:
        """Send an arbitrary Int32 value to the Pico immediately."""
        payload = str(int(value)).encode("utf-8")
        self._udp_socket.sendto(payload, self._pico_endpoint)
        self.get_logger().info(
            f"[pico_cmd] manual send {int(value)} to {self._pico_ip}:{self._pico_port}"
        )

    def destroy_node(self) -> None:
        self._udp_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrayGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()