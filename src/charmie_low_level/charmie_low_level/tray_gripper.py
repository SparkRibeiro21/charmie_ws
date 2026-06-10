#!/usr/bin/env python3
"""
milk_ros.py
ROS 2 Python node that sends UDP commands to the W6100-EVB-Pico.

Topics
------
  Subscribes : /pico_cmd        (std_msgs/Int32)   - command to send to Pico

The Pico listens on UDP port 5005 and expects a plain integer payload such as
"0", "90", or "180".
"""

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MilkPicoNode(Node):
    def __init__(self):
        super().__init__("milk_pico_node")

        self.declare_parameter("pico_ip", "192.168.1.177")
        self.declare_parameter("pico_port", 5005)

        self._pico_ip = str(self.get_parameter("pico_ip").value)
        self._pico_port = int(self.get_parameter("pico_port").value or 5005)

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pico_endpoint = (self._pico_ip, self._pico_port)

        self._cmd_sub = self.create_subscription(
            Int32,
            "pico_cmd",
            self._cmd_callback,
            10,
        )

        self.get_logger().info(
            f"milk_pico_node sending UDP to {self._pico_ip}:{self._pico_port}"
        )

    def _cmd_callback(self, msg: Int32) -> None:
        pct = max(0, min(100, int(msg.data)))
        value = pct * 120 // 100
        payload = str(value).encode("utf-8")
        
        self._udp_socket.sendto(payload, self._pico_endpoint)
        self.get_logger().info(
            f"[pico_cmd] sent {pct}% = {value} (0-120 scale) to {self._pico_ip}:{self._pico_port}"
        )

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
    node = MilkPicoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()