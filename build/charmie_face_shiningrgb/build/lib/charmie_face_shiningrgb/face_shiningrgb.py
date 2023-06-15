#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

import socket


class RobotFace():
    def __init__(self):
        # Define the server address and port
        server_address = ('192.168.4.1', 8810)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the server's address and port
        self.sock.connect(server_address)

        # Send a hex message to the server
        self.hex_message = '5aa5000004000403030000'


class FaceNode(Node):

    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")
        self.charmie_face = RobotFace()

        self.speaker_command_subscriber = self.create_subscription(Int16, "face_command", self.face_command_callback, 10)
        
    def face_command_callback(self, command: Int16):
        print("Received Command:", command.data)

        byte_message = bytes.fromhex(self.charmie_face.hex_message)
        self.charmie_face.sock.sendall(byte_message)

        # Receive the response from the server
        data = self.charmie_face.sock.recv(1024)

        # Print the response from the server
        print('Received:', data.decode())

        # Close the socket
        self.charmie_face.sock.close()
        
        
def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
