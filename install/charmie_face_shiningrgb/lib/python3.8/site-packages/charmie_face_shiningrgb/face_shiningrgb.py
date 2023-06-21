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
        self.hex_message = []
        self.hex_message.append('5aa5000004000403030000')
        self.hex_message.append('5aa50000040004030a0009')

        self.max_command = len(self.hex_message)
        print("Number of faces templates created:", self.max_command)

    def send_command(self, mode: Int16):
        
        print("Set Face No:", mode.data, ", with ID:", self.hex_message[mode.data])
        byte_message = bytes.fromhex(self.hex_message[mode.data])
        self.sock.sendall(byte_message)

        # Receive the response from the server
        # data = self.sock.recv(1024)

        # Print the response from the server
        # print('Received:', data.decode())

        # Close the socket
        # self.sock.close()



class FaceNode(Node):

    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")
        self.charmie_face = RobotFace()

        self.face_command_subscriber = self.create_subscription(Int16, "face_command", self.face_command_callback, 10)

    """
        # self.create_timer(2, self.timer_callback)
        # self.counter = 0

    # def timer_callback(self):
        face = Int16()
        face.data = self.counter
        self.charmie_face.send_command(face)
        self.counter += 1
        if self.counter > 1:
            self.counter = 0
    """

        
    def face_command_callback(self, command: Int16):
        # print("Received Command:", command.data)
        if command.data < self.charmie_face.max_command and command.data >= 0:
            self.charmie_face.send_command(command)
        else:
            print("Error! Invalid Face ID!")

        
        
def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
