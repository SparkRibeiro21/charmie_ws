#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool

import time
import socket
import subprocess


class RobotFace():
    def __init__(self):

        self.ssid = "ShiningRGB_7747"
        self.password = ""
        self.is_connected_to_face = False

        # Define the server address and port
        self.server_address = ('192.168.4.1', 8810)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Send a hex message to the server
        self.hex_message = []
        self.hex_message.append('5aa5000004000403030000')
        self.hex_message.append('5aa50000040004030a0009')

        self.max_command = len(self.hex_message)
        print("Number of faces templates created:", self.max_command)

        # DEBUG to list all wifi networks
        # wifi_networks = self.list_wifi_networks()
        # print("Available Wi-Fi Networks:")
        # for network in wifi_networks:
        #     print(network)


    def init_face(self):

        # Connect the socket to the server's address and port
        self.sock.connect(self.server_address)

        # Send Initial Face
        init_command = Int16()
        init_command.data = 0
        self.send_command(init_command)


    def connect_shining_rgb_face_wifi(self):
        try:
            subprocess.check_output(['nmcli', 'dev', 'wifi', 'connect', self.ssid, 'password', self.password], universal_newlines=True)
            return True
        except subprocess.CalledProcessError:
            return False

    def list_wifi_networks(self):
        try:
            output = subprocess.check_output(['nmcli', '-f', 'SSID', 'dev', 'wifi', 'list'], universal_newlines=True)
            networks = output.strip().split('\n')[1:]
            return [network.strip() for network in networks]
        except subprocess.CalledProcessError:
            return []


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
        self.get_logger().info("Connecting to ShiningRBG_7747...")    
        while(not self.charmie_face.is_connected_to_face):
            if self.charmie_face.connect_shining_rgb_face_wifi():
                self.charmie_face.is_connected_to_face = True
                self.get_logger().info("Successfully connected to ShiningRBG_7747 (CHARMIE Face Wi-fi).")
            else:
                self.get_logger().info("Failed to connect to ShiningRBG_7747 (CHARMIE Face Wi-fi).")
            time.sleep(1.0)


        
        self.face_command_subscriber = self.create_subscription(Int16, "face_command", self.face_command_callback, 10)


        self.charmie_face.init_face()

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

        self.face_diagnostic_publisher = self.create_publisher(Bool, "face_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.face_diagnostic_publisher.publish(flag_diagn)

        
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
