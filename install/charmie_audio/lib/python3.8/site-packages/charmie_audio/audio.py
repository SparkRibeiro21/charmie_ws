#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import socket
import threading

class RaspAudio():
    def __init__(self):
        self.comando = ""
        self.running = 1
        self.flag_new_command = False

        localIP = "192.168.1.1"
        RaspIP = "192.168.1.2"

        localPort = 5005
        RaspPort = 5001

        self.adr = (RaspIP, RaspPort)

        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.setblocking(0)
        self.UDPServerSocket.bind((localIP, localPort))

    def check_received_audio(self):
        
        while self.running:
            # print(".")
            try:
                message, adr = self.UDPServerSocket.recvfrom(1024)
                self.comando = message.decode('utf8', 'strict')
                print(self.comando)
                if self.comando == 'ERROR':
                    print("Deu Erro!")
                    # speaker, frase a dizer: I could not understand what you said. Could you please repeat?
            
                    # self.UDPServerSocket.sendto(str.encode("OUVE"), self.adr)
            
                else:
                    print("Nao deu Erro!")
                    self.flag_new_command = True
            except Exception as e:
                self.running = 1


    def send_to_audio(self, my_str: str):
        self.UDPServerSocket.sendto(str.encode(my_str), self.adr)
            
class AudioNode(Node):

    def __init__(self):
        super().__init__("Audio")
        self.get_logger().info("Initialised CHARMIE Audio Node")

        self.charmie_audio = RaspAudio()


        self.create_timer(30, self.timer_callback)
        # self.create_timer(0.05, self.timer_callback2)
    
        self.charmie_audio.send_to_audio("OUVE")
        print("OUVE PRINT")


    def timer_callback(self):
        self.charmie_audio.send_to_audio("OUVE")
        print("OUVE PRINT")

    # def timer_callback2(self):
        # print(".")
        # self.charmie_audio.check_received_audio()


def thread_audio(node):
    node.charmie_audio.check_received_audio()

    
def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()

    # crete thread to listen to the controller since it blocks the liten function
    th_aud = threading.Thread(target=thread_audio, args=(node,), daemon=True)
    th_aud.start()

    rclpy.spin(node)
    rclpy.shutdown()
