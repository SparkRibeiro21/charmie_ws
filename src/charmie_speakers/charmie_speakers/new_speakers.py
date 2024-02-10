#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from charmie_interfaces.srv import SpeechCommand

from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
import pygame
from pathlib import Path

import subprocess
import re

VOICE_NO = 1


class RobotSpeak():
    def __init__(self):
        pygame.init()



class SpeakerNode(Node):

    def __init__(self):
        super().__init__("Speaker")
        self.get_logger().info("Initialised CHARMIE Speaker Node")

        self.server_speech_command = self.create_service(SpeechCommand, "speech_command", self.callback_speech_command) 
        self.get_logger().info("Speech Command Server has been started")
        

        self.charmie_speech = RobotSpeak()


    def callback_speech_command(self, request, response):
        print("Received request")
        # int64 a
        # int64 b
        # ---
        # int64 sum
        print(request.filename)

        response.success = True
        response.message = request.filename
        time.sleep(2)
        # self.get_logger().info(str(request.a) + "+" + str(request.b) + "=" + str(response.sum))
        return response



def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
