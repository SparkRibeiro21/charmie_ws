#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from charmie_interfaces.msg import RobotSpeech

import pygame
from gtts import gTTS

class RobotSpeak():
    def __init__(self):
        self.filename = '0.mp3'
        pygame.init()

    def speak(self, speech: String):
        # tts = gTTS(text=str(speech.data), lang='en')
        tts = gTTS(text=str(speech.data), lang='pt')
        tts.save(self.filename)
        pygame.mixer.music.load(self.filename)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pass
        
        # tts = gTTS(text=str(speech), lang='pt', tld='pt')

        

class SpeakerNode(Node):

    def __init__(self):
        super().__init__("Speaker")
        self.get_logger().info("Initialised CHARMIE Speaker Node")
        self.charmie_speech = RobotSpeak()
        print("oh yes oh yes")

        self.speaker_command_subscriber = self.create_subscription(String, "speaker_command", self.speaker_command_callback, 10)
        self.flag_speech_done_publisher = self.create_publisher(Bool, "flag_speech_done", 10)
        
    def speaker_command_callback(self, speech: String):

        print("Received String:", speech.data)
        self.charmie_speech.speak(speech)
        print("Finished Speaking.")
        


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
