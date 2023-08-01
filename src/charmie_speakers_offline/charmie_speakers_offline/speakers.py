#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from charmie_interfaces.msg import RobotSpeech

# import pygame
# from gtts import gTTS
# from pathlib import Path
import pyttsx3

import subprocess
import re

class RobotSpeak():
    def __init__(self):
        # self.filename = '1.mp3'
        # pygame.init()
        # self.home = str(Path.home())
        # print(self.home)
        self.engine = pyttsx3.init()
        ### RATE ###
        rate = self.engine.getProperty('rate')   # getting details of current speaking rate
        # print (rate)                        #printing current voice rate
        self.engine.setProperty('rate', 120)     # setting up new voice rate


    def speak(self, speech: RobotSpeech):

        if speech.language == 'pt':
            lang = speech.language
            print("Language: Portuguese")
        elif speech.language == 'en':
            lang = speech.language
            print("Language: English")
        else:
            lang = 'en'
            print("Language: Other (Default: English)")

        self.engine.say(speech.command)
        self.engine.runAndWait()
        # print("DONE")


        # tts = gTTS(text=str(speech.command), lang=lang)
        # tts.save(self.home+'/'+self.filename)
        # pygame.mixer.music.load(self.home+'/'+self.filename)
        # pygame.mixer.music.play()
        # while pygame.mixer.music.get_busy():
        #     pass
        
    
    def get_active_speaker_info(self):
        try:
            output = subprocess.check_output(["pacmd", "list-sinks"]).decode()

            active_port_match = re.search(r'active port:.*?<(.*?)>', output)
            if active_port_match:
                active_port = active_port_match.group(1).strip().lower()

                print(f"Active Port: {active_port}")

                if "headphones" in active_port:
                    return "External"
                elif "speaker" in active_port:
                    return "Internal"

            return "Unknown"

        except (subprocess.CalledProcessError, FileNotFoundError):
            return "Unknown"




class SpeakerNode(Node):

    def __init__(self):
        super().__init__("Speaker")
        self.get_logger().info("Initialised CHARMIE Offline Speaker Node")
        self.charmie_speech = RobotSpeak()

        self.speaker_command_subscriber = self.create_subscription(RobotSpeech, "speech_command", self.speaker_command_callback, 10)
        self.flag_speech_done_publisher = self.create_publisher(Bool, "flag_speech_done", 10)

        self.speakers_offline_diagnostic_publisher = self.create_publisher(Bool, "speakers_offline_diagnostic", 10)
        flag_diagn = Bool()
        flag_diagn.data = True

        
        
        print("\nOutput Sound Devices:")

        active_speaker_type = self.charmie_speech.get_active_speaker_info()

        if active_speaker_type != "Unknown":
            print(f"The active speaker is: {active_speaker_type}")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")
            flag_diagn.data = True
        else:
            print("Unable to determine the active speaker.")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")
            flag_diagn.data = False

        self.speakers_offline_diagnostic_publisher.publish(flag_diagn)

        #self.print_speaker_names()

        
        
    def speaker_command_callback(self, speech: RobotSpeech):

        print("\nReceived String:", speech.command)
        self.get_logger().info("Received Speech String")
        self.charmie_speech.speak(speech)
        flag = Bool()
        flag.data = True
        self.flag_speech_done_publisher.publish(flag)
        print("Finished Speaking.")
        self.get_logger().info("Finished Speaking")

    """ def print_speaker_names(self):
        devices = sd.query_devices()
        speakers = [device['name'] for device in devices if 'output' in device['name'].lower()]

        if not speakers:
            print("No speakers found.")
        else:
            print("Available speakers:")
            for idx, speaker in enumerate(speakers):
                print(f"{idx + 1}. {speaker}") """



def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
