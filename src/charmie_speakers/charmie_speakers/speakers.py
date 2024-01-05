#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from charmie_interfaces.msg import RobotSpeech

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
        self.path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.path)
        pygame.init()


        if VOICE_NO == 1: # jenny
            self.model_name = "jenny"
            mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/jenny/jenny") # LENTO 8/10
            # voc_path, voc_config_path, _ = self.model_manager.download_model(model_item["default_vocoder"])
            self.syn = Synthesizer(
                tts_checkpoint= mode_path,
                tts_config_path= config_path,
                # vocoder_checkpoint= voc_path,
                # vocoder_config= voc_config_path
            )
        elif VOICE_NO == 2: # tacotron2-DDC_ph
            self.model_name = "tacotron2-DDC_ph"
            mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/ljspeech/tacotron2-DDC_ph") # 6/10
            voc_path, voc_config_path, _ = self.model_manager.download_model(model_item["default_vocoder"])
            self.syn = Synthesizer(
                tts_checkpoint= mode_path,
                tts_config_path= config_path,
                vocoder_checkpoint= voc_path,
                vocoder_config= voc_config_path
            )
           
        else: # VOICE_NO == 3: # overflow
            self.model_name = "overflow"
            mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/ljspeech/overflow") # 5/10
            voc_path, voc_config_path, _ = self.model_manager.download_model(model_item["default_vocoder"])
            self.syn = Synthesizer(
                tts_checkpoint= mode_path,
                tts_config_path= config_path,
                vocoder_checkpoint= voc_path,
                vocoder_config= voc_config_path
            )

        filename = "last_speaked.wav"
        home = str(Path.home())
        midpath = "charmie_ws/src/charmie_speakers/charmie_speakers"
        self.complete_path = home+'/'+midpath+'/'+filename


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

        init_time = time.time()
        outputs = self.syn.tts(speech.command)
        self.syn.save_wav(outputs, self.complete_path)
        print(time.time()-init_time)
        pygame.mixer.music.load(self.complete_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pass


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
        self.get_logger().info("Initialised CHARMIE Speaker Node")

        self.charmie_speech = RobotSpeak()

        self.speaker_command_subscriber = self.create_subscription(RobotSpeech, "speech_command", self.speaker_command_callback, 10)
        self.flag_speech_done_publisher = self.create_publisher(Bool, "flag_speech_done", 10)

        self.speakers_diagnostic_publisher = self.create_publisher(Bool, "speakers_diagnostic", 10)

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

        self.speakers_diagnostic_publisher.publish(flag_diagn)
        
    def speaker_command_callback(self, speech: RobotSpeech):

        print("\nReceived String:", speech.command)
        self.get_logger().info("Received Speech String")
        self.charmie_speech.speak(speech)
        flag = Bool()
        flag.data = True
        self.flag_speech_done_publisher.publish(flag)
        print("Finished Speaking.")
        self.get_logger().info("Finished Speaking")


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
