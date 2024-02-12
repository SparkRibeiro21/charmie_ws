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
import os

import subprocess
import re


home = str(Path.home())
midpath = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
complete_path = home+'/'+midpath+'/'


class RobotSpeak():
    def __init__(self):
        self.voice_models_path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.voice_models_path)
        pygame.init()

        # automatically sets the computer speakers to 100% of the volume.
        # it does not work with over amplification. 
        # So in loud environments remove the following line and manually set the volume to max
        pygame.mixer.music.set_volume(1.0) 


        # good quality but slow render voice
        # self.model_name = "jenny"
        mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/jenny/jenny")
        self.syn_jenny = Synthesizer(
            tts_checkpoint= mode_path,
            tts_config_path= config_path
        )

        # worse quality but quick render voice
        # self.model_name = "tacotron2-DDC_ph"
        mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/ljspeech/tacotron2-DDC_ph")
        voc_path, voc_config_path, _ = self.model_manager.download_model(model_item["default_vocoder"])
        self.syn_taco = Synthesizer(
            tts_checkpoint= mode_path,
            tts_config_path= config_path,
            vocoder_checkpoint= voc_path,
            vocoder_config= voc_config_path
        )


    def play_command(self, filename):
        
        # missing sending commands to face.............................................................................................
        
        if os.path.isfile(complete_path+filename+".txt"):
            print("File sent to face!")
        else:
            print("File not sent to face!")


        if os.path.isfile(complete_path+filename+".wav"):
            pygame.mixer.music.load(complete_path+filename+".wav")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass
            success = True
            message = ""

        else:
            success = False
            message = "File does not exist!!!"

        return success, message


    def load_and_play_command(self, jenny_or_taco, command):
        
        temp_filename = "temp.wav"

        if jenny_or_taco: # tacotron synthetiser
            init_time = time.time()
            outputs = self.syn_taco.tts(command)
            self.syn_taco.save_wav(outputs, complete_path+temp_filename)
            print(time.time()-init_time)
            pygame.mixer.music.load(complete_path+temp_filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass
        else: # jenny synthetiser
            init_time = time.time()
            outputs = self.syn_jenny.tts(command)
            self.syn_jenny.save_wav(outputs, complete_path+temp_filename)
            print(time.time()-init_time)
            pygame.mixer.music.load(complete_path+temp_filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

        return True, ""


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

        # initialize models loading 
        self.charmie_speech = RobotSpeak()


        self.server_speech_command = self.create_service(SpeechCommand, "speech_command", self.callback_speech_command) 
        self.get_logger().info("Speech Command Server has been started")



        # Diagnostics for the speakers package
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

        # print("Test mode")
        # time.sleep(2)
        # self.test()


    # def test(self):
    #     self.charmie_speech.speak_command(False, "Hello, my name is charmie. How can I help you?")


    def callback_speech_command(self, request, response):
        print("Received request")

        # string filename # name of audio file to be played
        # string command  # if there is no filename, a command string can be sent to be played in real time 
        # bool quick_voice # if you do not want to use the pretty voice that takes more time to load, raising this flag uses the secondary quick voice
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        if request.filename == "":
            # speakers mode where received string must be synthetised and played now
            success, message = self.charmie_speech.load_and_play_command(request.quick_voice, request.command)
        
        else:
            # speakers mode where received filename must be played
            success, message = self.charmie_speech.play_command(request.filename)

        response.success = success
        response.message = message
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
