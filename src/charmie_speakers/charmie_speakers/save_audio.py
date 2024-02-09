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

home = str(Path.home())
midpath = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
complete_path = home+'/'+midpath+'/'

# COMMAND = "Hello, my name is charmie."
# FILENAME = "introduction"

# COMMAND = "Waiting for start button to be pressed."
# FILENAME = "waiting_start_button"

# COMMAND = "Waiting for door to be opened."
# FILENAME = "waiting_door_open"

# COMMAND = "I am ready to start the serve the breakfast task."
# FILENAME = "ready_serve_breakfast"

# COMMAND = "The host name is Paris and the favourite drink is milk."
# FILENAME = "receptionist_host_info"

COMMAND = "Please stay on my left until I give you instructions on where to sit."
FILENAME = "receptionist_stay_left"

class RobotSpeak():
    def __init__(self):
        self.path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.path)


        self.model_name = "jenny"
        mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/jenny/jenny") # LENTO 8/10
        self.syn = Synthesizer(
            tts_checkpoint= mode_path,
            tts_config_path= config_path,
        )
        
        self.filename = FILENAME+".wav"


        # this is just a test, it uses sentences pre recorded so that the robot does not waste any time processing what it wants to say.
        # if speech.command == "Please say your order.":
        #     
        #     pygame.mixer.music.load(self.complete_path+"voice1.wav")
        #     pygame.mixer.music.play()
        #     while pygame.mixer.music.get_busy():
        #         pass
        # else:
        #     init_time = time.time()
        #     outputs = self.syn.tts(speech.command)
        #     self.syn.save_wav(outputs, self.complete_path+self.filename)
        #     print(time.time()-init_time)
        #     pygame.mixer.music.load(self.complete_path+self.filename)
        #     pygame.mixer.music.play()
        #     while pygame.mixer.music.get_busy():
        #         pass
        
        print("Initialised synthetisation.")
        
        init_time = time.time()
        outputs = self.syn.tts(COMMAND)
        self.syn.save_wav(outputs, complete_path+self.filename)
        print(time.time()-init_time)
        pygame.mixer.music.load(complete_path+self.filename)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pass

def main(args=None):
    
    pygame.init()
    
    
    # charmie_speech = RobotSpeak()



    pygame.mixer.music.load(complete_path+"music_soy_el_fuego.wav")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pass