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

# MODE can be the following commands:
# "STANDARD": convert one command into wav and txt 
# "RECEPTIONIST": reads names and drinks arrays and generates all commands for first guest names, second guest names and favourite drinks


names_list = ["Adam", "Paris", "William"]
drinks_list = ["milk", "orange juice", "red wine"]

MODE = "RECEPTIONIST"

COMMAND = "Leia."
FILENAME = "leia"


class RobotSpeak():
    def __init__(self):
        pygame.init()
        pygame.mixer.music.set_volume(1.0)

        self.path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.path)


        self.model_name = "jenny"
        mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/jenny/jenny") # LENTO 8/10
        self.syn = Synthesizer(
            tts_checkpoint= mode_path,
            tts_config_path= config_path,
        )

    def convert_command(self, play_sound):

        self.filename = FILENAME+".wav"

        # create txt file with command for face package 
        f = open(complete_path+FILENAME+".txt", "w")
        f.write(COMMAND)
        f.close()
        
        # create wav file for speakers package 
        print("Initialised synthetisation.")
        init_time = time.time()
        outputs = self.syn.tts(COMMAND)
        self.syn.save_wav(outputs, complete_path+self.filename)
        print(time.time()-init_time)

        if play_sound:
            # play wav file in speakers
            pygame.mixer.music.load(complete_path+self.filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

def main(args=None):
    
    charmie_speech = RobotSpeak()
    
    global COMMAND, FILENAME

    if MODE == "STANDARD":
        charmie_speech.convert_command(play_sound=True)
    elif MODE == "RECEPTIONIST":
        for name in names_list:
            pass
            # COMMAND = "The first guest name is "+name+"."
            # FILENAME = "recep_first_guest_"+name.lower()
            # print(COMMAND, FILENAME)
            # charmie_speech.convert_command(play_sound=True)
            # COMMAND = "The second guest name is "+name+"."
            # FILENAME = "recep_second_guest_"+name.lower()
            # print(COMMAND, FILENAME)
            # charmie_speech.convert_command(play_sound=True)
        for drink in drinks_list:
            COMMAND = "The favourite drink is "+drink+"."
            FILENAME = "recep_drink_"+drink.lower().replace(" ", "_")
            print(COMMAND, FILENAME)
            charmie_speech.convert_command(play_sound=True)


     

    #  while True:
   #      pass

    # pygame.mixer.music.load(complete_path+"ready_serve_breakfast.wav")
    # pygame.mixer.music.play()
    # while pygame.mixer.music.get_busy():
    #     pass

    # pygame.mixer.music.load(complete_path+"music_pre_start.wav")
    # pygame.mixer.music.play()
    # while pygame.mixer.music.get_busy():
    #     pass


    #open and read the file after the overwriting:
    # f2 = open(complete_path+FILENAME+".txt", "r")
    # print(f2.read())


    # pygame.mixer.music.load(complete_path+"renata_teste.wav")
    # pygame.mixer.music.play()
    # while pygame.mixer.music.get_busy():
    #     pass
