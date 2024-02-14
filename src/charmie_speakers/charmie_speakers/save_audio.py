#!/usr/bin/env python3
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
import pygame
from pathlib import Path

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
        # starts pygame, library that reproduces audio files
        pygame.init()      

        # automatically sets the computer speakers to 100% of the volume.
        # it does not work with over amplification. 
        # So in loud environments remove the following line and manually set the volume to max
        pygame.mixer.music.set_volume(1.0) 

        # info regarding the paths for the recorded files intended to be played
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # TTS synthetiser models path 
        self.voice_models_path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.voice_models_path)

        # good quality but slow render voice
        # self.model_name = "jenny"
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

        # play wav file in speakers
        if play_sound:
            pygame.mixer.music.load(complete_path+self.filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

def main(args=None):
    
    charmie_speech = RobotSpeak()
    
    global COMMAND, FILENAME

    # select which mode you want to use to save audios:
    # STANDARD is used for all alone sentences. Just edit COMMAND and FILENAME at the top of the code
    # RECEPTIONIST is used for receptionist task. Just edit names_list and drinks_list
    if MODE == "STANDARD":
        charmie_speech.convert_command(play_sound=True)
    elif MODE == "RECEPTIONIST":
        for name in names_list:
            COMMAND = "The first guest name is "+name+"."
            FILENAME = "recep_first_guest_"+name.lower()
            print(COMMAND, FILENAME)
            charmie_speech.convert_command(play_sound=True)
            COMMAND = "The second guest name is "+name+"."
            FILENAME = "recep_second_guest_"+name.lower()
            print(COMMAND, FILENAME)
            charmie_speech.convert_command(play_sound=True)
        for drink in drinks_list:
            COMMAND = "The favourite drink is "+drink+"."
            FILENAME = "recep_drink_"+drink.lower().replace(" ", "_")
            print(COMMAND, FILENAME)
            charmie_speech.convert_command(play_sound=True)
