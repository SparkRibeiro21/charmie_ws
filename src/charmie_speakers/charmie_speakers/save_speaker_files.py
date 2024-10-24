#!/usr/bin/env python3
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import json
import time
import pygame
from pathlib import Path

# names_list = ["Adel", "Angel", "Axel", "Charlie", "Jane", "John", "Jules", "Morgan", "Paris", "Robin", "Simone"]
#names_list = ["Kai", "Noah", "Unique", "Luca", "Evelyn", "Jayden", "James", "Riley", "Gabrielle", "Robin", "Avery", "Remi", "River", "Atlas"]
names_list = ["Sophie", "Julia", "Emma", "Sara", "Laura", "Hayley", "Susan", "Fleur", "Gabrielle", "Robin", "John", "Liam", "William", "Kevin", "Jesse", "Noah", "Harrie", "Peter"]
#drinks_list = ["Red Wine", "Juice Pack", "Cola", "Tropical Juice", "Milk", "Iced Tea", "Orange Juice", "7up", "Water"] # the 7up is weird, must be redone manually
drinks_list = ["Big Coke", "Cola", "Dubblefris", "Milk", "Iced Tea", "Fanta", "Water"] # the 7up is weird, must be redone manually
host_name = "John"

# MODE can be the following commands:
# "STANDARD": convert one command into wav and txt 
# "RECEPTIONIST": reads names and drinks arrays and generates all commands for first guest names, second guest names and favourite drinks
MODE = "STANDARD"

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

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath_configuration = "charmie_ws/src/configuration_files"
        self.complete_path_configuration = self.home+'/'+self.midpath_configuration+'/'

        # Open all configuration files
        try:
            with open(self.complete_path_configuration + 'save_speaker_files.json', encoding='utf-8') as json_file:
                self.json_commands = json.load(json_file)
                print(self.json_commands)
        except:
            print("Could NOT import data from json configuration files. (save_speaker_files)")
        
        # TTS synthetiser models path 
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.voice_models_path = self.home+"/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.voice_models_path)

        # good quality but slow render voice
        # self.model_name = "jenny"
        mode_path, config_path, model_item = self.model_manager.download_model("tts_models/en/jenny/jenny") # LENTO 8/10
        self.syn = Synthesizer(
            tts_checkpoint= mode_path,
            tts_config_path= config_path,
        )

    def convert_command(self, commands, play_sound):

        for filename, command in commands.items():

            self.filename = filename+".wav"

            # create txt file with command for face package 
            f = open(self.complete_path+filename+".txt", "w")
            f.write(command)
            f.close()

            # create wav file for speakers package 
            print("Initialised synthetisation.")
            init_time = time.time()
            outputs = self.syn.tts(command)
            self.syn.save_wav(outputs, self.complete_path+self.filename)
            print(time.time()-init_time)

            # play wav file in speakers
            if play_sound:
                pygame.mixer.music.load(self.complete_path+self.filename)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    pass

def main(args=None):

    charmie_speech = RobotSpeak()

    # select which mode you want to use to save audios:
    # STANDARD is used for all alone sentences. Just edit src/configuration_files/save_speaker_files.json
    # RECEPTIONIST is used for receptionist task. Just edit names_list and drinks_list
    if MODE == "STANDARD":
        charmie_speech.convert_command(charmie_speech.json_commands, play_sound=True)
    elif MODE == "RECEPTIONIST":
        receptionist_commands = {}
        # for name in names_list:
        #     receptionist_commands['receptionist/names/'+name.replace(" ","_").lower()] = name
        # for drink in drinks_list:
        #     receptionist_commands['objects_names/_'+drink.lower().replace(' ', '_')] = drink+'.'
        
        charmie_speech.convert_command(receptionist_commands, play_sound=True)