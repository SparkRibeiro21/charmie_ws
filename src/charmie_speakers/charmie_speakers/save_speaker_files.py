#!/usr/bin/env python3
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import json
import time
import pygame
from pathlib import Path

# MODE can be the following commands:
# "STANDARD": convert save_speaker command into wav(speaker) and txt(show face) 
# "NAMES": reads names from json file and exports all names to list_of_sentences/person_names
# "OBJECTS": reads objects and objects_classe from json file and exports all objects to list_of_sentences/objects_names and objects_classes to list_of_sentences/objects_classes 
# "HOUSE": reads rooms and furniture from json file and exports all rooms to list_of_sentences/rooms and furniture to list_of_sentences/furniture 
# "COMPETITION": to improve time-efficiency in competitions, this mode does NAMES, OBJECTS and HOUSE modes 

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
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        midpath_list_of_sentences = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
        self.complete_path_list_of_sentences = self.home+'/'+midpath_list_of_sentences+'/'

        # info regarding the paths for the recorded files intended to be played
        midpath_configuration_save_speaker = "charmie_ws/src/configuration_files/save_speaker"
        self.complete_path_configuration_save_speaker = self.home+'/'+midpath_configuration_save_speaker+'/'

        # info regarding the paths for the names, objects, objects_class, furniture, rooms to create speaker files
        midpath_configuration_files = "charmie_ws/src/configuration_files"
        self.complete_path_configuration_files = self.home+'/'+midpath_configuration_files+'/'

        # Open the save_speaker configuration files
        try:
            with open(self.complete_path_configuration_save_speaker + 'save_speaker_files.json', encoding='utf-8') as json_file:
                self.json_commands = json.load(json_file)
                print(self.json_commands)
        except:
            print("")
            print("ERROR: Could NOT import data from json configuration files. (save_speaker_files)")
            print("Please check if you have already created save_speaker_files.json in charmie_ws/src/configuration_files/save_speaker")
            while True:
                pass

        # Opens files with objects, objects_classes, rooms, furniture, names
        try:
            with open(self.complete_path_configuration_files + 'objects.json', encoding='utf-8') as json_file:
                self.objects_file = json.load(json_file)
            # print(self.objects_file)
            with open(self.complete_path_configuration_files + 'objects_classes.json', encoding='utf-8') as json_file:
                self.objects_classes_file = json.load(json_file)
            # print(self.objects_classes_file)
            with open(self.complete_path_configuration_files + 'rooms.json', encoding='utf-8') as json_file:
                self.rooms_file = json.load(json_file)
            # print(self.rooms_file)
            with open(self.complete_path_configuration_files + 'furniture.json', encoding='utf-8') as json_file:
                self.furniture_file = json.load(json_file)
            # print(self.furniture_file)
            with open(self.complete_path_configuration_files + 'names.json', encoding='utf-8') as json_file:
                self.names_file = json.load(json_file)
            # print(self.names_file)
            print("Successfully Imported data from json configuration files. (objects, objects_classes, rooms, furniture, names)")
        except:
            print("Could NOT import data from json configuration files. (objects, objects_classes, rooms, furniture, names)")
        
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
            f = open(self.complete_path_list_of_sentences+filename+".txt", "w")
            f.write(command)
            f.close()

            # create wav file for speakers package 
            print("Initialised synthetisation.")
            init_time = time.time()
            outputs = self.syn.tts(command)
            self.syn.save_wav(outputs, self.complete_path_list_of_sentences+self.filename)
            print(time.time()-init_time)

            # play wav file in speakers
            if play_sound:
                pygame.mixer.music.load(self.complete_path_list_of_sentences+self.filename)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    pass

    def generate_speaker_files_from_configuration_files(self, folder, config_file):
        
        commands = {}
        for command in config_file:
            message = command['name']
            filename = folder+'/'+message.replace(" ","_").lower()
            
            full_path = Path.home() / (self.complete_path_list_of_sentences+filename+".wav")
            if not full_path.is_file():
                commands[filename] = message
                print(message, "- NEW FILE!")
            else:
                print(message)

        print(commands)
        self.convert_command(commands, play_sound=True)


def main(args=None):

    charmie_speech = RobotSpeak()

    if MODE == "STANDARD": # from save_speaker_files
        charmie_speech.convert_command(commands=charmie_speech.json_commands, play_sound=True)
    
    elif MODE == "NAMES": # people names
        charmie_speech.generate_speaker_files_from_configuration_files(folder="person_names", config_file=charmie_speech.names_file)

    elif MODE == "OBJECTS": # object names and classes
        charmie_speech.generate_speaker_files_from_configuration_files(folder="objects_names", config_file=charmie_speech.objects_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="objects_classes", config_file=charmie_speech.objects_classes_file)

    elif MODE == "HOUSE": # furniture and rooms
        charmie_speech.generate_speaker_files_from_configuration_files(folder="rooms", config_file=charmie_speech.rooms_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="furniture", config_file=charmie_speech.furniture_file)
    
    elif MODE == "COMPETITION": # furniture and rooms
        charmie_speech.generate_speaker_files_from_configuration_files(folder="person_names", config_file=charmie_speech.names_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="objects_names", config_file=charmie_speech.objects_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="objects_classes", config_file=charmie_speech.objects_classes_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="rooms", config_file=charmie_speech.rooms_file)
        charmie_speech.generate_speaker_files_from_configuration_files(folder="furniture", config_file=charmie_speech.furniture_file)
