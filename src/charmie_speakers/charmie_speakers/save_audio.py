#!/usr/bin/env python3
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
import pygame
from pathlib import Path

names_list = ["Axel", "John", "Paris", "Robin", "Simone"]
drinks_list = ["Red Wine", "Juice Pack", "Cola", "Tropical Juice", "Milk", "Iced Tea", "Orange Juice", "7up", "Water"] # the 7up is weird, must be redone manually


# MODE can be the following commands:
# "STANDARD": convert one command into wav and txt 
# "RECEPTIONIST": reads names and drinks arrays and generates all commands for first guest names, second guest names and favourite drinks
MODE = "STANDARD"


#COMMANDS = {
#    'waiting_start_button': 'Waiting for start button to be pressed.',
#    'waiting_door_open': 'Waiting for door to be opened.',
#    'recep_start_task': 'I am ready to start my receptionist task'
#}

"""
COMMANDS = {
    'sponge': 'a sponge.',
    'cleanser': 'a cleanser.',
    'dishwasher_tab': 'a dishwasher tab.',
    'bag': 'a bag.',
    'red_wine': 'a red wine.',
    'juice_pack': 'a juice pack.',
    'cola': 'a cola.',
    'tropical_juice': 'a tropical juice.',
    'milk': 'a milk.',
    'iced_tea': 'an iced tea.',
    'orange_juice': 'an orange juice.',
    '7up': 'a seven up.',
    'water': 'a water.',
    'tuna': 'a tuna.',
    'tomato_soup': 'a tomato soup.',
    'spam': 'a spam.',
    'mustard': 'a mustard.',
    'strawberry_jello': 'a strawberry jello.',
    'chocolate_jello': 'a chocolate jello.',
    'coffee_grounds': 'coffee grounds.',
    'sugar': 'a sugar.',
    'pear': 'a pear.',
    'plum': 'a plum.',
    'peach': 'a peach.',
    'lemon': 'a lemon.',
    'orange': 'an orange.',
    'strawberry': 'a strawberry.',
    'banana': 'a banana.',
    'apple': 'an apple.',
    'tennis_ball': 'a tennis ball.',
    'soccer_ball': 'a soccer ball.',
    'rubiks_cube': 'a rubiks cube.',
    'dice': 'a dice.',
    'baseball': 'a baseball.',
    'pringles': 'pringles.',
    'cornflakes': 'cornflakes.',
    'cheezit': 'cheezit.',
    'spoon': 'a spoon.',
    'plate': 'a plate.',
    'cup': 'a cup.',
    'fork': 'a fork.',
    'bowl': 'a bowl.',
    'knife': 'a knife.'
}
"""

# COMMANDS = {
#     'problem_detecting_change_object': 'There seems to be a problem with detecting the objects. Can you please slightly move and rotate the following objects?',
# }

COMMANDS = {
    'finish_receptionist': 'Thank you. I have finished my receptionist task.',
    'race_caucasian': 'Caucasian.',
    'race_asian': 'Asian.',
    'race_black': 'African descendant.',
    'race_middleeastern': 'Middle Eastern.',
    'race_indian': 'Indian.',
    'race_hispanic': 'Hispanic.',
    'gender_male':'Male.',
    'gender_female': 'Female.',
    'height_taller': 'Taller than me.',
    'height_smaller': 'Shorter than me.',
    'height_equal': 'Approximately the same height as me.',
    'under_20':'Under 20 years old.',
    'between18_32':'Between 18 and 32 years old.',
    'between28_42':'Between 28 and 40 years old.',
    'between40_60':'Between 40 and 60 years old.',
    'over60':'Over 60 years old.',
    'please_follow_me':'Thank you. Please follow me.',
    'please_stay_on_my_left':'Please stay on my left until I give you instructions on where to sit.',
    'please_stay_on_my_right':'Please stay on my right until I give you instructions on where to sit.',
    'present_everyone':'Hello, I will present everyone in this room.',
    'please_sit_sofa':'Please take a sit on the sofa that I am looking at.',
    'please_sit_place':'Please take a sit on the place that I am looking at.',
    'please_sit_chair':'Please take a sit on the chair that I am looking at.',
    'dear_host':'Dear host.',
    'dear_guest':'Dear guest.',
    'ready_receive_guest':'I am ready to receive a new guest. Please stand in front of me.',
    'presentation_answer_after_green_face':'Hello! My name is Charmie. I will make you some questions. Please speak loud and clear. Answer me after the green light on my face.'
}


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
    
    global COMMANDS

    # select which mode you want to use to save audios:
    # STANDARD is used for all alone sentences. Just edit COMMAND and FILENAME at the top of the code
    # RECEPTIONIST is used for receptionist task. Just edit names_list and drinks_list
    if MODE == "STANDARD":
        charmie_speech.convert_command(COMMANDS, play_sound=True)
    elif MODE == "RECEPTIONIST":
        receptionist_commands = {}
        for name in names_list:
            receptionist_commands['receptionist/recep_first_guest_'+name.lower()] = 'The first guest name is '+name+'.'
            receptionist_commands['receptionist/recep_second_guest_'+name.lower()] = 'The second guest name is '+name+'.'
        for drink in drinks_list:
            receptionist_commands['receptionist/recep_drink_'+drink.lower().replace(' ', '_')] = 'The favourite drink is '+drink+'.'
            
        charmie_speech.convert_command(receptionist_commands, play_sound=True)
        