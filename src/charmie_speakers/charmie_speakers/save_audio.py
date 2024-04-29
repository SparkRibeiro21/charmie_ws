#!/usr/bin/env python3
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
import pygame
from pathlib import Path

names_list = ["Adel", "Angel", "Axel", "Charlie", "Jane", "John", "Jules", "Morgan", "Paris", "Robin", "Simone"]
drinks_list = ["Red Wine", "Juice Pack", "Cola", "Tropical Juice", "Milk", "Iced Tea", "Orange Juice", "7up", "Water"] # the 7up is weird, must be redone manually


# MODE can be the following commands:
# "STANDARD": convert one command into wav and txt 
# "RECEPTIONIST": reads names and drinks arrays and generates all commands for first guest names, second guest names and favourite drinks
MODE = "STANDARD"


COMMANDS = {
    'waiting_door_bell' : 'Waiting for a new guest to ring the door bell.',
    'cornflakes_poured' : 'Cornflakes poured.',
    'milk_poured' : 'Milk poured.',
}

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

"""
COMMANDS = {
    'start_sftr':'I am ready to start the Stickler for the rules task!',
    'go_forbidden_room':'I will start by checking if there is someone breaking the forbidden room rule.',
    'start_searching':'Start tracking.',
    'no_detection_forbidden_room':'I did not detect anyone breaking the forbidden room rule.',
    'detection_forbidden_room':'I detected a guest breaking the forbidden room rule.',
    'looking_guest_forbidden_room':'I am looking at the detected guest breaking the forbidden room rule.',
    'guest_breaking_rule_forbidden_room':'Hello there guest. You are breaking the forbidden room rule.',
    'action_forbidden_room':'To stop breaking this rule, you must get out of the office and join the other guests in the other rooms of the house.',
    'follow_robot_outside_room':'I will turn around for you to follow me. Dont try to trick me.',
    'dont_try_to_trick_me':'Follow me! Dont try to trick me.',
    'no_longer_breaking_rule':'You are no longer breaking the rule. Keep enjoying the party without breaking any rules.',
    'finish_sftr':'Thank you. I have finished my stickler for the rules task.'
}
"""


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
            receptionist_commands['receptionist/recep_host_'+name.lower()] = 'The host name is '+name+'.'
            receptionist_commands['receptionist/recep_first_guest_'+name.lower()] = 'The first guest name is '+name+'.'
            receptionist_commands['receptionist/recep_second_guest_'+name.lower()] = 'The second guest name is '+name+'.'
        for drink in drinks_list:
            receptionist_commands['receptionist/recep_drink_'+drink.lower().replace(' ', '_')] = 'The favourite drink is '+drink+'.'
            
        charmie_speech.convert_command(receptionist_commands, play_sound=True)
        