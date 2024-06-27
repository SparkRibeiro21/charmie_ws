#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand

from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
import pygame
from pathlib import Path
import os

import subprocess
import re

class RobotSpeak():
    def __init__(self, node):
        # imports the main speakers node to publish face info
        self.node = node

        # starts pygame, library that reproduces audio files
        pygame.init()

        # automatically sets the computer speakers to 100% of the volume.
        # it does not work with over amplification. 
        # So in loud environments remove the following line and manually set the volume to max
        pygame.mixer.music.set_volume(1.0) 

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # TTS synthetiser models path 
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.voice_models_path = self.home+"/.local/lib/python3.10/site-packages/TTS/.models.json"
        self.model_manager = ModelManager(self.voice_models_path)

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


    # function for pre recorded commands 
    def play_command(self, filename, show_in_face):
                
        # if there is an audio
        if os.path.isfile(self.complete_path+filename+".wav"):

        
            if show_in_face:
                # if there is a txt file corresponding to the audio
                if os.path.isfile(self.complete_path+filename+".txt"):

                    string_from_file = open(self.complete_path+filename+".txt", "r")

                    # send string to face to ease UI
                    str = String()
                    str.data = string_from_file.read()
                    self.node.speech_to_face_publisher.publish(str)

                    message = "Text Sent to Face Node"
                    # print("File sent to face! - '", str.data, "'")
                else:
                    message = "Text File not Found. NOT sent to face."
                    # print("File not sent to face!")


            # play the recorded file 
            pygame.mixer.music.load(self.complete_path+filename+".wav")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

            if show_in_face:
                # sends empty string to tell face that the audio has finished to be played
                str = String()
                str.data = ""
                # print(str.data)
                # print("File sent to face! - End of Sentence")
                self.node.speech_to_face_publisher.publish(str)

            success = True
            message = ""

        else:
            success = False
            message = "File does not exist!!!"

        # retrieves the success and message to be returned by the server, so the requester has info on what happened
        return success, message


    # function for commands to be created in the moment 
    def load_and_play_command(self, command, jenny_or_taco, show_in_face):
        
        temp_filename = "temp/temp.wav"

        if jenny_or_taco: # tacotron synthesizer

            # creates the audio file from the command received
            init_time = time.time()
            outputs = self.syn_taco.tts(command)
            self.syn_taco.save_wav(outputs, self.complete_path+temp_filename)
            print(time.time()-init_time)

            if show_in_face:
                # send string to face to ease UI
                str = String()
                str.data = command
                # print(str.data)
                self.node.speech_to_face_publisher.publish(str)

            # plays the created audio file
            pygame.mixer.music.load(self.complete_path+temp_filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

            if show_in_face:
                # sends empty string to tell face that the audio has finished to be played
                str = String()
                str.data = ""
                # print(str.data)
                self.node.speech_to_face_publisher.publish(str)
        
        else: # jenny synthesizer

            # creates the audio file from the command received
            init_time = time.time()
            outputs = self.syn_jenny.tts(command)
            self.syn_jenny.save_wav(outputs, self.complete_path+temp_filename)
            print(time.time()-init_time)

            if show_in_face:
                # send string to face to ease UI
                str = String()
                str.data = command
                # print(str.data)
                self.node.speech_to_face_publisher.publish(str)

            # plays the created audio file
            pygame.mixer.music.load(self.complete_path+temp_filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pass

            if show_in_face:
                # sends empty string to tell face that the audio has finished to be played
                str = String()
                str.data = ""
                # print(str.data)
                self.node.speech_to_face_publisher.publish(str)

        return True, ""

    # diagnostics function to know which speaker is being used by the PC - debug purposes
    def get_active_speaker_info(self):
        try:
            # checks which the speakers list available 
            output = subprocess.check_output(["pacmd", "list-sinks"]).decode()
            active_port_match = re.search(r'active port:.*?<(.*?)>', output)
            if active_port_match:
                active_port = active_port_match.group(1).strip().lower()

                # print(f"Active Port: {active_port}")

                # returns the type of speakers going to be used 
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

        # initialize robot speech class with acess to node variables
        self.charmie_speech = RobotSpeak(self)

        # TOPICS:
        # To publish the received strings to the face node
        self.speech_to_face_publisher = self.create_publisher(String, "display_speech_face", 10)    
        # Diagnostics for the speakers package
        self.speakers_diagnostic_publisher = self.create_publisher(Bool, "speakers_diagnostic", 10) 
        
        # SERVICES:
        # Main receive commads 
        self.server_speech_command = self.create_service(SpeechCommand, "speech_command", self.callback_speech_command) 
        self.save_server_speech_command = self.create_service(SaveSpeechCommand, "save_speech_command", self.callback_save_speech_command) 
        self.get_logger().info("Speech Servers have been started")

        # Get Information regarding which speakers are being used 
        # print("\nOutput Sound Devices:")
        active_speaker_type = self.charmie_speech.get_active_speaker_info()

        flag_diagn = Bool()
        # if a known system is being used 
        if active_speaker_type != "Unknown":
            # print(f"The active speaker is: {active_speaker_type}")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")
            flag_diagn.data = True
        # if an unkown system is being used
        else:
            # print("Unable to determine the active speaker.")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")
            flag_diagn.data = False

        # Sends information to diagnostics node
        self.speakers_diagnostic_publisher.publish(flag_diagn)

        # Initial Speaking "Hello" for debug purposes
        self.charmie_speech.play_command("generic/introduction_hello", False) 

        # Test Function for some quick tests if necessary
        # self.test()


    # Test Function for some quick tests if necessary
    def test(self):
        self.charmie_speech.load_and_play_command(True, "What is your name and favourite drink?")


    # Main Function regarding received commands
    def callback_speech_command(self, request, response):
        # print("Received request")

        # Type of service received: 
        # string filename # name of audio file to be played
        # string command  # if there is no filename, a command string can be sent to be played in real time 
        # bool quick_voice # if you do not want to use the pretty voice that takes more time to load, raising this flag uses the secondary quick voice
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        # if filename comes empty it is automatically assumed that it is intended to use the load and play mode
        print("show_in_face:", request.show_in_face)


        if request.filename == "":
            # speakers mode where received string must be synthesized and played now
            self.get_logger().info("SPEAKERS received (custom) - %s" %request.command)
            success, message = self.charmie_speech.load_and_play_command(request.command, request.quick_voice, request.show_in_face)
        
        else:
            # speakers mode where received filename must be played
            success, message = self.charmie_speech.play_command(request.filename, request.show_in_face)
            if success == False:
                self.get_logger().error("SPEAKERS received (file) does not exist! - %s" %request.filename)
            else:
                self.get_logger().info("SPEAKERS received (file) - %s" %request.filename)

        # returns whether the message was played and some informations regarding status
        response.success = success
        response.message = message
        return response

    # Main Function regarding saving commands
    def callback_save_speech_command(self, request, response):
        # print("Received request")

        # Type of service received: 
        # string[] filename   # name of audio file
        # string[] command    # speech command string 
        # bool quick_voice  # if you do not want to use the pretty voice that takes more time to load, raising this flag uses the secondary quick voice
        # bool play_command   # if you want to play the sound right after saving it 
        # bool show_in_face   # whether or not it is intended for the speech command to be shown in the face
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        commands = {}
        for i in range(len(request.filename)):
            commands[request.filename[i]] = request.command[i]

        for filename, command in commands.items():

            filename = "temp/"+filename
            self.filename = filename+".wav"

            # create txt file with command for face package 
            f = open(self.charmie_speech.complete_path+filename+".txt", "w")
            f.write(command)
            f.close()
            
            # create wav file for speakers package 
            print("Initialised synthetisation.")
            init_time = time.time()

            if request.quick_voice:
                outputs = self.charmie_speech.syn_taco.tts(command)
                self.charmie_speech.syn_taco.save_wav(outputs, self.charmie_speech.complete_path+self.filename)
                print(time.time()-init_time)
            else:
                init_time = time.time()
                outputs = self.charmie_speech.syn_jenny.tts(command)
                self.charmie_speech.syn_jenny.save_wav(outputs, self.charmie_speech.complete_path+self.filename)
                print(time.time()-init_time)

            if request.play_command:
                ### ADICIONAR QUE TAMBEM FOI PLAYED
                self.charmie_speech.play_command(filename, request.show_in_face) 
                response.success = True
                response.message = str(len(request.filename))+" new speech files saved and sound played"
            else:
                response.success = True
                response.message = str(len(request.filename))+" new speech files saved"

        return response


# Standard ROS2 main function
def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
