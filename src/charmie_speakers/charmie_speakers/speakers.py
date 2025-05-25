#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from charmie_interfaces.srv import SpeechCommand, SaveSpeechCommand, SetTextFace

# from TTS.utils.manage import ModelManager
# from TTS.utils.synthesizer import Synthesizer

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

        # Sets speaker values to maximum according to status of over-amplification toggle (either 100% or 150%)
        # checks over-amplification settings toggle status 
        try:
            # Run gsettings to check the status of over-amplification
            result = subprocess.run(
                ["gsettings", "get", "org.gnome.desktop.sound", "allow-volume-above-100-percent"],
                capture_output=True,
                text=True
            )

            # Parse the output
            status = result.stdout.strip()
            if status == "true":
                # automatically sets the computer speakers to 150% of the volume.
                subprocess.run("pactl set-sink-mute @DEFAULT_SINK@ false; pactl set-sink-volume @DEFAULT_SINK@ 150%", shell = True, executable="/bin/bash")
                print("Over-amplification toggle is enabled! Volume set to 150%!")
            else:
                # automatically sets the computer speakers to 100% of the volume.
                subprocess.run("pactl set-sink-mute @DEFAULT_SINK@ false; pactl set-sink-volume @DEFAULT_SINK@ 100%", shell = True, executable="/bin/bash")
                print("Over-amplification toggle is disabled! Volume set to 100%!")

        except Exception as e:
            print(f"An error occurred while checking over-amplification settings toggle status: {e}")
            subprocess.run("pactl set-sink-mute @DEFAULT_SINK@ false; pactl set-sink-volume @DEFAULT_SINK@ 100%", shell = True, executable="/bin/bash")
            print("Over-amplification toggle is unknown! Volume set to 100%!")
        
        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_speakers/charmie_speakers/list_of_sentences"
        self.complete_path = self.home+'/'+self.midpath+'/'

        """
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
        """

    # function for pre recorded commands 
    def play_command(self, filename, show_in_face=False, long_pause=False, breakable_play=False, break_play=False):
                
        # if there is an audio
        if os.path.isfile(self.complete_path+filename+".wav"):

        
            if show_in_face:
                # if there is a txt file corresponding to the audio
                if os.path.isfile(self.complete_path+filename+".txt"):

                    string_from_file = open(self.complete_path+filename+".txt", "r")

                    str = SetTextFace.Request()
                    str.data = string_from_file.read()
                    self.node.speech_to_face_command.call_async(str)

                    message = "Text Sent to Face Node"
                    # print("File sent to face! - '", str.data, "'")
                else:
                    message = "Text File not Found. NOT sent to face."
                    # print("File not sent to face!")

            if break_play:
                pygame.mixer.music.stop()

            # play the recorded file 
            pygame.mixer.music.load(self.complete_path+filename+".wav")
            pygame.mixer.music.play()
            if not breakable_play:
                while pygame.mixer.music.get_busy():
                    pass

            if show_in_face:
                # sends empty string to tell face that the audio has finished to be played
                str = SetTextFace.Request()
                str.data = ""
                str.long_pause = long_pause # long pause only needs to be sent here, because here is the end of the face showing
                # print(str.data)
                # print("File sent to face! - End of Sentence")
                self.node.speech_to_face_command.call_async(str)

            success = True
            message = ""

        else:
            success = False
            message = "File does not exist!!!"

        # retrieves the success and message to be returned by the server, so the requester has info on what happened
        return success, message


    # function for commands to be created in the moment 
    def load_and_play_command(self, filename="", command="", quick_voice=False, show_in_face=False, long_pause=False, play_command=False):
        
        if filename == "":
            temp_filename = "temp/temp"
        else:
            temp_filename = "temp/"+filename

        # create txt file with command for face package
        f = open(self.complete_path+temp_filename+".txt", "w")
        f.write(command)
        f.close()
            
        # create wav file for speakers package 
        print("Initialised synthetisation.")
        init_time = time.time()

        if quick_voice: 
            outputs = self.syn_taco.tts(command)
            self.syn_taco.save_wav(outputs, self.complete_path+temp_filename+".wav")
        else:
            outputs = self.syn_jenny.tts(command)
            self.syn_jenny.save_wav(outputs, self.complete_path+temp_filename+".wav")
            
        print(time.time()-init_time)

        if play_command:
            self.play_command(filename=temp_filename, show_in_face=show_in_face, long_pause=long_pause) 


    # function to know which speaker is being used by the PC - debug purposes
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

        # SERVICES:
        # Main receive commads 
        self.server_speech_command = self.create_service(SpeechCommand, "speech_command", self.callback_speech_command) 
        self.save_server_speech_command = self.create_service(SaveSpeechCommand, "save_speech_command", self.callback_save_speech_command) 
        # To publish the received strings to the face node
        self.speech_to_face_command = self.create_client(SetTextFace, "display_speech_face")
        self.get_logger().info("Speech Servers have been started")

        # Get Information regarding which speakers are being used 
        # print("\nOutput Sound Devices:")
        active_speaker_type = self.charmie_speech.get_active_speaker_info()

        # if a known system is being used 
        if active_speaker_type != "Unknown":
            # print(f"The active speaker is: {active_speaker_type}")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")
        # if an unkown system is being used
        else:
            # print("Unable to determine the active speaker.")
            self.get_logger().info(f"The active speaker is: {active_speaker_type}")

        # Initial Speaking "Hello" for debug purposes
        self.charmie_speech.play_command(filename="generic/introduction_hello") 

        # Test Function for some quick tests if necessary
        # self.test()


    # Test Function for some quick tests if necessary
    # def test(self):
    #     self.charmie_speech.load_and_play_command(command="What is your friend name and favourite drink?", quick_voice=False, show_in_face=False, play_command=True)


    # Main Function regarding received commands
    def callback_speech_command(self, request, response):
        # print("Received request")

        # Type of service received: 
        # string filename     # name of audio file to be played
        # string command      # if there is no filename, a command string can be sent to be played in real time 
        # bool quick_voice    # if you do not want to use the pretty voice that takes more time to load, raising this flag uses the secondary quick voice
        # bool show_in_face   # whether or not it is intended for the speech command to be shown in the face
        # bool long_pause_show_in_face    # whether after showing in face, a long or a short pause should be added, for user easier reading
        # bool breakable_play # if this command is inteded to be stopped by a following command
        # bool break_play     # if a command is already playing, it stopps the previous command
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        # speakers mode where received filename must be played
        success, message = self.charmie_speech.play_command(filename=request.filename, show_in_face=request.show_in_face, \
                                                            long_pause=request.long_pause_show_in_face, \
                                                            breakable_play=request.breakable_play, break_play=request.break_play)
        if success == False:
            self.get_logger().error("SPEAKERS received (file) does not exist! - %s" %request.filename)

            if request.command != "": # case where wrongfully task is using speakers and not speakers_with_save
                self.get_logger().error("Command is not valid. Using the wrong ROS2 speakers node. Pleace check if using -speakers- or -speakers_with_save- node.")
                message = "Command is not valid. Using the wrong ROS2 speakers node. Pleace check if using -speakers- or -speakers_with_save- node."
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
        # bool long_pause_show_in_face    # whether after showing in face, a long or a short pause should be added, for user easier reading
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        """
        any_empty_command = False
        commands = {}
        for i in range(len(request.filename)):
            commands[request.filename[i]] = request.command[i]

        for filename, command in commands.items():

            if command == "":
                
                self.get_logger().error("Empty command.")
                any_empty_command = True

            else:

                self.charmie_speech.load_and_play_command(filename=filename, command=command, quick_voice=request.quick_voice, \
                                                        show_in_face=request.show_in_face, play_command=request.play_command)
                
                if request.play_command:
                    success = True
                    message = str(len(request.filename))+" new speech files saved and sound played"
                else:
                    success = True
                    message = str(len(request.filename))+" new speech files saved"
        
        if any_empty_command:
            success = False
            message = "Empty command."

        """
        success = False
        message = "Command is not valid. Using the wrong ROS2 speakers node. Pleace check if using -speakers- or -speakers_with_save- node."
        self.get_logger().error("Command is not valid. Using the wrong ROS2 speakers node. Pleace check if using -speakers- or -speakers_with_save- node.")
                
        response.success = success
        response.message = message
        return response


# Standard ROS2 main function
def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
