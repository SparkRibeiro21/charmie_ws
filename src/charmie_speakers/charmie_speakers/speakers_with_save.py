#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from charmie_interfaces.srv import SaveSpeechCommand, SpeechCommand

from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

import time
from pathlib import Path

class RobotSpeak():
    def __init__(self, node):
        # imports the main speakers node to publish face info
        self.node = node

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
        
    # function for commands to be created in the moment 
    def load_and_play_command(self, filename="", command="", quick_voice=False):
        
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


class SpeakerNode(Node):

    def __init__(self):
        super().__init__("Speaker")
        self.get_logger().info("Initialised CHARMIE Speaker Node")

        # initialize robot speech class with acess to node variables
        self.charmie_speech = RobotSpeak(self)

        # SERVICES:
        # Received to save command
        self.save_server_speech_command = self.create_service(SaveSpeechCommand, "save_speech_command", self.callback_save_speech_command) 

        # To speak a file via SPEAKERS node
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        
        self.get_logger().info("Save Speech Servers have been started")

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

        any_empty_command = False
        commands = {}
        for i in range(len(request.filename)):
            request.filename[i] = request.filename[i].lower()
            commands[request.filename[i]] = request.command[i]

        for filename, command in commands.items():

            if command == "":
                
                self.get_logger().error("Empty command.")
                any_empty_command = True

            else:

                self.charmie_speech.load_and_play_command(filename=filename, command=command, quick_voice=request.quick_voice)
                
                if request.play_command:
                    self.get_logger().warn("Currently play command via SAVE can not be done with WFEO=True. It is advised to do SAVE and then SPEAK in the task node not here.")

                    req = SpeechCommand.Request()
                    req.filename = "temp/"+filename
                    req.command = command
                    req.quick_voice = request.quick_voice
                    req.show_in_face = request.show_in_face
                    req.long_pause_show_in_face = False
                    req.breakable_play = False
                    req.break_play = False

                    ### PLAY COMMAND HERE
                    ### IN ORDER TO HAVE WAIT FOR END OF I WOULD NEED TO HAVE A SEPARATE THREAD
                    self.speech_command_client.call_async(req)

                    success = True
                    message = str(len(request.filename))+" new speech files saved and sound played"
                else:
                    success = True
                    message = str(len(request.filename))+" new speech files saved"
        
        if any_empty_command:
            success = False
            message = "Empty command."

        response.success = success
        response.message = message
        return response
            

# Standard ROS2 main function
def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
