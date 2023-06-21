#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from charmie_interfaces.msg import SpeechType

import io
from pydub import AudioSegment
import speech_recognition as sr
import tempfile
import os 
# import click

import whisper
import time
import torch

# it is necessary to state the package before the import since it executes from the install file and not here
from charmie_audio.words_dict import names_dict, drinks_dict, yes_no_dict, charmie_dict

class WhisperAudio():
    def __init__(self):
        self.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        print("\tUsing Device:", self.DEVICE, torch.cuda.is_available())

        self.temp_dir = tempfile.mkdtemp()
        self.save_path = os.path.join(self.temp_dir, "temp.wav")

        self.flag_new_listening_process = False
        self.MIN_AVG_LOG_PROB = -0.8 # -1.05
        self.MIN_NO_SPEECH_PROB = 0.5

        # @click.command()
        # @click.option("--model", default="tiny", help="Model to use", type=click.Choice(["tiny","base", "small","medium","large"]))
        # @click.option("--english", default=True, help="Whether to use English model",is_flag=True, type=bool)
        # @click.option("--verbose", default=False, help="Whether to print verbose output", is_flag=True,type=bool)
        # @click.option("--energy", default=300, help="Energy level for mic to detect", type=int)
        # @click.option("--dynamic_energy", default=False,is_flag=True, help="Flag to enable dynamic engergy", type=bool)
        # @click.option("--pause", default=1.0, help="Pause time before entry ends", type=float)
        self.model = "tiny"
        self.english = True
        self.verbose = False
        self.energy = 300
        self.dynamic_energy = False # still need to test this
        self.pause = 1.0
        
        #there are no english models for large
        if self.model != "large" and self.english:
            self.model = self.model + ".en"
        self.audio_model = whisper.load_model(self.model, device=self.DEVICE)

        print("\tUsing English model:", self.model)

        #load the speech recognizer and set the initial energy threshold and pause threshold
        self.r = sr.Recognizer()
        self.r.energy_threshold = self.energy
        self.r.pause_threshold = self.pause
        self.r.dynamic_energy_threshold = self.dynamic_energy

        # for index, name in enumerate(sr.Microphone.list_microphone_names()):
        #     print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

        self.adjust_ambient_noise()

    def adjust_ambient_noise(self):
        # print("Ready to Start")
        with sr.Microphone(sample_rate=16000) as source:

            self.r.adjust_for_ambient_noise(source)
            # listen for 1 second to calibrate the energy threshold for ambient noise levels
            print("\tENERGY THRESHOLD = ", self.r.energy_threshold)

            # now when we listen, the energy threshold is already set to a good value, and we can reliably catch speech right away
            print("\tReady to Start")

    def hear_speech(self):
        
        with sr.Microphone(sample_rate=16000) as source:
            print("\tReady to Listen...")
            self.audio = self.r.listen(source)
            print("\tMESSAGE HEARD :-)")
    
    def check_speech(self):
            
        audio = self.audio    
        start1 = time.time()
        data = io.BytesIO(audio.get_wav_data())
        audio_clip = AudioSegment.from_file(data)
        audio_clip.export(self.save_path, format="wav")
        end1 = time.time()
        # print("Create Audio File Time:", end1-start1)

        start2 = time.time()

        # english = True
        # result = audio_model.transcribe(save_path, language='english', fp16=False)
        
        audio = whisper.load_audio(self.save_path)
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(self.audio_model.device)
        options = whisper.DecodingOptions(language="en", without_timestamps=True, fp16=False)
        result = whisper.decode(self.audio_model, mel, options)
        end2 = time.time()

        # print("Analysing Audio File Time:", end2-start2)
        print("\tElapsed Time:", end2-start1)
        print(result)

        # predicted_text = result["text"]
        # print("You said: " + predicted_text)
        # print("avg_logprob: " + result["avg_logprob"])
        # print(result)
        # x = result["avg_logprob"]
        # avg_logprob = result["segments"][0]["avg_logprob"]
        avg_logprob = result.avg_logprob
        no_speech_prob = result.no_speech_prob
        print("\tavg_log_prob =", avg_logprob)
        print("\tno_speech_prob =", no_speech_prob)
        if avg_logprob > self.MIN_AVG_LOG_PROB and no_speech_prob < self.MIN_NO_SPEECH_PROB: # tem que ser algo ouvido com alguma fiabilidade senao ignora
            predicted_text = result.text
            # print("\tYou said: " + predicted_text)
            return predicted_text
            # E AQUIIIIII
        else:
            # print("\tI THINK WHAT I HEARD WAS NOISE.")
            return "ERROR"
        
    def check_keywords(self, speech, command: SpeechType):

        speech = speech.lower()

        if command.yes_or_no == True:
            print("YES_OR_NO KEYWORDS!")
            yn_predicted = ''
            yn_ctr = 0
            
            print("YES OR NO:")
            for key in yes_no_dict:
                res = self.compare_commands(yes_no_dict, speech, [key])
                print('    ', key, '\t->', res)
                if res:
                    yn_predicted = key
                    yn_ctr += 1
            print("Yes_No Detected =", yn_predicted, "(", yn_ctr, ")")
            print()

            if yn_ctr == 1:
                final_str=yn_predicted
                print("INFO SENT:'%s'" %  final_str)
                return final_str
            else:
                print("SENT YES_NO ERROR")
                return "ERROR"


        elif command.receptionist == True:
            print("RECEPTIONIST KEYWORDS!")
            name_predicted = ''
            name_ctr = 0
            drink_predicted = ''
            drink_ctr = 0

            print("NAMES:")
            for key in names_dict:
                res = self.compare_commands(names_dict, speech, [key])
                print('    ', key, '\t->', res)
                if res:
                    name_predicted = key
                    name_ctr += 1
            print("Name Detected =", name_predicted, "(", name_ctr, ")")
            print()

            print("DRINKS:")
            for key in drinks_dict:
                res = self.compare_commands(drinks_dict, speech, [key])
                print('    ', key, '\t->', res)
                if res:
                    drink_predicted = key
                    drink_ctr += 1
            print("Drink Detected =", drink_predicted, "(", drink_ctr, ")") 
            print()

            if name_ctr == 1 and drink_ctr == 1:
                final_str=name_predicted + ' ' + drink_predicted
                print("INFO SENT:'%s'" %  final_str)
                return final_str
            else:
                print("SENT RECEPTIONIST ERROR")
                return "ERROR"
            
        elif command.restaurant == True:
            print("RESTAURANT KEYWORDS!")
        elif command.gpsr == True:
            print("GPSR KEYWORDS!")
            pass
        else:
            print("ERROR SELECTING AUDIO MODE!")
            return "ERROR"


    def compare_commands(self, w_dict, predicted_text, lst):

        ctr_tot = 1

        for commands in lst:
            ctr = 0
            # print("command:", commands, end="")
            for word in w_dict[commands]:
                if word in predicted_text:
                    ctr = ctr + 1
            # print(" (", ctr, ")", sep='')
            ctr_tot *= ctr
        # print("ctr_tot:",ctr_tot)

        if ctr_tot > 0:
            return True
        else:
            return False
            
class AudioNode(Node):

    def __init__(self):
        super().__init__("Audio")
        self.get_logger().info("Initialised CHARMIE Audio v2 Node")

        self.audio_command_subscriber = self.create_subscription(SpeechType, "audio_command", self.audio_command_callback, 10)
        self.flag_listening_publisher = self.create_publisher(Bool, "flag_listening", 10)
        self.get_speech_publisher = self.create_publisher(String, "get_speech", 10)
        
        self.charmie_audio = WhisperAudio()


    def audio_command_callback(self, comm: SpeechType):
        print(comm)
        
        # publish rgb estou a ouvir
        self.charmie_audio.hear_speech()
        flag = Bool()
        flag.data = True
        self.flag_listening_publisher.publish(flag)
        
        # publish rgb estou a criar o speech
        speech_heard = self.charmie_audio.check_speech()
        print("\tYou said: " + speech_heard)
        
        # publish rgb estou a calcular as keywords
        keywords = self.charmie_audio.check_keywords(speech_heard, comm)
        # print("Keywords:", keywords)

        speech = String()
        if keywords is None:
            speech.data = speech_heard
        else:
            speech.data = keywords

        self.get_speech_publisher.publish(speech)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    rclpy.shutdown()
