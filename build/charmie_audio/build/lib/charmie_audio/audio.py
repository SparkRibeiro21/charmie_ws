#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech

import io
from pydub import AudioSegment
import speech_recognition as sr
import tempfile
import os 
# import click
    
import whisper
import time
import torch
import pulsectl

# it is necessary to state the package before the import since it executes from the install file and not here
from charmie_audio.words_dict import names_dict, drinks_dict, yes_no_dict, charmie_dict, foods_dict

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
        self.check_threshold = Float32()

        # for index, name in enumerate(sr.Microphone.list_microphone_names()):
        #     print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

        self.adjust_ambient_noise()

    def adjust_ambient_noise(self):

        print("RECALIBRATED AUDIO!!!")
        # print("Ready to Start")
        with sr.Microphone(sample_rate=16000) as source:

            self.r.adjust_for_ambient_noise(source)
            # listen for 1 second to calibrate the energy threshold for ambient noise levels
            print("\tENERGY THRESHOLD = ", self.r.energy_threshold)
            print(type(self.r.energy_threshold))
            self.check_threshold = self.r.energy_threshold


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
            foods_predicted = ''
            foods_ctr = 0
            drink_predicted = ''
            drink_ctr = 0

            foods_predicted2 = ''
            foods_ctr2 = 0
            drink_predicted2 = ''
            drink_ctr2 = 0

            final_str = ''

            foods = []
            drinks = []


            print("FOODS:")
            for key in foods_dict:
                res = self.compare_commands(foods_dict, speech, [key])
                print('    ', key, '\t->', res)
                if res:
                    foods_predicted = key
                    foods_ctr += 1
                    foods.append(key)

            print("Name Detected =", foods_predicted, "(", foods_ctr, ")")
            print()

            print("DRINKS:")
            for key in drinks_dict:
                res = self.compare_commands(drinks_dict, speech, [key])
                print('    ', key, '\t->', res)
                if res:
                    drink_predicted = key
                    drink_ctr += 1
                    drinks.append(key)

            print("Drink Detected =", drink_predicted, "(", drink_ctr, ")") 
            print()

            print(foods, drinks)

            ### falta alterar para mais que um de cada

            for f in foods:
                final_str += f + ' '

            for d in drinks:
                final_str += d + ' '

            # final_str=foods_predicted + ' ' + drink_predicted
            print("INFO SENT:'%s'" %  final_str)

            foods.clear()
            drinks.clear()
    
            if final_str != '':
                print("INFO SENT:'%s'" %  final_str)
                return final_str
            else:
                print("SENT RESTAURANT ERROR")
                return "ERROR"
        

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

            
    def get_pulsectl_device_names(self):
        pulse = pulsectl.Pulse('get-device-names')
        input_devices = pulse.source_list()

        return input_devices
            
    
    def is_external_micro(self, device):
    ### This routine checks if the name of the microphone starts with "Shure" because at the date of 28/07/2023 it was the name of the external microphone used
    ### by the CHARMIE. If it changes, the name must be changed. The purpose is to avoid cases where the programmer forgets to change the source of the internal
    ### sound device and the voice of CHARMIE can't be heard. This way, a diagnose can be made.
        proplist = device.proplist
        if "device.intended_roles" in proplist:
            intended_roles = proplist["device.intended_roles"].split(",")
            if "phone" in intended_roles or "microphone" in intended_roles:
                return True
        if "device.description" in proplist:
            device_description = proplist["device.description"].lower()
            if device_description.startswith("shure"):          ### <===== CHANGE ME (if needed)!!!
                return True
        return False
            

class AudioNode(Node):

    def __init__(self):
        super().__init__("Audio")
        self.get_logger().info("Initialised CHARMIE Audio v2 Node")

        # I publish and subscribe in the same topic so I can request new hearings when errors are received 
        self.audio_command_subscriber = self.create_subscription(SpeechType, "audio_command", self.audio_command_callback, 10)
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)

        self.flag_listening_publisher = self.create_publisher(Bool, "flag_listening", 10)
        self.get_speech_publisher = self.create_publisher(String, "get_speech", 10)

        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        
        self.calibrate_ambient_noise_subscriber = self.create_subscription(Bool, "calib_ambient_noise", self.calibrate_ambient_noise_callback, 10)
        self.audio_diagnostic_publisher = self.create_publisher(Bool, "audio_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = False
        aux_flag_diagn = Bool()
        aux_flag_diagn.data = False
        
        self.charmie_audio = WhisperAudio()

        self.speech_str = RobotSpeech()
        self.flag_speech_done = False
        self.audio_error = False
        self.latest_command = SpeechType()

        

        """ device_name, host_api = self.charmie_audio.find_speaker_device()

        if device_name:
            print('It is connected to the correct device, named ', device_name)
        
        else:
            print('It is not onnected to the correct device. Please, change it') """
        
        """ audio_devices = self.charmie_audio.list_audio_devices()
        print("Connected Audio Devices:")
        for audio_device in audio_devices:
            print(audio_device)
        """

        input_devices = self.charmie_audio.get_pulsectl_device_names()
        print("Input Sound Devices:")
        for device in input_devices:
            device_type = "External" if self.charmie_audio.is_external_micro(device) else "Internal"
            if device_type == "External":
                aux_flag_diagn.data = True
            print(f"{device.index}: {device.description} ({device_type})")

        if self.charmie_audio.check_threshold < 50.0:
            self.get_logger().info(f"Threshold of {self.charmie_audio.check_threshold} value is wrong!")
            flag_diagn.data = False
        
        else:
            self.get_logger().info(f"Nice Threshold value! Value of {self.charmie_audio.check_threshold}")
            if aux_flag_diagn.data == True:
                flag_diagn.data = True
            else:
                flag_diagn.data = False
            
        print(flag_diagn)
        self.audio_diagnostic_publisher.publish(flag_diagn)




    def calibrate_ambient_noise_callback(self, flag: Bool):
        self.charmie_audio.adjust_ambient_noise()


    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.get_logger().info("Received Speech Flag")
        self.flag_speech_done = True
        if self.audio_error:
            self.audio_error = False
            print("Stopped Waiting until CHARMIE speaking is over")
            self.audio_command_publisher.publish(self.latest_command)


    def audio_command_callback(self, comm: SpeechType):
        print(comm)

        self.latest_command = comm
        self.get_logger().info("Received Audio Command")
        # publish rgb estou a ouvir
        self.charmie_audio.hear_speech()
        flag = Bool()
        flag.data = True
        self.flag_listening_publisher.publish(flag)
        self.get_logger().info("Finished Hearing, Start Processing")
        
        # publish rgb estou a criar o speech
        speech_heard = self.charmie_audio.check_speech()
        print("\tYou said: " + speech_heard)
        self.get_logger().info("Finished Processing")
        
        # publish rgb estou a calcular as keywords
        keywords = self.charmie_audio.check_keywords(speech_heard, comm)
        # print("Keywords:", keywords)

        if keywords == "ERROR" or keywords == None:
            self.get_logger().info("Got error, gonna retry the hearing")
            self.speech_str.command = "I did not understand what you said. Could you please repeat?"
            self.flag_speech_done = False # to prevent that flag may be true from other speak moments that have nothing to do with this node 
            self.speaker_publisher.publish(self.speech_str)
            
            # activates the flag that puts everything on hold waiting for the end of sentece said
            self.audio_error = True
            
        else:
            self.get_logger().info("Success Hearing")
            speech = String()
            speech.data = keywords
            self.get_speech_publisher.publish(speech)

    
def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    rclpy.shutdown()