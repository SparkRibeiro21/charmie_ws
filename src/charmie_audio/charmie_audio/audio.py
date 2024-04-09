#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, String, Float32, Int16
# from charmie_interfaces.msg import SpeechType, RobotSpeech
from charmie_interfaces.srv import GetAudio, CalibrateAudio

import io
from pydub import AudioSegment
import speech_recognition as sr
import tempfile
import os 
from pathlib import Path
from datetime import datetime

# Surpress Deprecation Warnings
# Some dependencies libraries from Whisper have Deprecation Warnings, this way
# these do not show in terminal, when whisper updates Numba version, can be removed 
from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning
import warnings
warnings.simplefilter('ignore', category=NumbaDeprecationWarning)
warnings.simplefilter('ignore', category=NumbaPendingDeprecationWarning)

import whisper
import time
import torch
import pulsectl
import signal
import threading # for double audio detection
import sounddevice as sd # for some reason when i added this import all the ALSA lib writtings on the terminal stopped. Heck yeah!
import pyaudio
import numpy as np
import wave

# it is necessary to state the package before the import since it executes from the install file and not here
from charmie_audio.words_dict import names_dict, drinks_dict, yes_no_dict, charmie_dict, foods_dict, numbers_dict, max_number_of_chars_of_keys

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8 = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

# this variable when True is used to enter a calibration mode for the dict words, without being necessary
# to run any other node. It is used (mainly in competition) when new words are added to what the robot must be 
# able to recognise. Check 'words_dict' to see the words the robot must recognise on each category.
DICT_CALIBRATION = False
CALIBRATION_PRINTS = True
FULL_CALIBRATION_PRINTS = False # leave false unless you need to chack a more in-depth audio keywords detected

# post robocup 23 tasks for audio
    # - dois sistemas de audição em paralelo, para que normalmente use o que ouve e pára no fim da frase
    # mas quando não ouve a paragem usa o que grava com o timeout
# - modo de ouvir keywords constante (sempre a ouvir à espera de uma certa keyword)
# - mais idiomas (adicionar tambem o portugues)
# - adicionar a lingua ao speechtype recebido 
# - solução para palavras iguais
# - definir keywords com antecedência (deixar tudo pronto para os objetos do dataset ycb)
# - define all the error, diferent from each other
# - recalibrar audios quando der erro X vezes?
# 
# 
# ---/---
#
# already done post robocup 23 tasks for audio 
# - resolver bug do 1o processamento (fica 30 segundos a processar o que deveria ser feito em 0.1 segundos)
# - definir um tempo maximo de processamento
# - definir um tempo maximo de audição e pára
# - sistema que define tempo maximo de audiçao e ouve o que tem!
# - remover as merdinhas todas do alsalib (sempre que se faz - with sr.Microphone(sample_rate=16000) as source:  )
# - calibrar de x em x segundos ou usar o dynamic energy (revelou-se muito instável e decidiu-se por uma questão de estabilidade continuar com o comando de calibração quando necessário pelo alto nível)
# - modo de treino, para calibrar palavras sem ser necesário mais nenhum node estar a correr
# - reestruturar codigo pos robocup
# - simplificar diagnostic para se ver no terminal
# - ligação direta aos rgb
# - ligacao direta à fala


"""
to do sovalhão Tiago Ribeiro:

- testar o timeout do processing do whisper


- adicionar que se der erro no check_speech nem vale a pena entrar no keywords...
    ja esta no calibration mas ainda não está no main


- colocar record e listen em paralelo
  -> 
- testar o timeout do hearing

"""


class TimeoutException(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutException("Timeout Exception TR")

class WhisperAudio():
    def __init__(self, node):

        self.node = node

        self.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        print("\tInitilisations [", end='')
        print("Device:", self.DEVICE, torch.cuda.is_available())

        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_audio/charmie_audio"
        self.complete_path = self.home+'/'+self.midpath+'/'
        # self.save_temp_path = self.complete_path, "temp.wav"
        
        
        # os.path.join(self.complete_path, "temp.wav")
        print(self.complete_path+"temp.wav")

        self.flag_new_listening_process = False
        self.MIN_AVG_LOG_PROB = -0.8 # -1.05
        self.MIN_NO_SPEECH_PROB = 0.5
        
        # @click.command()
        # @click.option("--model", default="tiny", help="Model to use", type=click.Choice(["tiny","base", "small","medium","large"]))
        # @click.option("--english", default=True, help="Whether to use English model",is_flag=True, type=bool)
        # @click.option("--verbose", default=False, help="Whether to print verbose output", is_flag=True,type=bool)
        # @click.option("--energy", default=300, help="Energy level for mic to detect", type=int)
        # @click.option("--dynamic_energy", default=False,is_flag=True, help="Flag to enable dynamic energy", type=bool)
        # @click.option("--pause", default=1.0, help="Pause time before entry ends", type=float)
        self.model = "tiny"
        self.english = True
        self.verbose = False
        self.energy = 300
        self.dynamic_energy = False # already tested, turn pout to be quite unstable
        self.pause = 1.0
        self.hearing_timeout = 8.0 # time in seconds for audio listening timeout
        self.processing_timeout = 5 # must be integer
        self.record_thread_active = True
        self.audio_data = None
        
        signal.signal(signal.SIGALRM, timeout_handler) # alarm handler 
        
        #there are no english models for large
        if self.model != "large" and self.english:
            self.model = self.model + ".en"
        self.audio_model = whisper.load_model(self.model, device=self.DEVICE)
        print(" Model: English (", self.model, ")]")

        #load the speech recognizer and set the initial energy threshold and pause threshold
        self.r = sr.Recognizer()
        self.r.energy_threshold = self.energy
        self.r.pause_threshold = self.pause
        self.r.dynamic_energy_threshold = self.dynamic_energy
        self.check_threshold = Float32()

        self.ERRO_MAXIMO = False # temp var unltil i fix the timeout when no speak start is detected
        
        # TO CHECK INFO REGARDING DEVICES ...

        # METHOD 1:
        # for index, name in enumerate(sr.Microphone.list_microphone_names()):
        #     print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

        # METHOD 2:
        # p = pyaudio.PyAudio()
        # info = p.get_host_api_info_by_index(0)
        # num_devices = info.get('deviceCount')
        # print("Available audio input devices:")
        # for i in range(num_devices):
        #     device_info = p.get_device_info_by_host_api_device_index(0, i)
        #     print(f"Device {i}: {device_info['name']}")
        # p.terminate()

        # METHOD 3:
        # Query available input devices and print their information
        # input_devices = sd.query_devices(kind='input')
        # for i, device in enumerate(input_devices):
        #     print(f"Device {i}: {device['name']}")

        # input_devices = sd.query_devices(kind='input')
        # for i, device in enumerate(input_devices):
        #     print(f"Device {i}: {device['name']}")

        # input_devices = sd.query_devices(kind='input')
        # print(input_devices)

        # for i, device in enumerate(input_devices):
        #     print(f"Device {i}: {device['name']}")

        self.adjust_ambient_noise()

        """
        # sistema que usa RECORD em vez de listen, funciona a 100%
        with sr.Microphone(sample_rate=16000) as source:
            print("Started Record")
            self.audio = self.r.record(source, duration=5)
            print("Stopped Record")

        speech_heard = self.check_speech()
        print("\tYou said: " + speech_heard)


        while True:
            pass
        """

    
    def adjust_ambient_noise(self):

        self.node.set_rgb(CYAN+ALTERNATE_QUARTERS)

        print("\tCalibrating energy for ambient noise levels...", end='', flush=True)
        # print("Ready to Start")
        with sr.Microphone(sample_rate=16000) as source:
            # listen for 1 second to calibrate the energy threshold for ambient noise levels
            self.r.adjust_for_ambient_noise(source)
            # self.check_threshold = "{:.3f}".format(self.r.energy_threshold)
            self.check_threshold = round(self.r.energy_threshold, 3)
            print(" ENERGY THRESHOLD =", self.check_threshold)
            
        self.node.set_rgb(CLEAR)


    """
    def hear_speech(self):
        
        with sr.Microphone(sample_rate=16000) as source:
            print("\tReady to Listen...")
            print(" ENERGY THRESHOLD = ", self.check_threshold)
            try:
                self.audio = self.r.listen(source, self.hearing_timeout)
                print("\tMESSAGE HEARD :-)")
                # ||||| add rgb protocol
            except sr.WaitTimeoutError as e:
                self.audio = None
                print("\tLISTENING TIMEOUT ERROR - (", e, ")")
                # ||||| add rgb protocol
    """


    """
    Library explanation:
    recognizer_instance.listen(source, timeout = None, phrase_time_limit=None)

    Records a single phrase from source (an AudioSource instance) into an AudioData 
    instance, which it returns. This is done by waiting until the audio has an energy
    above recognizer_instance.energy_threshold (the user has started speaking), and
    then recording until it encounters recognizer_instance.pause_threshold seconds of
    non-speaking or there is no more audio input. The ending silence is not included.
    The timeout parameter is the maximum number of seconds that it will wait for a 
    phrase to start before giving up and throwing an speech_recognition.WaitTimeoutError 
    exception. If timeout is None, it will wait indefinitely. The phrase_time_limit 
    is the maximum amount of time (in seconds) that the method will continue capturing
    audio after the start of the phrase (speech). If the user pauses for longer than 
    this time, the capturing will stop even if the timeout has not been reached.
    """

    """
    Explanation from TR:
    a funcao listen ouve até exitir o fim de uma fala detetada:
    
    problema 1 (fica infinitamente a ouvir):
    - as vezes nao deteta o inicio da fala
    solução: 
    - temos o parametro timeout que gera um sr.WaitTimeoutError
    
    problemas 2 (fica infinitamente a ouvir):
    - as vezes nao deteta o fim da fala
    solução: 
    - temos o parametro phrase_time_limit que finaliza o ficheiro audio apos este tempo
    
    a solução ao problema 1 faz com que se cancele a audição 
    e o ficheiro NÃO FICA desponivel para analise do whisper
    
    a solução ao problema 2 faz com que se páre a audição 
    e o ficheiro FICA desponivel para analise do whisper
    
    para colmatar o problema 1 e arranjar maneira que mesmo essa audio que foi
    cancelado fique disponivel para analise do whisper, é necessário arranjar uma
    solução que funcione em paralelo. Para isso há duas soluções que descrevi
    (já com codigo) no word que guardei nos Documents do PC do CHARMIE.

    Neste momento estou a analisar qual das opções a melhor...  
    """

    def record_audio_timeout(self): #self, source):

        # sistema que usa RECORD em vez de listen, funciona a 100%
        # with sr.Microphone(sample_rate=16000) as source_:
        #     print("Started Record")
            # self.audio_rec = self.r.record(source_, duration=self.hearing_timeout)
        #     print("Stopped Record")

        # speech_heard = self.check_speech()
        # print("\tYou said (RECORD): " + speech_heard)

        # print("I am in record")
        # pass
        
        audio_buffer = []
        

        def audio_callback(indata, frames, time, status):
            if status:
                print(status)
            audio_buffer.extend(indata)

        with sd.InputStream(callback=audio_callback, channels=1, dtype=np.int16, samplerate=16000):
            while self.record_thread_active:
                pass  # Continue recording until the flag is set to False

        self.audio_data = np.array(audio_buffer)

        output_filename = "output.wav"
                
        try:
            with wave.open(output_filename, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000) #sd.query_devices(None, 'input')['default_samplerate'])
                wf.writeframes(self.audio_data.tobytes())
            print(f"Saved recorded audio to {output_filename}")
        except Exception as e:
            print("An error occurred while saving the file:", e)

        import os
        print("Current working directory:", os.getcwd())

        # while self.record_thread_active:
        #     pass
        print("\t\t\tFinished Timeout thread")
        print(self.audio_data)
        time.sleep(2)


    def hear_speech(self):
        
        # proc.start()
        # Terminate the process
        # time.sleep(3)
        # proc.terminate()  # sends a SIGTERM


        # self.record_thread_active = True
        # Start background audio recording thread using sounddevice
        # record_thread = threading.Thread(target=self.record_audio_timeout)
        # record_thread.start()


        self.node.set_rgb(CYAN+ROTATE)
            
        print("TEST SOUNDDEVICE")
        with sr.Microphone(sample_rate=16000) as source:
            print("\tListening for speech...", end='')
            # print("\t(ENERGY THRESHOLD =", self.check_threshold, end=')', flush=True)
            print("\t(", self.check_threshold, end=' )', flush=True)
            
            try:
                self.audio = self.r.listen(source, timeout=self.hearing_timeout, phrase_time_limit=self.hearing_timeout)
                # rec_thread.terminate()
                # self.audio_rec = None
                print(" MESSAGE HEARD :)")
                # self.record_thread_active = False
                # sd.stop()
                self.node.set_rgb(CYAN+HALF_ROTATE)
                
                
            except sr.WaitTimeoutError as e:
                print(" LISTENING TIMEOUT ERROR - (", e, ")")
                self.ERRO_MAXIMO = True # temp var unltil i fix the timeout when no speak start is detected
                print("ERRO MAXIMO !!!!!!!!!!")
                # self.record_thread_active = False
                # record_thread.join()
                # self.audio = self.audio_rec
                self.node.set_rgb(RED+ROTATE)
                

        # time.sleep(10)
        # self.record_thread_active = False
        # record_thread.join()


        # audio_data_as_audiodata = sr.AudioData(
        #     self.audio_data.tobytes(),
        #     sample_rate=16000,  # Adjust the sample rate if needed
        #     sample_width=2  # Adjust the sample width if needed
        # ) 
                
            
        # self.audio = audio_data_as_audiodata
        print("\tReady for the next task!!!")

            # por uma flag no hear rec quando termina para ter a certea e não ficar om um valor anterior???


    def check_speech(self):

        final_text = ""

        if self.audio is not None:    
            audio = self.audio    
            start1 = time.time()
            data = io.BytesIO(audio.get_wav_data())
            audio_clip = AudioSegment.from_file(data)
            audio_clip.export(self.complete_path+"temp.wav", format="wav")
            
            # if True: # just for debug
            if not DICT_CALIBRATION:
                current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
                audio_clip.export(self.complete_path+"list_of_previous_audios/"+current_datetime+".wav", format="wav")
            
            end1 = time.time()
            # print("Create Audio File Time:", end1-start1)

            start2 = time.time()

            # english = True
            # result = audio_model.transcribe(save_path, language='english', fp16=False)


            signal.alarm(self.processing_timeout) # defines timeout for whisper processing

            try:
        
                audio = whisper.load_audio(self.complete_path+"temp.wav")
                # audio = whisper.load_audio(self.complete_path+"list_of_previous_audios/"+"2024-04-02 22-01-24.wav") # just for debug
                audio = whisper.pad_or_trim(audio)
                mel = whisper.log_mel_spectrogram(audio).to(self.audio_model.device)
                options = whisper.DecodingOptions(language="en", without_timestamps=True, fp16=False)
                result = whisper.decode(self.audio_model, mel, options)
                end2 = time.time()

                print("\tElapsed Times [", end='')
                print("Config File:", round(end1-start1, 3), end=', ')
                print("Whisper:", round(end2-start2, 3), end=', ')
                print("Total:", round(end2-start1, 3), "]")

                # print(result)

                # predicted_text = result["text"]
                # print("You said: " + predicted_text)
                # print("avg_logprob: " + result["avg_logprob"])
                # print(result)
                # x = result["avg_logprob"]
                # avg_logprob = result["segments"][0]["avg_logprob"]



                avg_logprob = result.avg_logprob
                no_speech_prob = result.no_speech_prob
                # print("\tavg_log_prob =", avg_logprob)
                # print("\tno_speech_prob =", no_speech_prob)
                
                


                print("\tSpeech Probs [", end='')
                print("avg_log_prob:", round(avg_logprob, 3), end=', ')
                print("no_speech_prob:", round(no_speech_prob, 3), "]")
                
                
                if avg_logprob > self.MIN_AVG_LOG_PROB and no_speech_prob < self.MIN_NO_SPEECH_PROB: # tem que ser algo ouvido com alguma fiabilidade senao ignora
                    predicted_text = result.text
                    # print("\tYou said: " + predicted_text)
                    # return predicted_text
                    final_text = predicted_text
                    self.node.set_rgb(CYAN+BACK_AND_FORTH_8)
                
                else:
                    # print("\tI THINK WHAT I HEARD WAS NOISE.")
                    # return "ERROR"
                    final_text = "ERROR"
                    self.node.set_rgb(RED+HALF_ROTATE)
                    
            except TimeoutException:        
                print("Requested task, took too Long")
                final_text = "ERROR"
                self.node.set_rgb(MAGENTA+HALF_ROTATE)
            
            # else:
            #     print("Requested task, ran within expected time")
            
            signal.alarm(0)
        else: 
            # return "ERROR"
            final_text = "ERROR"
            self.node.set_rgb(RED+ROTATE)

        return final_text


    def check_keywords(self, speech, command_type):

        speech = speech.lower()
        speech = speech.replace(",","")
        speech = speech.replace(".","")
        speech = speech.replace("?","")
        speech = speech.replace("!","")

        if command_type == "yes_or_no":
            print("YES_OR_NO KEYWORDS!")
            yn_predicted = ''
            yn_ctr = 0
            
            print("YES OR NO:")
            for key in yes_no_dict:
                res, idx = self.compare_commands(yes_no_dict, speech, [key])
                print('    ', key, end='')
                for spaces in range(max_number_of_chars_of_keys-len(key)):
                    print('.', end='') 
                print('->', res)
                if res:
                    yn_predicted = key
                    yn_ctr += 1
            print("Yes_No Detected =", yn_predicted, "(", yn_ctr, ")")
            print()

            if yn_ctr == 1:
                final_str=yn_predicted
                print("INFO SENT:'%s'" %  final_str)
                self.node.set_rgb(GREEN+BACK_AND_FORTH_8)
                return final_str
            else:
                print("SENT YES_NO ERROR")
                self.node.set_rgb(RED+BACK_AND_FORTH_8)
                return "ERROR"

        elif command_type == "receptionist":
            print("RECEPTIONIST KEYWORDS!")
            name_predicted = ''
            name_ctr = 0
            drink_predicted = ''
            drink_ctr = 0

            print("NAMES:")
            for key in names_dict:
                res, idx = self.compare_commands(names_dict, speech, [key])
                print('    ', key, end='')
                for spaces in range(max_number_of_chars_of_keys-len(key)):
                    print('.', end='') 
                print('->', res)
                if res:
                    name_predicted = key
                    name_ctr += 1
            print("Name Detected =", "(", name_ctr, ")", name_predicted)
            print()

            print("DRINKS:")
            for key in drinks_dict:
                res, idx = self.compare_commands(drinks_dict, speech, [key])
                print('    ', key, end='')
                for spaces in range(max_number_of_chars_of_keys-len(key)):
                    print('.', end='') 
                print('->', res)
                if res:
                    drink_predicted = key
                    drink_ctr += 1
            print("Drink Detected =", "(", drink_ctr, ")", drink_predicted) 
            print()

            if name_ctr == 1 and drink_ctr == 1:
                final_str=name_predicted + ' ' + drink_predicted
                print("INFO SENT:'%s'" %  final_str)
                self.node.set_rgb(GREEN+BACK_AND_FORTH_8)
                return final_str
            else:
                print("SENT RECEPTIONIST ERROR")
                self.node.set_rgb(RED+BACK_AND_FORTH_8)
                return "ERROR"
            
        elif command_type == "restaurant":
            print("RESTAURANT KEYWORDS!")
            foods_predicted = ''
            foods_ctr = 0
            drink_predicted = ''
            drink_ctr = 0

            final_str = ''

            foods = []
            foods_idx = []
            drinks = []
            drinks_idx = []

            complete_order = []
            complete_order_idx = []


            print("FOODS:")
            for key in foods_dict:
                res, idx = self.compare_commands(foods_dict, speech, [key])
                print('    ', key, end='')
                for spaces in range(max_number_of_chars_of_keys-len(key)):
                    print('.', end='') 
                print('->', res)
                if res:
                    foods_predicted += key+" " # different from receptionist so I can see all the different foods requested
                    foods_ctr += 1
                    foods.append(key)
                    foods_idx.append(idx)

            print("Food Detected =", "(", foods_ctr, ")", foods_predicted)
            print()

            print("DRINKS:")
            for key in drinks_dict:
                res, idx = self.compare_commands(drinks_dict, speech, [key])
                print('    ', key, end='')
                for spaces in range(max_number_of_chars_of_keys-len(key)):
                    print('.', end='') 
                print('->', res)
                if res:
                    drink_predicted = key+" "# different from receptionist so I can see all the different drinks requested
                    drink_ctr += 1
                    drinks.append(key)
                    drinks_idx.append(idx)

            print("Drink Detected =", "(", drink_ctr, ")", drink_predicted) 
            print()

            # print(foods, drinks, foods_idx, drinks_idx)
            complete_order = foods + drinks
            complete_order_idx = foods_idx + drinks_idx
            # print(complete_order, complete_order_idx)

            # Combine the two lists into a list of tuples
            combined = list(zip(complete_order, complete_order_idx))
            # Sort the list of tuples based on the numbers
            sorted_combined = sorted(combined, key=lambda x: x[1])
            # Extract the sorted names
            sorted_names = [item[0] for item in sorted_combined]
            print(sorted_names)

            for names in sorted_names:
                final_str += names + ' '
            final_str = final_str[:-1] # to remove the last ' ', so a new instance is not created when doing split(' ')

            foods.clear()
            foods_idx.clear()
            drinks.clear()
            drinks_idx.clear()

            if final_str != '':
                print("INFO SENT:'%s'" %  final_str)
                self.node.set_rgb(GREEN+BACK_AND_FORTH_8)
                return final_str
            else:
                print("SENT RESTAURANT ERROR")
                self.node.set_rgb(RED+BACK_AND_FORTH_8)
                # ||||| add rgb protocol - acho melhor se der erro ficar o rgb de cima que assim sabemos qual o erro
                # tenho de ver com calma porque isto tambem é erro 
                return "ERROR"
        

        elif command_type == "gpsr":
            print("GPSR KEYWORDS!")
            self.node.set_rgb(GREEN+BACK_AND_FORTH_8)
            pass
        else:
            print("ERROR SELECTING AUDIO MODE!")
            self.node.set_rgb(RED+BACK_AND_FORTH_8)
            return "ERROR"


    def compare_commands(self, w_dict, predicted_text, lst):

        ctr_tot = 1
        index = 0

        for commands in lst:
            ctr = 0
            # print("command:", commands, end="")
            for word in w_dict[commands]:
                if word in predicted_text:
                    ctr = ctr + 1
                    index = predicted_text.find(word)
                    # print("INDEX =", index)
            # print(" (", ctr, ")", sep='')
            ctr_tot *= ctr
        # print("ctr_tot:",ctr_tot)

        if ctr_tot > 0:
            return True, index
        else:
            return False, index

    # gets the devices names for audio input
    def get_pulsectl_device_names(self):
        pulse = pulsectl.Pulse('get-device-names')
        input_devices = pulse.source_list()

        return input_devices
            
    
    def is_external_micro(self, device):
    ### This routine checks if the name of the microphone starts with "Shure" because at the date of 28/07/2023 it was the name of the external microphone used
    ### by CHARMIE. If it changes, the name must be changed. The purpose is to avoid cases where the programmer forgets to change the source of the internal
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
        # self.audio_command_subscriber = self.create_subscription(SpeechType, "audio_command", self.audio_command_callback, 10)
        # self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)

        # self.flag_listening_publisher = self.create_publisher(Bool, "flag_listening", 10)
        # self.get_speech_publisher = self.create_publisher(String, "get_speech", 10)

        # self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        
        # self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        # self.calibrate_ambient_noise_subscriber = self.create_subscription(Bool, "calib_ambient_noise", self.calibrate_ambient_noise_callback, 10)
        self.audio_diagnostic_publisher = self.create_publisher(Bool, "audio_diagnostic", 10)

        # Low Level: RGB
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        self.charmie_audio = WhisperAudio(self)

        self.server_audio = self.create_service(GetAudio, "audio_command", self.callback_audio)
        self.server_calibrate_ambient_noise = self.create_service(CalibrateAudio, "calibrate_audio", self.callback_calibrate_audio)
        self.get_logger().info("Audio Servers have been started")

        # self.speech_str = RobotSpeech()
        # self.flag_speech_done = False
        # self.audio_error = False
        # self.latest_command = SpeechType()

        self.check_diagnostics()

        if DICT_CALIBRATION:
            print("\tCALIBRATION MODE ACTIVATED!")
            while True:
                self.dict_calibration() 


    # similar to audio_command_callback() but without the information sent to the speak module
    def dict_calibration(self):

        global CALIBRATION_PRINTS

        self.get_logger().info("Received Audio Command")
        # publish rgb estou a ouvir
        self.charmie_audio.hear_speech()
                
        # flag = Bool()
        # flag.data = True
        # self.flag_listening_publisher.publish(flag)
        self.get_logger().info("Finished Hearing, Start Processing")


        if not self.charmie_audio.ERRO_MAXIMO: # temp var unltil i fix the timeout when no speak start is detected
            # publish rgb estou a criar o speech
            speech_heard = self.charmie_audio.check_speech()
            print("\n  -->\tYou said: " + speech_heard, end='\n\n')
            self.get_logger().info("Finished Processing")

            if CALIBRATION_PRINTS:
                speech = speech_heard.lower()
                speech = speech.replace(",","")
                speech = speech.replace(".","")
                speech = speech.replace("?","")
                speech = speech.replace("!","")

                print("  -->\tPost Filter: " + speech, end='\n\n')

                if speech != "error":
                    name_predicted = ''
                    name_ctr = 0
                    if FULL_CALIBRATION_PRINTS:
                        print("NAMES:")
                    for key in names_dict:
                        res, idx = self.charmie_audio.compare_commands(names_dict, speech, [key])
                        if FULL_CALIBRATION_PRINTS:
                            print('    ', key, end='')
                            for spaces in range(max_number_of_chars_of_keys-len(key)):
                                print('.', end='') 
                            print('->', res)
                        if res:
                            name_predicted += key+" "
                            name_ctr += 1
                    print("Name Detected    =", "(", name_ctr, ")", name_predicted)
                    # print()

                    foods_predicted = ''
                    foods_ctr = 0
                    if FULL_CALIBRATION_PRINTS:
                        print("FOODS:")
                    for key in foods_dict:
                        res, idx = self.charmie_audio.compare_commands(foods_dict, speech, [key])
                        if FULL_CALIBRATION_PRINTS:
                            print('    ', key, end='')
                            for spaces in range(max_number_of_chars_of_keys-len(key)):
                                print('.', end='') 
                            print('->', res)
                        if res:
                            foods_predicted += key+" "
                            foods_ctr += 1
                    print("Foods Detected   =", "(", foods_ctr, ")", foods_predicted)
                    # print()

                    drink_predicted = ''
                    drink_ctr = 0
                    if FULL_CALIBRATION_PRINTS:
                        print("DRINKS:")
                    for key in drinks_dict:
                        res, idx = self.charmie_audio.compare_commands(drinks_dict, speech, [key])
                        if FULL_CALIBRATION_PRINTS:
                            print('    ', key, end='')
                            for spaces in range(max_number_of_chars_of_keys-len(key)):
                                print('.', end='') 
                            print('->', res)
                        if res:
                            drink_predicted += key+" "
                            drink_ctr += 1
                    print("Drink Detected   =", "(", drink_ctr, ")", drink_predicted) 
                    # print()

                    numbers_predicted = ''
                    numbers_ctr = 0
                    if FULL_CALIBRATION_PRINTS:
                        print("NUMBERS:")
                    for key in numbers_dict:
                        res, idx = self.charmie_audio.compare_commands(numbers_dict, speech, [key])
                        if FULL_CALIBRATION_PRINTS:
                            print('    ', key, end='')
                            for spaces in range(max_number_of_chars_of_keys-len(key)):
                                print('.', end='') 
                            print('->', res)
                        if res:
                            numbers_predicted += key+" "
                            numbers_ctr += 1
                    print("Numbers Detected =", "(", numbers_ctr, ")", numbers_predicted) 
                    # print()    

                    yn_predicted = ''
                    yn_ctr = 0         
                    if FULL_CALIBRATION_PRINTS:           
                        print("YES OR NO:")
                    for key in yes_no_dict:
                        res, idx = self.charmie_audio.compare_commands(yes_no_dict, speech, [key])
                        if FULL_CALIBRATION_PRINTS:
                            print('    ', key, end='')
                            for spaces in range(max_number_of_chars_of_keys-len(key)):
                                print('.', end='') 
                            print('->', res)
                        if res:
                            yn_predicted += key+" "
                            yn_ctr += 1
                    print("Yes_No Detected  =", "(", yn_ctr, ")", yn_predicted)
                    print()

                    self.set_rgb(GREEN+BACK_AND_FORTH_8) # green same as when checking speech and keywords

                else:
                    self.set_rgb(RED+BACK_AND_FORTH_8)
        else:
            self.charmie_audio.ERRO_MAXIMO = False # temp var unltil i fix the timeout when no speak start is detected


    def check_diagnostics(self):

        self.flag_diagn = Bool()
        self.flag_diagn.data = False
        self.aux_flag_diagn = Bool()
        self.aux_flag_diagn.data = False
        
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
        # print("Input Sound Devices:")
        for device in input_devices:
            device_type = "External" if self.charmie_audio.is_external_micro(device) else "Internal"
            if device_type == "External":
                self.aux_flag_diagn.data = True
            # print(f"{device.index}: {device.description} ({device_type})")
        
        if self.aux_flag_diagn.data:
            self.get_logger().info(f"( "u"\u2713"+f" ) - External Microphone 'SHURE MV5' Connected!")
        else:
            self.get_logger().info(f"( X ) - External Microphone 'SHURE MV5' NOT CONNECTED!")


        # temp_threshold = "{:.2f}".format(self.charmie_audio.check_threshold)
        if self.charmie_audio.check_threshold < 50.0 or self.charmie_audio.check_threshold > 10000.0:
            self.get_logger().info(f"( X ) - Threshold of {self.charmie_audio.check_threshold} value is wrong!")
            self.flag_diagn.data = False
        
        else:
            self.get_logger().info(f"( "u"\u2713"+f" ) - Correct Threshold Value! Value of {self.charmie_audio.check_threshold}")
            if self.aux_flag_diagn.data == True:
                self.flag_diagn.data = True
            else:
                self.flag_diagn.data = False
            
        # print(flag_diagn)
        self.audio_diagnostic_publisher.publish(self.flag_diagn)



    # def calibrate_ambient_noise_callback(self, flag: Bool):
    #     self.charmie_audio.adjust_ambient_noise()


    # def get_speech_done_callback(self, state: Bool):
    #     print("Received Speech Flag:", state.data)
    #     self.get_logger().info("Received Speech Flag")
    #     self.flag_speech_done = True
    #     if self.audio_error:
    #         self.audio_error = False
    #         print("Stopped Waiting until CHARMIE speaking is over")
    #         ### MUST CHANGE TO SERVICES
    #         # self.audio_command_publisher.publish(self.latest_command)


    def callback_calibrate_audio(self, request, response):
        
        # Type of service received: 
        # (nothing)
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages

        self.charmie_audio.adjust_ambient_noise()
    
        response.success = True
        response.message = "Calibrated Audio Ambient Noise"
        return response


    def callback_audio(self, request, response):
        
        # Type of service received: 
        # bool yes_or_no      # if just want to receive a yes or no answer
        # bool receptionist   # receptionist info: a name of a person and a drinnk
        # bool gpsr           # gpsr info: the full command
        # bool restaurant     # restaurant info: the 2 or 3 items from drinks, fruits, foods and snacks to be served
        # ---
        # string command      # the items requested separated by a space (' ')

        # self.get_logger().info("Received Get Audio %s" %("("+str(request.yes_or_no)+", "+str(request.receptionist)+", "+str(request.gpsr)+", "+str(request.restaurant)+")"))
        
        command_type = ""
        if request.yes_or_no:
            command_type = "yes_or_no"
        elif request.receptionist:
            command_type = "receptionist"
        elif request.restaurant:
            command_type = "restaurant"
        elif request.gpsr:
            command_type = "gpsr"
        keywords = ""

        # self.latest_command = comm
        self.get_logger().info("Received Audio Command")
        # publish rgb estou a ouvir
        
        # while keywords=="" or keywords=="ERROR" or keywords==None:
            
        self.charmie_audio.hear_speech()
        self.get_logger().info("Finished Hearing, Start Processing")
        
        if not self.charmie_audio.ERRO_MAXIMO: # temp var unltil i fix the timeout when no speak start is detected
            # publish rgb estou a criar o speech
            speech_heard = self.charmie_audio.check_speech()
            print("\tYou said: " + speech_heard)
            self.get_logger().info("Finished Processing")
            
            # publish rgb estou a calcular as keywords
            keywords = self.charmie_audio.check_keywords(speech_heard, command_type)
            # print("Keywords:", keywords)
        else:
            self.charmie_audio.ERRO_MAXIMO = False # temp var unltil i fix the timeout when no speak start is detected
            keywords = None
        
        
        if keywords=="" or keywords=="ERROR" or keywords==None:
            self.get_logger().info("Got error, have to retry the hearing")
            keywords = "ERROR"
            # self.speech_str.command = "I did not understand what you said. Could you please repeat?"
            # self.flag_speech_done = False # to prevent that flag may be true from other speak moments that have nothing to do with this node 
            # self.speaker_publisher.publish(self.speech_str)
            
            # activates the flag that puts everything on hold waiting for the end of sentece said
            # self.audio_error = True
            # response.command = "Error"

        ### POR ISTO AUTOMATICO

        # else:
            # self.get_logger().info("Success Hearing")
            # speech = String()
            # speech.data = keywords
            # self.get_speech_publisher.publish(speech)

        response.command = keywords
        return response

    def set_rgb(self, command):
        
        rgb = Int16()
        rgb.data = command
        self.rgb_mode_publisher.publish(rgb)
        print("Published RGB:", command)


    """
    ### MUST CHANGE TO SERVICES
    def audio_command_callback(self, comm: SpeechType):
        print(comm)

        # self.latest_command = comm
        self.get_logger().info("Received Audio Command")
        # publish rgb estou a ouvir
        self.charmie_audio.hear_speech()
        # flag = Bool()
        # flag.data = True
        # self.flag_listening_publisher.publish(flag)
        self.get_logger().info("Finished Hearing, Start Processing")
        
        if not self.charmie_audio.ERRO_MAXIMO: # temp var unltil i fix the timeout when no speak start is detected
            # publish rgb estou a criar o speech
            speech_heard = self.charmie_audio.check_speech()
            print("\tYou said: " + speech_heard)
            self.get_logger().info("Finished Processing")
            
            # publish rgb estou a calcular as keywords
            keywords = self.charmie_audio.check_keywords(speech_heard, comm)
            # print("Keywords:", keywords)
        else:
            self.charmie_audio.ERRO_MAXIMO = False # temp var unltil i fix the timeout when no speak start is detected
            keywords = None

        if keywords == "ERROR" or keywords == None:
            self.get_logger().info("Got error, gonna retry the hearing")
            self.speech_str.command = "I did not understand what you said. Could you please repeat?"
            self.flag_speech_done = False # to prevent that flag may be true from other speak moments that have nothing to do with this node 
            self.speaker_publisher.publish(self.speech_str)
            
            # activates the flag that puts everything on hold waiting for the end of sentece said
            self.audio_error = True

        ### POR ISTO AUTOMATICO

        else:
            self.get_logger().info("Success Hearing")
            speech = String()
            speech.data = keywords
            self.get_speech_publisher.publish(speech)

    # def wait_for_end_of_speaking()
    """
        
def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    rclpy.shutdown()
