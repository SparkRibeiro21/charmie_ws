# this piece of code is just a bunch of examples when testing the new coqui-ai
# also there are a list of all the voices to be compared
# check for more info: https://github.com/coqui-ai/TTS

"""
import torch
from TTS.api import TTS

# Get device
device = "cuda" if torch.cuda.is_available() else "cpu"

# List available 🐸TTS models
print(TTS().list_models())

# Init TTS
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

# Run TTS
# ❗ Since this model is multi-lingual voice cloning model, we must set the target speaker_wav and language
# Text to speech list of amplitude values as output
wav = tts.tts(text="Hello world!", speaker_wav="my/cloning/audio.wav", language="en")
# Text to speech to a file
tts.tts_to_file(text="Hello world!", speaker_wav="my/cloning/audio.wav", language="en", file_path="output.wav")
"""



from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer
import numpy as np
from pydub import AudioSegment
from pydub.playback import play
import array
import site
import time


import pygame
from pathlib import Path

# path = "/home/robot4/.local/lib/python3.10/site-packages/TTS/.models.json"
path = "/home/utilizador/.local/lib/python3.10/site-packages/TTS/.models.json"
model_manager = ModelManager(path)
pygame.init()


#pip3 install pyttsx3 sudo apt install espeak pip3 install pyaudio or use sudo apt install python3-pyaudio

# 10) tts_models/en/ek1/tacotron2 (342.02)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ek1/tacotron2") # MUITO MUITO LENTO
# 11) tts_models/en/ljspeech/tacotron2-DDC (8.35)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/tacotron2-DDC") # 3/10
# 12) tts_models/en/ljspeech/tacotron2-DDC_ph (5.16)
mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/tacotron2-DDC_ph") # 6/10
# 13) tts_models/en/ljspeech/glow-tts (1.61)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/glow-tts") # 5/10
# 14) tts_models/en/ljspeech/speedy-speech 
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/speedy-speech") # nao funciona
# 15) tts_models/en/ljspeech/tacotron2-DCA
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/tacotron2-DCA") # 5/10
# 16) tts_models/en/ljspeech/vits
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/vits") # 6/10
# 17) tts_models/en/ljspeech/vits--neon
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/vits--neon") # 5/10 'you'
# 18) tts_models/en/ljspeech/fast_pitch (2.72)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/fast_pitch") # 4/10
# 19) tts_models/en/ljspeech/overflow (2.72)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/overflow") # 5/10
# 20) tts_models/en/ljspeech/neural_hmm
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/ljspeech/neural_hmm") # 4/10
# 21) tts_models/en/vctk/vits
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/vctk/vits") # erro
# 22) tts_models/en/vctk/fast_pitch
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/vctk/fast_pitch") # erros...
# 23) tts_models/en/sam/tacotron-DDC
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/sam/tacotron-DDC") # 3/10
#  24) tts_models/en/blizzard2013/capacitron-t2-c50
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/blizzard2013/capacitron-t2-c50") # 1/10
#  25) tts_models/en/blizzard2013/capacitron-t2-c150_v2 
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/blizzard2013/capacitron-t2-c150_v2") # erro do espeak
#  26) tts_models/en/multi-dataset/tortoise-v2
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/multi-dataset/tortoise-v2") # nao funciona
# 27) tts_models/en/jenny/jenny (34.76)
# mode_path, config_path, model_item = model_manager.download_model("tts_models/en/jenny/jenny") # LENTO 8/10

# PT) horrible since it does not have a voice encoder
# mode_path, config_path, model_item = model_manager.download_model("tts_models/pt/cv/vits")

voc_path, voc_config_path, _ = model_manager.download_model(model_item["default_vocoder"])

syn = Synthesizer(
    tts_checkpoint= mode_path,
    tts_config_path= config_path,
    vocoder_checkpoint= voc_path,
    vocoder_config= voc_config_path
)




# text = "Olá, o meu nome é charmie, o teu robô assitente. Como posso ajudar? Precisas que te ajuda a carregar o teu saco? Por favor diz-me o teu nome e a tua bebida preferida. Por favor diz o teu pedido. Não estás a cumprir a regra do quarto proibido. Segue-me por favor"

# text = "Hello there. My name is charmie, your robot assistant. How can I help you? Do you need me to carry your luggage? Please say your name and favourite drink. Please say your order. You are breaking the forbidden room rule. Please follow me."

# text = "Welcome. Please say your name and favourite drink."





filename = "last_speaked.wav"
home = str(Path.home())
midpath = "charmie_ws/src/charmie_speakers/charmie_speakers"
complete_path = home+'/'+midpath+'/'+filename


print("Starting Speaking ...")

sentences = []
sentences.append("Please hand me the spoon.")
sentences.append("Please hand me the cereal box.")
sentences.append("Please hand me the milk.")
sentences.append("Please hand me the bowl.")

print(sentences)

for sen in sentences:
	a = time.time()
	text = sen
	outputs = syn.tts(text)
	syn.save_wav(outputs, complete_path)
	print(time.time()-a)
	pygame.mixer.music.load(complete_path)
	pygame.mixer.music.play()
	while pygame.mixer.music.get_busy():
		pass
            

#audio_bytes = np.array(outputs, dtype=np.int16).tobytes()
#sample_rate = syn.tts_model.get("sample_rate", 22050)

#audio_bytes = outputs.astype(np.int16).tobytes()




"""audio_segment = AudioSegment(
    audio_bytes,
    frame_rate=22050,
    sample_width=2,
    channels=1
)

play(audio_segment)


audio_segment = AudioSegment(
    audio_data.tobytes(),
    frame_rate=audio_data.sample_rate,
    sample_width=audio_data.sample_width,
    channels=audio_data.num_channels
)

play(audio_segment)

"""
