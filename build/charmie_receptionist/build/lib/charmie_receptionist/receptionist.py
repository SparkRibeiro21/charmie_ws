#!/usr/bin/env python 3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Int16, Bool, Int8MultiArray, Float32MultiArray, Int8, Float32, String
import cv2
from sensor_msgs.msg import Image as Img
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import time
from charmie_interfaces.msg import Encoders, PS4Controller, RobotSpeech, SpeechType
#from charmie_debug.debug_main import get_color_image_callback
import cv2
import numpy as np
import face_recognition
import os
import tensorflow as tf
from datetime import datetime
from deepface import DeepFace
import shutil
import PIL
from PIL import Image
import mediapipe as mp
import math
import threading
import socket
import argparse
from mediapipe.python.solutions.drawing_utils import _normalized_to_pixel_coordinates
import random

class ReceptionistNode(Node):

    def _init_(self):
        super()._init_("Receptionist")
        self.get_logger().info("Initiliased Receptionist Node")

        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback, 10)
        self.flag_neck_position_publisher = self.create_publisher(Bool, "flag_neck_pos", 10)

        # Low Level Topics
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
            #self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
            #self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
            #self.vccs_subscriber = self.create_subscription(Pose2D, "get_vccs", self.get_vccs_callback, 10)
            #self.flag_vccs_publisher = self.create_publisher(Bool, "flag_vccs", 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
            #self.get_encoders_subscriber = self.create_subscription(Encoders, "get_encoders", self.get_encoders_callback, 10)
            #self.flag_encoders_publisher = self.create_publisher(Bool, "flag_encoders", 10)
        
        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
            #self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)

        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        self.flag_speaker_publisher = self.create_publisher(Bool, "flag_speech_done", 10)

        # Audio
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)
        self.flag_listening_subscriber = self.create_subscription(Bool, "flag_listening", self.flag_listening_callback, 10)
        self.get_speech_subscriber = self.create_subscription(String, "get_speech", self.get_speech_callback, 10)

        # Face Shining RGB
        self.face_publisher = self.create_publisher(Int16, "face_command", 10)

        # Odometry
        self.odometry_subscriber = self.create_subscription(Odometry, "odom_robot",self.get_odometry_robot_callback, 10)

        # Timer
        self.create_timer(0.05, self.timer_callback)

        # Changing Variables
        self.state = 0
        self.door_coordinates = Vector3()
        self.door_coordinates.x = 0.0
        self.door_coordinates.y = 0.0
        self.door_coordinates.z = 0.0 #orientation
        
        self.place_to_sit = Vector3()
        self.place_to_sit.x = 0.0
        self.place_to_sit.y = 0.0
        self.place_to_sit.z = 0.0

        self.sofa_coordinates = Vector3()
        self.sofa_coordinates.x = 0.0
        self.sofa_coordinates.y = 0.0
        self.sofa_coordinates.z = 0.0 #orientation

        self.chair1_coordinates = Vector3()
        self.chair1_coordinates.x = 0.0
        self.chair1_coordinates.y = 0.0
        self.chair1_coordinates.z = 0.0 #orientation

        self.chair2_coordinates = Vector3()
        self.chair2_coordinates.x = 0.0
        self.chair2_coordinates.y = 0.0
        self.chair2_coordinates.z = 0.0 #orientation

        #Temos de pedir para ficar sempre no mesmo sítio
        self.guest_coordinates = Vector3()
        self.guest_coordinates.x = 0.0
        self.guest_coordinates.y = 0.0
        self.guest_coordinates.z = 0.0 #orientation

        #Código para ele olhar para a cara da pessoa e centrar a cara da pessoa na imagem
        self.door_neck_coordinates = Pose2D()
        self.door_neck_coordinates.x = 180.0
        self.door_neck_coordinates.y = 180.0
        #Definir como se estivesse a olhar ligeiramente para baixo - Perceber o que precisamos que olhe para baixo para detetarmos tudo direito
        self.place_to_sit_neck = Pose2D()
        self.place_to_sit_neck.x = 180.0
        self.place_to_sit_neck.y = 180.0

        self.navigation_neck = Pose2D()
        self.navigation_neck.x = 180.0
        self.navigation_neck.y = 180.0

        self.neck_pos = Pose2D()



        self.count_guest = 0
        self.flag_Host = Bool()
        self.flag_Guest1 = Bool()
        self.flag_place_to_sit = Bool()
        self.flag_already_sit = Bool()
        self.flag_Host = False
        self.flag_Guest1 = False
        self.flag_place_to_sit = False
        self.flag_already_sit = False

        self.colour_img = Image()

        self.speech_str = RobotSpeech()

        self.speech_type = SpeechType()
        self.speech_type.receptionist = True
        self.speech_type.yes_or_no = False
        self.speech_type.restaurant = False
        self.speech_type.gpsr = False

        self.get_speech = String()

        self.keywords = String()

        self.contador_conhecidos = 0
        

        
        
        #Pastas relativas às funções das características como por exemplo, características, reconhecimento, etc.
        #CAMINHOS QUE É PRECISO MUDAR NOME

        self.drinks = ["Seven up"]
        self.names = ["Jack"]
        
        self.get_caract = 0
        self.caracteristics_ = []
        self.filename = String()


        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        # MODEL_MEAN_VALUES = (78, 87, 114)

        self.ageList = ['(0-2)', '(4-6)', '(8-12)', '(15-22)', '(23-32)', '(38-43)', '(48-55)', '(60-100)']
        self.genderList = ['Male', 'Female']


        self.folder_path = {"/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/FolderTest"} 

        self.images = []
        self.classNames = []
        
        self.model_loaded = tf.keras.models.load_model('/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_raca.h5')

        self.faceProto = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector_uint8.pb"
        self.faceModel = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector.pbtxt"
        self.ageProto = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_age.prototxt"
        self.ageModel = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/age_net.caffemodel"
        self.genderProto = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_gender.prototxt"
        self.genderModel = "/home/renata/charmie_ws/src/charmie_receptionist/charmie_receptionist/gender_net.caffemodel"


    def timer_callback(self):
        #Estado comum aos dois Guests
        if self.state == 0:
            #Informa que está pronto para começar a tarefa

            self.neck_position_publisher.publish(self.navigation_neck)
            self.speech_str.command = "Hello! I am ready to start the receptionist task! Here I go"
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)
            self.state = 1
            
            


        #Estado para verificar se está alguém na porta e ficar a repetir de 5 em 5 segundos enquanto não estiver lá ninguém
        #Estado para o Guest 1
        elif self.state == 1:    

            #vai para a porta

            self.neck_position_publisher.publish(self.navigation_neck)
            #CORRIGIR CÓDIGO PARA SE MOVER PARA A PORTA E FICAR A OLHAR PARA A PORTA

            self.neck_position_publisher.publish(self.door_neck_coordinates)

            #Informa que está pronto para receber convidado
            self.speech_str.command = "I am ready to receive a new guest"
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)

            #Começar a contar o tempo
            self.init_time = time.time()
            

            #Verificar se existe uma pessoa na porta
            cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
            #Função de deteção facial 
            num_faces = self.detect_face(cv2_img)
            if num_faces != 0 :
                
                #Se detetar alguém o CHARMIE apresenta se e pede nome e bebida
                self.speech_str.command = "Hello! My name is Charmie. What's your name and favourite drink?"
                self.speech_str.language = 'en'
                self.audio_command_publisher.publish(self.speech_str)

                #Informo o Charmie que acabei de falar e posso começar a ouvir
                self.flag_speaker_publisher.publish(True)
                
                #Isto vai acontecer 2x. Para o guest 1 e 2. Mas tem de ir para estados diferentes porque para o guest 2 não é 
                #necessário extrair as características
                self.count_guest+=1
                if self.count_guest==1:
                    self.state = 2
                if self.count_guest==2:
                    self.state = 3
            else:
                if time.time()-self.init_time>5.0:
                    # Repete a ação anterior
                    self.speech_str.command = "I am ready to receive a new guest"
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    #Dá reset ao tempo
                    self.init_time = time.time()
        

        #Estado apenas para o Guest1
        elif self.state == 2:
            
            #CORRIGIR PARA CÓDIGO QUE AJUSTA A POSIÇÃO DA CARA PARA O CENTRO DA FACE DA PESSOA

            #Falar para pedir para olhar para camara
            self.speech_str.command = "I will take you a picture. Please look at me"
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)

            #Obter e guardar a Imagem            
            cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
            self.filename = self.names[1]
            file_path = os.path.join(self.folder_path, self.filename)
            cv2.imwrite(file_path, cv2_img)

            # Apend ao nome e imagem guardada acima
            myList = os.listdir(self.folder_path)
            for cl in myList:
                curImg = cv2.imread(f'{self.folder_path}/{cl}')
                self.images.append(curImg)
                self.classNames.append(os.path.splitext(cl)[0])

            #Recolho as características das imagens
            age,gender,error_agegender = self.detectGenderAge(cv2_img)
            resultrace, error_race = self.detectrace(cv2_img)

            #Para prevenir que o código empanque, garanto que retorno sempre alguma coisa, seja as características ou erro
            if error_race == 0 and error_agegender == 0:
                self.get_caract = 3
                self.caracteristics.append(gender)
                self.caracteristics.append(age)
                self.caracteristics.append(resultrace)
                
            elif error_race == 0 and error_agegender == 1:
                self.get_caract = 1
                self.caracteristics.append(resultrace)
                
            elif error_race == 1 and error_agegender == 0:
                self.get_caract=2
                self.caracteristics.append(gender)
                self.caracteristics.append(age)
                
            else:
                self.caracteristics = "error"
                
            self.state=3

        #Estado comum aos 2 guests
        elif self.state == 3:
            #Agradece e pede para o seguir até à zona dos sofás
            self.speech_str.command = "Thank you. Please follow me."
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)

            #vai para o sofá

            self.neck_position_publisher.publish(self.navigation_neck)

            #CORRIGIR PARA SE MOVER PARA O SOFÁ E O PESCOÇO FICAR ORIENTADO PARA BAIXO NA DIREÇÃO DAS PESSOAS SENTADAS

            self.neck_position_publisher.publish(self.place_to_sit_neck)
            #Diz ao guest para ficar na mesma posição até lhe indicar onde se sentar
            self.speech_str.command = "Please stay here until I give you instructions on where to sit."
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)
            if self.count_guest == 1:
                self.state = 4
            elif self.count_guest == 2:
                self.state = 7
            

        #Estado relativo ao Guest1 no SOFÁ
        elif self.state == 4:
            #Guardar Imagem            
            cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
            num_faces=self.detect_face(cv2_img)
            #Código para ver se encontro faces
            if num_faces != 0:
                if num_faces == 1:
                    #Guarda as coordenadas do sofá como local para sentar e ativa a flag a indicar que já tem um local para sentar
                    self.place_to_sit = self.sofa_coordinates
                    self.flag_place_to_sit=True
                    #Função para reconhecer pessoas. Retorna uma variavel pessoa que tem um centro associado a um nome
                    pessoas_detetadas=self.recognizeGuest(cv2_img)
                    for pessoa in pessoas_detetadas:
                        self.contador_conhecidos = 0
                        if pessoa != 'unknown' and pessoa in self.names:
                            self.contador_conhecidos += 1
                    #Se não for ninguém conhecido avança para o estado seguinte 
                    if self.contador_conhecidos == 0:
                        self.state = 5
                    #Se conheceu alguém é porque reconheceu o Host.
                    elif self.contador_conhecidos == 1:
                        #OLHAR PARA A POSIÇÃO DO GUEST
                        self.neck_position_publisher.publish(self.navigation_neck)



                        #Apresentação do guest e do host
                        self.speech_str.command = f"On the couch is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True

                        #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ ENQUANTO FALA, PODE SER RODAR PESCOÇO OU BASE

                        self.neck_position_publisher.publish(self.place_to_sit_neck)

                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ, PODE SER RODAR PESCOÇO OU BASE
                        
                        self.neck_position_publisher.publish(self.place_to_sit_neck)

                        #Sentou a o convidado
                        self.speech_str.command = f"Please take a sit on the sofa that i'm looking for."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit = True
                        #Não sei o numero do estado, tou a assumir que é o último
                        self.state = 1
                    
                elif num_faces >=2:
                    #Código para reconhecer as pessoas
                    # Contagem das pessoas conhecidas na imagem
                    
                    pessoas_detetadas=self.recognizeGuest(cv2_img)
                    
                    for pessoa in pessoas_detetadas:
                        if pessoa != 'unknown' and pessoa in self.names:
                            self.contador_conhecidos += 1

                    if self.contador_conhecidos == 0:
                        self.state = 5
                    elif self.contador_conhecidos == 1:
                        #Apresentação do guest e do host
                        #CORRIGIR PARA SE VIRAR PARA A PORTA
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"On the couch is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ
                        self.neck_position_publisher.publish(self.place_to_sit_neck)

                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True
                        self.contador_conhecidos = 0
                        self.state = 5

            else: 
                self.place_to_sit = self.sofa_coordinates
                self.flag_place_to_sit=True
                #Guardar a coordenada do sofá
                self.state=5
       
       #Estado da fase do GUEST1 de olhar para a cadeira: ramo da esquerda
        elif self.state == 5:
            if self.flag_Host==True:

                self.neck_position_publisher.publish(self.navigation_neck)

                #CORRIGIR PARA IR PARA A ZONA DA CADEIRA 1
                
                self.neck_position_publisher.publish(self.place_to_sit_neck)
                cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")

                num_faces = self.detect_face(cv2_img)
                if num_faces !=0:

                    self.neck_position_publisher.publish(self.navigation_neck)  
                    #CORRIGIR PARA IR PARA A ZONA DA CADEIRA 2

                    self.neck_position_publisher.publish(self.place_to_sit_neck)

                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces = self.detect_face(cv2_img)
                    if num_faces !=0:
                        self.speech_str.command = "Please stand. I couldn't find an empty seat."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_place_to_sit=True
                        self.flag_already_sit=True
                        self.state = 1
                        
                    else:
                        self.place_to_sit = self.chair2_coordinates
                        self.flag_place_to_sit=True
                        
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the Chair in front of me."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.state = 1
                        self.flag_already_sit=True

                else:
                    self.place_to_sit = self.chair1_coordinates
                    self.flag_place_to_sit=True

                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                    self.speech_str.command = f"Please take a sit on the Chair in front of me."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.state = 1
                    self.flag_already_sit=True
                            
            else:

                self.neck_position_publisher.publish(self.navigation_neck)
                #CORRIGIR PARA IR PARA A CADEIRA 1

                self.neck_position_publisher.publish(self.place_to_sit_neck)

                if self.flag_place_to_sit == True:
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces = self.detect_face(cv2_img)

                    if num_faces !=0:

                        pessoas_detetadas=self.recognizeGuest(cv2_img)  
                        for pessoa in pessoas_detetadas:
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            self.state = 6
                        elif self.contador_conhecidos == 1:
                            self.neck_position_publisher.publish(self.sofa_coordinates)
                            self.speech_str.command = f"On the chair is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                            self.neck_position_publisher.publish(self.guest_coordinates)
                            self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Host = True
                            
                            #CORRIGIR PARA SE ORIENTAR PARA O LOCAL QUE FOI ANTERIORMENTE GUARDADO COMO LOCAL
                            #PARA SENTAR

                            self.neck_position_publisher.publish(self.place_to_sit)

                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_already_sit = True
                            self.contador_conhecidos = 0
                            self.state = 1

                    else:
                        self.state=6

                else:
                    
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces = self.detect_face(cv2_img)
                    if num_faces !=0:

                        pessoas_detetadas=self.recognizeGuest(cv2_img)
                        for pessoa in pessoas_detetadas:
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            self.state = 6
                        elif self.contador_conhecidos == 1:
                            self.neck_position_publisher.publish(self.sofa_coordinates)
                            self.speech_str.command = f"On the chair is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                            self.neck_position_publisher.publish(self.guest_coordinates)
                            self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Host=True

                            self.contador_conhecidos = 0
                            self.state = 6

                    else:
                        self.place_to_sit = self.chair1_coordinates
                        self.flag_place_to_sit = True
                        self.state = 6

                    
        elif self.state == 6:
            self.neck_position_publisher.publish(self.navigation_neck)
            #CORRIGIR PARA ANDAR PARA A CADEIRA 2
            
            self.neck_position_publisher.publish(self.place_to_sit_neck)

            cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
            num_faces = self.detect_face(cv2_img)
            if num_faces !=0:
                if self.flag_Host == True:
                    self.speech_str.command = "Please stand. I couldn't find an empty seat."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.flag_place_to_sit=True
                    self.flag_already_sit=True
                    self.state = 1
                else:

                    pessoas_detetadas=self.recognizeGuest(cv2_img)
                    for pessoa in pessoas_detetadas:
                        if pessoa != 'unknown' and pessoa in self.names:
                            self.contador_conhecidos += 1

                    if self.contador_conhecidos == 0:

                        #Apresentação do guest e do host para o ar
                        #CORRIGIR PARA OLHAR PARA O GUEST
                        self.neck_position_publisher.publish(self.navigation_neck)

                        self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)


                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True

                        self.speech_str.command = "Please stand. I couldn't find an empty seat."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_place_to_sit=True
                        self.flag_already_sit=True
                        self.state = 1

                    elif self.contador_conhecidos == 1:
                        #CORRIGIR PARA SE ORIENTAR PARA O GUEST
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"On the chair is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        #CORRIGIR PARA OLHAR PARA O HOST
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True

                        
                        if self.flag_place_to_sit == True:
                            #CORRIGIR PARA OLHAR PARA O LOCAL A SENTAR

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 1
                        else:
                            #CORRIGIR PARA FICAR A OLHAR PARA ONDE O GUEST ESTÁ
                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = "Please stand. I couldn't find an empty seat."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_place_to_sit=True
                            self.flag_already_sit=True
                            self.state = 1
                        self.contador_conhecidos=0    
            else:
                if self.flag_place_to_sit == True:
                   if self.flag_Host == True:
                        #CORRIGIR PARA FICAR A OLHAR PARA O LUGAR ONDE SE VAI SENTAR A PESSOA
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state = 1
                   else:
                        #CORRIGIR PARA FICAR A OLHAR PARA O GUEST QUE ESTÁ EM PÉ AINDA 
                        #Apresentação do guest e do host para o ar

                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True
                        
                        #CORRIGIR PARA FICAR A OLHAR PARA O LUGAR ONDE SE VAI SENTAR

                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state = 1
                    
                else:
                    self.place_to_sit = self.chair2_coordinates
                    self.flag_place_to_sit = True
                    if self.flag_Host == True:
                        self.neck_position_publisher.publish(self.place_to_sit)
                        self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state = 1
                    else:
                        #CORRIGIR PARA FICAR A OLHAR PARA ONDE O GUEST ESTÁ
                        #Apresentação do guest e do host para o ar

                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True

                        #CORRIGIR PARA FICAR A OLHAR PARA O LOCAL ONDE SE VAI SENTAR QUE FOI GUARDADO ACIMA
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state = 1

########################## FIM DO CÓDIGO PARA RECEÇÃO, APRESENTAÇÃO E PROCURA DE LUGAR PARA O GUEST 1 ###########################

        #CÓDIGO RELATIVO AO GUEST 2
        elif self.state ==7:
            cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
            num_faces=self.detect_face(cv2_img)
            self.contador_conhecidos = 0
            #Código para ver se encontro faces
            if num_faces != None:
                if num_faces == 1:
                    #Guarda as coordenadas do sofá uma vez que como só tem uma pessoa é um local onde se pode sentar
                    self.place_to_sit = self.sofa_coordinates
                    self.flag_place_to_sit=True
                    pessoas_detetadas=self.recognizeGuest(cv2_img)
                    for pessoa in pessoas_detetadas:
                        self.contador_conhecidos = 0
                        if pessoa != 'unknown' and pessoa in self.names:
                            self.contador_conhecidos += 1

                    if self.contador_conhecidos == 0:
                        self.state = 8
                    elif self.contador_conhecidos == 1:
                        if pessoa in self.names == self.names[0]:
                            #CORRIGIR PARA OLHAR O GUEST NOVO

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"On the couch is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Host==True

                            #CORRIGIR PARA OLHAR PARA O SOFÁ NOVAMENTE

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                            self.state = 8

                        elif pessoa in self.names == self.names[1]:
                            #CORRIGIR PARA OLHAR O CONVIDADO 2

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"On the couch is guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)


                            #CORRIGIR PARA OLHAR PARA O SOFÁ 
                            self.neck_position_publisher.publish(self.guest_coordinates)
                            self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Guest1 = True

                            self.state = 8
                        

                elif num_faces >=2:
                    #Código para reconhecer as pessoas
                    # Contagem das pessoas conhecidas na imagem
                    

                    for pessoa in pessoas_detetadas:
                        self.contador_conhecidos = 0
                        if pessoa != 'unknown' and pessoa in self.names:
                            self.contador_conhecidos += 1

                    if self.contador_conhecidos == 0:
                        self.state = 8
                    elif self.contador_conhecidos == 1:
                        if pessoa in self.names == self.names[0]:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"On the couch is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                            #CORRIGIR PARA OLHAR PARA O SOFÁ 
                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"And the new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Host==True
                            self.state = 8
                        elif pessoa in self.names == self.names[1]:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2
                            self.neck_position_publisher.publish(self.sofa_coordinates)
                            self.speech_str.command = f"On the couch is guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)

                            #CORRIGIR PARA OLHAR PARA O SOFA
                            self.neck_position_publisher.publish(self.guest_coordinates)
                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Guest1 = True
                            self.contador_conhecidos=0
                            self.state = 8

                    elif self.contador_conhecidos == 2:
                        #CORRIGIR PARA OLHAR PARA O GUEST2
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"On the couch are host {self.names[0]} and his favorite drink is {self.drinks[0]} and the guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        #CORRIGIR PARA OLHAR PARA O SOFÁ
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"And the new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host==True
                        self.flag_Guest1 = True
                        self.contador_conhecidos=0
                        self.state = 8
            else:
                self.place_to_sit=self.sofa_coordinates
                self.flag_place_to_sit = True
                self.state = 8

        elif self.state == 8:
            #CORRIGIR PARA MOVER PARA A CADEIRA 1

            self.neck_position_publisher.publish(self.place_to_sit_neck)
            if self.flag_Host == True:
                if self.flag_Guest1 == True:
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces=self.detect_face(cv2_img)
                    if self.num_faces != 0 :
                        self.state = 9
                    else:
                        self.place_to_sit = self.chair1_coordinates
                        self.flag_place_to_sit = True


                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state = 11

                else:
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces=self.detect_face(cv2_img)
                    if self.num_faces != 0 :
                        for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            self.state = 9
                        elif self.contador_conhecidos != 0:
                            if pessoa in self.names == self.names[2]:
                                #CORRIGIR PARA OLHAR PARA O GUEST2

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"On the chair is guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA O GUEST 1
                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Guest1==True
                                self.contador_conhecidos=0
                                self.state = 9
                    else:
                        if self.flag_place_to_sit == True:
                            self.state = 9

                        else:
                            self.place_to_sit = self.chair1_coordinates
                            self.flag_place_to_sit = True
                            self.state = 9
                            


            else:
                if self.flag_Guest1 == True:
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces=self.detect_face(cv2_img)
                    if self.num_faces != 0 :
                        pessoas_detetadas=self.recognizeGuest(cv2_img)
                        for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            self.state = 9
                        elif self.contador_conhecidos == 1:
                            #CORRIGIR PARA OLHAR PARA O GUEST2

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"On the couch is host {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_Host==True

                            #CORRIGIR PARA OLHAR PARA O HOST QUE ESTÁ NA CADEIRA 1

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.contador_conhecidos=0
                        
                            self.state = 9
                    else:
                        if self.flag_place_to_sit == True:
                            self.state = 9
                        else:
                            self.place_to_sit = self.chair1_coordinates
                            self.flag_place_to_sit = True
                            self.state = 9
                    
                else:         
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces=self.detect_face(cv2_img)
                    if self.num_faces != 0 :
                        pessoas_detetadas=self.recognizeGuest(cv2_img)
                        for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            self.state = 9
                        elif self.contador_conhecidos == 1:
                            if pessoa in self.names == self.names[0]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                            
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA A CADEIRA 1 ONDE ESTÁ O HOST

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Host==True
                                self.contador_conhecidos=0
                                self.state = 9

                            elif pessoa in self.names == self.names[1]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"On the couch is guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA O GUEST 1 QUE ESTÁ SENTADO
                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Guest1 = True
                                self.contador_conhecidos=0
                                self.state = 9   

                    else:
                        if self.flag_place_to_sit == True:
                            self.state = 9
                        else:
                            self.place_to_sit = self.chair1_coordinates
                            self.flag_place_to_sit = True
                            self.state = 9

        elif self.state == 9:     

            if self.flag_Guest1 and self.flag_Host and self.place_to_sit:
                self.state == 11
            else:
                #CORRIGIR PARA OLHAR PARA A CADEIRA 2

                self.neck_position_publisher.publish(self.place_to_sit_neck)

                cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                num_faces=self.detect_face(cv2_img)
                pessoas_detetadas=self.recognizeGuest(cv2_img)
                if self.num_faces == 0 :
                    self.state = 10
                else:
                    if self.flag_Host == True:
                        if self.flag_Guest1 == True:
                            self.speech_str.command = "Please stand. I couldn't find an empty seat."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 11
                        else:
                            
                            for pessoa in pessoas_detetadas:
                                self.contador_conhecidos = 0
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1

                            if self.contador_conhecidos == 0:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Guest1==True
                                self.state=11
                            elif self.contador_conhecidos == 1:
                                if pessoa in self.names == self.names[1]:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"On the chair is guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        self.speech_str.language = 'en'
                                        self.audio_command_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        self.speech_str.language = 'en'
                                        self.audio_command_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        self.speech_str.language = 'en'
                                        self.audio_command_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA O GUEST 1 NA CADEIRA 2
                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                    self.flag_Guest1==True
                                    self.contador_conhecidos=0
                                    self.state = 11
                         
                    else:
                        for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1

                        if self.contador_conhecidos == 0:
                            if self.flag_Guest1 == True:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)

                                self.neck_position_publisher.publish(self.guest_coordinates)
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Host==True
                                self.state=11
                            else:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)

                                self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)    
                                
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Guest1==True
                                self.state=11

                        elif self.contador_conhecidos == 1:
                            if pessoa in self.names == self.names[0]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA O HOST

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Host==True
                                self.state=11

                            elif pessoa in self.names == self.names[1]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    self.speech_str.language = 'en'
                                    self.audio_command_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA O LOCAL ONDE ESTÁ O GUEST 1
                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                self.speech_str.language = 'en'
                                self.audio_command_publisher.publish(self.speech_str)
                                self.flag_Guest1= True
                                self.state=11

               

        elif self.state == 10:
            if self.flag_place_to_sit == True:
                if self.flag_Guest1 == True:
                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                    self.neck_position_publisher.publish(self.navigation_neck)
                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)

                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.flag_Host=True
                    self.flag_Guest1= True

                    self.state=11
                else:
                    if self.flag_Host == True:
                        #CORRIGIR PARA OLHAR PARA O GUEST 2

                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Guest1= True
                        self.state=11
                    else:
                        #CORRIGIR PARA OLHAR PARA O GUEST 2
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host=True
                        self.flag_Guest1= True
                        self.state=11
            else:
                self.place_to_sit = self.chair2_coordinates
                self.flag_place_to_sit = True
                if self.flag_Guest1 == True:
                    if self.flag_Host == True:
                        #CORRIGIR PARA OLHAR PARA O LOCAL ONDE SE VAI SENTAR

                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_already_sit=True
                        self.state=11
                    else:
                        #CORRIGIR PARA OLHAR PARA O LOCAL ONDE ESTÁ O GUEST 2
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Host=True
                        self.state=11
                else:
                    if self.flag_Host == True:
                        #CORRIGIR PARA OLHAR PARA O GUEST 2
                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        
                        self.flag_Guest1= True
                        self.state=11
                    else:
                        #CORRIGIR PARA OLHAR PARA O GUEST 2
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            self.speech_str.language = 'en'
                            self.audio_command_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                        #CORRIGIR PARA OLHAR PARA O LOCAL ONDE SE VAI SENTAR
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                        self.flag_Guest1=True
                        self.flag_Host=True
                        self.flag_already_sit=True
                        self.state=11

        elif self.state == 11:
            if self.flag_Host == True:
                if self.flag_Guest1 == True:
                    self.state=12
                else:
                    #CORRIGIR PARA OLHAR PARA UMA POSIÇÃO
                    self.neck_position_publisher.publish(self.navigation_neck)

                    self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    if self.get_caract==3:
                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                    elif self.get_caract==2:
                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                    elif self.get_caract==1:
                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.flag_Guest1=True
                    self.state=12
            else:
                if self.flag_Guest1 == True:
                    #CORRIGIR PARA OLHAR PARA UMA POSIÇÃO
                    self.neck_position_publisher.publish(self.navigation_neck)

                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)

                    self.speech_str.command = f"The new guest is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.flag_Host=True
                    self.state=12

                else:
                    self.neck_position_publisher.publish(self.navigation_neck)
                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)

                    self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    if self.get_caract==3:
                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                    elif self.get_caract==2:
                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)
                    elif self.get_caract==1:
                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                        self.speech_str.language = 'en'
                        self.audio_command_publisher.publish(self.speech_str)

                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.flag_Host=True
                    self.flag_Guest1=True
                    self.state=12

        elif self.state == 12:
            if self.flag_place_to_sit == True:
                if self.flag_already_sit == True:
                    self.speech_str.command = f"Thank you. I finished my receptionist task."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                else:
                    #CORRIGIR PARA SE ORIENTAR PARA A ZONA DO LUGAR DE SENTAR

                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                    self.speech_str.command = f"Please sit down on the place that I'm looking for."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
                    self.speech_str.command = f"Thank you. I finished my receptionist task."
                    self.speech_str.language = 'en'
                    self.audio_command_publisher.publish(self.speech_str)
            else:
                self.speech_str.command = "Please stand. I couldn't find an empty seat."
                self.speech_str.language = 'en'
                self.audio_command_publisher.publish(self.speech_str)
                self.speech_str.command = f"Thank you. I finished my receptionist task."
                self.speech_str.language = 'en'
                self.audio_command_publisher.publish(self.speech_str)



    def detect_face(self, image):
        mp_face_detection = mp.solutions.face_detection.FaceDetection()

        # Converte a imagem para o formato RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Detecta rostos na imagem
        results = mp_face_detection.process(image_rgb)

        num_faces= len(results.detections)
        # Verifica se foram encontrados rostos
        if not results.detections:
            return None

        # Desenha um retângulo em torno do rosto detectado
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            height, width, _ = image.shape
            start_x = int(bbox.xmin * width)
            start_y = int(bbox.ymin * height)
            end_x = int((bbox.xmin + bbox.width) * width)
            end_y = int((bbox.ymin + bbox.height) * height)
            cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)

        return num_faces
    

    def highlightFace(self, net, frame, conf_threshold=0.7):
        frameOpencvDnn = frame.copy()
        frameHeight = frameOpencvDnn.shape[0]
        frameWidth = frameOpencvDnn.shape[1]
        blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)

        net.setInput(blob)
        detections = net.forward()
        faceBoxes = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf_threshold:
                x1 = int(detections[0, 0, i, 3] * frameWidth)
                y1 = int(detections[0, 0, i, 4] * frameHeight)
                x2 = int(detections[0, 0, i, 5] * frameWidth)
                y2 = int(detections[0, 0, i, 6] * frameHeight)
                faceBoxes.append([x1, y1, x2, y2])
                cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight / 150)), 8)
        return frameOpencvDnn, faceBoxes

    
    def detectGenderAge(self, image):
        parser = argparse.ArgumentParser()
        parser.add_argument('--image')

        args = parser.parse_args()

        self.faceNet = cv2.dnn.readNet(self.faceModel, self.faceProto)
        self.ageNet = cv2.dnn.readNet(self.ageModel, self.ageProto)
        self.genderNet = cv2.dnn.readNet(self.genderModel, self.genderProto)

        padding = 20
        flag_age_gender = True
        while flag_age_gender:
            flag_age_gender = False
            # hasFrame, frame = video.read()
            # frame = cv2.imread(image)
            # if not hasFrame:
            #   cv2.waitKey()
            #  break

            resultImg, faceBoxes = self.highlightFace(self.faceNet, image)
            if not faceBoxes:
                print("No face detected")
                age = ''
                gender = ''
                error = 1
                break
            else:
                error = 0
                for faceBox in faceBoxes:
                    face = image[max(0, faceBox[1] - padding):
                                min(faceBox[3] + padding, image.shape[0] - 1), max(0, faceBox[0] - padding)
                                                                                :min(faceBox[2] + padding, image.shape[1] - 1)]

                    blob = cv2.dnn.blobFromImage(face, 1.0, (227, 227), self.MODEL_MEAN_VALUES, swapRB=False)
                self.genderNet.setInput(blob)
                genderPreds = self.genderNet.forward()
                gender = self.genderList[genderPreds[0].argmax()]
                #print("Gender : {}, conf = {:.3f}".format(gender, genderPreds[0].max()))

                self.ageNet.setInput(blob)
                agePreds = self.ageNet.forward()
                age = self.ageList[agePreds[0].argmax()]
                #print("Age Output : {}".format(agePreds))
                #print("Age : {}, conf = {:.3f}".format(age, agePreds[0].max()))

        return age, gender,error

    def detectrace(self, image):
            
            color_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img=cv2.resize(color_img,(224,224))
            normalized_img = img / 255.0
            expanded_img = np.expand_dims(normalized_img, axis=0)
            # Realizar a previsão usando o modelo carregado
            predictions = self.model_loaded.predict(expanded_img)  # Substitua ... pelos dados da imagem pré-processada
            
            # Obter a previsão da raça
            raça_predominante_index = np.argmax(predictions, axis=1)
            raça_predominante = ['Asian', 'Indian', 'Black or African Descendent', 'White', 'Middle Eastern', 'Latino Hispanic'][raça_predominante_index[0]]
            
            # Verificar se a previsão está disponível
            if raça_predominante is not None:
                raça_dominante = raça_predominante
                error = 0
            else:
                raça_dominante = None
                error = 1
            
            return raça_dominante, error
  

    def findEncodings(self,images):
        encodeList = []
        for img in images:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            encode = face_recognition.face_encodings(img)[0]
            encodeList.append(encode)
        return encodeList


    def markAttendance(self,name):
        with open('Attendance.csv', 'r+') as f:
            myDataList = f.readlines()
            nameList = []
            for line in myDataList:
                entry = line.split(',')
                nameList.append(entry[0])
                if name not in nameList:
                    now = datetime.now()
                    dtString = now.strftime('%H:%M:%S')
                    f.writelines(f'n{name},{dtString}')


    
    
    def recognizeGuest(self,image):
        encodeListKnown = self.findEncodings(self.images)
        pessoas = []
        imgS = cv2.resize(image, (0, 0), None, 0.25, 0.25)
        imgS = cv2.cvtColor(imgS, cv2.COLOR_BGR2RGB)

        facesCurFrame = face_recognition.face_locations(imgS)
        num_faces = len(facesCurFrame)
        encodesCurFrame = face_recognition.face_encodings(imgS, facesCurFrame)

        for encodeFace, faceLoc in zip(encodesCurFrame, facesCurFrame):

            matches = face_recognition.compare_faces(encodeListKnown, encodeFace)
            faceDis = face_recognition.face_distance(encodeListKnown, encodeFace)
            matchIndex = np.argmin(faceDis)
            name = self.classNames[matchIndex].upper()

            y1, x2, y2, x1 = faceLoc
            center = (x1 + x2) // 2

            if matches[matchIndex]:
                self.markAttendance(name)
                pessoas.append([name])
                #pessoas.append([center, name])
            else:
                self.markAttendance(name)
                pessoas.append(['Unknown'])
                #pessoas.append([center, 'Unknown'])
        return pessoas
        
        


    def get_color_image_callback(self, img: Image):
            self.colour_img = img
            # print(img)
            print("---")
            self.get_logger().info('Receiving color video frame')
            current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
            cv2.imshow("c_camera", current_frame)   
            cv2.waitKey(1)

    def get_speech_callback(self, keywords : String):
        if keywords == 'ERROR':
            self.speech_str.command = "I did not understand what you said. What's your name and favourite drink?"
            self.speech_str.language = 'en'
            self.audio_command_publisher.publish(self.speech_str)

            #Informo o Charmie que acabei de falar e posso começar a ouvir
            self.flag_speaker_publisher.publish(True)
        else:
            self.keywords = keywords
            #pegar na string e dividir em nomes e bebidas e depois dar append para guardar o host guest1 e guest2
            self.keyword_list = keywords.split(" ")
            if len(self.keyword_list) > 0:
                self.names.append(self.keyword_list[0])
            if len(self.keyword_list) > 1:
                self.drinks.append(self.keyword_list[1])

                # Só para testar
                #print("Array 1:", array1)
                #print("Array 2:", array2)



    def get_speech_callback(self, speech: String):
        speech_str = RobotSpeech()

        self.get_speech = speech

        if speech.data == "ERROR":
            speech_str.command = "I could not understand what you said, can you repeat please?"
            speech_str.language = 'en'
            self.speaker_publisher.publish(speech_str)
        else:
            # speech_str.command = "Hello my name is Tiago."
            # speech_str.language = 'en'
            # speech_str.command = "Bom dia, o meu nome é Tiago e gosto de Robôs. Já agora, qual é a comida na cantina hoje?"
            # speech_str.command = "Qual é a comida na cantina hoje?"
            speech_str.command = speech.data
            speech_str.language = 'en'
            self.speaker_publisher.publish(speech_str)

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag: ", state.data)
        self.start_audio()


    def start_audio(self):
        self.audio_command_publisher.publish(self.speech_type)


    def get_odometry_robot_callback(self, odom:Odometry):
        self.robot_current_position = odom


def main(args=None):
    rclpy.init(args=args)
    node = ReceptionistNode()
    rclpy.spin(node)
    rclpy.shutdown()