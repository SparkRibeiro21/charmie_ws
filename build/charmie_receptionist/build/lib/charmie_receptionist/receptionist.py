#!/usr/bin/env python 3

import rclpy
from rclpy.node import Node
import threading

from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Int16, Bool, String
import cv2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge,  CvBridgeError
from charmie_interfaces.msg import RobotSpeech, SpeechType, TarNavSDNL
#from charmie_debug.debug_main import get_color_image_callback

import numpy as np
import face_recognition
import time
import os
import tensorflow as tf
from datetime import datetime

import mediapipe as mp

import argparse
from mediapipe.python.solutions.drawing_utils import _normalized_to_pixel_coordinates

class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("ReceptionistNode")
        self.get_logger().info("Initiliased Receptionist Node")

        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_pos", 10)
        self.neck_error_publisher = self.create_publisher(Pose2D, "neck_error", 10)
        #self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback, 10)
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
        
    
        # Speaker
        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Audio
        self.audio_command_publisher = self.create_publisher(SpeechType, "audio_command", 10)
        #self.flag_listening_subscriber = self.create_subscription(Bool, "flag_listening", self.flag_listening_callback, 10)
        self.get_speech_subscriber = self.create_subscription(String, "get_speech", self.get_speech_callback, 10)

        # Face Shining RGB
        self.face_publisher = self.create_publisher(Int16, "face_command", 10)

        # Odometry
        self.odometry_subscriber = self.create_subscription(Odometry, "odom",self.get_odometry_robot_callback, 10)

        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        #self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)

        
        # Timer
        # self.create_timer(0.1, self.timer_callback)

        # Changing Variables
        self.state = 0
        
        #self.state_aux = 0
        #self.state_ant = 0
        #self.first_time_speech_state = True
        #self.prox_state = 0

        #Coordenadas onde o robô começa
        self.begin_coordinates = Pose2D()
        self.begin_coordinates.x = 0.0
        self.begin_coordinates.y = 0.0


        self.begin_coordinates_orientation = Pose2D()
        self.begin_coordinates_orientation.x = 0.0
        self.begin_coordinates_orientation.y = 0.0


        #Coordenadas da porta 
        self.door_coordinates = Pose2D()
        self.door_coordinates.x = 0.0
        self.door_coordinates.y = 1.95

        self.door_coordinates_orientation = Pose2D()
        self.door_coordinates_orientation.x = 1.8
        self.door_coordinates_orientation.y = 1.95

        self.door_second_coordinates = Pose2D()
        self.door_second_coordinates.x = 0.0
        self.door_second_coordinates.y = 1.75
        
        #Coordenadas do sitio onde se vai sentar. Variavel apenas para guardar as coordenadas certas  
        self.place_to_sit = Pose2D()
        self.place_to_sit.x = 0.0
        self.place_to_sit.y = 0.0

        self.place_to_sit_orientation = Pose2D()
        self.place_to_sit_orientation.x = 0.0
        self.place_to_sit_orientation.y = 0.0


        #Coordenadas de onde vai para olhar para todos os locais sentados
        self.find_coordinates = Pose2D()
        self.find_coordinates.x = 1.0
        self.find_coordinates.y = -4.0

        #Coordenadas para estar no centro e ver para o todo o lado
        self.find_coordinates_orientation = Pose2D()
        self.find_coordinates_orientation.x = -1.00
        self.find_coordinates_orientation.y = -5.80

        #Coordenadas para o target intermédio de quando o robô tem de retornar à porta
        self.return_door_coordinates = Pose2D()
        self.return_door_coordinates.x = 0.0
        self.return_door_coordinates.y = -3.0

        #Coordenadas do sofá
        self.sofa_coordinates = Pose2D()
        self.sofa_coordinates.x = -3.00
        self.sofa_coordinates.y = -4.50

        #Coordenadas para orientar para o sofá
        #self.sofa_coordinates_orientation = Pose2D()
        #self.sofa_coordinates_orientation.x = -2.60
        #self.sofa_coordinates_orientation.y = -4.20

        #Coordenadas da cadeira 1
        self.chair1_coordinates = Pose2D()
        self.chair1_coordinates.x = -3.00
        self.chair1_coordinates.y = -6.10

        #Coordenadas para orientar para a cadeira 1
        #self.chair1_coordinates_orientation = Pose2D()
        #self.chair1_coordinates_orientation.x = 7.7
        #self.chair1_coordinates_orientation.y = -3.36

        #Coordenadas da cadeira 2
        self.chair2_coordinates = Pose2D()
        self.chair2_coordinates.x = 0.5
        self.chair2_coordinates.y = -6.10

        #Coordenadas para orientar para a cadeira 2
        #self.chair2_coordinates_orientation = Pose2D()
        #self.chair2_coordinates_orientation.x = 6.47
        #self.chair2_coordinates_orientation.y = 0.0


        #Temos de pedir para ficar sempre no mesmo sítio
        #Coordenadas do guest de forma a nos conseguirmos orientar para ele
        #self.guest_coordinates = Pose2D()
        #self.guest_coordinates.x = 6.17
        #self.guest_coordinates.y = -1.06


        self.coordinates = TarNavSDNL()
        #Código para ele olhar para a cara da pessoa e centrar a cara da pessoa na imagem
        self.door_neck_coordinates = Pose2D()
        self.door_neck_coordinates.x = 180.0
        self.door_neck_coordinates.y = 200.0
        
        #Definir como se estivesse a olhar ligeiramente para baixo - Perceber o que precisamos que olhe para baixo para detetarmos tudo direito
        self.place_to_sit_neck = Pose2D()
        self.place_to_sit_neck.x = 180.0
        self.place_to_sit_neck.y = 160.0

        self.chair1_neck = Pose2D()
        self.chair1_neck.x = 200.0
        self.chair1_neck.y = 180.0

        self.chair2_neck = Pose2D()
        self.chair2_neck.x = 90.0
        self.chair2_neck.y = 180.0

        self.sofa_neck = Pose2D()
        self.sofa_neck.x = 230.0
        self.sofa_neck.y = 180.0

        self.guest_neck = Pose2D()
        self.guest_neck.x = 90.0
        self.guest_neck.y = 180.0


        self.navigation_neck = Pose2D()
        self.navigation_neck.x = 180.0
        self.navigation_neck.y = 150.0

        self.talk_neck = Pose2D()
        self.talk_neck.x = 180.0
        self.talk_neck.y = 180.0


        self.neck_pos = Pose2D()

        self.br = CvBridge()


        self.flag_speech_done = False
        self.flag_navigation_done = False
        self.flag_audio_done = False
        # self.keyword_list = []
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

        self.drinks = ["7up"]
        self.names = ["Jack"]
        
        self.get_caract = 0
        self.caracteristics = []
        self.filename = String()


        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        # MODEL_MEAN_VALUES = (78, 87, 114)

        self.ageList = ['(0-2)', '(4-6)', '(8-12)', '(15-22)', '(23-32)', '(38-43)', '(48-55)', '(60-100)']
        self.genderList = ['Male', 'Female']


        self.pasta_imagens_conhecidas = "/home/charmie/charmie_ws/build/charmie_receptionist/charmie_receptionist/images"
        
        self.imagens_conhecidas = os.listdir(self.pasta_imagens_conhecidas)

        self.codificacoes_conhecidas = []
        self.nomes_conhecidos = []
        

        self.images = []
        self.classNames = []
        
        self.model_loaded = tf.keras.models.load_model('/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_raca.h5')

        self.faceProto = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector_uint8.pb"
        self.faceModel = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector.pbtxt"
        self.ageProto = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_age.prototxt"
        self.ageModel = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/age_net.caffemodel"
        self.genderProto = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_gender.prototxt"
        self.genderModel = "~/charmie_ws/src/charmie_receptionist/charmie_receptionist/gender_net.caffemodel"


    def detect_face(self, image):
        mp_face_detection = mp.solutions.face_detection.FaceDetection()

        # Converte a imagem para o formato RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Detecta rostos na imagem
        try:
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
        except:
            num_faces = 0

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

            resultImg, faceBoxes = self.highlightFace(self.faceNet, image)
            if not faceBoxes:
                #print("No face detected")
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
  

    def reconhecimento_facial(self, imagem, imagens_conhecidas):
        # Carregar a imagem de entrada
        imagem_entrada = face_recognition.load_image_file(imagem)

        # Codificar a imagem de entrada
        codificacao_entrada = face_recognition.face_encodings(imagem_entrada)[0]

        # Carregar as imagens conhecidas
        
        for imagem_conhecida in imagens_conhecidas:
            imagem = face_recognition.load_image_file(imagem_conhecida)
            codificacao = face_recognition.face_encodings(imagem)[0]
            self.codificacoes_conhecidas.append(codificacao)
            nome_conhecido = os.path.basename(imagem_conhecida).split('.')[0]
            self.nomes_conhecidos.append(nome_conhecido)


        # Comparar a codificação da imagem de entrada com as codificações conhecidas
        resultados = face_recognition.compare_faces(self.codificacoes_conhecidas, codificacao_entrada)

        # Encontrar os índices das correspondências verdadeiras
        indices_correspondencias = [i for i, resultado in enumerate(resultados) if resultado]

        # Obter os nomes correspondentes aos índices
        nomes_correspondentes = [self.nomes_conhecidos[i] for i in indices_correspondencias]

        return nomes_correspondentes
        
        

    def get_color_image_callback(self, img: Image):
        # self.colour_img = img
        # print(img)
        # print("---")
        # self.get_logger().info('Receiving color video frame')
        # current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        # cv2.imshow("c_camera", current_frame)   
        # cv2.waitKey(1)
        pass


    #Erro


    """

            #pegar na string e dividir em nomes e bebidas e depois dar append para guardar o host guest1 e guest2
            self.keyword_list = keywords.split(" ")
            if len(self.keyword_list) > 0:
                self.names.append(self.keyword_list[0])
            if len(self.keyword_list) > 1:
                self.drinks.append(self.keyword_list[1])

                # Só para testar
                #print("Array 1:", array1)
                #print("Array 2:", array2)

    """
        
    def get_speech_callback(self, keywords : String):
        print("Received Audio:", keywords.data)
        self.keywords = keywords
        self.flag_audio_done = True

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
    
    def flag_pos_reached_callback(self, state: Bool):
        print("Received Navigation Flag:", state.data)
        self.flag_navigation_done = True
    
    def start_audio(self):
        self.audio_command_publisher.publish(self.speech_type)


    def get_odometry_robot_callback(self, odom:Odometry):
        self.robot_current_position = odom



def main(args=None):
    rclpy.init(args=args)
    node = ReceptionistNode()
    th_main = threading.Thread(target=thread_main_receptionist, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_receptionist(node: ReceptionistNode):
    main = ReceptionistMain(node)
    main.main()


class ReceptionistMain():
    
    def __init__(self, node: ReceptionistNode):
        self.node = node
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False

    
    def wait_for_end_of_navigation(self):
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False


    def wait_for_end_of_audio(self):
        while not self.node.flag_audio_done:
            pass
        self.node.flag_audio_done = False
        self.node.flag_speech_done = False  
        # Since audio also uses speaker for errors

    """         if self.node.keywords == 'ERROR':

            print("deu erro", self.node.keywords)

            self.node.speech_str.command = "I did not understand what you said. Could you please repeat?"
            self.node.speaker_publisher.publish(self.speech_str)
            self.wait_for_end_of_speaking()

            #Informo o Charmie que acabei de falar e posso começar a ouvir
            self.node.audio_command_publisher.publish(self.node.speech_type)
            # aqui temos recursividade, será o ideal para este caso???
            self.wait_for_end_of_audio()        
        else:
            print(self.node.keywords) 
    """


    def main(self):
        print("IN NEW MAIN")

        while True:

            
            """if self.node.state == 0:
                self.node.get_logger().info("estado 0")
                #self.node.neck_position_publisher.publish(self.node.navigation_neck)
                #vai para a porta

                #self.node.coordinates.move_target_coordinates = self.node.door_coordinates
                #self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                #self.node.coordinates.flag_not_obs = False
                #self.node.target_position_publisher.publish(self.node.coordinates)
                #self.wait_for_end_of_navigation()

                #CORRIGIR CÓDIGO PARA SE MOVER PARA A PORTA E FICAR A OLHAR PARA A PORTA
                self.node.neck_position_publisher.publish(self.node.door_neck_coordinates)

                #Informa que está pronto para receber convidado
                self.node.speech_str.command = "I am ready to receive a new guest. Please stand in front of me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                
                 #Começar a contar o tempo
                # self.init_time = time.time()
                

                #Verificar se existe uma pessoa na porta
                cv2_img = self.node.br.imgmsg_to_cv2(self.node.colour_img, "bgr8")
                #Função de deteção facial 
                num_faces = self.node.detect_face(cv2_img)
                if num_faces != 0 :
                    
                    #Se detetar alguém o CHARMIE apresenta se e pede nome e bebida
                    self.node.speech_str.command = "Hello! My name is Charmie. What's your name and favourite drink?"
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                    
                    #Informo o Charmie que acabei de falar e posso começar a ouvir
                    #flag_spk_publisher = Bool()
                    #flag_spk_publisher.data = True
                    #self.flag_speaker_publisher.publish(flag_spk_publisher)
                    self.node.audio_command_publisher.publish(self.node.speech_type)
                    self.wait_for_end_of_audio()
                    

                    #pegar na string e dividir em nomes e bebidas e depois dar append para guardar o host guest1 e guest2
                    keyword_list= self.node.keywords.data.split(" ")
                    print(keyword_list)
                    # if len(self.node.keyword_list) > 0:
                    self.node.names.append(keyword_list[0])
                    # if len(self.node.keyword_list) > 1:
                    self.node.drinks.append(keyword_list[1])

                    print(self.node.names, self.node.drinks)
                    self.node.state=1
                #Isto vai acontecer 2x. Para o guest 1 e 2. Mas tem de ir para estados diferentes porque para o guest 2 não é 
                #necessário extrair as características
                    #self.node.count_guest+=1
                    #if self.node.count_guest==1:
                    #    self.node.state = 2
                    #if self.node.count_guest==2:
                    #    self.node.state = 3

            

            #elif self.node.state == 2:
            #    print("CHEGUEI ESTADO 2")


            #elif self.node.state == 3:
            #    print("CHEGUEI ESTADO 3") 

            elif self.node.state == 1:
                self.node.speech_str.command = "I will take you a picture. Please look at me"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                #Obter e guardar a Imagem            
                cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                self.filename = self.names[1]
                file_path = os.path.join(self.pasta_imagens_conhecidas, self.filename)
                cv2.imwrite(file_path, cv2_img) 

                # Apend ao nome e imagem guardada acima
                myList = os.listdir(self.node.pasta_imagens_conhecidas)
                for cl in myList:
                    curImg = cv2.imread(f'{self.node.pasta_imagens_conhecidas}/{cl}')
                    self.node.images.append(curImg)
                    self.node.classNames.append(os.path.splitext(cl)[0])

                #Recolho as características das imagens
                #age,gender,error_agegender = self.detectGenderAge(cv2_img)
                #resultrace, error_race = self.detectrace(cv2_img)

                #Para prevenir que o código empanque, garanto que retorno sempre alguma coisa, seja as características ou erro
                #if error_race == 0 and error_agegender == 0:
                #   self.get_caract = 3
                #    self.caracteristics.append(gender)
                #    self.caracteristics.append(age)
                #   self.caracteristics.append(resultrace)
                    
                #elif error_race == 0 and error_agegender == 1:
                #    self.get_caract = 1
                #    self.caracteristics.append(resultrace)
                    
                #elif error_race == 1 and error_agegender == 0:
                #    self.get_caract=2
                #    self.caracteristics.append(gender)
                #    self.caracteristics.append(age)
                    
                #else:
                #    self.caracteristics = "error" 
                    
                self.node.state=3
                self.node.get_logger().info("Fim do estado 2")
                #Se detetar alguém o CHARMIE apresenta se e pede nome e bebida
                #self.node.speech_str.command = "Hello! My name is Charmie. What's your name and favourite drink?"
                #self.node.speaker_publisher.publish(self.node.speech_str)
                #self.wait_for_end_of_speaking()
                
                #Informo o Charmie que acabei de falar e posso começar a ouvir
                #flag_spk_publisher = Bool()
                #flag_spk_publisher.data = True
                #print("mandei audio comm")
                #self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.wait_for_end_of_audio()
                

                #pegar na string e dividir em nomes e bebidas e depois dar append para guardar o host guest1 e guest2
                #keyword_list= self.node.keywords.data.split(" ")
                #print(keyword_list)
                #self.node.names.append(keyword_list[0])
                #self.node.drinks.append(keyword_list[1])
            
            elif self.node.state == 3:
                print("CHEGUEI ESTADO 3") 
            """
            #Estado comum aos dois Guests
            if self.node.state == 0:
                #Informa que está pronto para começar a tarefa
                self.node.get_logger().info("estado 0")

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                self.node.speech_str.command = "Hello! I am ready to start the receptionist task! Here I go"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.state = 1


            #Estado para verificar se está alguém na porta e ficar a repetir de 5 em 5 segundos enquanto não estiver lá ninguém
            #Estado para o Guest 1
            elif self.node.state == 1:    
                self.node.get_logger().info("estado 1")
                self.node.neck_position_publisher.publish(self.node.navigation_neck)
                #vai para a porta

                self.node.coordinates.move_target_coordinates = self.node.door_coordinates
                self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                self.node.coordinates.flag_not_obs = False
                self.node.target_position_publisher.publish(self.node.coordinates)
                self.wait_for_end_of_navigation()

                #CORRIGIR CÓDIGO PARA SE MOVER PARA A PORTA E FICAR A OLHAR PARA A PORTA
                self.node.neck_position_publisher.publish(self.node.door_neck_coordinates)

                #Informa que está pronto para receber convidado
                self.node.speech_str.command = "I am ready to receive a new guest. Please stand in front of me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                
                 #Começar a contar o tempo
                # self.init_time = time.time()
                

                #Verificar se existe uma pessoa na porta
                # cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                #Função de deteção facial 
                # num_faces = self.detect_face(cv2_img)
                # if num_faces != 0 :
                    
                #Se detetar alguém o CHARMIE apresenta se e pede nome e bebida
                self.node.speech_str.command = "Hello! My name is Charmie. What's your name and favourite drink?"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                
                #Informo o Charmie que acabei de falar e posso começar a ouvir
                #flag_spk_publisher = Bool()
                #flag_spk_publisher.data = True
                #self.flag_speaker_publisher.publish(flag_spk_publisher)
                self.node.audio_command_publisher.publish(self.node.speech_type)
                self.wait_for_end_of_audio()
                

                #pegar na string e dividir em nomes e bebidas e depois dar append para guardar o host guest1 e guest2
                keyword_list= self.node.keywords.data.split(" ")
                print(keyword_list)
                # if len(self.node.keyword_list) > 0:
                self.node.names.append(keyword_list[0])
                # if len(self.node.keyword_list) > 1:
                self.node.drinks.append(keyword_list[1])

                print(self.node.names, self.node.drinks)

                #Isto vai acontecer 2x. Para o guest 1 e 2. Mas tem de ir para estados diferentes porque para o guest 2 não é 
                #necessário extrair as características
                self.node.count_guest+=1
                if self.node.count_guest==1:
                    self.node.state = 2
                if self.node.count_guest==2:
                    self.node.state = 3

            

                #elif self.node.state == 2:
                #    print("CHEGUEI ESTADO 2")


                #elif self.node.state == 3:
                #    print("CHEGUEI ESTADO 3") 
               


                #  else:
                #     if time.time()-self.init_time>5.0:
                #         # Repete a ação anterior
                #         self.speech_str.command = "I am ready to receive a new guest"
                #         
                #         self.speaker_publisher.publish(self.speech_str)
                #         #Dá reset ao tempo
                #         self.init_time = time.time()    
                #         self.get_logger().info("Fim do estado 1")  
                
            

            #Estado apenas para o Guest1
            elif self.node.state == 2:
                self.node.get_logger().info("estado 2")
                #CORRIGIR PARA CÓDIGO QUE AJUSTA A POSIÇÃO DA CARA PARA O CENTRO DA FACE DA PESSOA

                #Falar para pedir para olhar para camara
                self.node.speech_str.command = "I will take you a picture. Please look at me"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                #Obter e guardar a Imagem            
                #cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                #self.filename = self.names[1]
                #file_path = os.path.join(self.pasta_imagens_conhecidas, self.filename)
                #cv2.imwrite(file_path, cv2_img) 

                # Apend ao nome e imagem guardada acima
                #myList = os.listdir(self.node.pasta_imagens_conhecidas)
                #for cl in myList:
                #    curImg = cv2.imread(f'{self.node.pasta_imagens_conhecidas}/{cl}')
                #    self.node.images.append(curImg)
                #    self.node.classNames.append(os.path.splitext(cl)[0])

                #Recolho as características das imagens
                #age,gender,error_agegender = self.detectGenderAge(cv2_img)
                #resultrace, error_race = self.detectrace(cv2_img)

                #Para prevenir que o código empanque, garanto que retorno sempre alguma coisa, seja as características ou erro
                #if error_race == 0 and error_agegender == 0:
                #   self.get_caract = 3
                #    self.caracteristics.append(gender)
                #    self.caracteristics.append(age)
                #   self.caracteristics.append(resultrace)
                    
                #elif error_race == 0 and error_agegender == 1:
                #    self.get_caract = 1
                #    self.caracteristics.append(resultrace)
                    
                #elif error_race == 1 and error_agegender == 0:
                #    self.get_caract=2
                #    self.caracteristics.append(gender)
                #    self.caracteristics.append(age)
                    
                #else:
                #    self.caracteristics = "error" 
                    
                self.node.state=3
                self.node.get_logger().info("Fim do estado 2")

            #Estado comum aos 2 guests
            elif self.node.state == 3:
                self.node.get_logger().info("estado 3")
                #Agradece e pede para o seguir até à zona dos sofás
                self.node.speech_str.command = "Thank you. Please follow me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.neck_position_publisher.publish(self.node.navigation_neck)

                self.node.coordinates.move_target_coordinates = self.node.door_coordinates
                self.node.coordinates.rotate_target_coordinates = self.node.find_coordinates_orientation
                self.node.coordinates.flag_not_obs = False
                self.node.target_position_publisher.publish(self.node.coordinates)
                self.wait_for_end_of_navigation()


                #vai para o sofá
                self.node.coordinates.move_target_coordinates = self.node.find_coordinates
                self.node.coordinates.rotate_target_coordinates = self.node.find_coordinates_orientation
                self.node.coordinates.flag_not_obs = False
                self.node.target_position_publisher.publish(self.node.coordinates)
                self.wait_for_end_of_navigation()

                #CORRIGIR PARA SE MOVER PARA O SOFÁ E O PESCOÇO FICAR ORIENTADO PARA BAIXO NA DIREÇÃO DAS PESSOAS SENTADAS

                self.node.neck_position_publisher.publish(self.node.talk_neck)
                #Diz ao guest para ficar na mesma posição até lhe indicar onde se sentar
                self.node.speech_str.command = "Please stay on my left until I give you instructions on where to sit."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                if self.node.count_guest == 1:
                    self.node.state = 4
                elif self.node.count_guest == 2:
                    self.node.state = 7

                self.node.get_logger().info("Fim do estado 3")
                
            #elif self.node.state == 4:
            #    print("CHEGUEI ESTADO 4")

            elif self.node.state == 7:
                print("CHEGUEI ESTADO 7") 


            #Estado relativo ao Guest1 no SOFÁ
            elif self.node.state == 4:
                self.node.get_logger().info("estado 4")
                #Guardar Imagem            
                #cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                #num_faces=self.detect_face(cv2_img)
                #Código para ver se encontro faces
                num_faces =1
                if num_faces != 0:
                    if num_faces == 1:
                        #Guarda as coordenadas do sofá como local para sentar e ativa a flag a indicar que já tem um local para sentar
                        self.node.place_to_sit_neck = self.node.sofa_coordinates
                        #self.node.place_to_sit_orientation=self.node.sofa_coordinates_orientation
                        self.node.flag_place_to_sit=True
                        #Função para reconhecer pessoas. Retorna uma variavel pessoa que tem um centro associado a um nome
                        #nomes=self.reconhecimento_facial(cv2_img)
                        self.node.flag_Host=True
                        
                        #Se conheceu alguém é porque reconheceu o Host.
                        #if self.node.names[0] in nomes:
                        if self.node.flag_Host :
                            #OLHAR PARA A POSIÇÃO DO GUEST

                            self.node.neck_position_publisher.publish(self.node.talk_neck)

                            #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                            #self.node.coordinates.rotate_target_coordinates = self.node.guest_coordinates
                            #self.node.coordinates.flag_not_obs = False
                            #self.node.target_position_publisher.publish(self.node.coordinates)
                            #self.wait_for_end_of_navigation()


                            self.node.neck_position_publisher.publish(self.node.guest_neck)



                            #Apresentação do guest e do host
                            self.node.speech_str.command = f"The host is {self.node.names[0]} and his favorite drink is {self.node.drinks[0]}."
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking()
                            self.node.flag_Host==True

                            #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ ENQUANTO FALA, PODE SER RODAR PESCOÇO OU BASE

                            #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                            #self.node.coordinates.rotate_target_coordinates = self.node.sofa_coordinates_orientation
                            #self.node.coordinates.flag_not_obs = False
                            #self.node.target_position_publisher.publish(self.node.coordinates)
                            #self.wait_for_end_of_navigation()

                            self.node.neck_position_publisher.publish(self.node.sofa_coordinates)

                            self.node.speech_str.command = f"The guest is {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking()

                            
                            self.node.neck_position_publisher.publish(self.node.sofa_coordinates)

                            #Sentou a o convidado
                            self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.flag_already_sit = True
                            self.wait_for_end_of_speaking()
                            #Não sei o numero do estado, tou a assumir que é o último

                            self.node.neck_position_publisher.publish(self.node.navigation_neck)

                            self.node.coordinates.move_target_coordinates = self.node.find_coordinates
                            self.node.coordinates.rotate_target_coordinates = self.node.return_door_coordinates
                            self.node.coordinates.flag_not_obs = False
                            self.node.target_position_publisher.publish(self.node.coordinates)
                            self.wait_for_end_of_navigation()

                            #PONHO O A PASSAR PELAS COORDENADAS DE INICIO PORQUE NÃO CHEGA AO LOCAL DESEJADO
                            #self.node.coordinates.move_target_coordinates = self.node.door_second_coordinates
                            #self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                            #self.node.coordinates.flag_not_obs = False
                            #self.node.target_position_publisher.publish(self.node.coordinates)
                            #self.wait_for_end_of_navigation()


                            self.node.state = 1

                        else:
                            self.node.state = 5

                        
                    #elif num_faces >=2:
                        #Código para reconhecer as pessoas
                        # Contagem das pessoas conhecidas na imagem
                        
                        #nomes=self.reconhecimento_facial(cv2_img)
                        
                        
                        
                    #    if self.names[0] in nomes:
                            #Apresentação do guest e do host
                            #CORRIGIR PARA SE VIRAR PARA O GUEST

                    #        self.neck_position_publisher.publish(self.navigation_neck)

                    #        self.coordinates.move_target_coordinates = self.sofa_coordinates
                    #        self.coordinates.rotate_target_coordinates = self.guest_coordinates
                    #        self.coordinates.flag_not_obs = False

                    #        self.target_position_publisher.publish(self.coordinates)

                    #        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                    #        self.speaker_publisher.publish(self.speech_str)

                            #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ
                    #        self.coordinates.move_target_coordinates = self.sofa_coordinates
                    #        self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                    #        self.coordinates.flag_not_obs = False

                    #        self.target_position_publisher.publish(self.coordinates)

                    #        self.neck_position_publisher.publish(self.place_to_sit_neck)

                    #        self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                    #        self.speaker_publisher.publish(self.speech_str)
                    #        self.flag_Host==True
                    #        self.contador_conhecidos = 0
                    #        self.state = 5
                    #    else:
                    #        self.state = 5
            
                else: 
                    self.node.place_to_sit = self.node.sofa_coordinates
                    #self.node.place_to_sit_orientation = self.node.sofa_coordinates_orientation
                    self.node.flag_place_to_sit=True
                    #Guardar a coordenada do sofá
                    self.node.state=5
                self.node.get_logger().info("Fim do estado 4")
        
            elif self.node.state == 5:
                print("cheguei ao estado 5")
            
            
























        #Estado da fase do GUEST1 de olhar para a cadeira: ramo da esquerda
            """elif self.state == 5:
                self.get_logger().info("estado 5")
                if self.flag_Host==True:

                    self.neck_position_publisher.publish(self.navigation_neck)

                    #CORRIGIR PARA IR PARA A ZONA DA CADEIRA 1

                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                    self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                    self.coordinates.flag_not_obs = False

                    self.target_position_publisher.publish(self.coordinates)
            
                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")

                    num_faces = self.detect_face(cv2_img)
                    if num_faces !=0:

                        self.neck_position_publisher.publish(self.navigation_neck)  
                        #CORRIGIR PARA IR PARA A ZONA DA CADEIRA 2
                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                        self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                        self.coordinates.flag_not_obs = False

                        self.target_position_publisher.publish(self.coordinates)

                        self.neck_position_publisher.publish(self.place_to_sit_neck)

                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces = self.detect_face(cv2_img)
                        if num_faces !=0:
                            self.speech_str.command = "Please stand. I couldn't find an empty seat."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_place_to_sit=True
                            self.flag_already_sit=True
                            self.state = 1
                            
                        else:
                            self.place_to_sit = self.chair2_coordinates
                            self.place_to_sit_orientation = self.chair2_coordinates_orientation
                            self.flag_place_to_sit=True
                            
                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the sit in front of me."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.state = 1
                            self.flag_already_sit=True

                    else:
                        self.place_to_sit = self.chair1_coordinates
                        self.place_to_sit_orientation = self.chair1_coordinates_orientation
                        self.flag_place_to_sit=True


                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please take a sit on the Chair in front of me."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.state = 1
                        self.flag_already_sit=True
                                
                else:

                    self.neck_position_publisher.publish(self.navigation_neck)
                    #CORRIGIR PARA IR PARA A CADEIRA 1
                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                    self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                    self.coordinates.flag_not_obs = False

                    self.target_position_publisher.publish(self.coordinates)

                    self.neck_position_publisher.publish(self.place_to_sit_neck)

                    if self.flag_place_to_sit == True:
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces = self.detect_face(cv2_img)

                        if num_faces !=0:

                            nomes=self.reconhecimento_facial(cv2_img)  
                            '''for pessoa in pessoas_detetadas:
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            
                            if self.names[0] in nomes:
                                self.neck_position_publisher.publish(self.sofa_coordinates)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                                self.neck_position_publisher.publish(self.guest_coordinates)
                                self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Host = True
                                
                                #CORRIGIR PARA SE ORIENTAR PARA O LOCAL QUE FOI ANTERIORMENTE GUARDADO COMO LOCAL
                                #PARA SENTAR

                                self.coordinates.move_target_coordinates = self.place_to_sit
                                self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)

                                self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_already_sit = True
                                self.contador_conhecidos = 0
                                self.state = 1
                            else:
                                self.state = 6

                        else:
                            self.state=6

                    else:
                        
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces = self.detect_face(cv2_img)
                        if num_faces !=0:

                            nomes=self.reconhecimento_facial(cv2_img)
                            '''for pessoa in pessoas_detetadas:
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            
                            if self.names[0] in nomes:
                                self.neck_position_publisher.publish(self.sofa_coordinates)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                                self.neck_position_publisher.publish(self.guest_coordinates)
                                self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Host=True

                                
                                self.state = 6
                            else:
                                self.state = 6

                        else:
                            self.place_to_sit = self.chair1_coordinates
                            self.place_to_sit_orientation = self.chair1_coordinates_orientation
                            self.flag_place_to_sit = True
                            self.state = 6
                self.get_logger().info("Fim do estado 5")

                        
            elif self.state == 6:
                self.get_logger().info("estado 6")
                self.neck_position_publisher.publish(self.navigation_neck)
                #CORRIGIR PARA ANDAR PARA A CADEIRA 2
                self.coordinates.move_target_coordinates = self.chair2_coordinates
                self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                self.coordinates.flag_not_obs = False

                self.target_position_publisher.publish(self.coordinates)

                self.neck_position_publisher.publish(self.place_to_sit_neck)

                cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                num_faces = self.detect_face(cv2_img)
                if num_faces !=0:
                    if self.flag_Host == True:
                        self.speech_str.command = "Please stand. I couldn't find an empty seat."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.flag_place_to_sit=True
                        self.flag_already_sit=True
                        self.state = 1
                    else:

                        nomes=self.reconhecimento_facial(cv2_img)
                        '''for pessoa in pessoas_detetadas:
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1'''

                        

                        if self.names[0] in nomes:
                            #CORRIGIR PARA SE ORIENTAR PARA O GUEST

                            self.neck_position_publisher.publish(self.navigation_neck)

                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                        
                            self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            #CORRIGIR PARA OLHAR PARA O HOST
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host==True

                            
                            if self.flag_place_to_sit == True:
                                #CORRIGIR PARA OLHAR PARA O LOCAL A SENTAR
                                self.coordinates.move_target_coordinates = self.place_to_sit
                                self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_already_sit=True
                                self.state = 1
                            else:

                                #CORRIGIR PARA FICAR A OLHAR PARA ONDE O GUEST ESTÁ
                                self.neck_position_publisher.publish(self.navigation_neck)

                                self.coordinates.move_target_coordinates = self.chair2_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.speech_str.command = "Please stand. I couldn't find an empty seat."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_place_to_sit=True
                                self.flag_already_sit=True
                                self.state = 1
                            self.contador_conhecidos=0 
                        else:

                            #Apresentação do guest e do host para o ar
                            #CORRIGIR PARA OLHAR PARA O GUEST
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.navigation_neck)

                            self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)


                            self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host==True

                            self.speech_str.command = "Please stand. I couldn't find an empty seat."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_place_to_sit=True
                            self.flag_already_sit=True
                            self.state = 1   
                else:
                    if self.flag_place_to_sit == True:
                    if self.flag_Host == True:
                            #CORRIGIR PARA FICAR A OLHAR PARA O LUGAR ONDE SE VAI SENTAR A PESSOA
                            self.coordinates.move_target_coordinates = self.place_to_sit
                            self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 1
                    else:
                            #CORRIGIR PARA FICAR A OLHAR PARA O GUEST QUE ESTÁ EM PÉ AINDA 
                            #Apresentação do guest e do host para o ar

                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host==True
                            
                            #CORRIGIR PARA FICAR A OLHAR PARA O LUGAR ONDE SE VAI SENTAR
                            self.coordinates.move_target_coordinates = self.place_to_sit
                            self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 1
                        
                    else:
                        self.place_to_sit = self.chair2_coordinates
                        self.place_to_sit_orientation = self.chair2_coordinates_orientation
                        self.flag_place_to_sit = True
                        if self.flag_Host == True:
                            self.neck_position_publisher.publish(self.place_to_sit)
                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 1
                        else:
                            #CORRIGIR PARA FICAR A OLHAR PARA ONDE O GUEST ESTÁ
                            #Apresentação do guest e do host para o ar
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The host name is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host==True

                            #CORRIGIR PARA FICAR A OLHAR PARA O LOCAL ONDE SE VAI SENTAR QUE FOI GUARDADO ACIMA
                            self.coordinates.move_target_coordinates = self.place_to_sit
                            self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)


                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the spot I'm looking at."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 1 """

    ########################## FIM DO CÓDIGO PARA RECEÇÃO, APRESENTAÇÃO E PROCURA DE LUGAR PARA O GUEST 1 ###########################

            #CÓDIGO RELATIVO AO GUEST 2
            """ elif self.state ==7:
                cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                num_faces=self.detect_face(cv2_img)
                self.contador_conhecidos = 0
                #Código para ver se encontro faces
                if num_faces != None:
                    if num_faces == 1:
                        #Guarda as coordenadas do sofá uma vez que como só tem uma pessoa é um local onde se pode sentar
                        self.place_to_sit = self.sofa_coordinates
                        self.place_to_sit_orientation = self.sofa_coordinates_orientation
                        self.flag_place_to_sit=True
                        nomes=self.reconhecimento_facial(cv2_img)
                        '''for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1'''

                        
                        if self.names in nomes:
                            if nomes == self.names[0]:
                                #CORRIGIR PARA OLHAR O GUEST NOVO
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Host==True

                                #CORRIGIR PARA OLHAR PARA O SOFÁ NOVAMENTE
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                                self.state = 8

                            elif nomes == self.names[1]:
                                #CORRIGIR PARA OLHAR O CONVIDADO 2
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)


                                #CORRIGIR PARA OLHAR PARA O SOFÁ 
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Guest1 = True

                                self.state = 8
                        else:
                            self.state = 8        
                            

                    elif num_faces >=2:
                        #Código para reconhecer as pessoas
                        # Contagem das pessoas conhecidas na imagem
                        

                        '''for pessoa in pessoas_detetadas:
                            self.contador_conhecidos = 0
                            if pessoa != 'unknown' and pessoa in self.names:
                                self.contador_conhecidos += 1'''

                        
                        if self.names in nomes:
                            if nomes == self.names[0]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                                #CORRIGIR PARA OLHAR PARA O SOFÁ 
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"And the new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Host==True
                                self.state = 8
                            elif nomes == self.names[1]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.sofa_coordinates)
                                self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                if self.get_caract==3:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                elif self.get_caract==2:
                                    self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                elif self.get_caract==1:
                                    self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)


                                #CORRIGIR PARA OLHAR PARA O SOFA
                                self.coordinates.move_target_coordinates = self.sofa_coordinates
                                self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Guest1 = True
                                self.contador_conhecidos=0
                                self.state = 8

                        elif self.contador_conhecidos == 2:
                            #CORRIGIR PARA OLHAR PARA O GUEST2
                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.coordinates.move_target_coordinates = self.sofa_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            self.speech_str.command = f"On the couch are host {self.names[0]} and his favorite drink is {self.drinks[0]} and the guest {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            #CORRIGIR PARA OLHAR PARA O SOFÁ
                            self.coordinates.move_target_coordinates = self.sofa_coordinates
                            self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            
                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"And the new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host==True
                            self.flag_Guest1 = True
                            self.contador_conhecidos=0
                            self.state = 8
                        else:
                            self.state = 8
                else:
                    self.place_to_sit=self.sofa_coordinates
                    self.place_to_sit_orientation=self.sofa_coordinates_orientation
                    self.flag_place_to_sit = True
                    self.state = 8

            elif self.state == 8:
                #CORRIGIR PARA MOVER PARA A CADEIRA 1
                self.coordinates.move_target_coordinates = self.chair1_coordinates
                self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                self.coordinates.flag_not_obs = False

                self.target_position_publisher.publish(self.coordinates)

                self.neck_position_publisher.publish(self.place_to_sit_neck)
                if self.flag_Host == True:
                    if self.flag_Guest1 == True:
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces=self.detect_face(cv2_img)
                        if self.num_faces != 0 :
                            self.state = 9
                        else:
                            self.place_to_sit = self.chair1_coordinates
                            self.place_to_sit_orientation = self.chair1_coordinates_orientation
                            self.flag_place_to_sit = True


                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state = 11

                    else:
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces=self.detect_face(cv2_img)
                        if self.num_faces != 0 :
                            '''for pessoa in pessoas_detetadas:
                                self.contador_conhecidos = 0
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            
                            if self.names in nomes:
                                if nomes == self.names[2]:
                                    #CORRIGIR PARA OLHAR PARA O GUEST2
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates_orientation
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA O GUEST 1
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                                    self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Guest1==True
                                    self.contador_conhecidos=0
                                    self.state = 9
                            else:
                                self.state = 9
                        else:
                            if self.flag_place_to_sit == True:
                                self.state = 9

                            else:
                                self.place_to_sit = self.chair1_coordinates
                                self.place_to_sit_orientation = self.chair1_coordinates_orientation
                                self.flag_place_to_sit = True
                                self.state = 9
                                


                else:
                    if self.flag_Guest1 == True:
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces=self.detect_face(cv2_img)
                        if self.num_faces != 0 :
                            nomes =self.reconhecimento_facial(cv2_img)
                            '''for pessoa in pessoas_detetadas:
                                self.contador_conhecidos = 0
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            
                            if self.names[0] == nomes:
                                #CORRIGIR PARA OLHAR PARA O GUEST2
                                self.coordinates.move_target_coordinates = self.chair1_coordinates
                                self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)
                                self.neck_position_publisher.publish(self.navigation_neck)
                                self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_Host==True

                                #CORRIGIR PARA OLHAR PARA O HOST QUE ESTÁ NA CADEIRA 1
                                self.coordinates.move_target_coordinates = self.chair1_coordinates
                                self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                                self.coordinates.flag_not_obs = False

                                self.target_position_publisher.publish(self.coordinates)

                                self.neck_position_publisher.publish(self.place_to_sit_neck)
                                self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.contador_conhecidos=0
                            
                                self.state = 9
                            else:
                                self.state = 9
                        else:
                            if self.flag_place_to_sit == True:
                                self.state = 9
                            else:
                                self.place_to_sit = self.chair1_coordinates
                                self.place_to_sit_orientation = self.chair1_coordinates_orientation
                                self.flag_place_to_sit = True
                                self.state = 9
                        
                    else:         
                        cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                        num_faces=self.detect_face(cv2_img)
                        if self.num_faces != 0 :
                            nomes=self.reconhecimento_facial(cv2_img)
                            '''for pessoa in pessoas_detetadas:
                                self.contador_conhecidos = 0
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            
                            if self.names in nomes:
                                if nomes == self.names[0]:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                
                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA A CADEIRA 1 ONDE ESTÁ O HOST
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                                    self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"The new guest's name is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Host==True
                                    self.contador_conhecidos=0
                                    self.state = 9

                                elif nomes == self.names[1]:

                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates 
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA O GUEST 1 QUE ESTÁ SENTADO
                                    self.coordinates.move_target_coordinates = self.chair1_coordinates
                                    self.coordinates.rotate_target_coordinates = self.chair1_coordinates_orientation
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Guest1 = True
                                    self.contador_conhecidos=0
                                    self.state = 9   

                            else:   
                                self.state = 9

                        else:
                            if self.flag_place_to_sit == True:
                                self.state = 9
                            else:
                                self.place_to_sit = self.chair1_coordinates
                                self.place_to_sit_orientation = self.chair1_coordinates_orientation
                                self.flag_place_to_sit = True
                                self.state = 9

            elif self.state == 9:     

                if self.flag_Guest1 and self.flag_Host and self.place_to_sit:
                    self.state == 11
                else:
                    #CORRIGIR PARA OLHAR PARA A CADEIRA 2
                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                    self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                    self.coordinates.flag_not_obs = False

                    self.target_position_publisher.publish(self.coordinates)

                    self.neck_position_publisher.publish(self.place_to_sit_neck)

                    cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    num_faces=self.detect_face(cv2_img)
                    nomes=self.reconhecimento_facial(cv2_img)
                    if self.num_faces == 0 :
                        self.state = 10
                    else:
                        if self.flag_Host == True:
                            if self.flag_Guest1 == True:
                                self.speech_str.command = "Please stand. I couldn't find an empty seat."
                                
                                self.speaker_publisher.publish(self.speech_str)
                                self.flag_already_sit=True
                                self.state = 11
                            else:
                                
                                '''for pessoa in pessoas_detetadas:
                                    self.contador_conhecidos = 0
                                    if pessoa != 'unknown' and pessoa in self.names:
                                        self.contador_conhecidos += 1'''

                                
                                if self.names in nomes:
                                    if nomes == self.names[1]:

                                        #CORRIGIR PARA OLHAR PARA O GUEST 2
                                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                                        self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                        self.coordinates.flag_not_obs = False

                                        self.target_position_publisher.publish(self.coordinates)

                                        self.neck_position_publisher.publish(self.navigation_neck)
                                        self.speech_str.command = f"The guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                        if self.get_caract==3:
                                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                            
                                            self.speaker_publisher.publish(self.speech_str)
                                        elif self.get_caract==2:
                                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                            
                                            self.speaker_publisher.publish(self.speech_str)
                                        elif self.get_caract==1:
                                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                            
                                            self.speaker_publisher.publish(self.speech_str)

                                        #CORRIGIR PARA OLHAR PARA O GUEST 1 NA CADEIRA 2
                                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                                        self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                                        self.coordinates.flag_not_obs = False

                                        self.target_position_publisher.publish(self.coordinates)

                                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                                        self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                        self.flag_Guest1==True
                                        self.contador_conhecidos=0
                                        self.state = 11
                                else:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)

                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Guest1==True
                                    self.state=11
                            
                        else:
                            '''for pessoa in pessoas_detetadas:
                                self.contador_conhecidos = 0
                                if pessoa != 'unknown' and pessoa in self.names:
                                    self.contador_conhecidos += 1'''

                            

                            if self.names in nomes:
                                if nomes == self.names[0]:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA O HOST
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Host==True
                                    self.state=11

                                elif nomes == self.names[1]:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                    
                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)

                                    #CORRIGIR PARA OLHAR PARA O LOCAL ONDE ESTÁ O GUEST 1
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.chair2_coordinates_orientation
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.place_to_sit_neck)
                                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Guest1= True
                                    self.state=11
                            else:
                                if self.flag_Guest1 == True:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)

                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)

                                    self.neck_position_publisher.publish(self.guest_coordinates)
                                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Host==True
                                    self.state=11
                                else:
                                    #CORRIGIR PARA OLHAR PARA O GUEST 2
                                    self.coordinates.move_target_coordinates = self.chair2_coordinates
                                    self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                    self.coordinates.flag_not_obs = False

                                    self.target_position_publisher.publish(self.coordinates)
                                    self.neck_position_publisher.publish(self.navigation_neck)
                                    self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)

                                    self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    if self.get_caract==3:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==2:
                                        self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                        
                                        self.speaker_publisher.publish(self.speech_str)
                                    elif self.get_caract==1:
                                        self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                        
                                        self.speaker_publisher.publish(self.speech_str)    
                                    
                                    self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                                    
                                    self.speaker_publisher.publish(self.speech_str)
                                    self.flag_Guest1==True
                                    self.state=11

                

            elif self.state == 10:
                if self.flag_place_to_sit == True:
                    if self.flag_Guest1 == True:
                        #CORRIGIR PARA OLHAR PARA O GUEST 2
                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                        self.coordinates.rotate_target_coordinates = self.guest_coordinates
                        self.coordinates.flag_not_obs = False

                        self.target_position_publisher.publish(self.coordinates)
                        
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        
                        self.speaker_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.flag_Host=True
                        self.flag_Guest1= True

                        self.state=11
                    else:
                        if self.flag_Host == True:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Guest1= True
                            self.state=11
                        else:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            
                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host=True
                            self.flag_Guest1= True
                            self.state=11
                else:
                    self.place_to_sit = self.chair2_coordinates
                    self.flag_place_to_sit = True
                    if self.flag_Guest1 == True:
                        if self.flag_Host == True:
                            #CORRIGIR PARA OLHAR PARA O LOCAL ONDE SE VAI SENTAR
                            self.coordinates.move_target_coordinates = self.place_to_sit
                            self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)

                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_already_sit=True
                            self.state=11
                        else:
                            #CORRIGIR PARA OLHAR PARA O LOCAL ONDE ESTÁ O GUEST 2
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            self.flag_Host=True
                            self.state=11
                    else:
                        if self.flag_Host == True:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            
                            self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            
                            self.flag_Guest1= True
                            self.state=11
                        else:
                            #CORRIGIR PARA OLHAR PARA O GUEST 2
                            self.coordinates.move_target_coordinates = self.chair2_coordinates
                            self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            self.neck_position_publisher.publish(self.navigation_neck)
                            self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                            if self.get_caract==3:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==2:
                                self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                                
                                self.speaker_publisher.publish(self.speech_str)
                            elif self.get_caract==1:
                                self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                                
                                self.speaker_publisher.publish(self.speech_str)

                            self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                            #CORRIGIR PARA OLHAR PARA O LOCAL ONDE SE VAI SENTAR
                            self.coordinates.move_target_coordinates = self.place_to_sit
                            self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                            self.coordinates.flag_not_obs = False

                            self.target_position_publisher.publish(self.coordinates)
                            self.neck_position_publisher.publish(self.place_to_sit_neck)
                            self.speech_str.command = f"Please take a sit on the chair that I'm looking for."
                            
                            self.speaker_publisher.publish(self.speech_str)
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
                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                        self.coordinates.rotate_target_coordinates = self.sofa_coordinates
                        self.coordinates.flag_not_obs = False

                        self.target_position_publisher.publish(self.coordinates)

                        self.neck_position_publisher.publish(self.navigation_neck)

                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            
                            self.speaker_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.flag_Guest1=True
                        self.state=12
                else:
                    if self.flag_Guest1 == True:
                        #CORRIGIR PARA OLHAR PARA UMA POSIÇÃO
                        self.coordinates.move_target_coordinates = self.chair2_coordinates
                        self.coordinates.rotate_target_coordinates = self.sofa_coordinates
                        self.coordinates.flag_not_obs = False

                        self.target_position_publisher.publish(self.coordinates)
                        
                        self.neck_position_publisher.publish(self.navigation_neck)

                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        
                        self.speaker_publisher.publish(self.speech_str)

                        self.speech_str.command = f"The new guest is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.flag_Host=True
                        self.state=12

                    else:
                        self.neck_position_publisher.publish(self.navigation_neck)
                        self.speech_str.command = f"The host is {self.names[0]} and his favorite drink is {self.drinks[0]}."
                        
                        self.speaker_publisher.publish(self.speech_str)

                        self.speech_str.command = f"The first guest is {self.names[1]} and his favorite drink is {self.drinks[1]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        if self.get_caract==3:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)
                        elif self.get_caract==2:
                            self.speech_str.command = f"The first guest is gender {self.caracteristics[0]}, is in age group {self.caracteristics[1]}. He is taller than me."
                            
                            self.speaker_publisher.publish(self.speech_str)
                        elif self.get_caract==1:
                            self.speech_str.command = f"The first guest is taller than me and with respect to ethnicity is {self.caracteristics[2]}."
                            
                            self.speaker_publisher.publish(self.speech_str)

                        self.speech_str.command = f"And the new guest's name is {self.names[2]} and his favorite drink is {self.drinks[2]}."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.flag_Host=True
                        self.flag_Guest1=True
                        self.state=12

            elif self.state == 12:
                if self.flag_place_to_sit == True:
                    if self.flag_already_sit == True:
                        self.speech_str.command = f"Thank you. I finished my receptionist task."
                        
                        self.speaker_publisher.publish(self.speech_str)
                    else:
                        #CORRIGIR PARA SE ORIENTAR PARA A ZONA DO LUGAR DE SENTAR
                        self.coordinates.move_target_coordinates = self.place_to_sit
                        self.coordinates.rotate_target_coordinates = self.place_to_sit_orientation
                        self.coordinates.flag_not_obs = False

                        self.target_position_publisher.publish(self.coordinates)
                        
                        self.neck_position_publisher.publish(self.place_to_sit_neck)
                        self.speech_str.command = f"Please sit down on the place that I'm looking for."
                        
                        self.speaker_publisher.publish(self.speech_str)
                        self.speech_str.command = f"Thank you. I finished my receptionist task."
                        
                        self.speaker_publisher.publish(self.speech_str)
                else:
                    self.speech_str.command = "Please stand. I couldn't find an empty seat."
                    
                    self.speaker_publisher.publish(self.speech_str)
                    self.speech_str.command = f"Thank you. I finished my receptionist task."
                    
                    self.speaker_publisher.publish(self.speech_str)  """
    
