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

import numpy as np
# import face_recognition
import time
import os
# import tensorflow as tf
from datetime import datetime

import math

import mediapipe as mp

import argparse
from mediapipe.python.solutions.drawing_utils import _normalized_to_pixel_coordinates

class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("ReceptionistNode")
        self.get_logger().info("Initiliased Receptionist Node")
        
        #RGB
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        # Neck Topics
        self.neck_position_publisher = self.create_publisher(Pose2D, "neck_to_pos", 10)
        #self.neck_get_position_subscriber = self.create_subscription(Pose2D, "get_neck_pos", self.get_neck_position_callback, 10)

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
        self.calibrate_ambient_noise_publisher = self.create_publisher(Bool, "calib_ambient_noise", 10)


        # Face Shining RGB
        self.face_publisher = self.create_publisher(Int16, "face_command", 10)

        # Odometry
        self.odometry_subscriber = self.create_subscription(Odometry, "odom",self.get_odometry_robot_callback, 10)
        self.localisation_subscriber = self.create_subscription(Odometry, "odom_a", self.get_localisation_robot_callback, 10)

        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        

        # Low Level: Start Button
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)


        #self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)

        
        # Timer
        # self.create_timer(0.1, self.timer_callback)
        self.start_button_state = False

        #RGB
        self.rgb_ctr = 2
        self.rgb = Int16()

        #Arena 1
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0


        #Arena 1
        #self.begin_coordinates = (0.0, 3.0) # <----- CHANGE ME
        self.begin_coordinates = (self.robot_x, self.robot_y) # <----- CHANGE ME
        
        #self.door_coordinates = (-0.95, 1.5) # <----- CHANGE ME
        self.door_coordinates = (-1.0, 1.6) # <----- CHANGE ME
        #self.door_coordinates_orientation= (-0.95, 0.0) # <----- CHANGE ME
        self.door_coordinates_orientation= (1.3, 2.1) # <----- CHANGE ME
        self.find_coordinates = (-0.82 , 3.5) # <----- CHANGE ME
      
        self.sofa_coordinates = (-3.4, 3.6) # <----- CHANGE ME
    
        self.guest_coordinates = (-0.82 , 2.4) # <----- CHANGE ME
        

        #---------------------------------------------------

        """self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0


        #Arena 2
        #self.begin_coordinates = (0.0, 3.0) # <----- CHANGE ME
        self.begin_coordinates = (self.robot_x, self.robot_y) # <----- CHANGE ME

        
        self.door_coordinates = (1.0, 1.6) # <----- CHANGE ME

        self.door_coordinates_orientation= (1.05, 0.5) # <----- CHANGE ME
        
        self.find_coordinates = (-0.7 , 2.8) # <----- CHANGE ME
      
        self.sofa_coordinates = (-2.8, 2.8) # <----- CHANGE ME
    
        self.guest_coordinates = (-0.7 , 1.7) # <----- CHANGE ME"""
      

        #self.place_to_sit = (0.0 , 0.0) 
       

       
        #self.chair1_coordinates = (3.7 , 8.9)
        
        
        #self.chair2_coordinates = (0.6, 9.05)

        self.state = 0
        

       

        self.coordinates = TarNavSDNL()
        #Código para ele olhar para a cara da pessoa e centrar a cara da pessoa na imagem
        self.door_neck_coordinates = Pose2D()
        self.door_neck_coordinates.x = 180.0
        self.door_neck_coordinates.y = 180.0
        
        #Definir como se estivesse a olhar ligeiramente para baixo - Perceber o que precisamos que olhe para baixo para detetarmos tudo direito
        self.place_to_sit_neck = Pose2D()
        self.place_to_sit_neck.x = 180.0
        self.place_to_sit_neck.y = 160.0

 
        

        self.guest_neck = Pose2D()
        self.guest_neck.x = 270.0
        self.guest_neck.y = 193.0


        self.navigation_neck = Pose2D()
        self.navigation_neck.x = 180.0
        self.navigation_neck.y = 170.0

        self.talk_neck = Pose2D()
        self.talk_neck.x = 180.0
        self.talk_neck.y = 193.0

        self.sofa_neck = Pose2D()
        self.sofa_neck.x = 180.0
        self.sofa_neck.y = 180.0

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

        self.num_faces = 0

        self.image_color = Image()
        #self.colour_img = Image()
        #self.current_frame = Image()
        self.speech_str = RobotSpeech()

        self.speech_type = SpeechType()
        self.speech_type.receptionist = True
        self.speech_type.yes_or_no = False
        self.speech_type.restaurant = False
        self.speech_type.gpsr = False

        self.get_speech = String()

        self.keywords = String()


        self.cv2_img = []
        
        self.width = 0
        self.height = 0
        
        
        #Pastas relativas às funções das características como por exemplo, características, reconhecimento, etc.
        #CAMINHOS QUE É PRECISO MUDAR NOME

        self.drinks = ["Milk"]
        self.names = ["John"]
        
        self.get_caract = 0
        self.caracteristics = []
        self.filename = String()


        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        # MODEL_MEAN_VALUES = (78, 87, 114)

        self.ageList = ['(0 and 2)', '(4 and 6)', '(8 and 12)', '(15 and 22)', '(23 and 32)', '(38 and 43)', '(48 and 55)', '(60 and 100)']
        self.genderList = ['Male', 'Female']


        self.pasta_imagens_conhecidas = "/home/charmie/charmie_ws/build/charmie_receptionist/charmie_receptionist/images"
        
        self.imagens_conhecidas = os.listdir(self.pasta_imagens_conhecidas)

        self.codificacoes_conhecidas = []
        self.nomes_conhecidos = []
        

        self.images = []
        self.classNames = []
        
        self.model_loaded_race = tf.keras.models.load_model('/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_raca.h5')
        self.model_loaded_gender = tf.keras.models.load_model('/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_gender.h5')
        
        self.faceProto = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector_uint8.pb"
        self.faceModel = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/opencv_face_detector.pbtxt"
        self.ageProto = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_age.prototxt"
        self.ageModel = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/age_net.caffemodel"
        self.genderProto = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/deploy_gender.prototxt"
        self.genderModel = "/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/gender_net.caffemodel"
        
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_detection = self.mp_face_detection.FaceDetection()
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

    def detect_face(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Detecta rostos na imagem
        
        results = self.mp_face_detection.process(image_rgb)
        
        if results.detections:
            num_faces = len(results.detections)
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                #print(bbox)
                height, width, _ = image.shape
                #print(image.shape)
                start_x = int(bbox.xmin * width)
                start_y = int(bbox.ymin * height)
                end_x = int((bbox.xmin + bbox.width) * width)
                end_y = int((bbox.ymin + bbox.height) * height)
                #cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
                center_face_x = int((start_x + end_x) / 2)
                center_face_y = int((start_y + end_y) / 2)
                
                center_x = int(width/2)
                center_y = int(height/2)

                print(center_x, center_y)
                #cv2.circle(image, (center_x, center_y), 1,(255, 0, 0), 2)

                #print(center_face_x, center_face_y)
                #cv2.circle(image, (center_face_x, center_face_y), 1,(0, 0, 255), 2)
                

                #cv2.line(image, (center_x,center_y), (center_face_x,center_face_y), (0, 255, 255), 2)
                cv2.imshow("c_camera", image)
                cv2.waitKey(1)
               
        else:
            num_faces = 0
            center_y = 0
            center_x = 0
            center_face_y = 0
            center_face_x = 0

        #print(num_faces)
        return num_faces, center_face_x, center_face_y, center_x, center_y

        

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

    def detectGender(self,image):
        color_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img=cv2.resize(color_img,(224,224))
        normalized_img = img / 255.0
        expanded_img = np.expand_dims(normalized_img, axis=0)
        # Realizar a previsão usando o modelo carregado
        predictions = self.model_loaded_gender.predict(expanded_img)  # Substitua ... pelos dados da imagem pré-processada
        
        # Obter a previsão da raça
        gender_predominante_index = np.argmax(predictions, axis=1)
        gender_predominante = ['Female', 'Male'][gender_predominante_index[0]]
        
        # Verificar se a previsão está disponível
        if gender_predominante is not None:
            """ if predictions[0] < 0.9:
                gender_dominante = 'Male'
                error = 0
            else: """
            error = 0
            gender_dominante = gender_predominante
        else:
            gender_dominante = None
            error = 1

        return gender_dominante, error
    
    def detectAge(self, image):
        parser = argparse.ArgumentParser()
        parser.add_argument('--image')

        args = parser.parse_args()

        self.faceNet = cv2.dnn.readNet(self.faceModel, self.faceProto)
        self.ageNet = cv2.dnn.readNet(self.ageModel, self.ageProto)
        #self.genderNet = cv2.dnn.readNet(self.genderModel, self.genderProto)

        padding = 20
        flag_age_gender = True
        while flag_age_gender:
            flag_age_gender = False

            resultImg, faceBoxes = self.highlightFace(self.faceNet, image)
            if not faceBoxes:
                #print("No face detected")
                age = ''
         #       gender = ''
                error = 1
                break
            else:
                error = 0
                for faceBox in faceBoxes:
                    face = image[max(0, faceBox[1] - padding):
                                min(faceBox[3] + padding, image.shape[0] - 1), max(0, faceBox[0] - padding)
                                                                                :min(faceBox[2] + padding, image.shape[1] - 1)]

                    blob = cv2.dnn.blobFromImage(face, 1.0, (227, 227), self.MODEL_MEAN_VALUES, swapRB=False)
       

                self.ageNet.setInput(blob)
                agePreds = self.ageNet.forward()
                age_index = agePreds[0].argmax()
                age= self.ageList[age_index]
                if age_index < 2:  
                    age = "(15 and 22)"
                elif age_index < 4: 
                    age = "(23 and 32)"
                

        return age, error

    def detectrace(self, image):
            
        color_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img=cv2.resize(color_img,(224,224))
        normalized_img = img / 255.0
        expanded_img = np.expand_dims(normalized_img, axis=0)
        # Realizar a previsão usando o modelo carregado
        predictions = self.model_loaded_race.predict(expanded_img)  # Substitua ... pelos dados da imagem pré-processada
        
        # Obter a previsão da race
        race_predominante_index = np.argmax(predictions, axis=1)
        race_predominante = ['Asian', 'Indian', 'Black or African Descendent', 'White', 'Middle Eastern', 'Latino Hispanic'][race_predominante_index[0]]
        
        # Verificar se a previsão está disponível
        if race_predominante is not None:
            race_dominante = race_predominante
            error = 0
        else:
            race_dominante = None
            error = 1
        
        return race_dominante, error
  

    """def reconhecimento_facial(self, imagem, imagens_conhecidas):
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

        return nomes_correspondentes"""
        
    """ def follow_center_face(self, erro_x, erro_y):
        erro = Pose2D()
        
        # erro.x= float(erro_x)
        # erro.y= float(erro_y)
        
        
        while ( abs(erro_x) >= 50 ):
            print('CORRIGE X')
            print('erro: ', abs(erro.x), abs(erro.y))
            cv2_img = self.br.imgmsg_to_cv2(self.image_color, "bgr8")
            num_faces, center_face_x, center_face_y, center_x, center_y  = self.detect_face(cv2_img)
            error_x = center_face_x - center_x
            error_y = center_face_y - center_y
            
            erro.x= float(error_x)
            erro.y= 0.0
            self.neck_error_publisher.publish(erro)


        if abs(erro_x) < 50 :
                              
            while ( abs(erro_y) > 50 ):
                print('CORRIGE Y')
                print('erro: ', abs(erro.x), abs(erro.y))
                cv2_img = self.br.imgmsg_to_cv2(self.image_color, "bgr8")
                num_faces, center_face_x, center_face_y, center_x, center_y  = self.detect_face(cv2_img)
                error_x = center_face_x - center_x
                error_y = center_face_y - center_y
                
                erro.x= 0.0
                erro.y= float(error_y)
                self.neck_error_publisher.publish(erro)

        print('erro: ', abs(erro.x), abs(erro.y)) """
    

    def follow_center_face(self, erro_x, erro_y):
        print('Center face')

        erro = Pose2D()
        dist_2 = erro_x**2 + erro_y**2
        dist = math.sqrt(dist_2)
        

        while dist >= 10:
            print('erro: ', abs(erro.x), abs(erro.y))
            cv2_img = self.br.imgmsg_to_cv2(self.image_color, "bgr8")
            #num_faces, center_face_x, center_face_y, center_x, center_y  = self.detect_face(cv2_img)
            #print('Variaveis função renata:', num_faces, center_face_x, center_face_y, center_x, center_y)
            #error_x = center_face_x - center_x
            #error_y = center_face_y - center_y
            num_faces, face_x, face_y, shoulder, hip, center_x, center_y = self.found_landmarks(cv2_img)
            error_x = face_x - center_x
            error_y = face_y - center_y

            print('erro_atualizado =', int(error_x), int(error_y))
            dist_2 = error_x**2 + error_y**2
            dist = math.sqrt(dist_2)
            erro.x= float(error_x)
            erro.y= float(error_y)
            print("erro:", erro)
            while(True):
                print("You used function neck error! This function no longer exists, please change this! in this case should be neck_follow_person .")
                # self.neck_error_publisher.publish(erro)
            # print('dist 2:', dist_2)
            print('dist:', dist)
            #time.sleep(0.05)
            cv2.imshow("c_camera", cv2_img)
            cv2.waitKey(1)

                   

        print('centrei')

    def found_landmarks(self, image):
        self.get_logger().info('FUNCTION DETECT LANDMARKS')

        
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        height, width, _ = image.shape
        results = self.pose.process(image)
        #print("RESULTS")

        print('A')

         
        center_x = int(width/2)
        center_y = int(height/2)

        if results.pose_landmarks:
            print('found landmarks')
            self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2), self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=2))
            #LEFT_SIDE_IMG
            
            point_face_x = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE].x*width,2))
            point_face_y = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE].y*height,2))
            
            print(point_face_x, point_face_y)

            hip = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_HIP ].y*height,2))
            
            print(hip)
            
            shoulder = (round(results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y*height,2))
            
            print(shoulder)
            
            #cv2.line(image, (center_x,center_y), (point_face[0],point_face[1]), (0, 255, 255), 2)

            num_person = 1
            
            cv2.imshow("Landmarks", image)
            cv2.waitKey(1)
        else:
            print('didnt found landmarks')
            num_person = 0
            point_face_x = 0
            point_face_y = 0
            shoulder = 0
            hip = 0


        
        return num_person,point_face_x, point_face_y, shoulder, hip, center_x, center_y
        
    def coordinates_to_navigation(self, p1, p2, bool):
        nav = TarNavSDNL()
        nav.flag_not_obs = bool
        nav.move_target_coordinates.x = p1[0]
        nav.move_target_coordinates.y = p1[1]
        nav.rotate_target_coordinates.x = p2[0]
        nav.rotate_target_coordinates.y = p2[1]
        self.target_position_publisher.publish(nav)

    def get_color_image_callback(self, img: Image):
        #self.colour_img = img
        # print(img)
        #print("---")
        #self.get_logger().info('Receiving color video frame')
        #print("entrei callback camara")
        self.image_color = img
        #self.cv2_img = self.br.imgmsg_to_cv2(self.image_color, "bgr8")
        #current_frame = self.br.imgmsg_to_cv2(img, "bgr8")
        #self.num_faces = self.camera_callback(self.current_frame)
        """ cv2.imshow("c_camera", current_frame)
        cv2.waitKey(1) """
        #pass


        

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

        
    def get_speech_callback(self, keywords : String):
        print("Received Audio:", keywords.data)
        self.keywords = keywords
        self.flag_audio_done = True

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
        #self.start_audio()
    
    def flag_pos_reached_callback(self, state: Bool):
        print("Received Navigation Flag:", state.data)
        self.flag_navigation_done = True
    
    def start_audio(self):
        self.audio_command_publisher.publish(self.speech_type)

    def get_odometry_robot_callback(self, odom:Odometry):
        self.robot_current_position = odom

    def get_localisation_robot_callback(self, loc:Odometry):
        self.robot_x = loc.pose.pose.position.x
        self.robot_y = loc.pose.pose.position.y

        qx = loc.pose.pose.orientation.x
        qy = loc.pose.pose.orientation.y
        qz = loc.pose.pose.orientation.z
        qw = loc.pose.pose.orientation.w

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

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
     

    def wait_for_start_button(self):
        while not self.node.start_button_state:
            pass
        f = Bool()
        f.data = False 
        self.node.flag_start_button_publisher.publish(f)

    

    def main(self):
        time.sleep(1)
        print("IN NEW MAIN")
        while True:
            """ try:
                cv2_img = self.node.br.imgmsg_to_cv2(self.node.image_color, "bgr8")
                num_faces, center_face_x, center_face_y, center_x, center_y  = self.node.detect_face(cv2_img)
                

                if num_faces != 0 :
                    error_x = center_face_x - center_x
                    error_y = center_face_y - center_y
                    self.node.follow_center_face(error_x,error_y)
                self.node.state = -3
            except Exception as e:
                print(e) """
            #Estado comum aos dois Guests
            if self.node.state == 0:

                """
                print("1")
                #Informo o Charmie que acabei de falar e posso começar a ouvir
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()
                print('b')


                print("2")
                #Informo o Charmie que acabei de falar e posso começar a ouvir
                self.node.audio_command_publisher.publish(self.node.speech_type)
                #self.node.start_audio()
                self.wait_for_end_of_audio()
                print('b')


                time.sleep(5)
                calib = Bool()
                calib.data = True
                self.node.calibrate_ambient_noise_publisher.publish(calib)
                """




                #Informa que está pronto para começar a tarefa
                self.node.get_logger().info("estado 0")

                self.node.rgb_ctr = 22
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), self.node.door_coordinates, False)
                self.wait_for_end_of_navigation()

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                self.node.speech_str.command = "Hello! I am ready to start the receptionist task! Waiting for start button to be pressed."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                t = Bool()
                t.data = True
                self.node.flag_start_button_publisher.publish(t)
                self.wait_for_start_button()
                
                
                self.node.rgb_ctr = 41
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                time.sleep(2)


                self.node.neck_position_publisher.publish(self.node.navigation_neck)
                #vai para a porta

                self.node.coordinates_to_navigation(self.node.door_coordinates, self.node.door_coordinates_orientation, False)
                self.wait_for_end_of_navigation()


                time.sleep(1)
                calib = Bool()
                calib.data = True
                self.node.calibrate_ambient_noise_publisher.publish(calib)
                time.sleep(3) # obrigatorio depois de recalibrar audio



                self.node.state = 1
                print(self.node.state)


            #Estado para verificar se está alguém na porta e ficar a repetir de 5 em 5 segundos enquanto não estiver lá ninguém
            #Estado para o Guest 1
            elif self.node.state == 1:    
                self.node.get_logger().info("estado 1")



                #CORRIGIR CÓDIGO PARA SE MOVER PARA A PORTA E FICAR A OLHAR PARA A PORTA
                self.node.neck_position_publisher.publish(self.node.door_neck_coordinates)

                #Informa que está pronto para receber convidado
                self.node.speech_str.command = "I am ready to receive a new guest. Please stand in front of me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                
                 #Começar a contar o tempo
                
                cv2_img = self.node.br.imgmsg_to_cv2(self.node.image_color, "bgr8")
                print('Opened image')
                #Verificar através do yolo pose se a cara está enquadrada na imagem
                #No caso de estar enquadrada prossegue no código,no caso de não estar sobe o pescoço até central o nariz



                num_faces, face_x, face_y, shoulder, hip, center_x, center_y = self.node.found_landmarks(cv2_img)
                print('called_landmarks')

                #num_faces, center_face_x, center_face_y, center_x, center_y  = self.node.detect_face(cv2_img)
                cv2.imshow("c_camera", cv2_img)
                cv2.waitKey(1)

                print(num_faces)


                if num_faces != 0 :
                    self.node.speech_str.command = "Please look at me." #"Please stand still."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                    #error_x = center_face_x - center_x
                    #error_y = center_face_y - center_y


                    if (face_y < shoulder and shoulder < hip):
                        error_x = face_x-center_x
                        error_y = face_y-center_y
                        self.node.follow_center_face(error_x,error_y)


                        self.node.rgb_ctr = 32
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)

                        self.node.speech_str.command = "Hello! My name is Charmie. I will make you some questions. Please speak loud and clear. Answer me after the green light under my wheels."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.node.speech_str.command = "What's your name and favourite drink?"
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        print('a')

                        self.node.rgb_ctr = 12
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)
                        
                        #Informo o Charmie que acabei de falar e posso começar a ouvir
                        self.node.audio_command_publisher.publish(self.node.speech_type)
                        #self.node.start_audio()
                        self.wait_for_end_of_audio()
                        print('b')

                        self.node.rgb_ctr = 22
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)
                        

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
                            """self.node.rgb_ctr = 12
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb)"""
                        if self.node.count_guest==2:
                            self.node.state = 3 
                            """self.node.rgb_ctr = 12
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb)"""
                        
                            #self.node.state = 2
                else:
                    print("Não detetou cara")

                print(self.node.state)
                
                
            

            #Estado apenas para o Guest1
            elif self.node.state == 2:
                self.node.get_logger().info("estado 2")
                #CORRIGIR PARA CÓDIGO QUE AJUSTA A POSIÇÃO DA CARA PARA O CENTRO DA FACE DA PESSOA

                #Falar para pedir para olhar para camara
                self.node.speech_str.command = "Please look into my eyes. I will take you a picture!"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
  
                cv2_img = self.node.br.imgmsg_to_cv2(self.node.image_color, "bgr8")
                print(' :) ')
                #print(cv2_img.shape)
                #self.node.width,self.node.height, _ = cv2_img.shape

                #COLOCAR NOVAMENTE A PARTE DO YOLO
                
                #Criar função apra verificar se os keypoints da cabeça estão ou não na imagem. Se estiverem retornarmos TRUE, senão chamamos a função para enquadrar com a cara.
                

                #Função de deteção facial 
                num_faces, center_face_x, center_face_y, center_x, center_y  = self.node.detect_face(cv2_img)
                
                if num_faces != 0 :
                
                    error_x = center_face_x - center_x
                    error_y = center_face_y - center_y
                    self.node.follow_center_face(error_x,error_y)
                
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    self.filename = self.node.names[1]
                    file_path = os.path.join(self.node.pasta_imagens_conhecidas, self.filename + '.png')
                    cv2.imwrite(file_path, cv2_img) 

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)

                    # Apend ao nome e imagem guardada acima
                    #myList = os.listdir(self.node.pasta_imagens_conhecidas)
                    #for cl in myList:
                    #    curImg = cv2.imread(f'{self.node.pasta_imagens_conhecidas}/{cl}')
                    #    self.node.images.append(curImg)
                    #    self.node.classNames.append(os.path.splitext(cl)[0])

                    #Recolho as características das imagens
                    age,error_age = self.node.detectAge(cv2_img)
                    gender,error_gender =self.node.detectGender(cv2_img)
                    race, error_race = self.node.detectrace(cv2_img)

                    #Para prevenir que o código empanque, garanto que retorno sempre alguma coisa, seja as características ou erro
                    if error_race == 0 and error_age  == 0 and error_gender== 0:
                        self.node.get_caract = 3
                        self.node.caracteristics.append(gender)
                        self.node.caracteristics.append(age)
                        self.node.caracteristics.append(race)
                        
                    elif error_race == 0 and error_age == 0 and error_gender == 1:
                        self.node.get_caract = 2
                        self.node.caracteristics.append(age)
                        self.node.caracteristics.append(race)

                    elif error_race == 1 and error_age == 0 and error_gender == 0:
                        self.node.get_caract = 21
                        self.node.caracteristics.append(gender)
                        self.node.caracteristics.append(age)

                    elif error_race == 0 and error_age == 1 and error_gender == 0:
                        self.node.get_caract = 22
                        self.node.caracteristics.append(gender)
                        self.node.caracteristics.append(race)
                        
                    elif error_race == 1 and error_age == 1 and error_gender == 0:
                        self.node.get_caract=1
                        self.node.caracteristics.append(gender)

                    elif error_race == 0 and error_age == 1 and error_gender == 1:
                        self.node.get_caract = 11
                        self.node.caracteristics.append(race)

                    elif error_race == 1 and error_age == 0 and error_gender == 1:
                        self.node.get_caract = 12
                        self.node.caracteristics.append(age)
                    else:
                        self.node.caracteristics = "error" 
                    
                            
                    self.node.state=3

                else:
                    print("Não detetou cara")
                """ except Exception as e:
                    print(e)  """

            #Estado comum aos 2 guests
            elif self.node.state == 3:
                self.node.get_logger().info("estado 3")
                #Agradece e pede para o seguir até à zona dos sofás

                


                self.node.speech_str.command = "Thank you. Please follow me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                self.node.neck_position_publisher.publish(self.node.navigation_neck)


                self.node.coordinates_to_navigation(self.node.door_coordinates, self.node.find_coordinates, False)
                self.wait_for_end_of_navigation()


                #vai para o sofá
                self.node.coordinates_to_navigation(self.node.find_coordinates, self.node.sofa_coordinates, False)
                self.wait_for_end_of_navigation()



                self.node.rgb_ctr= 2
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                #CORRIGIR PARA SE MOVER PARA O SOFÁ E O PESCOÇO FICAR ORIENTADO PARA BAIXO NA DIREÇÃO DAS PESSOAS SENTADAS

                self.node.neck_position_publisher.publish(self.node.guest_neck)
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

            #elif self.node.state == 7:
            #    print("CHEGUEI ESTADO 7") 


            #Estado relativo ao Guest1 no SOFÁ

            # AQUI TEMOS DE OLHAR PARA O SOFA E PARA O GUEST E PARA O SOFÁ
            elif self.node.state == 4:
                self.node.get_logger().info("estado 4")
                #Guardar Imagem            
                #try:
                    #cv2_img = self.node.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                    #num_faces=self.node.detect_face(cv2_img)
                    #Código para ver se encontro faces
                #    num_faces =1
                #    if num_faces != 0:
                #        if num_faces == 1:
                            #Guarda as coordenadas do sofá como local para sentar e ativa a flag a indicar que já tem um local para sentar
                self.node.neck_position_publisher.publish(self.node.sofa_neck)

                self.node.speech_str.command = f"Hello, I will present everyone in this room."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.flag_Host==True
                            #self.node.place_to_sit_orientation=self.node.sofa_coordinates_orientation
                #self.node.flag_place_to_sit=True
                            #Função para reconhecer pessoas. Retorna uma variavel pessoa que tem um centro associado a um nome

                            #nomes=self.reconhecimento_facial(cv2_img)
                #self.node.flag_Host=True
                            
                            #Se conheceu alguém é porque reconheceu o Host.
                            #if self.node.names[0] in nomes:
                #            if self.node.flag_Host :
                                #OLHAR PARA A POSIÇÃO DO GUEST

                                #self.node.neck_position_publisher.publish(self.node.talk_neck)

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

                self.node.neck_position_publisher.publish(self.node.sofa_neck)

                self.node.speech_str.command = f"The guest name is {self.node.names[1]} and the favorite drink is {self.node.drinks[1]}."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                
                #self.node.neck_position_publisher.publish(self.node.sofa_coordinates)

                #Sentou a o convidado
                self.node.speech_str.command = f"Please take a sit on the sofa that I'm looking at."               
                #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.flag_already_sit = True
                self.wait_for_end_of_speaking()
                #Não sei o numero do estado, tou a assumir que é o último

                self.node.neck_position_publisher.publish(self.node.navigation_neck)

                self.node.coordinates_to_navigation(self.node.find_coordinates, self.node.door_coordinates, False)
                self.wait_for_end_of_navigation()


                #self.node.coordinates.move_target_coordinates = self.node.find_coordinates
                #self.node.coordinates.rotate_target_coordinates = self.node.return_door_coordinates
                #self.node.coordinates.flag_not_obs = False
                #self.node.target_position_publisher.publish(self.node.coordinates)
                #self.wait_for_end_of_navigation()

                #PONHO O A PASSAR PELAS COORDENADAS DE INICIO PORQUE NÃO CHEGA AO LOCAL DESEJADO
                #self.node.coordinates.move_target_coordinates = self.node.door_second_coordinates
                #self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                #self.node.coordinates.flag_not_obs = False
                #self.node.target_position_publisher.publish(self.node.coordinates)
                #self.wait_for_end_of_navigation()

                self.node.neck_position_publisher.publish(self.node.navigation_neck)
                #vai para a porta

                self.node.coordinates_to_navigation(self.node.door_coordinates, self.node.door_coordinates_orientation, False)
                self.wait_for_end_of_navigation()

                #self.node.coordinates.move_target_coordinates = self.node.door_coordinates
                #self.node.coordinates.rotate_target_coordinates = self.node.door_coordinates_orientation
                #self.node.coordinates.flag_not_obs = False
                #self.node.target_position_publisher.publish(self.node.coordinates)
                #self.wait_for_end_of_navigation()
                self.node.state = 1


                time.sleep(1)
                calib = Bool()
                calib.data = True
                self.node.calibrate_ambient_noise_publisher.publish(calib)
                time.sleep(3) # obrigatorio depois de recalibrar audio




                """ else:
                                self.node.state = 5

                        
                        elif num_faces >=2:
                            #Código para reconhecer as pessoas
                            # Contagem das pessoas conhecidas na imagem
                            
                            nomes=self.node.reconhecimento_facial(cv2_img)
                            
                            
                            
                            if self.node.names[0] in nomes:
                                #Apresentação do guest e do host
                                #CORRIGIR PARA SE VIRAR PARA O GUEST

                                self.node.neck_position_publisher.publish(self.node.guest_neck)

                                #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                                #self.node.coordinates.rotate_target_coordinates = self.node.guest_coordinates
                                #self.node.coordinates.flag_not_obs = False
                                #self.node.target_position_publisher.publish(self.node.coordinates)
                                #self.wait_for_end_of_navigation

                                self.node.speech_str.command = f"The host is {self.node.names[0]} and his favorite drink is {self.node.drinks[0]}."
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()

                                #CORRIGIR PARA FICAR A OLHAR PARA O SOFÁ
                                #self.coordinates.move_target_coordinates = self.sofa_coordinates
                                #self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                #self.coordinates.flag_not_obs = False
                                #self.target_position_publisher.publish(self.coordinates)
                                #self.wait_for_end_of_navigation

                                self.node.neck_position_publisher.publish(self.node.sofa_coordinates)

                                self.node.speech_str.command = f"The guest is {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."  
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()
                                self.node.flag_Host==True
                                self.node.state = 5
                            else:
                                self.node.state = 5
            
                    else: 
                        self.node.place_to_sit = self.node.sofa_coordinates
                        #self.node.place_to_sit_orientation = self.node.sofa_coordinates_orientation
                        self.node.flag_place_to_sit=True
                        #Guardar a coordenada do sofá
                        self.node.state=5

                except Exception as e:
                    print(e)     

                self.node.get_logger().info("Fim do estado 4")"""

            elif self.node.state == 7:
                self.node.get_logger().info("estado 7")
                #cv2_img = self.br.imgmsg_to_cv2(self.colour_img, "bgr8")
                #num_faces=self.detect_face(cv2_img)
                #num_faces=1
                #Código para ver se encontro faces
                #if num_faces != 0:
                #    if num_faces == 1:
                        #Guarda as coordenadas do sofá uma vez que como só tem uma pessoa é um local onde se pode sentar
                #self.node.place_to_sit = self.node.sofa_coordinates
                self.node.neck_position_publisher.publish(self.node.sofa_neck)
                self.node.speech_str.command = f"Hello, I will present everyone in this room."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.flag_Host==True
                        #self.place_to_sit_orientation = self.sofa_coordinates_orientation
                #self.node.flag_place_to_sit=True
                        #nomes=self.node.reconhecimento_facial(cv2_img)
                    
                        #nomes == self.node.names[0]
                        #if self.node.names in nomes:
                        #if nomes == self.node.names[0]:
                
                            #CORRIGIR PARA OLHAR O GUEST NOVO
                            #self.coordinates.move_target_coordinates = self.sofa_coordinates
                            #self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            #self.coordinates.flag_not_obs = False
                            #self.target_position_publisher.publish(self.coordinates)

                self.node.neck_position_publisher.publish(self.node.guest_neck)

                self.node.speech_str.command = f"The host is {self.node.names[0]} and his favorite drink is {self.node.drinks[0]}."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.flag_Host==True

                            #CORRIGIR PARA OLHAR PARA O SOFÁ NOVAMENTE
                            #self.coordinates.move_target_coordinates = self.sofa_coordinates
                            #self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                            #self.coordinates.flag_not_obs = False
                            #self.target_position_publisher.publish(self.coordinates)

                self.node.neck_position_publisher.publish(self.node.sofa_neck)
                self.node.speech_str.command = f"The new guest's name is {self.node.names[2]} and the favorite drink is {self.node.drinks[2]}."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                #self.node.state = 8

                        #elif nomes == self.node.names[1]:
                            #CORRIGIR PARA OLHAR O CONVIDADO 2
                            #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                            #self.node.coordinates.rotate_target_coordinates = self.node.guest_coordinates
                            #self.node.coordinates.flag_not_obs = False

                            #self.node.target_position_publisher.publish(self.node.coordinates)

                self.node.neck_position_publisher.publish(self.node.guest_neck)
                self.node.speech_str.command = f"The first guest name is {self.node.names[1]} and the favorite drink is {self.node.drinks[1]}."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                if self.node.get_caract==3:
                    self.node.speech_str.command =f"The first guest is gender {self.node.caracteristics[0]}, the age is between {self.node.caracteristics[1]}. The guest is taller than me. The eyes are brown and with respect to ethnicity is {self.node.caracteristics[2]}."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                elif self.node.get_caract==2:
                    self.node.speech_str.command =f"The first guest is gender male, the age is between {self.node.caracteristics[0]}. the guest is taller than me, the eyes are brown and with respect to ethnicity is {self.node.caracteristics[1]}. "
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                elif self.node.get_caract==21:
                    self.node.speech_str.command = f"The first guest is gender {self.node.caracteristics[0]}, the age is between {self.node.caracteristics[1]}. the guest is taller than me and the eyes are brown."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                elif self.node.get_caract==22:
                    self.node.speech_str.command =f"The first guest is gender {self.node.caracteristics[0]}, the age is between 25 and 35 and with respect to ethnicity is {self.node.caracteristics[1]}. the guest is taller than me and the eyes are brown."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                elif self.node.get_caract==1:
                    self.node.speech_str.command =f"The first guest is gender male, is taller than me, the eyes are brown and the age is between 25 and 35 and the gender is {self.node.caracteristics[0]}."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                elif self.node.get_caract==11:
                    self.node.speech_str.command =f"The first guest is gender male, is taller than me, the eyes are brown and the age is between 25 and 35 and with respect to ethnicity is {self.node.caracteristics[0]}."
                    self.node.speaker_publisher.publish(self.node.speech_str)

                    self.wait_for_end_of_speaking()
                elif self.node.get_caract==12:
                    self.node.speech_str.command =f"The first guest is gender male, is taller than me ,the eyes are brown and the age is between {self.node.caracteristics[0]}."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                else:
                    self.node.speech_str.command =f"The first guest is gender male, is taller than me, the eyes are brown and the age is between 25 and 35."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()


                self.node.neck_position_publisher.publish(self.node.sofa_neck)

                self.node.speech_str.command = f"Please take a sit on the sofa that I'm looking at."     
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                            #CORRIGIR PARA OLHAR PARA O SOFÁ 
                            #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                            #self.node.coordinates.rotate_target_coordinates = self.node.sofa_coordinates_orientation
                            #self.node.coordinates.flag_not_obs = False

                            #self.node.target_position_publisher.publish(self.node.coordinates)

                            
                            #self.node.neck_position_publisher.publish(self.node.talk_neck)
                            
                            #self.node.speech_str.command = f"The new guest's name is {self.node.names[2]} and his favorite drink is {self.node.drinks[2]}."
                            #self.node.speaker_publisher.publish(self.node.speech_str)
                            #self.wait_for_end_of_speaking()

                            #self.flag_Guest1 = True

                            #self.node.state = 8
                        #else:
                self.node.state = 8       
                            

                """ elif num_faces >=2:
                        #Código para reconhecer as pessoas
                        # Contagem das pessoas conhecidas na imagem
                        
                        
                        if self.node.names in nomes:
                            if nomes == self.node.names[0]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                                #self.node.coordinates.rotate_target_coordinates = self.node.guest_coordinates
                                #self.node.coordinates.flag_not_obs = False
                                #self.node.target_position_publisher.publish(self.node.coordinates)

                                self.node.neck_position_publisher.publish(self.node.talk_neck)
                                self.node.speech_str.command = f"The host is {self.node.names[0]} and his favorite drink is {self.node.drinks[0]}."
                                self.node.speaker_publisher.publish(self.node.speech_str)

                                #CORRIGIR PARA OLHAR PARA O SOFÁ 
                                #self.node.coordinates.move_target_coordinates = self.node.sofa_coordinates
                                #self.node.coordinates.rotate_target_coordinates = self.node.sofa_coordinates_orientation
                                #self.node.coordinates.flag_not_obs = False
                                #self.node.target_position_publisher.publish(self.nodecoordinates)


                                self.node.neck_position_publisher.publish(self.node.talk_neck)
                                
                                self.node.speech_str.command = f"And the new guest's name is {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()

                                self.node.flag_Host==True
                                self.state = 8
                            elif nomes == self.node.names[1]:
                                #CORRIGIR PARA OLHAR PARA O GUEST 2
                                #self.coordinates.move_target_coordinates = self.sofa_coordinates
                                #self.coordinates.rotate_target_coordinates = self.guest_coordinates
                                #self.coordinates.flag_not_obs = False

                                #self.target_position_publisher.publish(self.coordinates)

                                self.node.neck_position_publisher.publish(self.node.talk_neck)

                                self.node.speech_str.command = f"The guest is {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()
                                if self.node.get_caract==3:
                                    self.node.speech_str.command =f"The first guest is gender {self.node.caracteristics[0]}, is in age group {self.node.caracteristics[1]}. the guest is taller than me and with respect to ethnicity is {self.node.caracteristics[2]}."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()
                                elif self.node.get_caract==2:
                                    self.node.speech_str.command =f"The first guest is in age group {self.node.caracteristics[0]}. the guest is taller than me. And with respect to ethnicity is {self.node.caracteristics[1]}. "
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()
                                elif self.node.get_caract==21:
                                    self.node.speech_str.command = f"The first guest is gender {self.node.caracteristics[0]}, is in age group {self.node.caracteristics[1]}. the guest is taller than me."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()


                                elif self.node.get_caract==22:
                                    self.node.speech_str.command =f"The first guest is gender {self.node.caracteristics[0]}, and with respect to ethnicity is {self.node.caracteristics[1]}. the guest is taller than me."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()

                                elif self.node.get_caract==1:
                                    self.node.speech_str.command =f"The first guest is taller than me and the gender is {self.node.caracteristics[0]}."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()

                                elif self.node.get_caract==11:
                                    self.node.speech_str.command =f"The first guest is taller than me and with respect to ethnicity is {self.node.caracteristics[0]}."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()
                                elif self.node.get_caract==12:
                                    self.node.speech_str.command =f"The first guest is taller than me and is in age group {self.node.caracteristics[0]}."
                                    self.node.speaker_publisher.publish(self.node.speech_str)
                                    self.wait_for_end_of_speaking()



                                #CORRIGIR PARA OLHAR PARA O SOFA
                                #self.coordinates.move_target_coordinates = self.sofa_coordinates
                                #self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                                #self.coordinates.flag_not_obs = False

                                #self.target_position_publisher.publish(self.coordinates)

                                self.node.neck_position_publisher.publish(self.node.talk_neck)
                                self.node.speech_str.command = f"And the new guest's name is {self.node.names[2]} and his favorite drink is {self.node.drinks[2]}."
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()
                                self.node.flag_Guest1 = True
                                self.node.state = 8

                        elif nomes == (self.node.names[0] and self.node.names[1]) :
                            #CORRIGIR PARA OLHAR PARA O GUEST2
                            #self.neck_position_publisher.publish(self.navigation_neck)
                            #self.coordinates.move_target_coordinates = self.sofa_coordinates
                            #self.coordinates.rotate_target_coordinates = self.guest_coordinates
                            #self.coordinates.flag_not_obs = False

                            #self.target_position_publisher.publish(self.coordinates)
                            self.node.speech_str.command = f"On the couch are host {self.node.names[0]} and his favorite drink is {self.node.drinks[0]} and the guest {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking()

                            #CORRIGIR PARA OLHAR PARA O SOFÁ
                            #self.coordinates.move_target_coordinates = self.sofa_coordinates
                            #self.coordinates.rotate_target_coordinates = self.sofa_coordinates_orientation
                            #self.coordinates.flag_not_obs = False

                            #self.node.target_position_publisher.publish(self.node.coordinates)
                            
                            self.node.neck_position_publisher.publish(self.node.place_to_sit_neck)
                            self.node.speech_str.command = f"And the new guest's name is {self.node.names[1]} and his favorite drink is {self.node.drinks[1]}."
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking()
                            self.node.flag_Host==True
                            self.node.flag_Guest1 = True
                            self.node.state = 8
                        else:
                            self.node.state = 8
                else:
                    self.node.place_to_sit=self.node.sofa_coordinates
                    #self.place_to_sit_orientation=self.sofa_coordinates_orientation
                    self.node.flag_place_to_sit = True
                    self.node.state = 8 """

            elif self.node.state == 8:
                self.node.get_logger().info("estado 8")
                self.node.neck_position_publisher.publish(self.node.talk_neck)
                self.node.speech_str.command = f"Thank you. I finished my receptionist task"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                self.node.neck_position_publisher.publish(self.node.navigation_neck)
                self.node.state = 9

            elif self.node.state == 9:
                self.node.rgb_ctr = 100
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                print("Acabou")


        
        