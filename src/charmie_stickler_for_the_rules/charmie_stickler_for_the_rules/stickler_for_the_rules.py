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
from charmie_interfaces.msg import RobotSpeech, SpeechType, TarNavSDNL, Yolov8Pose, NeckPosition

import numpy as np
import time
import os
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
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        #self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)

        # Low Level Topics
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
    
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
        self.localisation_subscriber = self.create_subscription(Odometry, "odom_a", self.get_localisation_robot_callback, 10)

        # Navigation 
        self.target_position_publisher = self.create_publisher(TarNavSDNL, "target_pos", 10)
        self.flag_pos_reached_subscriber = self.create_subscription(Bool, "flag_pos_reached", self.flag_pos_reached_callback, 10)

        # Intel Realsense
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        
        # YOLO
        #self.depth_image_subscriber = self.create_subscription(Image, "/depth/image_rect_raw", self.get_depth_image_callback, 10)

        #Comms with Main
        self.get_person_publisher = self.create_publisher(Bool, "get_person", 10)
        self.person_info_subscriber = self.create_subscription(Pose2D, "person_info", self.person_info_callback, 10)
        


        #RGB
        self.rgb_ctr = 2
        self.rgb = Int16()

        # Changing Variables
        self.state = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0


        #Arena 1
        self.begin_coordinates = (0.0, 3.0) # <----- CHANGE M
        
        self.forbiden_room_coordinates = (2.0, 6.9)
        self.kitchen_room_coordinates = (0.5, 6.9) # coordenada da cozinha na porta entre cozinha e quarto
        self.hall_room_coordinates = (1.3, 3.5)
        self.living_room_coordinates = (1.3, 3.5)


        self.entrance_coordinates = (0.0, 0.0)  

        self.entrance_coordinates_orientation= (0.0, 0.0)

        

        self.bin_coordinates_orientation= (0.0, 0.0) # ----------> CHANGE ME
    
        self.find_coordinates = (1.3 , 8.5)

        self.talk_neck = NeckPosition()
        self.talk_neck.pan = 180.0
        self.talk_neck.tilt = 193.0

        
      
       

       #Esta variavel vai servir para contar as vezes em que o robô vai verificar os sapatos e bebida, vai 
       #fazê-lo em 3 diferentes espaços da casa 
        self.count_rooms = 0
       #COORDENADAS DE AUXILIO - VOU USAR OS TARGETS NAS PORTAS



        #da sala para o escritório
        self.A_target_sala = (0.5 , 3.0)
        self.A_target_escritorio = (2.0 , 3.0)

        #da sala para a cozinha

        self.B_target_sala = (-1.5, 4.1)
        self.B_target_cozinha = (-1.5, 6.0)

        #da cozinha para o quarto

        self.C_target_cozinha = (0.5, 6.9)
        self.C_target_quarto = (2.0, 6.9)

        self.wardrobe_bedroom = (4.8 , 5.8)
        self.small_table = (2.5 , 9.2)
        #coordenadas de pontos estratégicos para onde poderá olhar no quarto
        self.coordinates_bed = (3.75 , 8.0)

        self.coordinates_corner_opposite_bed = (4.5 , 5.2)

        self.coordinates_corner_right = (2.0 , 5.5)

        self.coordinates_corner_left = (2.0 , 9.2)

        #coordenadas de pontos estratégicos para onde poderá olhar na cozinha
        self.coordinates_table = (-2.5 , 7.5)

        self.coordinates_pia = (-0.6 , 4.8)

        self.bin_coordinates = (0.8, 8.0)

        #coordenadas de pontos estratégicos para onde poderá olhar na sala
        self.sofa_coordinates = (-3.4, 3.6) 

        self.coordinates_corner_living_room = (0.8 , 3.8)

        self.door_coordinates_orientation= (1.3, 2.1)
        #coordenadas de pontos estratégicos para onde poderá olhar no escritório

        self.middle_point_rigth = (3.4, 1.9)

        self.chair_left = (3.4 , 3.5)



        self.coordinates = TarNavSDNL()
        #Código para ele olhar para a cara da pessoa e centrar a cara da pessoa na imagem
        self.door_neck_coordinates = NeckPosition()
        self.door_neck_coordinates.pan = 180.0
        self.door_neck_coordinates.tilt = 200.0
        
        #Definir como se estivesse a olhar ligeiramente para baixo - Perceber o que precisamos que olhe para baixo para detetarmos tudo direito
        self.place_to_sit_neck = NeckPosition()
        self.place_to_sit_neck.pan = 180.0
        self.place_to_sit_neck.tilt = 160.0       

        self.guest_neck = NeckPosition()
        self.guest_neck.pan = 270.0
        self.guest_neck.tilt = 193.0


        self.navigation_neck = NeckPosition()
        self.navigation_neck.pan = 180.0
        self.navigation_neck.tilt = 170.0

        self.talk_neck = NeckPosition()
        self.talk_neck.pan = 180.0
        self.talk_neck.tilt = 193.0

        self.br = CvBridge()

        self.flag_left= False
        self.flag_right= False


        self.flag_first_time=True
        self.flag_speech_done = False
        self.flag_navigation_done = False
        self.flag_audio_done = False
        # self.keyword_list = []
        self.person_forbiden_room = 0
        self.person_with_shoes = 0
        self.person_with_drink = 0


        self.flag_shoes = False
        self.flag_drink = False
        self.flag_forbiden = False
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


        self.rules = 0

        self.cv2_img = []
        
        self.width = 0
        self.height = 0
        
        
        #Pastas relativas às funções das características como por exemplo, características, reconhecimento, etc.
        #CAMINHOS QUE É PRECISO MUDAR NOME


        self.get_caract = 0
        self.caracteristics = []
        self.filename = String()


        self.MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
        # MODEL_MEAN_VALUES = (78, 87, 114)

        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_detection = self.mp_face_detection.FaceDetection()
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

        self.person_detected = Pose2D()
        self.person_detected.theta = -1.0

    def person_info_callback(self, person: Pose2D):
        self.person_detected = person
        # print('RECEBI DADOS')

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


    """ def yolo_pose_callback(self, yolo:Yolov8Pose):
        self.yolo_pose=yolo
        self.yolo_pose.persons """
    

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
        self.image_color = img
        
    def get_speech_callback(self, keywords : String):
        print("Received Audio:", keywords.data)
        self.keywords = keywords
        self.flag_audio_done = True

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
        #self.start_audio()
    
    def flag_pos_reached_callback(self, state: Bool):
        #print("Received Navigation Flag:", state.data)
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
        self.start = time.time()
        self.end = time.time()
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False

    
    def wait_for_end_of_navigation(self):
        time.sleep(0.05)
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False


    def wait_for_end_of_audio(self):
        while not self.node.flag_audio_done:
            pass
        self.node.flag_audio_done = False
        self.node.flag_speech_done = False  
     
    def main(self):
        time.sleep(1)
        print("IN NEW MAIN")
        while True:
            #Navegação até ao espaço proibido
            if self.node.state == 0:
                print('state 0')
                self.node.rgb_ctr = 0
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)
                #Informa que está pronto para começar a tarefa
                self.node.get_logger().info("estado 0")


                #self.node.neck_position_publisher.publish(self.node.talk_neck)

                self.node.speech_str.command = "Hello! I am ready to start the Stickler for the rules task!"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking() 


                #COMEÇAMOS DENTRO DA AREA NOS PONTOS PRÉ DEFINIDOS ()
                #ASSUMINDO POR EXEMPLO QUE A SALA PROIBIDA É O QUARTO!!!!!!!!!!!!! <------- COULD CHANGEEEEE

                # Começamos no starting point! 

                # Se o quarto for a sala proibida fazemos - quarto, cozinha, sala, escritório ----------- OPÇÃO 1
                # Se o escritório for a sala proibida fazemos - escritório, entrada, cozinha e quarto --- OPÇÃO 2


                #VAMOS PARA A DIVISÃO DA CASA PROIBIDA

                # vai ser preciso colocar pontos intermédios para ir até ao sitio

                #NO SITIO ONDE ESTÁ RODA PARA A PORTA ONDE TEM DE IR
                #self.node.coordinates_to_navigation(self.node.begin_coordinates, self.node.B_target_sala, False)
                #self.wait_for_end_of_navigation()

                #VAI PARA A PORTA
                #self.node.coordinates_to_navigation(self.node.B_target_sala, self.node.B_target_cozinha, False)
                #self.wait_for_end_of_navigation()

                #AVANÇA A PORTA NA DIREÇÃO DA SALA PROIBIDA
                self.node.coordinates_to_navigation(self.node.B_target_cozinha, self.node.C_target_cozinha, False)
                self.wait_for_end_of_navigation()

                #AVANÇA PARA A ENTRADA DA SALA PROIBIDA
                #self.node.coordinates_to_navigation(self.node.C_target_cozinha, self.node.C_target_quarto, False)
                #self.wait_for_end_of_navigation()

                #VAI PARA O PONTO
                self.node.coordinates_to_navigation(self.node.forbiden_room_coordinates, self.node.coordinates_corner_right, False)
                self.wait_for_end_of_navigation()


                #PRECISO QUE AQUI O ROBÔ RODE SOBRE SI PRÓPRIO E OLHE PARA OS VÁRIOS PONTOS DA SALA PROIBIDA
                #EM TODOS OS PONTOS PRECISO DE FAZER VERIFICAÇÃO DO YOLO SE DETETO PESSOAS E SE ESTÃO DENTRO DO MAPA
                #SERÁ UMA ESPÉCIE DE IF QUE ME COLOCA A 1 A VARIAVEL SE TIVER DETETADO ALGUÉM NA SALA PROIBIDA.


                #Quero que veja as duas posições porque pode estar mais do que uma pessoa na sala proibida

                self.node.coordinates_to_navigation(self.node.coordinates_corner_right, self.node.coordinates_bed, False)
                self.wait_for_end_of_navigation() 

                #TIRA FOTO 
                #print('AAAAAAAA')
                #self.node.coordinates_to_navigation(self.node.coordinates_corner_right, self.node.coordinates_bed, False)
                #self.wait_for_end_of_navigation()
                #print('BBBBBBBB')

                # time.sleep(1)

                b = Bool()
                b.data = True
                self.node.get_person_publisher.publish(b)
                


                self.node.state=1

                self.start = time.time()



                #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS

            elif self.node.state == 1:
                

                    # if not self.node.flag_left:
                        # print("repeti -1")
                        #self.node.coordinates_to_navigation(self.node.coordinates_corner_right, self.node.small_table, False)
                        #self.wait_for_end_of_navigation()
                        # self.node.flag_left=True
                        
                    #RECOLHER NOVAMENTE OS PARAMETROS PARA OUTRA POSIÇÃO               
                    
                if self.node.person_detected.theta == 0 or self.node.person_detected.theta == -1:
                    # print("0")
                    
                    # self.node.coordinates_to_navigation(self.node.coordinates_corner_right, (self.node.person_detected.x,self.node.person_detected.y), False)
                    
                    self.node.rgb_ctr = 102
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    
                    if self.node.flag_first_time:
                        
                        #Informar a regra desrespeitada
                        self.node.speech_str.command = f"There is a guest breaking the forbidden room rule."               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.node.speech_str.command = f"Hello there guest. You are breaking the forbidden room rule "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()


                        self.node.speech_str.command = f"To stop breaking this rule, you must get out of the bedroom and join the other guests in the other rooms of the house."               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        #Informar a ação a tomar
                        self.node.speech_str.command = f"Please walk very slowly to another room so i can make eye contact with you and confirm when you stop breaking the rule"               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.flag_forbiden = True
                        self.node.flag_first_time=False


                    # self.node.coordinates_to_navigation((self.node.robot_x, self.node.robot_y), (self.node.person_detected.x,self.node.person_detected.y), False)
                    # self.wait_for_end_of_navigation() 


                
                    self.end = time.time()
                    if ((self.end - self.start)  >  20.0):
                        print(self.end - self.start)
                        #Informar a ação a tomar
                        self.node.speech_str.command = f"Please walk very slowly to the outside of the bedroom."               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.start = time.time()


                    #self.wait_for_end_of_speaking()
                    #self.node.rules+=1

                    #CHANGE THIS  ----- > self.node.coordinates_to_navigation(self.node.coordinates_corner_right, self.node.wardrobe_bedroom, False)
                    #CHANGE THIS  ----- > self.wait_for_end_of_navigation()



                    ### FALTA CODIGO DA CONFIRMACAO


                    #continuar a receber informação do yolo para verificar quando a pessoa sai
                elif self.node.person_detected.theta == 1:
                    print("1")

                    self.node.state=500
                #VERIFICAR SE SAI

                #CONTINUAR A DETEÇÃO COM O YOLO POSE, NO CASO DE CONTINUAR A DETETAR CONTINUA A DIZER QUE TEM DE SAIR, 
                #SE SAIR, PODEMOS DIZER QUE CONFIRMOU QUE SAIU DO QUARTO E VAI AVANÇAR
                else:
                    print("Ainda não detetei ninguém nesta sala")
                    #FICAR NA POSIÇÃO DO CANTO ATÉ DETETAR
                
            elif self.node.state == 500:  # detetou que pessoa bazou da casa

                self.node.rgb_ctr = 12
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)


                self.node.speech_str.command = f"I confirm that the party guest stopped breaking the rule by leaving the forbidden room."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                
                
                time.sleep(10)
                
                
                self.node.state=2

                

                
                print("SAIU DA CASA")



            elif self.node.state==2:
                self.node.rgb_ctr = 2
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                #print("estado 2, saí do quarto, porque nao esta ninguem")
            


            #Estado na cozinha
            """ elif self.node.state == 1:
                self.node.get_logger().info("estado 1")
                self.node.count_rooms +=1


                #VAMOS PARA A LOCALIZAÇÃO AO LADO DA LOCALIZAÇÃO PROIBIDA, NESTE CASO TOU A CONSIDERAR 
                # A COZINHA <------- COULD CHANGEEEEE

                #Voltamos para o target da porta na sala proibida para não causar problemas na navegação
                self.node.coordinates_to_navigation(self.node.forbiden_room_coordinates, self.node.C_target_cozinha, False)
                self.wait_for_end_of_navigation()

                #Vamos para a cozinha e ficamos orientados para a frente
                self.node.coordinates_to_navigation(self.node.kitchen_room_coordinates, self.node.kitchen_room_coordinates, False)
                self.wait_for_end_of_navigation()

                self.node.coordinates_to_navigation(self.node.bin_coordinates, self.node.kitchen_room_coordinates, False)
                self.wait_for_end_of_navigation()

        
                #COMEÇAMOS A PROCURAR COM O YOLO PESSOA COM SAPATOS E BEBIDAS NA MÃO, SE PERCEBERMOS QUE É DEMASIADO ANDAMOS
                #COM O PESCOÇO MAIS PARA BAIXO PARA SÓ DETETAR TRONCO E PROCURAR SAPATOS. NESTE CASO, PRECISAMOS DE TER O AJUSTE 
                #DO PESCOÇO COM OS KEYPOINTS

                #SE ENCONTRAR SAPATOS A FLAG PERSON WITH SHOES VAI A 1


                self.node.coordinates_to_navigation(self.node.bin_coordinates, self.node.coordinates_pia, False)
                self.wait_for_end_of_navigation()

                #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS


                self.node.coordinates_to_navigation(self.node.bin_coordinates, self.node.coordinates_table, False)
                self.wait_for_end_of_navigation()

                #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS


                self.node.state =2

            elif self.node.state==2:

                self.node.get_logger().info("estado 2")
                
                cv2_img = self.node.br.imgmsg_to_cv2(self.node.image_color, "bgr8")
                num_faces, face_x, face_y, shoulder, hip, center_x, center_y = self.node.found_landmarks(cv2_img)
            

                if self.node.person_with_shoes == 1 and self.node.person_with_drink == 1:
                    if (face_y < shoulder and shoulder < hip):
                        error_x = face_x-center_x
                        error_y = face_y-center_y
                        self.node.follow_center_face(error_x,error_y)


                        self.node.rgb_ctr = 12
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)


                        #COLOCAR CÓDIGO SE OLHAR PARA A PESSOA CERTA COM O PESCOÇO PARA DEMONSTRAR QUE A PESSOA FOI IDENTIFICADA 
                        #E PODEMOS APROXIMARMO-NOS DA PESSOA TAMBÉM

                        #Informar a regra desrespeitada
                        self.node.speech_str.command = f"Hi.You are with shoes in my house and you are without a drink and it's a party!!"               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        

                        #Informar a ação a tomar
                        self.node.speech_str.command = f"You have to go to the entrance and take off your shoes and after that go to the kitchen and get your drink!! "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                    
                        self.flag_shoes = True
                        self.flag_drink = True
                        self.node.rules +=2


                elif self.node.person_with_drink == 1 and self.node.person_with_shoes == 0:
                     if (face_y < shoulder and shoulder < hip):
                        error_x = face_x-center_x
                        error_y = face_y-center_y
                        self.node.follow_center_face(error_x,error_y)


                        self.node.rgb_ctr = 12
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)

                        #COLOCAR CÓDIGO SE OLHAR PARA A PESSOA CERTA COM O PESCOÇO PARA DEMONSTRAR QUE A PESSOA FOI IDENTIFICADA 
                        #E PODEMOS APROXIMARMO-NOS DA PESSOA TAMBÉM

                        #Informar a regra desrespeitada
                        self.node.speech_str.command = f"Hi.You are without a drink and it's a party. "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        

                        #Informar a ação a tomar
                        self.node.speech_str.command = f"Go to the kitchen and get your drink! "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        
                        self.flag_drink = True
                        self.node.rules+=1

                elif self.node.person_with_shoes == 1 and self.node.person_with_drink == 0:
                     if (face_y < shoulder and shoulder < hip):
                        error_x = face_x-center_x
                        error_y = face_y-center_y
                        self.node.follow_center_face(error_x,error_y)


                        self.node.rgb_ctr = 12
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb)

                        #Informar a regra desrespeitada
                        self.node.speech_str.command = f"Hi.You are with shoes in my house. "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        

                        #Informar a ação a tomar
                        self.node.speech_str.command = f"You have to go to the entrance and take off your shoes! "               
                        #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.flag_shoes = True
                        self.node.rules+=1

                else:
                    print("Ainda não detetei ninguém sem bebidas ou com sapatos nesta sala") 

                if self.node.count_rooms == 1:
                    self.node.state = 3
                elif self.node.count_rooms == 2:
                    self.node.state = 4
                else: 
                    self.node.state = 5
                    
                    print("VERIFIQUEI AS SALAS TODAS")



            #Estado na sala
            elif self.node.state ==3:
                self.node.get_logger().info("estado 3")

                self.node.count_rooms +=1
                #PROCURAR PESSOAS COM O YOLO POSE
                
                #VAMOS PARA A LOCALIZAÇÃO AO LADO DA LOCALIZAÇÃO ANTERIOR, NESTE CASO TOU A CONSIDERAR 
                # A SALA <------- COULD CHANGEEEEE

                #Voltamos para o target da porta na sala proibida para não causar problemas na navegação
                self.node.coordinates_to_navigation(self.node.B_target_cozinha, self.node.B_target_sala, False)
                self.wait_for_end_of_navigation()

                #Vamos para a sala e ficamos orientados para a frente
                self.node.coordinates_to_navigation(self.node.B_target_sala, self.node.B_target_sala, False)
                self.wait_for_end_of_navigation()

                

        
                #COMEÇAMOS A PROCURAR COM O YOLO PESSOA COM SAPATOS E BEBIDAS NA MÃO, SE PERCEBERMOS QUE É DEMASIADO ANDAMOS
                #COM O PESCOÇO MAIS PARA BAIXO PARA SÓ DETETAR TRONCO E PROCURAR SAPATOS. NESTE CASO, PRECISAMOS DE TER O AJUSTE 
                #DO PESCOÇO COM OS KEYPOINTS

                #SE ENCONTRAR SAPATOS A FLAG PERSON WITH SHOES VAI A 1


                self.node.coordinates_to_navigation(self.node.B_target_sala, self.node.coordinates_corner_living_room, False)
                self.wait_for_end_of_navigation()

                 #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS

                self.node.coordinates_to_navigation(self.node.coordinates_corner_living_room, self.node.door_coordinates_orientation, False)
                self.wait_for_end_of_navigation()

                 #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS

                self.node.coordinates_to_navigation(self.node.coordinates_corner_living_room, self.node.sofa_coordinates, False)
                self.wait_for_end_of_navigation()

                 #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS

                self.node.state =2

            #Estado no escritório
            elif self.node.state ==4:
                self.node.get_logger().info("estado 4")

                self.node.count_rooms +=1
                #PROCURAR PESSOAS COM O YOLO POSE
                
                #VAMOS PARA A LOCALIZAÇÃO AO LADO DA LOCALIZAÇÃO ANTERIOR, NESTE CASO TOU A CONSIDERAR 
                # O ESCRITÓRIO <------- COULD CHANGEEEEE

                #Voltamos para o target da porta do escritório na sala para não causar problemas na navegação
                self.node.coordinates_to_navigation(self.node.A_target_sala, self.node.A_target_escritorio, False)
                self.wait_for_end_of_navigation()

                #Vamos para o escritório e ficamos orientados para a frente
                self.node.coordinates_to_navigation(self.node.A_target_escritorio, self.node.A_target_escritorio, False)
                self.wait_for_end_of_navigation()

                

        
                #COMEÇAMOS A PROCURAR COM O YOLO PESSOA COM SAPATOS E BEBIDAS NA MÃO, SE PERCEBERMOS QUE É DEMASIADO ANDAMOS
                #COM O PESCOÇO MAIS PARA BAIXO PARA SÓ DETETAR TRONCO E PROCURAR SAPATOS. NESTE CASO, PRECISAMOS DE TER O AJUSTE 
                #DO PESCOÇO COM OS KEYPOINTS

                #SE ENCONTRAR SAPATOS A FLAG PERSON WITH SHOES VAI A 1


                self.node.coordinates_to_navigation(self.node.A_target_escritorio, self.node.chair_left, False)
                self.wait_for_end_of_navigation()

                 #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS


                self.node.coordinates_to_navigation(self.node.A_target_escritorio, self.node.middle_point_rigth, False)
                self.wait_for_end_of_navigation()

                 #PROCURAR PESSOAS COM O YOLO POSE
                #JUNTAR A PARTE DE DETETAR SAPATOS E BEBIDAS


                self.node.state =5
            
            
            elif self.node.state == 5:
                
                self.node.get_logger().info("estado 5")
                break 
                if self.node.rules >= 4:

                    self.node.neck_position_publisher.publish(self.node.talk_neck)
                    self.node.speech_str.command = f"I already find the guests breaking 4 rules.. "               
                    #self.node.speech_str.command = f"Please take a sit on the sit that i'm looking for."                            
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                else:
                    if self.node.flag_forbiden == True:
                        if self.node.flag_drink == False or self.node.flag_shoes == False:
                            self.node.state=1
                        else:
                            pass
                    else:
                        self.node.state=0 """



